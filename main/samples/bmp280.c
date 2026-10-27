#include "sdkconfig.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bmp280";

static i2c_master_dev_handle_t s_dev;
static int32_t s_t_fine;

enum {
    BMP280_REG_CALIB = 0x88,
    BMP280_REG_ID = 0xD0,
    BMP280_REG_RESET = 0xE0,
    BMP280_REG_STATUS = 0xF3,
    BMP280_REG_CTRL_MEAS = 0xF4,
    BMP280_REG_PRESS = 0xF7,
};

#define BMP280_ID 0x58
#define BMP280_ID_BME280 0x60
#define BMP280_RESET 0xB6
#define BMP280_STATUS_MEASURING 0x08
#define BMP280_STATUS_IM_UPDATE 0x01
/* x1 oversampling, forced mode. A new sample starts on every write. */
#define BMP280_CTRL_FORCED 0x25
/* Standard sea-level pressure. Replace with the local weather station's
 * current QNH, otherwise the altitude below is not the real elevation. */
#define BMP280_SEA_LEVEL_PA 101325.0

typedef struct {
    uint16_t t1;
    int16_t t2;
    int16_t t3;
    uint16_t p1;
    int16_t p2;
    int16_t p3;
    int16_t p4;
    int16_t p5;
    int16_t p6;
    int16_t p7;
    int16_t p8;
    int16_t p9;
} bmp280_cal_t;

static bmp280_cal_t s_cal;

static esp_err_t bmp_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

static esp_err_t bmp_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len, 100);
}

static uint16_t le_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t le_i16(const uint8_t *p)
{
    return (int16_t)le_u16(p);
}

static esp_err_t i2c_init(void)
{
    const i2c_master_bus_config_t bus = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_SAMPLE_BMP280_SDA,
        .scl_io_num = CONFIG_SAMPLE_BMP280_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle = NULL;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus, &bus_handle), TAG, "i2c bus");

    const i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_SAMPLE_BMP280_ADDR,
        .scl_speed_hz = 100000,
    };
    return i2c_master_bus_add_device(bus_handle, &dev, &s_dev);
}

static esp_err_t bmp_load_cal(void)
{
    uint8_t raw[24];
    ESP_RETURN_ON_ERROR(bmp_read(BMP280_REG_CALIB, raw, sizeof(raw)), TAG, "cal");
    s_cal.t1 = le_u16(&raw[0]);
    s_cal.t2 = le_i16(&raw[2]);
    s_cal.t3 = le_i16(&raw[4]);
    s_cal.p1 = le_u16(&raw[6]);
    s_cal.p2 = le_i16(&raw[8]);
    s_cal.p3 = le_i16(&raw[10]);
    s_cal.p4 = le_i16(&raw[12]);
    s_cal.p5 = le_i16(&raw[14]);
    s_cal.p6 = le_i16(&raw[16]);
    s_cal.p7 = le_i16(&raw[18]);
    s_cal.p8 = le_i16(&raw[20]);
    s_cal.p9 = le_i16(&raw[22]);
    if (s_cal.t1 == 0 || s_cal.t1 == 0xFFFF || s_cal.p1 == 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t bmp_prepare(void)
{
    ESP_RETURN_ON_ERROR(bmp_write(BMP280_REG_RESET, BMP280_RESET), TAG, "reset");
    for (int i = 0; i < 20; i++) {
        uint8_t status = 0xFF;
        vTaskDelay(pdMS_TO_TICKS(2));
        ESP_RETURN_ON_ERROR(bmp_read(BMP280_REG_STATUS, &status, 1), TAG, "status");
        if ((status & BMP280_STATUS_IM_UPDATE) == 0) {
            return bmp_load_cal();
        }
    }
    return ESP_ERR_TIMEOUT;
}

/* Bosch integer compensation. Temperature unit is 0.01 C. */
static int32_t compensate_temp(int32_t adc_t)
{
    int32_t var1 = ((((adc_t >> 3) - ((int32_t)s_cal.t1 << 1))) * (int32_t)s_cal.t2) >> 11;
    int32_t var2 = (((((adc_t >> 4) - (int32_t)s_cal.t1) * ((adc_t >> 4) - (int32_t)s_cal.t1)) >> 12) *
                    (int32_t)s_cal.t3) >> 14;
    s_t_fine = var1 + var2;
    return (s_t_fine * 5 + 128) >> 8;
}

/* Pressure in Pa, Q24.8. Divide by 256 for whole pascals. */
static uint32_t compensate_press(int32_t adc_p)
{
    int64_t var1 = (int64_t)s_t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)s_cal.p6;
    var2 = var2 + ((var1 * (int64_t)s_cal.p5) << 17);
    var2 = var2 + (((int64_t)s_cal.p4) << 35);
    var1 = ((var1 * var1 * (int64_t)s_cal.p3) >> 8) + ((var1 * (int64_t)s_cal.p2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)s_cal.p1) >> 33;
    if (var1 == 0) {
        return 0;
    }

    int64_t p = 1048576 - adc_p;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)s_cal.p9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)s_cal.p8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)s_cal.p7) << 4);
    return (uint32_t)p;
}

static int32_t raw_adc(const uint8_t *p)
{
    return ((int32_t)p[0] << 12) | ((int32_t)p[1] << 4) | ((int32_t)p[2] >> 4);
}

static void fmt_dec(char *out, size_t out_len, int32_t value, int32_t divisor, int places)
{
    const bool neg = value < 0;
    int32_t mag = neg ? -value : value;
    int32_t whole = mag / divisor;
    int32_t frac_scale = 1;
    for (int i = 0; i < places; i++) {
        frac_scale *= 10;
    }
    int32_t frac = (mag % divisor) * frac_scale / divisor;
    snprintf(out, out_len, "%s%ld.%.*ld", neg ? "-" : "", (long)whole, places, (long)frac);
}

static esp_err_t bmp_sample(void)
{
    ESP_RETURN_ON_ERROR(bmp_write(BMP280_REG_CTRL_MEAS, BMP280_CTRL_FORCED), TAG, "measure");
    uint8_t status = BMP280_STATUS_MEASURING;
    for (int i = 0; i < 20 && (status & BMP280_STATUS_MEASURING) != 0; i++) {
        vTaskDelay(pdMS_TO_TICKS(2));
        ESP_RETURN_ON_ERROR(bmp_read(BMP280_REG_STATUS, &status, 1), TAG, "status");
    }
    if ((status & BMP280_STATUS_MEASURING) != 0) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t raw[6];
    ESP_RETURN_ON_ERROR(bmp_read(BMP280_REG_PRESS, raw, sizeof(raw)), TAG, "data");
    const int32_t temp_c100 = compensate_temp(raw_adc(&raw[3]));
    const uint32_t press_q24 = compensate_press(raw_adc(&raw[0]));
    if (press_q24 == 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const int32_t hpa_x100 = (int32_t)(press_q24 / 256);
    const double altitude_m = 44330.0 * (1.0 - pow(((double)press_q24 / 256.0) / BMP280_SEA_LEVEL_PA, 0.1903));
    char temp[16];
    char press[16];
    char alt[16];
    fmt_dec(temp, sizeof(temp), temp_c100, 100, 2);
    fmt_dec(press, sizeof(press), hpa_x100, 100, 2);
    fmt_dec(alt, sizeof(alt), (int32_t)(altitude_m * 10.0), 10, 1);
    ESP_LOGI(TAG, "温度 %s C | 气压 %s hPa | 海拔 %s m", temp, press, alt);
    return ESP_OK;
}

void sample_run(void)
{
    ESP_LOGI(TAG, "SDA=%d SCL=%d addr=0x%02X",
             CONFIG_SAMPLE_BMP280_SDA, CONFIG_SAMPLE_BMP280_SCL, CONFIG_SAMPLE_BMP280_ADDR);
    ESP_ERROR_CHECK(i2c_init());

    uint8_t id = 0;
    esp_err_t err = bmp_read(BMP280_REG_ID, &id, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "chip id failed: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "check 3V3, GND, SDA, SCL, and SDO");
    } else {
        ESP_LOGI(TAG, "chip id 0x%02X", id);
        if (id == BMP280_ID_BME280) {
            ESP_LOGI(TAG, "BME280; this sample reads temperature and pressure");
        } else if (id != BMP280_ID) {
            ESP_LOGW(TAG, "expected 0x58 for a BMP280");
        }
    }

    for (;;) {
        err = bmp_prepare();
        if (err == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "init failed: %s", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "海拔按 1013.25 hPa 估算；基准气压要改成当地气象站实时值，否则海拔不准");
    for (;;) {
        err = bmp_sample();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "read failed: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
