#include "sdkconfig.h"

#include <stdbool.h>
#include <stdio.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mpu6050";

static i2c_master_dev_handle_t s_dev;

enum {
    MPU6050_REG_SMPLRT_DIV = 0x19,
    MPU6050_REG_CONFIG = 0x1A,
    MPU6050_REG_GYRO_CONFIG = 0x1B,
    MPU6050_REG_ACCEL_CONFIG = 0x1C,
    MPU6050_REG_ACCEL_XOUT_H = 0x3B,
    MPU6050_REG_PWR_MGMT_1 = 0x6B,
    MPU6050_REG_WHO_AM_I = 0x75,
};

#define MPU6050_ACCEL_LSB_PER_G 16384
#define MPU6050_GYRO_LSB_PER_DPS 131

static esp_err_t mpu_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

static esp_err_t mpu_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len, 100);
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static esp_err_t i2c_init(void)
{
    const i2c_master_bus_config_t bus = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_SAMPLE_MPU6050_SDA,
        .scl_io_num = CONFIG_SAMPLE_MPU6050_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle = NULL;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus, &bus_handle), TAG, "i2c bus");

    const i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_SAMPLE_MPU6050_ADDR,
        .scl_speed_hz = 100000,
    };
    return i2c_master_bus_add_device(bus_handle, &dev, &s_dev);
}

static esp_err_t mpu_wake(void)
{
    ESP_RETURN_ON_ERROR(mpu_write(MPU6050_REG_PWR_MGMT_1, 0x80), TAG, "reset");
    vTaskDelay(pdMS_TO_TICKS(100));
    /* PLL from the X gyro, ±2 g, ±250 dps, ~200 Hz sample rate. */
    ESP_RETURN_ON_ERROR(mpu_write(MPU6050_REG_PWR_MGMT_1, 0x01), TAG, "clock");
    ESP_RETURN_ON_ERROR(mpu_write(MPU6050_REG_CONFIG, 0x03), TAG, "dlpf");
    ESP_RETURN_ON_ERROR(mpu_write(MPU6050_REG_SMPLRT_DIV, 0x04), TAG, "rate");
    ESP_RETURN_ON_ERROR(mpu_write(MPU6050_REG_GYRO_CONFIG, 0x00), TAG, "gyro");
    return mpu_write(MPU6050_REG_ACCEL_CONFIG, 0x00);
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

static void log_sample(const uint8_t raw[14])
{
    const int32_t ax_mg = (int32_t)be16(&raw[0]) * 1000 / MPU6050_ACCEL_LSB_PER_G;
    const int32_t ay_mg = (int32_t)be16(&raw[2]) * 1000 / MPU6050_ACCEL_LSB_PER_G;
    const int32_t az_mg = (int32_t)be16(&raw[4]) * 1000 / MPU6050_ACCEL_LSB_PER_G;
    const int32_t gx_mdps = (int32_t)be16(&raw[8]) * 1000 / MPU6050_GYRO_LSB_PER_DPS;
    const int32_t gy_mdps = (int32_t)be16(&raw[10]) * 1000 / MPU6050_GYRO_LSB_PER_DPS;
    const int32_t gz_mdps = (int32_t)be16(&raw[12]) * 1000 / MPU6050_GYRO_LSB_PER_DPS;
    const int32_t temp_c100 = (int32_t)be16(&raw[6]) * 100 / 340 + 3653;
    int32_t temp_frac = temp_c100 % 100;
    if (temp_frac < 0) {
        temp_frac = -temp_frac;
    }

    char ax[16];
    char ay[16];
    char az[16];
    char gx[16];
    char gy[16];
    char gz[16];
    fmt_dec(ax, sizeof(ax), ax_mg, 1000, 2);
    fmt_dec(ay, sizeof(ay), ay_mg, 1000, 2);
    fmt_dec(az, sizeof(az), az_mg, 1000, 2);
    fmt_dec(gx, sizeof(gx), gx_mdps, 1000, 1);
    fmt_dec(gy, sizeof(gy), gy_mdps, 1000, 1);
    fmt_dec(gz, sizeof(gz), gz_mdps, 1000, 1);

    ESP_LOGI(TAG, "加速度 x=%s y=%s z=%s g | 角速度 x=%s y=%s z=%s 度/秒 | 温度 %ld.%ld C",
             ax, ay, az, gx, gy, gz, (long)(temp_c100 / 100), (long)(temp_frac / 10));
}

void sample_run(void)
{
    ESP_LOGI(TAG, "SDA=%d SCL=%d addr=0x%02X",
             CONFIG_SAMPLE_MPU6050_SDA, CONFIG_SAMPLE_MPU6050_SCL, CONFIG_SAMPLE_MPU6050_ADDR);
    ESP_ERROR_CHECK(i2c_init());

    uint8_t who = 0;
    esp_err_t err = mpu_read(MPU6050_REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I failed: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "check 3V3, GND, SDA, SCL, and AD0");
    } else {
        ESP_LOGI(TAG, "WHO_AM_I 0x%02X", who);
        if (who != 0x68) {
            ESP_LOGW(TAG, "expected 0x68 for an MPU6050");
        }
    }

    for (;;) {
        err = mpu_wake();
        if (err == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "wake failed: %s", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "平放时一个轴大约 ±1 g，另两个接近 0；静止时角速度接近 0");
    for (;;) {
        uint8_t raw[14];
        err = mpu_read(MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "read failed: %s", esp_err_to_name(err));
        } else {
            log_sample(raw);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
