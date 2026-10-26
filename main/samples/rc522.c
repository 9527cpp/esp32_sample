#include "sdkconfig.h"

#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "rc522";

static spi_device_handle_t s_spi;

enum {
    RC522_REG_COMMAND = 0x01,
    RC522_REG_COM_IRQ = 0x04,
    RC522_REG_DIV_IRQ = 0x05,
    RC522_REG_ERROR = 0x06,
    RC522_REG_STATUS2 = 0x08,
    RC522_REG_FIFO_DATA = 0x09,
    RC522_REG_FIFO_LEVEL = 0x0A,
    RC522_REG_CONTROL = 0x0C,
    RC522_REG_BIT_FRAMING = 0x0D,
    RC522_REG_MODE = 0x11,
    RC522_REG_TX_CONTROL = 0x14,
    RC522_REG_TX_ASK = 0x15,
    RC522_REG_T_MODE = 0x2A,
    RC522_REG_T_PRESCALER = 0x2B,
    RC522_REG_T_RELOAD_H = 0x2C,
    RC522_REG_T_RELOAD_L = 0x2D,
    RC522_REG_CRC_RESULT_H = 0x21,
    RC522_REG_CRC_RESULT_L = 0x22,
    RC522_REG_VERSION = 0x37,
};

enum {
    RC522_CMD_IDLE = 0x00,
    RC522_CMD_CALC_CRC = 0x03,
    RC522_CMD_TRANSCEIVE = 0x0C,
    RC522_CMD_MF_AUTHENT = 0x0E,
    RC522_CMD_SOFT_RESET = 0x0F,
};

#define RC522_IRQ_TIMER 0x01
#define RC522_IRQ_IDLE  0x10
#define RC522_IRQ_RX    0x20
#define RC522_IRQ_ALL   0x7F

#define RC522_ERR_PROTOCOL  0x01
#define RC522_ERR_PARITY    0x02
#define RC522_ERR_COLLISION 0x08
#define RC522_ERR_BUFFER    0x10
#define RC522_DIVIRQ_CRC    0x04
#define RC522_STATUS2_CRYPTO1 0x08

#define PICC_WUPA            0x52
#define PICC_SEL_CL1         0x93
#define PICC_SEL_NVB_COMPLETE 0x70
#define PICC_HLTA            0x50
#define PICC_MF_AUTH_KEY_A   0x60
#define PICC_MF_READ         0x30
#define PICC_CASCADE_TAG     0x88
#define PICC_SAK_CASCADE     0x04

#define RC522_FIFO_MAX 64

typedef enum {
    PICC_OK = 0,
    PICC_TIMEOUT,
    PICC_COLLISION,
    PICC_ERROR,
} picc_status_t;

typedef struct {
    picc_status_t status;
    uint8_t data[18];
    size_t len;
} picc_reply_t;

static const uint8_t s_key_a[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t s_done_uid[4];
static bool s_done;

static uint8_t rc522_addr(uint8_t reg, bool read)
{
    uint8_t addr = (uint8_t)((reg << 1) & 0x7E);
    if (read) {
        addr |= 0x80;
    }
    return addr;
}

static esp_err_t rc522_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t trans = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_transmit(s_spi, &trans);
}

static esp_err_t rc522_read(uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = {rc522_addr(reg, true), 0x00};
    uint8_t rx[2] = {0};
    esp_err_t err = rc522_transfer(tx, rx, sizeof(tx));
    if (err == ESP_OK) {
        *value = rx[1];
    }
    return err;
}

static esp_err_t rc522_write(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {rc522_addr(reg, false), value};
    uint8_t rx[2] = {0};
    return rc522_transfer(tx, rx, sizeof(tx));
}

static esp_err_t rc522_write_fifo(const uint8_t *data, size_t len)
{
    uint8_t tx[RC522_FIFO_MAX + 1];
    uint8_t rx[RC522_FIFO_MAX + 1];

    if (len > RC522_FIFO_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }
    tx[0] = rc522_addr(RC522_REG_FIFO_DATA, false);
    memcpy(&tx[1], data, len);
    return rc522_transfer(tx, rx, len + 1);
}

static esp_err_t rc522_read_fifo(uint8_t *data, size_t len)
{
    if (len == 0 || len > RC522_FIFO_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }
    /* Clocking out 0x00 after the address latches register 0, so a burst would
     * keep only the first FIFO byte. One transfer per byte stays on FIFODataReg. */
    for (size_t i = 0; i < len; i++) {
        esp_err_t err = rc522_read(RC522_REG_FIFO_DATA, &data[i]);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

static esp_err_t spi_init(void)
{
    const spi_bus_config_t bus = {
        .mosi_io_num = CONFIG_SAMPLE_RC522_MOSI,
        .miso_io_num = CONFIG_SAMPLE_RC522_MISO,
        .sclk_io_num = CONFIG_SAMPLE_RC522_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
    };
    const spi_device_interface_config_t dev = {
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .mode = 0,
        .clock_speed_hz = 1000000,
        .spics_io_num = CONFIG_SAMPLE_RC522_CS,
        .queue_size = 1,
    };

    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED), TAG, "spi bus");
    return spi_bus_add_device(SPI2_HOST, &dev, &s_spi);
}

static esp_err_t rc522_init(void)
{
    const gpio_num_t rst = (gpio_num_t)CONFIG_SAMPLE_RC522_RST;
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << rst,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "rst gpio");
    gpio_set_level(rst, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(rst, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_RETURN_ON_ERROR(rc522_write(RC522_REG_COMMAND, RC522_CMD_SOFT_RESET), TAG, "soft reset");
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Timer fires about 25 ms after a frame, so a missing card ends the wait. */
    ESP_RETURN_ON_ERROR(rc522_write(RC522_REG_T_MODE, 0x80), TAG, "timer");
    ESP_RETURN_ON_ERROR(rc522_write(RC522_REG_T_PRESCALER, 0xA9), TAG, "timer");
    ESP_RETURN_ON_ERROR(rc522_write(RC522_REG_T_RELOAD_H, 0x03), TAG, "timer");
    ESP_RETURN_ON_ERROR(rc522_write(RC522_REG_T_RELOAD_L, 0xE8), TAG, "timer");
    ESP_RETURN_ON_ERROR(rc522_write(RC522_REG_TX_ASK, 0x40), TAG, "ask");
    ESP_RETURN_ON_ERROR(rc522_write(RC522_REG_MODE, 0x3D), TAG, "crc preset");

    uint8_t tx_control = 0;
    ESP_RETURN_ON_ERROR(rc522_read(RC522_REG_TX_CONTROL, &tx_control), TAG, "antenna");
    return rc522_write(RC522_REG_TX_CONTROL, (uint8_t)(tx_control | 0x03));
}

static picc_reply_t picc_transceive(const uint8_t *tx, size_t tx_len, uint8_t tx_last_bits)
{
    picc_reply_t reply = {.status = PICC_ERROR};

    if (rc522_write(RC522_REG_COMMAND, RC522_CMD_IDLE) != ESP_OK ||
        rc522_write(RC522_REG_COM_IRQ, RC522_IRQ_ALL) != ESP_OK ||
        rc522_write(RC522_REG_FIFO_LEVEL, 0x80) != ESP_OK ||
        rc522_write_fifo(tx, tx_len) != ESP_OK ||
        rc522_write(RC522_REG_BIT_FRAMING, tx_last_bits & 0x07) != ESP_OK ||
        rc522_write(RC522_REG_COMMAND, RC522_CMD_TRANSCEIVE) != ESP_OK ||
        rc522_write(RC522_REG_BIT_FRAMING, (uint8_t)((tx_last_bits & 0x07) | 0x80)) != ESP_OK) {
        return reply;
    }

    uint8_t irq = 0;
    for (int i = 0; i < 2000; i++) {
        if (rc522_read(RC522_REG_COM_IRQ, &irq) != ESP_OK) {
            return reply;
        }
        if (irq & (RC522_IRQ_RX | RC522_IRQ_IDLE | RC522_IRQ_TIMER)) {
            break;
        }
    }

    if ((irq & RC522_IRQ_RX) == 0) {
        reply.status = PICC_TIMEOUT;
        return reply;
    }

    uint8_t error = 0;
    if (rc522_read(RC522_REG_ERROR, &error) != ESP_OK) {
        return reply;
    }
    if (error & RC522_ERR_COLLISION) {
        reply.status = PICC_COLLISION;
        return reply;
    }
    if (error & (RC522_ERR_PROTOCOL | RC522_ERR_PARITY | RC522_ERR_BUFFER)) {
        return reply;
    }

    uint8_t level = 0;
    if (rc522_read(RC522_REG_FIFO_LEVEL, &level) != ESP_OK) {
        return reply;
    }
    level &= 0x7F;
    if (level == 0 || level > sizeof(reply.data)) {
        return reply;
    }
    if (rc522_read_fifo(reply.data, level) != ESP_OK) {
        return reply;
    }

    reply.len = level;
    reply.status = PICC_OK;
    return reply;
}

static esp_err_t rc522_crc(const uint8_t *data, size_t len, uint8_t out[2])
{
    if (rc522_write(RC522_REG_COMMAND, RC522_CMD_IDLE) != ESP_OK ||
        rc522_write(RC522_REG_DIV_IRQ, RC522_DIVIRQ_CRC) != ESP_OK ||
        rc522_write(RC522_REG_FIFO_LEVEL, 0x80) != ESP_OK ||
        rc522_write_fifo(data, len) != ESP_OK ||
        rc522_write(RC522_REG_COMMAND, RC522_CMD_CALC_CRC) != ESP_OK) {
        return ESP_FAIL;
    }

    uint8_t irq = 0;
    for (int i = 0; i < 1000; i++) {
        if (rc522_read(RC522_REG_DIV_IRQ, &irq) != ESP_OK) {
            return ESP_FAIL;
        }
        if (irq & RC522_DIVIRQ_CRC) {
            break;
        }
    }
    if ((irq & RC522_DIVIRQ_CRC) == 0) {
        return ESP_ERR_TIMEOUT;
    }

    /* CRC is sent low byte first. Reading it does not go over the air. */
    if (rc522_read(RC522_REG_CRC_RESULT_L, &out[0]) != ESP_OK ||
        rc522_read(RC522_REG_CRC_RESULT_H, &out[1]) != ESP_OK) {
        return ESP_FAIL;
    }
    return rc522_write(RC522_REG_COMMAND, RC522_CMD_IDLE);
}

static bool crc_matches(const uint8_t *data, size_t len)
{
    uint8_t crc[2];
    if (len < 2 || rc522_crc(data, len - 2, crc) != ESP_OK) {
        return false;
    }
    return crc[0] == data[len - 2] && crc[1] == data[len - 1];
}

static bool picc_select(const uint8_t uid[4], uint8_t bcc, uint8_t *sak)
{
    uint8_t frame[9] = {
        PICC_SEL_CL1, PICC_SEL_NVB_COMPLETE, uid[0], uid[1], uid[2], uid[3], bcc,
    };
    if (rc522_crc(frame, 7, &frame[7]) != ESP_OK) {
        return false;
    }

    picc_reply_t reply = picc_transceive(frame, sizeof(frame), 0);
    if (reply.status != PICC_OK || reply.len != 3 || !crc_matches(reply.data, reply.len)) {
        return false;
    }
    *sak = reply.data[0];
    return true;
}

static bool picc_auth(uint8_t block, const uint8_t uid[4])
{
    uint8_t frame[12] = {
        PICC_MF_AUTH_KEY_A, block,
        s_key_a[0], s_key_a[1], s_key_a[2], s_key_a[3], s_key_a[4], s_key_a[5],
        uid[0], uid[1], uid[2], uid[3],
    };

    if (rc522_write(RC522_REG_COMMAND, RC522_CMD_IDLE) != ESP_OK ||
        rc522_write(RC522_REG_COM_IRQ, RC522_IRQ_ALL) != ESP_OK ||
        rc522_write(RC522_REG_FIFO_LEVEL, 0x80) != ESP_OK ||
        rc522_write_fifo(frame, sizeof(frame)) != ESP_OK ||
        rc522_write(RC522_REG_BIT_FRAMING, 0) != ESP_OK ||
        rc522_write(RC522_REG_COMMAND, RC522_CMD_MF_AUTHENT) != ESP_OK) {
        return false;
    }

    uint8_t irq = 0;
    for (int i = 0; i < 2000; i++) {
        if (rc522_read(RC522_REG_COM_IRQ, &irq) != ESP_OK) {
            return false;
        }
        if (irq & RC522_IRQ_IDLE) {
            break;
        }
        if (irq & RC522_IRQ_TIMER) {
            return false;
        }
    }
    if ((irq & RC522_IRQ_IDLE) == 0) {
        return false;
    }

    uint8_t error = 0;
    uint8_t status2 = 0;
    if (rc522_read(RC522_REG_ERROR, &error) != ESP_OK ||
        rc522_read(RC522_REG_STATUS2, &status2) != ESP_OK) {
        return false;
    }
    if (error & (RC522_ERR_PROTOCOL | RC522_ERR_PARITY | RC522_ERR_BUFFER)) {
        return false;
    }
    return (status2 & RC522_STATUS2_CRYPTO1) != 0;
}

static bool picc_read_block(uint8_t block, uint8_t data[16])
{
    uint8_t frame[4] = {PICC_MF_READ, block};
    if (rc522_crc(frame, 2, &frame[2]) != ESP_OK) {
        return false;
    }

    picc_reply_t reply = picc_transceive(frame, sizeof(frame), 0);
    if (reply.status != PICC_OK || reply.len != 18 || !crc_matches(reply.data, reply.len)) {
        return false;
    }
    memcpy(data, reply.data, sizeof(data[0]) * 16);
    return true;
}

static void picc_halt(void)
{
    uint8_t status2 = 0;
    if (rc522_read(RC522_REG_STATUS2, &status2) == ESP_OK && (status2 & RC522_STATUS2_CRYPTO1) != 0) {
        rc522_write(RC522_REG_STATUS2, (uint8_t)(status2 & ~RC522_STATUS2_CRYPTO1));
    }

    uint8_t frame[4] = {PICC_HLTA, 0x00};
    if (rc522_crc(frame, 2, &frame[2]) == ESP_OK) {
        /* A halted card does not answer. Timeout is the expected result. */
        picc_transceive(frame, sizeof(frame), 0);
    }
    rc522_write(RC522_REG_COMMAND, RC522_CMD_IDLE);
}

static void log_block(unsigned addr, const uint8_t data[16])
{
    ESP_LOGI(TAG,
             "sector %u block %u (addr %u): "
             "%02X %02X %02X %02X %02X %02X %02X %02X "
             "%02X %02X %02X %02X %02X %02X %02X %02X",
             addr / 4, addr % 4, addr,
             data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
             data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);
}

static void dump_sector(const uint8_t atqa[2], const uint8_t uid[4], uint8_t bcc)
{
    uint8_t sak = 0;
    if (!picc_select(uid, bcc, &sak)) {
        ESP_LOGW(TAG, "select failed");
        return;
    }

    ESP_LOGI(TAG, "ATQA %02X %02X UID %02X %02X %02X %02X SAK %02X",
             atqa[0], atqa[1], uid[0], uid[1], uid[2], uid[3], sak);
    if (uid[0] == PICC_CASCADE_TAG || (sak & PICC_SAK_CASCADE) != 0) {
        ESP_LOGI(TAG, "UID is longer than 4 bytes; sector read skipped");
        picc_halt();
        return;
    }

    const uint8_t first = (uint8_t)(CONFIG_SAMPLE_RC522_SECTOR * 4);
    if (!picc_auth(first, uid)) {
        ESP_LOGW(TAG, "auth failed for sector %d", CONFIG_SAMPLE_RC522_SECTOR);
        picc_halt();
        return;
    }

    for (uint8_t i = 0; i < 4; i++) {
        uint8_t block[16];
        const uint8_t addr = (uint8_t)(first + i);
        if (!picc_read_block(addr, block)) {
            ESP_LOGW(TAG, "read failed for block addr %u", addr);
            break;
        }
        log_block(addr, block);
    }
    picc_halt();
}

static void poll_card(void)
{
    /* WUPA wakes a card that was halted after the previous sector read. */
    const uint8_t wupa = PICC_WUPA;
    picc_reply_t atqa = picc_transceive(&wupa, 1, 7);
    if (atqa.status == PICC_TIMEOUT) {
        s_done = false;
        return;
    }
    if (atqa.status == PICC_COLLISION) {
        ESP_LOGW(TAG, "more than one card in the field");
        return;
    }
    if (atqa.status != PICC_OK || atqa.len != 2) {
        ESP_LOGW(TAG, "request failed");
        return;
    }

    const uint8_t sel[2] = {PICC_SEL_CL1, 0x20};
    picc_reply_t uid = picc_transceive(sel, sizeof(sel), 0);
    if (uid.status == PICC_COLLISION) {
        ESP_LOGW(TAG, "UID collision");
        return;
    }
    if (uid.status != PICC_OK || uid.len != 5) {
        ESP_LOGW(TAG, "anticollision failed");
        return;
    }

    const uint8_t bcc = (uint8_t)(uid.data[0] ^ uid.data[1] ^ uid.data[2] ^ uid.data[3]);
    if (bcc != uid.data[4]) {
        ESP_LOGW(TAG, "BCC mismatch, raw %02X %02X %02X %02X %02X",
                 uid.data[0], uid.data[1], uid.data[2], uid.data[3], uid.data[4]);
        return;
    }
    if (s_done && memcmp(s_done_uid, uid.data, sizeof(s_done_uid)) == 0) {
        return;
    }

    dump_sector(atqa.data, uid.data, bcc);
    memcpy(s_done_uid, uid.data, sizeof(s_done_uid));
    s_done = true;
}

void sample_run(void)
{
    ESP_LOGI(TAG, "SCK=%d MOSI=%d MISO=%d CS=%d RST=%d",
             CONFIG_SAMPLE_RC522_SCK, CONFIG_SAMPLE_RC522_MOSI, CONFIG_SAMPLE_RC522_MISO,
             CONFIG_SAMPLE_RC522_CS, CONFIG_SAMPLE_RC522_RST);

    ESP_ERROR_CHECK(spi_init());
    ESP_ERROR_CHECK(rc522_init());

    uint8_t version = 0;
    ESP_ERROR_CHECK(rc522_read(RC522_REG_VERSION, &version));
    ESP_LOGI(TAG, "version 0x%02X", version);
    if (version == 0x00 || version == 0xFF) {
        ESP_LOGW(TAG, "chip did not answer; check power, CS (module SDA), and SPI mode");
    }

    ESP_LOGI(TAG, "sector %d, key A FF FF FF FF FF FF", CONFIG_SAMPLE_RC522_SECTOR);
    ESP_LOGI(TAG, "hold a card on the antenna");
    for (;;) {
        poll_card();
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}
