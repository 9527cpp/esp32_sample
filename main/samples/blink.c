#include "sdkconfig.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "blink";

void sample_run(void)
{
    const gpio_num_t pin = (gpio_num_t)CONFIG_SAMPLE_BLINK_GPIO;
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_LOGI(TAG, "toggling GPIO %d", (int)pin);

    bool level = false;
    for (;;) {
        level = !level;
        gpio_set_level(pin, level);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
