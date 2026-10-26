#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "hello";

void sample_run(void)
{
    for (;;) {
        ESP_LOGI(TAG, "hello from ESP32-P4");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
