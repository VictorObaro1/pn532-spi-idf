#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "pn532_spi.h"

void app_main(void)
{
    pn532_config_t cfg = {
        .spi_host = SPI2_HOST,
        .pin_sck = GPIO_NUM_16,
        .pin_miso = GPIO_NUM_2,
        .pin_mosi = GPIO_NUM_1,
        .pin_ss = GPIO_NUM_14,
        .pin_irq = GPIO_NUM_47,
    };

    if (!pn532_init(&cfg) || !pn532_begin())
    {
        ESP_LOGE("MAIN", "PN532 init failed");
        return;
    }

    if (!pn532_set_passive_activation_retries(0x01))
    {
        ESP_LOGW("MAIN", "Failed to tune MaxRtyPassiveActivation, continuing with defaults");
    }

    uint8_t ver[4];
    if (pn532_get_firmware_version(ver, sizeof(ver)))
    {
        ESP_LOGI("MAIN", "PN532 firmware: IC=0x%02X Ver=%d.%d",
                 ver[0], ver[1], ver[2]);
    }

    while (1)
    {
        uint8_t uid[7];
        uint8_t uid_len = sizeof(uid);
        if (pn532_read_passive_target_uid(uid, &uid_len, 1000))
        {
            ESP_LOGI("MAIN", "Card UID length: %d", uid_len);
            for (int i = 0; i < uid_len; i++)
            {
                printf("%02X ", uid[i]);
            }
            printf("\n");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
