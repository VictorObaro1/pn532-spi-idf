#ifndef PN532_SPI_H
#define PN532_SPI_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    spi_host_device_t spi_host;
    gpio_num_t pin_sck;
    gpio_num_t pin_miso;
    gpio_num_t pin_mosi;
    gpio_num_t pin_ss;
    gpio_num_t pin_irq;
} pn532_config_t;

// One-time setup: SPI bus, device, IRQ ISR. Call once at boot.
bool pn532_init(const pn532_config_t *cfg);

// Wakes the PN532 from power-down and runs SAMConfiguration (normal mode).
// Call once after pn532_init(), before any card operations.
bool pn532_begin(void);

bool pn532_set_passive_activation_retries(uint8_t max_retries);

// Reads firmware version — good first smoke test, no card needed.
bool pn532_get_firmware_version(uint8_t *ver_out, uint8_t ver_out_len);

// Detects a passive ISO14443A target (card) and returns its UID.
bool pn532_read_passive_target_uid(uint8_t *uid, uint8_t *uid_len, uint32_t timeout_ms);

// Direct equivalent of Adafruit's nfc.inDataExchange() — sends a raw
// command/APDU to the currently-selected target, returns the response.
bool pn532_in_data_exchange(const uint8_t *send, uint8_t send_len,
                            uint8_t *response, uint8_t *response_len);

#ifdef __cplusplus
}
#endif
#endif