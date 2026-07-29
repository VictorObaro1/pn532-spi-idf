# pn532-spi-idf

A native **ESP-IDF** SPI driver for the PN532 NFC/RFID module — no Arduino, no `Adafruit_PN532` dependency, just direct `driver/spi_master.h` and `driver/gpio.h` calls.

Built while developing an ESP32-S3-based MIFARE DESFIRE EV2 card reader that reads MIFARE DESFire cards over SPI. This repo is the transport-layer driver, extracted and open-sourced on its own.

## Why not just use Adafruit_PN532?

The Adafruit library is a great starting point, but it's built around the Arduino `SPI` abstraction, and a lot of what makes PN532-over-SPI actually reliable is hidden a few layers down — timing quirks that matter a lot more once you're running FreeRTOS tasks alongside BLE, Wi-Fi, and a display, competing for the bus.

This driver exposes that layer directly:

- **CS-settle timing** — the PN532 needs a deliberate delay after asserting chip-select before the first clock edge, or the first byte of a frame gets dropped.
- **IRQ vs STATUS-byte race conditions** — there are two ways to know the PN532 has a response ready (its `IRQ` pin, or polling a status byte), and they can disagree if you don't handle the timing carefully.
- **Full-duplex SPI framing** — writes and reads happen in the same transaction, so the driver has to loop back and discard bytes correctly instead of treating it like a half-duplex UART.

If you've hit mysterious PN532 read failures with Arduino libraries under load, this is the layer where those bugs actually live.

## Features

- Pure ESP-IDF component — `driver/spi_master.h`, `driver/gpio.h`, `esp_timer.h`, `esp_log.h`
- Works alongside FreeRTOS, BLE, Wi-Fi, and other SPI/I2C peripherals without needing Arduino's cooperative scheduling assumptions
- Passive-mode card detection with configurable retry count via `RFConfiguration`
- Tested end-to-end reading MIFARE DESFire UIDs

## Hardware

Tested on an **ESP32-S3** wired to a PN532 breakout in SPI mode (check your board's DIP switches / solder jumpers — most PN532 breakouts default to I2C or HSU and need to be set to SPI mode).

| PN532 pin | ESP32-S3 GPIO (example) |
|-----------|--------------------------|
| SCK       | GPIO16                  |
| MISO      | GPIO2                   |
| MOSI      | GPIO1                   |
| SS (CS)   | GPIO14                  |
| IRQ       | GPIO47                  |

Pin assignments are passed in at runtime via `pn532_config_t` — nothing is hardcoded, so remap freely.

## Quick start

Add as a dependency in your project's `idf_component.yml`:

```yaml
dependencies:
  <your-github-username>/pn532-spi-idf: "^1.0.0"
```

Or clone directly into your project's `components/` folder.

```c
#include "pn532_spi.h"

void app_main(void)
{
    pn532_config_t cfg = {
        .spi_host = SPI2_HOST,
        .pin_sck  = GPIO_NUM_16,
        .pin_miso = GPIO_NUM_2,
        .pin_mosi = GPIO_NUM_1,
        .pin_ss   = GPIO_NUM_14,
        .pin_irq  = GPIO_NUM_47,
    };

    pn532_init(&cfg);
    pn532_sam_config();

    uint8_t uid[7];
    uint8_t uid_len;
    if (pn532_read_passive_target_id(&uid, &uid_len)) {
        ESP_LOGI("APP", "Card UID read successfully");
    }
}
```

See [`examples/read_uid`](examples/read_uid) for a complete, buildable project.

## Roadmap

- [ ] I2C transport variant
- [ ] Async / queue-based API instead of blocking calls
- [ ] Unit tests for frame parsing (no hardware required)
- [ ] Example built on top: MIFARE DESFire authentication

Contributions welcome — see [`CONTRIBUTING.md`](CONTRIBUTING.md).

## License

MIT — see [`LICENSE`](LICENSE).