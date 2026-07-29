# Contributing to pn532-spi-idf

Thanks for considering a contribution, this started as a driver extracted from a real MIFARE DESFIRE EV2 Card reader project, and the goal is to make the SPI transport layer for the PN532 something the wider ESP-IDF community can rely on and build on.

## Getting set up

1. Clone the repo and `cd` into `examples/read_uid` that's a complete, standalone ESP-IDF project that consumes the component via `EXTRA_COMPONENT_DIRS`.
2. Set your target and build:
   ```
   idf.py set-target esp32s3
   idf.py build
   ```
3. Flash to hardware with a PN532 wired in SPI mode (see the pin table in the main [README](README.md)) and confirm you get a UID read:
   ```
   idf.py -p <PORT> flash monitor
   ```

You'll need actual PN532 hardware to test most changes — this driver talks to real SPI timing behavior, so it isn't easily mockable.

## Making changes

- Keep the component itself (`src/`, `include/`) free of anything project-specific — no hardcoded pins, no board-specific `#define`s. Runtime config goes through `pn532_config_t`.
- If you're touching SPI timing (CS-settle delays, IRQ/STATUS polling), please describe what hardware you tested on and how you confirmed the fix, these bugs are often intermittent and hard to catch without a logic analyzer, so a clear repro/verification note saves everyone time in review.
- Match the existing code style (standard ESP-IDF conventions: `snake_case`, `esp_err_t` return codes where applicable, `ESP_LOGx` for logging rather than `printf`).

## Submitting a PR

1. Fork, branch, commit with a clear message describing what changed and why.
2. Make sure `idf.py build` succeeds from `examples/read_uid` before opening the PR.
3. Update the README if you've added a feature or changed the public API in `pn532_spi.h`.
4. Open the PR against `main` CI will run a build check automatically.

## Good first issues

If you're looking for a place to start:

- **I2C transport variant**: the current driver is SPI-only; an I2C mode alongside it would broaden hardware support.
- **Unit tests for frame parsing**: the ACK/NACK and response-frame parsing logic doesn't need real hardware and could be tested standalone with fixed byte sequences.
- **Additional target validation**: confirm the driver builds and works on ESP32 / ESP32-C3, not just ESP32-S3, and report back.

## Reporting bugs

Please include:
- ESP-IDF version (`idf.py --version`)
- Target chip
- PN532 breakout board / module used
- Full serial log output if the card isn't detected

## Code of conduct

Be respectful, assume good faith, and keep discussion focused on the technical problem. 
This is a small hobby/community project no formal enforcement process, just basic decency.