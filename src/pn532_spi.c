#include "pn532_spi.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "PN532_SPI";
static gpio_num_t s_ss_pin; // add this global

#define PN532_MAX_FRAME 264 // generous; covers largest DESFire secure response we expect
#define PN532_SPI_STATREAD 0x02
#define PN532_SPI_DATAWRITE 0x01
#define PN532_SPI_DATAREAD 0x03
#define PN532_SPI_READY 0x01
#define PN532_CMD_RFCONFIGURATION 0x32

#define PN532_PREAMBLE 0x00
#define PN532_STARTCODE1 0x00
#define PN532_STARTCODE2 0xFF
#define PN532_POSTAMBLE 0x00
#define PN532_HOSTTOPN532 0xD4
#define PN532_PN532TOHOST 0xD5

#define PN532_CMD_GETFIRMWAREVERSION 0x02
#define PN532_CMD_SAMCONFIGURATION 0x14
#define PN532_CMD_INLISTPASSIVETARGET 0x4A
#define PN532_CMD_INDATAEXCHANGE 0x40

static spi_device_handle_t s_spi;
static gpio_num_t s_irq_pin;
static SemaphoreHandle_t s_irq_sem;

static inline uint8_t reverse_byte(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

static void IRAM_ATTR pn532_irq_isr_handler(void *arg)
{
    BaseType_t higher_prio_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_irq_sem, &higher_prio_woken);
    if (higher_prio_woken)
    {
        portYIELD_FROM_ISR();
    }
}

// Returns true when PN532 SPI status register indicates "ready"
static bool pn532_spi_is_ready(void)
{
    // uint8_t stat_cmd = reverse_byte(PN532_SPI_STATREAD); // 0x02
    uint8_t stat_cmd = PN532_SPI_STATREAD; // 0x02
    uint8_t status;
    // print reversed command for debugging
    // ESP_LOGI(TAG, "Sending PN532 SPI status read command: 0x%02X", PN532_SPI_STATREAD);

    gpio_set_level(s_ss_pin, 0); // Assert CS
    // esp_rom_delay_us(2000);      // let PN532 SPI logic settle before clocking (Adafruit-equivalent)
    vTaskDelay(pdMS_TO_TICKS(2)); // 2secs delay
    // Send STATUS READ command
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &stat_cmd,
        .rx_buffer = NULL,
    };
    spi_device_transmit(s_spi, &t);

    // Read one status byte
    spi_transaction_t r = {
        .length = 8,
        .tx_buffer = NULL, // dummy
        .rx_buffer = &status,
    };
    spi_device_transmit(s_spi, &r);
    gpio_set_level(s_ss_pin, 1); // Deassert CS after reading status
    // return (status & PN532_SPI_READY) != 0;
    // status = reverse_byte(status); // Reverse the bits back to original order
    // print the status for debugging
    // ESP_LOGI(TAG, "PN532 SPI status: 0x%02X", status);
    return status == 0x01; // ready if exactly 0x01
}

// Block until the PN532 is ready to accept a command (or timeout)
static bool pn532_wait_ready_poll(uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms))
    {
        if (pn532_spi_is_ready())
        {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Poll every 10 ms
    }
    return false;
}

static bool pn532_wait_irq(uint32_t timeout_ms)
{
    // Catches the case where the response was already sitting ready
    // before we started waiting — a real race, not a hypothetical one.
    if (gpio_get_level(s_irq_pin) == 0)
    {
        return true;
    }
    return xSemaphoreTake(s_irq_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

// ---- Low level SPI transactions -------------------------------------

static bool spi_write_bytes(const uint8_t *data, size_t len)
{
    gpio_set_level(s_ss_pin, 0);  // Assert CS
    vTaskDelay(pdMS_TO_TICKS(2)); // 2secs delay
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
        .rx_buffer = NULL,
    };
    esp_err_t ret = spi_device_transmit(s_spi, &t);
    gpio_set_level(s_ss_pin, 1); // Deassert CS
    return ret == ESP_OK;
}

static bool spi_read_bytes(uint8_t *out, size_t len)
{
    uint8_t dummy_tx[PN532_MAX_FRAME] = {0};
    gpio_set_level(s_ss_pin, 0);  // Assert CS
    vTaskDelay(pdMS_TO_TICKS(2)); // 2secs delay
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = dummy_tx,
        .rx_buffer = out,
    };
    esp_err_t ret = spi_device_transmit(s_spi, &t);
    gpio_set_level(s_ss_pin, 1); // Deassert CS
    return ret == ESP_OK;
}

static void pn532_wakeup(void)
{
    gpio_set_level(s_ss_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(s_ss_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10)); // let the chip come out of power-down
}

// Sends `cmd_len` bytes, then immediately reads `out_len` bytes,
// all under a single continuous CS-low session — matches the
// PN532 SPI protocol (datasheet §2.4.1.1): the control byte and
// the data that follows it are one exchange, not two.
static bool spi_cmd_then_read(const uint8_t *cmd, size_t cmd_len,
                              uint8_t *out, size_t out_len)
{
    gpio_set_level(s_ss_pin, 0); // Assert CS once
    // esp_rom_delay_us(2000);      // let PN532 SPI logic settle before clocking
    vTaskDelay(pdMS_TO_TICKS(20)); // 2secs delay
    spi_transaction_t tx = {
        .length = cmd_len * 8,
        .tx_buffer = cmd,
        .rx_buffer = NULL,
    };
    esp_err_t ret = spi_device_transmit(s_spi, &tx);

    if (ret == ESP_OK && out_len > 0)
    {
        uint8_t dummy_tx[PN532_MAX_FRAME] = {0};
        spi_transaction_t rx = {
            .length = out_len * 8,
            .tx_buffer = dummy_tx,
            .rx_buffer = out,
        };
        ret = spi_device_transmit(s_spi, &rx);
    }

    gpio_set_level(s_ss_pin, 1); // Deassert CS once, after both halves
    return ret == ESP_OK;
}

static void pn532_flush(void)
{
    // Read and discard any stale data / error frame from PN532
    uint8_t stat_cmd = PN532_SPI_STATREAD;      // 0x02
    uint8_t data_read_cmd = PN532_SPI_DATAREAD; // 0x03
    uint8_t status;
    for (int attempt = 0; attempt < 5; attempt++)
    {
        // Send STATUS READ
        spi_cmd_then_read(&stat_cmd, 1, &status, 1);

        if (!(status & 0x01))
        {
            // No more data to read
            break;
        }

        // Data is waiting – read and discard the response frame
        uint8_t dummy[32];
        spi_cmd_then_read(&data_read_cmd, 1, dummy, sizeof(dummy));
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Sends a full PN532 information frame containing `cmd` + `params`.
static bool pn532_write_frame(uint8_t cmd, const uint8_t *params, uint8_t params_len)
{
    uint8_t frame[PN532_MAX_FRAME];
    uint8_t idx = 0;

    uint8_t data_len = params_len + 1;                        // +1 for the command byte itself
    uint8_t len_checksum = (uint8_t)(0x100 - (1 + data_len)); // LCS over TFI+DATA length

    frame[idx++] = PN532_SPI_DATAWRITE;
    frame[idx++] = PN532_PREAMBLE;
    frame[idx++] = PN532_STARTCODE1;
    frame[idx++] = PN532_STARTCODE2;
    frame[idx++] = 1 + data_len; // LEN = TFI + cmd + params
    frame[idx++] = len_checksum;
    frame[idx++] = PN532_HOSTTOPN532; // TFI

    uint8_t dcs_sum = PN532_HOSTTOPN532;
    frame[idx++] = cmd;
    dcs_sum += cmd;

    for (uint8_t i = 0; i < params_len; i++)
    {
        frame[idx++] = params[i];
        dcs_sum += params[i];
    }

    frame[idx++] = (uint8_t)(0x100 - dcs_sum); // DCS
    frame[idx++] = PN532_POSTAMBLE;

    ESP_LOGD(TAG, "TX frame, %d bytes", idx);
    return spi_write_bytes(frame, idx);
}

// Confirms the 6-byte ACK frame the PN532 sends after accepting a command.
static bool pn532_read_ack(uint32_t timeout_ms)
{
    // send the DATAREAD command to read the ACK frame
    uint8_t hdr = PN532_SPI_DATAREAD; // 0x03
    uint8_t ack[6];
    if (!spi_cmd_then_read(&hdr, 1, ack, 6))
    {
        return false;
    }

    static const uint8_t expected_ack[6] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
    if (memcmp(ack, expected_ack, 6) != 0)
    {
        ESP_LOGW(TAG, "Bad ACK: %02X %02X %02X %02X %02X %02X",
                 ack[0], ack[1], ack[2], ack[3], ack[4], ack[5]);
        return false;
    }
    return true;
}

// Reads the PN532's response frame after a command, extracting just the
// payload (i.e. everything after TFI, matching Adafruit's response semantics).
static bool pn532_read_response(uint8_t *out, uint8_t *out_len, uint32_t timeout_ms)
{
    if (!pn532_wait_irq(timeout_ms))
    {
        ESP_LOGW(TAG, "Timeout waiting for IRQ (response)");
        return false;
    }

    uint8_t hdr = PN532_SPI_DATAREAD;
    uint8_t raw[PN532_MAX_FRAME];
    if (!spi_cmd_then_read(&hdr, 1, raw, PN532_MAX_FRAME))
    {
        return false;
    }

    if (raw[0] != PN532_PREAMBLE || raw[1] != PN532_STARTCODE1 || raw[2] != PN532_STARTCODE2)
    {
        ESP_LOGW(TAG, "Bad frame preamble: %02X %02X %02X", raw[0], raw[1], raw[2]);
        return false;
    }

    uint8_t len = raw[3];
    uint8_t lcs = raw[4];
    if ((uint8_t)(len + lcs) != 0)
    {
        ESP_LOGW(TAG, "LEN/LCS checksum mismatch");
        return false;
    }

    if (raw[5] != PN532_PN532TOHOST)
    {
        ESP_LOGW(TAG, "Unexpected TFI: %02X", raw[5]);
        return false;
    }

    // Payload = everything after TFI, excluding the trailing DCS/postamble.
    // len counts TFI + command-echo + data, so actual returned data is len - 2
    // (command-echo byte is skipped here to match inDataExchange-style output).
    uint8_t payload_len = len - 2;
    if (payload_len > *out_len)
    {
        ESP_LOGW(TAG, "Response too large for buffer (%d > %d)", payload_len, *out_len);
        return false;
    }

    memcpy(out, raw + 7, payload_len); // skip preamble/start/len/lcs/tfi/cmd-echo
    *out_len = payload_len;
    return true;
}

// RFConfiguration, CfgItem 5 (MaxRetries): sets how many times the PN532
// retries InListPassiveTarget's activation phase before giving up.
// AN133910 only mentions this parameter by name (§3.3.1.1) — full byte
// layout is in the PN532 User Manual UM0502-06, CfgItem 0x05:
//   [CfgItem=0x05, MxRtyATR, MxRtyPSL, MxRtyPassiveActivation]
// Default MxRtyPassiveActivation is 0xFF (retry forever). Setting it to
// a small finite value makes InListPassiveTarget return quickly with
// "0 targets found" instead of blocking your polling loop.
bool pn532_set_passive_activation_retries(uint8_t max_retries)
{
    uint8_t params[] = {
        0x05,       // CfgItem: MaxRetries
        0xFF,       // MxRtyATR      - leave at default
        0x01,       // MxRtyPSL      - leave at default
        max_retries // MxRtyPassiveActivation - the one we're tuning
    };

    if (!pn532_write_frame(PN532_CMD_RFCONFIGURATION, params, sizeof(params)))
    {
        ESP_LOGE(TAG, "Failed to send RFConfiguration");
        return false;
    }

    if (!pn532_wait_ready_poll(1000))
    {
        ESP_LOGE(TAG, "Timeout waiting for RFConfiguration ACK-ready");
        return false;
    }
    if (!pn532_read_ack(1000))
    {
        return false;
    }

    if (!pn532_wait_ready_poll(1000))
    {
        ESP_LOGE(TAG, "Timeout waiting for RFConfiguration response");
        return false;
    }

    uint8_t resp[4];
    uint8_t resp_len = sizeof(resp);
    if (!pn532_read_response(resp, &resp_len, 1000))
    {
        ESP_LOGE(TAG, "No response to RFConfiguration");
        return false;
    }

    ESP_LOGI(TAG, "RFConfiguration: MaxRtyPassiveActivation set to 0x%02X", max_retries);
    return true;
}

// ---- Public API -------------------------------------------------------

bool pn532_init(const pn532_config_t *cfg)
{
    s_irq_pin = cfg->pin_irq;
    s_irq_sem = xSemaphoreCreateBinary();

    spi_bus_config_t bus_cfg = {
        .miso_io_num = cfg->pin_miso,
        .mosi_io_num = cfg->pin_mosi,
        .sclk_io_num = cfg->pin_sck,
        .quadwp_io_num = -1, // not used
        .quadhd_io_num = -1, // not used
        .max_transfer_sz = PN532_MAX_FRAME,
    };
    if (spi_bus_initialize(cfg->spi_host, &bus_cfg, SPI_DMA_CH_AUTO) != ESP_OK)
    {
        ESP_LOGE(TAG, "spi_bus_initialize failed");
        return false;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 1000000, // 1MHz to start; PN532 supports up to 5MHz once validated
        .mode = 0,                 // SPI mode 0 (CPOL=0, CPHA=0)
        //.spics_io_num = cfg->pin_ss,
        .spics_io_num = -1, // disable CS
        .queue_size = 7,
        //.flags = 0,
        .flags = SPI_DEVICE_TXBIT_LSBFIRST | SPI_DEVICE_RXBIT_LSBFIRST, // PN532 SPI is LSB-first
    };

    // Manual SS control
    s_ss_pin = cfg->pin_ss;
    gpio_set_direction(s_ss_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(s_ss_pin, 1); // idle high

    if (spi_bus_add_device(cfg->spi_host, &dev_cfg, &s_spi) != ESP_OK)
    {
        ESP_LOGE(TAG, "spi_bus_add_device failed");
        return false;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << cfg->pin_irq,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(cfg->pin_irq, pn532_irq_isr_handler, NULL);

    ESP_LOGI(TAG, "SPI bus + IRQ initialized");
    return true;
}

bool pn532_begin(void)
{
    ESP_LOGI(TAG, "IRQ=%d", gpio_get_level(s_irq_pin));

    // --- Wake PN532 from power-down ---
    // Elechouse V3 boards boot into power-down. A CS low->high pulse
    // (AN133910 sec 2.4.1.1 / user manual wake-up sequence) is required
    // before the status register will report anything but 0x00.
    pn532_wakeup();

    // --- Now send SAMConfiguration command ---
    uint8_t sam_params[] = {0x01, 0x14, 0x01}; // Normal mode, timeout 20*50ms, use IRQ
    if (!pn532_write_frame(PN532_CMD_SAMCONFIGURATION, sam_params, sizeof(sam_params)))
    {
        ESP_LOGE(TAG, "Failed to send SAMConfiguration");
        return false;
    }
    pn532_wait_ready_poll(1000); // Wait for the PN532 to be ready before reading ACK

    if (!pn532_read_ack(1000))
    {
        return false;
    }
    pn532_wait_ready_poll(1000); // Wait for the PN532 to be ready before reading ACK

    uint8_t resp[16];
    uint8_t resp_len = sizeof(resp);
    if (!pn532_read_response(resp, &resp_len, 1000))
    {
        ESP_LOGE(TAG, "No response to SAMConfiguration");
        return false;
    }

    ESP_LOGI(TAG, "SAMConfiguration OK");
    return true;
}

bool pn532_get_firmware_version(uint8_t *ver_out, uint8_t ver_out_len)
{
    if (!pn532_write_frame(PN532_CMD_GETFIRMWAREVERSION, NULL, 0))
    {
        return false;
    }
    if (!pn532_read_ack(1000))
    {
        return false;
    }
    return pn532_read_response(ver_out, &ver_out_len, 1000);
}

bool pn532_read_passive_target_uid(uint8_t *uid, uint8_t *uid_len, uint32_t timeout_ms)
{
    uint8_t params[] = {0x01, 0x00}; // MaxTg=1, BrTy=106kbps ISO14443A
    if (!pn532_write_frame(PN532_CMD_INLISTPASSIVETARGET, params, sizeof(params)))
    {
        return false;
    }

    if (!pn532_wait_ready_poll(1000)) // Wait for the PN532 to be ready before reading ACK
    {
        ESP_LOGD(TAG, "No response ready yet (no target)");
        return false;
    }

    if (!pn532_read_ack(1000))
    {
        return false;
    }

    if (!pn532_wait_ready_poll(1000)) // Wait for the PN532 to be ready before reading ACK
    {
        ESP_LOGD(TAG, "No response ready yet (no target)");
        return false;
    }

    uint8_t resp[32];
    uint8_t resp_len = sizeof(resp);
    if (!pn532_read_response(resp, &resp_len, timeout_ms))
    {
        return false;
    }

    // Response payload: NbTg, Tg, SENS_RES(2), SEL_RES(1), NFCIDLength, NFCID...
    if (resp_len < 6 || resp[0] == 0)
    {
        ESP_LOGW(TAG, "No target found");
        return false;
    }

    uint8_t nfcid_len = resp[5];
    if (nfcid_len > *uid_len)
    {
        ESP_LOGW(TAG, "UID longer than buffer");
        return false;
    }
    memcpy(uid, resp + 6, nfcid_len);
    *uid_len = nfcid_len;
    return true;
}

bool pn532_in_data_exchange(const uint8_t *send, uint8_t send_len,
                            uint8_t *response, uint8_t *response_len)
{
    uint8_t params[PN532_MAX_FRAME];
    params[0] = 0x01; // Target number 1 (matches single-target InListPassiveTarget above)
    memcpy(params + 1, send, send_len);

    if (!pn532_write_frame(PN532_CMD_INDATAEXCHANGE, params, send_len + 1))
    {
        return false;
    }
    if(!pn532_wait_ready_poll(1000))
    {
        ESP_LOGW(TAG, "Timeout waiting for InDataExchange ACK-ready");
        return false;
    }
    if (!pn532_read_ack(1000))
    {
        return false;
    }
    if(!pn532_wait_ready_poll(1000))
    {
        ESP_LOGW(TAG, "Timeout waiting for InDataExchange response");
        return false;
    }

    uint8_t resp[PN532_MAX_FRAME];
    uint8_t resp_len = (sizeof(resp) > UINT8_MAX) ? UINT8_MAX : sizeof(resp); // cap to 255 for uint8_t
    if (!pn532_read_response(resp, &resp_len, 500))
    {
        return false;
    }

    // resp[0] is the InDataExchange status byte (0x00 = success); the
    // rest is the card's actual APDU response, which is what the DESFire
    // layer above expects — matches Adafruit's inDataExchange output shape.
    if (resp[0] != 0x00)
    {
        ESP_LOGW(TAG, "InDataExchange status error: 0x%02X", resp[0]);
        return false;
    }

    uint8_t payload_len = resp_len - 1;
    if (payload_len > *response_len)
    {
        ESP_LOGW(TAG, "Response too large for caller's buffer");
        return false;
    }
    memcpy(response, resp + 1, payload_len);
    *response_len = payload_len;
    return true;
}