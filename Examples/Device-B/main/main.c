/*
 * DW1000 ESP-IDF example - Device B (ANCHOR / ranging responder)
 *
 * Answers a paired TAG:
 *   - replies to POLL with POLL_ACK,
 *   - computes the DS-TWR distance and sends it back (RANGE_REPORT),
 *   - accepts joint antenna-delay calibration updates from the tag
 *     (CAL_SET / CAL_ACK) so both boards stay in sync.
 *
 * Runs forever (never returns). No calibration is started here - the TAG
 * drives the calibration (see Examples/Device-A).
 *
 * Build / flash / monitor:
 *   idf.py set-target esp32
 *   idf.py build flash monitor
 *
 * Wiring (Adafruit Feather ESP32 <-> DW1000):
 *   SCK 25 / MISO 26 / MOSI 27 / CS 14 / IRQ 13 / RST 32   (module at 3.3 V)
 */
#include <stdio.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_mac.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "my_dw1000.h"
#include "lora.h"
#include "nvs_flash.h"
#include "nvs.h"
/* ------------------------- wiring ------------------------- */
// ----- for ESP32
// const uint8_t PIN_SCK  = 25;
// const uint8_t PIN_MISO = 26;
// const uint8_t PIN_MOSI = 27;
// const uint8_t PIN_CS   = 14;
// const uint8_t PIN_IRQ  = 13;
// const uint8_t PIN_RST  = 32;

// ----- for ESP32S3
const uint8_t PIN_SCK  = 12;
const uint8_t PIN_MISO = 13;
const uint8_t PIN_MOSI = 11;

const uint8_t PIN_CS   = 10;
const uint8_t PIN_IRQ  = 14;
const uint8_t PIN_RST  = 9;

const uint8_t LORA_SS  = 46;
const uint8_t LORA_RST  = 3;

// const uint8_t PIN_SCK  = 7;
// const uint8_t PIN_MISO = 8;
// const uint8_t PIN_MOSI = 6;
// const uint8_t PIN_CS   = 5;
// const uint8_t PIN_IRQ  = 9;
// const uint8_t PIN_RST  = 4;

/* ------------------- shared radio settings -------------------
   The PAN, channel and mode MUST match on both boards. */
#define MY_PAN_ID        0xDECA
#define MY_SHORT_ADDR    0x1002   /* Device B */
uint16_t antenna_delay = 16464;   /* calibrated value - same on BOTH boards */

/* ------------------------- pairing -------------------------
   Pair with the OTHER module using ITS ESP32 BLE MAC (6 bytes, MSB first,
   printed at boot as "My BLE MAC: ..."). Device B uses Device A's MAC. */
static const uint8_t peer_eui[6] = {0x28, 0x05, 0xA5, 0x2A, 0x66, 0xCE};

#include "driver/spi_master.h"

/* -------------------------- task --------------------------- */
static void dw1000_radio_task(void *arg)
{
    uint8_t mac[6];

    /* Print this board's BLE MAC - you need the OTHER board's MAC to pair. */
    if (esp_read_mac(mac, ESP_MAC_BT) == ESP_OK) {
        ESP_LOGW("DW1000", "My BLE MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 (unsigned)mac[0], (unsigned)mac[1], (unsigned)mac[2],
                 (unsigned)mac[3], (unsigned)mac[4], (unsigned)mac[5]);
    }
    
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0, // LORA
        // .max_transfer_sz   = 1024,   //DW1000
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));


    lora_init(LORA_SS,LORA_RST,PIN_MOSI,PIN_MISO,PIN_SCK);
    /* Init SPI bus + reset + LDE microcode load */
    dw1000_init(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS, PIN_IRQ, PIN_RST,
                 MY_PAN_ID, MY_SHORT_ADDR,
                  DW1000_MODE_LONGDATA_FAST_ACCURACY,
                  DW1000_CHANNEL_5, antenna_delay);

    /* pair with the tag */
    dw1000_set_peer_eui(peer_eui);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

void app_main(void)
{

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW("TAG", "NVS partition corrupted, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    if (err != ESP_OK) {
        ESP_LOGE("TAG", "Failed to initialize NVS: %s", esp_err_to_name(err));
    }

    ESP_LOGI("TAG", "NVS initialized successfully");

    xTaskCreate(dw1000_radio_task, "dw1000_radio", 8192, NULL, 5, NULL);
}
