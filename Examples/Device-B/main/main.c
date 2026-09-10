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

/* ------------------------- wiring ------------------------- */
const uint8_t PIN_SCK  = 25;
const uint8_t PIN_MISO = 26;
const uint8_t PIN_MOSI = 27;
const uint8_t PIN_CS   = 14;
const uint8_t PIN_IRQ  = 13;
const uint8_t PIN_RST  = 32;

/* ------------------- shared radio settings -------------------
   The PAN, channel and mode MUST match on both boards. */
#define MY_PAN_ID        0xDECA
#define MY_SHORT_ADDR    0x1002   /* Device B */
uint16_t antenna_delay = 16464;   /* calibrated value - same on BOTH boards */

/* ------------------------- pairing -------------------------
   Pair with the OTHER module using ITS ESP32 BLE MAC (6 bytes, MSB first,
   printed at boot as "My BLE MAC: ..."). Device B uses Device A's MAC. */
static const uint8_t peer_eui[6] = {0x28, 0x05, 0xA5, 0x2A, 0x66, 0xCC};

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

    /* Init SPI bus + reset + LDE microcode load */
    dw1000_init(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS, PIN_IRQ, PIN_RST,
                 MY_PAN_ID, MY_SHORT_ADDR,
                  DW1000_MODE_SHORTDATA_FAST_ACCURACY,
                  DW1000_CHANNEL_5, antenna_delay);

    /* pair with the tag */
    dw1000_set_peer_eui(peer_eui);

    /* answer ranging + calibration requests forever */
    // dw1000_run_anchor(PIN_IRQ);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // ----- Run Anchor ------
    // int ping_counter = 0;

    // while(1){
    //     bool ok = dw1000_ping(PIN_IRQ, 100);
    //     ESP_LOGI("MAIN", "Ping Counter: %d", ping_counter);
    //     ping_counter++;
    //     if (ok) ESP_LOGI("MAIN", ">>   Ping Successful");
    //     else ESP_LOGW("MAIN", "NO Ping");
    //     vTaskDelay(pdMS_TO_TICKS(500));
    // }
}

void app_main(void)
{
    xTaskCreate(dw1000_radio_task, "dw1000_radio", 8192, NULL, 5, NULL);
}
