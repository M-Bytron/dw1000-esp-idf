/*
 * ESP32 DW1000 connectivity test - ESP-IDF (C)
 *
 * STEP 1 (this build): ONLY verify that the DW1000 module is connected and
 * reachable over SPI. The firmware initializes the SPI bus / driver and reads
 * the DW1000 device registers ("probe").
 *
 * All DW1000 logic lives in the pure-C component library "DW1000"
 * (components/DW1000/my_dw1000.h) - plain C functions only, no classes.
 *
 * Wiring (Adafruit Feather ESP32  <->  DW1000 / DWM1000):
 *   SCK  (GPIO 25)  <-> SPICLK
 *   MISO (GPIO 26)  <-> SPIMISO
 *   MOSI (GPIO 27)  <-> SPIMOSI
 *   CS   (GPIO 14)  <-> SPICSn
 *   IRQ  (GPIO 13)  <-> IRQ
 *   RST  (GPIO 32)  <-> RSTn
 *
 * Notes:
 *  - The module must be powered with 3.3 V (the Feather's 3V pin is OK).
 *  - GPIO12 (RSTn) is a boot strapping pin on the ESP32; the driver leaves
 *    it floating except for a brief low pulse during reset, so boot is fine.
 *
 * Build / flash / monitor (ESP-IDF):
 *   idf.py set-target esp32
 *   idf.py -p COMx flash monitor
 */

#include <stdio.h>
#include <stdint.h>
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "my_dw1000.h"

/* DW1000 wiring on the Adafruit Feather ESP32 */
const uint8_t PIN_SCK  = 25;
const uint8_t PIN_MISO = 26;
const uint8_t PIN_MOSI = 27;
const uint8_t PIN_CS   = 14;
const uint8_t PIN_IRQ  = 13;
const uint8_t PIN_RST  = 32;

/* ===== Test role =====
   TAG  (initiator): drives the two-way ranging exchange and prints the distance.
   ANCHOR (responder): replies, computes the distance (DS-TWR), sends it back.
   Both boards must use the same channel/mode (same config test). */
#define ROLE_TAG    1
#define ROLE_ANCHOR 0
#define THIS_ROLE   ROLE_TAG   /* A: TAG, B: ANCHOR */

/* The DS-TWR ranging engine (message ids, callback type and dw1000_run_tag /
   dw1000_run_anchor) now lives in the DW1000 library: see my_dw1000.h. This
   file only picks the role and decides when to trigger an exchange. */

/* (radio configuration is handled by dw1000_config() in the DW1000 library) */

/* (tag/anchor DS-TWR implementation is now in the DW1000 library:
   dw1000_run_tag / dw1000_run_anchor - see my_dw1000.h) */

/* Example result callback used by the TAG loop below. */
static bool on_distance(float distance_m, bool got_reading)
{
    if (!got_reading) {
        ESP_LOGI("DW1000", "RANGE TIMEOUT (no data)");
        return false;
    }
    if (distance_m < 0.05f) {   /* a real reading, but essentially touching */
        ESP_LOGI("DW1000", "Too Close: %.3f m", (double)distance_m);
        return true;
    }
    ESP_LOGI("DW1000", "DISTANCE: %.2f m", (double)distance_m);
    return true;
}

static void dw1000_radio_task(void *arg)
{
    printf("==================================\n");
    printf("========= DEVICE A (radio) =======\n");
    printf("==================================\n");

    /* Init the SPI bus / driver and select the chip. */
    dw1000_init(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS, PIN_IRQ, PIN_RST);

    /* Probe the module. */
    if (!dw1000_probe()) {
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    /* Configure the radio (both boards must use the same channel/mode). */
    dw1000_config(0xDECA, 0x1001, DW1000_MODE_SHORTDATA_FAST_ACCURACY,
                  DW1000_CHANNEL_5, 16400);
    dw1000_set_peer_address(0x1002);   /* only range with the paired anchor */

    /* Run this board's role. */
    if (THIS_ROLE == ROLE_TAG) {
        printf("--- ROLE: TAG (DS-TWR initiator) ---\n");
        while (1) {
            dw1000_run_tag(PIN_IRQ, DW1000_RANGE_TIMEOUT_MS, on_distance);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    } else {
        dw1000_run_anchor(PIN_IRQ);
    }
}

void app_main(void)
{
    xTaskCreate(dw1000_radio_task, "dw1000_radio", 8192, NULL, 5, NULL);
}
