/*
 * DW1000 ESP-IDF example - Device A (TAG / ranging initiator)
 *
 * Demonstrates the DW1000 component library:
 *   1. init + probe (wiring check)
 *   2. radio configuration
 *   3. pairing with a specific peer (by its ESP32 BLE MAC)
 *   4. optional joint antenna-delay calibration (CALIBRATE_DISTANCE_CM)
 *   5. continuous DS-TWR ranging (prints the distance)
 *
 * The matching ANCHOR (responder) example is Examples/Device-B.
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
#define MY_SHORT_ADDR    0x1001   /* Device A */
uint16_t antenna_delay = 16464;   /* calibrated value - same on BOTH boards */

/* Joint antenna-delay calibration. Put the two modules exactly this many cm
   apart (clear line of sight), boot BOTH boards with the same value, and the
   tag calibrates both radios over the air. Set to 0 to skip calibration. */
#define CALIBRATE_DISTANCE_CM  0

/* ------------------------- pairing -------------------------
   Pair with the OTHER module using ITS ESP32 BLE MAC (6 bytes, MSB first,
   printed at boot as "My BLE MAC: ..."). Device A uses Device B's MAC. */
static const uint8_t peer_eui[6] = {0xD4, 0x8C, 0x49, 0xE3, 0xA4, 0x6E};

/* ------------------------- callback ------------------------ */
static bool on_distance(float distance_m, bool got_reading)
{
    if (!got_reading) {
        ESP_LOGI("DW1000", "RANGE TIMEOUT (no data)");
        return false;
    }
    ESP_LOGI("DW1000", "DISTANCE: %.2f m", (double)distance_m);
    return true;
}

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

    /* 1. init SPI bus + reset + LDE microcode load */
    dw1000_init(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS, PIN_IRQ, PIN_RST);

    /* 2. verify the module is reachable */
    if (!dw1000_probe()) {
        ESP_LOGE("DW1000", "Module not detected - check wiring/power!");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    /* 3. configure the radio (own EUI is derived from this board's BLE MAC) */
    dw1000_config(MY_PAN_ID, MY_SHORT_ADDR,
                  DW1000_MODE_SHORTDATA_FAST_ACCURACY,
                  DW1000_CHANNEL_5, antenna_delay);

    /* 4. pair with the anchor */
    dw1000_set_peer_eui(peer_eui);

    /* 5a. optional joint antenna-delay calibration */
    if (CALIBRATE_DISTANCE_CM > 0) {
        ESP_LOGI("DW1000", "Joint calibration: known distance %d cm",
                 (int)CALIBRATE_DISTANCE_CM);
        uint16_t ad = dw1000_calibrate_antenna_delay_iterative(
                          PIN_IRQ,
                          (float)CALIBRATE_DISTANCE_CM,
                          DW1000_CAL_CONVERGENCE_TICKS,
                          DW1000_CAL_MAX_ITERATIONS,
                          on_distance);
        ESP_LOGI("DW1000", "CALIBRATION DONE: antenna_delay = %u "
                           "(set this on BOTH boards)", (unsigned)ad);
    }

    /* 5b. continuous DS-TWR ranging */
    while (1) {
        dw1000_run_tag(PIN_IRQ, DW1000_RANGE_TIMEOUT_MS, on_distance);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    xTaskCreate(dw1000_radio_task, "dw1000_radio", 8192, NULL, 5, NULL);
}
