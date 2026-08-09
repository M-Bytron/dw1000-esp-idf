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
#include <string.h>
#include <stdint.h>

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

static void dw1000_probe_task(void *arg)
{
    printf("==================================\n");
    printf("========= DEVICE A (probe) =======\n");
    printf("==================================\n");

    /* Init the SPI bus / driver and select the chip. */
    dw1000_init(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS, PIN_IRQ, PIN_RST);

    /* Probe the module and print its identity. */
    if (dw1000_probe()) {
        dw1000_print_device_info();
    }

    /* Step 1: nothing else to do - keep the task alive. */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    /* The DW1000 IRQ handler runs in a high-priority task that is created
       lazily by the Arduino-compat shim on the first attachInterrupt() call
       inside dw1000_begin(). */
    xTaskCreate(dw1000_probe_task, "dw1000_probe", 8192, NULL, 5, NULL);
}
