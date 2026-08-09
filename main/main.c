/*
 * ESP32 DW1000 connectivity test - ESP-IDF (C)
 *
 * STEP 1 (this build): ONLY verify that the DW1000 module is connected and
 * reachable over SPI. The firmware initializes the SPI bus / driver and reads
 * the DW1000 device registers ("probe").
 *
 * The DW1000 driver itself is a C++ library (arduino-dw1000). It is exposed
 * to this C file through the plain C API in dw1000_c.h (see
 * components/arduino_dw1000).
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

#include "dw1000_c.h"

/* DW1000 wiring on the Adafruit Feather ESP32 */
const uint8_t PIN_SCK  = 25;
const uint8_t PIN_MISO = 26;
const uint8_t PIN_MOSI = 27;
const uint8_t PIN_CS   = 14;
const uint8_t PIN_IRQ  = 13;
const uint8_t PIN_RST  = 32;

static void probe(void)
{
    char msg[128];

    /* Device identifier register (0x00). A real DW1000 returns 0xDECA0130,
       so the printed string always starts with "DECA". */
    dw1000_get_device_id(msg);

    if (strncmp(msg, "DECA", 4) == 0) {
        printf("==============================================\n");
        printf(">>>  DW1000 MODULE DETECTED - SPI OK!  <<<\n");
        printf("==============================================\n");
        printf("Device ID  : %s\n", msg);

        dw1000_get_eui(msg);
        printf("Unique ID  : %s\n", msg);

        dw1000_get_net_addr(msg);
        printf("Net/Addr   : %s\n", msg);

        dw1000_get_device_mode(msg);
        printf("Device mode: %s\n", msg);
    } else {
        printf("**********************************************\n");
        printf(">>>  NO DW1000 RESPONSE - check wiring/power!\n");
        printf(">>>  Register read as: %s\n", msg);
        printf("**********************************************\n");
    }
}

static void dw1000_probe_task(void *arg)
{
    printf("==================================\n");
    printf("========= DEVICE A (probe) =======\n");
    printf("==================================\n");

    /* Assign the custom SPI pins (required on ESP32). */
    dw1000_set_spi_pins(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

    /* Initialize the driver and select/reset the chip. */
    dw1000_begin(PIN_IRQ, PIN_RST);
    dw1000_select(PIN_CS);

    printf("Driver initialized. Probing the DW1000 ...\n");
    probe();

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
