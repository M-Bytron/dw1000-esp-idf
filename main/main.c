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

/* ===== Test role =====
   Set THIS_ROLE to ROLE_SENDER on one board and ROLE_RECEIVER on the other.
   Both boards must use the same channel/mode (they run the same config test). */
#define ROLE_SENDER   1
#define ROLE_RECEIVER 0
#define THIS_ROLE     ROLE_SENDER   /* <-- change to ROLE_RECEIVER on the 2nd board */

/* A tiny command frame sent between the two boards. */
typedef struct {
    uint8_t cmd;    /* command id */
    uint8_t seq;    /* sequence number */
    uint8_t data;   /* one data byte */
} uwb_cmd_t;

/* Configure the radio with the new pure-C config API and print the result.
   The order matters: set the mode (which sets the pulse frequency) BEFORE
   set_channel, because set_channel picks the preamble code from the PRF. */
static void dw1000_config_test(void)
{
    printf("\n--- Config test (pure C) ---\n");

    /* 1. idle + load the current chip state into the driver caches. */
    dw1000_begin_config();
    dw1000_set_defaults();   /* standard defaults (also enables the IRQ events) */

    /* 2. customise the caches. */
    dw1000_set_network_id(0xDECA);
    dw1000_set_device_address(0x1001);
    dw1000_enable_mode(DW1000_MODE_SHORTDATA_FAST_ACCURACY); /* 6.8 Mb/s, 64 MHz, 128-sym */
    dw1000_set_channel(DW1000_CHANNEL_5);
    dw1000_set_antenna_delay(16400);   /* calibrate for your board later */

    /* 3. write everything to the chip + re-tune the radio. */
    dw1000_commit_config();

    /* Print identity + mode. "Net/Addr" is read back from the chip, so it
       verifies the commit actually wrote; "Device mode" shows the driver state. */
    dw1000_print_device_info();

    printf("--- Config test done ---\n");
}

/* Sender: broadcast a short command frame every second. */
static void run_sender(void)
{
    uwb_cmd_t cmd;
    uint8_t seq = 0;

    printf("--- ROLE: SENDER ---\n");
    while (1) {
        cmd.cmd  = 0x01;                       /* CMD_POLL */
        cmd.seq  = seq++;
        cmd.data = (uint8_t)(seq * 3);

        dw1000_send((const uint8_t *)&cmd, sizeof(cmd));

        /* wait until the frame has actually left the chip (10 ms polls so the
           sender never tight-loops and trips the task watchdog) */
        for (int i = 0; i < 200 && !dw1000_is_tx_done(); i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        printf("TX: cmd=%u seq=%u data=%u | tx_ts=%llu\n",
               (unsigned)cmd.cmd, (unsigned)cmd.seq, (unsigned)cmd.data,
               (unsigned long long)dw1000_get_tx_timestamp());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* Called from the DW1000 IRQ dispatch task when a frame is received. */
static void on_frame_received(void)
{
    uwb_cmd_t cmd;
    uint16_t len = dw1000_get_data((uint8_t *)&cmd, sizeof(cmd));
    printf("RX (%u): cmd=%u seq=%u data=%u | rx_ts=%llu pwr=%.1f dBm fp=%.1f dBm\n",
           (unsigned)len,
           (unsigned)cmd.cmd, (unsigned)cmd.seq, (unsigned)cmd.data,
           (unsigned long long)dw1000_get_rx_timestamp(),
           (double)dw1000_get_rx_power_dbm(),
           (double)dw1000_get_first_path_power_dbm());
}

/* Receiver: interrupt-driven - no polling loop, so no watchdog pressure. */
static void run_receiver(void)
{
    printf("--- ROLE: RECEIVER (interrupt-driven) ---\n");
    dw1000_irq_start(PIN_IRQ);
    dw1000_receive_permanently(true);
    dw1000_on_received(on_frame_received);
    dw1000_start_receive();

    /* everything happens in on_frame_received(); the driver re-arms RX */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
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

    /* Configure the radio (both boards use the same config). */
    dw1000_config_test();

    /* Run this board's role. */
    if (THIS_ROLE == ROLE_SENDER) {
        run_sender();
    } else {
        run_receiver();
    }
}

void app_main(void)
{
    xTaskCreate(dw1000_radio_task, "dw1000_radio", 8192, NULL, 5, NULL);
}
