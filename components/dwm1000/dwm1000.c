/*
 * DW1000 ranging application library - Device A (tag / initiator) - ESP-IDF (C)
 *
 * Ranging-only sender: runs the two-way ranging exchange with Device B
 * (POLL -> POLL_ACK -> RANGE -> RANGE_REPORT) and prints the distance.
 * Normal broadcast message sending is disabled.
 * Device B runs the matching responder firmware.
 *
 * All DW1000 application logic lives in this library (dwm1000.c / dwm1000.h);
 * main.c only calls dwm1000_start().
 *
 * The DW1000 driver is a C++ library (arduino-dw1000). It is exposed to this
 * C file through the plain C API in dw1000_c.h (see components/arduino_dw1000).
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
 *  - Antenna delay tuning over the monitor (non-blocking):
 *        '1' increase  '2' decrease  '0' reset  '9' show current delay
 *
 * Build / flash / monitor (ESP-IDF):
 *   idf.py set-target esp32
 *   idf.py -p COMx flash monitor
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"

#include "dwm1000.h"
#include "dw1000_c.h"

/* DW1000 wiring on the Adafruit Feather ESP32 */
const uint8_t PIN_SCK  = 25;
const uint8_t PIN_MISO = 26;
const uint8_t PIN_MOSI = 27;
const uint8_t PIN_CS   = 14;
const uint8_t PIN_IRQ  = 13;
const uint8_t PIN_RST  = 32;

/* ---- ranging protocol (Device A = tag / initiator) ---- */
#define POLL            0
#define POLL_ACK        1
#define RANGE           2
#define RANGE_REPORT    3
#define RANGE_FAILED    255
#define LEN_DATA        16
#define RANGE_INTERVAL_MS  500   /* show a new distance every 500 ms */

/* ------------------------------------------------------------------
 * Antenna delay calibration
 *   1) Place the two devices EXACTLY 2.00 m apart, clear line of sight.
 *   2) Flash this code onto BOTH devices.
 *   3) Monitor (115200): tune while watching "RANGE OK":
 *        '1' increase  '2' decrease  '0' reset  '9' show delay
 *      Tune BOTH devices until they read 2.00 m.
 *   4) Set ANTENNA_DELAY to the tuned value (same on both).
 * ------------------------------------------------------------------ */
#define ANTENNA_DELAY      16496   /* factory default; set to your tuned value */
#define ANTENNA_DELAY_STEP 16      /* ~7.5 cm per step (16 ticks x 0.0047 m/tick) */
static uint16_t antenna_delay = ANTENNA_DELAY;

/* ranging state */
static volatile bool sent_ack      = false;
static volatile bool received_ack  = false;
static bool ranging_busy           = false;   /* a ranging exchange is in flight */
static uint32_t ranging_timeout    = 0;
static uint32_t next_range_time    = 0;       /* next ranging exchange start (500 ms pacing) */
static uint8_t expected_msg_id     = POLL_ACK;
static bool protocol_failed        = false;
static uint8_t data[LEN_DATA];

/* raw 5-byte DW1000 timestamps */
static uint8_t time_poll_sent[5];
static uint8_t time_poll_ack_received[5];
static uint8_t time_range_sent[5];
static float last_range = -1.0f;

/* millisecond clock (Arduino millis() equivalent) */
static uint32_t millis_now(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

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

static void handle_sent(void)
{
    sent_ack = true;
}

static void handle_received(void)
{
    received_ack = true;
}

static void receiver(void)
{
    dw1000_new_receive();
    dw1000_set_defaults();
    dw1000_receive_permanently(1);   /* auto re-enable RX after each TX */
    dw1000_start_receive();
}

/* forward declaration (transmit_poll is defined below start_ranging) */
static void transmit_poll(void);

static void start_ranging(void)
{
    ranging_busy = true;
    expected_msg_id = POLL_ACK;
    protocol_failed = false;
    ranging_timeout = millis_now() + 1500;   /* 1.5 s to finish one exchange */
    transmit_poll();
}

static void transmit_poll(void)
{
    dw1000_new_transmit();
    dw1000_set_defaults();
    data[0] = POLL;
    dw1000_set_data(data, LEN_DATA);
    dw1000_start_transmit();
}

static void transmit_range(void)
{
    dw1000_new_transmit();
    dw1000_set_defaults();
    data[0] = RANGE;
    /* delay sending the frame 3000 us in the future and read back the
       expected TX timestamp (it is embedded in the payload for Device B) */
    dw1000_set_delay_us(3000, time_range_sent);
    memcpy(data + 1,  time_poll_sent, 5);
    memcpy(data + 6,  time_poll_ack_received, 5);
    memcpy(data + 11, time_range_sent, 5);
    dw1000_set_data(data, LEN_DATA);
    dw1000_start_transmit();
}

/* ---- antenna delay tuning over the console (non-blocking) ---- */
static void handle_serial_tuning(void)
{
    uint8_t c;
    int n = uart_read_bytes(UART_NUM_0, &c, 1, 0);
    if (n <= 0) {
        return;
    }

    printf("Key: %c\n", c);
    if (c == '1') {
        antenna_delay += ANTENNA_DELAY_STEP;
        dw1000_set_antenna_delay(antenna_delay);
        dw1000_commit_configuration();
        printf("Ant delay: %u\n", antenna_delay);
    } else if (c == '2') {
        if (antenna_delay > ANTENNA_DELAY_STEP) {
            antenna_delay -= ANTENNA_DELAY_STEP;
        }
        dw1000_set_antenna_delay(antenna_delay);
        dw1000_commit_configuration();
        printf("Ant delay: %u\n", antenna_delay);
    } else if (c == '0') {
        antenna_delay = ANTENNA_DELAY;
        dw1000_set_antenna_delay(antenna_delay);
        dw1000_commit_configuration();
        printf("Ant delay reset: %u\n", antenna_delay);
    } else if (c == '9') {
        printf("Ant delay: %u\n", antenna_delay);
    }
}

static void dw1000_ranging_task(void *arg)
{
    printf("==================================\n");
    printf("========= DEVICE A (ranging) =====\n");
    printf("==================================\n");

    /* Assign the custom SPI pins (required on ESP32). */
    dw1000_set_spi_pins(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

    /* Initialize the driver and select/reset the chip. */
    dw1000_begin(PIN_IRQ, PIN_RST);
    dw1000_select(PIN_CS);

    printf("Driver initialized. Probing the DW1000 ...\n");
    probe();

    /* ---- Basic TX setup: ranging frames ---- */
    dw1000_new_configuration();
    dw1000_set_defaults();
    dw1000_set_device_address(5);
    dw1000_set_network_id(10);
    dw1000_enable_mode(DW1000_TRX_RATE_110KBPS, DW1000_TX_PULSE_FREQ_16MHZ, DW1000_TX_PREAMBLE_LEN_2048);
    dw1000_set_antenna_delay(antenna_delay);   /* calibrated antenna delay */
    dw1000_commit_configuration();

    dw1000_attach_sent_handler(handle_sent);
    dw1000_attach_received_handler(handle_received);

    /* listen for ranging replies; radio stays in RX after every TX */
    receiver();
    printf("Ranging only - normal broadcast sending disabled.\n");

    while (1) {
        /* antenna delay tuning */
        handle_serial_tuning();

        /* handle radio events (ranging only) */
        if (sent_ack) {
            sent_ack = false;
            if (ranging_busy) {
                uint8_t msg_id = data[0];
                if (msg_id == POLL) {
                    dw1000_get_transmit_timestamp(time_poll_sent);
                } else if (msg_id == RANGE) {
                    dw1000_get_transmit_timestamp(time_range_sent);
                }
            }
        }

        if (received_ack) {
            received_ack = false;
            dw1000_get_data(data, LEN_DATA);
            uint8_t msg_id = data[0];
            if (ranging_busy && msg_id == POLL_ACK) {
                dw1000_get_receive_timestamp(time_poll_ack_received);
                expected_msg_id = RANGE_REPORT;
                transmit_range();
            } else if (ranging_busy && msg_id == RANGE_REPORT) {
                memcpy(&last_range, data + 1, 4);
                ranging_busy = false;
                printf("RANGE OK: %.2f m\n", last_range);
            } else if (ranging_busy && msg_id == RANGE_FAILED) {
                ranging_busy = false;
                printf("RANGE FAILED\n");
            }
        }

        /* abort a ranging exchange that never completed */
        if (ranging_busy && millis_now() > ranging_timeout) {
            ranging_busy = false;
            printf("RANGE TIMEOUT\n");
        }

        /* pacing: start one ranging exchange every 500 ms */
        if (!ranging_busy && millis_now() >= next_range_time) {
            next_range_time = millis_now() + RANGE_INTERVAL_MS;
            start_ranging();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void dwm1000_start(void)
{
    /* Route the console UART through the UART driver so the firmware can do
       non-blocking reads for the antenna delay tuning (standard ESP-IDF
       console pattern). printf() keeps working through the same driver. */
    uart_config_t uart_cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_cfg);
    uart_vfs_dev_use_driver(UART_NUM_0);

    /* The DW1000 IRQ handler runs in a high-priority task that is created
       lazily by the Arduino-compat shim on the first attachInterrupt() call
       inside dw1000_begin(). */
    xTaskCreate(dw1000_ranging_task, "dw1000_ranging", 8192, NULL, 5, NULL);
}
