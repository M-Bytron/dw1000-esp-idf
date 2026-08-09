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
   TAG  (initiator): drives the two-way ranging exchange and prints the distance.
   ANCHOR (responder): replies, computes the distance (DS-TWR), sends it back.
   Both boards must use the same channel/mode (same config test). */
#define ROLE_TAG    1
#define ROLE_ANCHOR 0
#define THIS_ROLE   ROLE_TAG   /* A: TAG, B: ANCHOR */

/* Asymmetric double-sided two-way ranging (mirrors arduino-dw1000).
   All four reply/round times are MEASURED and exchanged, so neither the
   reply delay nor the clock offset has to be assumed. */
#define MSG_POLL         0
#define MSG_POLL_ACK     1
#define MSG_RANGE        2
#define MSG_RANGE_REPORT 3
#define LEN_DATA         16
#define REPLY_DELAY_US   3000u   /* nominal delay; measured, not assumed */
#define RANGE_TIMEOUT_MS 1000    /* how long one exchange may take (ms) */

/* Callback for the tag's ranging result. Called with the measured distance
   in meters, or with a NEGATIVE value if no distance arrived within the
   timeout. Return true to accept / false to reject. */
typedef bool (*dw1000_distance_cb_t)(float distance_m);

/* m per raw timestamp tick (c * ~15.65 ps) */
#define DW1000_M_PER_TICK 0.0046917639786159f

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

/* ---- shared frame buffer + tag/anchor state ---- */

static uint8_t s_data[LEN_DATA];
static uint8_t s_last_sent_type;   /* tag: which frame was just transmitted */

static uint64_t s_t1;              /* tag: poll TX timestamp */
static uint64_t s_t2, s_t3;        /* anchor: poll RX, poll_ack TX timestamp */

/* single-shot exchange result (written by the IRQ task, read by run_tag) */
static volatile bool s_result_ready;   /* true when a RANGE_REPORT arrived */
static float          s_last_distance; /* last measured distance (m) */
static bool           s_tag_init_done; /* run_tag radio setup done once */

static void put_ts(uint8_t *buf, uint64_t ts)
{
    int i;
    for (i = 0; i < 5; i++) {
        buf[i] = (uint8_t)((ts >> (i * 8)) & 0xFF);
    }
}

static uint64_t get_ts(const uint8_t *buf)
{
    uint64_t ts = 0;
    int i;
    for (i = 0; i < 5; i++) {
        ts |= (uint64_t)buf[i] << (i * 8);
    }
    return ts;
}

/* Time difference with 40-bit rollover handling (mirrors DW1000Time::wrap).
   The system counter wraps every 2^40 ticks (~17 s); if a difference goes
   negative because later/earlier straddle the rollover, add 2^40 back. */
static int64_t diff_ts(uint64_t later, uint64_t earlier)
{
    int64_t d = (int64_t)(later - earlier);
    if (d < 0) {
        d += (int64_t)0x10000000000LL;   /* 2^40 */
    }
    return d;
}

/* ---- TAG (initiator) ---- */

static void tag_on_sent(void)
{
    if (s_last_sent_type == MSG_POLL) {
        s_t1 = dw1000_get_tx_timestamp();
    }
}

static void tag_on_received(void)
{
    uint16_t len = dw1000_get_data(s_data, sizeof(s_data));
    if (len < 1) {
        return;
    }

    if (s_data[0] == MSG_POLL_ACK) {
        /* anchor replied: send RANGE with T1 (poll), T4 (ack rx), T5 (range) */
        uint64_t t4 = dw1000_get_rx_timestamp();
        uint64_t now = dw1000_get_system_timestamp();
        uint64_t target = now + (uint64_t)((float)REPLY_DELAY_US * 63897.6f);
        /* + antenna delay: TX times must be antenna-referenced (like setDelay/TX_TIME) */
        uint64_t t5 = (target & ~0x1FFULL) + (uint64_t)dw1000_get_antenna_delay();

        s_data[0] = MSG_RANGE;
        put_ts(s_data + 1,  s_t1);
        put_ts(s_data + 6,  t4);
        put_ts(s_data + 11, t5);
        s_last_sent_type = MSG_RANGE;
        dw1000_send_at_ticks(s_data, LEN_DATA, target);
    } else if (s_data[0] == MSG_RANGE_REPORT) {
        memcpy(&s_last_distance, s_data + 1, sizeof(s_last_distance));
        s_result_ready = true;
        dw1000_start_receive();   /* re-arm for the next exchange */
    }
}

/*
 * Send ONE ranging exchange (POLL -> POLL_ACK -> RANGE -> RANGE_REPORT) and
 * wait up to timeout_ms for the distance. Single-shot: the caller decides how
 * often to call it. The radio runs interrupt-driven; this only blocks the
 * calling task.
 *
 * Returns the callback's value: on success the callback receives the measured
 * distance in meters, on timeout a NEGATIVE value so it can return false.
 */
bool run_tag(int timeout_ms, dw1000_distance_cb_t on_distance)
{
    bool ok;

    if (!s_tag_init_done) {
        s_tag_init_done = true;
        dw1000_irq_start(PIN_IRQ);
        dw1000_receive_permanently(true);
        dw1000_on_sent(tag_on_sent);
        dw1000_on_received(tag_on_received);
    }

    /* start one exchange */
    s_result_ready = false;
    memset(s_data, 0, sizeof(s_data));
    s_data[0] = MSG_POLL;
    s_last_sent_type = MSG_POLL;
    dw1000_send(s_data, LEN_DATA);

    /* wait for the result or the timeout */
    TickType_t t0 = xTaskGetTickCount();
    while (!s_result_ready &&
           (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(timeout_ms)) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ok = s_result_ready;
    if (on_distance != NULL) {
        return on_distance(ok ? s_last_distance : -1.0f);
    }
    return ok;
}

/* ---- ANCHOR (responder): replies, computes DS-TWR, sends the range ---- */

static void anchor_on_received(void)
{
    uint16_t len = dw1000_get_data(s_data, sizeof(s_data));
    if (len < 1) {
        return;
    }

    if (s_data[0] == MSG_POLL) {
        /* reply after REPLY_DELAY_US; the actual reply time is measured */
        s_t2 = dw1000_get_rx_timestamp();
        uint64_t now = dw1000_get_system_timestamp();
        uint64_t target = now + (uint64_t)((float)REPLY_DELAY_US * 63897.6f);
        /* + antenna delay: the TX timestamp must be antenna-referenced (like TX_TIME) */
        s_t3 = (target & ~0x1FFULL) + (uint64_t)dw1000_get_antenna_delay();

        s_data[0] = MSG_POLL_ACK;
        dw1000_send_at_ticks(s_data, LEN_DATA, target);
    } else if (s_data[0] == MSG_RANGE) {
        /* we have T2/T3; RANGE carries T1/T4/T5; measure T6 now */
        uint64_t t1 = get_ts(s_data + 1);
        uint64_t t4 = get_ts(s_data + 6);
        uint64_t t5 = get_ts(s_data + 11);
        uint64_t t6 = dw1000_get_rx_timestamp();

        /* asymmetric DS-TWR (arduino-dw1000 computeRangeAsymmetric) */
        int64_t round1 = diff_ts(t4, t1);      /* tag:   poll -> ack rx */
        int64_t reply1 = diff_ts(s_t3, s_t2);  /* anchor: poll rx -> ack tx */
        int64_t round2 = diff_ts(t6, s_t3);    /* anchor: ack tx -> range rx */
        int64_t reply2 = diff_ts(t5, t4);      /* tag:   ack rx -> range tx */
        int64_t num = round1 * round2 - reply1 * reply2;
        int64_t den = round1 + round2 + reply1 + reply2;
        int64_t tof = (den != 0) ? (num / den) : 0;
        float range = (float)tof * DW1000_M_PER_TICK;

        printf("RANGE OK: %.2f m\n", (double)range);

        /* send the computed range back to the tag */
        s_data[0] = MSG_RANGE_REPORT;
        memcpy(s_data + 1, &range, sizeof(range));
        dw1000_send(s_data, LEN_DATA);
    }
}

static void run_anchor(void)
{
    printf("--- ROLE: ANCHOR (DS-TWR responder) ---\n");
    dw1000_irq_start(PIN_IRQ);
    dw1000_receive_permanently(true);
    dw1000_on_received(anchor_on_received);
    dw1000_start_receive();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* Example result callback used by the TAG loop below. */
static bool on_distance(float distance_m)
{
    if (distance_m < 0.0f) {
        ESP_LOGI("DW1000", "Too Close");
        return false;
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

    /* Configure the radio (both boards use the same config). */
    dw1000_config_test();

    /* Run this board's role. */
    if (THIS_ROLE == ROLE_TAG) {
        printf("--- ROLE: TAG (DS-TWR initiator) ---\n");
        while (1) {
            run_tag(RANGE_TIMEOUT_MS, on_distance);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    } else {
        run_anchor();
    }
}

void app_main(void)
{
    xTaskCreate(dw1000_radio_task, "dw1000_radio", 8192, NULL, 5, NULL);
}
