/*
 * my_dw1000.h - Pure-C DW1000 / DWM1000 UWB transceiver library.
 *
 * A small, reusable, C-only API for the Decawave DW1000. No classes, no
 * objects - every operation is a plain C function. The register-level work
 * is delegated to the arduino-dw1000 C++ driver through its C bridge
 * (dw1000_c.h) inside components/arduino_dw1000.
 *
 * Typical usage:
 *   1. dw1000_init(sck, miso, mosi, cs, irq, rst);
 *   2. dw1000_probe();                       // verify the module responds
 *   3. dw1000_enable_mode(DW1000_MODE_...);  // pick a radio mode
 *      dw1000_apply_config();
 *   4. dw1000_start_receive();  or  dw1000_send(...);
 *
 * To reuse this component in another project, copy the components/
 * arduino_dw1000, arduino_shim and DW1000 folders into that project and add
 * "DW1000" to your component's REQUIRES list.
 */
#ifndef MY_DW1000_H
#define MY_DW1000_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

/* Register map + encoding constants (DW1000_RATE_*, DW1000_PRF_*,
   DW1000_PREAMBLE_LEN_*, DW1000_CHANNEL_*, ...). */
#include "dw1000_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pre-defined operating modes. Each is a 3-byte array
 * { data_rate, pulse_frequency, preamble_length }.
 */
extern const uint8_t DW1000_MODE_LONGDATA_RANGE_LOWPOWER[3];
extern const uint8_t DW1000_MODE_SHORTDATA_FAST_LOWPOWER[3];
extern const uint8_t DW1000_MODE_LONGDATA_FAST_LOWPOWER[3];
extern const uint8_t DW1000_MODE_SHORTDATA_FAST_ACCURACY[3];
extern const uint8_t DW1000_MODE_LONGDATA_FAST_ACCURACY[3];
extern const uint8_t DW1000_MODE_LONGDATA_RANGE_ACCURACY[3];

/* Callback type for DW1000 events (executed from ISR context - keep short) */
typedef void (*dw1000_handler_t)(void);

/* ============================ init / probe ============================ */

/*
 * Initialise the SPI bus, reset and select the DW1000.
 * Call this once before any other function.
 */
void dw1000_init(uint8_t sck, uint8_t miso, uint8_t mosi,
                 uint8_t cs, uint8_t irq, uint8_t rst);

/*
 * Read the device identifier register and report whether the module
 * answered. Returns true if a DW1000 was detected.
 */
bool dw1000_probe(void);

/* Print device ID, EUI, network/address and mode to the ESP log. */
void dw1000_print_device_info(void);

/* Fill a caller-provided buffer (>= 128 bytes) with printable info. */
void dw1000_get_device_id(char *buf);   /* starts with "DECA" on a real DW1000 */
void dw1000_get_eui(char *buf);
void dw1000_get_net_addr(char *buf);
void dw1000_get_device_mode(char *buf);

/* ============================ addressing ============================ */

/* Set the 8-byte extended unique identifier, e.g. "AA:BB:CC:DD:EE:FF:00:11". */
void dw1000_set_eui(const char *eui);

/* Set the 16-bit network (PAN) identifier. */
void dw1000_set_network_id(uint16_t net_id);

/* Set the 16-bit short device address. */
void dw1000_set_device_address(uint16_t addr);

/* ============================ RF configuration ============================ */

void dw1000_set_channel(uint8_t channel);
void dw1000_set_data_rate(uint8_t rate);
void dw1000_set_pulse_frequency(uint8_t freq);
void dw1000_set_preamble_length(uint8_t len);
void dw1000_set_preamble_code(uint8_t code);
void dw1000_set_antenna_delay(uint16_t delay_us);
uint16_t dw1000_get_antenna_delay(void);

/* Select one of the DW1000_MODE_* presets (3-byte array). */
void dw1000_enable_mode(const uint8_t mode[3]);

/* Load the driver's standard defaults (frame filter off, channel 5, ...). */
void dw1000_set_defaults(void);

/*
 * Two-phase RF configuration. The dw1000_set_*() calls only update the
 * driver's cached configuration:
 *   1. dw1000_begin_config();   // idle + load current chip state
 *   2. dw1000_set_*();          // modify the caches
 *   3. dw1000_commit_config();  // write everything + re-tune the radio
 */
void dw1000_begin_config(void);
void dw1000_commit_config(void);

/* Convenience: begin_config() + commit_config(). */
void dw1000_apply_config(void);

/*
 * One-call radio configuration: loads the chip defaults, applies the given
 * settings and re-tunes the radio, then prints the resulting identity/mode
 * via ESP_LOGI so you can verify the write. Both boards must use the SAME
 * channel/mode for ranging.
 *
 *   mode          - one of the DW1000_MODE_* presets
 *                   ({ data_rate, pulse_frequency, preamble_length })
 *   antenna_delay - calibrated antenna delay in raw ticks (same on both boards)
 */
void dw1000_config(uint16_t network_id,
                   uint16_t device_address,
                   const uint8_t mode[3],
                   uint8_t channel,
                   uint16_t antenna_delay);

/* ============================ transceiver control ============================ */

/* Put the radio in idle state (no RX / TX active). */
void dw1000_idle(void);

/* Start a single reception. Read the frame with dw1000_get_data(). */
void dw1000_start_receive(void);

/* Auto-re-enable the receiver after each frame (polling mode). */
void dw1000_receive_permanently(bool enable);

/* Transmit a frame immediately. */
void dw1000_send(const uint8_t *data, uint16_t len);

/* Transmit a frame after a relative delay (us) - useful for ranging. */
void dw1000_send_at(const uint8_t *data, uint16_t len, uint32_t delay_us);

/* Transmit a frame at an absolute DW1000 timestamp (raw 40-bit ticks).
   Used for precise two-way-ranging replies: the peer replies at exactly
   rx_timestamp + reply_delay_ticks. */
void dw1000_send_at_ticks(const uint8_t *data, uint16_t len, uint64_t target_ticks);

/* ============================ data buffer ============================ */

/*
 * Copy the received frame into buf. Returns the number of bytes copied
 * (clamped to max_len).
 */
uint16_t dw1000_get_data(uint8_t *buf, uint16_t max_len);

/* ============================ timestamps ============================
 * Raw 40-bit DW1000 system counter values (one tick ~= 15.65 ps).
 * Convert to time-of-flight distance with dw1000_ticks_to_meters(). */

uint64_t dw1000_get_tx_timestamp(void);
uint64_t dw1000_get_rx_timestamp(void);
uint64_t dw1000_get_system_timestamp(void);

/* Convert a difference of raw timestamps into meters. */
float dw1000_ticks_to_meters(uint64_t tick_diff);

/* ============================ receive quality ============================ */

float dw1000_get_rx_power_dbm(void);        /* total receive power (dBm) */
float dw1000_get_first_path_power_dbm(void); /* first path power (dBm) */
float dw1000_get_rx_quality(void);          /* rx quality (dB) */

/* ============================ status flags ============================ */

bool dw1000_is_tx_done(void);
bool dw1000_is_rx_done(void);
bool dw1000_is_rx_failed(void);
bool dw1000_is_rx_timeout(void);

/* ============================ event callbacks ============================ */

void dw1000_on_error(dw1000_handler_t cb);
void dw1000_on_sent(dw1000_handler_t cb);
void dw1000_on_received(dw1000_handler_t cb);
void dw1000_on_receive_failed(dw1000_handler_t cb);
void dw1000_on_receive_timeout(dw1000_handler_t cb);
void dw1000_on_receive_timestamp_available(dw1000_handler_t cb);

/* Enable / disable which events raise the DW1000 IRQ pin (write the SYS_MASK
   cache; apply it with dw1000_commit_config()). */
void dw1000_interrupt_on_sent(bool enable);
void dw1000_interrupt_on_received(bool enable);
void dw1000_interrupt_on_receive_failed(bool enable);
void dw1000_interrupt_on_receive_timeout(bool enable);
void dw1000_interrupt_on_receive_timestamp_available(bool enable);

/* Configure the IRQ GPIO and start the interrupt dispatch task. Call once
   after the radio is configured. The handlers attached with dw1000_on_*()
   run in a high-priority task (not ISR context). */
esp_err_t dw1000_irq_start(uint8_t irq_gpio);

/* ============================ diagnostics ============================ */

/* Read on-chip temperature (Celsius) and battery voltage (Volts). */
void dw1000_get_temp_and_vbat(float *temp_c, float *vbat_v);

/* ============================ DS-TWR ranging ============================ */

/*
 * Asymmetric double-sided two-way ranging (DS-TWR) - a ready-to-use ranging
 * engine built on the transceiver primitives above (mirrors the classic
 * arduino-dw1000 ranging example).
 *
 * One exchange looks like this (~6 ms):
 *
 *     TAG (initiator)                    ANCHOR (responder)
 *      |  POLL (T1)  --------------------->|   T2 = poll RX time
 *      |  POLL_ACK (T3) <------------------|   reply after DW1000_REPLY_DELAY_US
 *      |  RANGE (T1, T4, T5) ------------->|   T6 = range RX time
 *      |                                   |   anchor computes the DS-TWR distance
 *      |  RANGE_REPORT (float meters) <----|   anchor prints it (ESP_LOGI)
 *      |  on_distance(distance) called     |
 *
 * All four round/reply times are MEASURED from the chip's 40-bit timestamps
 * and exchanged, so neither the reply delay nor the two boards' crystal
 * offset has to be assumed. Set an accurate antenna delay
 * (dw1000_set_antenna_delay) on BOTH boards for a correct absolute distance.
 * Both boards must use the SAME channel/mode configuration.
 *
 * Typical tag usage:
 *
 *   static bool on_distance(float meters) {
 *       if (meters < 0.0f) { printf("range timeout\n"); return false; }
 *       printf("distance: %.2f m\n", (double)meters);
 *       return true;
 *   }
 *   ...
 *   dw1000_init(sck, miso, mosi, cs, irq, rst);
 *   ... configure the radio (dw1000_commit_config) ...
 *   for (;;) { dw1000_run_tag(irq, DW1000_RANGE_TIMEOUT_MS, on_distance); }
 *
 * The anchor side simply calls dw1000_run_anchor(irq) once (never returns).
 */

/* Message identifiers (first byte of every ranging frame). */
#define DW1000_MSG_POLL         0   /* tag -> anchor: start the exchange       */
#define DW1000_MSG_POLL_ACK     1   /* anchor -> tag: reply to the poll        */
#define DW1000_MSG_RANGE        2   /* tag -> anchor: carries T1 / T4 / T5     */
#define DW1000_MSG_RANGE_REPORT 3   /* anchor -> tag: computed distance (float) */

#define DW1000_LEN_DATA         16     /* payload length of a ranging frame;
                                           the full frame adds a 21-byte IEEE
                                           802.15.4 extended-address header    */
#define DW1000_REPLY_DELAY_US   3000u  /* nominal reply delay (measured anyway) */
#define DW1000_RANGE_TIMEOUT_MS 1000   /* tag: max wait for a result, ms        */

/* Callback for the tag's ranging result.
   - got_reading == true  : distance_m is a real measured distance (meters).
   - got_reading == false : no distance arrived within the timeout
                            (distance_m is -1.0f / undefined).
   Return true to accept the reading, false to reject it. */
typedef bool (*dw1000_distance_cb_t)(float distance_m, bool got_reading);

/*
 * Pair this device with a specific peer using its 6-byte ESP32 BLE MAC
 * (peer_mac[0] is the most-significant byte), e.g. {0xD4, 0x8C, 0x49, 0xE3,
 * 0xA4, 0x6E}. The 6-byte MAC is padded to the 8-byte IEEE 802.15.4 extended
 * address used in the frame header and for the hardware frame filter.
 *
 * This device's own address is derived from ITS OWN ESP32 BLE MAC in
 * dw1000_config(). Calling this enables the DW1000 receive frame filter so
 * only frames destined to this device are accepted, and every ranging frame
 * is addressed to the peer - so the device only talks to its paired peer
 * (Option C: full IEEE 802.15.4 extended-address frames).
 */
void dw1000_set_peer_eui(const uint8_t peer_mac[6]);

/*
 * Run ONE ranging exchange as the tag (initiator): sends a POLL and blocks up
 * to timeout_ms waiting for the RANGE_REPORT, then calls on_distance() with the
 * result and returns the callback's return value. Single-shot - the caller
 * decides how often to call it. The radio runs interrupt-driven; only the
 * calling task blocks.
 *
 * on_distance() is called with got_reading = true and the measured meters on
 * success, or got_reading = false (distance_m = -1.0f) on timeout - so the
 * caller can tell "a real reading" apart from "no data".
 */
bool dw1000_run_tag(int irq_gpio, int timeout_ms, dw1000_distance_cb_t on_distance);

/*
 * Run forever as the anchor (responder): answers POLL with POLL_ACK and RANGE
 * with RANGE_REPORT, computing the DS-TWR distance and printing it via
 * ESP_LOGI. Does NOT return.
 */
void dw1000_run_anchor(int irq_gpio);

#ifdef __cplusplus
}
#endif

#endif /* MY_DW1000_H */