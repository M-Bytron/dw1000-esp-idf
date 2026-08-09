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

#ifdef __cplusplus
}
#endif

#endif /* MY_DW1000_H */