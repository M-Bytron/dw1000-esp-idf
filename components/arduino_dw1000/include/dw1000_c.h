/*
 * Plain C API for the arduino-dw1000 C++ driver.
 *
 * The underlying DW1000 driver is a C++ library (DW1000Class / DW1000Time),
 * which cannot be called from a C file directly. This header + the matching
 * dw1000_c.cpp wrapper expose the subset needed by the application as
 * ordinary C functions, so the application can be written in C (main.c).
 */
#ifndef DW1000_C_H
#define DW1000_C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- init ---- */
void dw1000_set_spi_pins(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t ss);
void dw1000_begin(uint8_t irq, uint8_t rst);
void dw1000_select(uint8_t ss);

/* ---- device info: fill a caller-provided buffer (>= 128 bytes) ---- */
void dw1000_get_device_id(char *buf);   /* starts with "DECA" on a real DW1000 */
void dw1000_get_eui(char *buf);
void dw1000_get_net_addr(char *buf);
void dw1000_get_device_mode(char *buf);

/* ================= low-level configuration ================= */

/* Device address / filtering */
void dw1000_c_set_network_id(uint16_t val);
void dw1000_c_set_device_address(uint16_t val);
void dw1000_c_set_eui(const char *eui);   /* "AA:BB:CC:DD:EE:FF:00:11" */

/* RF settings (applied by commitConfiguration) */
void dw1000_c_set_data_rate(uint8_t rate);
void dw1000_c_set_pulse_frequency(uint8_t freq);
void dw1000_c_set_preamble_length(uint8_t len);
void dw1000_c_set_preamble_code(uint8_t code);
void dw1000_c_set_channel(uint8_t channel);
void dw1000_c_use_smart_power(int enabled);
void dw1000_c_set_antenna_delay(uint16_t delay);
uint16_t dw1000_c_get_antenna_delay(void);

/* Pre-defined mode (3 bytes: data rate, pulse freq, preamble length) */
void dw1000_c_enable_mode(const uint8_t mode[3]);
void dw1000_c_set_defaults(void);

/* ================= transceiver control ================= */

void dw1000_c_idle(void);
void dw1000_c_new_configuration(void);
void dw1000_c_commit_configuration(void);
void dw1000_c_new_receive(void);
void dw1000_c_start_receive(void);
void dw1000_c_new_transmit(void);
void dw1000_c_start_transmit(void);
void dw1000_c_receive_permanently(int enabled);

/* Relative delay (in us) for the next delayed TX / RX */
void dw1000_c_set_delay(uint64_t delay_us);

/* ================= data buffer ================= */

void dw1000_c_set_data(const uint8_t *data, uint16_t n);
void dw1000_c_get_data(uint8_t *data, uint16_t n);
uint16_t dw1000_c_get_data_length(void);

/* ================= timestamps =================
 * Raw 40-bit DW1000 system counter values. One tick is ~15.65 ps. */

uint64_t dw1000_c_get_transmit_timestamp(void);
uint64_t dw1000_c_get_receive_timestamp(void);
uint64_t dw1000_c_get_system_timestamp(void);

/* ================= receive quality ================= */

float dw1000_c_get_receive_power(void);      /* dBm */
float dw1000_c_get_first_path_power(void);   /* dBm */
float dw1000_c_get_receive_quality(void);    /* dB */

/* ================= interrupt configuration ================= */

void dw1000_c_interrupt_on_sent(int enabled);
void dw1000_c_interrupt_on_received(int enabled);
void dw1000_c_interrupt_on_receive_failed(int enabled);
void dw1000_c_interrupt_on_receive_timeout(int enabled);
void dw1000_c_interrupt_on_receive_timestamp_available(int enabled);

/* ================= status flags ================= */

int dw1000_c_is_transmit_done(void);
int dw1000_c_is_receive_done(void);
int dw1000_c_is_receive_failed(void);
int dw1000_c_is_receive_timeout(void);
int dw1000_c_is_receive_timestamp_available(void);

/* ================= callbacks (called from ISR context) ================= */

void dw1000_c_attach_error_handler(void (*cb)(void));
void dw1000_c_attach_sent_handler(void (*cb)(void));
void dw1000_c_attach_received_handler(void (*cb)(void));
void dw1000_c_attach_receive_failed_handler(void (*cb)(void));
void dw1000_c_attach_receive_timeout_handler(void (*cb)(void));
void dw1000_c_attach_receive_timestamp_available_handler(void (*cb)(void));

/* ================= misc / diagnostics ================= */

void dw1000_c_get_temp_and_vbat(float *temp, float *vbat);

#ifdef __cplusplus
}
#endif

#endif /* DW1000_C_H */
