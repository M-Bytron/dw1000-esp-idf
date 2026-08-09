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

/* ================= interrupt configuration ================= */

void dw1000_c_interrupt_on_sent(int enabled);
void dw1000_c_interrupt_on_received(int enabled);
void dw1000_c_interrupt_on_receive_failed(int enabled);
void dw1000_c_interrupt_on_receive_timeout(int enabled);
void dw1000_c_interrupt_on_receive_timestamp_available(int enabled);

/* ================= status flags ================= */

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
