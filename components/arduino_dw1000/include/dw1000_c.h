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

/* (ranging functions will be added here in a later step) */

#ifdef __cplusplus
}
#endif

#endif /* DW1000_C_H */
