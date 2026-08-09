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

/* ---- general configuration ---- */
void dw1000_new_configuration(void);
void dw1000_set_defaults(void);
void dw1000_set_device_address(uint16_t addr);
void dw1000_set_network_id(uint16_t id);
void dw1000_enable_mode(uint8_t data_rate, uint8_t pulse_freq, uint8_t preamble_len);
void dw1000_set_antenna_delay(uint16_t value);
void dw1000_commit_configuration(void);

/* predefined mode bytes for dw1000_enable_mode() (mirror DW1000Class) */
#define DW1000_TRX_RATE_110KBPS      0x00
#define DW1000_TRX_RATE_850KBPS      0x01
#define DW1000_TRX_RATE_6800KBPS     0x02
#define DW1000_TX_PULSE_FREQ_16MHZ   0x01
#define DW1000_TX_PULSE_FREQ_64MHZ   0x02
#define DW1000_TX_PREAMBLE_LEN_2048  0x0A

/* ---- event handlers (attach plain C callbacks) ---- */
void dw1000_attach_error_handler(void (*handler)(void));
void dw1000_attach_sent_handler(void (*handler)(void));
void dw1000_attach_received_handler(void (*handler)(void));
void dw1000_attach_receive_failed_handler(void (*handler)(void));

/* ---- reception ---- */
void dw1000_new_receive(void);
void dw1000_receive_permanently(int val);
void dw1000_start_receive(void);

/* ---- transmission ---- */
void dw1000_new_transmit(void);
void dw1000_set_data(uint8_t *data, uint16_t n);
void dw1000_get_data(uint8_t *data, uint16_t n);
/* Schedule the next transmit `us` microseconds in the future. If expected_ts
   is non-NULL it receives the (5-byte) expected TX timestamp that the DW1000
   reports when the transmit is scheduled. */
void dw1000_set_delay_us(uint32_t us, uint8_t expected_ts[5]);
void dw1000_start_transmit(void);

/* ---- timestamps (raw 5-byte / 40-bit DW1000 timestamps) ---- */
void dw1000_get_transmit_timestamp(uint8_t ts[5]);
void dw1000_get_receive_timestamp(uint8_t ts[5]);

/* ---- asymmetric two-way ranging ----
   Computes the time-of-flight distance (in meters) from the six raw
   timestamps captured during one POLL/POLL_ACK/RANGE exchange:
     poll_sent         TX timestamp of the POLL      (initiator / tag)
     poll_received     RX timestamp of the POLL      (responder / anchor)
     poll_ack_sent     TX timestamp of the POLL_ACK  (responder / anchor)
     poll_ack_received RX timestamp of the POLL_ACK  (initiator / tag)
     range_sent        TX timestamp of the RANGE     (initiator / tag)
     range_received    RX timestamp of the RANGE     (responder / anchor) */
float dw1000_ranging_compute_asymmetric(const uint8_t poll_sent[5],
                                        const uint8_t poll_received[5],
                                        const uint8_t poll_ack_sent[5],
                                        const uint8_t poll_ack_received[5],
                                        const uint8_t range_sent[5],
                                        const uint8_t range_received[5]);

#ifdef __cplusplus
}
#endif

#endif /* DW1000_C_H */
