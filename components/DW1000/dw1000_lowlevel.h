/*
 * dw1000_lowlevel.h - Pure-C low-level DW1000 driver.
 *
 * SPI transport + reset + clock selection, implemented directly on top of
 * the ESP-IDF SPI master driver. No Arduino, no C++, no external library.
 *
 * This is the foundation the rest of the pure-C port builds on. Register
 * constants live in dw1000_regs.h.
 */
#ifndef DW1000_LOWLEVEL_H
#define DW1000_LOWLEVEL_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Initialise the SPI master bus + a device for the DW1000 (mode 0, MSB first,
 * starts at 2 MHz). Call once before any register access.
 * Returns ESP_OK on success (also OK if already initialised).
 */
esp_err_t dw1000_ll_spi_init(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs);

/*
 * Read `n` bytes from a register. Use DW1000_NO_SUB (from dw1000_regs.h) as
 * `sub` to access the whole register without sub-addressing.
 *
 * Returns ESP_OK on success, or the SPI/driver error on failure. ON FAILURE
 * `data` IS ZERO-FILLED, so a failed read is indistinguishable from a real
 * all-zero register unless you check the return value. Every failure is also
 * logged (first few only) and counted - see dw1000_ll_diag_dump().
 */
esp_err_t dw1000_ll_read(uint8_t reg, uint16_t sub, uint8_t *data, uint16_t n);

/* Write `n` bytes to a register (see dw1000_ll_read for `sub`).
   Returns ESP_OK on success, or the SPI/driver error on failure. */
esp_err_t dw1000_ll_write(uint8_t reg, uint16_t sub, const uint8_t *data, uint16_t n);

/*
 * Hard reset via the RSTn pin (active low). The pin is pulsed low and then
 * left floating, as required by the DW1000 data sheet (5.6.1).
 * Pass 0xFF if no reset line is wired.
 */
void dw1000_ll_reset(uint8_t rst);

/* Select the clock source: DW1000_AUTO_CLOCK / DW1000_XTI_CLOCK / DW1000_PLL_CLOCK. */
void dw1000_ll_clock(uint8_t clock);

/* Read the raw 32-bit device identifier (0xDECA0130 on a real DW1000). */
uint32_t dw1000_ll_read_device_id(void);

/* ============================ device addressing ============================ */

void dw1000_ll_set_network_id(uint16_t val);
void dw1000_ll_set_device_address(uint16_t val);
void dw1000_ll_set_eui(const char *eui);   /* "AA:BB:CC:DD:EE:FF:00:11" */

/* Read the 8-byte EUI as raw register bytes (least-significant byte first,
   i.e. the order used on the air / in the IEEE 802.15.4 header). */
void dw1000_ll_get_eui_bytes(uint8_t eui[8]);

/* ============================ RF configuration ============================
 * The setters below only update the driver's cached configuration. Call
 * dw1000_ll_new_configuration() first (loads the current chip state), then
 * the setters, then dw1000_ll_commit_configuration() to write everything
 * and re-tune the radio. */

void dw1000_ll_set_data_rate(uint8_t rate);
void dw1000_ll_set_pulse_frequency(uint8_t freq);
void dw1000_ll_set_preamble_length(uint8_t len);
void dw1000_ll_set_preamble_code(uint8_t code);
void dw1000_ll_set_channel(uint8_t channel);
void dw1000_ll_enable_mode(const uint8_t mode[3]);
void dw1000_ll_set_defaults(void);
void dw1000_ll_use_smart_power(int enabled);

/* Enable/disable the hardware receive frame filter (writes SYS_CFG now).
   allow_extended = accept frames destined to MY extended (EUI) address.
   allow_broadcast = accept broadcast/multicast frames.
   (both 0 = filter disabled). */
void dw1000_ll_apply_frame_filter(int allow_extended, int allow_broadcast);

void dw1000_ll_set_antenna_delay(uint16_t delay);
uint16_t dw1000_ll_get_antenna_delay(void);

/* ============================ configuration lifecycle ============================ */

void dw1000_ll_idle(void);
void dw1000_ll_new_configuration(void);
void dw1000_ll_commit_configuration(void);
void dw1000_ll_receive_permanently(int enabled);

/* ============================ current mode (diagnostics) ============================ */

uint8_t dw1000_ll_get_channel(void);
uint8_t dw1000_ll_get_data_rate(void);
uint8_t dw1000_ll_get_pulse_frequency(void);
uint8_t dw1000_ll_get_preamble_length(void);
uint8_t dw1000_ll_get_preamble_code(void);

/* ============================ power-up defaults + LDE ============================
 * After reset the LDE microcode must be loaded from OTP before the receiver
 * can be used. Call both once during init, after dw1000_ll_reset(). */

void dw1000_ll_init_defaults(void);   /* PAN/addr 0xFFFF, disable double RX buffer,
                                         active-high IRQ, no interrupts enabled */
void dw1000_ll_manage_lde(void);      /* load LDE microcode from OTP */

/* ============================ RX / TX ============================
 * RX: dw1000_ll_new_receive() + dw1000_ll_start_receive(), then poll the
 *     dw1000_ll_is_receive_*() flags; read the frame with dw1000_ll_get_data().
 * TX: dw1000_ll_set_data() + dw1000_ll_new_transmit() + dw1000_ll_start_transmit(). */

void dw1000_ll_new_receive(void);
void dw1000_ll_start_receive(void);
void dw1000_ll_new_transmit(void);
void dw1000_ll_start_transmit(void);

void dw1000_ll_set_data(const uint8_t *data, uint16_t n);
void dw1000_ll_get_data(uint8_t *data, uint16_t n);
uint16_t dw1000_ll_get_data_length(void);   /* payload length (FCS excluded) */

bool dw1000_ll_is_transmit_done(void);
bool dw1000_ll_is_receive_done(void);
bool dw1000_ll_is_receive_failed(void);
bool dw1000_ll_is_receive_timeout(void);

/* ============================ timestamps ============================
 * Raw 40-bit DW1000 system counter values (one tick ~= 15.65 ps). */

uint64_t dw1000_ll_get_transmit_timestamp(void);
uint64_t dw1000_ll_get_receive_timestamp(void);
uint64_t dw1000_ll_get_system_timestamp(void);

/* ============================ receive quality ============================ */

float dw1000_ll_get_receive_power(void);      /* dBm */
float dw1000_ll_get_first_path_power(void);   /* dBm */
float dw1000_ll_get_receive_quality(void);    /* dB */

/* ============================ delayed TX ============================ */

/* Arm a relative delay (us) for the next transmit. Call between
   dw1000_ll_new_transmit() and dw1000_ll_start_transmit(). */
void dw1000_ll_set_delay(uint64_t delay_us);

/* Start a transmit at an absolute DW1000 timestamp (raw 40-bit ticks).
   Call between dw1000_ll_new_transmit() and this function - used for
   precise two-way-ranging replies. */
void dw1000_ll_start_transmit_at(uint64_t target_ticks);

/* ============================ interrupts / callbacks ============================
 * The DW1000 IRQ pin goes high when an enabled event occurs. Start the IRQ
 * service with dw1000_ll_irq_start(), attach a handler with dw1000_ll_attach_*,
 * and enable the events with dw1000_ll_interrupt_on_*. Handlers run in a
 * high-priority FreeRTOS task (not in ISR context), so they may do SPI I/O. */

typedef void (*dw1000_ll_handler_t)(void);

void dw1000_ll_attach_error_handler(dw1000_ll_handler_t cb);
void dw1000_ll_attach_sent_handler(dw1000_ll_handler_t cb);
void dw1000_ll_attach_received_handler(dw1000_ll_handler_t cb);
void dw1000_ll_attach_receive_failed_handler(dw1000_ll_handler_t cb);
void dw1000_ll_attach_receive_timeout_handler(dw1000_ll_handler_t cb);
void dw1000_ll_attach_receive_timestamp_available_handler(dw1000_ll_handler_t cb);

/* Enable / disable which events raise the IRQ pin (writes the SYS_MASK cache;
   apply it by calling dw1000_ll_commit_configuration() afterwards). */
void dw1000_ll_interrupt_on_sent(int enabled);
void dw1000_ll_interrupt_on_received(int enabled);
void dw1000_ll_interrupt_on_receive_failed(int enabled);
void dw1000_ll_interrupt_on_receive_timeout(int enabled);
void dw1000_ll_interrupt_on_receive_timestamp_available(int enabled);

/* Configure the IRQ GPIO and start the dispatch task. Call once after the
   radio is configured. Returns ESP_OK on success. */
esp_err_t dw1000_ll_irq_start(uint8_t irq_gpio);

/* Radio sequence lock, shared between the IRQ task and the application TX
   task. Take it around any group of register accesses that must not be
   interleaved with another context's. Never hold it while waiting for TXFRS. */
void dw1000_ll_radio_lock(void);
void dw1000_ll_radio_unlock(void);

/* TX-done handshake: the IRQ task signals TXFRS; the TX task clears before a
   transmit and waits for it afterwards. */
void dw1000_ll_tx_done_clear(void);
bool dw1000_ll_tx_done_wait(uint32_t timeout_ms);

/* Put the radio back into receive mode without touching SYS_STATUS (the IRQ
   handler already cleared exactly the events it read). */
void dw1000_ll_rearm_receive(void);

/* Read SYS_STATUS and dispatch to the attached handlers (clears the events).
   Called internally by the IRQ task; exposed for completeness. */
void dw1000_ll_handle_interrupt(void);

/* ============================ diagnostics ============================ */

/* On-chip temperature (Celsius) and battery voltage (Volts). */
void dw1000_ll_get_temp_and_vbat(float *temp, float *vbat);

/* Print the interrupt/race counters (see dw1000_lowlevel.c). Safe to call from
   any task; it is reported periodically by the driver's low-priority diag task. */
void dw1000_ll_diag_dump(void);

#ifdef __cplusplus
}
#endif

#endif /* DW1000_LOWLEVEL_H */
