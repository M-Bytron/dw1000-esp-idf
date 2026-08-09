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

#ifdef __cplusplus
extern "C" {
#endif

/* ============================ constants ============================ */

/* Data transmission rate (DW1000DataRate) */
#define DW1000_RATE_110KBPS   0x00
#define DW1000_RATE_850KBPS   0x01
#define DW1000_RATE_6800KBPS  0x02

/* Pulse repetition frequency (DW1000PulseFrequency) */
#define DW1000_PRF_16MHZ      0x01
#define DW1000_PRF_64MHZ      0x02

/* Preamble length (DW1000PreambleLength) */
#define DW1000_PREAMBLE_LEN_64    0x01
#define DW1000_PREAMBLE_LEN_128   0x05
#define DW1000_PREAMBLE_LEN_256   0x09
#define DW1000_PREAMBLE_LEN_512   0x0D
#define DW1000_PREAMBLE_LEN_1024  0x02
#define DW1000_PREAMBLE_LEN_1536  0x06
#define DW1000_PREAMBLE_LEN_2048  0x0A
#define DW1000_PREAMBLE_LEN_4096  0x03

/* Operating channel (DW1000Channel) */
#define DW1000_CHANNEL_1  1
#define DW1000_CHANNEL_2  2
#define DW1000_CHANNEL_3  3
#define DW1000_CHANNEL_4  4
#define DW1000_CHANNEL_5  5
#define DW1000_CHANNEL_7  7

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

/*
 * Apply all pending RF settings. Equivalent to the driver's
 * newConfiguration() + commitConfiguration().
 */
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

/* ============================ diagnostics ============================ */

/* Read on-chip temperature (Celsius) and battery voltage (Volts). */
void dw1000_get_temp_and_vbat(float *temp_c, float *vbat_v);

#ifdef __cplusplus
}
#endif

#endif /* MY_DW1000_H */