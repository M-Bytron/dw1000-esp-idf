/*
 * my_dw1000.c - Pure-C DW1000 / DWM1000 UWB transceiver library.
 *
 * Plain C implementation. All register-level work is delegated to the
 * arduino-dw1000 C++ driver through the C bridge in dw1000_c.h.
 */
#include "my_dw1000.h"
#include "dw1000_c.h"

#include <string.h>

#include "esp_log.h"

static const char *TAG = "DW1000";

/* ~15.65 ps per raw timestamp tick -> meters per tick */
#define DW1000_METERS_PER_TICK 0.0046917639786159f

/* ============================ mode presets ============================ */

const uint8_t DW1000_MODE_LONGDATA_RANGE_LOWPOWER[3] =
    {DW1000_RATE_110KBPS, DW1000_PRF_16MHZ, DW1000_PREAMBLE_LEN_2048};
const uint8_t DW1000_MODE_SHORTDATA_FAST_LOWPOWER[3] =
    {DW1000_RATE_6800KBPS, DW1000_PRF_16MHZ, DW1000_PREAMBLE_LEN_128};
const uint8_t DW1000_MODE_LONGDATA_FAST_LOWPOWER[3] =
    {DW1000_RATE_6800KBPS, DW1000_PRF_16MHZ, DW1000_PREAMBLE_LEN_1024};
const uint8_t DW1000_MODE_SHORTDATA_FAST_ACCURACY[3] =
    {DW1000_RATE_6800KBPS, DW1000_PRF_64MHZ, DW1000_PREAMBLE_LEN_128};
const uint8_t DW1000_MODE_LONGDATA_FAST_ACCURACY[3] =
    {DW1000_RATE_6800KBPS, DW1000_PRF_64MHZ, DW1000_PREAMBLE_LEN_1024};
const uint8_t DW1000_MODE_LONGDATA_RANGE_ACCURACY[3] =
    {DW1000_RATE_110KBPS, DW1000_PRF_64MHZ, DW1000_PREAMBLE_LEN_2048};

/* ============================ init / probe ============================ */

void dw1000_init(uint8_t sck, uint8_t miso, uint8_t mosi,
                 uint8_t cs, uint8_t irq, uint8_t rst)
{
    /* Assign the custom SPI pins (required on ESP32). */
    dw1000_set_spi_pins(sck, miso, mosi, cs);

    /* Initialize the driver and reset/select the chip. */
    dw1000_begin(irq, rst);
    dw1000_select(cs);

    ESP_LOGI(TAG, "Driver initialized. Probing the DW1000 ...");
}

bool dw1000_probe(void)
{
    char msg[128];

    /* A real DW1000 answers the device-id register (0x00) with a value
       that prints as "DECA...". */
    dw1000_get_device_id(msg);

    if (strncmp(msg, "DECA", 4) == 0) {
        ESP_LOGI(TAG, "DW1000 module detected - SPI OK");
        return true;
    }

    ESP_LOGE(TAG, "No DW1000 response - check wiring/power!");
    ESP_LOGE(TAG, "Register read as: %s", msg);
    return false;
}

void dw1000_print_device_info(void)
{
    char msg[128];

    dw1000_get_device_id(msg);
    ESP_LOGI(TAG, "Device ID  : %s", msg);

    dw1000_get_eui(msg);
    ESP_LOGI(TAG, "Unique ID  : %s", msg);

    dw1000_get_net_addr(msg);
    ESP_LOGI(TAG, "Net/Addr   : %s", msg);

    dw1000_get_device_mode(msg);
    ESP_LOGI(TAG, "Device mode: %s", msg);
}

/* ============================ addressing ============================ */

void dw1000_set_eui(const char *eui)
{
    dw1000_c_set_eui(eui);
}

void dw1000_set_network_id(uint16_t net_id)
{
    dw1000_c_set_network_id(net_id);
}

void dw1000_set_device_address(uint16_t addr)
{
    dw1000_c_set_device_address(addr);
}

/* ============================ RF configuration ============================ */

void dw1000_set_channel(uint8_t channel)
{
    dw1000_c_set_channel(channel);
}

void dw1000_set_data_rate(uint8_t rate)
{
    dw1000_c_set_data_rate(rate);
}

void dw1000_set_pulse_frequency(uint8_t freq)
{
    dw1000_c_set_pulse_frequency(freq);
}

void dw1000_set_preamble_length(uint8_t len)
{
    dw1000_c_set_preamble_length(len);
}

void dw1000_set_preamble_code(uint8_t code)
{
    dw1000_c_set_preamble_code(code);
}

void dw1000_set_antenna_delay(uint16_t delay_us)
{
    dw1000_c_set_antenna_delay(delay_us);
}

uint16_t dw1000_get_antenna_delay(void)
{
    return dw1000_c_get_antenna_delay();
}

void dw1000_enable_mode(const uint8_t mode[3])
{
    dw1000_c_enable_mode(mode);
}

void dw1000_apply_config(void)
{
    dw1000_c_new_configuration();
    dw1000_c_commit_configuration();
}

/* ============================ transceiver control ============================ */

void dw1000_idle(void)
{
    dw1000_c_idle();
}

void dw1000_start_receive(void)
{
    dw1000_c_new_receive();
    dw1000_c_start_receive();
}

void dw1000_receive_permanently(bool enable)
{
    dw1000_c_receive_permanently(enable ? 1 : 0);
}

void dw1000_send(const uint8_t *data, uint16_t len)
{
    dw1000_c_set_data(data, len);
    dw1000_c_new_transmit();
    dw1000_c_start_transmit();
}

void dw1000_send_at(const uint8_t *data, uint16_t len, uint32_t delay_us)
{
    dw1000_c_set_data(data, len);
    dw1000_c_new_transmit();
    dw1000_c_set_delay(delay_us);
    dw1000_c_start_transmit();
}

/* ============================ data buffer ============================ */

uint16_t dw1000_get_data(uint8_t *buf, uint16_t max_len)
{
    uint16_t len = dw1000_c_get_data_length();
    if (len > max_len) {
        len = max_len;
    }
    if (len > 0) {
        dw1000_c_get_data(buf, len);
    }
    return len;
}

/* ============================ timestamps ============================ */

uint64_t dw1000_get_tx_timestamp(void)
{
    return dw1000_c_get_transmit_timestamp();
}

uint64_t dw1000_get_rx_timestamp(void)
{
    return dw1000_c_get_receive_timestamp();
}

uint64_t dw1000_get_system_timestamp(void)
{
    return dw1000_c_get_system_timestamp();
}

float dw1000_ticks_to_meters(uint64_t tick_diff)
{
    return (float)tick_diff * DW1000_METERS_PER_TICK;
}

/* ============================ receive quality ============================ */

float dw1000_get_rx_power_dbm(void)
{
    return dw1000_c_get_receive_power();
}

float dw1000_get_first_path_power_dbm(void)
{
    return dw1000_c_get_first_path_power();
}

float dw1000_get_rx_quality(void)
{
    return dw1000_c_get_receive_quality();
}

/* ============================ status flags ============================ */

bool dw1000_is_tx_done(void)
{
    return dw1000_c_is_transmit_done() != 0;
}

bool dw1000_is_rx_done(void)
{
    return dw1000_c_is_receive_done() != 0;
}

bool dw1000_is_rx_failed(void)
{
    return dw1000_c_is_receive_failed() != 0;
}

bool dw1000_is_rx_timeout(void)
{
    return dw1000_c_is_receive_timeout() != 0;
}

/* ============================ event callbacks ============================ */

void dw1000_on_error(dw1000_handler_t cb)
{
    dw1000_c_attach_error_handler(cb);
}

void dw1000_on_sent(dw1000_handler_t cb)
{
    dw1000_c_attach_sent_handler(cb);
}

void dw1000_on_received(dw1000_handler_t cb)
{
    dw1000_c_attach_received_handler(cb);
}

void dw1000_on_receive_failed(dw1000_handler_t cb)
{
    dw1000_c_attach_receive_failed_handler(cb);
}

void dw1000_on_receive_timeout(dw1000_handler_t cb)
{
    dw1000_c_attach_receive_timeout_handler(cb);
}

void dw1000_on_receive_timestamp_available(dw1000_handler_t cb)
{
    dw1000_c_attach_receive_timestamp_available_handler(cb);
}

/* ============================ diagnostics ============================ */

void dw1000_get_temp_and_vbat(float *temp_c, float *vbat_v)
{
    float t = 0.0f;
    float v = 0.0f;
    dw1000_c_get_temp_and_vbat(&t, &v);
    if (temp_c) {
        *temp_c = t;
    }
    if (vbat_v) {
        *vbat_v = v;
    }
}