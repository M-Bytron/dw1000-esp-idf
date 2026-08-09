/*
 * my_dw1000.c - Pure-C DW1000 / DWM1000 UWB transceiver library.
 *
 * Plain C implementation. The init / probe / device-info path is fully pure
 * C (dw1000_lowlevel.c on the ESP-IDF SPI driver). The remaining functions
 * still go through the C++ bridge (dw1000_c.h) for now and are ported step
 * by step.
 */
#include "my_dw1000.h"
#include "dw1000_regs.h"
#include "dw1000_lowlevel.h"

#include <stdio.h>
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
    (void)irq; /* interrupt handling is ported in a later step */

    if (dw1000_ll_spi_init(sck, miso, mosi, cs) != ESP_OK) {
        ESP_LOGE(TAG, "SPI init failed - aborting");
        return;
    }

    /* Power-up / reset sequence (DW1000 User Manual 5.6). */
    dw1000_ll_reset(rst);
    dw1000_ll_clock(DW1000_AUTO_CLOCK);

    /* Chip-select defaults + LDE microcode load (required before RX works). */
    dw1000_ll_init_defaults();
    dw1000_ll_manage_lde();

    ESP_LOGI(TAG, "Driver initialized (pure C). Probing the DW1000 ...");
}

bool dw1000_probe(void)
{
    uint32_t id = dw1000_ll_read_device_id();

    /* A real DW1000 answers 0xDECA0130 -> top 16 bits are 0xDECA. */
    if ((id & 0xFFFF0000u) == 0xDECA0000u) {
        ESP_LOGI(TAG, "-> DW1000 module detected - SPI OK (ID: 0x%08X)", (unsigned)id);
        return true;
    }

    ESP_LOGE(TAG, "-> No DW1000 response - check wiring/power! (ID: 0x%08X)", (unsigned)id);
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

/* ============================ device info (pure C) ============================ */

void dw1000_get_device_id(char *buf)
{
    uint8_t data[4];
    dw1000_ll_read(DW1000_DEV_ID, DW1000_NO_SUB, data, sizeof(data));
    /* data[3] is the most significant byte; a real chip prints "DECA". */
    sprintf(buf, "%04X", (unsigned)((data[3] << 8) | data[2]));
}

void dw1000_get_eui(char *buf)
{
    uint8_t data[8];
    dw1000_ll_read(DW1000_EUI, DW1000_NO_SUB, data, sizeof(data));
    sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
            (unsigned)data[7], (unsigned)data[6], (unsigned)data[5], (unsigned)data[4],
            (unsigned)data[3], (unsigned)data[2], (unsigned)data[1], (unsigned)data[0]);
}

void dw1000_get_net_addr(char *buf)
{
    uint8_t data[4];
    dw1000_ll_read(DW1000_PANADR, DW1000_NO_SUB, data, sizeof(data));
    sprintf(buf, "PAN: %02X, Short Address: %02X",
            (unsigned)((data[3] << 8) | data[2]),
            (unsigned)((data[1] << 8) | data[0]));
}

/* map the encoded preamble length to the number of symbols */
static uint32_t dw1000_preamble_symbols(uint8_t enc)
{
    switch (enc) {
    case DW1000_PREAMBLE_LEN_64:    return 64;
    case DW1000_PREAMBLE_LEN_128:   return 128;
    case DW1000_PREAMBLE_LEN_256:   return 256;
    case DW1000_PREAMBLE_LEN_512:   return 512;
    case DW1000_PREAMBLE_LEN_1024:  return 1024;
    case DW1000_PREAMBLE_LEN_1536:  return 1536;
    case DW1000_PREAMBLE_LEN_2048:  return 2048;
    case DW1000_PREAMBLE_LEN_4096:  return 4096;
    default: return 0;
    }
}

void dw1000_get_device_mode(char *buf)
{
    uint8_t dr   = dw1000_ll_get_data_rate();
    uint8_t prf  = dw1000_ll_get_pulse_frequency();
    uint8_t plen = dw1000_ll_get_preamble_length();
    uint8_t pcode = dw1000_ll_get_preamble_code();
    uint8_t ch   = dw1000_ll_get_channel();
    uint32_t dr_kbps = (dr == DW1000_RATE_110KBPS)  ? 110  :
                       (dr == DW1000_RATE_850KBPS)  ? 850  :
                       (dr == DW1000_RATE_6800KBPS) ? 6800 : 0;
    uint32_t prf_mhz  = (prf == DW1000_PRF_16MHZ) ? 16 :
                        (prf == DW1000_PRF_64MHZ) ? 64 : 0;
    uint32_t plen_sym = dw1000_preamble_symbols(plen);

    sprintf(buf, "Data rate: %u kb/s, PRF: %u MHz, Preamble: %u symbols (code #%u), Channel: #%u",
            (unsigned)dr_kbps, (unsigned)prf_mhz, (unsigned)plen_sym,
            (unsigned)pcode, (unsigned)ch);
}

/* ============================ addressing ============================ */

void dw1000_set_eui(const char *eui)
{
    dw1000_ll_set_eui(eui);
}

void dw1000_set_network_id(uint16_t net_id)
{
    dw1000_ll_set_network_id(net_id);
}

void dw1000_set_device_address(uint16_t addr)
{
    dw1000_ll_set_device_address(addr);
}

/* ============================ RF configuration ============================ */

void dw1000_set_channel(uint8_t channel)
{
    dw1000_ll_set_channel(channel);
}

void dw1000_set_data_rate(uint8_t rate)
{
    dw1000_ll_set_data_rate(rate);
}

void dw1000_set_pulse_frequency(uint8_t freq)
{
    dw1000_ll_set_pulse_frequency(freq);
}

void dw1000_set_preamble_length(uint8_t len)
{
    dw1000_ll_set_preamble_length(len);
}

void dw1000_set_preamble_code(uint8_t code)
{
    dw1000_ll_set_preamble_code(code);
}

void dw1000_set_antenna_delay(uint16_t delay_us)
{
    dw1000_ll_set_antenna_delay(delay_us);
}

uint16_t dw1000_get_antenna_delay(void)
{
    return dw1000_ll_get_antenna_delay();
}

void dw1000_enable_mode(const uint8_t mode[3])
{
    dw1000_ll_enable_mode(mode);
}

/* Load the chip defaults (frame filter off, interrupts, channel 5, ...). */
void dw1000_set_defaults(void)
{
    dw1000_ll_set_defaults();
}

/*
 * Two-phase configuration:
 *   dw1000_begin_config();   // idle + load current chip state into caches
 *   dw1000_set_*();          // modify the caches
 *   dw1000_commit_config();  // write everything to the chip + re-tune
 */
void dw1000_begin_config(void)
{
    dw1000_ll_new_configuration();
}

void dw1000_commit_config(void)
{
    dw1000_ll_commit_configuration();
}

/* Convenience: begin_config() + commit_config(). */
void dw1000_apply_config(void)
{
    dw1000_ll_new_configuration();
    dw1000_ll_commit_configuration();
}

/* ============================ transceiver control ============================ */

/* Put the radio in idle state (no RX / TX active). */
void dw1000_idle(void)
{
    dw1000_ll_idle();
}

void dw1000_start_receive(void)
{
    dw1000_ll_new_receive();
    dw1000_ll_start_receive();
}

void dw1000_receive_permanently(bool enable)
{
    dw1000_ll_receive_permanently(enable ? 1 : 0);
}

void dw1000_send(const uint8_t *data, uint16_t len)
{
    dw1000_ll_set_data(data, len);
    dw1000_ll_new_transmit();
    dw1000_ll_start_transmit();
}

void dw1000_send_at(const uint8_t *data, uint16_t len, uint32_t delay_us)
{
    dw1000_ll_set_data(data, len);
    dw1000_ll_new_transmit();
    dw1000_ll_set_delay(delay_us);
    dw1000_ll_start_transmit();
}

void dw1000_send_at_ticks(const uint8_t *data, uint16_t len, uint64_t target_ticks)
{
    dw1000_ll_set_data(data, len);
    dw1000_ll_new_transmit();
    dw1000_ll_start_transmit_at(target_ticks);
}

/* ============================ data buffer ============================ */

uint16_t dw1000_get_data(uint8_t *buf, uint16_t max_len)
{
    uint16_t len = dw1000_ll_get_data_length();
    if (len > max_len) {
        len = max_len;
    }
    if (len > 0) {
        dw1000_ll_get_data(buf, len);
    }
    return len;
}

/* ============================ timestamps ============================ */

uint64_t dw1000_get_tx_timestamp(void)
{
    return dw1000_ll_get_transmit_timestamp();
}

uint64_t dw1000_get_rx_timestamp(void)
{
    return dw1000_ll_get_receive_timestamp();
}

uint64_t dw1000_get_system_timestamp(void)
{
    return dw1000_ll_get_system_timestamp();
}

float dw1000_ticks_to_meters(uint64_t tick_diff)
{
    return (float)tick_diff * DW1000_METERS_PER_TICK;
}

/* ============================ receive quality ============================ */

float dw1000_get_rx_power_dbm(void)
{
    return dw1000_ll_get_receive_power();
}

float dw1000_get_first_path_power_dbm(void)
{
    return dw1000_ll_get_first_path_power();
}

float dw1000_get_rx_quality(void)
{
    return dw1000_ll_get_receive_quality();
}

/* ============================ status flags ============================ */

bool dw1000_is_tx_done(void)
{
    return dw1000_ll_is_transmit_done();
}

bool dw1000_is_rx_done(void)
{
    return dw1000_ll_is_receive_done();
}

bool dw1000_is_rx_failed(void)
{
    return dw1000_ll_is_receive_failed();
}

bool dw1000_is_rx_timeout(void)
{
    return dw1000_ll_is_receive_timeout();
}

/* ============================ event callbacks ============================ */

void dw1000_on_error(dw1000_handler_t cb)
{
    dw1000_ll_attach_error_handler(cb);
}

void dw1000_on_sent(dw1000_handler_t cb)
{
    dw1000_ll_attach_sent_handler(cb);
}

void dw1000_on_received(dw1000_handler_t cb)
{
    dw1000_ll_attach_received_handler(cb);
}

void dw1000_on_receive_failed(dw1000_handler_t cb)
{
    dw1000_ll_attach_receive_failed_handler(cb);
}

void dw1000_on_receive_timeout(dw1000_handler_t cb)
{
    dw1000_ll_attach_receive_timeout_handler(cb);
}

void dw1000_on_receive_timestamp_available(dw1000_handler_t cb)
{
    dw1000_ll_attach_receive_timestamp_available_handler(cb);
}

/* ============================ interrupts ============================ */

void dw1000_interrupt_on_sent(bool enable)
{
    dw1000_ll_interrupt_on_sent(enable ? 1 : 0);
}

void dw1000_interrupt_on_received(bool enable)
{
    dw1000_ll_interrupt_on_received(enable ? 1 : 0);
}

void dw1000_interrupt_on_receive_failed(bool enable)
{
    dw1000_ll_interrupt_on_receive_failed(enable ? 1 : 0);
}

void dw1000_interrupt_on_receive_timeout(bool enable)
{
    dw1000_ll_interrupt_on_receive_timeout(enable ? 1 : 0);
}

void dw1000_interrupt_on_receive_timestamp_available(bool enable)
{
    dw1000_ll_interrupt_on_receive_timestamp_available(enable ? 1 : 0);
}

esp_err_t dw1000_irq_start(uint8_t irq_gpio)
{
    return dw1000_ll_irq_start(irq_gpio);
}

/* ============================ diagnostics ============================ */

void dw1000_get_temp_and_vbat(float *temp_c, float *vbat_v)
{
    float t = 0.0f;
    float v = 0.0f;
    dw1000_ll_get_temp_and_vbat(&t, &v);
    if (temp_c) {
        *temp_c = t;
    }
    if (vbat_v) {
        *vbat_v = v;
    }
}