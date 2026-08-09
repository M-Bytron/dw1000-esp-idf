/*
 * C API wrapper around the arduino-dw1000 C++ class.
 *
 * This file is C++ on purpose: it bridges the C++ DW1000Class to the plain C
 * functions declared in dw1000_c.h, so the application (main.c) can be C.
 */
#include "dw1000_c.h"

#include <string.h>

#include "DW1000.h"

extern "C" {

void dw1000_set_spi_pins(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t ss)
{
    DW1000.setSpiPins(sck, miso, mosi, ss);
}

void dw1000_begin(uint8_t irq, uint8_t rst)
{
    DW1000.begin(irq, rst);
}

void dw1000_select(uint8_t ss)
{
    DW1000.select(ss);
}

void dw1000_get_device_id(char *buf)
{
    DW1000.getPrintableDeviceIdentifier(buf);
}

void dw1000_get_eui(char *buf)
{
    DW1000.getPrintableExtendedUniqueIdentifier(buf);
}

void dw1000_get_net_addr(char *buf)
{
    DW1000.getPrintableNetworkIdAndShortAddress(buf);
}

void dw1000_get_device_mode(char *buf)
{
    DW1000.getPrintableDeviceMode(buf);
}

/* ================= low-level configuration ================= */

void dw1000_c_set_network_id(uint16_t val)
{
    DW1000.setNetworkId(val);
}

void dw1000_c_set_device_address(uint16_t val)
{
    DW1000.setDeviceAddress(val);
}

void dw1000_c_set_eui(const char *eui)
{
    char buf[24];
    size_t n = strlen(eui);
    size_t i = 0;
    for (; i < n && i < sizeof(buf) - 1; i++) {
        buf[i] = eui[i];
    }
    buf[i] = '\0';
    DW1000.setEUI(buf);
}

void dw1000_c_set_data_rate(uint8_t rate)
{
    DW1000.setDataRate(rate);
}

void dw1000_c_set_pulse_frequency(uint8_t freq)
{
    DW1000.setPulseFrequency(freq);
}

void dw1000_c_set_preamble_length(uint8_t len)
{
    DW1000.setPreambleLength(len);
}

void dw1000_c_set_preamble_code(uint8_t code)
{
    DW1000.setPreambleCode(code);
}

void dw1000_c_set_channel(uint8_t channel)
{
    DW1000.setChannel(channel);
}

void dw1000_c_use_smart_power(int enabled)
{
    DW1000.useSmartPower(enabled ? true : false);
}

void dw1000_c_set_antenna_delay(uint16_t delay)
{
    DW1000.setAntennaDelay(delay);
}

uint16_t dw1000_c_get_antenna_delay(void)
{
    return DW1000.getAntennaDelay();
}

void dw1000_c_enable_mode(const uint8_t mode[3])
{
    DW1000.enableMode(mode);
}

void dw1000_c_set_defaults(void)
{
    DW1000.setDefaults();
}

/* ================= transceiver control ================= */

void dw1000_c_idle(void)
{
    DW1000.idle();
}

void dw1000_c_new_configuration(void)
{
    DW1000.newConfiguration();
}

void dw1000_c_commit_configuration(void)
{
    DW1000.commitConfiguration();
}

void dw1000_c_new_receive(void)
{
    DW1000.newReceive();
}

void dw1000_c_start_receive(void)
{
    DW1000.startReceive();
}

void dw1000_c_new_transmit(void)
{
    DW1000.newTransmit();
}

void dw1000_c_start_transmit(void)
{
    DW1000.startTransmit();
}

void dw1000_c_receive_permanently(int enabled)
{
    DW1000.receivePermanently(enabled ? true : false);
}

void dw1000_c_set_delay(uint64_t delay_us)
{
    DW1000Time t;
    t.setTime((float)delay_us);
    DW1000.setDelay(t);
}

/* ================= data buffer ================= */

void dw1000_c_set_data(const uint8_t *data, uint16_t n)
{
    DW1000.setData(const_cast<byte *>(data), n);
}

void dw1000_c_get_data(uint8_t *data, uint16_t n)
{
    DW1000.getData(data, n);
}

uint16_t dw1000_c_get_data_length(void)
{
    return DW1000.getDataLength();
}

/* ================= timestamps ================= */

uint64_t dw1000_c_get_transmit_timestamp(void)
{
    DW1000Time t;
    DW1000.getTransmitTimestamp(t);
    return (uint64_t)t.getTimestamp();
}

uint64_t dw1000_c_get_receive_timestamp(void)
{
    DW1000Time t;
    DW1000.getReceiveTimestamp(t);
    return (uint64_t)t.getTimestamp();
}

uint64_t dw1000_c_get_system_timestamp(void)
{
    DW1000Time t;
    DW1000.getSystemTimestamp(t);
    return (uint64_t)t.getTimestamp();
}

/* ================= receive quality ================= */

float dw1000_c_get_receive_power(void)
{
    return DW1000.getReceivePower();
}

float dw1000_c_get_first_path_power(void)
{
    return DW1000.getFirstPathPower();
}

float dw1000_c_get_receive_quality(void)
{
    return DW1000.getReceiveQuality();
}

/* ================= interrupt configuration ================= */

void dw1000_c_interrupt_on_sent(int enabled)
{
    DW1000.interruptOnSent(enabled ? true : false);
}

void dw1000_c_interrupt_on_received(int enabled)
{
    DW1000.interruptOnReceived(enabled ? true : false);
}

void dw1000_c_interrupt_on_receive_failed(int enabled)
{
    DW1000.interruptOnReceiveFailed(enabled ? true : false);
}

void dw1000_c_interrupt_on_receive_timeout(int enabled)
{
    DW1000.interruptOnReceiveTimeout(enabled ? true : false);
}

void dw1000_c_interrupt_on_receive_timestamp_available(int enabled)
{
    DW1000.interruptOnReceiveTimestampAvailable(enabled ? true : false);
}

/* ================= status flags ================= */

int dw1000_c_is_transmit_done(void)
{
    return DW1000.isTransmitDone() ? 1 : 0;
}

int dw1000_c_is_receive_done(void)
{
    return DW1000.isReceiveDone() ? 1 : 0;
}

int dw1000_c_is_receive_failed(void)
{
    return DW1000.isReceiveFailed() ? 1 : 0;
}

int dw1000_c_is_receive_timeout(void)
{
    return DW1000.isReceiveTimeout() ? 1 : 0;
}

int dw1000_c_is_receive_timestamp_available(void)
{
    return DW1000.isReceiveTimestampAvailable() ? 1 : 0;
}

/* ================= callbacks ================= */

void dw1000_c_attach_error_handler(void (*cb)(void))
{
    DW1000.attachErrorHandler(cb);
}

void dw1000_c_attach_sent_handler(void (*cb)(void))
{
    DW1000.attachSentHandler(cb);
}

void dw1000_c_attach_received_handler(void (*cb)(void))
{
    DW1000.attachReceivedHandler(cb);
}

void dw1000_c_attach_receive_failed_handler(void (*cb)(void))
{
    DW1000.attachReceiveFailedHandler(cb);
}

void dw1000_c_attach_receive_timeout_handler(void (*cb)(void))
{
    DW1000.attachReceiveTimeoutHandler(cb);
}

void dw1000_c_attach_receive_timestamp_available_handler(void (*cb)(void))
{
    DW1000.attachReceiveTimestampAvailableHandler(cb);
}

/* ================= misc / diagnostics ================= */

void dw1000_c_get_temp_and_vbat(float *temp, float *vbat)
{
    float t = 0.0f;
    float v = 0.0f;
    DW1000.getTempAndVbat(t, v);
    if (temp) {
        *temp = t;
    }
    if (vbat) {
        *vbat = v;
    }
}

} /* extern "C" */
