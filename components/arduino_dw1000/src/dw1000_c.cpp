/*
 * C API wrapper around the arduino-dw1000 C++ class.
 *
 * This file is C++ on purpose: it bridges the C++ DW1000Class to the plain C
 * functions declared in dw1000_c.h, so the application (main.c) can be C.
 */
#include "dw1000_c.h"

#include "DW1000.h"
#include "DW1000Time.h"

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

/* ---- general configuration ---- */

void dw1000_new_configuration(void)
{
    DW1000.newConfiguration();
}

void dw1000_set_defaults(void)
{
    DW1000.setDefaults();
}

void dw1000_set_device_address(uint16_t addr)
{
    DW1000.setDeviceAddress(addr);
}

void dw1000_set_network_id(uint16_t id)
{
    DW1000.setNetworkId(id);
}

void dw1000_enable_mode(uint8_t data_rate, uint8_t pulse_freq, uint8_t preamble_len)
{
    byte mode[3] = { data_rate, pulse_freq, preamble_len };
    DW1000.enableMode(mode);
}

void dw1000_set_antenna_delay(uint16_t value)
{
    DW1000.setAntennaDelay(value);
}

void dw1000_commit_configuration(void)
{
    DW1000.commitConfiguration();
}

/* ---- event handlers ---- */

void dw1000_attach_error_handler(void (*handler)(void))
{
    DW1000.attachErrorHandler(handler);
}

void dw1000_attach_sent_handler(void (*handler)(void))
{
    DW1000.attachSentHandler(handler);
}

void dw1000_attach_received_handler(void (*handler)(void))
{
    DW1000.attachReceivedHandler(handler);
}

void dw1000_attach_receive_failed_handler(void (*handler)(void))
{
    DW1000.attachReceiveFailedHandler(handler);
}

/* ---- reception ---- */

void dw1000_new_receive(void)
{
    DW1000.newReceive();
}

void dw1000_receive_permanently(int val)
{
    DW1000.receivePermanently(val != 0);
}

void dw1000_start_receive(void)
{
    DW1000.startReceive();
}

/* ---- transmission ---- */

void dw1000_new_transmit(void)
{
    DW1000.newTransmit();
}

void dw1000_set_data(uint8_t *data, uint16_t n)
{
    DW1000.setData(data, n);
}

void dw1000_get_data(uint8_t *data, uint16_t n)
{
    DW1000.getData(data, n);
}

void dw1000_set_delay_us(uint32_t us, uint8_t expected_ts[5])
{
    DW1000Time deltaTime((int32_t)us, DW1000Time::MICROSECONDS);
    DW1000Time expected = DW1000.setDelay(deltaTime);
    if (expected_ts != NULL) {
        expected.getTimestamp(expected_ts);
    }
}

void dw1000_start_transmit(void)
{
    DW1000.startTransmit();
}

/* ---- timestamps ---- */

void dw1000_get_transmit_timestamp(uint8_t ts[5])
{
    DW1000Time t;
    DW1000.getTransmitTimestamp(t);
    t.getTimestamp(ts);
}

void dw1000_get_receive_timestamp(uint8_t ts[5])
{
    DW1000Time t;
    DW1000.getReceiveTimestamp(t);
    t.getTimestamp(ts);
}

/* ---- asymmetric two-way ranging ---- */

float dw1000_ranging_compute_asymmetric(const uint8_t poll_sent[5],
                                        const uint8_t poll_received[5],
                                        const uint8_t poll_ack_sent[5],
                                        const uint8_t poll_ack_received[5],
                                        const uint8_t range_sent[5],
                                        const uint8_t range_received[5])
{
    DW1000Time tPollSent((byte *)poll_sent);
    DW1000Time tPollReceived((byte *)poll_received);
    DW1000Time tPollAckSent((byte *)poll_ack_sent);
    DW1000Time tPollAckReceived((byte *)poll_ack_received);
    DW1000Time tRangeSent((byte *)range_sent);
    DW1000Time tRangeReceived((byte *)range_received);

    /* asymmetric two-way ranging (same algorithm as the Arduino example) */
    DW1000Time round1 = (tPollAckReceived - tPollSent).wrap();
    DW1000Time reply1 = (tPollAckSent - tPollReceived).wrap();
    DW1000Time round2 = (tRangeReceived - tPollAckSent).wrap();
    DW1000Time reply2 = (tRangeSent - tPollAckReceived).wrap();

    DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
    return tof.getAsMeters();
}

} /* extern "C" */
