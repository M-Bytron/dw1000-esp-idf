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
