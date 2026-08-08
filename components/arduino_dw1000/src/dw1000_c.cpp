/*
 * C API wrapper around the arduino-dw1000 C++ class.
 *
 * This file is C++ on purpose: it bridges the C++ DW1000Class to the plain C
 * functions declared in dw1000_c.h, so the application (main.c) can be C.
 */
#include "dw1000_c.h"

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

} /* extern "C" */
