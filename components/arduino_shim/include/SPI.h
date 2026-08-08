/*
 * Arduino-compat SPI shim for ESP-IDF.
 *
 * Implements the small subset of the Arduino SPI API used by the
 * arduino-dw1000 library, on top of the ESP-IDF SPI master driver:
 *   - SPI.begin(sck, miso, mosi, ss)
 *   - SPI.beginTransaction(SPISettings) / SPI.endTransaction()
 *   - SPI.transfer(uint8_t)
 *   - SPI.end()
 *
 * The DW1000 driver toggles the CS line itself via digitalWrite(), so the
 * SPI master is configured with automatic CS disabled and the SS pin is
 * driven as a plain GPIO output.
 */
#ifndef ARDUINO_SHIM_SPI_H
#define ARDUINO_SHIM_SPI_H

#include <stdint.h>
#include <stddef.h>

/* bit order */
enum BitOrder {
    LSBFIRST = 0,
    MSBFIRST = 1
};

/* SPI mode */
#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3

class SPISettings {
public:
    SPISettings()
        : _clock(16000000UL), _bitOrder(MSBFIRST), _dataMode(SPI_MODE0) {}
    SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
        : _clock(clock), _bitOrder(bitOrder), _dataMode(dataMode) {}

    uint32_t _clock;
    uint8_t  _bitOrder;
    uint8_t  _dataMode;
};

class SPIClass {
public:
    SPIClass();

    /* initialize the SPI bus on arbitrary pins (required on ESP32) */
    void begin(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t ss);
    /* no-op; DW1000 always uses the 4-pin form on ESP32 */
    void begin();
    /* de-initialize the SPI bus */
    void end();

    void beginTransaction(const SPISettings &settings);
    void endTransaction();

    uint8_t transfer(uint8_t data);
    void transfer(uint8_t *buf, size_t count);

private:
    bool _inited;
    bool _inTransaction;
    void *_devFast;   /* spi_device_handle_t, 16 MHz, mode 0 */
    void *_devSlow;   /* spi_device_handle_t, 2 MHz,  mode 0 */
    void *_current;   /* device used by the active transaction */
    void *_busMutex;  /* SemaphoreHandle_t serialising transactions */
    uint8_t _scratchRx;
};

extern SPIClass SPI;

#endif /* ARDUINO_SHIM_SPI_H */
