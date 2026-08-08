/*
 * Arduino-compat shim for ESP-IDF: SPI master.
 *
 * The DW1000 driver toggles CS itself with digitalWrite(), so automatic CS
 * is disabled (spics_io_num = -1) and the SS pin is a plain GPIO output.
 *
 * The library uses two SPISettings: 16 MHz (fast) and 2 MHz (slow) while the
 * DW1000 is running on the XTI clock during init. Two devices on the same
 * bus are created and the active one is selected in beginTransaction().
 *
 * SPI access is serialised with a FreeRTOS mutex held from beginTransaction()
 * to endTransaction() so a transaction (CS low -> CS high) is never
 * interleaved with another task (e.g. the deferred DW1000 ISR task).
 */
#include "SPI.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define SPI_BUS_HOST SPI2_HOST
#define SPI_FAST_HZ  (16 * 1000 * 1000)
#define SPI_SLOW_HZ  (2 * 1000 * 1000)

static const char *TAG = "arduino_spi";

SPIClass::SPIClass()
    : _inited(false), _inTransaction(false), _devFast(NULL),
      _devSlow(NULL), _current(NULL), _busMutex(NULL)
{
}

void SPIClass::begin(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t ss)
{
    if (_inited) {
        return;
    }

    _busMutex = (void *)xSemaphoreCreateMutex();
    if (_busMutex == NULL) {
        ESP_LOGE(TAG, "failed to create SPI mutex");
        return;
    }

    spi_bus_config_t buscfg = {};
    buscfg.sclk_io_num = (int)sck;
    buscfg.mosi_io_num = (int)mosi;
    buscfg.miso_io_num = (int)miso;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 64;

    esp_err_t ret = spi_bus_initialize(SPI_BUS_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    /* CS is managed manually by the DW1000 driver -> plain GPIO output */
    gpio_set_direction((gpio_num_t)ss, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)ss, 1);

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = SPI_FAST_HZ;
    devcfg.mode = 0;
    devcfg.spics_io_num = -1;       /* manual CS */
    devcfg.queue_size = 4;
    devcfg.flags = 0;

    spi_device_handle_t fast = NULL;
    spi_device_handle_t slow = NULL;

    ret = spi_bus_add_device(SPI_BUS_HOST, &devcfg, &fast);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "add fast device failed: %s", esp_err_to_name(ret));
        return;
    }

    devcfg.clock_speed_hz = SPI_SLOW_HZ;
    ret = spi_bus_add_device(SPI_BUS_HOST, &devcfg, &slow);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "add slow device failed: %s", esp_err_to_name(ret));
        return;
    }

    _devFast = (void *)fast;
    _devSlow = (void *)slow;
    _current = _devFast;
    _inited = true;
}

void SPIClass::begin()
{
    ESP_LOGW(TAG, "SPI.begin() without pins is a no-op; use begin(sck,miso,mosi,ss)");
}

void SPIClass::end()
{
    if (_devFast != NULL) {
        spi_bus_remove_device((spi_device_handle_t)_devFast);
        _devFast = NULL;
    }
    if (_devSlow != NULL) {
        spi_bus_remove_device((spi_device_handle_t)_devSlow);
        _devSlow = NULL;
    }
    if (_inited) {
        spi_bus_free(SPI_BUS_HOST);
        _inited = false;
    }
    if (_busMutex != NULL) {
        vSemaphoreDelete((SemaphoreHandle_t)_busMutex);
        _busMutex = NULL;
    }
    _current = NULL;
    _inTransaction = false;
}

void SPIClass::beginTransaction(const SPISettings &settings)
{
    if (_busMutex != NULL) {
        xSemaphoreTake((SemaphoreHandle_t)_busMutex, portMAX_DELAY);
    }
    _inTransaction = true;
    /* DW1000 uses 16 MHz (fast) and 2 MHz (slow); pick the closest device */
    _current = (settings._clock < 8000000UL) ? _devSlow : _devFast;
}

void SPIClass::endTransaction()
{
    _inTransaction = false;
    if (_busMutex != NULL) {
        xSemaphoreGive((SemaphoreHandle_t)_busMutex);
    }
}

uint8_t SPIClass::transfer(uint8_t data)
{
    bool ownTransaction = false;
    if (!_inTransaction) {
        /* transfer() outside begin/endTransaction: guard this call only */
        if (_busMutex != NULL) {
            xSemaphoreTake((SemaphoreHandle_t)_busMutex, portMAX_DELAY);
        }
        ownTransaction = true;
    }
    if (_current == NULL) {
        _current = _devFast;
    }

    uint8_t rx = 0;
    spi_transaction_t t = {};
    t.length = 8;
    t.rxlength = 8;
    t.tx_buffer = &data;
    t.rx_buffer = &rx;

    esp_err_t ret = spi_device_polling_transmit((spi_device_handle_t)_current, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "polling transmit failed: %s", esp_err_to_name(ret));
    }

    if (ownTransaction && _busMutex != NULL) {
        xSemaphoreGive((SemaphoreHandle_t)_busMutex);
    }
    return rx;
}

void SPIClass::transfer(uint8_t *buf, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        buf[i] = transfer(buf[i]);
    }
}

SPIClass SPI;
