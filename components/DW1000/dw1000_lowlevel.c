/*
 * dw1000_lowlevel.c - Pure-C low-level DW1000 driver.
 *
 * Implemented directly on the ESP-IDF SPI master driver (driver/spi_master.h)
 * and GPIO driver. No Arduino / no C++.
 *
 * SPI protocol (DW1000 User Manual chapter 6):
 *   - first byte is a header: bit7 read(0)/write(1), bit6 sub-address(1),
 *     bits5..0 register address
 *   - if sub-addressing, 1 byte (<128) or 2 bytes (>=128) of sub-address
 *   - then the data bytes (or dummy bytes for reads)
 *   - mode 0, MSB first
 */
#include "dw1000_lowlevel.h"
#include "dw1000_regs.h"

#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"          /* esp_rom_delay_us */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "DW1000_LL";

/* The DW1000 is a single SPI slave; one handle is enough. */
static spi_device_handle_t s_spi;

/* Serialises all radio/SPI sequences between the IRQ task (status handling +
   RX capture) and the application TX task (mode change + transmit). */
static SemaphoreHandle_t s_radio_lock = NULL;

/* Given by the IRQ handler when a transmit completes (TXFRS). */
static SemaphoreHandle_t s_tx_done = NULL;

/* SPI failures: counted always, logged only the first few times so a burst
   cannot flood the UART (and cannot slow the IRQ handler down). */
#define DW1000_SPI_ERR_LOG_MAX 10
static volatile uint32_t s_spi_err_count = 0;

/* SPI clock speeds used by the reference driver (DW1000.cpp). */
#define DW1000_SPI_CLK_SLOW 2000000   /* 2 MHz, used while on XTI clock  */
#define DW1000_SPI_CLK_FAST 16000000  /* 16 MHz, used on PLL/AUTO clock  */

esp_err_t dw1000_ll_spi_init(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs)
{
    esp_err_t ret;

    if (s_radio_lock == NULL) {
        s_radio_lock = xSemaphoreCreateMutex();
        if (s_radio_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    spi_bus_config_t buscfg = {
        .sclk_io_num       = sck,
        .mosi_io_num       = mosi,
        .miso_io_num       = miso,
        .quadwp_io_num     = -1,
        .quadhd_io_num     = -1,
        .max_transfer_sz   = 1024,
    };
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .mode            = 2,                  /* CPOL=0, CPHA=0, MSB first */
        .clock_speed_hz  = 1E6,
        .spics_io_num    = cs,
        .queue_size      = 4,
        .cs_ena_posttrans = 10,                /* CS hold time (SPI clocks) */
    };

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SPI master ready (sck=%u miso=%u mosi=%u cs=%u, 2 MHz)",
             (unsigned)sck, (unsigned)miso, (unsigned)mosi, (unsigned)cs);
    return ESP_OK;
}

/* Build the SPI header for a register access; returns header length. */
static uint8_t dw1000_ll_build_header(uint8_t base, uint8_t reg, uint16_t sub,
                                      uint8_t header[3])
{
    if (sub == DW1000_NO_SUB) {
        header[0] = base | reg;
        return 1;
    }
    /* WRITE_SUB == WRITE|0x40 and READ_SUB == READ|0x40, so OR-ing the
       sub-address bit (0x40) on top of the read/write base selects the
       right "with sub-address" command. */
    header[0] = base | DW1000_READ_SUB | reg;
    if (sub < 128) {
        header[1] = (uint8_t)sub;
        return 2;
    }
    header[1] = DW1000_RW_SUB_EXT | (uint8_t)sub;
    header[2] = (uint8_t)(sub >> 7);
    return 3;
}

esp_err_t dw1000_ll_read(uint8_t reg, uint16_t sub, uint8_t *data, uint16_t n)
{
    uint8_t header[3];
    uint8_t hlen = dw1000_ll_build_header(DW1000_READ, reg, sub, header);

    /* tx = header + dummy bytes, rx = (ignored header) + data */
    uint8_t *tx = (uint8_t *)malloc(hlen + n);
    uint8_t *rx = (uint8_t *)malloc(hlen + n);
    if (tx == NULL || rx == NULL) {
        free(tx);
        free(rx);
        memset(data, 0, n);
        s_spi_err_count++;
        if (s_spi_err_count <= DW1000_SPI_ERR_LOG_MAX) {
            ESP_LOGE(TAG, "SPI READ reg 0x%02X sub 0x%02X len %u FAILED: no memory (err #%u)",
                     (unsigned)reg, (unsigned)sub, (unsigned)n,
                     (unsigned)s_spi_err_count);
        }
        return ESP_ERR_NO_MEM;
    }

    memcpy(tx, header, hlen);
    memset(tx + hlen, DW1000_JUNK, n);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length   = (hlen + n) * 8;   /* bits transmitted on MOSI   */
    t.rxlength = (hlen + n) * 8;   /* bits received on MISO      */
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    esp_err_t ret = spi_device_polling_transmit(s_spi, &t);
    if (ret != ESP_OK) {
        /* Zero-fill, but the caller MUST check the return value: an all-zero
           register and a failed read look identical in `data`. */
        memset(data, 0, n);
        s_spi_err_count++;
        if (s_spi_err_count <= DW1000_SPI_ERR_LOG_MAX) {
            ESP_LOGE(TAG, "SPI READ reg 0x%02X sub 0x%02X len %u FAILED: %s (err #%u)",
                     (unsigned)reg, (unsigned)sub, (unsigned)n,
                     esp_err_to_name(ret), (unsigned)s_spi_err_count);
        }
    } else {
        memcpy(data, rx + hlen, n);
    }

    free(tx);
    free(rx);
    return ret;
}

esp_err_t dw1000_ll_write(uint8_t reg, uint16_t sub, const uint8_t *data, uint16_t n)
{
    uint8_t header[3];
    uint8_t hlen = dw1000_ll_build_header(DW1000_WRITE, reg, sub, header);

    uint8_t *tx = (uint8_t *)malloc(hlen + n);
    if (tx == NULL) {
        s_spi_err_count++;
        if (s_spi_err_count <= DW1000_SPI_ERR_LOG_MAX) {
            ESP_LOGE(TAG, "SPI WRITE reg 0x%02X sub 0x%02X len %u FAILED: no memory (err #%u)",
                     (unsigned)reg, (unsigned)sub, (unsigned)n,
                     (unsigned)s_spi_err_count);
        }
        return ESP_ERR_NO_MEM;
    }

    memcpy(tx, header, hlen);
    memcpy(tx + hlen, data, n);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length   = (hlen + n) * 8;
    t.tx_buffer = tx;

    esp_err_t ret = spi_device_polling_transmit(s_spi, &t);
    if (ret != ESP_OK) {
        s_spi_err_count++;
        if (s_spi_err_count <= DW1000_SPI_ERR_LOG_MAX) {
            ESP_LOGE(TAG, "SPI WRITE reg 0x%02X sub 0x%02X len %u FAILED: %s (err #%u)",
                     (unsigned)reg, (unsigned)sub, (unsigned)n,
                     esp_err_to_name(ret), (unsigned)s_spi_err_count);
        }
    }

    free(tx);
    return ret;
}

void dw1000_ll_reset(uint8_t rst)
{
    if (rst == 0xFF) {
        ESP_LOGW(TAG, "No RST pin wired (0xFF) - hard reset skipped");
        return;
    }

    /* Drive RSTn low. */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << rst),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(rst, 0);
    esp_rom_delay_us(2000);   /* RSTn low: min 50 ns, use margin */

    /* Leave RSTn floating (input, no pull) as required by DW1000 5.6.1. */
    io.mode         = GPIO_MODE_INPUT;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io);
    vTaskDelay(pdMS_TO_TICKS(10));  /* DWM1000 power-up: ~3 ms, use margin */
}

void dw1000_ll_clock(uint8_t clock)
{
    uint8_t pmscctrl0[4];

    dw1000_ll_read(DW1000_PMSC, DW1000_PMSC_CTRL0_SUB, pmscctrl0, sizeof(pmscctrl0));
    if (clock == DW1000_AUTO_CLOCK) {
        pmscctrl0[0] = DW1000_AUTO_CLOCK;
        pmscctrl0[1] &= 0xFE;
    } else if (clock == DW1000_XTI_CLOCK) {
        pmscctrl0[0] &= 0xFC;
        pmscctrl0[0] |= DW1000_XTI_CLOCK;
    } else if (clock == DW1000_PLL_CLOCK) {
        pmscctrl0[0] &= 0xFC;
        pmscctrl0[0] |= DW1000_PLL_CLOCK;
    } else {
        ESP_LOGW(TAG, "Unknown clock %u, ignoring", clock);
        return;
    }
    dw1000_ll_write(DW1000_PMSC, DW1000_PMSC_CTRL0_SUB, pmscctrl0, sizeof(pmscctrl0));
}

uint32_t dw1000_ll_read_device_id(void)
{
    uint8_t data[4];

    dw1000_ll_read(DW1000_DEV_ID, DW1000_NO_SUB, data, sizeof(data));
    /* data[3] is the most significant byte (0xDE, 0xCA for a real chip). */
    return ((uint32_t)data[3] << 24) | ((uint32_t)data[2] << 16) |
           ((uint32_t)data[1] << 8) | (uint32_t)data[0];
}

/* ==========================================================================
 * Configuration state, setters and tuning (pure-C port of DW1000.cpp).
 *
 * Most DW1000 settings live in register caches that are only written to the
 * chip by commitConfiguration(). We mirror that with static state here:
 *   1. dw1000_ll_new_configuration()   - idle + load caches from the chip
 *   2. dw1000_ll_set_*()               - modify the caches (and remember)
 *   3. dw1000_ll_commit_configuration()- write caches + re-tune the radio
 * ======================================================================== */

/* register caches */
static uint8_t g_syscfg[DW1000_LEN_SYS_CFG]      = {0};
static uint8_t g_sysmask[DW1000_LEN_SYS_MASK]    = {0};
static uint8_t g_txfctrl[DW1000_LEN_TX_FCTRL]    = {0};
static uint8_t g_chanctrl[DW1000_LEN_CHAN_CTRL]  = {0};
static uint8_t g_network_addr[DW1000_LEN_PANADR] = {0};

/* remembered configuration */
static uint8_t  g_device_mode          = DW1000_MODE_IDLE;
static uint8_t  g_data_rate            = 0;
static uint8_t  g_pulse_frequency      = 0;
static uint8_t  g_preamble_length      = 0;
static uint8_t  g_preamble_code        = 0;
static uint8_t  g_channel              = 0;
static uint8_t  g_pac_size             = DW1000_PAC_SIZE_64;
static uint8_t  g_extended_frame_length = DW1000_FRAME_LENGTH_NORMAL;
static bool     g_smart_power          = false;
static bool     g_frame_check          = true;  /* FCS enabled by default */
static bool     g_permanent_receive    = false;
static bool     g_tx_delayed           = false; /* delayed TX armed by set_delay */
static uint64_t g_antenna_delay        = 0;
static bool     g_antenna_calibrated   = false;
static uint8_t  g_vmeas3v3            = 0;  /* OTP: 3.3 V reading from production test */
static uint8_t  g_tmeas23C            = 0;  /* OTP: 23 C reading from production test */

/* default mode used by setDefaults(): LONGDATA_RANGE_LOWPOWER */
static const uint8_t DW1000_LL_DEFAULT_MODE[3] = {
    DW1000_RATE_110KBPS, DW1000_PRF_16MHZ, DW1000_PREAMBLE_LEN_2048
};

/* ---- internal helpers ---- */

static void dw1000_ll_set_bit(uint8_t *data, uint16_t n, uint16_t bit, bool val)
{
    uint16_t idx = bit / 8;
    if (idx >= n) {
        return;
    }
    if (val) {
        data[idx] |= (uint8_t)(1u << (bit % 8));
    } else {
        data[idx] &= (uint8_t)~(1u << (bit % 8));
    }
}

/* write a little-endian numeric value into a byte array */
static void dw1000_ll_write_value(uint8_t *data, uint32_t val, uint16_t n)
{
    uint16_t i;
    for (i = 0; i < n; i++) {
        data[i] = (uint8_t)((val >> (i * 8)) & 0xFF);
    }
}

static uint8_t dw1000_ll_nibble(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    return 0xFF;
}

/* read a 4-byte word from OTP memory (DW1000 user manual 6.3.3) */
static void dw1000_ll_read_otp(uint16_t address, uint8_t data[4])
{
    uint8_t addr_bytes[2];
    uint8_t ctrl;

    addr_bytes[0] = (uint8_t)(address & 0xFF);
    addr_bytes[1] = (uint8_t)((address >> 8) & 0xFF);
    dw1000_ll_write(DW1000_OTP_IF, DW1000_OTP_ADDR_SUB, addr_bytes, 2);

    ctrl = 0x03;   /* OTPRDEN | OTPREAD */
    dw1000_ll_write(DW1000_OTP_IF, DW1000_OTP_CTRL_SUB, &ctrl, 1);
    ctrl = 0x01;   /* OTPRDEN */
    dw1000_ll_write(DW1000_OTP_IF, DW1000_OTP_CTRL_SUB, &ctrl, 1);
    dw1000_ll_read(DW1000_OTP_IF, DW1000_OTP_RDAT_SUB, data, 4);
    ctrl = 0x00;
    dw1000_ll_write(DW1000_OTP_IF, DW1000_OTP_CTRL_SUB, &ctrl, 1);
}

/* ---- register cache read/write ---- */

static void dw1000_ll_read_syscfg(void)
{
    dw1000_ll_read(DW1000_SYS_CFG, DW1000_NO_SUB, g_syscfg, DW1000_LEN_SYS_CFG);
}

static void dw1000_ll_write_syscfg(void)
{
    dw1000_ll_write(DW1000_SYS_CFG, DW1000_NO_SUB, g_syscfg, DW1000_LEN_SYS_CFG);
}

/* ---- frame filtering (hardware receive filter) ---- */

void dw1000_ll_apply_frame_filter(int allow_extended, int allow_broadcast)
{
    int enable = (allow_extended != 0) || (allow_broadcast != 0);
    dw1000_ll_set_bit(g_syscfg, DW1000_LEN_SYS_CFG, DW1000_FFE_BIT, enable);
    dw1000_ll_set_bit(g_syscfg, DW1000_LEN_SYS_CFG, DW1000_FFAE_BIT, allow_extended);
    dw1000_ll_set_bit(g_syscfg, DW1000_LEN_SYS_CFG, DW1000_FFBC_BIT, allow_broadcast);
    dw1000_ll_set_bit(g_syscfg, DW1000_LEN_SYS_CFG, DW1000_FFAA_BIT, 0);
    dw1000_ll_write_syscfg();
}

static void dw1000_ll_use_extended_frame_length(bool val)
{
    g_extended_frame_length = val ? DW1000_FRAME_LENGTH_EXTENDED : DW1000_FRAME_LENGTH_NORMAL;
    g_syscfg[2] &= 0xFC;
    g_syscfg[2] |= g_extended_frame_length;
}

static void dw1000_ll_suppress_frame_check(bool val)
{
    g_frame_check = !val;
}

static void dw1000_ll_set_receiver_auto_reenable(bool val)
{
    dw1000_ll_set_bit(g_syscfg, DW1000_LEN_SYS_CFG, DW1000_RXAUTR_BIT, val);
}

void dw1000_ll_interrupt_on_sent(int enabled)
{
    dw1000_ll_set_bit(g_sysmask, DW1000_LEN_SYS_MASK, DW1000_TXFRS_BIT, enabled != 0);
}

void dw1000_ll_interrupt_on_received(int enabled)
{
    dw1000_ll_set_bit(g_sysmask, DW1000_LEN_SYS_MASK, DW1000_RXDFR_BIT, enabled != 0);
    dw1000_ll_set_bit(g_sysmask, DW1000_LEN_SYS_MASK, DW1000_RXFCG_BIT, enabled != 0);
}

void dw1000_ll_interrupt_on_receive_failed(int enabled)
{
    dw1000_ll_set_bit(g_sysmask, DW1000_LEN_SYS_MASK, DW1000_LDEERR_BIT, enabled != 0);
    dw1000_ll_set_bit(g_sysmask, DW1000_LEN_SYS_MASK, DW1000_RXFCE_BIT, enabled != 0);
    dw1000_ll_set_bit(g_sysmask, DW1000_LEN_SYS_MASK, DW1000_RXPHE_BIT, enabled != 0);
    dw1000_ll_set_bit(g_sysmask, DW1000_LEN_SYS_MASK, DW1000_RXRFSL_BIT, enabled != 0);
}

void dw1000_ll_interrupt_on_receive_timeout(int enabled)
{
    dw1000_ll_set_bit(g_sysmask, DW1000_LEN_SYS_MASK, DW1000_RXRFTO_BIT, enabled != 0);
}

void dw1000_ll_interrupt_on_receive_timestamp_available(int enabled)
{
    dw1000_ll_set_bit(g_sysmask, DW1000_LEN_SYS_MASK, DW1000_LDEDONE_BIT, enabled != 0);
}

/* internal: used by setDefaults() */
static void dw1000_ll_interrupt_on_automatic_ack_trigger(bool val)
{
    dw1000_ll_set_bit(g_sysmask, DW1000_LEN_SYS_MASK, DW1000_AAT_BIT, val);
}

/* ---- device addressing ---- */

void dw1000_ll_set_network_id(uint16_t val)
{
    g_network_addr[2] = (uint8_t)(val & 0xFF);
    g_network_addr[3] = (uint8_t)((val >> 8) & 0xFF);
}

void dw1000_ll_set_device_address(uint16_t val)
{
    g_network_addr[0] = (uint8_t)(val & 0xFF);
    g_network_addr[1] = (uint8_t)((val >> 8) & 0xFF);
}

void dw1000_ll_set_eui(const char *eui)
{
    uint8_t bytes[8];
    uint8_t reversed[8];
    int i;

    for (i = 0; i < 8; i++) {
        bytes[i] = (uint8_t)((dw1000_ll_nibble(eui[i * 3]) << 4) |
                             dw1000_ll_nibble(eui[i * 3 + 1]));
    }
    /* the EUI register expects the bytes in reverse order */
    for (i = 0; i < 8; i++) {
        reversed[i] = bytes[7 - i];
    }
    dw1000_ll_write(DW1000_EUI, DW1000_NO_SUB, reversed, 8);
}

void dw1000_ll_get_eui_bytes(uint8_t eui[8])
{
    dw1000_ll_read(DW1000_EUI, DW1000_NO_SUB, eui, 8);
}

/* ---- RF configuration ---- */

void dw1000_ll_set_data_rate(uint8_t rate)
{
    uint8_t sfd_length;

    rate &= 0x03;
    g_txfctrl[1] &= 0x83;
    g_txfctrl[1] |= (uint8_t)((rate << 5) & 0xFF);

    if (rate == DW1000_RATE_110KBPS) {
        dw1000_ll_set_bit(g_syscfg, DW1000_LEN_SYS_CFG, DW1000_RXM110K_BIT, true);
    } else {
        dw1000_ll_set_bit(g_syscfg, DW1000_LEN_SYS_CFG, DW1000_RXM110K_BIT, false);
    }

    /* SFD mode and type (fixed per data rate, DW1000 user manual table) */
    if (rate == DW1000_RATE_6800KBPS) {
        dw1000_ll_set_bit(g_chanctrl, DW1000_LEN_CHAN_CTRL, DW1000_DWSFD_BIT, false);
        dw1000_ll_set_bit(g_chanctrl, DW1000_LEN_CHAN_CTRL, DW1000_TNSSFD_BIT, false);
        dw1000_ll_set_bit(g_chanctrl, DW1000_LEN_CHAN_CTRL, DW1000_RNSSFD_BIT, false);
    } else if (rate == DW1000_RATE_850KBPS) {
        dw1000_ll_set_bit(g_chanctrl, DW1000_LEN_CHAN_CTRL, DW1000_DWSFD_BIT, true);
        dw1000_ll_set_bit(g_chanctrl, DW1000_LEN_CHAN_CTRL, DW1000_TNSSFD_BIT, true);
        dw1000_ll_set_bit(g_chanctrl, DW1000_LEN_CHAN_CTRL, DW1000_RNSSFD_BIT, true);
    } else {
        dw1000_ll_set_bit(g_chanctrl, DW1000_LEN_CHAN_CTRL, DW1000_DWSFD_BIT, true);
        dw1000_ll_set_bit(g_chanctrl, DW1000_LEN_CHAN_CTRL, DW1000_TNSSFD_BIT, false);
        dw1000_ll_set_bit(g_chanctrl, DW1000_LEN_CHAN_CTRL, DW1000_RNSSFD_BIT, false);
    }

    if (rate == DW1000_RATE_6800KBPS) {
        sfd_length = 0x08;
    } else if (rate == DW1000_RATE_850KBPS) {
        sfd_length = 0x10;
    } else {
        sfd_length = 0x40;
    }
    dw1000_ll_write(DW1000_USR_SFD, DW1000_SFD_LENGTH_SUB, &sfd_length, 1);

    g_data_rate = rate;
}

void dw1000_ll_set_pulse_frequency(uint8_t freq)
{
    freq &= 0x03;
    g_txfctrl[2] &= 0xFC;
    g_txfctrl[2] |= (uint8_t)(freq & 0xFF);
    g_chanctrl[2] &= 0xF3;
    g_chanctrl[2] |= (uint8_t)((freq << 2) & 0xFF);
    g_pulse_frequency = freq;
}

void dw1000_ll_set_preamble_length(uint8_t prealen)
{
    prealen &= 0x0F;
    g_txfctrl[2] &= 0xC3;
    g_txfctrl[2] |= (uint8_t)((prealen << 2) & 0xFF);

    if (prealen == DW1000_PREAMBLE_LEN_64 || prealen == DW1000_PREAMBLE_LEN_128) {
        g_pac_size = DW1000_PAC_SIZE_8;
    } else if (prealen == DW1000_PREAMBLE_LEN_256 || prealen == DW1000_PREAMBLE_LEN_512) {
        g_pac_size = DW1000_PAC_SIZE_16;
    } else if (prealen == DW1000_PREAMBLE_LEN_1024) {
        g_pac_size = DW1000_PAC_SIZE_32;
    } else {
        g_pac_size = DW1000_PAC_SIZE_64;
    }
    g_preamble_length = prealen;
}

void dw1000_ll_set_preamble_code(uint8_t preacode)
{
    preacode &= 0x1F;
    g_chanctrl[2] &= 0x3F;
    g_chanctrl[2] |= (uint8_t)((preacode << 6) & 0xFF);
    g_chanctrl[3] = (uint8_t)((((preacode >> 2) & 0x07) | (preacode << 3)) & 0xFF);
    g_preamble_code = preacode;
}

void dw1000_ll_set_channel(uint8_t channel)
{
    channel &= 0x0F;
    g_chanctrl[0] = (uint8_t)((channel | (channel << 4)) & 0xFF);
    g_channel = channel;

    /* pick a default preamble code for the channel (user manual 10.5, table 61) */
    if (g_channel == DW1000_CHANNEL_1) {
        if (g_pulse_frequency == DW1000_PRF_16MHZ) {
            dw1000_ll_set_preamble_code(DW1000_PREAMBLE_CODE_16MHZ_2);
        } else {
            dw1000_ll_set_preamble_code(DW1000_PREAMBLE_CODE_64MHZ_10);
        }
    } else if (g_channel == DW1000_CHANNEL_3) {
        if (g_pulse_frequency == DW1000_PRF_16MHZ) {
            dw1000_ll_set_preamble_code(DW1000_PREAMBLE_CODE_16MHZ_6);
        } else {
            dw1000_ll_set_preamble_code(DW1000_PREAMBLE_CODE_64MHZ_10);
        }
    } else if (g_channel == DW1000_CHANNEL_4 || g_channel == DW1000_CHANNEL_7) {
        if (g_pulse_frequency == DW1000_PRF_16MHZ) {
            dw1000_ll_set_preamble_code(DW1000_PREAMBLE_CODE_16MHZ_8);
        } else {
            dw1000_ll_set_preamble_code(DW1000_PREAMBLE_CODE_64MHZ_18);
        }
    } else if (g_pulse_frequency == DW1000_PRF_16MHZ) {
        dw1000_ll_set_preamble_code(DW1000_PREAMBLE_CODE_16MHZ_4);
    } else {
        dw1000_ll_set_preamble_code(DW1000_PREAMBLE_CODE_64MHZ_10);
    }
}

void dw1000_ll_enable_mode(const uint8_t mode[3])
{
    dw1000_ll_set_data_rate(mode[0]);
    dw1000_ll_set_pulse_frequency(mode[1]);
    dw1000_ll_set_preamble_length(mode[2]);
}

void dw1000_ll_use_smart_power(int enabled)
{
    g_smart_power = (enabled != 0);
    dw1000_ll_set_bit(g_syscfg, DW1000_LEN_SYS_CFG, DW1000_DIS_STXP_BIT, !g_smart_power);
}

void dw1000_ll_set_defaults(void)
{
    if (g_device_mode != DW1000_MODE_IDLE) {
        return;
    }
    dw1000_ll_use_extended_frame_length(false);
    dw1000_ll_use_smart_power(false);
    dw1000_ll_suppress_frame_check(false);
    dw1000_ll_apply_frame_filter(0, 0);   /* frame filtering off by default */
    dw1000_ll_interrupt_on_sent(true);
    dw1000_ll_interrupt_on_received(true);
    dw1000_ll_interrupt_on_receive_failed(true);
    dw1000_ll_interrupt_on_receive_timestamp_available(false);
    dw1000_ll_interrupt_on_automatic_ack_trigger(true);
    dw1000_ll_set_receiver_auto_reenable(true);
    dw1000_ll_enable_mode(DW1000_LL_DEFAULT_MODE);
    dw1000_ll_set_channel(DW1000_CHANNEL_5);
    if (g_pulse_frequency == DW1000_PRF_16MHZ) {
        dw1000_ll_set_preamble_code(DW1000_PREAMBLE_CODE_16MHZ_4);
    } else {
        dw1000_ll_set_preamble_code(DW1000_PREAMBLE_CODE_64MHZ_10);
    }
}

/* ---- antenna delay ---- */

void dw1000_ll_set_antenna_delay(uint16_t value)
{
    g_antenna_delay = value;
    g_antenna_calibrated = true;
}

uint16_t dw1000_ll_get_antenna_delay(void)
{
    return (uint16_t)g_antenna_delay;
}

/* ---- transceiver state ---- */

void dw1000_ll_idle(void)
{
    uint8_t sysctrl[DW1000_LEN_SYS_CTRL] = {0};

    dw1000_ll_set_bit(sysctrl, DW1000_LEN_SYS_CTRL, DW1000_TRXOFF_BIT, true);
    g_device_mode = DW1000_MODE_IDLE;
    dw1000_ll_write(DW1000_SYS_CTRL, DW1000_NO_SUB, sysctrl, DW1000_LEN_SYS_CTRL);
}

void dw1000_ll_new_configuration(void)
{
    dw1000_ll_idle();
    dw1000_ll_read(DW1000_PANADR, DW1000_NO_SUB, g_network_addr, DW1000_LEN_PANADR);
    dw1000_ll_read_syscfg();
    dw1000_ll_read(DW1000_CHAN_CTRL, DW1000_NO_SUB, g_chanctrl, DW1000_LEN_CHAN_CTRL);
    dw1000_ll_read(DW1000_TX_FCTRL, DW1000_NO_SUB, g_txfctrl, DW1000_LEN_TX_FCTRL);
    dw1000_ll_read(DW1000_SYS_MASK, DW1000_NO_SUB, g_sysmask, DW1000_LEN_SYS_MASK);
}

void dw1000_ll_receive_permanently(int enabled)
{
    g_permanent_receive = (enabled != 0);
    if (g_permanent_receive) {
        dw1000_ll_set_receiver_auto_reenable(true);
        dw1000_ll_write_syscfg();
    }
}

/* ---- tuning tables (DW1000 user manual; ported from DW1000.cpp) ---- */

static void dw1000_ll_tune(void)
{
    uint8_t agctune1[2],  agctune2[4],  agctune3[2];
    uint8_t drxtune0b[2], drxtune1a[2], drxtune1b[2], drxtune2[4], drxtune4H[2];
    uint8_t ldecfg1[1],   ldecfg2[2],   lderepc[2];
    uint8_t txpower[4];
    uint8_t rfrxctrlh[1], rftxctrl[4];
    uint8_t tcpgdelay[1];
    uint8_t fspllcfg[4],  fsplltune[1], fsxtalt[1];
    uint8_t buf_otp[4];

    /* AGC_TUNE1 */
    if (g_pulse_frequency == DW1000_PRF_16MHZ) {
        dw1000_ll_write_value(agctune1, 0x8870, DW1000_LEN_AGC_TUNE1);
    } else if (g_pulse_frequency == DW1000_PRF_64MHZ) {
        dw1000_ll_write_value(agctune1, 0x889B, DW1000_LEN_AGC_TUNE1);
    }
    dw1000_ll_write_value(agctune2, 0x2502A907u, DW1000_LEN_AGC_TUNE2);
    dw1000_ll_write_value(agctune3, 0x0035, DW1000_LEN_AGC_TUNE3);

    /* DRX_TUNE0b */
    if (g_data_rate == DW1000_RATE_110KBPS) {
        dw1000_ll_write_value(drxtune0b, 0x0016, DW1000_LEN_DRX_TUNE0b);
    } else if (g_data_rate == DW1000_RATE_850KBPS) {
        dw1000_ll_write_value(drxtune0b, 0x0006, DW1000_LEN_DRX_TUNE0b);
    } else if (g_data_rate == DW1000_RATE_6800KBPS) {
        dw1000_ll_write_value(drxtune0b, 0x0001, DW1000_LEN_DRX_TUNE0b);
    }

    /* DRX_TUNE1a */
    if (g_pulse_frequency == DW1000_PRF_16MHZ) {
        dw1000_ll_write_value(drxtune1a, 0x0087, DW1000_LEN_DRX_TUNE1a);
    } else if (g_pulse_frequency == DW1000_PRF_64MHZ) {
        dw1000_ll_write_value(drxtune1a, 0x008D, DW1000_LEN_DRX_TUNE1a);
    }

    /* DRX_TUNE1b */
    if (g_preamble_length == DW1000_PREAMBLE_LEN_1536 ||
        g_preamble_length == DW1000_PREAMBLE_LEN_2048 ||
        g_preamble_length == DW1000_PREAMBLE_LEN_4096) {
        if (g_data_rate == DW1000_RATE_110KBPS) {
            dw1000_ll_write_value(drxtune1b, 0x0064, DW1000_LEN_DRX_TUNE1b);
        }
    } else if (g_preamble_length != DW1000_PREAMBLE_LEN_64) {
        if (g_data_rate == DW1000_RATE_850KBPS || g_data_rate == DW1000_RATE_6800KBPS) {
            dw1000_ll_write_value(drxtune1b, 0x0020, DW1000_LEN_DRX_TUNE1b);
        }
    } else {
        if (g_data_rate == DW1000_RATE_6800KBPS) {
            dw1000_ll_write_value(drxtune1b, 0x0010, DW1000_LEN_DRX_TUNE1b);
        }
    }

    /* DRX_TUNE2 */
    if (g_pac_size == DW1000_PAC_SIZE_8) {
        if (g_pulse_frequency == DW1000_PRF_16MHZ) {
            dw1000_ll_write_value(drxtune2, 0x311A002Du, DW1000_LEN_DRX_TUNE2);
        } else if (g_pulse_frequency == DW1000_PRF_64MHZ) {
            dw1000_ll_write_value(drxtune2, 0x313B006Bu, DW1000_LEN_DRX_TUNE2);
        }
    } else if (g_pac_size == DW1000_PAC_SIZE_16) {
        if (g_pulse_frequency == DW1000_PRF_16MHZ) {
            dw1000_ll_write_value(drxtune2, 0x331A0052u, DW1000_LEN_DRX_TUNE2);
        } else if (g_pulse_frequency == DW1000_PRF_64MHZ) {
            dw1000_ll_write_value(drxtune2, 0x333B00BEu, DW1000_LEN_DRX_TUNE2);
        }
    } else if (g_pac_size == DW1000_PAC_SIZE_32) {
        if (g_pulse_frequency == DW1000_PRF_16MHZ) {
            dw1000_ll_write_value(drxtune2, 0x351A009Au, DW1000_LEN_DRX_TUNE2);
        } else if (g_pulse_frequency == DW1000_PRF_64MHZ) {
            dw1000_ll_write_value(drxtune2, 0x353B015Eu, DW1000_LEN_DRX_TUNE2);
        }
    } else if (g_pac_size == DW1000_PAC_SIZE_64) {
        if (g_pulse_frequency == DW1000_PRF_16MHZ) {
            dw1000_ll_write_value(drxtune2, 0x371A011Du, DW1000_LEN_DRX_TUNE2);
        } else if (g_pulse_frequency == DW1000_PRF_64MHZ) {
            dw1000_ll_write_value(drxtune2, 0x373B0296u, DW1000_LEN_DRX_TUNE2);
        }
    }

    /* DRX_TUNE4H */
    if (g_preamble_length == DW1000_PREAMBLE_LEN_64) {
        dw1000_ll_write_value(drxtune4H, 0x0010, DW1000_LEN_DRX_TUNE4H);
    } else {
        dw1000_ll_write_value(drxtune4H, 0x0028, DW1000_LEN_DRX_TUNE4H);
    }

    /* RF_RXCTRLH */
    if (g_channel != DW1000_CHANNEL_4 && g_channel != DW1000_CHANNEL_7) {
        dw1000_ll_write_value(rfrxctrlh, 0xD8, DW1000_LEN_RF_RXCTRLH);
    } else {
        dw1000_ll_write_value(rfrxctrlh, 0xBC, DW1000_LEN_RF_RXCTRLH);
    }

    /* RF_TXCTRL */
    if (g_channel == DW1000_CHANNEL_1) {
        dw1000_ll_write_value(rftxctrl, 0x00005C40u, DW1000_LEN_RF_TXCTRL);
    } else if (g_channel == DW1000_CHANNEL_2) {
        dw1000_ll_write_value(rftxctrl, 0x00045CA0u, DW1000_LEN_RF_TXCTRL);
    } else if (g_channel == DW1000_CHANNEL_3) {
        dw1000_ll_write_value(rftxctrl, 0x00086CC0u, DW1000_LEN_RF_TXCTRL);
    } else if (g_channel == DW1000_CHANNEL_4) {
        dw1000_ll_write_value(rftxctrl, 0x00045C80u, DW1000_LEN_RF_TXCTRL);
    } else if (g_channel == DW1000_CHANNEL_5) {
        dw1000_ll_write_value(rftxctrl, 0x001E3FE0u, DW1000_LEN_RF_TXCTRL);
    } else if (g_channel == DW1000_CHANNEL_7) {
        dw1000_ll_write_value(rftxctrl, 0x001E7DE0u, DW1000_LEN_RF_TXCTRL);
    }

    /* TC_PGDELAY */
    if (g_channel == DW1000_CHANNEL_1) {
        dw1000_ll_write_value(tcpgdelay, 0xC9, DW1000_LEN_TC_PGDELAY);
    } else if (g_channel == DW1000_CHANNEL_2) {
        dw1000_ll_write_value(tcpgdelay, 0xC2, DW1000_LEN_TC_PGDELAY);
    } else if (g_channel == DW1000_CHANNEL_3) {
        dw1000_ll_write_value(tcpgdelay, 0xC5, DW1000_LEN_TC_PGDELAY);
    } else if (g_channel == DW1000_CHANNEL_4) {
        dw1000_ll_write_value(tcpgdelay, 0x95, DW1000_LEN_TC_PGDELAY);
    } else if (g_channel == DW1000_CHANNEL_5) {
        dw1000_ll_write_value(tcpgdelay, 0xC0, DW1000_LEN_TC_PGDELAY);
    } else if (g_channel == DW1000_CHANNEL_7) {
        dw1000_ll_write_value(tcpgdelay, 0x93, DW1000_LEN_TC_PGDELAY);
    }

    /* FS_PLLCFG / FS_PLLTUNE */
    if (g_channel == DW1000_CHANNEL_1) {
        dw1000_ll_write_value(fspllcfg, 0x09000407u, DW1000_LEN_FS_PLLCFG);
        dw1000_ll_write_value(fsplltune, 0x1E, DW1000_LEN_FS_PLLTUNE);
    } else if (g_channel == DW1000_CHANNEL_2 || g_channel == DW1000_CHANNEL_4) {
        dw1000_ll_write_value(fspllcfg, 0x08400508u, DW1000_LEN_FS_PLLCFG);
        dw1000_ll_write_value(fsplltune, 0x26, DW1000_LEN_FS_PLLTUNE);
    } else if (g_channel == DW1000_CHANNEL_3) {
        dw1000_ll_write_value(fspllcfg, 0x08401009u, DW1000_LEN_FS_PLLCFG);
        dw1000_ll_write_value(fsplltune, 0x56, DW1000_LEN_FS_PLLTUNE);
    } else if (g_channel == DW1000_CHANNEL_5 || g_channel == DW1000_CHANNEL_7) {
        dw1000_ll_write_value(fspllcfg, 0x0800041Du, DW1000_LEN_FS_PLLCFG);
        dw1000_ll_write_value(fsplltune, 0xBE, DW1000_LEN_FS_PLLTUNE);
    }

    /* LDE_CFG1 / LDE_CFG2 */
    dw1000_ll_write_value(ldecfg1, 0xD, DW1000_LEN_LDE_CFG1);
    if (g_pulse_frequency == DW1000_PRF_16MHZ) {
        dw1000_ll_write_value(ldecfg2, 0x1607, DW1000_LEN_LDE_CFG2);
    } else if (g_pulse_frequency == DW1000_PRF_64MHZ) {
        dw1000_ll_write_value(ldecfg2, 0x0607, DW1000_LEN_LDE_CFG2);
    }

    /* LDE_REPC */
    {
        uint16_t repc = 0;
        if (g_preamble_code == DW1000_PREAMBLE_CODE_16MHZ_1 ||
            g_preamble_code == DW1000_PREAMBLE_CODE_16MHZ_2) {
            repc = 0x5998;
        } else if (g_preamble_code == DW1000_PREAMBLE_CODE_16MHZ_3 ||
                   g_preamble_code == DW1000_PREAMBLE_CODE_16MHZ_8) {
            repc = 0x51EA;
        } else if (g_preamble_code == DW1000_PREAMBLE_CODE_16MHZ_4) {
            repc = 0x428E;
        } else if (g_preamble_code == DW1000_PREAMBLE_CODE_16MHZ_5) {
            repc = 0x451E;
        } else if (g_preamble_code == DW1000_PREAMBLE_CODE_16MHZ_6) {
            repc = 0x2E14;
        } else if (g_preamble_code == DW1000_PREAMBLE_CODE_16MHZ_7) {
            repc = 0x8000;
        } else if (g_preamble_code == DW1000_PREAMBLE_CODE_64MHZ_9) {
            repc = 0x28F4;
        } else if (g_preamble_code == DW1000_PREAMBLE_CODE_64MHZ_10 ||
                   g_preamble_code == DW1000_PREAMBLE_CODE_64MHZ_17) {
            repc = 0x3332;
        } else if (g_preamble_code == DW1000_PREAMBLE_CODE_64MHZ_11) {
            repc = 0x3AE0;
        } else if (g_preamble_code == DW1000_PREAMBLE_CODE_64MHZ_12) {
            repc = 0x3D70;
        } else if (g_preamble_code == DW1000_PREAMBLE_CODE_64MHZ_18 ||
                   g_preamble_code == DW1000_PREAMBLE_CODE_64MHZ_19) {
            repc = 0x35C2;
        } else if (g_preamble_code == DW1000_PREAMBLE_CODE_64MHZ_20) {
            repc = 0x47AE;
        }
        if (g_data_rate == DW1000_RATE_110KBPS) {
            repc >>= 3;
        }
        dw1000_ll_write_value(lderepc, repc, DW1000_LEN_LDE_REPC);
    }

    /* TX_POWER (smart TX power control) */
    if (g_channel == DW1000_CHANNEL_1 || g_channel == DW1000_CHANNEL_2) {
        if (g_pulse_frequency == DW1000_PRF_16MHZ) {
            dw1000_ll_write_value(txpower, g_smart_power ? 0x15355575u : 0x75757575u, DW1000_LEN_TX_POWER);
        } else if (g_pulse_frequency == DW1000_PRF_64MHZ) {
            dw1000_ll_write_value(txpower, g_smart_power ? 0x07274767u : 0x67676767u, DW1000_LEN_TX_POWER);
        }
    } else if (g_channel == DW1000_CHANNEL_3) {
        if (g_pulse_frequency == DW1000_PRF_16MHZ) {
            dw1000_ll_write_value(txpower, g_smart_power ? 0x0F2F4F6Fu : 0x6F6F6F6Fu, DW1000_LEN_TX_POWER);
        } else if (g_pulse_frequency == DW1000_PRF_64MHZ) {
            dw1000_ll_write_value(txpower, g_smart_power ? 0x2B4B6B8Bu : 0x8B8B8B8Bu, DW1000_LEN_TX_POWER);
        }
    } else if (g_channel == DW1000_CHANNEL_4) {
        if (g_pulse_frequency == DW1000_PRF_16MHZ) {
            dw1000_ll_write_value(txpower, g_smart_power ? 0x1F1F3F5Fu : 0x5F5F5F5Fu, DW1000_LEN_TX_POWER);
        } else if (g_pulse_frequency == DW1000_PRF_64MHZ) {
            dw1000_ll_write_value(txpower, g_smart_power ? 0x3A5A7A9Au : 0x9A9A9A9Au, DW1000_LEN_TX_POWER);
        }
    } else if (g_channel == DW1000_CHANNEL_5) {
        if (g_pulse_frequency == DW1000_PRF_16MHZ) {
            dw1000_ll_write_value(txpower, g_smart_power ? 0x0E082848u : 0x48484848u, DW1000_LEN_TX_POWER);
        } else if (g_pulse_frequency == DW1000_PRF_64MHZ) {
            dw1000_ll_write_value(txpower, g_smart_power ? 0x25456585u : 0x85858585u, DW1000_LEN_TX_POWER);
        }
    } else if (g_channel == DW1000_CHANNEL_7) {
        if (g_pulse_frequency == DW1000_PRF_16MHZ) {
            dw1000_ll_write_value(txpower, g_smart_power ? 0x32527292u : 0x92929292u, DW1000_LEN_TX_POWER);
        } else if (g_pulse_frequency == DW1000_PRF_64MHZ) {
            dw1000_ll_write_value(txpower, g_smart_power ? 0x5171B1D1u : 0xD1D1D1D1u, DW1000_LEN_TX_POWER);
        }
    }

    /* crystal trim from OTP (mid-range 0x10 if no value stored) */
    dw1000_ll_read_otp(0x01E, buf_otp);
    if (buf_otp[0] == 0) {
        dw1000_ll_write_value(fsxtalt, (uint8_t)((0x10 & 0x1F) | 0x60), DW1000_LEN_FS_XTALT);
    } else {
        dw1000_ll_write_value(fsxtalt, (uint8_t)((buf_otp[0] & 0x1F) | 0x60), DW1000_LEN_FS_XTALT);
    }

    /* write everything to the chip */
    dw1000_ll_write(DW1000_AGC_TUNE, DW1000_AGC_TUNE1_SUB, agctune1,  DW1000_LEN_AGC_TUNE1);
    dw1000_ll_write(DW1000_AGC_TUNE, DW1000_AGC_TUNE2_SUB, agctune2,  DW1000_LEN_AGC_TUNE2);
    dw1000_ll_write(DW1000_AGC_TUNE, DW1000_AGC_TUNE3_SUB, agctune3,  DW1000_LEN_AGC_TUNE3);
    dw1000_ll_write(DW1000_DRX_TUNE, DW1000_DRX_TUNE0b_SUB, drxtune0b, DW1000_LEN_DRX_TUNE0b);
    dw1000_ll_write(DW1000_DRX_TUNE, DW1000_DRX_TUNE1a_SUB, drxtune1a, DW1000_LEN_DRX_TUNE1a);
    dw1000_ll_write(DW1000_DRX_TUNE, DW1000_DRX_TUNE1b_SUB, drxtune1b, DW1000_LEN_DRX_TUNE1b);
    dw1000_ll_write(DW1000_DRX_TUNE, DW1000_DRX_TUNE2_SUB,  drxtune2,  DW1000_LEN_DRX_TUNE2);
    dw1000_ll_write(DW1000_DRX_TUNE, DW1000_DRX_TUNE4H_SUB, drxtune4H, DW1000_LEN_DRX_TUNE4H);
    dw1000_ll_write(DW1000_LDE_IF,   DW1000_LDE_CFG1_SUB,   ldecfg1,   DW1000_LEN_LDE_CFG1);
    dw1000_ll_write(DW1000_LDE_IF,   DW1000_LDE_CFG2_SUB,   ldecfg2,   DW1000_LEN_LDE_CFG2);
    dw1000_ll_write(DW1000_LDE_IF,   DW1000_LDE_REPC_SUB,   lderepc,   DW1000_LEN_LDE_REPC);
    dw1000_ll_write(DW1000_TX_POWER, DW1000_NO_SUB,         txpower,   DW1000_LEN_TX_POWER);
    dw1000_ll_write(DW1000_RF_CONF,  DW1000_RF_RXCTRLH_SUB, rfrxctrlh, DW1000_LEN_RF_RXCTRLH);
    dw1000_ll_write(DW1000_RF_CONF,  DW1000_RF_TXCTRL_SUB,  rftxctrl,  DW1000_LEN_RF_TXCTRL);
    dw1000_ll_write(DW1000_TX_CAL,   DW1000_TC_PGDELAY_SUB, tcpgdelay, DW1000_LEN_TC_PGDELAY);
    dw1000_ll_write(DW1000_FS_CTRL,  DW1000_FS_PLLTUNE_SUB, fsplltune, DW1000_LEN_FS_PLLTUNE);
    dw1000_ll_write(DW1000_FS_CTRL,  DW1000_FS_PLLCFG_SUB,  fspllcfg,  DW1000_LEN_FS_PLLCFG);
    dw1000_ll_write(DW1000_FS_CTRL,  DW1000_FS_XTALT_SUB,   fsxtalt,   DW1000_LEN_FS_XTALT);
}

void dw1000_ll_commit_configuration(void)
{
    uint8_t antd[DW1000_LEN_TX_ANTD] = {0};
    int i;

    dw1000_ll_write(DW1000_PANADR, DW1000_NO_SUB, g_network_addr, DW1000_LEN_PANADR);
    dw1000_ll_write_syscfg();
    dw1000_ll_write(DW1000_CHAN_CTRL, DW1000_NO_SUB, g_chanctrl, DW1000_LEN_CHAN_CTRL);
    dw1000_ll_write(DW1000_TX_FCTRL, DW1000_NO_SUB, g_txfctrl, DW1000_LEN_TX_FCTRL);
    dw1000_ll_write(DW1000_SYS_MASK, DW1000_NO_SUB, g_sysmask, DW1000_LEN_SYS_MASK);

    dw1000_ll_tune();

    /* antenna delay: default 16384 (one chip "compatibility" value) */
    if (g_antenna_delay == 0 && !g_antenna_calibrated) {
        g_antenna_delay = 16384;
        g_antenna_calibrated = true;
    }
    for (i = 0; i < DW1000_LEN_TX_ANTD; i++) {
        antd[i] = (uint8_t)((g_antenna_delay >> (i * 8)) & 0xFF);
    }
    dw1000_ll_write(DW1000_TX_ANTD, DW1000_NO_SUB, antd, DW1000_LEN_TX_ANTD);
    dw1000_ll_write(DW1000_LDE_IF, DW1000_LDE_RXANTD_SUB, antd, DW1000_LEN_LDE_RXANTD);
}

/* ---- current mode getters ---- */

uint8_t dw1000_ll_get_channel(void)          { return g_channel; }
uint8_t dw1000_ll_get_data_rate(void)        { return g_data_rate; }
uint8_t dw1000_ll_get_pulse_frequency(void)  { return g_pulse_frequency; }
uint8_t dw1000_ll_get_preamble_length(void)  { return g_preamble_length; }
uint8_t dw1000_ll_get_preamble_code(void)    { return g_preamble_code; }

/* ==========================================================================
 * Power-up defaults, LDE microcode load and RX/TX (pure-C port of DW1000.cpp).
 * ======================================================================== */

/* cached copy of SYS_STATUS, refreshed on demand by the status functions */
static uint8_t g_sysstatus[DW1000_LEN_SYS_STATUS] = {0};

static bool dw1000_ll_get_bit(const uint8_t *data, uint16_t n, uint16_t bit)
{
    uint16_t idx = bit / 8;
    if (idx >= n) {
        return false;
    }
    return (data[idx] >> (bit % 8)) & 0x01;
}

static esp_err_t dw1000_ll_read_sysstatus(void)
{
    return dw1000_ll_read(DW1000_SYS_STATUS, DW1000_NO_SUB, g_sysstatus, DW1000_LEN_SYS_STATUS);
}

static void dw1000_ll_write_transmit_frame_control(void)
{
    dw1000_ll_write(DW1000_TX_FCTRL, DW1000_NO_SUB, g_txfctrl, DW1000_LEN_TX_FCTRL);
}

/* latched RX/TX status bits are cleared by writing 1 to them */
static void dw1000_ll_clear_receive_status(void)
{
    memset(g_sysstatus, 0, sizeof(g_sysstatus));
    dw1000_ll_set_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_RXDFR_BIT, true);
    dw1000_ll_set_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_LDEDONE_BIT, true);
    dw1000_ll_set_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_LDEERR_BIT, true);
    dw1000_ll_set_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_RXPHE_BIT, true);
    dw1000_ll_set_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_RXFCE_BIT, true);
    dw1000_ll_set_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_RXFCG_BIT, true);
    dw1000_ll_set_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_RXRFSL_BIT, true);
    dw1000_ll_write(DW1000_SYS_STATUS, DW1000_NO_SUB, g_sysstatus, DW1000_LEN_SYS_STATUS);
}

static void dw1000_ll_clear_transmit_status(void)
{
    memset(g_sysstatus, 0, sizeof(g_sysstatus));
    dw1000_ll_set_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_TXFRB_BIT, true);
    dw1000_ll_set_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_TXPRS_BIT, true);
    dw1000_ll_set_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_TXPHS_BIT, true);
    dw1000_ll_set_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_TXFRS_BIT, true);
    dw1000_ll_write(DW1000_SYS_STATUS, DW1000_NO_SUB, g_sysstatus, DW1000_LEN_SYS_STATUS);
}

/* ---- power-up defaults + LDE (port of select()) ---- */

void dw1000_ll_init_defaults(void)
{
    uint8_t net[4];
    uint8_t syscfg[4];
    uint8_t zero[4];

    /* default PAN id and short address: 0xFFFF */
    memset(net, 0xFF, sizeof(net));
    dw1000_ll_write(DW1000_PANADR, DW1000_NO_SUB, net, sizeof(net));

    /* default SYS_CFG: disable double-receive buffer, IRQ active high */
    memset(syscfg, 0, sizeof(syscfg));
    dw1000_ll_set_bit(syscfg, sizeof(syscfg), DW1000_DIS_DRXB_BIT, true);
    dw1000_ll_set_bit(syscfg, sizeof(syscfg), DW1000_HIRQ_POL_BIT, true);
    dw1000_ll_write(DW1000_SYS_CFG, DW1000_NO_SUB, syscfg, sizeof(syscfg));

    /* no interrupts enabled by default */
    memset(zero, 0, sizeof(zero));
    dw1000_ll_write(DW1000_SYS_MASK, DW1000_NO_SUB, zero, sizeof(zero));

    /* remember the production-test Vbat / temperature readings from OTP */
    {
        uint8_t otp[4];
        dw1000_ll_read_otp(0x008, otp);
        g_vmeas3v3 = otp[0];
        dw1000_ll_read_otp(0x009, otp);
        g_tmeas23C = otp[0];
    }
}

void dw1000_ll_manage_lde(void)
{
    uint8_t pmscctrl0[DW1000_LEN_PMSC_CTRL0] = {0};
    uint8_t otpctrl[DW1000_LEN_OTP_CTRL]     = {0};

    dw1000_ll_read(DW1000_PMSC, DW1000_PMSC_CTRL0_SUB, pmscctrl0, sizeof(pmscctrl0));
    dw1000_ll_read(DW1000_OTP_IF, DW1000_OTP_CTRL_SUB, otpctrl, sizeof(otpctrl));

    pmscctrl0[0] = 0x01;
    pmscctrl0[1] = 0x03;
    otpctrl[0]   = 0x00;
    otpctrl[1]   = 0x80;

    dw1000_ll_write(DW1000_PMSC, DW1000_PMSC_CTRL0_SUB, pmscctrl0, 2);
    dw1000_ll_write(DW1000_OTP_IF, DW1000_OTP_CTRL_SUB, otpctrl, 2);
    vTaskDelay(pdMS_TO_TICKS(5));

    pmscctrl0[0] = 0x00;
    pmscctrl0[1] &= 0x02;
    dw1000_ll_write(DW1000_PMSC, DW1000_PMSC_CTRL0_SUB, pmscctrl0, 2);
}

/* ---- RX / TX state ---- */

void dw1000_ll_new_receive(void)
{
    dw1000_ll_idle();
    dw1000_ll_clear_receive_status();
    g_device_mode = DW1000_MODE_RX;
}

void dw1000_ll_start_receive(void)
{
    uint8_t sysctrl[DW1000_LEN_SYS_CTRL] = {0};

    dw1000_ll_set_bit(sysctrl, sizeof(sysctrl), DW1000_SFCST_BIT, !g_frame_check);
    dw1000_ll_set_bit(sysctrl, sizeof(sysctrl), DW1000_RXENAB_BIT, true);
    dw1000_ll_write(DW1000_SYS_CTRL, DW1000_NO_SUB, sysctrl, sizeof(sysctrl));
}

void dw1000_ll_new_transmit(void)
{
    dw1000_ll_idle();
    dw1000_ll_clear_transmit_status();
    g_device_mode = DW1000_MODE_TX;
    ESP_LOGI("DW1000_LL", "DW1000_MODE_TX");
}

void dw1000_ll_start_transmit(void)
{
    uint8_t sysctrl[DW1000_LEN_SYS_CTRL] = {0};

    dw1000_ll_write_transmit_frame_control();
    dw1000_ll_set_bit(sysctrl, sizeof(sysctrl), DW1000_SFCST_BIT, !g_frame_check);
    if (g_tx_delayed) {
        dw1000_ll_set_bit(sysctrl, sizeof(sysctrl), DW1000_TXDLYS_BIT, true);
        g_tx_delayed = false;
    }
    dw1000_ll_set_bit(sysctrl, sizeof(sysctrl), DW1000_TXSTRT_BIT, true);
    dw1000_ll_write(DW1000_SYS_CTRL, DW1000_NO_SUB, sysctrl, sizeof(sysctrl));

    /* RX is deliberately NOT re-enabled here. The application TX task owns the
       TX/RX transition and re-arms RX after TXFRS, so RXENAB is never written
       while the transmitter is still busy and the transition lives in one
       place. g_permanent_receive still gates the IRQ fail/timeout re-arm. */
    g_device_mode = DW1000_MODE_TX;
    ESP_LOGI("DW1000_LL", "DW1000_MODE_TX");
}

/* ---- data buffer ---- */

void dw1000_ll_set_data(const uint8_t *data, uint16_t n)
{
    uint16_t frame_len = n;
    uint8_t *buf;

    if (g_frame_check) {
        frame_len += 2;   /* two bytes CRC-16 */
    }
    if (frame_len > DW1000_LEN_EXT_UWB_FRAMES) {
        return;
    }
    if (frame_len > DW1000_LEN_UWB_FRAMES && !g_extended_frame_length) {
        return;
    }

    buf = (uint8_t *)malloc(frame_len);
    if (buf == NULL) {
        ESP_LOGE(TAG, "set_data: malloc failed (n=%u)", (unsigned)n);
        return;
    }
    memcpy(buf, data, n);
    memset(buf + n, 0, frame_len - n);   /* FCS bytes (the chip computes them) */
    dw1000_ll_write(DW1000_TX_BUFFER, DW1000_NO_SUB, buf, frame_len);

    /* remember the total frame length in TX_FCTRL */
    g_txfctrl[0] = (uint8_t)(frame_len & 0xFF);
    g_txfctrl[1] &= 0xE0;
    g_txfctrl[1] |= (uint8_t)((frame_len >> 8) & 0x03);

    free(buf);
}

uint16_t dw1000_ll_get_data_length(void)
{
    uint16_t len = 0;

    if (g_device_mode == DW1000_MODE_TX) {
        /* 10 bits of the TX frame control register */
        len = ((((uint16_t)g_txfctrl[1] << 8) | g_txfctrl[0]) & 0x03FF);
    } else if (g_device_mode == DW1000_MODE_RX) {
        /* 10 bits of the RX frame info register */
        uint8_t info[DW1000_LEN_RX_FINFO];
        dw1000_ll_read(DW1000_RX_FINFO, DW1000_NO_SUB, info, sizeof(info));
        len = ((((uint16_t)info[1] << 8) | info[0]) & 0x03FF);
    }
    if (g_frame_check && len > 2) {
        len -= 2;   /* exclude the two FCS bytes */
    }
    return len;
}

void dw1000_ll_get_data(uint8_t *data, uint16_t n)
{
    if (n == 0) {
        return;
    }
    dw1000_ll_read(DW1000_RX_BUFFER, DW1000_NO_SUB, data, n);
}

/* ---- status flags ---- */

bool dw1000_ll_is_transmit_done(void)
{
    dw1000_ll_read_sysstatus();
    return dw1000_ll_get_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_TXFRS_BIT);
}

bool dw1000_ll_is_receive_done(void)
{
    dw1000_ll_read_sysstatus();
    if (g_frame_check) {
        return dw1000_ll_get_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_RXFCG_BIT);
    }
    return dw1000_ll_get_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_RXDFR_BIT);
}

bool dw1000_ll_is_receive_failed(void)
{
    dw1000_ll_read_sysstatus();
    return dw1000_ll_get_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_LDEERR_BIT) ||
           dw1000_ll_get_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_RXFCE_BIT) ||
           dw1000_ll_get_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_RXPHE_BIT) ||
           dw1000_ll_get_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_RXRFSL_BIT);
}

bool dw1000_ll_is_receive_timeout(void)
{
    dw1000_ll_read_sysstatus();
    return dw1000_ll_get_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_RXRFTO_BIT) ||
           dw1000_ll_get_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_RXPTO_BIT) ||
           dw1000_ll_get_bit(g_sysstatus, sizeof(g_sysstatus), DW1000_RXSFDTO_BIT);
}

/* ============================ timestamps ============================ */

/* read a 40-bit little-endian timestamp register */
static uint64_t dw1000_ll_read_timestamp(uint8_t reg, uint16_t sub)
{
    uint8_t data[DW1000_LEN_STAMP];
    uint64_t ts = 0;
    int i;

    dw1000_ll_read(reg, sub, data, DW1000_LEN_STAMP);
    for (i = 0; i < DW1000_LEN_STAMP; i++) {
        ts |= (uint64_t)data[i] << (i * 8);
    }
    return ts;
}

uint64_t dw1000_ll_get_transmit_timestamp(void)
{
    return dw1000_ll_read_timestamp(DW1000_TX_TIME, DW1000_TX_STAMP_SUB);
}

uint64_t dw1000_ll_get_receive_timestamp(void)
{
    return dw1000_ll_read_timestamp(DW1000_RX_TIME, DW1000_RX_STAMP_SUB);
}

uint64_t dw1000_ll_get_system_timestamp(void)
{
    return dw1000_ll_read_timestamp(DW1000_SYS_TIME, DW1000_NO_SUB);
}

/* ============================ receive quality ============================ */

float dw1000_ll_get_receive_quality(void)
{
    uint8_t noise_b[2], f2_b[2];
    uint16_t noise, f2;

    dw1000_ll_read(DW1000_RX_FQUAL, DW1000_STD_NOISE_SUB, noise_b, 2);
    dw1000_ll_read(DW1000_RX_FQUAL, DW1000_FP_AMPL2_SUB, f2_b, 2);
    noise = (uint16_t)noise_b[0] | ((uint16_t)noise_b[1] << 8);
    f2    = (uint16_t)f2_b[0] | ((uint16_t)f2_b[1] << 8);
    return (float)f2 / (float)noise;
}

float dw1000_ll_get_first_path_power(void)
{
    uint8_t f1_b[2], f2_b[2], f3_b[2], info[DW1000_LEN_RX_FINFO];
    uint16_t f1, f2, f3, N;
    float A, corrFac, est;

    dw1000_ll_read(DW1000_RX_TIME, DW1000_FP_AMPL1_SUB, f1_b, 2);
    dw1000_ll_read(DW1000_RX_FQUAL, DW1000_FP_AMPL2_SUB, f2_b, 2);
    dw1000_ll_read(DW1000_RX_FQUAL, DW1000_FP_AMPL3_SUB, f3_b, 2);
    dw1000_ll_read(DW1000_RX_FINFO, DW1000_NO_SUB, info, sizeof(info));

    f1 = (uint16_t)f1_b[0] | ((uint16_t)f1_b[1] << 8);
    f2 = (uint16_t)f2_b[0] | ((uint16_t)f2_b[1] << 8);
    f3 = (uint16_t)f3_b[0] | ((uint16_t)f3_b[1] << 8);
    N  = (uint16_t)(((uint16_t)info[2] >> 4) & 0xFF) | ((uint16_t)info[3] << 4);

    if (g_pulse_frequency == DW1000_PRF_16MHZ) {
        A       = 113.77f;
        corrFac = 2.3334f;
    } else {
        A       = 121.74f;
        corrFac = 1.1667f;
    }
    est = 10.0f * log10f(((float)f1 * f1 + (float)f2 * f2 + (float)f3 * f3) / ((float)N * N)) - A;
    if (est > -88.0f) {
        est += (est + 88.0f) * corrFac;
    }
    return est;
}

float dw1000_ll_get_receive_power(void)
{
    uint8_t cir_b[2], info[DW1000_LEN_RX_FINFO];
    uint16_t C, N;
    float A, corrFac, est;

    dw1000_ll_read(DW1000_RX_FQUAL, DW1000_CIR_PWR_SUB, cir_b, 2);
    dw1000_ll_read(DW1000_RX_FINFO, DW1000_NO_SUB, info, sizeof(info));

    C = (uint16_t)cir_b[0] | ((uint16_t)cir_b[1] << 8);
    N = (uint16_t)(((uint16_t)info[2] >> 4) & 0xFF) | ((uint16_t)info[3] << 4);

    if (g_pulse_frequency == DW1000_PRF_16MHZ) {
        A       = 113.77f;
        corrFac = 2.3334f;
    } else {
        A       = 121.74f;
        corrFac = 1.1667f;
    }
    est = 10.0f * log10f(((float)C * 131072.0f) / ((float)N * N)) - A;
    if (est > -88.0f) {
        est += (est + 88.0f) * corrFac;
    }
    return est;
}

/* ============================ delayed TX ============================ */

void dw1000_ll_set_delay(uint64_t delay_us)
{
    uint8_t delay_bytes[DW1000_LEN_STAMP];
    uint64_t now = dw1000_ll_get_system_timestamp();
    /* convert us -> 15.65 ps ticks (TIME_RES_INV = 63897.6) */
    uint64_t future = now + (uint64_t)((float)delay_us * 63897.6f);
    int i;

    for (i = 0; i < DW1000_LEN_STAMP; i++) {
        delay_bytes[i] = (uint8_t)((future >> (i * 8)) & 0xFF);
    }
    delay_bytes[0] = 0;
    delay_bytes[1] &= 0xFE;

    dw1000_ll_write(DW1000_DX_TIME, DW1000_NO_SUB, delay_bytes, DW1000_LEN_STAMP);
    g_tx_delayed = true;
}

void dw1000_ll_start_transmit_at(uint64_t target_ticks)
{
    uint8_t sysctrl[DW1000_LEN_SYS_CTRL] = {0};
    uint8_t dx[DW1000_LEN_STAMP] = {0};
    int i;

    dw1000_ll_write_transmit_frame_control();

    /* write the absolute target time to the delay register */
    for (i = 0; i < DW1000_LEN_STAMP; i++) {
        dx[i] = (uint8_t)((target_ticks >> (i * 8)) & 0xFF);
    }
    dx[0] = 0;
    dx[1] &= 0xFE;
    dw1000_ll_write(DW1000_DX_TIME, DW1000_NO_SUB, dx, DW1000_LEN_STAMP);

    dw1000_ll_set_bit(sysctrl, sizeof(sysctrl), DW1000_SFCST_BIT, !g_frame_check);
    dw1000_ll_set_bit(sysctrl, sizeof(sysctrl), DW1000_TXDLYS_BIT, true);
    dw1000_ll_set_bit(sysctrl, sizeof(sysctrl), DW1000_TXSTRT_BIT, true);
    dw1000_ll_write(DW1000_SYS_CTRL, DW1000_NO_SUB, sysctrl, sizeof(sysctrl));

    /* See dw1000_ll_start_transmit(): the TX task re-arms RX after TXFRS. */
    g_device_mode = DW1000_MODE_TX;
}

/* ==========================================================================
 * Event callbacks, interrupt service and diagnostics (pure-C port).
 * ======================================================================== */

static dw1000_ll_handler_t g_on_error                    = NULL;
static dw1000_ll_handler_t g_on_sent                     = NULL;
static dw1000_ll_handler_t g_on_received                 = NULL;
static dw1000_ll_handler_t g_on_receive_failed           = NULL;
static dw1000_ll_handler_t g_on_receive_timeout          = NULL;
static dw1000_ll_handler_t g_on_receive_timestamp_available = NULL;

void dw1000_ll_attach_error_handler(dw1000_ll_handler_t cb)               { g_on_error = cb; }
void dw1000_ll_attach_sent_handler(dw1000_ll_handler_t cb)                { g_on_sent = cb; }
void dw1000_ll_attach_received_handler(dw1000_ll_handler_t cb)            { g_on_received = cb; }
void dw1000_ll_attach_receive_failed_handler(dw1000_ll_handler_t cb)      { g_on_receive_failed = cb; }
void dw1000_ll_attach_receive_timeout_handler(dw1000_ll_handler_t cb)     { g_on_receive_timeout = cb; }
void dw1000_ll_attach_receive_timestamp_available_handler(dw1000_ll_handler_t cb) { g_on_receive_timestamp_available = cb; }

/* ==========================================================================
 * Interrupt statistics (diagnostics).
 *
 * The interrupt handler only increments these counters - it never formats or
 * prints, so it does not block on the UART. A separate low-priority task
 * reports them via dw1000_ll_diag_dump(). All of them are written only by the
 * IRQ task.
 * ======================================================================== */
static volatile uint32_t s_irq_count         = 0;   /* handler runs with events       */
static volatile uint32_t s_irq_empty_count   = 0;   /* runs with nothing latched       */
static volatile uint32_t s_irq_pending_count = 0;   /* runs during which new events
                                                       arrived (serviced next run)    */
static volatile uint32_t s_irq_tx_count      = 0;   /* TXFRS events handled            */
static volatile uint32_t s_irq_rx_good_count = 0;   /* good frames handled (RXFCG/DFR) */
static volatile uint32_t s_irq_rx_fail_count = 0;   /* failed frames (CRC/sync/LDE)    */
static volatile uint32_t s_irq_rx_to_count   = 0;   /* RX timeouts                     */
static volatile uint32_t s_irq_spi_err_count = 0;   /* SYS_STATUS reads that failed    */

/* -------------------------------------------------------------------------
 * DIAGNOSTICS (no logging, counters only - they are printed by
 * dw1000_ll_diag_dump() from the low-priority task).
 * ---------------------------------------------------------------------- */
static volatile uint32_t s_run_evt          = 0;  /* run carried a MASKED event bit   */
static volatile uint32_t s_run_prog         = 0;  /* run carried only UNMASKED bits   */
static volatile uint32_t s_empty_hi         = 0;  /* empty status, IRQ pin still HIGH */
static volatile uint32_t s_empty_lo         = 0;  /* empty status, IRQ pin already LOW*/
static volatile uint32_t s_seen_ldedone     = 0;  /* runs with LDEDONE (LDE done)     */
static volatile uint32_t s_seen_pre         = 0;  /* runs with RXPRD  (preamble seen) */
static volatile uint32_t s_seen_sfd         = 0;  /* runs with RXSFDD (SFD seen)      */
static volatile uint32_t s_seen_phr         = 0;  /* runs with RXPHD  (PHY header)    */
static volatile uint32_t s_seen_good        = 0;  /* runs with RXDFR|RXFCG (frame ok) */
static volatile uint32_t s_seen_bad         = 0;  /* runs with PHE/FCE/RFSL/LDEERR    */
static volatile uint32_t s_seen_to          = 0;  /* runs with RXRFTO/RXPTO/RXSFDTO   */
static volatile uint32_t s_seen_pll         = 0;  /* runs with RFPLL_LL/CLKPLL_LL     */
static int s_irq_gpio                        = -1; /* IRQ pin, for the level check   */

/* Given by the GPIO ISR; the handler may give it to itself to re-run (see the
   end of dw1000_ll_handle_interrupt). */
static SemaphoreHandle_t s_irq_sem = NULL;

/* -------------------------------------------------------------------------
 * Very short, self-terminating IRQ trace.
 *
 * Logs at most DW1000_IRQ_TRACE_RUNS handler runs, ONE short line each, then
 * switches itself off. UART output inside the handler is expensive (~3 ms per
 * line at 115200 baud), so keep the budget tiny (e.g. 12) - it captures about
 * one exchange and afterwards the timing is normal again. Do not leave it big.
 *
 * Line:  irq <5 status bytes> <tag>
 *   tag 'T' = TXFRS         ('T'ransmit done)
 *       'G' = RXFCG/RXDFR   ('G'ood frame: decoded, CRC ok)
 *       'F' = RX failed (preamble header / CRC / sync loss / LDE error)
 *       'O' = RX timed out (frame wait / preamble / SFD timeout)
 *       'L' = LDEDONE only  (LDE ran: a signal was processed, no frame)
 *       'P' = clock/RF PLL loss of lock
 *       'e' = edge with NOTHING latched
 *       '-' = other/undefined bit only
 *
 * Set to 0 to compile the trace out completely.
 * ---------------------------------------------------------------------- */
#define DW1000_IRQ_TRACE_RUNS 0

#if DW1000_IRQ_TRACE_RUNS > 0
static int s_trace_left = DW1000_IRQ_TRACE_RUNS;
#endif

static bool dw1000_ll_status_bit(uint16_t bit)
{
    return dw1000_ll_get_bit(g_sysstatus, sizeof(g_sysstatus), bit);
}

/* Re-enable the receiver for permanent-RX operation WITHOUT touching
   SYS_STATUS. The IRQ handler has already cleared exactly the events it read
   (see dw1000_ll_handle_interrupt), so clearing again here with a fixed mask
   could wipe a frame that arrived in the meantime. Used by the IRQ task
   (good-frame / fail / timeout re-arm) and by the application TX task. */
void dw1000_ll_rearm_receive(void)
{
    dw1000_ll_idle();
    g_device_mode = DW1000_MODE_RX;
    dw1000_ll_start_receive();
    /* NO logging here: this now runs from the IRQ handler on every received
       frame, and UART output inside the handler makes the radio deaf. */
}

void dw1000_ll_handle_interrupt(void)
{
    uint8_t status_entry[DW1000_LEN_SYS_STATUS];
    uint8_t status_after[DW1000_LEN_SYS_STATUS];

    /* Serialise with the application TX task: it may be mid-transmit or about
       to change the mode. */
    dw1000_ll_radio_lock();

    /* A failed SPI read zero-fills the buffer, which would look EXACTLY like an
       "empty" edge (all zeros). Detect it, retry once, and never report it as
       empty - the frame would otherwise be silently thrown away. */
    esp_err_t rd = dw1000_ll_read_sysstatus();
    if (rd != ESP_OK) {
        rd = dw1000_ll_read_sysstatus();       /* single retry */
    }
    if (rd != ESP_OK) {
        s_irq_spi_err_count++;
        if (s_irq_spi_err_count <= DW1000_SPI_ERR_LOG_MAX) {
            ESP_LOGE(TAG, "IRQ: SYS_STATUS read failed (%s) - events left latched (irq_err #%u)",
                     esp_err_to_name(rd), (unsigned)s_irq_spi_err_count);
        }
        dw1000_ll_radio_unlock();
        return;
    }
    memcpy(status_entry, g_sysstatus, sizeof(status_entry));

#if DW1000_IRQ_TRACE_RUNS > 0
    /* Short, self-terminating trace: exactly what is latched at entry. */
    if (s_trace_left > 0) {
        char tag = '-';
        if ((status_entry[0] | status_entry[1] | status_entry[2] |
             status_entry[3] | status_entry[4]) == 0) {
            tag = 'e';                      /* edge, but nothing latched    */
        } else if (status_entry[0] & 0x80) {
            tag = 'T';                      /* TXFRS                        */
        } else if (status_entry[1] & 0x60) {
            tag = 'G';                      /* RXDFR | RXFCG (good frame)   */
        } else if ((status_entry[1] & 0x90) ||
                   (status_entry[2] & 0x05)) {
            tag = 'F';                      /* PHE/FCE, RFSL/LDEERR         */
        } else if ((status_entry[2] & 0x22) ||
                   (status_entry[3] & 0x04)) {
            tag = 'O';                      /* RXRFTO/RXPTO/RXSFDTO         */
        } else if (status_entry[1] & 0x04) {
            tag = 'L';                      /* LDEDONE: signal, no frame    */
        } else if (status_entry[3] & 0x03) {
            tag = 'P';                      /* CLKPLL/RFPLL loss of lock    */
        }
        s_trace_left--;
        ESP_LOGI(TAG, "irq %02X%02X%02X%02X%02X %c",
                 (unsigned)status_entry[0], (unsigned)status_entry[1],
                 (unsigned)status_entry[2], (unsigned)status_entry[3],
                 (unsigned)status_entry[4], tag);
    }
#endif

    /* ---- DIAGNOSTICS: classify this run (counters only, no logging) ----
       Count which event groups are latched right now, and whether a MASKED
       bit is present (only masked bits can raise the IRQ line). */
    {
        int i;
        uint8_t masked = 0;
        for (i = 0; i < DW1000_LEN_SYS_MASK; i++) {
            masked |= (uint8_t)(status_entry[i] & g_sysmask[i]);
        }
        if (status_entry[1] & 0x04) { s_seen_ldedone++; }  /* LDEDONE      */
        if (status_entry[1] & 0x01) { s_seen_pre++; }      /* RXPRD        */
        if (status_entry[1] & 0x02) { s_seen_sfd++; }      /* RXSFDD       */
        if (status_entry[1] & 0x08) { s_seen_phr++; }      /* RXPHD        */
        if (status_entry[1] & 0x60) { s_seen_good++; }     /* RXDFR|RXFCG  */
        if ((status_entry[1] & 0x90) ||
            (status_entry[2] & 0x05)) { s_seen_bad++; }    /* fail bits    */
        if ((status_entry[2] & 0x22) ||
            (status_entry[3] & 0x04)) { s_seen_to++; }     /* timeout bits */
        if (status_entry[3] & 0x03) { s_seen_pll++; }      /* PLL loss     */

        if ((status_entry[0] | status_entry[1] | status_entry[2] |
             status_entry[3] | status_entry[4]) == 0) {
            /* Nothing latched. The IRQ pin level separates the two causes:
                 pin still HIGH -> an event WAS cleared before we read it
                 pin already LOW -> nothing was ever latched: glitch / extra
                                    edge (not a DW1000 event at all) */
            s_irq_empty_count++;
            if (s_irq_gpio >= 0 && gpio_get_level((gpio_num_t)s_irq_gpio) != 0) {
                s_empty_hi++;
            } else {
                s_empty_lo++;
            }
            dw1000_ll_radio_unlock();
            return;
        }
        if (masked) {
            s_run_evt++;    /* a real (masked) event: this edge is legit    */
        } else {
            s_run_prog++;   /* only unmasked bits: they cannot raise the
                               line, so this edge came from somewhere else */
        }
    }
    s_irq_count++;

    /* Clear EXACTLY the events we just read, and do it now.
       - A SYS_STATUS bit is cleared by writing 1 to it and left untouched by
         writing 0, so writing our snapshot back clears only what we already
         read.
       - NEVER write 0xFF: that would also clear events which latched after the
         read above and which we have not handled yet (the lost-frame race).
       - Clearing now also pulls the IRQ line low, so any event that latches
         while we process below raises a fresh rising edge and is picked up by
         a follow-up run instead of being left latched with no edge. */
    dw1000_ll_write(DW1000_SYS_STATUS, DW1000_NO_SUB, status_entry, sizeof(status_entry));

    /* clock / PLL loss of lock */
    if (dw1000_ll_status_bit(DW1000_CLKPLL_LL_BIT) ||
        dw1000_ll_status_bit(DW1000_RFPLL_LL_BIT)) {
        if (g_on_error) {
            g_on_error();
        }
    }

    /* TX frame sent */
    if (dw1000_ll_status_bit(DW1000_TXFRS_BIT)) {
        s_irq_tx_count++;
        if (g_on_sent) {
            g_on_sent();
        }
        if (s_tx_done != NULL) {
            xSemaphoreGive(s_tx_done);   /* wakes the application TX task */
        }
    }

    /* RX timestamp available */
    if (dw1000_ll_status_bit(DW1000_LDEDONE_BIT) && g_on_receive_timestamp_available) {
        g_on_receive_timestamp_available();
    }

    /* RX failed / timeout: re-arm automatically in permanent RX.
       BUT: if the receiver has seen the preamble/SFD/PHY header and the frame
       has not completed yet, a reception is still in flight - calling idle()
       now would abort it. Any fail/timeout bit we read then is stale. (This
       matters now that LDEDONE is masked, because a run can arrive in the
       middle of a frame.) */
    {
        bool rx_in_progress =
            ((status_entry[1] & 0x0B) != 0) &&      /* RXPRD|RXSFDD|RXPHD */
            ((status_entry[1] & 0xE0) == 0);        /* no DFR/FCG/FCE     */

        if (dw1000_ll_status_bit(DW1000_LDEERR_BIT) ||
            dw1000_ll_status_bit(DW1000_RXFCE_BIT) ||
            dw1000_ll_status_bit(DW1000_RXPHE_BIT) ||
            dw1000_ll_status_bit(DW1000_RXRFSL_BIT)) {
            s_irq_rx_fail_count++;
            if (g_on_receive_failed) {
                g_on_receive_failed();
            }
            if (g_permanent_receive && !rx_in_progress) {
                dw1000_ll_rearm_receive();
            }
        } else if (dw1000_ll_status_bit(DW1000_RXRFTO_BIT) ||
                   dw1000_ll_status_bit(DW1000_RXPTO_BIT) ||
                   dw1000_ll_status_bit(DW1000_RXSFDTO_BIT)) {
            s_irq_rx_to_count++;
            if (g_on_receive_timeout) {
                g_on_receive_timeout();
            }
            if (g_permanent_receive && !rx_in_progress) {
                dw1000_ll_rearm_receive();
            }
        } else if (dw1000_ll_status_bit(DW1000_RXFCG_BIT) ||
                   dw1000_ll_status_bit(DW1000_RXDFR_BIT)) {
            /* A good frame: capture it. The RX re-arm is NOT done here - the
               application answers every received frame and its TX task owns the
               TX/RX transition, re-arming RX after TXFRS. */
            s_irq_rx_good_count++;
            if (g_on_received) {
                g_on_received();
            }
        }
    }

    /* Diagnostics: did anything latch while we were handling? */
    dw1000_ll_read(DW1000_SYS_STATUS, DW1000_NO_SUB, status_after, sizeof(status_after));
    {
        int i;
        uint8_t masked_after = 0;
        for (i = 0; i < DW1000_LEN_SYS_MASK; i++) {
            masked_after |= (uint8_t)(status_after[i] & g_sysmask[i]);
        }
        if (status_after[0] | status_after[1] | status_after[2] |
            status_after[3] | status_after[4]) {
            s_irq_pending_count++;
        }
        /* A MASKED event that latched during this run leaves the IRQ line HIGH
           (our clear only wrote back what we read), so no new rising edge will
           ever be generated for it. Re-run the handler instead of waiting for
           an edge that cannot come - otherwise that event is lost. */
        if (masked_after && s_irq_sem != NULL) {
            xSemaphoreGive(s_irq_sem);
        }
    }

    dw1000_ll_rearm_receive();
    dw1000_ll_radio_unlock();
}

/* ---- IRQ service: GPIO ISR + high-priority dispatch task ---- */

static TaskHandle_t s_irq_task = NULL;
static TaskHandle_t s_diag_task = NULL;

static void IRAM_ATTR dw1000_ll_gpio_isr(void *arg)
{
    BaseType_t higher_prio = pdFALSE;
    if (s_irq_sem != NULL) {
        xSemaphoreGiveFromISR(s_irq_sem, &higher_prio);
    }
    portYIELD_FROM_ISR(higher_prio);
}

static void dw1000_ll_irq_task(void *arg)
{
    for (;;) {
        xSemaphoreTake(s_irq_sem, portMAX_DELAY);
        dw1000_ll_handle_interrupt();
    }
}

/* Print the interrupt counters. Safe to call from any task - it is NOT called
   from the interrupt handler, so printing cannot widen the event/clear race. */
void dw1000_ll_diag_dump(void)
{
    ESP_LOGW(TAG, "IRQ runs=%u evt=%u prog=%u empty=%u(hi=%u,lo=%u) pend=%u | tx=%u rxgood=%u rxfail=%u rxto=%u | seen lde=%u pre=%u sfd=%u phr=%u frame=%u bad=%u tmo=%u pll=%u | spi_err=%u irq_rd_err=%u",
             (unsigned)s_irq_count, (unsigned)s_run_evt, (unsigned)s_run_prog,
             (unsigned)s_irq_empty_count, (unsigned)s_empty_hi, (unsigned)s_empty_lo,
             (unsigned)s_irq_pending_count,
             (unsigned)s_irq_tx_count, (unsigned)s_irq_rx_good_count,
             (unsigned)s_irq_rx_fail_count, (unsigned)s_irq_rx_to_count,
             (unsigned)s_seen_ldedone, (unsigned)s_seen_pre, (unsigned)s_seen_sfd,
             (unsigned)s_seen_phr, (unsigned)s_seen_good, (unsigned)s_seen_bad,
             (unsigned)s_seen_to, (unsigned)s_seen_pll,
             (unsigned)s_spi_err_count, (unsigned)s_irq_spi_err_count);
}

/* Low-priority task that just reports the counters periodically. */
static void dw1000_ll_diag_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        dw1000_ll_diag_dump();
    }
}

/* ---- radio lock + TX-done handshake (shared with the application) ---- */

/* Take/release the mutex that serialises complete radio sequences between the
   IRQ task and the application TX task. Always released before waiting for a
   TX to complete, so there is no deadlock with the IRQ task. */
void dw1000_ll_radio_lock(void)
{
    if (s_radio_lock != NULL) {
        xSemaphoreTake(s_radio_lock, portMAX_DELAY);
    }
}

void dw1000_ll_radio_unlock(void)
{
    if (s_radio_lock != NULL) {
        xSemaphoreGive(s_radio_lock);
    }
}

/* Drop any stale TX-done signal (call before starting a transmit). */
void dw1000_ll_tx_done_clear(void)
{
    if (s_tx_done != NULL) {
        xSemaphoreTake(s_tx_done, 0);
    }
}

/* Wait for the IRQ task to report TXFRS. Returns false on timeout. */
bool dw1000_ll_tx_done_wait(uint32_t timeout_ms)
{
    if (s_tx_done == NULL) {
        return false;
    }
    return xSemaphoreTake(s_tx_done, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

esp_err_t dw1000_ll_irq_start(uint8_t irq_gpio)
{
    esp_err_t ret;

    s_irq_gpio = (int)irq_gpio;   /* for the empty-edge pin-level diagnostic */

    if (s_irq_sem == NULL) {
        s_irq_sem = xSemaphoreCreateBinary();
        if (s_irq_sem == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    if (s_tx_done == NULL) {
        s_tx_done = xSemaphoreCreateBinary();
        if (s_tx_done == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    if (s_irq_task == NULL) {
        if (xTaskCreate(dw1000_ll_irq_task, "dw1000_irq", 4096, NULL, 20, &s_irq_task) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
    }
    // if (s_diag_task == NULL) {
    //     /* lowest priority: it can never delay the IRQ task or the radio */
    //     xTaskCreate(dw1000_ll_diag_task, "dw1000_diag", 2560, NULL, 1, &s_diag_task);
    // }

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << irq_gpio),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE,   /* DW1000 IRQ is active-high */
    };
    gpio_config(&io);

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    ret = gpio_isr_handler_add(irq_gpio, dw1000_ll_gpio_isr, NULL);
    if (ret != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

void dw1000_ll_get_temp_and_vbat(float *temp, float *vbat)
{
    /* procedure from DW1000 user manual section 6.4 */
    uint8_t step = 0x80;
    uint8_t sar_lvbat = 0;
    uint8_t sar_ltemp = 0;
    float t = 0.0f, v = 0.0f;

    dw1000_ll_write(DW1000_RF_CONF, 0x11, &step, 1);
    step = 0x0A;
    dw1000_ll_write(DW1000_RF_CONF, 0x12, &step, 1);
    step = 0x0F;
    dw1000_ll_write(DW1000_RF_CONF, 0x12, &step, 1);
    step = 0x01;
    dw1000_ll_write(DW1000_TX_CAL, DW1000_NO_SUB, &step, 1);
    step = 0x00;
    dw1000_ll_write(DW1000_TX_CAL, DW1000_NO_SUB, &step, 1);
    dw1000_ll_read(DW1000_TX_CAL, 0x03, &sar_lvbat, 1);
    dw1000_ll_read(DW1000_TX_CAL, 0x04, &sar_ltemp, 1);

    v = (float)(sar_lvbat - g_vmeas3v3) / 173.0f + 3.3f;
    t = (float)(sar_ltemp - g_tmeas23C) * 1.14f + 23.0f;

    if (vbat) {
        *vbat = v;
    }
    if (temp) {
        *temp = t;
    }
}
