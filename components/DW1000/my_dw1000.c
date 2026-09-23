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
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"


static const char *TAG = "DW1000";

/* ~15.65 ps per raw timestamp tick -> meters per tick */
#define DW1000_METERS_PER_TICK 0.0046917639786159f

/* Pairing (Option C): PAN used in headers, our EUI and the paired peer's EUI
   (both kept in register order = LSB first). s_peer_set enables the hardware
   receive frame filter. */
static uint16_t s_pan_id       = 0xFFFF;
static uint8_t  s_own_eui[8];
static uint8_t  s_peer_eui[8]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static bool     s_peer_set     = false;
uint16_t rx_counter = 1;
uint16_t received_pings= 0;
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

/* ============================ TX / RX tasks ============================
 *
 * Stage 1 architecture: all transmissions are queued and performed by a single
 * TX task, which is also the only place that changes the radio between RX and
 * TX. Received frames are captured in the IRQ task (length + timestamp +
 * bytes) and processed by a separate RX task that enqueues any reply.
 *
 *   IRQ task  : status handling + RX capture      -> rx_queue
 *   RX  task  : protocol logic, builds replies    -> tx_queue
 *   TX  task  : owns TX/RX mode, transmits frames -> radio
 */
#define DW1000_TX_QUEUE_LEN       8
#define DW1000_RX_QUEUE_LEN       8
#define DW1000_MAX_FRAME_LEN      64
#define DW1000_TX_DONE_TIMEOUT_MS 20
/* DW1000_REPLY_DELAY_US is defined in my_dw1000.h */

typedef struct {
    uint16_t len;
    bool     scheduled;      /* true: use target_ticks (absolute time) */
    uint16_t reserved;
    uint32_t delay_us;       /* relative TX delay, 0 = immediate */
    uint64_t target_ticks;   /* absolute TX time when scheduled */
    uint8_t  data[DW1000_MAX_FRAME_LEN];
} dw1000_tx_job_t;

typedef struct {
    uint16_t len;
    uint16_t reserved;
    uint64_t rx_ts;          /* RX timestamp captured in the IRQ task */
    uint8_t  data[DW1000_MAX_FRAME_LEN];
} dw1000_rx_job_t;

static QueueHandle_t s_tx_queue = NULL;
static QueueHandle_t s_rx_queue = NULL;
static TaskHandle_t  s_tx_task_h = NULL;
static TaskHandle_t  s_rx_task_h = NULL;

/* TX statistics (written by the TX task, reported on failures) */
static volatile uint32_t s_tx_sent    = 0;
static volatile uint32_t s_tx_timeout = 0;
static volatile uint32_t s_tx_dropped = 0;
static volatile uint32_t s_tx_late    = 0;
static volatile uint32_t s_rx_dropped = 0;

static void dw1000_tx_task(void *arg);
static void dw1000_rx_task(void *arg);
static void dw1000_stats_task(void *arg);

/* ============================ init / probe ============================ */

void dw1000_on_received(dw1000_handler_t cb);
static void my_dw1000_on_received(void);

esp_err_t spi_init(uint8_t sck, uint8_t miso, uint8_t mosi)
{

    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .sclk_io_num       = sck,
        .mosi_io_num       = mosi,
        .miso_io_num       = miso,
        .quadwp_io_num     = -1,
        .quadhd_io_num     = -1,
        .max_transfer_sz   = 0,
    };
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "SPI master ready (sck=%u miso=%u mosi=%u)",
             (unsigned)sck, (unsigned)miso, (unsigned)mosi);
    return ESP_OK;

}

bool dw1000_init(uint8_t cs, uint8_t irq, uint8_t rst,  
                 uint16_t network_id,
                 uint16_t device_address,
                 const uint8_t mode[3],
                 uint8_t channel,
                 uint16_t antenna_delay)
{
    (void)irq; /* interrupt handling is ported in a later step */

    if (dw1000_ll_spi_init(cs) != ESP_OK) {
        ESP_LOGE(TAG, "SPI init failed - aborting");
        return false;
    }

    /* Power-up / reset sequence (DW1000 User Manual 5.6). */
    dw1000_ll_reset(rst);
    dw1000_ll_clock(DW1000_AUTO_CLOCK);

    /* Chip-select defaults + LDE microcode load (required before RX works). */
    dw1000_ll_init_defaults();
    dw1000_ll_manage_lde();   

    if (!dw1000_probe()) {
        ESP_LOGE("DW1000", "Module not detected - check wiring/power!");
        return false;
    }

    dw1000_config(network_id, device_address, mode, channel, antenna_delay);

    /* ---- DIAGNOSTIC 3: also interrupt on the LDE-done event ----------------
       LDEDONE is NOT in the mask by default, so a ping that is received but
       never decodes produces no interrupt at all and stays invisible. Masking
       it makes every reception attempt show up as a traceable IRQ run
       (tag 'L' in the trace / "seen lde=" in the stats line). */
    dw1000_begin_config();
    dw1000_interrupt_on_receive_timestamp_available(true);   /* LDEDONE */
    dw1000_commit_config();

    /* TX/RX worker tasks + queues (see the architecture note at the top). */
    s_tx_queue = xQueueCreate(DW1000_TX_QUEUE_LEN, sizeof(dw1000_tx_job_t));
    s_rx_queue = xQueueCreate(DW1000_RX_QUEUE_LEN, sizeof(dw1000_rx_job_t));
    if (s_tx_queue == NULL || s_rx_queue == NULL) {
        ESP_LOGE(TAG, "queue creation failed - aborting");
        return false;
    }
    xTaskCreate(dw1000_rx_task, "dw1000_rx", 4096, NULL, 12, &s_rx_task_h);
    xTaskCreate(dw1000_tx_task, "dw1000_tx", 4096, NULL, 10, &s_tx_task_h);
    // xTaskCreate(dw1000_stats_task, "dw1000_txs", 3072, NULL, 1, NULL);
    ESP_LOGI(TAG, "TX/RX tasks started (tx=10, rx=12)");

    ESP_LOGI(TAG, "Register DW1000 Callback Registered");
    dw1000_on_received(my_dw1000_on_received);

    ESP_LOGI(TAG, "ISR assigned");
    dw1000_irq_start(irq);

    ESP_LOGI(TAG, "Receiving Enabled");
    dw1000_receive_permanently(true);
    dw1000_rearm_receive();

    ESP_LOGI(TAG, "Driver initialized (pure C). Probing the DW1000 ...");
    return true;
}

bool dw1000_probe(void)
{
    uint32_t id = dw1000_ll_read_device_id();

    /* A real DW1000 answers 0xDECA0130 -> top 16 bits are 0xDECA. */
    if ((id & 0xFFFF0000u) == 0xDECA0000u) {
        ESP_LOGI(TAG, "---------------------------------------------------");
        ESP_LOGI(TAG, "-> DW1000 module detected - SPI OK (ID: 0x%08X)", (unsigned)id);
        ESP_LOGI(TAG, "---------------------------------------------------");
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
    dw1000_ll_radio_lock();
    dw1000_ll_commit_configuration();
    dw1000_ll_radio_unlock();
}

/* Convenience: begin_config() + commit_config(). */
void dw1000_apply_config(void)
{
    dw1000_ll_new_configuration();
    dw1000_ll_commit_configuration();
}

/* One-call radio configuration (see header). */
void dw1000_config(uint16_t network_id,
                   uint16_t device_address,
                   const uint8_t mode[3],
                   uint8_t channel,
                   uint16_t antenna_delay)
{
    s_pan_id = network_id;              /* PAN used in the frame headers */
    /* Derive our 8-byte extended address from this ESP32's unique 6-byte BLE
       MAC (two leading zero bytes) and load it into the DW1000 EUI register so
       the hardware frame filter (FFAE) matches our own address. */
    {
        uint8_t mac[6];
        int i;
        if (esp_read_mac(mac, ESP_MAC_BT) == ESP_OK) {
            for (i = 0; i < 6; i++) {
                s_own_eui[i] = mac[5 - i];   /* register order: LSB first */
            }
            s_own_eui[6] = 0x00;
            s_own_eui[7] = 0x00;
            dw1000_ll_write(DW1000_EUI, DW1000_NO_SUB, s_own_eui, 8);
        } else {
            dw1000_ll_get_eui_bytes(s_own_eui);   /* fallback */
        }
    }
    dw1000_begin_config();
    dw1000_set_defaults();   /* standard defaults (also enables the IRQ events) */

    dw1000_set_network_id(network_id);
    dw1000_set_device_address(device_address);
    dw1000_enable_mode(mode);
    dw1000_set_channel(channel);
    dw1000_set_antenna_delay(antenna_delay);

    dw1000_commit_config();

    ESP_LOGI(TAG, "Radio configured - EUI %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
             (unsigned)s_own_eui[7], (unsigned)s_own_eui[6], (unsigned)s_own_eui[5],
             (unsigned)s_own_eui[4], (unsigned)s_own_eui[3], (unsigned)s_own_eui[2],
             (unsigned)s_own_eui[1], (unsigned)s_own_eui[0]);
    dw1000_print_device_info();
}

/* ============================ transceiver control ============================ */

/* Put the radio in idle state (no RX / TX active). */
void dw1000_idle(void)
{
    dw1000_ll_idle();
}

void dw1000_start_receive(void)
{
    dw1000_ll_radio_lock();
    dw1000_ll_new_receive();
    dw1000_ll_start_receive();
    dw1000_ll_radio_unlock();
}

/* Put the radio back into RX without touching SYS_STATUS. Preferred over
   dw1000_start_receive() once running: it cannot wipe a frame that arrived
   while we were doing something else. */
void dw1000_rearm_receive(void)
{
    dw1000_ll_radio_lock();
    dw1000_ll_rearm_receive();
    dw1000_ll_radio_unlock();
}

void dw1000_receive_permanently(bool enable)
{
    dw1000_ll_radio_lock();
    dw1000_ll_receive_permanently(enable ? 1 : 0);
    dw1000_ll_radio_unlock();
}

/* Queue a frame for the TX task. The bytes are COPIED, so the caller may reuse
   its buffer immediately. Returns false if the frame was rejected. */
static bool dw1000_queue_send(const uint8_t *data, uint16_t len,
                              uint32_t delay_us, uint64_t target_ticks, bool scheduled)
{
    dw1000_tx_job_t job;

    if (s_tx_queue == NULL) {
        ESP_LOGW(TAG, "TX: queue not ready - dropped frame (len=%u)", (unsigned)len);
        return false;
    }
    if (len == 0 || len > sizeof(job.data)) {
        ESP_LOGW(TAG, "TX: bad length %u (max %u)", (unsigned)len,
                 (unsigned)sizeof(job.data));
        return false;
    }

    memset(&job, 0, sizeof(job));
    job.len          = len;
    job.delay_us     = delay_us;
    job.target_ticks = target_ticks;
    job.scheduled    = scheduled;
    memcpy(job.data, data, len);

    if (xQueueSend(s_tx_queue, &job, 0) != pdTRUE) {
        s_tx_dropped++;
        ESP_LOGW(TAG, "TX: queue full - dropped frame (len=%u, dropped=%u)",
                 (unsigned)len, (unsigned)s_tx_dropped);
        return false;
    }
    return true;
}

/* Highest-level send: queue a frame for the TX task. Non-blocking. */
bool dw1000_send(const uint8_t *data, uint16_t len)
{
    return dw1000_queue_send(data, len, 0, 0, false);
}

/* Queue a frame with a relative TX delay (us). */
bool dw1000_send_at(const uint8_t *data, uint16_t len, uint32_t delay_us)
{
    return dw1000_queue_send(data, len, delay_us, 0, false);
}

/* Queue a frame to be transmitted at an absolute DW1000 timestamp. */
bool dw1000_send_at_ticks(const uint8_t *data, uint16_t len, uint64_t target_ticks)
{
    return dw1000_queue_send(data, len, 0, target_ticks, true);
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

/* ============================ DS-TWR ranging ============================ */
/* Implementation of the ready-to-use ranging engine declared in the header.
   The tag drives the exchange; the anchor replies and computes the distance. */

/* Full IEEE 802.15.4 extended-address frame (header + DW1000_LEN_DATA payload):
     [0-1]   frame control (data, PAN compression, dest+src extended)
     [2]     sequence number
     [3-10]  destination EUI (LSB first)
     [11-12] source PAN id (PAN compression = single PAN, after dest addr)
     [13-20] source EUI (LSB first)
     [21..]  payload: [0]=type, [1-5]=T1, [6-10]=T4, [11-15]=T5
             (RANGE_REPORT reuses [1-4] for the float distance) */
#define DW1000_HDR_LEN     21
#define DW1000_OFF_DST_EUI 3
#define DW1000_OFF_SRC_EUI 13
#define DW1000_OFF_TYPE    (DW1000_HDR_LEN + 0)
#define DW1000_OFF_T1      (DW1000_HDR_LEN + 1)
#define DW1000_OFF_T4      (DW1000_HDR_LEN + 6)
#define DW1000_OFF_T5      (DW1000_HDR_LEN + 11)
#define DW1000_OFF_CAL_SESSION (DW1000_HDR_LEN + 1)   /* calibration session id    */
#define DW1000_OFF_CAL_AD      (DW1000_HDR_LEN + 2)   /* new antenna delay (2 B LE) */

/* ranging state (file scope) */
static uint8_t  s_data[DW1000_HDR_LEN + DW1000_LEN_DATA];
static uint8_t  s_last_sent_type;      /* tag: which frame was just transmitted */
static uint64_t s_t1;                  /* tag: poll TX timestamp */
static uint64_t s_t2, s_t3;            /* anchor: poll RX, poll_ack TX timestamp */
static volatile bool s_result_ready;   /* true when a RANGE_REPORT arrived */
static float  s_last_distance;         /* last measured distance (m) */
static bool   s_tag_init_done;         /* run_tag radio setup done once */
static uint8_t s_seq;                  /* IEEE header sequence counter */

/* calibration message state (CAL_SET tag->anchor / CAL_ACK anchor->tag) */
static volatile bool s_cal_ack_received = false;  /* true when a CAL_ACK arrived   */
static uint16_t      s_cal_ack_ad       = 0;      /* antenna delay the anchor set  */
static uint8_t       s_cal_session      = 0;      /* calibration session counter   */

/* write the IEEE 802.15.4 header (extended addressing) in front of the payload */
static void build_header(uint8_t *buf, uint16_t pan,
                         const uint8_t dst_eui[8], const uint8_t src_eui[8])
{
    buf[0] = 0x41;               /* frame type = data, PAN ID compression on */
    buf[1] = 0xCC;               /* IEEE 2011: dest addressing = ext, src = ext */
    buf[2] = s_seq++;
    memcpy(buf + 3,  dst_eui, 8);        /* destination EUI (LSB first) */
    buf[11] = (uint8_t)(pan & 0xFF);     /* source PAN (compression = one PAN) */
    buf[12] = (uint8_t)(pan >> 8);
    memcpy(buf + 13, src_eui, 8);        /* source EUI (LSB first) */
}

/* true if the received frame came from the paired peer (or pairing is off) */
static bool is_peer(const uint8_t *frame)
{
    if (!s_peer_set) {
        return true;
    }
    return memcmp(frame + DW1000_OFF_SRC_EUI, s_peer_eui, 8) == 0;
}

/* true if the frame is addressed to us (destination EUI == our own EUI) */
static bool is_addressed_to_me(const uint8_t *frame)
{
    return memcmp(frame + DW1000_OFF_DST_EUI, s_own_eui, 8) == 0;
}

/* write a 40-bit timestamp into 5 little-endian bytes */
static void put_ts(uint8_t *buf, uint64_t ts)
{
    int i;
    for (i = 0; i < 5; i++) {
        buf[i] = (uint8_t)((ts >> (i * 8)) & 0xFF);
    }
}

/* read a 40-bit timestamp from 5 little-endian bytes */
static uint64_t get_ts(const uint8_t *buf)
{
    uint64_t ts = 0;
    int i;
    for (i = 0; i < 5; i++) {
        ts |= (uint64_t)buf[i] << (i * 8);
    }
    return ts;
}

/* Time difference with 40-bit rollover handling (mirrors DW1000Time::wrap).
   The system counter wraps every 2^40 ticks (~17 s); if a difference goes
   negative because later/earlier straddle the rollover, add 2^40 back. */
static int64_t diff_ts(uint64_t later, uint64_t earlier)
{
    int64_t d = (int64_t)(later - earlier);
    if (d < 0) {
        d += (int64_t)0x10000000000LL;   /* 2^40 */
    }
    return d;
}

/* ---------------- tag side ---------------- */

static void tag_on_sent(void)
{
    if (s_last_sent_type == DW1000_MSG_POLL) {
        s_t1 = dw1000_get_tx_timestamp();
    }
}

/*static void tag_on_received(void)
{
    uint16_t len = dw1000_get_data(s_data, sizeof(s_data));
    if (len < DW1000_HDR_LEN + 1) {
        ESP_LOGW(TAG, "TAG RX: short frame len=%u", (unsigned)len);
        dw1000_start_receive();   // re-arm: we dropped this frame 
        return;
    }
    if (!is_peer(s_data)) {
        uint8_t *e = s_data + DW1000_OFF_SRC_EUI;
        ESP_LOGW(TAG, "TAG RX: dropped (not peer) type=%u src=%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                 (unsigned)s_data[DW1000_OFF_TYPE],
                 (unsigned)e[7], (unsigned)e[6], (unsigned)e[5], (unsigned)e[4],
                 (unsigned)e[3], (unsigned)e[2], (unsigned)e[1], (unsigned)e[0]);
        dw1000_start_receive();   // re-arm: we dropped this frame 
        return;
    }
    if (!is_addressed_to_me(s_data)) {
        uint8_t *d = s_data + DW1000_OFF_DST_EUI;
        ESP_LOGW(TAG, "TAG RX: dropped (not for me) type=%u dst=%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                 (unsigned)s_data[DW1000_OFF_TYPE],
                 (unsigned)d[7], (unsigned)d[6], (unsigned)d[5], (unsigned)d[4],
                 (unsigned)d[3], (unsigned)d[2], (unsigned)d[1], (unsigned)d[0]);
        dw1000_start_receive();   // re-arm: we dropped this frame 
        return;
    }
    ESP_LOGI(TAG, "TAG RX: peer frame type=%u len=%u",
             (unsigned)s_data[DW1000_OFF_TYPE], (unsigned)len);

    if (s_data[DW1000_OFF_TYPE] == DW1000_MSG_POLL_ACK) {
        // anchor replied: send RANGE with T1 (poll), T4 (ack rx), T5 (range) 
        uint64_t t4 = dw1000_get_rx_timestamp();
        uint64_t now = dw1000_get_system_timestamp();
        uint64_t target = now + (uint64_t)((float)DW1000_REPLY_DELAY_US * 63897.6f);
        // + antenna delay: TX times must be antenna-referenced (like setDelay/TX_TIME) 
        uint64_t t5 = (target & ~0x1FFULL) + (uint64_t)dw1000_get_antenna_delay();

        memset(s_data, 0, sizeof(s_data));
        s_data[DW1000_OFF_TYPE] = DW1000_MSG_RANGE;
        put_ts(s_data + DW1000_OFF_T1, s_t1);
        put_ts(s_data + DW1000_OFF_T4, t4);
        put_ts(s_data + DW1000_OFF_T5, t5);
        build_header(s_data, s_pan_id, s_peer_eui, s_own_eui);
        s_last_sent_type = DW1000_MSG_RANGE;
        dw1000_send_at_ticks(s_data, DW1000_HDR_LEN + DW1000_LEN_DATA, target);
    } else if (s_data[DW1000_OFF_TYPE] == DW1000_MSG_RANGE_REPORT) {
        memcpy(&s_last_distance, s_data + DW1000_OFF_T1, sizeof(s_last_distance));
        s_result_ready = true;
        dw1000_start_receive();   // re-arm for the next exchange 
    } else if (s_data[DW1000_OFF_TYPE] == DW1000_MSG_CAL_ACK) {
        // the anchor applied the antenna delay we asked for 
        s_cal_ack_ad = (uint16_t)(s_data[DW1000_OFF_CAL_AD] |
                                  ((uint16_t)s_data[DW1000_OFF_CAL_AD + 1] << 8));
        s_cal_ack_received = true;
        ESP_LOGI(TAG, "TAG RX: CAL_ACK session=%u antenna_delay=%u",
                 (unsigned)s_data[DW1000_OFF_CAL_SESSION], (unsigned)s_cal_ack_ad);
        dw1000_start_receive();   // re-arm for the next exchange 
    }
}*/

bool dw1000_run_tag(int irq_gpio, int timeout_ms, dw1000_distance_cb_t on_distance)
{
    bool ok;

    if (!s_tag_init_done) {
        s_tag_init_done = true;
        dw1000_irq_start(irq_gpio);
        dw1000_receive_permanently(true);
        // dw1000_on_sent(tag_on_sent);
        if (s_last_sent_type == DW1000_MSG_POLL) {
            s_t1 = dw1000_get_tx_timestamp();
        }
        // dw1000_on_received(my_dw1000_on_received);
    }

    // start one exchange */
    s_result_ready = false;
    uint8_t frame[DW1000_HDR_LEN + DW1000_LEN_DATA];
    memset(frame, 0, sizeof(frame));
    frame[DW1000_OFF_TYPE] = DW1000_MSG_POLL;
    build_header(frame, s_pan_id, s_peer_eui, s_own_eui);
    s_last_sent_type = DW1000_MSG_POLL;
    dw1000_send(frame, DW1000_HDR_LEN + DW1000_LEN_DATA);

    // wait for the result or the timeout */
    TickType_t t0 = xTaskGetTickCount();
    while (!s_result_ready &&
           (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(timeout_ms)) {
        /* MUST block at least one tick: pdMS_TO_TICKS(5) == 0 at
         * CONFIG_FREERTOS_HZ=100, and vTaskDelay(0) does not block. The caller
         * (mv_monitor, prio 5) would then spin at 100% CPU for the whole
         * timeout, starving IDLE0 and tripping the task watchdog. */
        vTaskDelay(1);    
    }

    ok = s_result_ready;
    if (on_distance != NULL) {
        return on_distance(ok ? s_last_distance : -1.0f, ok);
    }
    return ok;
}

/* ---------------- anchor side ---------------- */

/*static void anchor_on_received(void)
{
    uint16_t len = dw1000_get_data(s_data, sizeof(s_data));
    if (len < DW1000_HDR_LEN + 1) {
        ESP_LOGW(TAG, "ANCHOR RX: short frame len=%u", (unsigned)len);
        dw1000_start_receive();   // re-arm: we dropped this frame 
        return;
    }
    if (!is_peer(s_data)) {
        uint8_t *e = s_data + DW1000_OFF_SRC_EUI;
        ESP_LOGW(TAG, "ANCHOR RX: dropped (not peer) type=%u src=%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                 (unsigned)s_data[DW1000_OFF_TYPE],
                 (unsigned)e[7], (unsigned)e[6], (unsigned)e[5], (unsigned)e[4],
                 (unsigned)e[3], (unsigned)e[2], (unsigned)e[1], (unsigned)e[0]);
        dw1000_start_receive();   // re-arm: we dropped this frame 
        return;
    }
    if (!is_addressed_to_me(s_data)) {
        uint8_t *d = s_data + DW1000_OFF_DST_EUI;
        ESP_LOGW(TAG, "ANCHOR RX: dropped (not for me) type=%u dst=%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                 (unsigned)s_data[DW1000_OFF_TYPE],
                 (unsigned)d[7], (unsigned)d[6], (unsigned)d[5], (unsigned)d[4],
                 (unsigned)d[3], (unsigned)d[2], (unsigned)d[1], (unsigned)d[0]);
        dw1000_start_receive();   // re-arm: we dropped this frame 
        return;
    }
    ESP_LOGI(TAG, "ANCHOR RX: peer frame type=%u len=%u",
             (unsigned)s_data[DW1000_OFF_TYPE], (unsigned)len);

    if (s_data[DW1000_OFF_TYPE] == DW1000_MSG_POLL) {
        // reply after REPLY_DELAY_US; the actual reply time is measured 
        s_t2 = dw1000_get_rx_timestamp();
        uint64_t now = dw1000_get_system_timestamp();
        uint64_t target = now + (uint64_t)((float)DW1000_REPLY_DELAY_US * 63897.6f);
        // + antenna delay: the TX timestamp must be antenna-referenced (like TX_TIME) 
        s_t3 = (target & ~0x1FFULL) + (uint64_t)dw1000_get_antenna_delay();

        memset(s_data, 0, sizeof(s_data));
        s_data[DW1000_OFF_TYPE] = DW1000_MSG_POLL_ACK;
        build_header(s_data, s_pan_id, s_peer_eui, s_own_eui);
        dw1000_send_at_ticks(s_data, DW1000_HDR_LEN + DW1000_LEN_DATA, target);
    } else if (s_data[DW1000_OFF_TYPE] == DW1000_MSG_RANGE) {
        // we have T2/T3; RANGE carries T1/T4/T5; measure T6 now 
        uint64_t t1 = get_ts(s_data + DW1000_OFF_T1);
        uint64_t t4 = get_ts(s_data + DW1000_OFF_T4);
        uint64_t t5 = get_ts(s_data + DW1000_OFF_T5);
        uint64_t t6 = dw1000_get_rx_timestamp();

        // asymmetric DS-TWR (arduino-dw1000 computeRangeAsymmetric) 
        int64_t round1 = diff_ts(t4, t1);      // tag:   poll -> ack rx 
        int64_t reply1 = diff_ts(s_t3, s_t2);  // anchor: poll rx -> ack tx 
        int64_t round2 = diff_ts(t6, s_t3);    // anchor: ack tx -> range rx 
        int64_t reply2 = diff_ts(t5, t4);      // tag:   ack rx -> range tx 
        int64_t num = round1 * round2 - reply1 * reply2;
        int64_t den = round1 + round2 + reply1 + reply2;
        int64_t tof = (den != 0) ? (num / den) : 0;
        float range = (float)tof * DW1000_METERS_PER_TICK;
        s_last_distance = range;   // remember so calibration can use it 

        ESP_LOGI(TAG, "RANGE OK: %.2f m", (double)range);

        // send the computed range back to the tag 
        memset(s_data, 0, sizeof(s_data));
        s_data[DW1000_OFF_TYPE] = DW1000_MSG_RANGE_REPORT;
        memcpy(s_data + DW1000_OFF_T1, &range, sizeof(range));
        build_header(s_data, s_pan_id, s_peer_eui, s_own_eui);
        dw1000_send(s_data, DW1000_HDR_LEN + DW1000_LEN_DATA);
    } else if (s_data[DW1000_OFF_TYPE] == DW1000_MSG_CAL_SET) {
        // calibration: the tag asks us to apply a new shared antenna delay 
        uint8_t  session = s_data[DW1000_OFF_CAL_SESSION];
        uint16_t new_ad  = (uint16_t)(s_data[DW1000_OFF_CAL_AD] |
                                      ((uint16_t)s_data[DW1000_OFF_CAL_AD + 1] << 8));

        ESP_LOGI(TAG, "ANCHOR: CAL_SET session=%u - applying antenna_delay=%u",
                 (unsigned)session, (unsigned)new_ad);
        dw1000_set_antenna_delay(new_ad);
        dw1000_commit_config();   // writes TX_ANTD + LDE_RXANTD 
        ESP_LOGI(TAG, "ANCHOR: antenna_delay=%u applied (TX_ANTD + LDE_RXANTD)",
                 (unsigned)new_ad);

        // acknowledge so the tag can continue to the next iteration 
        memset(s_data, 0, sizeof(s_data));
        s_data[DW1000_OFF_TYPE]        = DW1000_MSG_CAL_ACK;
        s_data[DW1000_OFF_CAL_SESSION] = session;
        s_data[DW1000_OFF_CAL_AD]      = new_ad & 0xFF;
        s_data[DW1000_OFF_CAL_AD + 1]  = (uint8_t)(new_ad >> 8);
        build_header(s_data, s_pan_id, s_peer_eui, s_own_eui);
        dw1000_send(s_data, DW1000_HDR_LEN + DW1000_LEN_DATA);
    }
}*/

void dw1000_run_anchor(int irq_gpio)
{
    dw1000_irq_start(irq_gpio);
    dw1000_receive_permanently(true);
    dw1000_on_received(my_dw1000_on_received);
    // dw1000_on_received(anchor_on_received);
    dw1000_start_receive();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ---------------- Ping ---------------- */

bool dw1000_ping(int irq_gpio, int timeout_ms)
{
    bool ok;

    // start one exchange 
    s_result_ready = false;
    uint8_t frame[DW1000_HDR_LEN + DW1000_LEN_DATA];
    memset(frame, 0, sizeof(frame));
    frame[DW1000_OFF_TYPE] = DW1000_MSG_PING;
    build_header(frame, s_pan_id, s_peer_eui, s_own_eui);
    s_last_sent_type = DW1000_MSG_PING;
    dw1000_send(frame, DW1000_HDR_LEN + DW1000_LEN_DATA);

    ESP_LOGI("DW1000", "Trying to Ping!");

    // wait for the result or the timeout 
    TickType_t t0 = xTaskGetTickCount();
    while (!s_result_ready &&
           (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(timeout_ms)) {
        vTaskDelay(1);   /* >= 1 tick, see dw1000_run_tag() */
    }

    ok = s_result_ready;
    return ok;
}

/* ---------------- Type to String ---------------- */
static const char *dw1000_msg_type_to_string(uint8_t type)
{
    switch (type) {
        case DW1000_MSG_POLL:         return "POLL";
        case DW1000_MSG_POLL_ACK:     return "POLL_ACK";
        case DW1000_MSG_RANGE:        return "RANGE";
        case DW1000_MSG_RANGE_REPORT: return "RANGE_REPORT";
        case DW1000_MSG_CAL_SET:      return "CAL_SET";
        case DW1000_MSG_CAL_ACK:      return "CAL_ACK";
        case DW1000_MSG_PING:         return "PING";
        case DW1000_MSG_PING_ACK:     return "PING_ACK";
        default:                      return "UNKNOWN";
    }
}

/* ---------------- RX capture (runs in the IRQ task) ---------------------
 * Called by the low-level IRQ handler, which already holds the radio lock.
 * Captures only length, RX timestamp and bytes, then hands them to the RX
 * task. No protocol work and no logging here. */
static void my_dw1000_on_received(void)
{
    dw1000_rx_job_t job;
    // ESP_LOGI(TAG, "received somthing............");

    memset(&job, 0, sizeof(job));
    job.rx_ts = dw1000_ll_get_receive_timestamp();   /* read before anything else */
    job.len   = dw1000_get_data(job.data, sizeof(job.data));

    if (job.len == 0) {
        return;
    }
    if (xQueueSend(s_rx_queue, &job, 0) != pdTRUE) {
        s_rx_dropped++;   /* counted only: this runs in the IRQ task */
    }
}

/* ---------------- RX protocol (runs in the RX task) --------------------- */
static void dw1000_process_rx(const uint8_t *frame, uint16_t len, uint64_t rx_ts)
{
    (void)rx_ts;   /* Stage 2 will use the captured timestamp for ranging */
    if (len > sizeof(s_data)) {
        len = sizeof(s_data);
    }
    memcpy(s_data, frame, len);
    // ESP_LOGI("DW1000", "is_length_ok");
    if (len < DW1000_HDR_LEN + 1) {
        ESP_LOGW(TAG, "ANCHOR RX: short frame len=%u", (unsigned)len);
        return;
    }
    // ESP_LOGI("DW1000", "is_peer");
    if (!is_peer(s_data)) {
        uint8_t *e = s_data + DW1000_OFF_SRC_EUI;
        ESP_LOGW(TAG, "ANCHOR RX: dropped (not peer) type=%u src=%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                 (unsigned)s_data[DW1000_OFF_TYPE],
                 (unsigned)e[7], (unsigned)e[6], (unsigned)e[5], (unsigned)e[4],
                 (unsigned)e[3], (unsigned)e[2], (unsigned)e[1], (unsigned)e[0]);
        return;
    }
    // ESP_LOGI("DW1000", "is_addressed_to_me");
    if (!is_addressed_to_me(s_data)) {
        uint8_t *d = s_data + DW1000_OFF_DST_EUI;
        ESP_LOGW(TAG, "ANCHOR RX: dropped (not for me) type=%u dst=%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                 (unsigned)s_data[DW1000_OFF_TYPE],
                 (unsigned)d[7], (unsigned)d[6], (unsigned)d[5], (unsigned)d[4],
                 (unsigned)d[3], (unsigned)d[2], (unsigned)d[1], (unsigned)d[0]);
        return;
    }
    /* ---- DIAGNOSTIC 1: link quality of the frame we just decoded ---------
       Read under the radio lock: these registers live in the chip and the IRQ
       task could interleave its own SPI transactions with these reads. */
    {
        float rx_pwr, fp_pwr, qual;
        dw1000_ll_radio_lock();
        rx_pwr = dw1000_get_rx_power_dbm();
        fp_pwr = dw1000_get_first_path_power_dbm();
        qual   = dw1000_get_rx_quality();
        dw1000_ll_radio_unlock();
        ESP_LOGI(TAG, "RX: peer frame type=<%s> len=%u | rx_pwr=%.1f dBm first_path=%.1f dBm quality=%.1f",
                 dw1000_msg_type_to_string((s_data[DW1000_OFF_TYPE])), (unsigned)len,
                 (double)rx_pwr, (double)fp_pwr, (double)qual);
    }

    if (s_data[DW1000_OFF_TYPE] == DW1000_MSG_POLL) {
        // reply after REPLY_DELAY_US; the actual reply time is measured 
        ESP_LOGI(TAG, "DW1000_MSG_POLL");
        s_t2 = dw1000_get_rx_timestamp();
        uint64_t now = dw1000_get_system_timestamp();
        uint64_t target = now + (uint64_t)((float)DW1000_REPLY_DELAY_US * 63897.6f);
        // + antenna delay: the TX timestamp must be antenna-referenced (like TX_TIME) 
        s_t3 = (target & ~0x1FFULL) + (uint64_t)dw1000_get_antenna_delay();

        memset(s_data, 0, sizeof(s_data));
        s_data[DW1000_OFF_TYPE] = DW1000_MSG_POLL_ACK;
        build_header(s_data, s_pan_id, s_peer_eui, s_own_eui);
        dw1000_send_at_ticks(s_data, DW1000_HDR_LEN + DW1000_LEN_DATA, target);
    } else if (s_data[DW1000_OFF_TYPE] == DW1000_MSG_POLL_ACK) {
        ESP_LOGI(TAG, "DW1000_MSG_POLL_ACK");
        // anchor replied: send RANGE with T1 (poll), T4 (ack rx), T5 (range) 
        uint64_t t4 = dw1000_get_rx_timestamp();
        uint64_t now = dw1000_get_system_timestamp();
        uint64_t target = now + (uint64_t)((float)DW1000_REPLY_DELAY_US * 63897.6f);
        // + antenna delay: TX times must be antenna-referenced (like setDelay/TX_TIME) 
        uint64_t t5 = (target & ~0x1FFULL) + (uint64_t)dw1000_get_antenna_delay();

        memset(s_data, 0, sizeof(s_data));
        s_data[DW1000_OFF_TYPE] = DW1000_MSG_RANGE;
        put_ts(s_data + DW1000_OFF_T1, s_t1);
        put_ts(s_data + DW1000_OFF_T4, t4);
        put_ts(s_data + DW1000_OFF_T5, t5);
        build_header(s_data, s_pan_id, s_peer_eui, s_own_eui);
        s_last_sent_type = DW1000_MSG_RANGE;
        dw1000_send_at_ticks(s_data, DW1000_HDR_LEN + DW1000_LEN_DATA, target);
    } else if (s_data[DW1000_OFF_TYPE] == DW1000_MSG_RANGE) {
        ESP_LOGI(TAG, "DW1000_MSG_RANGE");
        // we have T2/T3; RANGE carries T1/T4/T5; measure T6 now 
        uint64_t t1 = get_ts(s_data + DW1000_OFF_T1);
        uint64_t t4 = get_ts(s_data + DW1000_OFF_T4);
        uint64_t t5 = get_ts(s_data + DW1000_OFF_T5);
        uint64_t t6 = dw1000_get_rx_timestamp();

        // asymmetric DS-TWR (arduino-dw1000 computeRangeAsymmetric) 
        int64_t round1 = diff_ts(t4, t1);      // tag:   poll -> ack rx 
        int64_t reply1 = diff_ts(s_t3, s_t2);  // anchor: poll rx -> ack tx 
        int64_t round2 = diff_ts(t6, s_t3);    // anchor: ack tx -> range rx 
        int64_t reply2 = diff_ts(t5, t4);      // tag:   ack rx -> range tx 
        int64_t num = round1 * round2 - reply1 * reply2;
        int64_t den = round1 + round2 + reply1 + reply2;
        int64_t tof = (den != 0) ? (num / den) : 0;
        float range = (float)tof * DW1000_METERS_PER_TICK;
        s_last_distance = range;   // remember so calibration can use it 

        ESP_LOGI(TAG, "RANGE OK: %.2f m", (double)range);

        // send the computed range back to the tag 
        memset(s_data, 0, sizeof(s_data));
        s_data[DW1000_OFF_TYPE] = DW1000_MSG_RANGE_REPORT;
        memcpy(s_data + DW1000_OFF_T1, &range, sizeof(range));
        build_header(s_data, s_pan_id, s_peer_eui, s_own_eui);
        dw1000_send(s_data, DW1000_HDR_LEN + DW1000_LEN_DATA);
    } else if (s_data[DW1000_OFF_TYPE] == DW1000_MSG_RANGE_REPORT) {
        ESP_LOGI(TAG, "DW1000_MSG_RANGE_REPORT");
        memcpy(&s_last_distance, s_data + DW1000_OFF_T1, sizeof(s_last_distance));
        s_result_ready = true;
        dw1000_start_receive();   // re-arm for the next exchange 
    } else if (s_data[DW1000_OFF_TYPE] == DW1000_MSG_CAL_SET) {
        // calibration: the tag asks us to apply a new shared antenna delay 
        uint8_t  session = s_data[DW1000_OFF_CAL_SESSION];
        uint16_t new_ad  = (uint16_t)(s_data[DW1000_OFF_CAL_AD] |
                                      ((uint16_t)s_data[DW1000_OFF_CAL_AD + 1] << 8));

        ESP_LOGI(TAG, "ANCHOR: CAL_SET session=%u - applying antenna_delay=%u",
                 (unsigned)session, (unsigned)new_ad);
        dw1000_set_antenna_delay(new_ad);
        dw1000_commit_config();   // writes TX_ANTD + LDE_RXANTD 
        ESP_LOGI(TAG, "ANCHOR: antenna_delay=%u applied (TX_ANTD + LDE_RXANTD)",
                 (unsigned)new_ad);

        // acknowledge so the tag can continue to the next iteration 
        memset(s_data, 0, sizeof(s_data));
        s_data[DW1000_OFF_TYPE]        = DW1000_MSG_CAL_ACK;
        s_data[DW1000_OFF_CAL_SESSION] = session;
        s_data[DW1000_OFF_CAL_AD]      = new_ad & 0xFF;
        s_data[DW1000_OFF_CAL_AD + 1]  = (uint8_t)(new_ad >> 8);
        build_header(s_data, s_pan_id, s_peer_eui, s_own_eui);
        dw1000_send(s_data, DW1000_HDR_LEN + DW1000_LEN_DATA);
    } else if (s_data[DW1000_OFF_TYPE] == DW1000_MSG_CAL_ACK) {
        ESP_LOGI(TAG, "DW1000_MSG_CALL_ACK");
        // the anchor applied the antenna delay we asked for 
        s_cal_ack_ad = (uint16_t)(s_data[DW1000_OFF_CAL_AD] |
                                  ((uint16_t)s_data[DW1000_OFF_CAL_AD + 1] << 8));
        s_cal_ack_received = true;
        ESP_LOGI(TAG, "TAG RX: CAL_ACK session=%u antenna_delay=%u",
                 (unsigned)s_data[DW1000_OFF_CAL_SESSION], (unsigned)s_cal_ack_ad);
        dw1000_start_receive();   // re-arm for the next exchange 
    } else if (s_data[DW1000_OFF_TYPE] == DW1000_MSG_PING) {
        // anchor replied: send RANGE with T1 (poll), T4 (ack rx), T5 (range) 
        ESP_LOGI("DW1000", "Ping");
        received_pings ++;
        // send the computed range back to the tag 
        memset(s_data, 0, sizeof(s_data));
        s_data[DW1000_OFF_TYPE] = DW1000_MSG_PING_ACK;
        build_header(s_data, s_pan_id, s_peer_eui, s_own_eui);
        dw1000_send(s_data, DW1000_HDR_LEN + DW1000_LEN_DATA);
    } else if (s_data[DW1000_OFF_TYPE] == DW1000_MSG_PING_ACK) {
        // anchor replied: send RANGE with T1 (poll), T4 (ack rx), T5 (range) 
        ESP_LOGI("DW1000", "Ping ACK");
        s_result_ready = true;
    } 

    ESP_LOGI("DW1000", "Receiver Counter: %d", rx_counter);
    ESP_LOGI("DW1000", "------------------------------");
    rx_counter++;
}

/* ---------------- RX task: protocol processing ---------------- */
static void dw1000_rx_task(void *arg)
{
    (void)arg;
    dw1000_rx_job_t job;

    for (;;) {
        if (xQueueReceive(s_rx_queue, &job, portMAX_DELAY) == pdTRUE) {
            dw1000_process_rx(job.data, job.len, job.rx_ts);
        }
    }
}

/* ---------------- TX task: single owner of the TX/RX transition ----------------
 * Keeps the radio in RX and, for every queued frame: idle -> load -> transmit
 * -> wait for TXFRS -> back to RX. All mode changes happen here, so send
 * problems can be attributed in one place. */
static void dw1000_tx_task(void *arg)
{
    (void)arg;
    dw1000_tx_job_t job;

    for (;;) {
        if (xQueueReceive(s_tx_queue, &job, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        dw1000_ll_radio_lock();

        if (job.scheduled) {
            uint64_t now = dw1000_ll_get_system_timestamp();
            if (now > job.target_ticks) {
                s_tx_late++;
                ESP_LOGW(TAG, "TX: scheduled time already passed by %llu ticks (late=%u)",
                         (unsigned long long)(now - job.target_ticks),
                         (unsigned)s_tx_late);
            }
        }

        dw1000_ll_set_data(job.data, job.len);
        dw1000_ll_new_transmit();          /* idle + clear TX status + mode = TX */
        dw1000_ll_tx_done_clear();

        if (job.scheduled) {
            dw1000_ll_start_transmit_at(job.target_ticks);
        } else if (job.delay_us != 0) {
            dw1000_ll_set_delay(job.delay_us);
            dw1000_ll_start_transmit();
        } else {
            dw1000_ll_start_transmit();
        }

        dw1000_ll_radio_unlock();

        /* Wait OUTSIDE the lock: the IRQ task needs it to report TXFRS. */
        if (dw1000_ll_tx_done_wait(DW1000_TX_DONE_TIMEOUT_MS)) {
            s_tx_sent++;
        } else {
            s_tx_timeout++;
            ESP_LOGW(TAG, "TX: no TXFRS within %d ms (len=%u, timeouts=%u)",
                     DW1000_TX_DONE_TIMEOUT_MS, (unsigned)job.len,
                     (unsigned)s_tx_timeout);
        }
        // ESP_LOGI(TAG, "sent successfull...................");

        /* Back to RX - the TX task is the only place this happens. */
        dw1000_ll_radio_lock();
        dw1000_ll_rearm_receive();
        dw1000_ll_radio_unlock();
    }
}

/* ---------------- periodic TX/RX statistics (lowest priority) ----------------
 * Never touches the radio and never blocks the TX/RX/IRQ tasks. */
static void dw1000_stats_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint8_t sysctrl[DW1000_LEN_SYS_CTRL] = {0};
        uint8_t sysmask[DW1000_LEN_SYS_MASK] = {0};

        vTaskDelay(pdMS_TO_TICKS(5000));

        /* ---- DIAGNOSTIC 2: is the receiver still armed? ------------------
           Read-only (nothing is cleared) and under the radio lock, so this
           cannot disturb the IRQ path. rxenab=0 while frames are missing ->
           the receiver got disarmed; rxenab=1 while frames are missing ->
           the receiver was armed but nothing decoded. */
        dw1000_ll_radio_lock();
        dw1000_ll_read(DW1000_SYS_CTRL, DW1000_NO_SUB, sysctrl, sizeof(sysctrl));
        dw1000_ll_read(DW1000_SYS_MASK, DW1000_NO_SUB, sysmask, sizeof(sysmask));
        dw1000_ll_radio_unlock();

        ESP_LOGW(TAG, "TX stats: sent=%u timeout=%u dropped=%u late=%u | rx_dropped=%u received_ping=%u | "
                      "SYS_CTRL=%02X%02X%02X%02X rxenab=%d trxoff=%d | SYS_MASK=%02X%02X%02X%02X",
                 (unsigned)s_tx_sent, (unsigned)s_tx_timeout,
                 (unsigned)s_tx_dropped, (unsigned)s_tx_late,
                 (unsigned)s_rx_dropped, (unsigned)received_pings,
                 (unsigned)sysctrl[0], (unsigned)sysctrl[1],
                 (unsigned)sysctrl[2], (unsigned)sysctrl[3],
                 (sysctrl[1] & 0x01) ? 1 : 0,     /* RXENAB = bit 8 */
                 (sysctrl[0] & 0x40) ? 1 : 0,     /* TRXOFF = bit 6 */
                 (unsigned)sysmask[0], (unsigned)sysmask[1],
                 (unsigned)sysmask[2], (unsigned)sysmask[3]);
    }
}

void dw1000_set_peer_eui(const uint8_t peer_mac[6])
{
    int i;
    // caller passes the peer's 6-byte ESP32 BLE MAC (peer_mac[0] = MSB). Pad
    //   to the 8-byte extended address with two leading zero bytes and store
    //   LSB-first (register / header order). 
    for (i = 0; i < 6; i++) {
        s_peer_eui[i] = peer_mac[5 - i];
    }
    s_peer_eui[6] = 0x00;
    s_peer_eui[7] = 0x00;
    s_peer_set = true;
    // Pairing is done in software (is_peer()): keep the hardware frame filter
    //   OFF so any valid PHY frame is received and we decide here by source EUI.
    //   This avoids depending on the DW1000's address-match hardware. 
    dw1000_ll_apply_frame_filter(0, 0);
    ESP_LOGW(TAG, "Paired with peer BLE MAC %02X:%02X:%02X:%02X:%02X:%02X",
             (unsigned)peer_mac[0], (unsigned)peer_mac[1], (unsigned)peer_mac[2],
             (unsigned)peer_mac[3], (unsigned)peer_mac[4], (unsigned)peer_mac[5]);
}

/*
 * Antenna-delay calibration (mirrors the arduino-dw1000 tuning procedure).
 *
 * Procedure: place the two modules at a KNOWN distance with clear line of
 * sight, let at least one ranging exchange complete (so a measured distance
 * is available), then call this with the true distance in centimetres. It
 * corrects the antenna delay so the measured distance matches the known
 * distance, writes the new value to the radio and returns it.
 *
 *   known_distance_cm = the true physical distance between the modules (cm)
 *   return value      = the corrected antenna delay, in raw ticks (use the
 *                       SAME value on BOTH boards)
 *
 * Relationship (verified against the arduino tuning steps): 1 tick of antenna
 * delay shifts the measured range by ~DW1000_METERS_PER_TICK (0.47 cm), so a
 * measured distance that reads too large is fixed by INCREASING the delay.
 * Call it again after a fresh reading to refine (it converges).
 */
/* Compute the antenna delay (in raw ticks) that would make the last measured
   distance equal the known distance. Pure math - no radio write. */
static uint16_t dw1000_compute_ideal_antenna_delay(float known_distance_cm)
{
    uint16_t old_ad = dw1000_get_antenna_delay();
    float true_m = known_distance_cm / 100.0f;
    float meas_m = s_last_distance;
    int32_t corr, new_ad;

    if (meas_m <= 0.0f) {
        ESP_LOGW(TAG, "Calibrate: no range reading yet, antenna delay unchanged (%u)",
                 (unsigned)old_ad);
        return old_ad;
    }

    // +1 tick of antenna delay shifts the measured range by -METERS_PER_TICK,
    //  so the correction (in ticks) is the measured error scaled to ticks. 
    corr = (int32_t)((meas_m - true_m) * (1.0f / DW1000_METERS_PER_TICK));
    new_ad = (int32_t)old_ad + corr;
    if (new_ad < 0) {
        new_ad = 0;
    }
    if (new_ad > 0xFFFF) {
        new_ad = 0xFFFF;
    }
    return (uint16_t)new_ad;
}

uint16_t dw1000_calibrate_antenna_delay(float known_distance_cm)
{
    uint16_t old_ad = dw1000_get_antenna_delay();
    uint16_t new_ad = dw1000_compute_ideal_antenna_delay(known_distance_cm);

    dw1000_set_antenna_delay(new_ad);
    dw1000_commit_config();   /* write TX_ANTD + LDE_RXANTD to the chip */

    ESP_LOGI(TAG, "Calibrate: known=%.2f m measured=%.2f m -> antenna_delay %u (was %u)",
             (double)(known_distance_cm / 100.0f), (double)s_last_distance,
             (unsigned)new_ad, (unsigned)old_ad);
    return new_ad;
}

// Tag side: tell the anchor to apply `new_ad` and wait for its CAL_ACK.
// Returns true when the anchor confirmed (and applied the same value).
static bool dw1000_send_cal_set_and_wait(uint16_t new_ad, uint16_t timeout_ms)
{
    uint8_t session = ++s_cal_session;
    TickType_t t0;
    uint8_t frame[DW1000_HDR_LEN + DW1000_LEN_DATA];

    s_cal_ack_received = false;
    s_cal_ack_ad = 0;

    memset(frame, 0, sizeof(frame));
    frame[DW1000_OFF_TYPE]        = DW1000_MSG_CAL_SET;
    frame[DW1000_OFF_CAL_SESSION] = session;
    frame[DW1000_OFF_CAL_AD]      = new_ad & 0xFF;
    frame[DW1000_OFF_CAL_AD + 1]  = (uint8_t)(new_ad >> 8);
    build_header(frame, s_pan_id, s_peer_eui, s_own_eui);
    dw1000_send(frame, DW1000_HDR_LEN + DW1000_LEN_DATA);

    t0 = xTaskGetTickCount();
    while (!s_cal_ack_received &&
           (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(timeout_ms)) {
        vTaskDelay(1);   /* >= 1 tick, see dw1000_run_tag() */
    }

    if (!s_cal_ack_received) {
        ESP_LOGW(TAG, "CALIBRATE: no CAL_ACK from the anchor within %u ms (session %u)",
                 (unsigned)timeout_ms, (unsigned)session);
        return false;
    }
    if (s_cal_ack_ad != new_ad) {
        ESP_LOGW(TAG, "CALIBRATE: anchor acked a DIFFERENT antenna_delay %u (sent %u)",
                 (unsigned)s_cal_ack_ad, (unsigned)new_ad);
    }
    return true;
}

uint16_t dw1000_calibrate_antenna_delay_iterative(
        int irq_gpio, float known_distance_cm,
        uint16_t convergence_threshold_ticks, int max_iterations,
        dw1000_distance_cb_t on_distance)
{
    uint16_t current_ad = dw1000_get_antenna_delay();
    uint16_t calc_ad = 0, new_ad = current_ad;
    bool converged = false, any_reading = false, any_acked = false;
    int iter;

    if (known_distance_cm <= 0.0f || max_iterations <= 0) {
        ESP_LOGE(TAG, "CALIBRATE: bad args (known_distance_cm=%d, max_iterations=%d)",
                 (int)known_distance_cm, max_iterations);
        return 0;
    }

    ESP_LOGI(TAG, "CALIBRATE: START - known distance %.2f m, initial antenna_delay=%u, "
                  "threshold <%u ticks, max %d iterations",
             (double)(known_distance_cm / 100.0f), (unsigned)current_ad,
             (unsigned)convergence_threshold_ticks, max_iterations);

    for (iter = 1; iter <= max_iterations; iter++) {
        ESP_LOGI(TAG, "CALIBRATE: ---- iteration %d/%d - current antenna_delay=%u",
                 iter, max_iterations, (unsigned)current_ad);

        /* 1) one ranging exchange -> measured distance */
        if (!dw1000_run_tag(irq_gpio, DW1000_RANGE_TIMEOUT_MS, on_distance)) {
            ESP_LOGW(TAG, "CALIBRATE: iteration %d - no range reading, retrying", iter);
            continue;
        }
        any_reading = true;
        ESP_LOGI(TAG, "CALIBRATE: iteration %d - measured %.3f m (known %.2f m)",
                 iter, (double)s_last_distance, (double)(known_distance_cm / 100.0f));

        /* 2) antenna delay that would make measured == known */
        calc_ad = dw1000_compute_ideal_antenna_delay(known_distance_cm);
        ESP_LOGI(TAG, "CALIBRATE: iteration %d - calculated ideal antenna_delay=%u",
                 iter, (unsigned)calc_ad);

        /* 3) average -> move halfway to the ideal (can never overshoot) */
        new_ad = (uint16_t)(((uint32_t)current_ad + calc_ad) >> 1);
        ESP_LOGI(TAG, "CALIBRATE: iteration %d - new antenna_delay=(%u+%u)/2=%u",
                 iter, (unsigned)current_ad, (unsigned)calc_ad, (unsigned)new_ad);

        /* 4) apply the shared value on THIS board */
        dw1000_set_antenna_delay(new_ad);
        dw1000_commit_config();
        ESP_LOGI(TAG, "CALIBRATE: iteration %d - applied antenna_delay=%u on this board",
                 iter, (unsigned)new_ad);

        /* 5) send it to the anchor and wait for its CAL_ACK */
        if (dw1000_send_cal_set_and_wait(new_ad, DW1000_CAL_ACK_TIMEOUT_MS)) {
            any_acked = true;
            ESP_LOGI(TAG, "CALIBRATE: iteration %d - anchor applied antenna_delay=%u (ACK)",
                     iter, (unsigned)new_ad);
        } else {
            ESP_LOGW(TAG, "CALIBRATE: iteration %d - anchor did NOT confirm; "
                          "antenna_delay=%u set on this board only",
                     iter, (unsigned)new_ad);
        }

        /* 6) convergence: |calc_ad - current_ad| below the threshold? */
        {
            int32_t diff = (int32_t)calc_ad - (int32_t)current_ad;
            if (diff < 0) {
                diff = -diff;
            }
            ESP_LOGI(TAG, "CALIBRATE: iteration %d - |calc - current| = %d ticks "
                          "(threshold %u)",
                     iter, (int)diff, (unsigned)convergence_threshold_ticks);
            current_ad = new_ad;
            if (diff < (int32_t)convergence_threshold_ticks) {
                converged = true;
                break;
            }
        }
    }

    if (converged) {
        ESP_LOGI(TAG, "CALIBRATE: CONVERGED after %d iteration(s) - "
                      "final shared antenna_delay=%u (put this on BOTH boards)",
                 iter, (unsigned)current_ad);
    } else if (!any_reading) {
        ESP_LOGE(TAG, "CALIBRATE: FAILED - never got a range reading");
        return 0;
    } else if (!any_acked) {
        ESP_LOGW(TAG, "CALIBRATE: finished without any anchor confirmation - "
                      "antenna_delay=%u set on this board only",
                 (unsigned)current_ad);
    } else {
        ESP_LOGW(TAG, "CALIBRATE: %d iteration(s) used without converging - "
                      "last antenna_delay=%u",
                 max_iterations, (unsigned)current_ad);
    }

    /* final sanity check: one more reading with the shared value */
    if (converged) {
        bool ok = dw1000_run_tag(irq_gpio, DW1000_RANGE_TIMEOUT_MS, on_distance);
        if (ok) {
            ESP_LOGI(TAG, "CALIBRATE: verify - measured %.3f m (known %.2f m) "
                          "with antenna_delay=%u",
                     (double)s_last_distance, (double)(known_distance_cm / 100.0f),
                     (unsigned)current_ad);
        } else {
            ESP_LOGW(TAG, "CALIBRATE: verify - no reading with the final value");
        }
    }
    return current_ad;
}