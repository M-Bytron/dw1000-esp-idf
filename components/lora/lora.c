/*
 * lora.c - LoRa radio, pairing (requester) and data sending for Device A.
 *
 * Device A is the pairing requester: it broadcasts a PAIR_REQUEST, and when
 * a Device B answers with a PAIR_RESPONSE it acknowledges with an MSG_PAIR_ACK;
 * on MSG_PAIR_CONFIRM the partner is saved to NVS. After pairing it can send
 * MSG_DATA packets (e.g. "hello") to the partner.
 *
 *   - lora_init() initializes the radio (SPI, module detection, LoRa
 *     parameters), sets up the device identity and loads the paired partner
 *     from NVS.
 *
 *   - LISTENING task ("lora_listen"): created after a successful pairing (and
 *     at boot when already paired). Receives messages from the paired device
 *     and forwards MSG_DATA packets to the registered data callback.
 *
 *   - PAIRING task ("lora_pair"): created when the device enters pairing mode
 *     (button). It stops the LISTENING task first, then broadcasts a
 *     PAIR_REQUEST every PAIR_REQUEST_INTERVAL_MS until Device B answers
 *     (PAIR_RESPONSE -> MSG_PAIR_ACK) and confirms (PAIR_CONFIRM -> partner
 *     saved). It exits on success or on the pairing-mode timeout; afterwards
 *     the LISTENING task is (re)created if a partner is stored.
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "nvs.h"
#include "sx127x.h"
#include "sx127x_registers.h"
#include "packet.h"
#include "device_identity.h"
#include "lora.h"

static const char *TAG = "lora";

/* The sx127x struct contains a large packet buffer, keep it in static
 * storage instead of on the stack to avoid overflowing task stacks. */
static sx127x s_device;

/* Set to true by tx_callback when a transmission finishes */
static volatile bool s_tx_done = false;

#define LORA_FREQUENCY 433000000UL   /* SX1278 module, 433 MHz band */

/* ---- This device's type (sent in the PAIR_REQUEST payload) ---- */
#define DEVICE_TYPE 'A'

/* ---- NVS namespace + key for the paired partner ID ---- */
#define NVS_NAMESPACE "pairing"
#define NVS_KEY_PARTNER "partner_id"

/* ---- Pairing mode timeout: leave pairing mode after 60 s ---- */
#define PAIR_MODE_TIMEOUT_MS 60000UL

/* ---- How often the requester (Device A) broadcasts a PAIR_REQUEST ---- */
#define PAIR_REQUEST_INTERVAL_MS 3000UL

/* ---- Max wait for the TxDone IRQ flag during a transmission ---- */
#define TX_TIMEOUT_MS 500UL

/* ---- Capacity of the ACK queue. Size 1: only the ACK for the single
 * in-flight message matters; any other value is consumed and discarded. ---- */
#define ACK_QUEUE_LENGTH 1

#define RX_BUFFER_SIZE 128

/* LoRa RF parameters (must match Device A) */
#define LORA_BANDWIDTH        SX127X_BW_125000
#define LORA_SPREADING_FACTOR SX127X_SF_7
#define LORA_CODING_RATE      SX127X_CR_4_5
#define LORA_SYNC_WORD        0x12
#define LORA_PREAMBLE_LENGTH  8
#define LORA_TX_POWER         14

#define LORA_TASK_STACK_SIZE  8192
#define LORA_TASK_PRIORITY    5

/* ---- Shared state ---- */
static uint64_t s_myDeviceID   = 0;
static uint16_t s_sendSequence = 0;
static uint64_t s_partnerID    = 0;
static volatile bool s_pairing = false;
static uint64_t s_pairing_start = 0;
static uint16_t s_pairing_msg_id = 0;   /* one ID shared by the whole handshake */

static TaskHandle_t s_listen_task = NULL;
static TaskHandle_t s_pair_task   = NULL;
static lora_data_callback_t s_data_cb = NULL;
static lora_pairing_callback_t s_pairing_cb = NULL;
static bool s_initialized = false;
static char s_device_role = 'B';


/* Serializes all access to the SX127x radio. The TX path (send_packet, called
 * from the app/sender task or the pairing task) and the RX path (listen/pairing
 * tasks polling sx127x_handle_interrupt) run in DIFFERENT FreeRTOS tasks. Without
 * this lock their multi-register operations interleave and wedge the radio, e.g.
 * the sender stops receiving ACKs after a while. */
static SemaphoreHandle_t s_radio_lock = NULL;

/* ACK queue: every MSG_ACK received on the listen path pushes the message ID
 * it acknowledges into s_ack_queue. lora_send_data_with_ack() blocks on this
 * queue (instead of polling a flag) until an ACK for its in-flight message ID
 * arrives or the per-attempt timeout elapses. */
static QueueHandle_t s_ack_queue = NULL;

/* RX storage filled by rx_callback, consumed by the active task */
static uint8_t s_rx_buffer[RX_BUFFER_SIZE];
static uint16_t s_rx_len = 0;
static volatile bool s_packet_ready = false;

static uint64_t now_ms(void) { return (uint64_t)(esp_timer_get_time() / 1000); }

/*----------------------------------------------*/
/* Random per-message ID (never 0). Every transmitted packet carries one; an
 * MSG_ACK echoes the ID of the message it acknowledges so the sender can
 * match ACKs to the message it currently has in flight. */
static uint16_t next_message_id(void)
{
    uint16_t id;
    do {
        id = (uint16_t)(esp_random() & 0xFFFFu);
    } while (id == 0);
    return id;
}

/*----------------------------------------------*/
/* Fill in the common packet header; the caller then sets the payload. */
static void init_packet(LoRaPacket *pkt, uint8_t type, uint64_t receiver,
                        uint16_t msg_id)
{
    memset(pkt, 0, sizeof(*pkt));
    pkt->version       = PACKET_VERSION;
    pkt->messageType   = type;
    pkt->messageID     = msg_id;
    pkt->senderID      = s_myDeviceID;
    pkt->receiverID    = receiver;
    pkt->sequence      = s_sendSequence++;
}

/*----------------------------------------------*/
static const char *format_id(uint64_t id);   /* forward decl (MAC string) */

/*----------------------------------------------*/
void module_reset(uint8_t reset_pin)
{
    ESP_ERROR_CHECK(gpio_set_direction(reset_pin, GPIO_MODE_OUTPUT));
    gpio_set_level(reset_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(reset_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "Module reset done (RST=GPIO%d)", reset_pin);
}

/*----------------------------------------------*/
/* Radio recovery after a TX timeout / wedged radio: re-apply the LoRa
 * configuration and settle back into continuous RX.
 *
 * NOTE: we deliberately do NOT hardware-reset the module here. A module
 * reset puts the SX127x back into FSK mode, but the driver's in-memory
 * modem state is not updated after the reset, so re-entering LoRa mode
 * silently fails and the radio gets stuck in FSK (OPMODE=0x03) where TX
 * never completes. A software re-config from the current LoRa state is
 * enough to recover and is safe. */
static void radio_reinit(void)
{
    sx127x_set_opmod(SX127X_MODE_STANDBY, SX127X_MODULATION_LORA, &s_device);
    sx127x_set_frequency(LORA_FREQUENCY, &s_device);
    sx127x_lora_reset_fifo(&s_device);
    sx127x_lora_set_bandwidth(LORA_BANDWIDTH, &s_device);
    sx127x_lora_set_spreading_factor(LORA_SPREADING_FACTOR, &s_device);
    sx127x_lora_set_syncword(LORA_SYNC_WORD, &s_device);
    sx127x_set_preamble_length(LORA_PREAMBLE_LENGTH, &s_device);
    sx127x_lora_set_implicit_header(NULL, &s_device);
    sx127x_tx_header_t header = {
        .enable_crc = true,
        .coding_rate = LORA_CODING_RATE,
    };
    sx127x_lora_tx_set_explicit_header(&header, &s_device);
    sx127x_tx_set_pa_config(SX127X_PA_PIN_BOOST, LORA_TX_POWER, &s_device);
    sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, &s_device);
}

/*----------------------------------------------*/
/* Force the radio back to a clean continuous-RX state and clear any stale
 * IRQ flags. Used after an ACK timeout so the sender always recovers to a
 * known-good listening state before the next send attempt. */
static void radio_force_rx(void)
{
    xSemaphoreTake(s_radio_lock, portMAX_DELAY);
    /* Re-apply the LoRa configuration and settle back into continuous RX.
     * No hardware reset: a reset desyncs the driver's modem state and
     * leaves the radio stuck in FSK mode (OPMODE=0x03, TX never completes). */
    radio_reinit();
    xSemaphoreGive(s_radio_lock);
    ESP_LOGW(TAG, "Radio recovered to continuous RX after ACK timeout");
}

/*----------------------------------------------*/
static void spi_init(spi_device_handle_t *handle, uint8_t ss_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t sck_pin)
{
//     spi_bus_config_t bus = {
//         .mosi_io_num = mosi_pin,
//         .miso_io_num = miso_pin,
//         .sclk_io_num = sck_pin,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//         .max_transfer_sz = 0,
//     };
    // ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 4E6,
        .spics_io_num = ss_pin,
        .queue_size = 16,
        .command_bits = 0,
        .address_bits = 8,
        .dummy_bits = 0,
        .mode = 0,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, handle));
}

/*----------------------------------------------*/
static void tx_callback(void *ctx) { s_tx_done = true; }

static void rx_callback(void *ctx, uint8_t *data, uint16_t len)
{
    if (len <= RX_BUFFER_SIZE) {
        memcpy(s_rx_buffer, data, len);
        s_rx_len = len;
        s_packet_ready = true;
        /* Diagnostic: log every packet the radio delivers so we can tell
         * whether an ACK is actually received (data[1] = message type). */
        ESP_LOGI(TAG, "RX radio: %u bytes, type=%u: %02X %02X %02X %02X %02X %02X",
                 (unsigned)len, (unsigned)(len > 1 ? data[1] : 0),
                 data[0], data[1], data[2], data[3], data[4], data[5]);
    } else {
        ESP_LOGW(TAG, "RX radio: packet too long (%u bytes) - dropped", (unsigned)len);
    }
}

/*----------------------------------------------*/
static void load_partner_from_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_OK) {
        err = nvs_get_u64(h, NVS_KEY_PARTNER, &s_partnerID);
        if (err != ESP_OK) {
            s_partnerID = 0;
            ESP_LOGW(TAG, "No partner stored in NVS (err 0x%X)", err);
        } else {
            ESP_LOGI(TAG, "Partner loaded from NVS: %s", format_id(s_partnerID));
        }
        nvs_close(h);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        s_partnerID = 0;
        ESP_LOGI(TAG, "No partner stored (first run)");
    } else {
        ESP_LOGE(TAG, "Failed to open NVS for reading partner (0x%X)", err);
    }
}

/*----------------------------------------------*/
static void save_partner_to_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u64(h, NVS_KEY_PARTNER, s_partnerID);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Partner saved to NVS: %s", format_id(s_partnerID));
    } else {
        ESP_LOGE(TAG, "Failed to open NVS for writing partner");
    }
}

/*----------------------------------------------*/
static const char *message_type_name(uint8_t type)
{
    switch (type) {
        case MSG_PING:          return "PING";
        case MSG_PAIR_ACK:      return "PAIR_ACK";
        case MSG_PAIR_REQUEST:  return "PAIR_REQUEST";
        case MSG_PAIR_RESPONSE: return "PAIR_RESPONSE";
        case MSG_DATA:          return "DATA";
        case MSG_PAIR_CONFIRM:  return "PAIR_CONFIRM";
        case MSG_ACK:           return "ACK";
        default:                return "UNKNOWN";
    }
}

/*----------------------------------------------*/
/* Format a 48-bit MAC/id as "XX:XX:XX:XX:XX:XX" (static buffer). */
static const char *format_id(uint64_t id)
{
    static char buf[18];
    device_identity_format_id(id, buf, sizeof(buf));
    return buf;
}

/*----------------------------------------------*/
/* Encode + transmit a packet, then return to RX mode.
 * Caller MUST hold s_radio_lock. */
static void send_packet_locked(const LoRaPacket *packet)
{
    uint8_t buffer[RX_BUFFER_SIZE];
    uint16_t len = packet_encode(packet, buffer);
    if (len == 0) {
        ESP_LOGW(TAG, "packet_encode failed - packet not sent");
        return;
    }

    if (sx127x_lora_tx_set_for_transmission(buffer, len, &s_device) != SX127X_OK) {
        ESP_LOGE(TAG, "TX set for transmission failed");
        return;
    }

    ESP_LOGI(TAG, "TX %s -> %llu (%u bytes)",
             message_type_name(packet->messageType),
             (unsigned long long)packet->receiverID, (unsigned)len);

    s_tx_done = false;
    if (sx127x_set_opmod(SX127X_MODE_TX, SX127X_MODULATION_LORA, &s_device) != SX127X_OK) {
        ESP_LOGE(TAG, "Failed to enter TX mode");
        return;
    }

    /* Wait for TxDone, polling the IRQ flags. */
    uint64_t start = now_ms();
        
    // uint64_t nowww;
    while (!s_tx_done) {
        // nowww = now_ms();
        sx127x_handle_interrupt(&s_device);
        if ((now_ms() - start) > TX_TIMEOUT_MS) {
            uint8_t reg_opmode = 0, reg_irq = 0;
            sx127x_read_register(REGOPMODE, &s_device.spi_device, &reg_opmode);
            sx127x_read_register(REGIRQFLAGS, &s_device.spi_device, &reg_irq);
            ESP_LOGE(TAG, "TX timeout (OPMODE=0x%02X IRQ=0x%02X) - no TxDone",
                     reg_opmode, reg_irq);
            ESP_LOGE(TAG, "Resetting radio to recover...");
            radio_reinit();
            return;   /* radio_reinit() already put the module into RX */
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "TX completed");

    /* ---- Robust return to RX ----
     * Don't jump straight from TX to RX_CONT: the SX127x can stop
     * listening (miss the next preamble) if the mode isn't settled
     * through STANDBY and the FIFO pointer isn't reset to the RX base.
     * This keeps RX reliable after every transmission. */
    sx127x_set_opmod(SX127X_MODE_STANDBY, SX127X_MODULATION_LORA, &s_device);
    sx127x_lora_reset_fifo(&s_device);
    sx127x_write_register(REGFIFOADDRPTR, 0x00, &s_device.spi_device); /* RX base = 0 */
    sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, &s_device);
}

/*----------------------------------------------*/
/* Locked wrapper: serializes TX against the listen/pairing tasks' interrupt
 * polling so the radio is never driven from two tasks at the same time. */
static void send_packet(const LoRaPacket *packet)
{
    xSemaphoreTake(s_radio_lock, portMAX_DELAY);
    send_packet_locked(packet);
    xSemaphoreGive(s_radio_lock);
}


/*----------------------------------------------*/
/* Broadcast a PAIR_REQUEST (Device A / requester role) */
static void send_pair_request(void)
{
    LoRaPacket pkt;
    s_pairing_msg_id = next_message_id();
    init_packet(&pkt, MSG_PAIR_REQUEST, BROADCAST_ID, s_pairing_msg_id);
    pkt.payloadLength = 1;
    pkt.payload[0] = s_device_role;  // Use the static variable
    send_packet(&pkt);
    ESP_LOGI(TAG, "PAIR_REQUEST broadcast (msg %u) as role %c", 
             s_pairing_msg_id, s_device_role);
}

/*----------------------------------------------*/
/* Send a PAIR_RESPONSE (Device B / responder role) */
static void send_pair_response(uint64_t target_id, uint32_t request_msg_id)
{
    LoRaPacket pkt;
    init_packet(&pkt, MSG_PAIR_RESPONSE, target_id, request_msg_id);
    pkt.payloadLength = 1;
    pkt.payload[0] = s_device_role;  // Use the static variable
    send_packet(&pkt);
    ESP_LOGI(TAG, "PAIR_RESPONSE -> %s (msg %lu) as role %c", 
             format_id(target_id), request_msg_id, s_device_role);
}

/*----------------------------------------------*/
/* Send a PAIR_ACK (Device A / requester role) */
static void send_pair_ack(uint64_t target_id)
{
    LoRaPacket pkt;
    init_packet(&pkt, MSG_PAIR_ACK, target_id, s_pairing_msg_id);
    pkt.payloadLength = 0;
    send_packet(&pkt);
    ESP_LOGI(TAG, "PAIR_ACK -> %s (msg %u)", format_id(target_id), s_pairing_msg_id);
}

/*----------------------------------------------*/
/* Send a PAIR_CONFIRM (Device B / responder role) */
static void send_pair_confirm(uint64_t target_id)
{
    LoRaPacket pkt;
    init_packet(&pkt, MSG_PAIR_CONFIRM, target_id, s_pairing_msg_id);
    pkt.payloadLength = 0;
    send_packet(&pkt);
    ESP_LOGI(TAG, "PAIR_CONFIRM -> %s (msg %u)", format_id(target_id), s_pairing_msg_id);
}

/*----------------------------------------------*/
/* Process one received packet while in PAIRING mode (requester). */
static void handle_pairing_packet(const uint8_t *buf, uint16_t len)
{

    LoRaPacket packet;
    if (!packet_decode(buf, len, &packet)) {
        return;
    }

    ESP_LOGI(TAG, "RX %s from %s", message_type_name(packet.messageType),
             format_id(packet.senderID));

    switch (packet.messageType) {
        case MSG_PAIR_REQUEST:
            /* Device A is requesting pairing */
            ESP_LOGW("Pairing", "MSG_PAIR_REQUEST");
            if (packet.payloadLength >= 1 && packet.payload[0] == 'A') {
                ESP_LOGI(TAG, "Received PAIR_REQUEST from A (msg %u) - sending response",
                         packet.messageID);
                s_partnerID = packet.senderID;
                s_pairing_msg_id = packet.messageID;  // Echo the same ID
                send_pair_response(packet.senderID, packet.messageID);
            } else {
                ESP_LOGW(TAG, "Invalid PAIR_REQUEST payload: expected 'A', got 0x%02X",
                         packet.payload[0]);
            }
            break;

        case MSG_PAIR_RESPONSE:
            /* A B device answered our PAIR_REQUEST: acknowledge it with the
             * same handshake message ID (s_pairing_msg_id). Device B should
             * echo the PAIR_REQUEST's messageID here so the whole handshake
             * shares one ID. */
            ESP_LOGW("Pairing", "MSG_PAIR_RESPONSE");
            if (packet.payloadLength >= 1 && packet.payload[0] == 'B') {
                ESP_LOGI(TAG, "Valid PAIR_RESPONSE from B (msg %u) - sending ACK",
                         packet.messageID);
                send_pair_ack(packet.senderID);
            }
            break;

        case MSG_PAIR_ACK:
            /* Device A acknowledged our response */
            ESP_LOGW("Pairing", "MSG_PAIR_ACK");
            if (packet.senderID == s_partnerID) {
                ESP_LOGI(TAG, "Received PAIR_ACK from A - sending CONFIRM");
                send_pair_confirm(packet.senderID);
                save_partner_to_nvs();
                s_pairing = false;
                ESP_LOGI(TAG, "---------------------------------");
                ESP_LOGW(TAG, "[PAIRING] SUCCESS - paired with %s", 
                         format_id(s_partnerID));
                ESP_LOGI(TAG, "---------------------------------");
                if (s_pairing_cb) {
                    s_pairing_cb(s_partnerID);
                }
            } else {
                ESP_LOGW(TAG, "PAIR_ACK from unknown sender %s (expected %s)",
                         format_id(packet.senderID), format_id(s_partnerID));
            }
            break;

        case MSG_PAIR_CONFIRM:
            ESP_LOGW("Pairing", "MSG_PAIR_CONFIRM");
            /* B confirmed the pairing: save B's MAC to NVS and leave
             * pairing mode. */
            s_partnerID = packet.senderID;   /* B's MAC */
            save_partner_to_nvs();           /* persist the sender's MAC */
            s_pairing = false;               /* leave pairing mode */
            {
                char idbuf[18];
                device_identity_format_id(s_partnerID, idbuf, sizeof(idbuf));
                ESP_LOGW(TAG, "---------------------------------------");
                ESP_LOGW(TAG, "[PAIRING] SUCCESS - paired with %s", idbuf);
                ESP_LOGW(TAG, "---------------------------------------");
            }
            if (s_pairing_cb) {
                s_pairing_cb(s_partnerID);
            }
            break;

        default:
            ESP_LOGW(TAG, "Unexpected packet type %d in responder mode", 
                     packet.messageType);
            break;

    }

    // LoRaPacket packet;
    // if (!packet_decode(buf, len, &packet)) {
    //     return;
    // }

    // ESP_LOGI(TAG, "RX %s from %s", message_type_name(packet.messageType),
    //          format_id(packet.senderID));

    // switch (packet.messageType) {
    //     case MSG_PAIR_RESPONSE:
    //         /* A B device answered our PAIR_REQUEST: acknowledge it with the
    //          * same handshake message ID (s_pairing_msg_id). Device B should
    //          * echo the PAIR_REQUEST's messageID here so the whole handshake
    //          * shares one ID. */
    //         if (packet.payloadLength >= 1 && packet.payload[0] == 'B') {
    //             ESP_LOGI(TAG, "Valid PAIR_RESPONSE from B (msg %u) - sending ACK",
    //                      packet.messageID);
    //             send_pair_ack(packet.senderID);
    //         }
    //         break;

    //     case MSG_PAIR_CONFIRM:
    //         /* B confirmed the pairing: save B's MAC to NVS and leave
    //          * pairing mode. */
    //         s_partnerID = packet.senderID;   /* B's MAC */
    //         save_partner_to_nvs();           /* persist the sender's MAC */
    //         s_pairing = false;               /* leave pairing mode */
    //         {
    //             char idbuf[18];
    //             device_identity_format_id(s_partnerID, idbuf, sizeof(idbuf));
    //             ESP_LOGW(TAG, "---------------------------------------");
    //             ESP_LOGW(TAG, "[PAIRING] SUCCESS - paired with %s", idbuf);
    //             ESP_LOGW(TAG, "---------------------------------------");
    //         }
    //         break;

    //     // case MSG_PAIR_ACK:
    //     //     /* B acked something else while pairing - not needed. */
    //     //     ESP_LOGI(TAG, "Got ACK from %s while pairing", format_id(packet.senderID));
    //     //     break;

    //     ///////////////////////////////////////
    //     case MSG_PAIR_REQUEST:
    //                 /* Device A is requesting pairing */
    //                 if (packet.payloadLength >= 1 && packet.payload[0] == 'A') {
    //                     ESP_LOGI(TAG, "Received PAIR_REQUEST from A (msg %u) - sending response",
    //                              packet.messageID);
    //                     s_partnerID = packet.senderID;
    //                     s_pairing_msg_id = packet.messageID;  // Echo the same ID
    //                     send_pair_response(packet.senderID, packet.messageID);
    //                 } else {
    //                     ESP_LOGW(TAG, "Invalid PAIR_REQUEST payload: expected 'A', got 0x%02X",
    //                              packet.payload[0]);
    //                 }
    //                 break;

    //             case MSG_PAIR_ACK:
    //                 /* Device A acknowledged our response */
    //                 if (packet.senderID == s_partnerID) {
    //                     ESP_LOGI(TAG, "Received PAIR_ACK from A - sending CONFIRM");
    //                     send_pair_confirm(packet.senderID);
    //                     save_partner_to_nvs();
    //                     s_pairing = false;
    //                     ESP_LOGI(TAG, "---------------------------------");
    //                     ESP_LOGW(TAG, "[PAIRING] SUCCESS - paired with %s", 
    //                              format_id(s_partnerID));
    //                     ESP_LOGI(TAG, "---------------------------------");
    //                 } else {
    //                     ESP_LOGW(TAG, "PAIR_ACK from unknown sender %s (expected %s)",
    //                              format_id(packet.senderID), format_id(s_partnerID));
    //                 }
    //                 break;

    //             // default:
    //             //     ESP_LOGW(TAG, "Unexpected packet type %d in responder mode", 
    //             //              packet.messageType);
    //             //     break;
                
    //     //////////////////////////////////////////////

    //     default:
    //         break;
    // }
}

/*----------------------------------------------*/
/* Process one received packet while LISTENING to the paired device. */
static void handle_listen_packet(const uint8_t *buf, uint16_t len)
{
    LoRaPacket packet;
    if (!packet_decode(buf, len, &packet)) {
        ESP_LOGW(TAG, "RX: packet_decode failed (%u bytes)", (unsigned)len);
        return;
    }

    /* Only messages from the paired device are of interest. */
    if (packet.senderID != s_partnerID) {
        ESP_LOGW(TAG, "RX %s from non-partner %s (len %u) - ignored",
                 message_type_name(packet.messageType),
                 format_id(packet.senderID), (unsigned)len);
        return;
    }

    ESP_LOGI(TAG, "RX %s from partner %s",
             message_type_name(packet.messageType),
             format_id(packet.senderID));

    int16_t rssi = 0;
    sx127x_rx_get_packet_rssi(&s_device, &rssi);

    switch (packet.messageType) {
        case MSG_DATA:
            ESP_LOGI("LORA_RX","_____________ MSG_DATA _______________");
            /* Application data from the paired device -> user callback.
             * ACK it with the SAME message ID so the sender can match it. */
            lora_send_ack(packet.messageID);
            if (s_data_cb) {
                s_data_cb(packet.messageType, packet.payload, packet.payloadLength,
                          packet.senderID, rssi);
            }
            break;

        case MSG_ACK:
            ESP_LOGE("LORA_RX","_____________ MSG_ACK _______________");
            /* An ACK for one of our messages: push the acknowledged message
             * ID into the ACK queue. lora_send_data_with_ack() consumes it
             * and matches it against the message it is currently waiting
             * for; stale/duplicate IDs are drained by the waiter. */
            if (xQueueSend(s_ack_queue, &packet.messageID, 0) != pdTRUE) {
                ESP_LOGW(TAG, "ACK queue full - dropped ACK for msg %u",
                         packet.messageID);
            }
            break;

        default:
            ESP_LOGI("LORA_RX","_____________ Default _______________");
            ESP_LOGW(TAG, "HESSAT (not waiting)");
            break;
    }
}

/*----------------------------------------------*/
/* LISTENING task: receives messages from the paired device. */
static void listen_task(void *arg)
{
    ESP_LOGI(TAG, "Listening task started (partner %s)", format_id(s_partnerID));
    xSemaphoreTake(s_radio_lock, portMAX_DELAY);
    sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, &s_device);
    xSemaphoreGive(s_radio_lock);

    for (;;) {
        /* If pairing mode was requested (button or auto-answer) while this
         * task was running, exit cleanly - the pairing task now owns the
         * radio. Never keep running after deleting ourselves. */
        if (s_pairing) {
            ESP_LOGI(TAG, "Listening task stopping (pairing mode)");
            s_listen_task = NULL;
            vTaskDelete(NULL);
        }

        /* Poll the radio under the lock so the TX path (send_packet) and
         * this RX path never manipulate the radio at the same time. */
        bool ready = false;
        xSemaphoreTake(s_radio_lock, portMAX_DELAY);
        sx127x_handle_interrupt(&s_device);
        if (s_packet_ready) {
            s_packet_ready = false;
            ready = true;
        }
        xSemaphoreGive(s_radio_lock);

        if (ready) {
            handle_listen_packet(s_rx_buffer, s_rx_len);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void lora_set_device_role(char role)
{
    if (role == 'A' || role == 'B') {
        s_device_role = role;
        ESP_LOGI(TAG, "Device role set to %c", s_device_role);
    } else {
        ESP_LOGE(TAG, "Invalid role: %c (must be 'A' or 'B')", role);
        s_device_role = 'A';  // Default to A on error
    }
}

char lora_get_device_role(void)
{
    return s_device_role;
}

/*----------------------------------------------*/
static void start_listening(void);   /* forward declaration */

/*----------------------------------------------*/
/* PAIRING task: runs the Device A requester handshake - broadcasts a
 * PAIR_REQUEST every PAIR_REQUEST_INTERVAL_MS, answers a PAIR_RESPONSE with
 * an MSG_PAIR_ACK and saves the partner on MSG_PAIR_CONFIRM. */
static void run_requester_pairing(void)
{
    xSemaphoreTake(s_radio_lock, portMAX_DELAY);
    sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, &s_device);
    xSemaphoreGive(s_radio_lock);

    uint64_t last_req = 0;

    while (s_pairing) {
        /* Broadcast a PAIR_REQUEST every PAIR_REQUEST_INTERVAL_MS */
        if ((now_ms() - last_req) >= PAIR_REQUEST_INTERVAL_MS) {
            last_req = now_ms();
            send_pair_request();
        }

        bool ready = false;
        xSemaphoreTake(s_radio_lock, portMAX_DELAY);
        sx127x_handle_interrupt(&s_device);
        if (s_packet_ready) {
            s_packet_ready = false;
            ready = true;
        }
        xSemaphoreGive(s_radio_lock);

        if (ready) {
            handle_pairing_packet(s_rx_buffer, s_rx_len);
        }

        /* Leave pairing mode after the timeout */
        if (s_pairing && (now_ms() - s_pairing_start >= PAIR_MODE_TIMEOUT_MS)) {
            s_pairing = false;
            ESP_LOGE(TAG, "[PAIRING] FAILED - timeout (no response)");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*----------------------------------------------*/
/* Run pairing as responder (Device B). 
 * Waits for PAIR_REQUEST from Device A, responds with PAIR_RESPONSE,
 * waits for PAIR_ACK, then sends PAIR_CONFIRM.
 */
static void run_responder_pairing(void)
{
    xSemaphoreTake(s_radio_lock, portMAX_DELAY);
    sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, &s_device);
    xSemaphoreGive(s_radio_lock);

    ESP_LOGI(TAG, "Responder B listening for PAIR_REQUEST...");

    while (s_pairing) {
        bool ready = false;
        xSemaphoreTake(s_radio_lock, portMAX_DELAY);
        sx127x_handle_interrupt(&s_device);
        if (s_packet_ready) {
            s_packet_ready = false;
            ready = true;
        }
        xSemaphoreGive(s_radio_lock);

        if (ready) {
            handle_pairing_packet(s_rx_buffer, s_rx_len);
        }

        /* Leave pairing mode after the timeout */
        if (s_pairing && (now_ms() - s_pairing_start >= PAIR_MODE_TIMEOUT_MS)) {
            s_pairing = false;
            ESP_LOGE(TAG, "[PAIRING] FAILED - timeout (no request/ack received)");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void pairing_task()
{
    if (s_device_role  == 'A') {
        ESP_LOGW(TAG, "Pairing task started (requester A)");
        // Act as A: send PAIR_REQUEST, wait for PAIR_RESPONSE
        run_requester_pairing();
    } else {
        ESP_LOGW(TAG, "Pairing task started (responder B)");
        // Act as B: wait for PAIR_REQUEST, respond with PAIR_RESPONSE
        run_responder_pairing();
    }
    
    // Cleanup after pairing
    if (s_partnerID != 0) {
        start_listening();
    }
    s_pair_task = NULL;
    vTaskDelete(NULL);
}

/*----------------------------------------------*/
static void start_listening(void)
{
    if (s_listen_task != NULL) {
        return;
    }
    xTaskCreate(listen_task, "lora_listen", LORA_TASK_STACK_SIZE, NULL,
                LORA_TASK_PRIORITY, &s_listen_task);
}

/*----------------------------------------------*/
bool lora_init(uint8_t ss_pin, uint8_t rst_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t sck_pin)
{
    if (s_initialized) {
        return true;
    }

    ESP_LOGI(TAG, "Device A LoRa starting...");

    s_radio_lock = xSemaphoreCreateMutex();
    if (s_radio_lock == NULL) {
        ESP_LOGE(TAG, "Failed to create radio mutex");
        return false;
    }

    /* ACK queue: stores the message IDs of received ACKs. */
    s_ack_queue = xQueueCreate(ACK_QUEUE_LENGTH, sizeof(uint16_t));
    if (s_ack_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create ACK queue");
        return false;
    }

    module_reset(rst_pin);

    spi_device_handle_t spi_dev;
    spi_init(&spi_dev, ss_pin, mosi_pin, miso_pin, sck_pin);

    /* ---- Module detection ---- */
    int code = sx127x_create(spi_dev, &s_device);
    if (code != SX127X_OK) {
        ESP_LOGE(TAG, ">>> MODULE NOT DETECTED (sx127x_create error 0x%X)", code);
        ESP_LOGE(TAG, ">>> Check wiring (SPI pins, RST, power) and that the module is powered");
        return false;
    }
    ESP_LOGI(TAG, ">>> SX127x MODULE DETECTED, chip version 0x%02X", s_device.chip_version);

    /* ---- Configure LoRa mode (must match Device B) ---- */
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_STANDBY, SX127X_MODULATION_LORA, &s_device));
    ESP_ERROR_CHECK(sx127x_set_frequency(LORA_FREQUENCY, &s_device));
    ESP_ERROR_CHECK(sx127x_lora_reset_fifo(&s_device));
    ESP_ERROR_CHECK(sx127x_lora_set_bandwidth(LORA_BANDWIDTH, &s_device));
    ESP_ERROR_CHECK(sx127x_lora_set_spreading_factor(LORA_SPREADING_FACTOR, &s_device));
    ESP_ERROR_CHECK(sx127x_lora_set_syncword(LORA_SYNC_WORD, &s_device));
    ESP_ERROR_CHECK(sx127x_set_preamble_length(LORA_PREAMBLE_LENGTH, &s_device));
    ESP_ERROR_CHECK(sx127x_lora_set_implicit_header(NULL, &s_device)); /* explicit */

    sx127x_tx_header_t header = {
        .enable_crc = true,
        .coding_rate = LORA_CODING_RATE,
    };
    ESP_ERROR_CHECK(sx127x_lora_tx_set_explicit_header(&header, &s_device));
    ESP_ERROR_CHECK(sx127x_tx_set_pa_config(SX127X_PA_PIN_BOOST, LORA_TX_POWER, &s_device));
    sx127x_tx_set_callback(tx_callback, &s_device, &s_device);
    sx127x_rx_set_callback(rx_callback, &s_device, &s_device);

    /* ---- Diagnostic: read back and log the actual RF registers so both
     * boards can be verified to be on the same frequency / sync word. ---- */
    uint8_t frf[3] = {0}, sync = 0;
    sx127x_read_register(REGFRFMSB, &s_device.spi_device, &frf[0]);
    sx127x_read_register(REGFRFMID, &s_device.spi_device, &frf[1]);
    sx127x_read_register(REGFRFLSB, &s_device.spi_device, &frf[2]);
    sx127x_read_register(REGSYNCWORD, &s_device.spi_device, &sync);
    uint32_t frf_val = ((uint32_t)frf[0] << 16) | ((uint32_t)frf[1] << 8) | frf[2];
    uint32_t freq_hz = (uint32_t)(((uint64_t)frf_val * 32000000ULL) >> 19);
    ESP_LOGI(TAG, "Radio: FRF=0x%06lX -> %lu Hz, sync=0x%02X",
             (unsigned long)frf_val, (unsigned long)freq_hz, sync);

    /* ---- Device Identity Manager ---- */
    if (device_identity_init()) {
        s_myDeviceID = device_identity_get_id();
        char idbuf[18];
        device_identity_format_id(s_myDeviceID, idbuf, sizeof(idbuf));
        ESP_LOGI(TAG, "Device ID (MAC): %s", idbuf);
    } else {
        ESP_LOGE(TAG, "Device Identity init failed!");
        s_myDeviceID = 0;
    }

    /* ---- Load paired device from NVS (if any) ---- */
    load_partner_from_nvs();
    if (s_partnerID != 0) {
        char idbuf[18];
        device_identity_format_id(s_partnerID, idbuf, sizeof(idbuf));
        ESP_LOGI(TAG, "Paired with: %s", idbuf);
    } else {
        ESP_LOGI(TAG, "Not paired");
    }

    s_initialized = true;

    /* Always listen: when unpaired this lets us auto-answer an incoming
     * PAIR_REQUEST (and enter pairing mode), so a remote Device A can
     * complete pairing without our pairing button being held. */
    start_listening();

    return true;
}

bool lora_is_initialized(void) { return s_initialized; }
bool lora_is_paired(void)      { return s_partnerID != 0; }
bool lora_is_pairing(void)     { return s_pairing; }

lora_state_t lora_get_state(void)
{
    if (s_partnerID != 0) return LORA_PAIRED;
    if (s_pairing)        return LORA_PAIRING;
    return LORA_UNPAIRED;
}

uint64_t lora_get_partner_id(void) { return s_partnerID; }

bool lora_enter_pairing(void)
{
    ESP_LOGW(TAG, "Button lora_enter_pairing");
    if (!s_initialized) {
        ESP_LOGE(TAG, "enter_pairing: radio not initialized");
        return false;
    }
    if (s_pairing) {
        return true;   /* already in pairing mode */
    }

    TaskHandle_t self = xTaskGetCurrentTaskHandle();

    /* Shut down the LISTENING task while pairing. If we are being called
     * from the listen task itself (auto-answer of a PAIR_REQUEST), do NOT
     * delete the running task here - it will notice s_pairing and delete
     * itself cleanly. */
    if (s_listen_task != NULL && s_listen_task != self) {
        vTaskDelete(s_listen_task);
        s_listen_task = NULL;
        ESP_LOGW(TAG, "Listening task stopped (entering pairing mode)");
    } else if (s_listen_task == self) {
        s_listen_task = NULL;   /* the running task will delete itself */
    }

    s_pairing = true;
    s_pairing_start = now_ms();
    if (s_partnerID != 0) {
        ESP_LOGI(TAG, "-> Re-pairing (existing partner will be overwritten)...");
    } else {
        ESP_LOGW(TAG, "-> Entering pairing mode...");
    }

    if (s_pair_task == NULL) {

        ESP_LOGW("Debug", "pairing_task");
        xTaskCreate(pairing_task, "lora_pair", LORA_TASK_STACK_SIZE, NULL,
                    LORA_TASK_PRIORITY, &s_pair_task);
    }
    return true;
}

void lora_stop_pairing(void)
{
    s_pairing = false;
    /* The pairing task notices the flag, exits, and restarts the listening
     * task if a partner is stored. */
}

void lora_clear_pairing(void)
{
    s_partnerID = 0;
    s_pairing = false;

    /* Erase the partner from NVS. */
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY_PARTNER);
        nvs_commit(h);
        nvs_close(h);
    }

    /* Stop the listening task. */
    if (s_listen_task != NULL) {
        vTaskDelete(s_listen_task);
        s_listen_task = NULL;
    }
    ESP_LOGI(TAG, "Pairing cleared");
}

void lora_set_pairing_callback(lora_pairing_callback_t cb)
{
    s_pairing_cb = cb;
}

void lora_set_data_callback(lora_data_callback_t cb)
{
    s_data_cb = cb;
}

/*----------------------------------------------*/
/* Build + send an MSG_DATA packet to the paired partner with the given
 * message ID. The caller keeps the ID stable across retries so the ACK for
 * the whole logical message can be matched against it. */
static bool send_data_msg(const uint8_t *payload, uint8_t len, uint16_t msg_id)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "send_data: radio not initialized");
        return false;
    }
    if (len > MAX_PAYLOAD_LENGTH) {
        ESP_LOGE(TAG, "send_data: payload too large (%u > %d)",
                 (unsigned)len, MAX_PAYLOAD_LENGTH);
        return false;
    }

    LoRaPacket pkt;
    init_packet(&pkt, MSG_DATA, s_partnerID, msg_id);
    pkt.payloadLength = len;
    if (len > 0) {
        memcpy(pkt.payload, payload, len);
    }
    send_packet(&pkt);

    ESP_LOGI(TAG, "DATA sent to %s (%u bytes, msg=%u)",
             format_id(s_partnerID), (unsigned)len, msg_id);
    return true;
}

bool lora_send_data(const uint8_t *payload, uint8_t len)
{
    /* Fire-and-forget send: each call gets a fresh random message ID. */
    return send_data_msg(payload, len, next_message_id());
}

/*----------------------------------------------*/
/* Upper-level (acknowledged) send: transmit an MSG_DATA carrying a fresh
 * random message ID, then block on the ACK queue waiting for the partner's
 * MSG_ACK to echo that same ID. The timeout is the queue's own blocking
 * timeout (LORA_ACK_TIMEOUT_MS, 200 ms) - there is no caller timeout. An ACK
 * with a non-matching ID is consumed and ignored; when the queue wait times
 * out the payload is re-sent (same ID), up to `retries` times. */
esp_err_t lora_send_data_with_ack(const uint8_t *payload, uint8_t len, uint16_t LORA_ACK_TIMEOUT_MS,
                                  uint8_t retries)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "send_data_with_ack: radio not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_partnerID == 0) {
        ESP_LOGW(TAG, "send_data_with_ack: no partner paired");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ack_queue == NULL) {
        ESP_LOGE(TAG, "send_data_with_ack: ack queue not created");
        return ESP_ERR_INVALID_STATE;
    }

    /* The message ID must stay fixed until the ACK arrives: generate it ONCE
     * and reuse it for every retransmission of this logical message. */
    uint16_t msg_id = next_message_id();

    /* The timeout is the ACK queue's own blocking timeout (LORA_ACK_TIMEOUT_MS,
     * 200 ms by default); there is no caller-supplied timeout. Each attempt
     * sends the payload (same msg_id) then waits on the queue: an ACK that
     * matches msg_id wins; a non-matching (stale/duplicate) ACK is consumed and
     * we keep waiting; if the wait itself times out with no ACK, re-send. The
     * total number of transmissions is retries + 1. */
    for (uint8_t attempt = 0; attempt <= retries; attempt++) {

        /* Flush stale ACK IDs left in the size-1 queue (from a previous
         * message or an earlier retry) so the matching ACK can always be
         * queued and is never dropped because the queue is already full. */
        uint16_t discard;
        while (xQueueReceive(s_ack_queue, &discard, 0) == pdTRUE) { }

        if (!send_data_msg(payload, len, msg_id)) {
            ESP_LOGE(TAG, "send_data_with_ack: lower-level send failed");
            return ESP_ERR_INVALID_STATE;
        }

        /* Per-attempt ACK budget: LORA_ACK_TIMEOUT_MS (200 ms) total. A stale
         * ACK is consumed and we keep waiting for only the REMAINING time, so
         * the whole attempt never exceeds the budget. */
        TickType_t start_time = xTaskGetTickCount();
        uint32_t timeout_ms = LORA_ACK_TIMEOUT_MS;

        for (;;) {
            uint32_t elapsed_ms = pdTICKS_TO_MS(xTaskGetTickCount() - start_time);
            uint32_t remaining_ms = (elapsed_ms >= timeout_ms) ? 0 : (timeout_ms - elapsed_ms);

            uint16_t acked_id = 0;
            if (xQueueReceive(s_ack_queue, &acked_id,
                              pdMS_TO_TICKS(remaining_ms)) != pdTRUE) {
                /* The remaining budget elapsed with no ACK -> this attempt
                 * failed, re-send (next loop iteration). */
                break;
            }

            if (acked_id == msg_id) {
                ESP_LOGI(TAG, "ACK received for msg %u (attempt %u)",
                         msg_id, (unsigned)(attempt + 1));
                return ESP_OK;
            }

            /* Non-matching ACK: xQueueReceive already removed it from the
             * queue; keep waiting for the remaining budget. */
            ESP_LOGI(TAG, "Dropped stale ACK for msg %u (waiting for %u)",
                     acked_id, msg_id);
        }

        if (attempt < retries) {
            ESP_LOGW(TAG, "No ACK within %lu ms (attempt %u/%u) - retrying",
                     (unsigned long)LORA_ACK_TIMEOUT_MS,
                     (unsigned)(attempt + 1), (unsigned)(retries + 1));
        }
    }

    ESP_LOGE(TAG, "send_data_with_ack: no ACK for msg %u after %u attempts",
             msg_id, (unsigned)(retries + 1));

    /* Diagnostic: read the radio state before recovering. The REGIRQFLAGS read
     * clears the flags, which is fine here (we are giving up on this ACK). */
    {
        uint8_t reg_opmode = 0, reg_irq = 0, reg_rxnb = 0;
        sx127x_read_register(REGOPMODE, &s_device.spi_device, &reg_opmode);
        sx127x_read_register(REGIRQFLAGS, &s_device.spi_device, &reg_irq);
        sx127x_read_register(REGRXNBBYTES, &s_device.spi_device, &reg_rxnb);
        ESP_LOGE(TAG, "ACK timeout radio state: OPMODE=0x%02X IRQ=0x%02X RXNBBYTES=%u",
                 reg_opmode, reg_irq, (unsigned)reg_rxnb);
    }

    /* Force the radio back into a clean listening state so the next send
     * starts from a known-good RX. A full module re-init is used because
     * re-asserting RX_CONT alone does not always recover a wedged radio. */
    radio_force_rx();

    return ESP_ERR_TIMEOUT;
}

/*----------------------------------------------*/
/* Send an MSG_ACK for a received message. The ACK echoes `msg_id` (the
 * messageID of the message being acknowledged) so the sender can match it. */
bool lora_send_ack(uint16_t msg_id)
{
    ESP_LOGI(TAG, "Trying to send ack (msg %u)", msg_id);
    if (!s_initialized) {
        ESP_LOGE(TAG, "send_ack: radio not initialized");
        return false;
    }

    LoRaPacket pkt;
    init_packet(&pkt, MSG_ACK, s_partnerID, msg_id);
    pkt.payloadLength = 0;
    send_packet(&pkt);

    ESP_LOGI(TAG, "ACK sent to %s (msg %u)", format_id(s_partnerID), msg_id);
    return true;
}
