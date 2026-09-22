/*
 * lora.h - LoRa radio, pairing (requester) and data sending for Device A.
 */
#ifndef LORA_H
#define LORA_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ------------------------------------------------------------------
 * LoRa state (used by the web server to report pairing status).
 * ------------------------------------------------------------------ */
typedef enum {
    LORA_UNPAIRED = 0,   /* no partner, not in pairing mode */
    LORA_PAIRING  = 1,   /* pairing mode active (pairing task running) */
    LORA_PAIRED   = 2    /* paired with a partner (listening task running) */
} lora_state_t;

typedef struct {
    bool as_requester;
} PairingTaskParams;

/* ------------------------------------------------------------------
 * Initializes the LoRa radio: resets the module, initializes SPI,
 * detects the SX127x, configures the LoRa parameters (must match Device
 * A), sets up the device identity and loads the paired partner from NVS.
 * If a partner is already stored, the listening task is started.
 * Returns true if the radio module was detected and configured.
 * ------------------------------------------------------------------ */
bool lora_init(uint8_t ss_pin, uint8_t rst_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t sck_pin);

/* True once lora_init() has successfully initialized the radio. */
bool lora_is_initialized(void);

/* True when paired (a partner device is stored). */
bool lora_is_paired(void);

/* True while the device is in pairing mode. */
bool lora_is_pairing(void);

/* Current LoRa state (UNPAIRED / PAIRING / PAIRED). */
lora_state_t lora_get_state(void);

/* Paired device ID (0 if not paired). */
uint64_t lora_get_partner_id(void);

/* ------------------------------------------------------------------
 * Enters pairing mode (requester role). Stops the LISTENING task (if
 * running) and starts the PAIRING task, which broadcasts a PAIR_REQUEST
 * every few seconds and answers Device B's PAIR_RESPONSE with an MSG_PAIR_ACK;
 * on MSG_PAIR_CONFIRM the partner is saved to NVS and the LISTENING task
 * is (re)started. On timeout the LISTENING task is restarted if a partner
 * is still stored. Returns false if the radio is not initialized.
 * ------------------------------------------------------------------ */
bool lora_enter_pairing(void);

/* Leaves pairing mode (e.g. from the web page). The pairing task exits
 * and the listening task is restarted if a partner is stored. */
void lora_stop_pairing(void);

/* Forgets the partner (unpair) and stops the listening task. */
void lora_clear_pairing(void);

/* ------------------------------------------------------------------
 * Sends an MSG_DATA packet carrying an application payload ("hello",
 * sensor readings, ...) to the paired partner. The payload is transmitted
 * immediately (blocking until TxDone) and the radio returns to RX mode
 * afterwards. Returns false if the radio is not initialized or the
 * payload is too large ( > MAX_PAYLOAD_LENGTH bytes).
 * ------------------------------------------------------------------ */
bool lora_send_data(const uint8_t *payload, uint8_t len);

/* ------------------------------------------------------------------
 * ACK wait timeout / retry count used by the acknowledged send. The timeout
 * is the ACK queue's own blocking timeout: lora_send_data_with_ack() waits
 * LORA_ACK_TIMEOUT_MS on the ACK queue for the matching ACK and re-sends the
 * payload (same message ID) up to LORA_ACK_RETRIES times before giving up.
 * ------------------------------------------------------------------ */
// #define LORA_ACK_TIMEOUT_MS  200UL
#define LORA_ACK_RETRIES     2

    /* ------------------------------------------------------------------
 * Upper-level (acknowledged) send: transmits an MSG_DATA payload carrying a
 * fresh random message ID, then blocks on the internal ACK queue waiting for
 * the partner's MSG_ACK to echo that same ID. The wait timeout is the queue's
 * own blocking timeout (LORA_ACK_TIMEOUT_MS) - there is no caller-supplied
 * timeout. An ACK with a non-matching ID is consumed and ignored; if no
 * matching ACK arrives the payload is re-sent (same ID), up to `retries`
 * times. Returns ESP_OK when the matching ACK was received, or
 * ESP_ERR_TIMEOUT when all attempts were exhausted without an ACK.
 * ------------------------------------------------------------------ */
esp_err_t lora_send_data_with_ack(const uint8_t *payload, uint8_t len, uint16_t LORA_ACK_TIMEOUT_MS,
                                  uint8_t retries);

/* ------------------------------------------------------------------
 * Data callback: invoked for application-data packets received from the
 * paired device (MSG_DATA and the movement/breach message types). The
 * message_type lets the app tell them apart (e.g. MSG_DISTANCE_BREACH_CLEARED).
 * ------------------------------------------------------------------ */
typedef void (*lora_data_callback_t)(uint8_t message_type,
                                     const uint8_t *payload, uint8_t len,
                                     uint64_t sender_id, int16_t rssi);
void lora_set_data_callback(lora_data_callback_t cb);


/* ------------------------------------------------------------------
 * Pairing-success callback: invoked once a fresh pairing handshake with a
 * partner completes and the new partner has been saved. partner_id is the
 * newly paired device's ID. The application can use this to e.g. push its
 * configuration to the new partner. Do NOT do blocking work here - queue it
 * or set a flag for the application loop.
 * ------------------------------------------------------------------ */
typedef void (*lora_pairing_callback_t)(uint64_t partner_id);
void lora_set_pairing_callback(lora_pairing_callback_t cb);

/* ------------------------------------------------------------------
 * Sends an MSG_ACK for a received message, echoing the message ID of the
 * message being acknowledged. Returns false if the radio is not initialized.
 * ------------------------------------------------------------------ */
bool lora_send_ack(uint16_t msg_id);

void module_reset(uint8_t reset_pin);

void lora_set_device_role(char role);
char lora_get_device_role(void);

#endif /* LORA_H */
