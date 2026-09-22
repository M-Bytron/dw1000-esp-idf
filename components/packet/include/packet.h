// --------------------------------------------
//  packet.h
//  LoRa packet structure + (de)serialization
//  (ported from the LORA_RECIEVER project)
// --------------------------------------------
#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>
#include <stdbool.h>

#define PACKET_VERSION      1
#define MAX_PAYLOAD_LENGTH  64
#define BROADCAST_ID        0xFFFFFFFFFFFFFFFFULL   /* 0xFFF...F = everyone */

/* Message types exchanged over the air */
typedef enum {
    MSG_PING          = 1,
    MSG_PAIR_ACK      = 2,   /* pairing handshake ACK (empty payload) */
    MSG_PAIR_REQUEST  = 3,
    MSG_PAIR_RESPONSE = 4,
    MSG_DATA          = 5,
    MSG_PAIR_CONFIRM  = 6,
    MSG_ACK           = 7    /* general ACK for NON-pairing messages; the ACK
                                echoes the messageID of the message it
                                acknowledges, so the sender can match the ACK
                                against the message it currently has in
                                flight */
} MessageType;

/* ------------------------------------------------------------------
 * LoRaPacket
 *   version      : packet format version (PACKET_VERSION)
 *   messageType  : one of MessageType
 *   messageID    : random per-message ID assigned by the sender to every
 *                  packet; an MSG_ACK echoes the ID of the message it
 *                  acknowledges, so received data and its ACK share the ID
 *   senderID     : device ID of the sender
 *   receiverID   : device ID of the destination (BROADCAST_ID = all)
 *   sequence     : per-sender sequence number
 *   payloadLength: number of valid payload bytes
 *   payload      : optional data (up to MAX_PAYLOAD_LENGTH bytes)
 * ------------------------------------------------------------------ */
typedef struct {
    uint8_t  version;
    uint8_t  messageType;

    uint16_t messageID;

    uint64_t senderID;
    uint64_t receiverID;

    uint16_t sequence;

    uint8_t  payloadLength;
    uint8_t  payload[MAX_PAYLOAD_LENGTH];
} LoRaPacket;

/* Serialize a packet into a byte buffer (big-endian wire format).
 * Returns the encoded length in bytes, or 0 on error. */
uint16_t packet_encode(const LoRaPacket *packet, uint8_t *buffer);

/* Deserialize a byte buffer into a packet.
 * Returns true on success, false if the buffer is invalid/truncated. */
bool packet_decode(const uint8_t *buffer, uint16_t length, LoRaPacket *packet);

#endif /* PACKET_H */
