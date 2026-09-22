// --------------------------------------------
//  packet.c
//  LoRa packet (de)serialization
//  (ported from the LORA_RECIEVER project)
//
//  Wire format (all multi-byte fields big-endian):
//    [0]      version        (1 byte)
//    [1]      messageType    (1 byte)
//    [2..3]   messageID      (2 bytes)
//    [4..11]  senderID       (8 bytes)
//    [12..19] receiverID     (8 bytes)
//    [20..21] sequence       (2 bytes)
//    [22]     payloadLength  (1 byte)
//    [23..]   payload        (payloadLength bytes)
//  Header is 23 bytes + payload.
// --------------------------------------------
#include "packet.h"

#include <string.h>

#define PACKET_HEADER_SIZE 23

/* ---- big-endian helpers ------------------------------------------ */
static void writeU16BE(uint8_t *buf, uint16_t value) {
  buf[0] = (value >> 8) & 0xFF;
  buf[1] = value & 0xFF;
}

static void writeU64BE(uint8_t *buf, uint64_t value) {
  for (int i = 0; i < 8; i++) {
    buf[i] = (value >> (56 - 8 * i)) & 0xFF;
  }
}

static uint16_t readU16BE(const uint8_t *buf) {
  return (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

static uint64_t readU64BE(const uint8_t *buf) {
  uint64_t value = 0;
  for (int i = 0; i < 8; i++) {
    value = (value << 8) | buf[i];
  }
  return value;
}

/* ---- encode ------------------------------------------------------- */
uint16_t packet_encode(const LoRaPacket *packet, uint8_t *buffer) {
  if (!packet || !buffer) {
    return 0;
  }
  if (packet->payloadLength > MAX_PAYLOAD_LENGTH) {
    return 0;
  }

  uint16_t length = PACKET_HEADER_SIZE + packet->payloadLength;

  buffer[0] = packet->version;
  buffer[1] = packet->messageType;
  writeU16BE(&buffer[2],  packet->messageID);
  writeU64BE(&buffer[4],  packet->senderID);
  writeU64BE(&buffer[12], packet->receiverID);
  writeU16BE(&buffer[20], packet->sequence);
  buffer[22] = packet->payloadLength;
  if (packet->payloadLength > 0) {
    memcpy(&buffer[23], packet->payload, packet->payloadLength);
  }

  return length;
}

/* ---- decode ------------------------------------------------------- */
bool packet_decode(const uint8_t *buffer, uint16_t length, LoRaPacket *packet) {
  if (!buffer || !packet) {
    return false;
  }
  if (length < PACKET_HEADER_SIZE) {
    return false;
  }

  packet->version       = buffer[0];
  packet->messageType   = buffer[1];
  packet->messageID     = readU16BE(&buffer[2]);
  packet->senderID      = readU64BE(&buffer[4]);
  packet->receiverID    = readU64BE(&buffer[12]);
  packet->sequence      = readU16BE(&buffer[20]);
  packet->payloadLength = buffer[22];

  if (packet->payloadLength > MAX_PAYLOAD_LENGTH) {
    return false;
  }
  if (length < PACKET_HEADER_SIZE + packet->payloadLength) {
    return false;
  }

  if (packet->payloadLength > 0) {
    memcpy(packet->payload, &buffer[23], packet->payloadLength);
  }

  return true;
}
