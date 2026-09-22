// --------------------------------------------
//  device_identity.h
//  Device Identity Manager
//  Provides a permanent, unique device ID derived
//  from the ESP32 factory-programmed Base MAC.
//  (ported from the LORA_RECIEVER project)
// --------------------------------------------
#ifndef DEVICE_IDENTITY_H
#define DEVICE_IDENTITY_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* NVS namespace + key used to persist the device ID */
#define DEVICE_NAMESPACE "device"
#define DEVICE_ID_KEY    "device_id"

/* ------------------------------------------------------------------
 * Initializes the identity module.
 *  - Opens NVS (device namespace)
 *  - Loads an existing device_id from NVS if present
 *  - Otherwise reads the ESP32 Base MAC, converts it to a uint64_t,
 *    stores it in NVS and caches it in RAM
 * Returns true on success, false on failure.
 * ------------------------------------------------------------------ */
bool device_identity_init(void);

/* Returns the cached uint64_t device ID (no NVS access). */
uint64_t device_identity_get_id(void);

/* Formats a device ID (uint64_t) as a MAC-style string, e.g.
 * "84:F7:03:12:34:56". buf must hold at least 18 bytes. */
void device_identity_format_id(uint64_t id, char *buf, size_t buflen);

/* Returns true once the module has been successfully initialized. */
bool device_identity_is_initialized(void);

#endif /* DEVICE_IDENTITY_H */
