// --------------------------------------------
//  device_identity.c
//  Device Identity Manager (ported to C)
// --------------------------------------------
#include "device_identity.h"

#include <stdio.h>
#include <esp_mac.h>
#include <nvs.h>

/* ---- cached state (RAM only) ---- */
static uint64_t s_deviceId = 0;
static bool     s_initialized = false;

/* Read the factory Base MAC and convert it to a 48-bit value in a uint64_t. */
static uint64_t macToUint64(const uint8_t mac[6]) {
  uint64_t id = 0;
  for (int i = 0; i < 6; i++) {
    id = (id << 8) | mac[i];
  }
  return id;
}

bool device_identity_init(void) {
  if (s_initialized) {
    return true;
  }

  /* Always read the Base MAC: it is needed both to generate the
   * device ID and to format the string representation. */
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);

  nvs_handle_t handle;
  if (nvs_open(DEVICE_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
    return false;   /* NVS not available */
  }

  esp_err_t err = nvs_get_u64(handle, DEVICE_ID_KEY, &s_deviceId);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    /* First boot: derive the ID from the Base MAC and persist it. */
    s_deviceId = macToUint64(mac);
    nvs_set_u64(handle, DEVICE_ID_KEY, s_deviceId);
    nvs_commit(handle);
  } else if (err != ESP_OK) {
    nvs_close(handle);
    return false;
  }

  nvs_close(handle);
  s_initialized = true;
  return true;
}

uint64_t device_identity_get_id(void) {
  return s_deviceId;
}

/* Formats a 48-bit device ID as "XX:XX:XX:XX:XX:XX". */
void device_identity_format_id(uint64_t id, char *buf, size_t buflen) {
  if (!buf || buflen < 18) {
    return;
  }
  snprintf(buf, buflen, "%02X:%02X:%02X:%02X:%02X:%02X",
           (uint8_t)(id >> 40), (uint8_t)(id >> 32),
           (uint8_t)(id >> 24), (uint8_t)(id >> 16),
           (uint8_t)(id >> 8),  (uint8_t)id);
}

bool device_identity_is_initialized(void) {
  return s_initialized;
}
