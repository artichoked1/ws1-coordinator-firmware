#include "lorawan_storage.hpp"
#include "config.h"
#include <esp_log.h>
#include <nvs_flash.h>
#include <cstring>

RTC_DATA_ATTR static uint32_t rtc_magic         = 0;
RTC_DATA_ATTR static uint8_t  rtc_session[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];
RTC_DATA_ATTR static uint8_t  rtc_nonces[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
RTC_DATA_ATTR static bool     rtc_nonces_valid  = false;

static const char *TAG = "lorawan_storage";

namespace lorawan {

void restoreBuffers(LoRaWANNode& node, bool coldBoot) {
  esp_err_t err;
  nvs_handle_t handle;

  ESP_LOGI(TAG, "Restoring LoRaWAN buffers...");

  err = nvs_open(LORAWAN_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "NVS open failed: %s", esp_err_to_name(err));
    return;
  }

  // Nonces: prefer RTC RAM on wake-from-deep-sleep
  if (!coldBoot && rtc_nonces_valid) {
    int status = node.setBufferNonces(rtc_nonces);
    ESP_LOGI(TAG, "RTC nonces restore: %d", status);
  } else {
    size_t len = sizeof(rtc_nonces);
    err = nvs_get_blob(handle, LORAWAN_KEY_NONCES, rtc_nonces, &len);
    if (err == ESP_OK) {
      int status = node.setBufferNonces(rtc_nonces);
      ESP_LOGI("lorawan", "Flash nonces restore: %d", status);
      rtc_nonces_valid = true;
    } else {
      ESP_LOGW(TAG, "No saved nonces in flash.");
    }
  }

  // Prefer RTC RAM if it has the magic tag
  if (rtc_magic == LORAWAN_SESSION_MAGIC) {
    int16_t status = node.setBufferSession(rtc_session);
    ESP_LOGI(TAG, "RTC session restore: %d", status);
  } else {
    size_t sess_len = sizeof(rtc_session);
    err = nvs_get_blob(handle, LORAWAN_KEY_SESSION, rtc_session, &sess_len);
    if (err == ESP_OK) {
      int16_t status = node.setBufferSession(rtc_session);
      ESP_LOGI(TAG, "Flash session restore: %d", status);
    } else {
      ESP_LOGW(TAG, "No saved session in flash.");
    }
  }

  nvs_close(handle);
}

void saveSessionToRTC(LoRaWANNode& node) {
  // Copy into RTC RAM
  const uint8_t* session = node.getBufferSession();
  memcpy(rtc_session, session, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
  rtc_magic = LORAWAN_SESSION_MAGIC;

  const uint8_t* nonces = node.getBufferNonces();
  memcpy(rtc_nonces, nonces, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
  rtc_nonces_valid = true;

  ESP_LOGI(TAG, "Saved session + nonces to RTC RAM");
}

void saveToFlashIfNeeded(LoRaWANNode& node, int bootCount) {
  // Save flash blobs only every 10 boots
  if (bootCount % LORAWAN_SAVE_BOOTCOUNT != 0) {
    ESP_LOGI(TAG, "Skipping flash save (boot %u)", bootCount);
    return;
  }

  esp_err_t err;
  nvs_handle_t handle;
  err = nvs_open(LORAWAN_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
    return;
  }

  const uint8_t* nonces = node.getBufferNonces();
  ESP_ERROR_CHECK(nvs_set_blob(handle, LORAWAN_KEY_NONCES, nonces, RADIOLIB_LORAWAN_NONCES_BUF_SIZE));
  ESP_LOGI(TAG, "Saved nonces to flash (boot %u)", bootCount);

  const uint8_t* session = node.getBufferSession();
  ESP_ERROR_CHECK(nvs_set_blob(handle, LORAWAN_KEY_SESSION, session, RADIOLIB_LORAWAN_SESSION_BUF_SIZE));
  ESP_LOGI(TAG, "Saved session to flash (boot %u)", bootCount);

  ESP_ERROR_CHECK(nvs_commit(handle));
  nvs_close(handle);
}

}
