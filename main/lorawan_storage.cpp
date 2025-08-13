// Based on Radiolib's radiolib-persistence utility functions, adapted for ESP-IDF:
// https://github.com/radiolib-org/radiolib-persistence/blob/main/examples/LoRaWAN_ESP32/LoRaWAN_ESP32.ino

#include "lorawan_storage.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include <esp_sleep.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

extern RTC_DATA_ATTR int boot_count;
RTC_DATA_ATTR static int bootCountSinceUnsuccessfulJoin = 0;
RTC_DATA_ATTR uint8_t LWsession[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];

static const char *TAG = "LorawanStorage";

namespace lorawan {
int lwActivate(LoRaWANNode& node)
{
	int state = RADIOLIB_ERR_UNKNOWN;
	nvs_handle_t handle;

	ESP_LOGI(TAG, "Recalling nonces and session...");

	if (nvs_open(LORAWAN_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
		size_t len = RADIOLIB_LORAWAN_NONCES_BUF_SIZE;
		uint8_t buffer[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
		if (nvs_get_blob(handle, LORAWAN_KEY_NONCES, buffer, &len) == ESP_OK) {
			state = node.setBufferNonces(buffer);
			if (state != RADIOLIB_ERR_NONE)
				ESP_LOGE(TAG, "Failed to set nonces buffer: %d", state);
			else
				ESP_LOGI(TAG, "Nonces buffer restored successfully.");
		}
		state = node.setBufferSession(LWsession);
		if (state != RADIOLIB_ERR_NONE && boot_count > 1)
			ESP_LOGE(TAG, "Restoring session buffer failed (code: %d, bootCount: %d)",
				 state, boot_count);
		else
			ESP_LOGI(TAG, "Session buffer restored successfully.");
		nvs_close(handle);
		if (state == RADIOLIB_ERR_NONE) {
			ESP_LOGI(TAG, "Session restored successfully. Attempting to join...");
			state = node.activateOTAA(3);
			if (state != RADIOLIB_LORAWAN_SESSION_RESTORED)
				ESP_LOGE(TAG, "Failed to restore session: %d", state);
			else
				ESP_LOGI(TAG, "Session restored successfully.");
			return state;
		}
	} else {
		ESP_LOGI(TAG, "No nonces found in NVS, starting new session...");
	}

	state = RADIOLIB_ERR_NETWORK_NOT_JOINED;

	while (state != RADIOLIB_LORAWAN_NEW_SESSION) {
		ESP_LOGI(TAG, "Attempting to join network...");
		state = node.activateOTAA();

		ESP_LOGI(TAG, "Saving nonces to flash...");
		if (nvs_open(LORAWAN_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
			const uint8_t *persist = node.getBufferNonces();
			if (nvs_set_blob(handle, LORAWAN_KEY_NONCES,
					 persist, RADIOLIB_LORAWAN_NONCES_BUF_SIZE) == ESP_OK) {
				nvs_commit(handle);
				ESP_LOGI(TAG, "Nonces saved successfully.");
			} else {
				ESP_LOGE(TAG, "Failed to save nonces to flash.");
			}
			nvs_close(handle);
		}

		if (state != RADIOLIB_LORAWAN_NEW_SESSION) {
			ESP_LOGI(TAG, "Join failed: %d", state);
			int sleepForSeconds = MIN((bootCountSinceUnsuccessfulJoin + 1) * 60, 180);
			bootCountSinceUnsuccessfulJoin++;
			ESP_LOGI(TAG, "Boots since last successful join: %d",
				 bootCountSinceUnsuccessfulJoin);
			ESP_LOGI(TAG, "Retrying in %d seconds...", sleepForSeconds);
			esp_sleep_enable_timer_wakeup(uint64_t(sleepForSeconds) * 1000000ULL);
			esp_deep_sleep_start();
		}
	}

	ESP_LOGI(TAG, "Join successful!");
	bootCountSinceUnsuccessfulJoin = 0;
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	return state;
}
}
