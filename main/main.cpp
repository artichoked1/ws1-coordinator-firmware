#include <WeatherBus.h>
#include <WeatherBusPayload.h>
#include <RS485.h>
#include <RadioLib.h>

#include <esp_log.h>
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "hal/radiolib_esp_hal.h"
#include "lorawan_storage.hpp"

using namespace lorawan;

//--- LoRaWAN Globals ---//

int radioLibState;

extern RTC_DATA_ATTR uint8_t LWsession[RADIOLIB_LORAWAN_SESSION_BUF_SIZE]; // from lorawan_storage.cpp

uint64_t joinEUI = RADIOLIB_LORAWAN_JOIN_EUI;
uint64_t devEUI = RADIOLIB_LORAWAN_DEV_EUI;
uint8_t appKey[] = { RADIOLIB_LORAWAN_APP_KEY };
uint8_t nwkKey[] = { RADIOLIB_LORAWAN_NWK_KEY };

EspHal *hal = new EspHal(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

const LoRaWANBand_t Region = RADIOLIB_LORAWAN_REGION;
const uint8_t subBand = RADIOLIB_LORAWAN_SUB_BAND;

SX1276 radio = new Module(
	hal,
	SX1276_CS_PIN,
	SX1276_IRQ_PIN,
	SX1276_RST_PIN,
	SX1276_BUSY_PIN);

LoRaWANNode node(&radio, &Region, subBand);


//--- WeatherBus Globals ---//

typedef struct {
	uint32_t		device_id;
	sensorbus_sensor_t	sensors[MAX_SENSOR_CNT];
	size_t			sensor_count;
} slave_entry_t;

RTC_DATA_ATTR slave_entry_t rtc_slaves[MAX_SLAVES];
RTC_DATA_ATTR size_t rtc_slave_count = 0;
slave_entry_t slaves[MAX_SLAVES];
size_t slave_count = 0;


//--- General Globals ---//

extern rs485_uart_t uart_dev;           // from hal/sensorbus_esp_hal.c

RTC_DATA_ATTR int bootCount = 0;        // on a cold boot, this will be 1


//--- Helpers ---//

static void update_sensor(slave_entry_t *slave, const sensorbus_sensor_t *incoming)
{
	for (size_t i = 0; i < slave->sensor_count; i++) {
		auto &existing = slave->sensors[i];
		if (existing.type == incoming->type &&
		    existing.index == incoming->index) {
			existing.format = incoming->format;
			// copy exactly the right number of bytes:
			uint8_t fmt = (uint8_t)incoming->format;
			uint8_t L = SENSORBUS_FMT_LEN[fmt];
			memcpy(existing.value, incoming->value, L);
			return;
		}
	}
}

static const char *TAG = "main";

extern "C" void app_main(void)
{
	esp_log_level_set("*", ESP_LOG_INFO);

	// Set boot variables
	bootCount++;
	esp_reset_reason_t reason = esp_reset_reason();
	bool warmWake = (reason == ESP_RST_DEEPSLEEP);
	bool coldBoot = !warmWake;
	ESP_LOGI(TAG, "Boot count: %d", bootCount);
	ESP_LOGI(TAG, "Cold boot: %s", coldBoot ? "Yes" : "No");

	// Initialize NVS
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
	    err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ESP_ERROR_CHECK(nvs_flash_init());
	}

	// Initialize radio and join LoRaWAN network
#ifndef LORAWAN_DRY_RUN_MODE
	ESP_LOGI(TAG, "[SX1276] Initializing radio...");
	radioLibState = radio.begin();
	if (radioLibState != RADIOLIB_ERR_NONE) {
		ESP_LOGE(TAG, "Failed to initialize radio: %d", radioLibState);
		return;
	}
	ESP_LOGI(TAG, "[LoRaWAN] Initializing node...");
	radioLibState = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
	if (radioLibState != RADIOLIB_ERR_NONE) {
		ESP_LOGE(TAG, "Failed to initialize LoRaWAN node: %d",
			 radioLibState);
		return;
	}
	radioLibState = lwActivate(node);
	node.setDutyCycle(false);
	node.setADR(false);
	node.setDatarate(3);
	node.setDwellTime(false);
#else
	ESP_LOGI(TAG, "Dry-run mode: skipping radio and LoRaWAN initialization.");
#endif

	// Init RS485 and WeatherBus
	sensorbus_init();

	// Restore slaves from RTC RAM if available
	if (!coldBoot && rtc_slave_count > 0) {
		slave_count = rtc_slave_count;
		memcpy(slaves, rtc_slaves, slave_count * sizeof(slave_entry_t));
		ESP_LOGI(TAG, "Restored %u slaves from RTC RAM", slave_count);
	}

	// Otherwise, if its a cold boot or no slaves are stored, discover slaves from scratch.
	if (coldBoot || slave_count == 0) {
		ESP_LOGI(TAG, "Starting slave discovery...");
		sensorbus_packet_t pkt;
		slave_count = 0;
		sensorbus_send(SENSORBUS_DISCOVERY, MASTER_ID, NULL, 0);

		// Wait for discovery replies
		uint32_t start = sensorbus_hal_get_time_ms();
		while (sensorbus_hal_get_time_ms() - start <
		       DISCOVERY_WINDOW_MS) {                                       // Start discovery window
			sensorbus_error_t r =
				sensorbus_receive_timeout(PKT_TIMEOUT_MS, &pkt);
			if (r != SENSORBUS_OK) continue;
			if (pkt.msg_type != SENSORBUS_DISCOVERY_REPLY) continue;
			sensorbus_sensor_t tmp[
				SNENSORBUS_MAX_SENSOR_DISCOVERY_SLOTS];
			size_t tmpn = 0;

			// Decode the payload into a temporary sensor array
			if (sensorbus_pb_decode_descriptors(pkt.payload,
							    pkt.payload_len, tmp,
							    &tmpn) != SENSORBUS_OK) continue;

			// Add or update the slave entry
			size_t si;
			for (si = 0; si < slave_count; si++)
				if (slaves[si].device_id ==
				    pkt.device_id) break;
			if (si == slave_count && slave_count < MAX_SLAVES) {
				slaves[si].device_id = pkt.device_id;
				slaves[si].sensor_count = 0;
				slave_count++;
			}
			for (size_t i = 0;
			     i < tmpn &&
			     slaves[si].sensor_count < MAX_SENSOR_CNT;
			     i++)
				slaves[si].sensors[slaves[si].sensor_count++] =
					tmp[i];
		}

		// Save discovered slaves to RTC RAM
		rtc_slave_count = slave_count;
		memcpy(rtc_slaves, slaves, slave_count * sizeof(slave_entry_t));
		ESP_LOGI(TAG, "Saved %u slaves to RTC RAM", slave_count);
		ESP_LOGI(TAG, "%u slave(s) discovered", (unsigned)slave_count);
	}

	// Query each slave and update readings
	for (size_t si = 0; si < slave_count; si++) {
		uint32_t dev_id = slaves[si].device_id;
		ESP_LOGI(TAG, "Querying slave 0x%08" PRIX32, dev_id);

		payload_builder_t query_buffer;
		pb_init(&query_buffer);
		for (size_t j = 0; j < slaves[si].sensor_count; j++) {
			if (query_buffer.len + 3 > SENSORBUS_MAX_PAYLOAD) break;
			sensorbus_pb_add_descriptor(&query_buffer, slaves[si].sensors[j].type, slaves[si].sensors[j].index);
		}

		sensorbus_send(SENSORBUS_QUERY, dev_id, query_buffer.buf,
			       query_buffer.len);
		sensorbus_packet_t resp;
		if (sensorbus_receive_timeout(200, &resp) != SENSORBUS_OK) {
			ESP_LOGW(TAG, "Slave 0x%08" PRIX32 " timed out",
				 dev_id);
			continue;
		}
		sensorbus_sensor_t out[SENSORBUS_MAX_TLVS];
		size_t outn = 0;
		if (sensorbus_pb_decode_sensors(resp.payload, resp.payload_len, out, &outn) != SENSORBUS_OK) {
			ESP_LOGW(TAG, "bad TLV from slave 0x%08" PRIX32, dev_id);
			continue;
		}
		for (size_t k = 0; k < outn; k++)
			update_sensor(&slaves[si], &out[k]);
	}

	// Fancy logging
	for (size_t si = 0; si < slave_count; si++) {
		ESP_LOGI(TAG, "┌── Slave 0x%08" PRIX32, slaves[si].device_id);
		for (size_t i = 0; i < slaves[si].sensor_count; i++) {
			const sensorbus_sensor_t &s = slaves[si].sensors[i];
			char hexbuf[3 * SENSORBUS_MAX_VALUE_LEN + 1] = { 0 };
			for (uint8_t j = 0; j < SENSORBUS_FMT_LEN[s.format]; j++)
				sprintf(&hexbuf[j * 3], "%02X ", s.value[j]);

			ESP_LOGI(TAG,
				 "│ type=0x%02X idx=%u fmt=%u len=%u val=%s",
				 s.type,
				 s.index,
				 s.format,
				 SENSORBUS_FMT_LEN[s.format],
				 hexbuf);
		}
		ESP_LOGI(TAG, "└──");
	}

	// Build and send a single LoRaWAN uplink
	payload_builder_t lb;
	pb_init(&lb);

	for (size_t si = 0; si < slave_count; si++) {
		for (size_t i = 0; i < slaves[si].sensor_count; i++) {
			const sensorbus_sensor_t &sensor = slaves[si].sensors[i];
			sensorbus_pb_add_sensor(&lb, sensor.type, sensor.format, sensor.index, sensor.value);
		}
  }

	size_t uplink_len = lb.len;
	if (uplink_len > 0) {
		ESP_LOGI(TAG, "Built uplink payload with %zu bytes", uplink_len);
		radioLibState = node.sendReceive(lb.buf, uplink_len);
		ESP_LOGI(TAG, "Radiolib code: %d, FCntUp now %" PRIu32, radioLibState, node.getFCntUp());
		memcpy(LWsession, node.getBufferSession(), RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
	} else {
		ESP_LOGW(TAG, "Nothing to send (payload %zu bytes)",
			 uplink_len);
	}

	esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_US);
	ESP_LOGI(TAG, "Entering deep sleep for %u seconds…",
		 (unsigned)(SLEEP_INTERVAL_US / 1000000ULL));
	esp_deep_sleep_start();
}
