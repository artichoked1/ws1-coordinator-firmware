#include <weatherbus.h>
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

RFM95 radio = new Module(
	hal,
	SX1276_CS_PIN,
	SX1276_IRQ_PIN,
	SX1276_RST_PIN,
	SX1276_BUSY_PIN);

LoRaWANNode node(&radio, &Region, subBand);


//--- WeatherBus Globals ---//

typedef struct {
	uint32_t device_id;
	sensorbus_sensor_t sensors[MAX_SENSOR_CNT];
	size_t sensor_count;
} slave_entry_t;

RTC_DATA_ATTR slave_entry_t rtc_slaves[MAX_SLAVES];
RTC_DATA_ATTR size_t rtc_slave_count = 0;
slave_entry_t slaves[MAX_SLAVES];
size_t slave_count = 0;

uint8_t uplink_payload[256];

//--- General Globals ---//

extern rs485_uart_t uart_dev; // from hal/sensorbus_esp_hal.c

RTC_DATA_ATTR int boot_count = 0;  // on a cold boot, this will be 1

static const char *TAG = "main";


//--- Helpers ---//

// Update exiisting sensor entry with new data
static void update_sensor(slave_entry_t *slave, const sensorbus_sensor_t *incoming)
{
	for (size_t i = 0; i < slave->sensor_count; i++) {
		sensorbus_sensor_t &existing = slave->sensors[i];

		// Check if the existing sensor matches the incoming one
		if (existing.type == incoming->type && existing.index == incoming->index) {
			// Update the existing sensor's value and format
			existing.format = incoming->format;
			// copy exactly the right number of bytes:
			uint8_t fmt = (uint8_t)incoming->format;
			uint8_t L = SENSORBUS_FMT_LEN[fmt];

			// Copy only the required number of bytes
			memcpy(existing.value, incoming->value, L);

			return;
		}
	}
}

// Group sensors by slave for the uplink payload.
// Format: [slave short ID][sensor count][sensor TLVs]... for every slave
static size_t append_slave_payload(uint8_t *out, const slave_entry_t *slave)
{
	payload_builder_t pb;

	pb_init(&pb);
	for (size_t i = 0; i < slave->sensor_count; ++i) {
		const sensorbus_sensor_t &sensor = slave->sensors[i];
		sensorbus_pb_add_sensor(&pb, sensor.type, sensor.format, sensor.index, sensor.value);
	}
	// Chop the first 2 bytes off the slave ID to save space. The first two are just vendor and product code.
	uint16_t sid = (uint16_t)(slave->device_id & 0xFFFF);

	out[0] = (uint8_t)(sid >> 8);
	out[1] = (uint8_t)(sid & 0xFF);
	out[2] = (uint8_t)slave->sensor_count;
	memcpy(out + 3, pb.buf, pb.len);
	return 3 + pb.len;
}

void flash_led(int times)
{
  for (int i = 0; i < times; i++) {
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void sleep_bus()
{
  // Put the RS485 bus to sleep
  ESP_LOGI(TAG, "Putting bus to sleep...");
  rs485_enter_shutdown(&uart_dev);
  gpio_set_level(GPIO_NUM_15, 0); // Set the wake pin low to put slaves to sleep
  ESP_LOGI(TAG, "Bus is now asleep.");
}

void wake_bus()
{
  // Wake up the RS485 bus
  ESP_LOGI(TAG, "Waking up bus...");
  rs485_exit_shutdown(&uart_dev);
  gpio_set_level(GPIO_NUM_15, 1); // Set the wake pin high to wake slaves
  vTaskDelay(pdMS_TO_TICKS(100)); // Wait for slaves to wake up
  ESP_LOGI(TAG, "Bus is now awake.");
}

static void wake_pin_init()
{
  // Make sure previous sleep didn't leave a hold latched
  gpio_hold_dis(GPIO_NUM_15);

  // Reset the pad mux & pulls to a clean state
  gpio_reset_pin(GPIO_NUM_15);

  // Force it to GPIO output, start low
  ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT));
  ESP_ERROR_CHECK(gpio_set_pull_mode(GPIO_NUM_15, GPIO_FLOATING)); // or GPIO_PULLDOWN_ONLY
  ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_15, 0));
}


extern "C" void app_main(void)
{
	esp_log_level_set("*", ESP_LOG_INFO);
  wake_pin_init();
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  // Set boot variables
	boot_count++;
	esp_reset_reason_t reason = esp_reset_reason();
	bool warmWake = (reason == ESP_RST_DEEPSLEEP);
	bool coldBoot = !warmWake;
	ESP_LOGI(TAG, "Boot count: %d", boot_count);
	ESP_LOGI(TAG, "Cold boot: %s", coldBoot ? "Yes" : "No");

  flash_led(2);

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
		ESP_LOGE(TAG, "Failed to initialize LoRaWAN node: %d", radioLibState);
		return;
	}
	radioLibState = lwActivate(node);
  node.setTxPower(14);
  node.setDutyCycle(false);
  node.setADR(false);
  node.setDatarate(4);

#else
	ESP_LOGI(TAG, "Dry-run mode: skipping radio and LoRaWAN initialization.");
#endif

	// Init RS485 and WeatherBus
	sensorbus_init();

  wake_bus();

	// Restore slaves from RTC RAM if available
	if (!coldBoot && rtc_slave_count > 0) {
		slave_count = rtc_slave_count;
		memcpy(slaves, rtc_slaves, slave_count * sizeof(slave_entry_t));
		ESP_LOGI(TAG, "Restored %zu slaves from RTC RAM", slave_count);
	}

	// Otherwise, if its a cold boot or no slaves are stored, discover slaves from scratch.
	if (coldBoot || slave_count == 0) {
		ESP_LOGI(TAG, "Starting slave discovery...");
		sensorbus_packet_t pkt;
		slave_count = 0;
		sensorbus_send(SENSORBUS_DISCOVERY, MASTER_ID, NULL, 0);

		// Wait for discovery replies
		uint32_t start = sensorbus_hal_get_time_ms();

		// Start discovery window
		while (sensorbus_hal_get_time_ms() - start < DISCOVERY_WINDOW_MS) {
			sensorbus_error_t r = sensorbus_receive_timeout(PKT_TIMEOUT_MS, &pkt);

			if (r != SENSORBUS_OK) continue;
			if (pkt.msg_type != SENSORBUS_DISCOVERY_REPLY) continue;

			sensorbus_sensor_t tmp[SNENSORBUS_MAX_SENSOR_DISCOVERY_SLOTS];
			size_t tmpn = 0;

			// Decode the payload into a temporary sensor array
			if (sensorbus_pb_decode_descriptors(pkt.payload, pkt.payload_len, tmp, &tmpn) != SENSORBUS_OK) continue;

			// Add or update the slave entry
			size_t si;
			for (si = 0; si < slave_count; si++) {
				if (slaves[si].device_id == pkt.device_id) break;
			}
			if (si == slave_count && slave_count < MAX_SLAVES) {
				slaves[si].device_id = pkt.device_id;
				slaves[si].sensor_count = 0;
				slave_count++;
			}
			for (size_t i = 0; i < tmpn && slaves[si].sensor_count < MAX_SENSOR_CNT; i++) {
				slaves[si].sensors[slaves[si].sensor_count++] = tmp[i];
			}
		}

		// Save discovered slaves to RTC RAM
		rtc_slave_count = slave_count;
		memcpy(rtc_slaves, slaves, slave_count * sizeof(slave_entry_t));
		ESP_LOGI(TAG, "Saved %zu slaves to RTC RAM", slave_count);
		ESP_LOGI(TAG, "%zu slave(s) discovered", slave_count);
	}

	// Query each slave and update readings
	for (size_t si = 0; si < slave_count; si++) {
		uint32_t dev_id = slaves[si].device_id;
		ESP_LOGI(TAG, "Querying slave 0x%08" PRIX32, dev_id);

		payload_builder_t query_buffer;
		pb_init(&query_buffer);

		// Query all sensors of the slave
		for (size_t j = 0; j < slaves[si].sensor_count; j++) {
			if (query_buffer.len + 3 > SENSORBUS_MAX_PAYLOAD) break;
			sensorbus_pb_add_descriptor(&query_buffer, slaves[si].sensors[j].type, slaves[si].sensors[j].index);
		}

		// Send query and collect response
		sensorbus_send(SENSORBUS_QUERY, dev_id, query_buffer.buf, query_buffer.len);
		sensorbus_packet_t resp;
		if (sensorbus_receive_timeout(200, &resp) != SENSORBUS_OK) { // Should probably check for correct response type/origin
			ESP_LOGW(TAG, "Slave 0x%08" PRIX32 " timed out", dev_id);
			continue;
		}

		// Convert response payload to sensors array
		sensorbus_sensor_t out[SENSORBUS_MAX_TLVS];
		size_t outn = 0;
		if (sensorbus_pb_decode_sensors(resp.payload, resp.payload_len, out, &outn) != SENSORBUS_OK) {
			ESP_LOGW(TAG, "bad TLV from slave 0x%08" PRIX32, dev_id);
			continue;
		}
		for (size_t k = 0; k < outn; k++) {
			update_sensor(&slaves[si], &out[k]);
		}
	}
	// Fancy logging
	for (size_t si = 0; si < slave_count; si++) {

		ESP_LOGI(TAG, "┌── Slave 0x%08" PRIX32, slaves[si].device_id);

		for (size_t i = 0; i < slaves[si].sensor_count; i++) {
			const sensorbus_sensor_t &s = slaves[si].sensors[i];

			// Buffer to hold hex string of the sensor val
			char hexbuf[3 * SENSORBUS_MAX_VALUE_LEN + 1] = { 0 };

			// Convert sensor value to hex string
			for (uint8_t j = 0; j < SENSORBUS_FMT_LEN[s.format]; j++) {
				sprintf(&hexbuf[j * 3], "%02X ", s.value[j]);
			}

			// Log the sensor information
			ESP_LOGI(TAG, "│ type=0x%02X idx=%u fmt=%u len=%u val=%s",
				 s.type,
				 s.index,
				 s.format,
				 SENSORBUS_FMT_LEN[s.format],
				 hexbuf);

		}
		ESP_LOGI(TAG, "└──");
	}

	// Build and send the uplink.
	size_t uplink_len = 0;

	for (size_t si = 0; si < slave_count; ++si) {
		uplink_len += append_slave_payload(uplink_payload + uplink_len, &slaves[si]);
	}

	if (uplink_len > 0) {
		ESP_LOGI(TAG, "Built uplink payload with %zu bytes", uplink_len);

#ifndef LORAWAN_DRY_RUN_MODE
		// Normal send process
		radioLibState = node.sendReceive(uplink_payload, uplink_len);
		ESP_LOGI(TAG, "Radiolib code: %d, FCntUp now %" PRIu32,
			 radioLibState, node.getFCntUp());
		memcpy(LWsession, node.getBufferSession(), RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
#else
		// Dump the bytes instead of sending on dry-run mode
		printf("DRY_RUN_MODE enabled — would send uplink (%zu bytes):\n", uplink_len);
		for (size_t i = 0; i < uplink_len; i++)
			printf("%02X ", uplink_payload[i]);
		printf("\n");
#endif
	} else {
		ESP_LOGW(TAG, "Nothing to send (payload %zu bytes)", uplink_len);
	}

  sleep_bus();
	esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_US);
	ESP_LOGI(TAG, "Entering deep sleep for %llu seconds...", (SLEEP_INTERVAL_US / 1000000ULL));
	esp_deep_sleep_start();
}
