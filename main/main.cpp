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
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_mqtt.hpp>
#include <cJSON.h>
#include <esp_sntp.h>
#include <ctime>

#include "config.h"
#include "hal/radiolib_esp_hal.h"
#include "lorawan_storage.hpp"
#include "esp_mqtt_client_config.hpp"

using namespace lorawan;

static const char *MAIN = "Main";
static const char *WIFI = "WiFi";
static const char *MQTT = "MQTT";
static const char *SNTP = "SNTP";
static const char *JSON = "JSON";
static const char *WBUS = "WeatherBus";
static const char *LORAWAN = "LoRaWAN";

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


//--- WiFi and MQTT ---//

#if MQTT_ENABLE == true

static void initialise_sntp(void)
{
    ESP_LOGI(SNTP, "Initialising SNTP");
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org"); // or your local NTP server
    esp_sntp_init();
}

static void wait_for_time_sync(void)
{
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 15;


    while (timeinfo.tm_year < (2020 - 1900) && ++retry < retry_count) {
        ESP_LOGI(SNTP, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(SNTP, "System time is now: %s", strftime_buf);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(WIFI, "Got IP, ready for MQTT!");
        initialise_sntp();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(WIFI, "Retrying WiFi...");
    }
}

void wifi_init_sta(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(WIFI, "WiFi init done.");
}

// Build a TTN-style JSON payload to send directly to the MQTT broker in WiFi mode.
static std::string build_json_payload(const slave_entry_t *slaves, size_t slave_count)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *uplink_message = cJSON_CreateObject();
    cJSON *decoded_payload = cJSON_CreateObject();
    cJSON *slaves_array = cJSON_CreateArray();
    cJSON *end_device_ids = cJSON_CreateObject();

    // Build slave entries
    for (size_t si = 0; si < slave_count; si++) {
        cJSON *slave_obj = cJSON_CreateObject();
        cJSON_AddNumberToObject(slave_obj, "id", slaves[si].device_id & 0xFFFF);

        cJSON *sensors_array = cJSON_CreateArray();

        for (size_t i = 0; i < slaves[si].sensor_count; i++) {
            const sensorbus_sensor_t &s = slaves[si].sensors[i];
            cJSON *sensor_obj = cJSON_CreateObject();

            cJSON_AddNumberToObject(sensor_obj, "type", s.type);
            cJSON_AddNumberToObject(sensor_obj, "index", s.index);
            cJSON_AddNumberToObject(sensor_obj, "format", s.format);

            // Decode the sensor value depending on format
            double value = 0.0;

            switch (s.format) {
                case SENSORBUS_FMT_UINT8: {
                    uint8_t v;
                    memcpy(&v, s.value, sizeof(v));
                    value = v;
                    break;
                }
                case SENSORBUS_FMT_UINT16: {
                    uint16_t v;
                    memcpy(&v, s.value, sizeof(v));
                    value = v;
                    break;
                }
                case SENSORBUS_FMT_FLOAT32: {
                    float v;
                    memcpy(&v, s.value, sizeof(v));
                    value = v;
                    break;
                }
                case SENSORBUS_FMT_FLOAT64: {
                    double v;
                    memcpy(&v, s.value, sizeof(v));
                    value = v;
                    break;
                }
                case SENSORBUS_FMT_SFIX16_2DP: { // signed fixed-point, /100
                    int16_t v;
                    memcpy(&v, s.value, sizeof(v));
                    value = static_cast<double>(v) / 100.0;
                    break;
                }
                case SENSORBUS_FMT_UFIX16_1DP: { // unsigned fixed-point, /10
                    uint16_t v;
                    memcpy(&v, s.value, sizeof(v));
                    value = static_cast<double>(v) / 10.0;
                    break;
                }
                default:
                    ESP_LOGW(JSON, "Unknown format %u", s.format);
                    break;
            }

            cJSON_AddNumberToObject(sensor_obj, "value", value);
            cJSON_AddItemToArray(sensors_array, sensor_obj);
        }

        cJSON_AddItemToObject(slave_obj, "sensors", sensors_array);
        cJSON_AddItemToArray(slaves_array, slave_obj);
    }

    // Construct the full JSON
    cJSON_AddItemToObject(decoded_payload, "slaves", slaves_array);
    cJSON_AddItemToObject(uplink_message, "decoded_payload", decoded_payload);
    cJSON_AddItemToObject(root, "uplink_message", uplink_message);
    cJSON_AddItemToObject(root, "end_device_ids", end_device_ids);

    // Add device EUI
    uint64_t eui = RADIOLIB_LORAWAN_DEV_EUI;
    char euibuf[17];
    snprintf(euibuf, sizeof(euibuf), "%016llX", (unsigned long long)eui);
    cJSON_AddStringToObject(end_device_ids, "dev_eui", euibuf);

    // Add received_at timestamp
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm timeinfo;
    gmtime_r(&tv.tv_sec, &timeinfo);

    char timebuf[40];
    strftime(timebuf, sizeof(timebuf), "%Y-%m-%dT%H:%M:%S", &timeinfo);

    char full_time[64];
    snprintf(full_time, sizeof(full_time), "%s.%06ld000Z", timebuf, tv.tv_usec);
    cJSON_AddStringToObject(root, "received_at", full_time);

    // Serialise to string
    char *json_str = cJSON_PrintUnformatted(root);
    std::string out(json_str);

    free(json_str);
    cJSON_Delete(root);
    return out;
}

// Grab the CA cert from linker section if TLS is enabled, put it in memory.
#if MQTT_USE_TLS == true
  extern const char mqtt_ca_cert_start[] asm("_binary_mqtt_ca_cert_pem_start");
  extern const char mqtt_ca_cert_end[] asm("_binary_mqtt_ca_cert_pem_end");
#endif

namespace mqtt = idf::mqtt;

namespace {

class MQTTClient final : public mqtt::Client {
public:
    using mqtt::Client::Client;

private:
    void on_connected(esp_mqtt_event_handle_t const event) override
    {
        using mqtt::QoS;
        subscribe(messages.get());
        subscribe(sent_load.get(), QoS::AtMostOnce);
    }
    void on_data(esp_mqtt_event_handle_t const event) override
    {
        if (messages.match(event->topic, event->topic_len)) {
            ESP_LOGI(MQTT, "Received in the messages topic");
        }
    }
    mqtt::Filter messages{"$SYS/broker/messages/received"};
    mqtt::Filter sent_load{"$SYS/broker/load/+/sent"};
};
}

    mqtt::BrokerConfiguration broker{
    .address = {
        mqtt::BrokerAddress{
            .address = {mqtt::URI{std::string{MQTT_BROKER_URL}}},
            .port = MQTT_BROKER_PORT
        }
    },
    
    #if MQTT_USE_TLS == true
      .security = mqtt::CryptographicInformation{mqtt::PEM{mqtt_ca_cert_start}}
    #else
      .security = mqtt::Insecure{}
    #endif
};

#if MQTT_USE_AUTH == true
mqtt::ClientCredentials credentials{
  .username = MQTT_USERNAME,
	.authentication = mqtt::Password{MQTT_PASSWORD}};
#else
mqtt::ClientCredentials credentials{};
#endif

mqtt::Configuration config{};

#endif


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
  ESP_LOGI(MAIN, "Putting bus to sleep...");
  rs485_enter_shutdown(&uart_dev);
  gpio_set_level(GPIO_NUM_15, 0); // Set the wake pin low to put slaves to sleep
  ESP_LOGI(MAIN, "Bus is now asleep.");
}

void wake_bus()
{
  // Wake up the RS485 bus
  ESP_LOGI(MAIN, "Waking up bus...");
  rs485_exit_shutdown(&uart_dev);
  gpio_set_level(GPIO_NUM_15, 1); // Set the wake pin high to wake slaves
  vTaskDelay(pdMS_TO_TICKS(100)); // Wait for slaves to wake up
  ESP_LOGI(MAIN, "Bus is now awake.");
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
	ESP_LOGI(MAIN, "Boot count: %d", boot_count);
	ESP_LOGI(MAIN, "Cold boot: %s", coldBoot ? "Yes" : "No");

  flash_led(2);

	// Initialize NVS
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
	    err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ESP_ERROR_CHECK(nvs_flash_init());
	}

  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  #if MQTT_ENABLE == true && DRY_RUN_MODE == false
  // Connect to Wi-Fi and sync time
  wifi_init_sta();  
  wait_for_time_sync();  

  // Connect to MQTT broker
  MQTTClient client{broker, credentials, config};
  vTaskDelay(pdMS_TO_TICKS(2000)); //wait a bit for it to connect
  #else
    ESP_LOGI(MAIN, "Skipping WiFi and MQTT initialisation.");
  #endif

	// Initialize radio and join LoRaWAN network
  #if LORAWAN_ENABLE == true && DRY_RUN_MODE == false
    ESP_LOGI(LORAWAN, "Initializing radio...");
    radioLibState = radio.begin();
    if (radioLibState != RADIOLIB_ERR_NONE) {
      ESP_LOGE(LORAWAN, "Failed to initialize radio: %d", radioLibState);
      return;
    }
    ESP_LOGI(LORAWAN, "Initializing node...");
    radioLibState = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
    if (radioLibState != RADIOLIB_ERR_NONE) {
      ESP_LOGE(LORAWAN, "Failed to initialize LoRaWAN node: %d", radioLibState);
      return;
    }
    radioLibState = lwActivate(node);
    node.setTxPower(14);
    node.setDutyCycle(false);
    node.setADR(false);
    node.setDatarate(4);

  #else
    ESP_LOGI(MAIN, "Skipping radio and LoRaWAN initialisation.");
  #endif

	// Init RS485 and WeatherBus
	sensorbus_init();

  wake_bus();

	// Restore slaves from RTC RAM if available
	if (!coldBoot && rtc_slave_count > 0) {
		slave_count = rtc_slave_count;
		memcpy(slaves, rtc_slaves, slave_count * sizeof(slave_entry_t));
		ESP_LOGI(WBUS, "Restored %zu slaves from RTC RAM", slave_count);
	}

	// Otherwise, if its a cold boot or no slaves are stored, discover slaves from scratch.
	if (coldBoot || slave_count == 0) {
		ESP_LOGI(WBUS, "Starting slave discovery...");
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
		ESP_LOGI(WBUS, "Saved %zu slaves to RTC RAM", slave_count);
		ESP_LOGI(WBUS, "%zu slave(s) discovered", slave_count);
	}

	// Query each slave and update readings
	for (size_t si = 0; si < slave_count; si++) {
		uint32_t dev_id = slaves[si].device_id;
		ESP_LOGI(WBUS, "Querying slave 0x%08" PRIX32, dev_id);

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
		if (sensorbus_receive_timeout(PKT_TIMEOUT_MS, &resp) != SENSORBUS_OK) { // Should probably check for correct response type/origin
			ESP_LOGW(WBUS, "Slave 0x%08" PRIX32 " timed out", dev_id);
			continue;
		}

		// Convert response payload to sensors array
		sensorbus_sensor_t out[SENSORBUS_MAX_TLVS];
		size_t outn = 0;
		if (sensorbus_pb_decode_sensors(resp.payload, resp.payload_len, out, &outn) != SENSORBUS_OK) {
			ESP_LOGW(WBUS, "bad TLV from slave 0x%08" PRIX32, dev_id);
			continue;
		}
		for (size_t k = 0; k < outn; k++) {
			update_sensor(&slaves[si], &out[k]);
		}
	}
	// Fancy logging
	for (size_t si = 0; si < slave_count; si++) {

		ESP_LOGI(WBUS, "┌── Slave 0x%08" PRIX32, slaves[si].device_id);

		for (size_t i = 0; i < slaves[si].sensor_count; i++) {
			const sensorbus_sensor_t &s = slaves[si].sensors[i];

			// Buffer to hold hex string of the sensor val
			char hexbuf[3 * SENSORBUS_MAX_VALUE_LEN + 1] = { 0 };

			// Convert sensor value to hex string
			for (uint8_t j = 0; j < SENSORBUS_FMT_LEN[s.format]; j++) {
				sprintf(&hexbuf[j * 3], "%02X ", s.value[j]);
			}

			// Log the sensor information
			ESP_LOGI(WBUS, "│ type=0x%02X idx=%u fmt=%u len=%u val=%s",
				 s.type,
				 s.index,
				 s.format,
				 SENSORBUS_FMT_LEN[s.format],
				 hexbuf);

		}
		ESP_LOGI(WBUS, "└──");
	}

	// Build and send the uplink.
	size_t uplink_len = 0;

	for (size_t si = 0; si < slave_count; ++si) {
		uplink_len += append_slave_payload(uplink_payload + uplink_len, &slaves[si]);
	}

	if (uplink_len > 0) {
		ESP_LOGI(MAIN, "Built uplink payload with %zu bytes", uplink_len);

    #if MQTT_ENABLE == true && DRY_RUN_MODE == false
      std::string json_payload = build_json_payload(slaves, slave_count);

      mqtt::Message msg = mqtt::Message<std::string>{
          .data = json_payload,
          .qos = mqtt::QoS::AtLeastOnce,
          .retain = mqtt::Retain::NotRetained,
      };

      auto msg_id = client.publish(MQTT_PUBLISH_TOPIC, msg);
      if (msg_id) {
          ESP_LOGI(MQTT, "Published JSON uplink with ID %d", static_cast<int>(*msg_id));
          ESP_LOGI(MQTT, "Payload: %s", json_payload.c_str());
      } else {
          ESP_LOGE(MQTT, "Failed to publish uplink");
      }

      vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for MQTT publish to complete
    #endif
    #if LORAWAN_ENABLE == true && DRY_RUN_MODE == false
        // Normal send process
        radioLibState = node.sendReceive(uplink_payload, uplink_len);
        ESP_LOGI(LORAWAN, "Radiolib code: %d, FCntUp now %" PRIu32,
          radioLibState, node.getFCntUp());
        memcpy(LWsession, node.getBufferSession(), RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
    #endif
    #if DRY_RUN_MODE == true
        // Dump the bytes instead of sending on dry-run mode
        printf("DRY_RUN_MODE enabled — would send uplink (%zu bytes):\n", uplink_len);
        for (size_t i = 0; i < uplink_len; i++)
          printf("%02X ", uplink_payload[i]);
        printf("\n");
    #endif
	} else {
		ESP_LOGW(MAIN, "Nothing to send (payload %zu bytes)", uplink_len);
	}

  sleep_bus();
	esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_US);
	ESP_LOGI(MAIN, "Entering deep sleep for %llu seconds...", (SLEEP_INTERVAL_US / 1000000ULL));
	esp_deep_sleep_start();
}
