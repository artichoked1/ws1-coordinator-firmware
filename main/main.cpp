#include <RS485.h>
#include <RadioLib.h>
#include <WeatherBus.h>
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

uint64_t joinEUI = RADIOLIB_LORAWAN_JOIN_EUI;
uint64_t devEUI = RADIOLIB_LORAWAN_DEV_EUI;
uint8_t appKey[] = {RADIOLIB_LORAWAN_APP_KEY};
uint8_t nwkKey[] = {RADIOLIB_LORAWAN_NWK_KEY};

EspHal *hal = new EspHal(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

const LoRaWANBand_t Region = RADIOLIB_LORAWAN_REGION;
const uint8_t subBand = RADIOLIB_LORAWAN_SUB_BAND;

SX1276 radio = new Module(hal, SX1276_CS_PIN, SX1276_IRQ_PIN, SX1276_RST_PIN,
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

extern rs485_uart_t uart_dev;

RTC_DATA_ATTR int bootCount = 0;

//--- Helper Functions ---//

static void update_sensor(slave_entry_t *slave,
                          const sensorbus_sensor_t *incoming) {
  for (size_t i = 0; i < slave->sensor_count; i++) {
    if (slave->sensors[i].type == incoming->type &&
        slave->sensors[i].index == incoming->index) {
      slave->sensors[i].len = incoming->len;
      memcpy(slave->sensors[i].value, incoming->value, incoming->len);
      return;
    }
  }
}

// Builds a slim TILV uplink payload from an array of slave_entry_t
size_t build_tilv_lorawan_payload(const slave_entry_t *slaves,
                                  size_t slave_count, uint8_t *out_buf,
                                  size_t max_len) {
  size_t offset = 0;

  for (size_t s = 0; s < slave_count; s++) {
    const slave_entry_t *slave = &slaves[s];

    if (slave->sensor_count == 0) continue;  // skip empty slaves

    if (offset + 2 > max_len) return offset;  // space for short_id

    uint16_t short_id = (uint16_t)(slave->device_id & 0xFFFF);
    out_buf[offset++] = (short_id >> 8) & 0xFF;
    out_buf[offset++] = short_id & 0xFF;

    for (size_t i = 0; i < slave->sensor_count; i++) {
      const sensorbus_sensor_t *sensor = &slave->sensors[i];

      size_t tilv_len = 3 + sensor->len;
      if (offset + tilv_len > max_len) return offset;

      out_buf[offset++] = sensor->type;
      out_buf[offset++] = sensor->index;
      out_buf[offset++] = sensor->len;
      if (sensor->len > 0) {
        memcpy(&out_buf[offset], sensor->value, sensor->len);
      }

      offset += sensor->len;
    }
  }

  return offset;
}

void wakeBus() {
  // set interrupt pin high
  gpio_set_level(WAKE_GPIO_NUM, 1);
  // Wake up RS485 transceiver
  rs485_exit_shutdown(&uart_dev);
}

void sleepBus() {
  // Set interrupt pin low
  gpio_set_level(WAKE_GPIO_NUM, 0);
  // Put RS485 transceiver into shutdown mode
  rs485_enter_shutdown(&uart_dev);
}

// Main loop, let's go!

static const char *TAG = "main";

extern "C" void app_main(void) {
  esp_log_level_set("*", ESP_LOG_INFO);

  // Increment boot count and check if it's a cold boot
  bootCount++;
  esp_reset_reason_t reason = esp_reset_reason();
  bool coldBoot = (reason == ESP_RST_POWERON || reason == ESP_RST_BROWNOUT);
  ESP_LOGI(TAG, "Boot count: %d", bootCount);
  ESP_LOGI(TAG, "Cold boot: %s", coldBoot ? "Yes" : "No");

  // Set up NVS flash
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

#ifndef LORAWAN_DRY_RUN_MODE

  // Start the radio
  ESP_LOGI(TAG, "[SX1276] Initializing radio...");
  radioLibState = radio.begin();
  if (radioLibState != RADIOLIB_ERR_NONE) {
    ESP_LOGE(TAG, "Failed to initialize radio: %d", radioLibState);
    return;
  }

  // Set LoRaWAN activation information
  ESP_LOGI(TAG, "[LoRaWAN] Initializing node...");
  radioLibState = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
  if (radioLibState != RADIOLIB_ERR_NONE) {
    ESP_LOGE(TAG, "Failed to initialize LoRaWAN node: %d", radioLibState);
    return;
  }

  // Restore session and nonces from RTC RAM or flash, if available
  restoreBuffers(node, coldBoot);

  // Try to join the network
  ESP_LOGI(TAG, "[LoRaWAN] Joining network...");
  radioLibState = node.activateOTAA();
  if (radioLibState == RADIOLIB_LORAWAN_NEW_SESSION) {
    ESP_LOGI(TAG, "Join successful! New session started.");
  } else if (radioLibState == RADIOLIB_LORAWAN_SESSION_RESTORED) {
    ESP_LOGI(TAG, "Join successful! Session restored.");
  } else if (radioLibState == RADIOLIB_ERR_NONE) {
    ESP_LOGI(TAG, "Join successful!");
  } else {
    ESP_LOGE(TAG, "Join failed: %d", radioLibState);
    return;
  }
#else
  ESP_LOGI(TAG, "Dry-run mode: skipping radio and LoRaWAN initialization.");
#endif

  sensorbus_init();

  if (!coldBoot && rtc_slave_count > 0) {
    slave_count = rtc_slave_count;
    memcpy(slaves, rtc_slaves, slave_count * sizeof(slave_entry_t));
    ESP_LOGI(TAG, "Restored %u slaves from RTC RAM", slave_count);
  }

  if (coldBoot || slave_count == 0) {
    ESP_LOGI(TAG, "Starting slave discovery...");

    sensorbus_packet_t pkt;
    slave_count = 0;

    // Send a discovery packet
    sensorbus_send(SENSORBUS_DISCOVERY, MASTER_ID, NULL, 0);

    // Start a timer for discovery window
    uint32_t start = sensorbus_hal_get_time_ms();
    while (true) {
      uint32_t now = sensorbus_hal_get_time_ms();
      uint32_t elapsed = now - start;
      if (elapsed >= DISCOVERY_WINDOW_MS) break;  // exit if time is up

      uint32_t remaining = DISCOVERY_WINDOW_MS - elapsed;
      uint32_t to =
          remaining < PKT_TIMEOUT_MS
              ? remaining
              : PKT_TIMEOUT_MS;  // Timeout for receiving packets. Either it's
                                 // the default PKT_TIMEOUT_MS or the remaining
                                 // time left in the discovery window.

      // Wait for a packet
      sensorbus_error_t r = sensorbus_receive_timeout(to, &pkt);

      // Check for errors
      if (r == SENSORBUS_ERR_CRC) {
        ESP_LOGW(TAG, "CRC error when receiving discovery response!");
        continue;
      }
      if (r != SENSORBUS_OK && r != SENSORBUS_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Error receiving discovery response: %d", r);
        continue;
      }

      // Discard non‐discovery replies
      if (pkt.msg_type != SENSORBUS_DISCOVERY_REPLY) continue;

      // Decode TLVs
      sensorbus_sensor_t tmp[SNENSORBUS_MAX_SENSOR_DISCOVERY_SLOTS];
      size_t tmpn = 0;
      if (pb_decode_tlv(pkt.payload, pkt.payload_len, tmp, &tmpn) !=
          SENSORBUS_OK) {
        ESP_LOGW(TAG, "Failed to decode discovery TLV packet!");
        continue;
      }

      // find or add
      size_t si;

      // Check if we already have this slave. If we have a matching device ID,
      // we break out immediately leaving SI at the index of the slave.
      for (si = 0; si < slave_count; si++) {
        if (slaves[si].device_id == pkt.device_id) break;
      }

      // If we didn't find a matching slave, we make a brand new one!
      if (si == slave_count && slave_count < MAX_SLAVES) {
        ESP_LOGI(TAG, "Discovered new slave 0x%08" PRIX32, pkt.device_id);
        slaves[si].device_id = pkt.device_id;
        slaves[si].sensor_count = 0;
        slave_count++;
      }

      // Append sensor descriptors…
      for (size_t i = 0; i < tmpn && slaves[si].sensor_count < MAX_SENSOR_CNT;
           i++) {
        slaves[si].sensors[slaves[si].sensor_count++] = tmp[i];
      }
    }

    rtc_slave_count = slave_count;
    memcpy(rtc_slaves, slaves, slave_count * sizeof(slave_entry_t));
    ESP_LOGI(TAG, "Saved %u slaves to RTC RAM", slave_count);
    // Discovery process done!
    ESP_LOGI(TAG, "%u slave(s) discovered", (unsigned)slave_count);
  }

  // For each slave, query each sensor one by one
  for (size_t si = 0; si < slave_count; si++) {
    uint32_t dev_id = slaves[si].device_id;
    size_t sc = slaves[si].sensor_count;
    ESP_LOGI(TAG, "Slave 0x%08" PRIX32 " (%u sensors)", dev_id, (unsigned)sc);

    // Build a single “all-sensors” query, dropping any that don't fit
    payload_builder_t qb;
    pb_init(&qb);
    for (size_t j = 0; j < sc; j++) {
      // each query TLV is 3 bytes. Stop if we overflow the payload size. There
      // should be a better way to do this with streaming in the future.
      if (qb.len + 3 > SENSORBUS_MAX_PAYLOAD) {
        ESP_LOGW(TAG, "Dropped %u sensors (payload full)", (unsigned)(sc - j));
        break;
      }
      pb_add_query(&qb, &slaves[si].sensors[j]);
    }

    // Send it

    if (slaves[si].sensor_count == 0) {
      ESP_LOGW(TAG, "No sensors to query. Skipping.");
      continue;
    }

    ESP_LOGI(TAG, "Querying %u sensors", (unsigned)(qb.len / 3));

    sensorbus_send(SENSORBUS_QUERY, dev_id, qb.buf, qb.len);

    // Wait for response
    sensorbus_packet_t resp;
    if (sensorbus_receive_timeout(200, &resp) != SENSORBUS_OK) {
      ESP_LOGW(TAG, "Timed out waiting for query response");
      continue;
    }

    // Decode all returned TLVs in one go
    sensorbus_sensor_t out[SENSORBUS_MAX_TLVS];
    size_t outn = 0;
    if (pb_decode_tlv(resp.payload, resp.payload_len, out, &outn) !=
        SENSORBUS_OK) {
      ESP_LOGW(TAG, "bad TLV stream!");
      continue;
    }

    // Update the sensors in the slave entry
    for (size_t k = 0; k < outn; k++) {
      update_sensor(&slaves[si], &out[k]);
    }

    // Print each returned value
    for (size_t k = 0; k < outn; k++) {
      // pull out the raw bytes
      if (out[k].len == 4) {
        float v;
        memcpy(&v, out[k].value, sizeof(v));
        ESP_LOGI(TAG, "Sensor 0x%02X idx %u → %f", out[k].type,
                 (unsigned)out[k].index, v);
      } else {
        ESP_LOGI(TAG, "Sensor 0x%02X idx %u → %u-byte data", out[k].type,
                 (unsigned)out[k].index, (unsigned)out[k].len);
      }
    }
    ESP_LOGI(TAG, "All queries complete.");

    // Build a slim TILV uplink payload from the discovered slaves
    uint8_t uplink_buf[51];
    size_t uplink_len = build_tilv_lorawan_payload(
        slaves, slave_count, uplink_buf, sizeof(uplink_buf));

    if (uplink_len > 0 && uplink_len <= sizeof(uplink_buf)) {
      ESP_LOGI(TAG, "Built TILV payload with %zu bytes:\n", uplink_len);

#ifndef LORAWAN_DRY_RUN_MODE
      node.setADR(false);
      node.setDatarate(3);

      if (uplink_len <= 2) {
        ESP_LOGW(TAG, "Payload too small. It would be ridiculous. Skipping.\n");
      } else {
        ESP_LOGI(TAG, "[LoRaWAN] Sending uplink...\n");
        radioLibState = node.sendReceive(uplink_buf, uplink_len);
        if (radioLibState == RADIOLIB_ERR_NONE) {
          ESP_LOGI(TAG, "Uplink sent successfully!");
        } else {
          ESP_LOGE(TAG, "Failed to send uplink: %d", radioLibState);
        }
      }
      saveSessionToRTC(node);
      saveToFlashIfNeeded(node, bootCount);
#else
      printf("Dry-run mode: skipping uplink send.\n");
      printf("Uplink payload ready to send:\n");
      // Print payload as hex
      for (size_t i = 0; i < uplink_len; i++) {
        printf("%02X ", uplink_buf[i]);
      }
      printf("\n");
#endif
    } else {
      ESP_LOGW(TAG,"Failed to build TILV payload or payload too large or too small!");
    }

    // configure deep sleep
    esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_US);
    ESP_LOGI(TAG, "Entering deep sleep for %llu seconds...\n",
             (unsigned long long)(SLEEP_INTERVAL_US / 1000000ULL));
    fflush(stdout);
    esp_deep_sleep_start();
  }
}