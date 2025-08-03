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

int radioLibState;
extern RTC_DATA_ATTR uint8_t LWsession[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];

uint64_t joinEUI = RADIOLIB_LORAWAN_JOIN_EUI;
uint64_t devEUI = RADIOLIB_LORAWAN_DEV_EUI;
uint8_t appKey[] = {RADIOLIB_LORAWAN_APP_KEY};
uint8_t nwkKey[] = {RADIOLIB_LORAWAN_NWK_KEY};

EspHal *hal = new EspHal(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
const LoRaWANBand_t Region = RADIOLIB_LORAWAN_REGION;
const uint8_t subBand = RADIOLIB_LORAWAN_SUB_BAND;
SX1276 radio = new Module(hal, SX1276_CS_PIN, SX1276_IRQ_PIN, SX1276_RST_PIN, SX1276_BUSY_PIN);
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

static void update_sensor(slave_entry_t *slave, const sensorbus_sensor_t *incoming) {
  for (size_t i = 0; i < slave->sensor_count; i++) {
    if (slave->sensors[i].type == incoming->type && slave->sensors[i].index == incoming->index) {
      slave->sensors[i].len = incoming->len;
      memcpy(slave->sensors[i].value, incoming->value, incoming->len);
      return;
    }
  }
}

size_t build_tilv_lorawan_payload(const slave_entry_t *slaves, size_t slave_count, uint8_t *out_buf, size_t max_len) {
  size_t offset = 0;
  for (size_t s = 0; s < slave_count; s++) {
    const slave_entry_t *slave = &slaves[s];
    if (slave->sensor_count == 0) continue;
    if (offset + 2 > max_len) break;
    uint16_t short_id = (uint16_t)(slave->device_id & 0xFFFF);
    out_buf[offset++] = (short_id >> 8) & 0xFF;
    out_buf[offset++] = short_id & 0xFF;
    for (size_t i = 0; i < slave->sensor_count; i++) {
      const sensorbus_sensor_t *sensor = &slave->sensors[i];
      size_t tilv_len = 3 + sensor->len;
      if (offset + tilv_len > max_len) break;
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

static const char *TAG = "main";

extern "C" void app_main(void) {
  esp_log_level_set("*", ESP_LOG_INFO);

  bootCount++;
  esp_reset_reason_t reason = esp_reset_reason();
  bool warmWake = (reason == ESP_RST_DEEPSLEEP);
  bool coldBoot = !warmWake;
  ESP_LOGI(TAG, "Boot count: %d", bootCount);
  ESP_LOGI(TAG, "Cold boot: %s", coldBoot ? "Yes" : "No");

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

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
  node.setDutyCycle(false);
  node.setADR(false);
  node.setDatarate(3);
  node.setDwellTime(false);
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
    sensorbus_send(SENSORBUS_DISCOVERY, MASTER_ID, NULL, 0);
    uint32_t start = sensorbus_hal_get_time_ms();
    while (sensorbus_hal_get_time_ms() - start < DISCOVERY_WINDOW_MS) {
      sensorbus_error_t r = sensorbus_receive_timeout(PKT_TIMEOUT_MS, &pkt);
      if (r != SENSORBUS_OK) continue;
      if (pkt.msg_type != SENSORBUS_DISCOVERY_REPLY) continue;
      sensorbus_sensor_t tmp[SNENSORBUS_MAX_SENSOR_DISCOVERY_SLOTS];
      size_t tmpn = 0;
      if (pb_decode_tlv(pkt.payload, pkt.payload_len, tmp, &tmpn) != SENSORBUS_OK) continue;
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
    rtc_slave_count = slave_count;
    memcpy(rtc_slaves, slaves, slave_count * sizeof(slave_entry_t));
    ESP_LOGI(TAG, "Saved %u slaves to RTC RAM", slave_count);
    ESP_LOGI(TAG, "%u slave(s) discovered", (unsigned)slave_count);
  }

  // Query each slave and update readings
  for (size_t si = 0; si < slave_count; si++) {
    uint32_t dev_id = slaves[si].device_id;
    ESP_LOGI(TAG, "Querying slave 0x%08" PRIX32, dev_id);
    payload_builder_t qb;
    pb_init(&qb);
    for (size_t j = 0; j < slaves[si].sensor_count; j++) {
      if (qb.len + 3 > SENSORBUS_MAX_PAYLOAD) break;
      pb_add_query(&qb, &slaves[si].sensors[j]);
    }
    sensorbus_send(SENSORBUS_QUERY, dev_id, qb.buf, qb.len);
    sensorbus_packet_t resp;
    if (sensorbus_receive_timeout(200, &resp) != SENSORBUS_OK) {
      ESP_LOGW(TAG, "Slave 0x%08" PRIX32 " timed out", dev_id);
      continue;
    }
    sensorbus_sensor_t out[SENSORBUS_MAX_TLVS];
    size_t outn = 0;
    if (pb_decode_tlv(resp.payload, resp.payload_len, out, &outn) != SENSORBUS_OK) {
      ESP_LOGW(TAG, "bad TLV from slave 0x%08" PRIX32, dev_id);
      continue;
    }
    for (size_t k = 0; k < outn; k++) {
      update_sensor(&slaves[si], &out[k]);
    }
  }

  // Build & send a single LoRaWAN uplink
  uint8_t uplink_buf[51];
  size_t uplink_len = build_tilv_lorawan_payload(slaves, slave_count, uplink_buf, sizeof(uplink_buf));
  if (uplink_len > 2) {
    ESP_LOGI(TAG, "Built TILV payload with %zu bytes", uplink_len);
    radioLibState = node.sendReceive(uplink_buf, uplink_len);
    ESP_LOGI(TAG, "sendReceive → %d, FCntUp now %" PRIu32, radioLibState, node.getFCntUp());
    const uint8_t* persist = node.getBufferSession();
    memcpy(LWsession, persist, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
  } else {
    ESP_LOGW(TAG, "Nothing to send (payload %zu bytes)", uplink_len);
  }

  esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_US);
  ESP_LOGI(TAG, "Entering deep sleep for %u seconds…", (unsigned)(SLEEP_INTERVAL_US/1000000ULL));
  esp_deep_sleep_start();
}

