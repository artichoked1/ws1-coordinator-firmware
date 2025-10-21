#pragma once

//-- General configuration ---//

// The SPI pins used for the LoRa module
#define SPI_MISO_PIN 12
#define SPI_MOSI_PIN 13
#define SPI_SCK_PIN 14

// The LoRa module's CS, IRQ, RST and BUSY pins
#define SX1276_CS_PIN 33
#define SX1276_IRQ_PIN 25 // DIO0
#define SX1276_RST_PIN 32
#define SX1276_BUSY_PIN 26 // DIO1, optional (i think)

// the wakeup line to wake all slaves on the bus
#define WAKE_GPIO_NUM GPIO_NUM_15

#define LED_PIN GPIO_NUM_4

// The time to dwell between measurements
#define SLEEP_INTERVAL_US (30 * 1000000ULL)

// RS485 UART config
#define UART_PORT UART_NUM_1
#define TX_PIN GPIO_NUM_17
#define RX_PIN GPIO_NUM_16
#define DE_PIN GPIO_NUM_18
#define RE_PIN GPIO_NUM_19
#define BAUD_RATE 9600

// Disables all form of rardio sending, dumps the would-be sent bytes to console instead.
#define DRY_RUN_MODE false

//--- LoRaWAN configuration ---//

#define LORAWAN_ENABLE false

// LoRaWAN keys for OTAA. Refer to the RadioLib documentation for details.
#define RADIOLIB_LORAWAN_JOIN_EUI 0x0000000000000000
#define RADIOLIB_LORAWAN_DEV_EUI 0x70B3D57ED0069153
#define RADIOLIB_LORAWAN_APP_KEY 0xE1, 0x03, 0x8F, 0x0B, 0xF6, 0x3D, 0x80, 0x80, 0x3D, 0x3B, 0xC6, 0x15, 0xA5, 0x24, 0x58, 0xD4
#define RADIOLIB_LORAWAN_NWK_KEY 0xC8, 0x7F, 0xC4, 0xFA, 0xF7, 0xD6, 0xFB, 0x72, 0x3E, 0x94, 0x0A, 0x7A, 0x33, 0xE0, 0x46, 0xC7

// LoRaWAN region and sub-band configuration. Make sure to set these according to your region's regulations.
#define RADIOLIB_LORAWAN_REGION AU915
#define RADIOLIB_LORAWAN_SUB_BAND 2

// the amount of deep sleep cycles before saving new session data to flash to avoid flash wear. On cold boots, a new session is always saved.
#define LORAWAN_SAVE_BOOTCOUNT 10


//--- WeatherBus configuration ---//

// The master ID for the WeatherBus. Not really important, since there is only one master.
#define MASTER_ID 0x11111111

// The maximum number of slaves that can be discovered on the bus.
// A block of static memory is allocated for this, so it should be set according to your needs.
#define MAX_SLAVES 8

// The maximum number of sensors that can be discovered per slave.
// This is also a static allocation, so it should be set according to your needs.
// Due to streaming support not yet added, it should be as large as the maximum number of sensors you can fit in a single packet.
#define MAX_SENSOR_CNT SENSORBUS_MAX_TLVS

// The window for discovery packets in milliseconds.
#define DISCOVERY_WINDOW_MS 3000

// The timeout for receiving packets in milliseconds.
#define PKT_TIMEOUT_MS 1000

//--- WiFi / MQTT configuration ---//

#define MQTT_ENABLE true

#define WIFI_SSID "ssid"
#define WIFI_PASS "very-secret"

#define MQTT_USE_V5 false
#define MQTT_USE_TLS false
#define MQTT_BROKER_URL "mqtt://test.mosquitto.org:1883"
#define MQTT_BROKER_PORT 8883

#define MQTT_USE_AUTH false
#define MQTT_USERNAME ""
#define MQTT_PASSWORD ""

#define MQTT_PUBLISH_TOPIC "weatherbus/coordinator/sensor_data"


