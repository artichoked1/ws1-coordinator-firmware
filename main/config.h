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

// The time to dwell between measurements
#define SLEEP_INTERVAL_US (15 * 60 * 1000000ULL)


//--- LoRaWAN configuration ---//

// Uncomment this to enable dry-run mode, which skips actual LoRaWAN uplink
// transmission and just prints the payload that would be sent.
// Useful for testing without LoRaWAN connectivity and getting ratelimit'd or busted by the "bandwidth police".
// #define LORAWAN_DRY_RUN_MODE

// LoRaWAN keys for OTAA. Refer to the RadioLib documentation for details.
#define RADIOLIB_LORAWAN_JOIN_EUI 0x0000000000000000
#define RADIOLIB_LORAWAN_DEV_EUI 0x---------------
#define RADIOLIB_LORAWAN_APP_KEY 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--
#define RADIOLIB_LORAWAN_NWK_KEY 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--, 0x--

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
#define PKT_TIMEOUT_MS 200