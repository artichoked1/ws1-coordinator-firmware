#include <weatherbus.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <RS485.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include "config.h"

rs485_uart_t uart_dev = {
	.uart_port	= UART_PORT,
	.tx_pin		= TX_PIN,
	.rx_pin		= RX_PIN,
	.de_pin		= DE_PIN,
	.re_pin		= RE_PIN,
	.baud_rate	= BAUD_RATE
};

sensorbus_error_t sensorbus_hal_uart_init()
{
	if (rs485_uart_init(&uart_dev) != ESP_OK)
		return SENSORBUS_ERR_FAILURE;
	return SENSORBUS_OK;
}

sensorbus_error_t sensorbus_hal_send_bytes(uint8_t *data, size_t len)
{
	if (rs485_uart_write(&uart_dev, data, len) != ESP_OK)
		return SENSORBUS_ERR_FAILURE;
	return SENSORBUS_OK;
}

sensorbus_error_t sensorbus_hal_receive_byte(uint8_t *byte, uint32_t timeout_ms)
{
	TickType_t ticks;

	if (timeout_ms == 0)
		ticks = portMAX_DELAY;
	else
		ticks = pdMS_TO_TICKS(timeout_ms);

	int r = uart_read_bytes(uart_dev.uart_port, byte, 1, ticks);

	if (r == 1)
		return SENSORBUS_OK;
	if (timeout_ms == 0)
		return SENSORBUS_ERR_FAILURE;
	return SENSORBUS_ERR_TIMEOUT;
}

void sensorbus_hal_delay_ms(uint32_t ms)
{
	vTaskDelay(pdMS_TO_TICKS(ms));
}

uint64_t sensorbus_hal_get_time_ms()
{
	return esp_timer_get_time() / 1000; // Convert microseconds to milliseconds
}

void sensorbus_hal_purge_rx(void)
{
	// drop any bytes in the UART driver
	uart_flush_input(UART_NUM_1);
}
