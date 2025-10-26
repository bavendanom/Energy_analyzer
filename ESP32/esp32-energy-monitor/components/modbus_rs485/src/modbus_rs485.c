#include "modbus_rs485.h"
#include "driver/uart.h"
#include "esp_log.h"

#define MODBUS_UART_NUM UART_NUM_1
#define BUF_SIZE (256)

static const char *TAG = "modbus_rs485";

esp_err_t modbus_rs485_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(MODBUS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(MODBUS_UART_NUM, 17, 16, 18, -1));
    ESP_ERROR_CHECK(uart_driver_install(MODBUS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_LOGI(TAG, "UART RS485 inicializado correctamente.");
    return ESP_OK;
}

esp_err_t modbus_read_parameters(void) {
    ESP_LOGI(TAG, "Lectura Modbus simulada (implementación pendiente).");
    return ESP_OK;
}
