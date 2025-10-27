#include "modbus_rs485.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#define MODBUS_UART_NUM UART_NUM_1
#define BUF_SIZE        (256)
#define MODBUS_DE_PIN   4    // Pin DE/RE para control de dirección RS485
#define MODBUS_SLAVE_ID 1    // ID del esclavo Modbus
#define MODBUS_READ_CMD 0x03 // Leer registros (Holding Registers)

// Dirección inicial y número de registros
#define MODBUS_START_ADDR 1
#define MODBUS_NUM_REGS   5

static const char *TAG = "modbus_rs485";

/**
 * @brief Cálculo CRC16 Modbus (polinomio 0xA001, LSB primero)
 */
static uint16_t modbus_crc16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/**
 * @brief Envía solicitud Modbus y recibe respuesta
 */
static esp_err_t modbus_send_request(const uint8_t *request, size_t req_len,
                                     uint8_t *response, size_t *resp_len) {
    // Habilitar transmisión RS485 (DE = HIGH)
    gpio_set_level(MODBUS_DE_PIN, 1);
    uart_write_bytes(MODBUS_UART_NUM, (const char *)request, req_len);
    uart_wait_tx_done(MODBUS_UART_NUM, pdMS_TO_TICKS(100));

    // Deshabilitar transmisión (DE = LOW) → recepción
    gpio_set_level(MODBUS_DE_PIN, 0);

    // Esperar respuesta
    int len = uart_read_bytes(MODBUS_UART_NUM, response, BUF_SIZE, pdMS_TO_TICKS(500));
    if (len <= 0) {
        ESP_LOGE(TAG, "No se recibió respuesta Modbus (timeout)");
        return ESP_FAIL;
    }

    *resp_len = len;
    return ESP_OK;
}

/**
 * @brief Inicializa la interfaz RS485 (UART1)
 */
esp_err_t modbus_rs485_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(MODBUS_UART_NUM, &uart_config));

    // Pines: TX=17, RX=16, RTS=DE=4, CTS=-1
    ESP_ERROR_CHECK(uart_set_pin(MODBUS_UART_NUM, 17, 16, MODBUS_DE_PIN, -1));

    // Instalar driver UART
    ESP_ERROR_CHECK(uart_driver_install(MODBUS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));

    // Configurar pin DE como salida manual (por seguridad)
    gpio_set_direction(MODBUS_DE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MODBUS_DE_PIN, 0); // recepción por defecto

    ESP_LOGI(TAG, "UART RS485 inicializado correctamente (DE pin: %d).", MODBUS_DE_PIN);
    return ESP_OK;
}

/**
 * @brief Envía una solicitud de lectura Modbus (Read Holding Registers 0x03)
 *        y decodifica los valores recibidos.
 */
esp_err_t modbus_read_parameters(void) {
    uint8_t request[8];
    uint8_t response[BUF_SIZE];
    size_t resp_len = 0;

    // Construir trama solicitud: [Slave ID][Func][StartHi][StartLo][NumHi][NumLo][CRC_L][CRC_H]
    request[0] = MODBUS_SLAVE_ID;
    request[1] = MODBUS_READ_CMD;
    request[2] = (MODBUS_START_ADDR - 1) >> 8;  // direcciones en Modbus comienzan en 0
    request[3] = (MODBUS_START_ADDR - 1) & 0xFF;
    request[4] = (MODBUS_NUM_REGS >> 8) & 0xFF;
    request[5] = MODBUS_NUM_REGS & 0xFF;

    uint16_t crc = modbus_crc16(request, 6);
    request[6] = crc & 0xFF;       // CRC_L
    request[7] = (crc >> 8) & 0xFF; // CRC_H

    ESP_LOGI(TAG, "Enviando solicitud Modbus: leer %d registros desde %d (ID=%d)",
             MODBUS_NUM_REGS, MODBUS_START_ADDR, MODBUS_SLAVE_ID);

    esp_err_t ret = modbus_send_request(request, sizeof(request), response, &resp_len);
    if (ret != ESP_OK) return ret;

    // Validar longitud mínima
    if (resp_len < 5) {
        ESP_LOGE(TAG, "Respuesta demasiado corta (%d bytes)", (int)resp_len);
        return ESP_FAIL;
    }

    // Verificar CRC de respuesta
    uint16_t crc_calc = modbus_crc16(response, resp_len - 2);
    uint16_t crc_recv = response[resp_len - 2] | (response[resp_len - 1] << 8);
    if (crc_calc != crc_recv) {
        ESP_LOGE(TAG, "CRC inválido: calc=0x%04X recv=0x%04X", crc_calc, crc_recv);
        return ESP_FAIL;
    }

    // Validar código de función
    if (response[1] != MODBUS_READ_CMD) {
        ESP_LOGE(TAG, "Código de función inesperado: 0x%02X", response[1]);
        return ESP_FAIL;
    }

    uint8_t byte_count = response[2];
    ESP_LOGI(TAG, "Respuesta Modbus válida: %d bytes de datos", byte_count);

    // Decodificar registros (16 bits por registro)
    for (int i = 0; i < MODBUS_NUM_REGS; i++) {
        int index = 3 + i * 2;
        if (index + 1 >= resp_len - 2) break;

        uint16_t reg_val = (response[index] << 8) | response[index + 1];
        ESP_LOGI(TAG, "Registro %d = %u (0x%04X)", MODBUS_START_ADDR + i, reg_val, reg_val);
    }

    return ESP_OK;
}
