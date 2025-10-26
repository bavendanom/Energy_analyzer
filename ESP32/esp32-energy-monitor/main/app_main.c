#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "sdkconfig.h"

#define COMPONENTS_INCLUDES
#include "manejo_wifi.h"
#include "ntp_sync.h"
#include "modbus_rs485.h"
#include "mqtt_client_app.h"

// --- Constantes de configuración ---
#define WIFI_SSID       "Marcela"
#define WIFI_PASS       "24335095M"
#define NTP_SERVER      "pool.ntp.org"
#define MQTT_BROKER_URI "mqtt://192.168.1.86"
#define MQTT_TOPIC      "energia/nodo1/datos"

static const char *TAG = "app_main";

// ============================================================================
// Funciones auxiliares de testeo
// ============================================================================

/**
 * @brief Prueba unitaria: Conexión Wi-Fi
 */
static esp_err_t test_wifi_connection(void) {
    ESP_LOGI(TAG, "[TEST] Iniciando prueba de conexión Wi-Fi...");
    esp_err_t ret = wifi_init_sta(WIFI_SSID, WIFI_PASS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "[OK] Wi-Fi inicializado correctamente.");
    } else {
        ESP_LOGE(TAG, "[FAIL] Error en inicialización Wi-Fi (%s)", esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(5000)); // Esperar conexión
    return ret;
}

/**
 * @brief Prueba unitaria: Sincronización NTP
 */
static esp_err_t test_ntp_sync(void) {
    ESP_LOGI(TAG, "[TEST] Iniciando sincronización NTP...");
    esp_err_t ret = ntp_sync_time(NTP_SERVER);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "[OK] Hora sincronizada correctamente con NTP.");
    } else {
        ESP_LOGE(TAG, "[FAIL] Falló la sincronización NTP (%s)", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Prueba unitaria: Inicialización de comunicación Modbus RS485
 */
static esp_err_t test_modbus_init(void) {
    ESP_LOGI(TAG, "[TEST] Inicializando Modbus RS485...");
    esp_err_t ret = modbus_rs485_init();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "[OK] Interfaz RS485 lista.");
    } else {
        ESP_LOGE(TAG, "[FAIL] Error inicializando RS485 (%s)", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Prueba unitaria: Publicación MQTT
 */
static esp_err_t test_mqtt_publish(void) {
    ESP_LOGI(TAG, "[TEST] Iniciando cliente MQTT...");
    esp_err_t ret = mqtt_app_start(MQTT_BROKER_URI);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[FAIL] No se pudo iniciar el cliente MQTT.");
        return ret;
    }

    // Mensaje JSON simulado
    const char *payload =
        "{\"nodo_id\":\"ESP32_01\","
        "\"timestamp\":\"2025-10-20T18:45:30Z\","
        "\"mediciones\":{\"voltaje_V\":220.3,\"corriente_A\":1.26,"
        "\"potencia_W\":277.8,\"energia_kWh\":1.842},"
        "\"estado_modbus\":\"OK\",\"qos\":1}";

    ret = mqtt_publish_data(MQTT_TOPIC, payload);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "[OK] Mensaje MQTT publicado correctamente en %s", MQTT_TOPIC);
    } else {
        ESP_LOGE(TAG, "[FAIL] Error publicando mensaje MQTT (%s)", esp_err_to_name(ret));
    }

    return ret;
}

// ============================================================================
// app_main(): Punto de entrada principal
// ============================================================================

void app_main(void) {
    ESP_LOGI(TAG, "===============================================");
    ESP_LOGI(TAG, "  Proyecto: ESP32 Energy Monitor");
    ESP_LOGI(TAG, "  Versión: 1.0.0");
    ESP_LOGI(TAG, "  Autor: Brayan Avendaño Mesa");
    ESP_LOGI(TAG, "===============================================");

    // 1️⃣ Prueba de inicialización Wi-Fi
    ESP_ERROR_CHECK(test_wifi_connection());

    // 2️⃣ Sincronización de tiempo vía NTP
    ESP_ERROR_CHECK(test_ntp_sync());

    // 3️⃣ Inicialización de interfaz Modbus
    ESP_ERROR_CHECK(test_modbus_init());

    // 4️⃣ Publicación de datos MQTT simulados
    ESP_ERROR_CHECK(test_mqtt_publish());

    ESP_LOGI(TAG, "===============================================");
    ESP_LOGI(TAG, "  Todas las pruebas completadas. Sistema listo.");
    ESP_LOGI(TAG, "===============================================");

    // Bucle principal simulado: ciclo de lectura cada 5 s
    while (true) {
        ESP_LOGI(TAG, "[CICLO] Lectura y publicación cada 5 s...");
        // (En el futuro: integrar lectura real Modbus + publicación MQTT)
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
