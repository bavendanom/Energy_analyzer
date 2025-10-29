#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include <time.h>

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
 * @brief Ciclo de lectura Modbus y publicación MQTT
 */
static void task_modbus_mqtt_cycle(void *pvParameters) {
    const char *TAG = "task_modbus_mqtt";
    uint16_t registros[5]= {0};

    while (true) {
        ESP_LOGI(TAG, "[CICLO] Iniciando lectura Modbus...");
        esp_err_t ret = modbus_read_parameters(registros);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "[OK] Lectura Modbus completada correctamente.");
            ESP_LOGI(TAG, "Valores: %d, %d, %d, %d, %d",
                     registros[0], registros[1], registros[2], registros[3], registros[4]);
            // Obtener tiempo actual
            time_t timestamp;
            time(&timestamp);

            // Construir JSON real
            char json_payload[256];
            snprintf(json_payload, sizeof(json_payload),
                "{\"nodo_id\":\"ESP32_01\",\"timestamp\":\"%lld\","
                "\"registros\":{\"r1\":%d,\"r2\":%d,\"r3\":%d,\"r4\":%d,\"r5\":%d}}",
                (long long)timestamp,
                registros[0], registros[1], registros[2], registros[3], registros[4]);

            // Publicar
            mqtt_publish_data(MQTT_TOPIC, json_payload);
            ESP_LOGI(TAG, "[OK] Publicación MQTT completada: %s", json_payload);
        } else {
            ESP_LOGE(TAG, "[FAIL] Error en lectura Modbus (%s)", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 5 s entre lecturas
    }
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

    // Prueba de inicialización Wi-Fi
    ESP_ERROR_CHECK(test_wifi_connection());

    // Sincronización de tiempo vía NTP
    ESP_ERROR_CHECK(test_ntp_sync());

    // Inicialización de interfaz Modbus
    ESP_ERROR_CHECK(test_modbus_init());

    // 3️⃣ Inicialización del cliente MQTT
    ESP_LOGI(TAG, "[TEST] Inicializando cliente MQTT...");
    ESP_ERROR_CHECK(mqtt_app_start(MQTT_BROKER_URI));

    // Lectura Modbus y publicación MQTT real
    ESP_LOGI(TAG, "[TEST] Lectura de registros Modbus y publicación MQTT...");

    uint16_t registros[5]= {0};
    esp_err_t ret = modbus_read_parameters(registros);  // ← Lectura real

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "[OK] Lectura Modbus completada. Construyendo JSON...");

        // Obtener timestamp actual
        time_t timestamp;
        time(&timestamp);

        // Construir JSON con valores reales
        char json_payload[256];
        snprintf(json_payload, sizeof(json_payload),
            "{\"nodo_id\":\"ESP32_01\",\"timestamp\":\"%lld\","
            "\"registros\":{\"r1\":%d,\"r2\":%d,\"r3\":%d,\"r4\":%d,\"r5\":%d}}",
            (long long)timestamp,
            registros[0], registros[1], registros[2], registros[3], registros[4]);

        // Publicar al broker MQTT
        ret = mqtt_publish_data(MQTT_TOPIC, json_payload);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "[OK] Publicación MQTT completada: %s", json_payload);
        } else {
            ESP_LOGE(TAG, "[FAIL] Error publicando MQTT (%s)", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "[FAIL] Error leyendo registros Modbus (%s)", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "===============================================");
    ESP_LOGI(TAG, "  Todas las pruebas completadas. Sistema listo.");
    ESP_LOGI(TAG, "===============================================");   


    // Bucle principal simulado: ciclo de lectura cada 5 s
    // Iniciar tarea cíclica Modbus + MQTT
    xTaskCreate(task_modbus_mqtt_cycle, "task_modbus_mqtt_cycle", 4096, NULL, 5, NULL);

}
