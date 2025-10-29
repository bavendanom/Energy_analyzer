#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "sdkconfig.h"

#include "manejo_wifi.h"
#include "ntp_sync.h"
#include "modbus_rs485.h"
#include "mqtt_client_app.h"
#include "led_status.h"

// ============================================================================
// 🔧 CONFIGURACIÓN GENERAL
// ============================================================================

#define WIFI_SSID       "Marcela"
#define WIFI_PASS       "24335095M"
#define NTP_SERVER      "pool.ntp.org"
#define MQTT_BROKER_URI "mqtt://192.168.1.86"
#define MQTT_TOPIC      "energia/nodo1/datos"

// Intervalo de publicación (ms)
#define MODBUS_MQTT_INTERVAL_MS 5000

// LED indicador (LED integrado en la ESP32)
#define STATUS_LED_GPIO 2
// Inicialización LEDs
#define LED_WIFI_PIN    18
#define LED_MQTT_PIN    19
#define LED_MODBUS_PIN  21


static const char *TAG = "app_main";

// ============================================================================
// 🧩 FUNCIONES AUXILIARES DE PRUEBA UNITARIA
// ============================================================================

static esp_err_t test_wifi_connection(void) {
    ESP_LOGI(TAG, "[TEST] Iniciando prueba de conexión Wi-Fi...");
    esp_err_t ret = wifi_init_sta(WIFI_SSID, WIFI_PASS);
    if (ret == ESP_OK) ESP_LOGI(TAG, "[OK] Wi-Fi inicializado correctamente.");
    else ESP_LOGE(TAG, "[FAIL] Error en inicialización Wi-Fi (%s)", esp_err_to_name(ret));
    vTaskDelay(pdMS_TO_TICKS(4000));  // Esperar estabilización
    return ret;
}

static esp_err_t test_ntp_sync(void) {
    ESP_LOGI(TAG, "[TEST] Iniciando sincronización NTP...");
    esp_err_t ret = ntp_sync_time(NTP_SERVER);
    if (ret == ESP_OK) ESP_LOGI(TAG, "[OK] Hora sincronizada correctamente con NTP.");
    else ESP_LOGE(TAG, "[FAIL] Falló la sincronización NTP (%s)", esp_err_to_name(ret));
    return ret;
}

static esp_err_t test_modbus_init(void) {
    ESP_LOGI(TAG, "[TEST] Inicializando Modbus RS485...");
    esp_err_t ret = modbus_rs485_init();
    if (ret == ESP_OK) ESP_LOGI(TAG, "[OK] Interfaz RS485 lista.");
    else ESP_LOGE(TAG, "[FAIL] Error inicializando RS485 (%s)", esp_err_to_name(ret));
    return ret;
}

// ============================================================================
// ✅ VERIFICACIÓN DE ESTADO ANTES DE COMENZAR CICLO PRINCIPAL
// ============================================================================

static bool system_check_ready(void) {
    ESP_LOGI(TAG, "[CHECK] Verificando estado del sistema...");

    // Wi-Fi
    if (!wifi_is_connected()) {
        ESP_LOGE(TAG, "[FAIL] Wi-Fi no conectado.");
        led_status_set(LED_WIFI_PIN, LED_BLINK);
        return ESP_FAIL;
    } else {
        led_status_set(LED_WIFI_PIN, LED_ON);
        ESP_LOGI(TAG, "[OK] Wi-Fi conectado.");
    }

    // 2️⃣ NTP (hora válida)
    time_t now;
    time(&now);
    if (now < 100000) {
        ESP_LOGE(TAG, "[FAIL] Hora NTP no sincronizada.");
        return false;
    }

    // MQTT
    if (!mqtt_is_ready()) {
        ESP_LOGE(TAG, "[FAIL] MQTT no listo.");
        led_status_set(LED_MQTT_PIN, LED_BLINK);
        return ESP_FAIL;
    } else {
        led_status_set(LED_MQTT_PIN, LED_ON);
        ESP_LOGI(TAG, "[OK] MQTT conectado.");
    }

    // 4️⃣ Modbus (prueba de lectura mínima)
    uint16_t dummy[1];
    if (modbus_read_parameters(dummy) != ESP_OK) {
        ESP_LOGE(TAG, "[FAIL] Comunicación Modbus fallida.");
        return false;
    }

    ESP_LOGI(TAG, "[OK] Todas las comprobaciones pasaron correctamente");
    return true;
}

// ============================================================================
// 🔁 TAREA PRINCIPAL: CICLO MODBUS + MQTT
// ============================================================================

static void task_modbus_mqtt_cycle(void *pvParameters) {
    const char *TAG = "task_modbus_mqtt";
    uint16_t registros[5] = {0};

    while (true) {
        ESP_LOGI(TAG, "[CICLO] Iniciando lectura Modbus...");
        esp_err_t ret = modbus_read_parameters(registros);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "[OK] Lectura Modbus completada correctamente.");

            // Obtener timestamp actual
            time_t timestamp;
            time(&timestamp);

            // Construir JSON real
            char json_payload[256];
            snprintf(json_payload, sizeof(json_payload),
                "{\"nodo_id\":\"ESP32_01\",\"timestamp\":\"%lld\","
                "\"registros\":{\"r1\":%d,\"r2\":%d,\"r3\":%d,\"r4\":%d,\"r5\":%d}}",
                (long long)timestamp,
                registros[0], registros[1], registros[2], registros[3], registros[4]);

            // Publicar al broker
            if (mqtt_publish_data(MQTT_TOPIC, json_payload) == ESP_OK) {
                ESP_LOGI(TAG, "[OK] Publicación MQTT completada: %s", json_payload);
            } else {
                ESP_LOGE(TAG, "[FAIL] Error publicando MQTT.");
            }

            // 🔆 Parpadeo de LED como indicador de ciclo completado
            gpio_set_level(STATUS_LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(STATUS_LED_GPIO, 0);

        } else {
            led_status_set(LED_MODBUS_PIN, LED_BLINK);
            ESP_LOGE(TAG, "[FAIL] Error en lectura Modbus (%s)", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(MODBUS_MQTT_INTERVAL_MS));
    }
}

// ============================================================================
// 🏁 app_main(): PUNTO DE ENTRADA PRINCIPAL
// ============================================================================

void app_main(void) {
    ESP_LOGI(TAG, "===============================================");
    ESP_LOGI(TAG, "  Proyecto: ESP32 Energy Monitor");
    ESP_LOGI(TAG, "  Versión: 1.0.1 (Fase D)");
    ESP_LOGI(TAG, "  Autor: Brayan Avendaño Mesa");
    ESP_LOGI(TAG, "===============================================");

    // Configurar LED indicador
    gpio_reset_pin(STATUS_LED_GPIO);
    gpio_set_direction(STATUS_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(STATUS_LED_GPIO, 0);

    // Inicialización LEDs
    led_status_init(LED_WIFI_PIN);
    led_status_init(LED_MQTT_PIN);
    led_status_init(LED_MODBUS_PIN);

    // 🔹 Inicializaciones secuenciales
    ESP_ERROR_CHECK(test_wifi_connection());
    ESP_ERROR_CHECK(test_ntp_sync());
    ESP_ERROR_CHECK(test_modbus_init());
    ESP_ERROR_CHECK(mqtt_app_start(MQTT_BROKER_URI));

    // 🔹 Comprobaciones de estado antes de iniciar tareas
    while (!system_check_ready()) {
        ESP_LOGW(TAG, "[WAIT] Algunos servicios no están listos. Reintentando...");
        // Parpadeo lento indicando espera
        gpio_set_level(STATUS_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(STATUS_LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(2500));
        
    }

    ESP_LOGI(TAG, "[START] Sistema listo. Iniciando tarea principal Modbus + MQTT...");
    gpio_set_level(STATUS_LED_GPIO, 1);  // LED fijo: sistema activo

    // Iniciar la tarea de ciclo continuo
    xTaskCreate(task_modbus_mqtt_cycle, "task_modbus_mqtt_cycle", 4096, NULL, 5, NULL);
}
