#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include <string.h>

#include "manejo_wifi.h"
#include "ntp_sync.h"
#include "modbus_rs485.h"
#include "mqtt_client_app.h"
#include "led_status.h"

// ============================================================================
// MARK: CONFIGURACIÓN GENERAL
// ============================================================================

#define WIFI_SSID       "Marcela"
#define WIFI_PASS       "24335095M"
#define NTP_SERVER      "pool.ntp.org"
#define MQTT_BROKER_URI "mqtt://192.168.1.86"
#define MQTT_TOPIC      "energia/nodo1/datos"

#define MODBUS_MQTT_INTERVAL_MS 5000

#define STATUS_LED_GPIO 2
#define LED_WIFI_PIN    18
#define LED_MQTT_PIN    19
#define LED_MODBUS_PIN  21

static const char *TAG = "app_main";

// Variables de estado
static bool modbus_last_status = false;
static int modbus_consecutive_failures = 0;
#define MODBUS_FAILURE_THRESHOLD 3

#define BUFFER_SIZE 20
static char message_buffer[BUFFER_SIZE][400];
static int buffer_head = 0;
static int buffer_tail = 0;
static int buffer_count = 0;


// Función auxiliar para agregar al buffer
static void buffer_add(const char *json) {
    if (buffer_count < BUFFER_SIZE) {
        strcpy(message_buffer[buffer_head], json);
        buffer_head = (buffer_head + 1) % BUFFER_SIZE;
        buffer_count++;
        ESP_LOGI(TAG, "Mensaje guardado en buffer local (%d/%d)", buffer_count, BUFFER_SIZE);
    } else {
        ESP_LOGW(TAG, "Buffer lleno, descartando mensaje más antiguo");
        buffer_tail = (buffer_tail + 1) % BUFFER_SIZE;
    }
}
// ============================================================================
// MARK: CALLBACKS DE ESTADO
// ============================================================================

static void wifi_status_changed(bool connected) {
    if (connected) {
        ESP_LOGI(TAG, "✅ [WiFi] Estado: CONECTADO");
        led_status_set(LED_WIFI_PIN, LED_ON);
    } else {
        ESP_LOGW(TAG, "❌ [WiFi] Estado: DESCONECTADO");
        led_status_set(LED_WIFI_PIN, LED_BLINK);
    }
}

static void mqtt_status_changed(bool connected) {
    if (connected) {
        ESP_LOGI(TAG, "✅ [MQTT] Estado: CONECTADO");
        led_status_set(LED_MQTT_PIN, LED_ON);
    } else {
        ESP_LOGW(TAG, "❌ [MQTT] Estado: DESCONECTADO");
        led_status_set(LED_MQTT_PIN, LED_BLINK);
    }
}

// ============================================================================
// MARK: FUNCIONES DE INICIALIZACIÓN
// ============================================================================

static esp_err_t test_wifi_connection(void) {
    ESP_LOGI(TAG, "[INIT] Inicializando WiFi...");
    
    // CRÍTICO: Registrar callback ANTES de inicializar
    wifi_set_status_callback(wifi_status_changed);
    
    esp_err_t ret = wifi_init_sta(WIFI_SSID, WIFI_PASS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "[OK] WiFi inicializado correctamente.");
    } else {
        ESP_LOGE(TAG, "[FAIL] Error en inicialización WiFi (%s)", esp_err_to_name(ret));
        return ret;
    }
    
    // Esperar conexión inicial
    vTaskDelay(pdMS_TO_TICKS(4000));
    return ret;
}

static esp_err_t test_ntp_sync(void) {
    ESP_LOGI(TAG, "[INIT] Sincronizando NTP...");
    esp_err_t ret = ntp_sync_time(NTP_SERVER);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "[OK] Hora sincronizada con NTP.");
    } else {
        ESP_LOGE(TAG, "[FAIL] Error en sincronización NTP (%s)", esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t test_modbus_init(void) {
    ESP_LOGI(TAG, "[INIT] Inicializando Modbus RS485...");
    
    // Paso 1: Inicializar UART
    esp_err_t ret = modbus_rs485_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[FAIL] Error inicializando UART RS485 (%s)", esp_err_to_name(ret));
        led_status_set(LED_MODBUS_PIN, LED_BLINK);
        modbus_last_status = false;
        return ret;
    }
    
    ESP_LOGI(TAG, "[OK] UART RS485 inicializado.");
    
    // Paso 2: Probar comunicación real con dispositivo KLEA 220P
    ESP_LOGI(TAG, "[TEST] Verificando dispositivo Modbus...");
    uint16_t test_regs[5];
    ret = modbus_read_parameters(test_regs);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ [OK] Dispositivo respondiendo.");
        led_status_set(LED_MODBUS_PIN, LED_ON);
        modbus_last_status = true;
        modbus_consecutive_failures = 0;
    } else {
        ESP_LOGW(TAG, "❌ [FAIL] Dispositivo no responde (desconectado o apagado).");
        led_status_set(LED_MODBUS_PIN, LED_BLINK);
        modbus_last_status = false;
        modbus_consecutive_failures = 1;  // Iniciar contador
    }
    
    // Siempre retornar OK para permitir que el sistema continúe
    return ESP_OK;
}

static esp_err_t test_mqtt_init(void) {
    ESP_LOGI(TAG, "[INIT] Inicializando cliente MQTT...");
    
    // CRÍTICO: Registrar callback ANTES de inicializar
    mqtt_set_status_callback(mqtt_status_changed);
    
    esp_err_t ret = mqtt_app_start(MQTT_BROKER_URI);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "[OK] Cliente MQTT inicializado.");
    } else {
        ESP_LOGE(TAG, "[FAIL] Error inicializando MQTT (%s)", esp_err_to_name(ret));
    }
    return ret;
}

// ============================================================================
// MARK: VERIFICACIÓN DE ESTADO
// ============================================================================

static bool system_check_ready(void) {
    bool all_ready = true;

    // WiFi - Solo verificar (el callback maneja el LED)
    if (!wifi_is_connected()) {
        ESP_LOGW(TAG, "[CHECK] WiFi no conectado");
        all_ready = false;
    }

    // NTP
    time_t now;
    time(&now);
    if (now < 1577836800) {
        ESP_LOGW(TAG, "[CHECK] Hora NTP no sincronizada (timestamp: %ld)", (long)now);
        all_ready = false;
    }

    // MQTT - Solo verificar (el callback maneja el LED)
    if (!mqtt_is_connected()) {
        ESP_LOGW(TAG, "[CHECK] MQTT no conectado");
        all_ready = false;
    }

    // ⚠️ MODBUS - Verificar Y actualizar LED manualmente
    uint16_t dummy[5];
    esp_err_t modbus_ret = modbus_read_parameters(dummy);
    
    if (modbus_ret == ESP_OK) {
        // Dispositivo responde
        if (!modbus_last_status) {
            // Recuperación: estaba fallando, ahora OK
            ESP_LOGI(TAG, "✅ [Modbus] Dispositivo recuperado");
            led_status_set(LED_MODBUS_PIN, LED_ON);
            modbus_last_status = true;
            modbus_consecutive_failures = 0;
        }
    } else {
        // Dispositivo no responde
        modbus_consecutive_failures++;
        
        if (modbus_consecutive_failures >= MODBUS_FAILURE_THRESHOLD && modbus_last_status) {
            // Primera vez que alcanza el umbral de fallo
            ESP_LOGW(TAG, "❌ [Modbus] El dispositivo no responde (%d intentos)", MODBUS_FAILURE_THRESHOLD);
            led_status_set(LED_MODBUS_PIN, LED_BLINK);
            modbus_last_status = false;
        }
        
        ESP_LOGW(TAG, "[CHECK] Modbus: %d/%d fallos consecutivos", 
                 modbus_consecutive_failures, MODBUS_FAILURE_THRESHOLD);
        all_ready = false;
    }

    if (all_ready) {
        ESP_LOGI(TAG, "✅ [CHECK] Todos los servicios operativos");
    } else {
        ESP_LOGW(TAG, "⚠️  [CHECK] Algunos servicios no están listos");
    }
    
    return all_ready;
}

// ============================================================================
// MARK: TAREA PRINCIPAL: CICLO MODBUS + MQTT
// ============================================================================

static void task_modbus_mqtt_cycle(void *pvParameters) {
    const char *TASK_TAG = "modbus_mqtt";
    uint16_t registros[5] = {0};

    ESP_LOGI(TASK_TAG, "Tarea iniciada. Intervalo: %d ms", MODBUS_MQTT_INTERVAL_MS);
    ESP_LOGI(TASK_TAG, "Formato JSON conforme a RF2 (QoS=1 con PUBACK)");

    while (true) {
        esp_err_t ret = modbus_read_parameters(registros);

        // Al inicio del loop, intentar enviar buffer pendiente:
        while (buffer_count > 0 && mqtt_is_connected()) {
            if (mqtt_publish_data(MQTT_TOPIC, message_buffer[buffer_tail]) == ESP_OK) {
                ESP_LOGI(TASK_TAG, "Mensaje del buffer enviado (%d restantes)", buffer_count-1);
                buffer_tail = (buffer_tail + 1) % BUFFER_SIZE;
                buffer_count--;
            } else {
                break;  // Si falla, intentar en el próximo ciclo
            }
        }

        if (ret == ESP_OK) {
            // ✅ Lectura exitosa del KLEA 220P
            if (modbus_consecutive_failures >= MODBUS_FAILURE_THRESHOLD) {
                ESP_LOGI(TASK_TAG, "✅ [Modbus] KLEA 220P recuperado después de %d fallos", 
                         modbus_consecutive_failures);
                led_status_set(LED_MODBUS_PIN, LED_ON);
                modbus_last_status = true;
            }
            modbus_consecutive_failures = 0;

            // Obtener timestamp
            time_t timestamp;
            time(&timestamp);
            
            // Convertir a formato ISO 8601 (RFC2 requisito)
            struct tm timeinfo;
            localtime_r(&timestamp, &timeinfo);
            char timestamp_iso[30];
            strftime(timestamp_iso, sizeof(timestamp_iso), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);

            // ⚠️ CONVERSIÓN DE REGISTROS MODBUS A PARÁMETROS ELÉCTRICOS
            // AJUSTA ESTOS FACTORES SEGÚN EL MANUAL DE TU KLEA 220P
            float voltaje_V = registros[0] / 10.0f;      // Ejemplo: r1=2203 → 220.3V
            float corriente_A = registros[1] / 100.0f;   // Ejemplo: r2=126 → 1.26A
            float potencia_W = registros[2];             // Ejemplo: r3=277 → 277W
            float energia_kWh = registros[3] / 1000.0f;  // Ejemplo: r4=1842 → 1.842kWh

            // Construir JSON según especificación RF2
            char json_payload[400];
            int len = snprintf(json_payload, sizeof(json_payload),
                "{"
                "\"nodo_id\":\"ESP32_01\","
                "\"timestamp\":\"%s\","
                "\"mediciones\":{"
                    "\"voltaje_V\":%.1f,"
                    "\"corriente_A\":%.2f,"
                    "\"potencia_W\":%.1f,"
                    "\"energia_kWh\":%.3f"
                "},"
                "\"estado_modbus\":\"OK\","
                "\"qos\":1"
                "}",
                timestamp_iso,
                voltaje_V, corriente_A, potencia_W, energia_kWh
            );

            if (len >= sizeof(json_payload)) {
                ESP_LOGW(TASK_TAG, "JSON truncado, aumentar buffer");
            }

            // Log detallado para verificación RF2
            ESP_LOGI(TASK_TAG, "[MODBUS] Lectura exitosa:");
            ESP_LOGI(TASK_TAG, "         Voltaje=%.1f V, Corriente=%.2f A, Potencia=%.1f W, Energia=%.3f kWh",
                     voltaje_V, corriente_A, potencia_W, energia_kWh);
            ESP_LOGI(TASK_TAG, "[JSON] Mensaje generado (longitud: %d bytes)", len);

            // Publicar MQTT con QoS=1
            ESP_LOGI(TASK_TAG, "[MQTT] Publicando en %s ...", MQTT_TOPIC);
            if (mqtt_publish_data(MQTT_TOPIC, json_payload) == ESP_OK) {
                // El PUBACK se loggeará automáticamente en mqtt_event_handler
                ESP_LOGI(TASK_TAG, "[MQTT] Publicación completada correctamente.");
                ESP_LOGI(TASK_TAG, "-------------------------------------------------------");
                
                // Flash rápido LED principal
                gpio_set_level(STATUS_LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(50));
                gpio_set_level(STATUS_LED_GPIO, 0);
            } else {
                ESP_LOGE(TASK_TAG, "[MQTT] Error en publicación");
            }

            if (mqtt_publish_data(MQTT_TOPIC, json_payload) != ESP_OK) {
                buffer_add(json_payload);  // Guardar localmente
            }

        } else {
            // ❌ Fallo de lectura del KLEA 220P
            modbus_consecutive_failures++;
            
            if (modbus_consecutive_failures == MODBUS_FAILURE_THRESHOLD) {
                ESP_LOGE(TASK_TAG, "❌ [Modbus] KLEA 220P no responde (fallo persistente)");
                led_status_set(LED_MODBUS_PIN, LED_BLINK);
                modbus_last_status = false;
            }
            
            ESP_LOGE(TASK_TAG, "Error Modbus: %d/%d fallos consecutivos", 
                     modbus_consecutive_failures, MODBUS_FAILURE_THRESHOLD);
                     
            // Publicar estado de error también (opcional pero recomendado)
            time_t timestamp;
            time(&timestamp);
            struct tm timeinfo;
            localtime_r(&timestamp, &timeinfo);
            char timestamp_iso[30];
            strftime(timestamp_iso, sizeof(timestamp_iso), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
            
            char json_error[250];
            snprintf(json_error, sizeof(json_error),
                "{\"nodo_id\":\"ESP32_01\","
                "\"timestamp\":\"%s\","
                "\"estado_modbus\":\"ERROR\","
                "\"fallos_consecutivos\":%d,"
                "\"qos\":1}",
                timestamp_iso, modbus_consecutive_failures
            );
            
            mqtt_publish_data(MQTT_TOPIC, json_error);
        }

        ESP_LOGI(TASK_TAG, "[MODBUS] Nueva lectura en %.1f s...", MODBUS_MQTT_INTERVAL_MS / 1000.0);
        vTaskDelay(pdMS_TO_TICKS(MODBUS_MQTT_INTERVAL_MS));
    }
}
// ============================================================================
//MARK: app_main()
// ============================================================================

void app_main(void) {
    ESP_LOGI(TAG, "===============================================");
    ESP_LOGI(TAG, "  ESP32 Energy Monitor v1.1.0");
    ESP_LOGI(TAG, "  Autor: Brayan Avendaño Mesa");
    ESP_LOGI(TAG, "  Monitoreo dinámico de estado con LEDs");
    ESP_LOGI(TAG, "===============================================");


    // Configurar LED de estado principal
    gpio_reset_pin(STATUS_LED_GPIO);
    gpio_set_direction(STATUS_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(STATUS_LED_GPIO, 0);

    // Inicializar LEDs de estado (timers no bloqueantes)
    ESP_LOGI(TAG, "[INIT] Configurando LEDs de estado...");
    led_status_init(LED_WIFI_PIN);
    led_status_init(LED_MQTT_PIN);
    led_status_init(LED_MODBUS_PIN);
    
    // Después de led_status_init()
    ESP_LOGI(TAG, "[DEBUG] Probando LED WiFi en modo BLINK...");
    led_status_set(LED_WIFI_PIN, LED_BLINK);
    vTaskDelay(pdMS_TO_TICKS(5000));  // Observar 5 segundos
    led_status_set(LED_WIFI_PIN, LED_OFF);

    // Test visual: parpadear todos los LEDs
    ESP_LOGI(TAG, "[TEST] Probando LEDs...");
    led_status_set(LED_WIFI_PIN, LED_ON);
    vTaskDelay(pdMS_TO_TICKS(300));
    led_status_set(LED_MQTT_PIN, LED_ON);
    vTaskDelay(pdMS_TO_TICKS(300));
    led_status_set(LED_MODBUS_PIN, LED_ON);
    vTaskDelay(pdMS_TO_TICKS(500));
    led_status_set(LED_WIFI_PIN, LED_OFF);
    led_status_set(LED_MQTT_PIN, LED_OFF);
    led_status_set(LED_MODBUS_PIN, LED_OFF);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Inicializaciones secuenciales
    ESP_ERROR_CHECK(test_wifi_connection());
    ESP_ERROR_CHECK(test_ntp_sync());
    ESP_ERROR_CHECK(test_modbus_init());
    ESP_ERROR_CHECK(test_mqtt_init());

    // Pequeña pausa para estabilización
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Esperar a que todos los servicios estén listos
    int check_attempts = 0;
    const int max_checks = 20;  // Máximo 1 minuto de espera
    
    while (!system_check_ready() && check_attempts < max_checks) {
        check_attempts++;
        ESP_LOGW(TAG, "[WAIT] Sistema no listo. Intento %d/%d...", check_attempts, max_checks);
        
        // Parpadeo del LED principal (3 pulsos de 250ms)
        for (int i = 0; i < 3; i++) {
            gpio_set_level(STATUS_LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(250));
            gpio_set_level(STATUS_LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(250));
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));  // Esperar 2s antes del siguiente check
    }

    if (check_attempts >= max_checks) {
        ESP_LOGE(TAG, "❌ Sistema no alcanzó estado listo después de %d intentos", max_checks);
        ESP_LOGE(TAG, "⚠️  Continuando de todas formas...");
    }

    ESP_LOGI(TAG, "✅ [START] Sistema operativo. Iniciando ciclo principal...");
    gpio_set_level(STATUS_LED_GPIO, 1);  // LED fijo: sistema activo

    // Crear tarea principal con prioridad 5
    BaseType_t task_created = xTaskCreate(
        task_modbus_mqtt_cycle,
        "modbus_mqtt",
        4096,
        NULL,
        5,
        NULL
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "❌ Error crítico: No se pudo crear la tarea principal");
        esp_restart();
    }

    ESP_LOGI(TAG, "Sistema en ejecución. Monitoreando...");
}