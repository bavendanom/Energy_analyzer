#include "mqtt_client_app.h"
#include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG = "mqtt_client_app";
static esp_mqtt_client_handle_t client = NULL;
static bool mqtt_connected = false;
static mqtt_status_callback_t status_callback = NULL;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                               int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT CONECTADO al broker");
            mqtt_connected = true;
            if (status_callback) {
                status_callback(true);
            }
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT DESCONECTADO del broker");
            mqtt_connected = false;
            if (status_callback) {
                status_callback(false);
            }
            break;
            
        case MQTT_EVENT_PUBLISHED:
            // CRÍTICO: Confirmar que el mensaje fue publicado (PUBACK recibido)
            ESP_LOGI(TAG, "[MQTT] PUBACK recibido (msg_id=%d, QoS=1)", event->msg_id);
            break;
            
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT ERROR");
            mqtt_connected = false;
            if (status_callback) {
                status_callback(false);
            }
            break;
            
        default:
            break;
    }
}

esp_err_t mqtt_app_start(const char *broker_uri) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = broker_uri,
    };
    
    client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == NULL) {
        ESP_LOGE(TAG, "Error inicializando cliente MQTT");
        return ESP_FAIL;
    }
    
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));
    
    ESP_LOGI(TAG, "Cliente MQTT iniciado con broker: %s", broker_uri);
    return ESP_OK;
}

esp_err_t mqtt_publish_data(const char *topic, const char *data) {
    if (client == NULL) {
        ESP_LOGE(TAG, "Cliente MQTT no inicializado");
        return ESP_FAIL;
    }

    if (!mqtt_connected) {
        ESP_LOGW(TAG, "MQTT no conectado, no se puede publicar");
        return ESP_FAIL;
    }

    // CRÍTICO: QoS=1 para recibir PUBACK
    int msg_id = esp_mqtt_client_publish(client, topic, data, 0, 1, 0);
    //                                                        ↑  ↑  ↑
    //                                                      len QoS retain
    
    if (msg_id >= 0) {
        ESP_LOGI(TAG, "[MQTT] Publicando en %s (msg_id=%d, QoS=1)", topic, msg_id);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Error publicando en %s", topic);
        return ESP_FAIL;
    }
}

bool mqtt_is_connected(void) {
    return mqtt_connected;
}

void mqtt_set_status_callback(mqtt_status_callback_t callback) {
    status_callback = callback;
}