#include "mqtt_client_app.h"
#include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG = "mqtt_client_app";
static esp_mqtt_client_handle_t client;

esp_err_t mqtt_app_start(const char *broker_uri) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = broker_uri,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));
    ESP_LOGI(TAG, "Cliente MQTT iniciado con broker: %s", broker_uri);
    return ESP_OK;
}

esp_err_t mqtt_publish_data(const char *topic, const char *data) {
    if (client == NULL) {
        ESP_LOGE("mqtt_client_app", "Cliente MQTT no inicializado.");
        return ESP_FAIL;
    }

    int msg_id = esp_mqtt_client_publish(client, topic, data, 0, 1, 0);
    if (msg_id >= 0) {
        ESP_LOGI("mqtt_client_app", "Publicado en %s (msg_id=%d): %s", topic, msg_id, data);
        return ESP_OK;
    } else {
        ESP_LOGE("mqtt_client_app", "Error publicando en %s", topic);
        return ESP_FAIL;
    }
}

