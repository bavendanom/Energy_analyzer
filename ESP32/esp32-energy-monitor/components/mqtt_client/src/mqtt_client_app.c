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

esp_err_t mqtt_publish_data(const char *topic, const char *payload) {
    if (!client) return ESP_FAIL;
    int msg_id = esp_mqtt_client_publish(client, topic, payload, 0, 1, 0);
    ESP_LOGI(TAG, "Publicado en %s (msg_id=%d)", topic, msg_id);
    return ESP_OK;
}
