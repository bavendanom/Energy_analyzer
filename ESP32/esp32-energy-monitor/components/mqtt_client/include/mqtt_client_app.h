#ifndef MQTT_CLIENT_APP_H
#define MQTT_CLIENT_APP_H

#include "esp_err.h"
#include <stdbool.h>

typedef void (*mqtt_status_callback_t)(bool connected);

esp_err_t mqtt_app_start(const char *broker_uri);
esp_err_t mqtt_publish_data(const char *topic, const char *data);
bool mqtt_is_connected(void);
void mqtt_set_status_callback(mqtt_status_callback_t callback);

#endif