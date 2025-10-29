#pragma once
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t mqtt_app_start(const char *broker_uri);
esp_err_t mqtt_publish_data(const char *topic, const char *payload);
bool mqtt_is_ready(void);

#ifdef __cplusplus
}
#endif
