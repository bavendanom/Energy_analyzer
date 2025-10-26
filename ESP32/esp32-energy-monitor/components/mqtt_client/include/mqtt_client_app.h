#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t mqtt_app_start(const char *broker_uri);
esp_err_t mqtt_publish_data(const char *topic, const char *payload);

#ifdef __cplusplus
}
#endif
