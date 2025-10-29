#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t wifi_init_sta(const char *ssid, const char *password);
bool wifi_is_connected(void);


#ifdef __cplusplus
}
#endif
