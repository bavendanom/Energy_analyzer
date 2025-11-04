#ifndef MANEJO_WIFI_H
#define MANEJO_WIFI_H

#include "esp_err.h"
#include <stdbool.h>

// Tipo de callback para notificar cambios de estado WiFi
typedef void (*wifi_status_callback_t)(bool connected);

esp_err_t wifi_init_sta(const char *ssid, const char *password);
bool wifi_is_connected(void);
void wifi_set_status_callback(wifi_status_callback_t callback);

#endif