#include "ntp_sync.h"
#include "esp_log.h"
#include "esp_sntp.h"

static const char *TAG = "ntp_sync";

esp_err_t ntp_sync_time(const char *ntp_server) {
    ESP_LOGI(TAG, "Iniciando sincronización NTP con %s", ntp_server);
    
    // Usar funciones nuevas de ESP-IDF v5.x
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, ntp_server);
    esp_sntp_init();

    // Esperar hasta 10 segundos para sincronizar
    int retry = 0;
    const int max_retries = 10;
    
    while (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < max_retries) {
        ESP_LOGI(TAG, "Esperando sincronización NTP... (%d/%d)", retry, max_retries);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (retry >= max_retries) {
        ESP_LOGE(TAG, "Timeout: No se pudo sincronizar con NTP");
        return ESP_FAIL;
    }

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    // Validar que la hora sea razonable (después del 1 de enero de 2020)
    if (now < 1577836800) {
        ESP_LOGE(TAG, "Hora NTP inválida: %ld", (long)now);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "✅ Hora sincronizada: %s", asctime(&timeinfo));
    return ESP_OK;
}