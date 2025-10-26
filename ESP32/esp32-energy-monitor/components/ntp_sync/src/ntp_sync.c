#include "ntp_sync.h"
#include "esp_log.h"
#include "esp_sntp.h"

static const char *TAG = "ntp_sync";

esp_err_t ntp_sync_time(const char *ntp_server) {
    ESP_LOGI(TAG, "Iniciando sincronización NTP con %s", ntp_server);
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, ntp_server);
    sntp_init();

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    ESP_LOGI(TAG, "Hora sincronizada: %s", asctime(&timeinfo));
    return ESP_OK;
}
