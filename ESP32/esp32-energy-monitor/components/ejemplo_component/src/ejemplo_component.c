#include "ejemplo_component.h"
#include "esp_log.h"
#include "sdkconfig.h"


static const char *TAG = "ejemplo_component";

void ejemplo_component_saludo(void) {
    ESP_LOGI(TAG, "Ejemplo de componente compilado correctamente.");
}
