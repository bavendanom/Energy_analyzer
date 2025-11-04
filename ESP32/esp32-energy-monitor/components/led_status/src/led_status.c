#include "led_status.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"

static const char *TAG = "led_status";

typedef struct {
    gpio_num_t pin;
    TimerHandle_t timer;
    led_state_t current_state;
    bool timer_running;
} led_control_t;

static led_control_t led_controls[3] = {0};
static int led_count = 0;

static void blink_timer_callback(TimerHandle_t xTimer) {
    led_control_t *led = (led_control_t *)pvTimerGetTimerID(xTimer);
    
    if (led->current_state == LED_BLINK) {
        // Toggle LED
        int current_level = gpio_get_level(led->pin);
        gpio_set_level(led->pin, !current_level);
    }
}

void led_status_init(gpio_num_t pin) {
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
    
    if (led_count < 3) {
        led_controls[led_count].pin = pin;
        led_controls[led_count].current_state = LED_OFF;
        led_controls[led_count].timer_running = false;
        
        // Timer de 400ms (200ms ON, 200ms OFF) para mejor visibilidad
        led_controls[led_count].timer = xTimerCreate(
            "led_blink",
            pdMS_TO_TICKS(400),
            pdTRUE,  // Auto-reload
            &led_controls[led_count],
            blink_timer_callback
        );
        
        if (led_controls[led_count].timer == NULL) {
            ESP_LOGE(TAG, "Error creando timer para GPIO %d", pin);
        }
        
        led_count++;
        ESP_LOGD(TAG, "LED inicializado en GPIO %d", pin);
    }
}

void led_status_set(gpio_num_t pin, led_state_t state) {
    // Buscar el control del LED
    led_control_t *led = NULL;
    for (int i = 0; i < led_count; i++) {
        if (led_controls[i].pin == pin) {
            led = &led_controls[i];
            break;
        }
    }
    
    if (led == NULL) {
        ESP_LOGW(TAG, "LED no encontrado en GPIO %d", pin);
        return;
    }

    // Si ya está en el estado solicitado, no hacer nada
    if (led->current_state == state) {
        return;
    }
    
    ESP_LOGD(TAG, "GPIO %d: cambiando de estado %d a %d", pin, led->current_state, state);
    
    // Detener timer si estaba corriendo
    if (led->timer_running && led->timer != NULL) {
        xTimerStop(led->timer, 0);
        led->timer_running = false;
    }
    
    // Actualizar estado
    led->current_state = state;
    
    switch (state) {
        case LED_ON:
            gpio_set_level(pin, 1);
            ESP_LOGD(TAG, "GPIO %d: LED encendido", pin);
            break;
            
        case LED_OFF:
            gpio_set_level(pin, 0);
            ESP_LOGD(TAG, "GPIO %d: LED apagado", pin);
            break;
            
        case LED_BLINK:
            if (led->timer != NULL) {
                gpio_set_level(pin, 1);  // Iniciar encendido
                BaseType_t result = xTimerStart(led->timer, 0);
                if (result == pdPASS) {
                    led->timer_running = true;
                    ESP_LOGD(TAG, "GPIO %d: Blink iniciado", pin);
                } else {
                    ESP_LOGE(TAG, "GPIO %d: Error iniciando timer", pin);
                }
            }
            break;
    }
}

led_state_t led_status_get(gpio_num_t pin) {
    for (int i = 0; i < led_count; i++) {
        if (led_controls[i].pin == pin) {
            return led_controls[i].current_state;
        }
    }
    return LED_OFF;
}

void led_status_flash(gpio_num_t pin, int flashes, int delay_ms) {
    for (int i = 0; i < flashes; i++) {
        gpio_set_level(pin, 1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        gpio_set_level(pin, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}