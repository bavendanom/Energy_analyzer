#include "led_status.h"

void led_status_init(gpio_num_t pin) {
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
}

void led_status_set(gpio_num_t pin, led_state_t state) {
    switch (state) {
        case LED_ON:
            gpio_set_level(pin, 1);
            break;
        case LED_OFF:
            gpio_set_level(pin, 0);
            break;
        case LED_BLINK:
            gpio_set_level(pin, 1);
            vTaskDelay(pdMS_TO_TICKS(150));
            gpio_set_level(pin, 0);
            break;
    }
}

void led_status_flash(gpio_num_t pin, int flashes, int delay_ms) {
    for (int i = 0; i < flashes; i++) {
        gpio_set_level(pin, 1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        gpio_set_level(pin, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}
