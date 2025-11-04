#ifndef LED_STATUS_H
#define LED_STATUS_H

#include "driver/gpio.h"

typedef enum {
    LED_OFF,
    LED_ON,
    LED_BLINK
} led_state_t;

void led_status_init(gpio_num_t pin);
void led_status_set(gpio_num_t pin, led_state_t state);
void led_status_flash(gpio_num_t pin, int flashes, int delay_ms);

#endif