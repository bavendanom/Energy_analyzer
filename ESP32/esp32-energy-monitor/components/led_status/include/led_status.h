#ifndef LED_STATUS_H
#define LED_STATUS_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef enum {
    LED_OFF = 0,
    LED_ON,
    LED_BLINK
} led_state_t;

/**
 * @brief Inicializa un pin GPIO como LED de estado.
 */
void led_status_init(gpio_num_t pin);

/**
 * @brief Cambia el estado de un LED.
 */
void led_status_set(gpio_num_t pin, led_state_t state);

/**
 * @brief Parpadea un LED rápidamente (ej. indicación de actividad).
 */
void led_status_flash(gpio_num_t pin, int flashes, int delay_ms);

#endif
