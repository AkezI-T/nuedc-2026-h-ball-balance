#ifndef BSP_FILL_LIGHT_H
#define BSP_FILL_LIGHT_H

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

esp_err_t bsp_fill_light_init(gpio_num_t gpio_num, size_t led_count);
esp_err_t bsp_fill_light_set_rgb(uint8_t red, uint8_t green, uint8_t blue);
esp_err_t bsp_fill_light_off(void);

#endif
