#ifndef LED_UTIL_H
#define LED_UTIL_H

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Initialize the onboard RGB LED strip
 * @return ESP_OK on success
 */
esp_err_t led_util_init(void);

/**
 * @brief Set the RGB LED color
 * @param red Red component (0-255)
 * @param green Green component (0-255)
 * @param blue Blue component (0-255)
 * @return ESP_OK on success
 */
esp_err_t led_util_set_color(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Turn the LED off
 * @return ESP_OK on success
 */
esp_err_t led_util_clear(void);

/**
 * @brief Toggle the LED between a color and off
 * @param red Red component
 * @param green Green component
 * @param blue Blue component
 * @return ESP_OK on success
 */
esp_err_t led_util_toggle(uint8_t red, uint8_t green, uint8_t blue);

#endif // LED_UTIL_H
