#include "led_util.h"
#include "led_strip.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "led_util";
static led_strip_handle_t led_strip = NULL;
static bool led_on = false;

esp_err_t led_util_init(void) {
    if (led_strip != NULL) {
        return ESP_OK;
    }

    /* LED strip initialization with the RMT backend */
    led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_BLINK_GPIO,
        .max_leds = 1,
    };

#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    esp_err_t err = led_strip_new_spi_device(&strip_config, &spi_config, &led_strip);
#else
    ESP_LOGE(TAG, "No LED strip backend configured");
    return ESP_FAIL;
#endif

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LED strip");
        return err;
    }

    led_strip_clear(led_strip);
    return ESP_OK;
}

esp_err_t led_util_set_color(uint8_t red, uint8_t green, uint8_t blue) {
    if (led_strip == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = led_strip_set_pixel(led_strip, 0, red, green, blue);
    if (err == ESP_OK) {
        err = led_strip_refresh(led_strip);
    }
    if (err == ESP_OK) {
        led_on = (red > 0 || green > 0 || blue > 0);
    }
    return err;
}

esp_err_t led_util_clear(void) {
    if (led_strip == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = led_strip_clear(led_strip);
    if (err == ESP_OK) {
        led_on = false;
    }
    return err;
}

esp_err_t led_util_toggle(uint8_t red, uint8_t green, uint8_t blue) {
    if (led_on) {
        return led_util_clear();
    } else {
        return led_util_set_color(red, green, blue);
    }
}
