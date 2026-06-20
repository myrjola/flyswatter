#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"

/* ---- Matrix geometry --------------------------------------------------- */
#define MATRIX_WIDTH   22
#define MATRIX_HEIGHT  12
#define MATRIX_PIXELS  (MATRIX_WIDTH * MATRIX_HEIGHT)   /* 264 LEDs */

/* Data pin feeding the matrix' first pixel (DIN).
 * NOTE: GPIO8 is the onboard RGB LED on many ESP32-C3 dev boards; pick a free
 * pin if you wire an external matrix there. */
#define MATRIX_GPIO    GPIO_NUM_10

/* Most WS2812 panels are wired as a boustrophedon (serpentine): row 0 runs
 * left->right, row 1 right->left, and so on. Set to 0 for progressive wiring. */
#define MATRIX_SERPENTINE 1

/* Global brightness cap (HSV "value", 0-255). Keeps current draw sane while
 * testing on USB power -- 264 LEDs at full white is ~15 A. */
#define MATRIX_BRIGHTNESS 40

static const char *TAG = "matrix";

/* Map (x, y) -> strip index, accounting for serpentine wiring. */
static inline uint32_t xy_to_index(int x, int y)
{
#if MATRIX_SERPENTINE
    if (y & 1) {
        x = (MATRIX_WIDTH - 1) - x;
    }
#endif
    return (uint32_t)(y * MATRIX_WIDTH + x);
}

static led_strip_handle_t matrix_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = MATRIX_GPIO,
        .max_leds = MATRIX_PIXELS,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        },
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,   /* 10 MHz, 0.1us per tick */
        .mem_block_symbols = 64,
        .flags = {
            .with_dma = false,               /* ESP32-C3 RMT has no DMA */
        },
    };

    led_strip_handle_t strip = NULL;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &strip));
    ESP_ERROR_CHECK(led_strip_clear(strip));
    ESP_LOGI(TAG, "%dx%d matrix ready on GPIO%d (%d LEDs)",
             MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX_GPIO, MATRIX_PIXELS);
    return strip;
}

void app_main(void)
{
    led_strip_handle_t strip = matrix_init();

    /* Hello, matrix: sweep a diagonal rainbow across the panel so every pixel
     * and the x/y mapping are exercised. */
    uint16_t phase = 0;
    while (1) {
        for (int y = 0; y < MATRIX_HEIGHT; y++) {
            for (int x = 0; x < MATRIX_WIDTH; x++) {
                uint16_t hue = (uint16_t)((x + y) * 12 + phase) % 360;
                ESP_ERROR_CHECK(led_strip_set_pixel_hsv(
                    strip, xy_to_index(x, y), hue, 255, MATRIX_BRIGHTNESS));
            }
        }
        ESP_ERROR_CHECK(led_strip_refresh(strip));

        phase = (phase + 6) % 360;
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}
