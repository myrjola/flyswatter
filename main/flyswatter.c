#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "led_strip.h"

/* ======================================================================== */
/* LED matrix (hello-world rainbow, runs as its own task)                   */
/* ======================================================================== */
#define ENABLE_MATRIX   1      /* set 0 to silence the panel while bench-testing */

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
#define MATRIX_BRIGHTNESS 20

/* ======================================================================== */
/* KY-039 heartbeat sensor -> onboard LED                                   */
/* ======================================================================== */
/* Wiring (3 cables):  S -> GPIO0 (ADC1_CH0),  + -> 3V3,  - -> GND          */
#define HB_ADC_UNIT     ADC_UNIT_1
#define HB_ADC_CHANNEL  ADC_CHANNEL_0     /* GPIO0 on the ESP32-C3 */

/* Onboard LED: plain single-colour LED on GPIO8, lit when driven LOW. */
#define LED_GPIO        GPIO_NUM_8
#define LED_ACTIVE_LOW  1

#define HB_SAMPLE_MS    20                /* 50 Hz sampling */
#define HB_FLASH_MS     60                /* how long the LED stays lit per beat */

/* Detector tuning (all in raw 12-bit ADC counts unless noted). */
#define HB_DC_ALPHA        0.01f   /* baseline EMA: ~2 s time constant @ 50 Hz */
#define HB_AC_ALPHA        0.30f   /* light smoothing of the AC component */
#define HB_ENV_DECAY       0.96f   /* peak-envelope decay per sample */
#define HB_THRESH_FRAC     0.50f   /* beat when AC rises past this fraction of envelope */
#define HB_MIN_AMPLITUDE   25.0f   /* noise floor: ignore wiggles smaller than this */
#define HB_REFRACTORY_US   300000  /* min 300 ms between beats (=> max 200 bpm) */
#define HB_INVERT          0       /* flip to 1 if beats point downward on your module */

/* Set to 1 to stream "raw,ac,thr" each sample for the Arduino-style serial
 * plotter -- handy for picking HB_MIN_AMPLITUDE and finger pressure. */
#define HB_PLOT            1

static const char *TAG = "flyswatter";

/* ------------------------------------------------------------------------ */
/* Matrix                                                                    */
/* ------------------------------------------------------------------------ */
static inline uint32_t xy_to_index(int x, int y)
{
#if MATRIX_SERPENTINE
    if (y & 1) {
        x = (MATRIX_WIDTH - 1) - x;
    }
#endif
    return (uint32_t)(y * MATRIX_WIDTH + x);
}

static void matrix_task(void *arg)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = MATRIX_GPIO,
        .max_leds = MATRIX_PIXELS,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = { .invert_out = false },
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,   /* 10 MHz, 0.1us per tick */
        .mem_block_symbols = 64,
        .flags = { .with_dma = false },       /* ESP32-C3 RMT has no DMA */
    };

    led_strip_handle_t strip = NULL;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &strip));
    ESP_ERROR_CHECK(led_strip_clear(strip));
    ESP_LOGI(TAG, "%dx%d matrix ready on GPIO%d (%d LEDs)",
             MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX_GPIO, MATRIX_PIXELS);

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

/* ------------------------------------------------------------------------ */
/* Heartbeat                                                                 */
/* ------------------------------------------------------------------------ */
static inline void led_set(bool on)
{
#if LED_ACTIVE_LOW
    gpio_set_level(LED_GPIO, on ? 0 : 1);
#else
    gpio_set_level(LED_GPIO, on ? 1 : 0);
#endif
}

void app_main(void)
{
    /* Onboard LED as output, start off. */
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    led_set(false);

    /* ADC1 one-shot on the sensor pin. 12 dB attenuation => full ~0-3.1 V range.
     * We work in raw counts with relative thresholds, so no calibration needed. */
    adc_oneshot_unit_handle_t adc = NULL;
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = HB_ADC_UNIT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, HB_ADC_CHANNEL, &chan_cfg));

#if ENABLE_MATRIX
    xTaskCreate(matrix_task, "matrix", 4096, NULL, 5, NULL);
#endif

    ESP_LOGI(TAG, "heartbeat: sampling ADC1_CH%d, blinking LED on GPIO%d",
             HB_ADC_CHANNEL, LED_GPIO);

    /* Detector state. Seed the baseline with the first reading. */
    int seed = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc, HB_ADC_CHANNEL, &seed));
    float dc = (float)seed;     /* slow baseline (DC offset) */
    float acf = 0.0f;           /* smoothed AC component */
    float env = HB_MIN_AMPLITUDE; /* peak envelope */
    bool was_above = false;
    int64_t last_beat_us = 0;
    int64_t led_off_at_us = 0;

    while (1) {
        int raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc, HB_ADC_CHANNEL, &raw));
        int64_t now = esp_timer_get_time();

        /* Isolate the pulsatile AC component. */
        dc += HB_DC_ALPHA * ((float)raw - dc);
        float ac = (float)raw - dc;
#if HB_INVERT
        ac = -ac;
#endif
        acf += HB_AC_ALPHA * (ac - acf);

        /* Dynamic threshold from a decaying peak envelope. */
        float thr = env * HB_THRESH_FRAC;
        bool above = (acf > thr) && (acf > HB_MIN_AMPLITUDE);

        if (above && !was_above && (now - last_beat_us) > HB_REFRACTORY_US) {
            int bpm = last_beat_us ? (int)(60000000LL / (now - last_beat_us)) : 0;
            last_beat_us = now;
            led_off_at_us = now + HB_FLASH_MS * 1000;
            if (bpm) {
                ESP_LOGI(TAG, "beat  ~%d bpm", bpm);
            } else {
                ESP_LOGI(TAG, "beat  (first)");
            }
        }
        was_above = above;

        /* Update envelope (peak follower with decay). */
        env *= HB_ENV_DECAY;
        if (acf > env) env = acf;
        if (env < HB_MIN_AMPLITUDE) env = HB_MIN_AMPLITUDE;

        led_set(now < led_off_at_us);

#if HB_PLOT
        printf("%d,%d,%d\n", raw, (int)acf, (int)thr);
#endif
        vTaskDelay(pdMS_TO_TICKS(HB_SAMPLE_MS));
    }
}
