#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "led_strip.h"

/* ======================================================================== */
/* LED matrix (animated sunrise, runs as its own task)                      */
/* ======================================================================== */
#define ENABLE_MATRIX   1      /* set 0 to silence the panel while bench-testing */

#define MATRIX_WIDTH   22
#define MATRIX_HEIGHT  12
#define MATRIX_PIXELS  (MATRIX_WIDTH * MATRIX_HEIGHT)   /* 264 LEDs */

/* Data pin feeding the matrix' first pixel (DIN). */
#define MATRIX_GPIO    GPIO_NUM_1

/* Most WS2812 panels are wired as a boustrophedon (serpentine): row 0 runs
 * left->right, row 1 right->left, and so on. Set to 0 for progressive wiring. */
#define MATRIX_SERPENTINE 0

/* Global brightness cap (HSV "value", 0-255). Keeps current draw sane while
 * testing on USB power -- 264 LEDs at full white is ~15 A. */
#define MATRIX_BRIGHTNESS 20

/* Sunrise animation. The sun rises and sets on a slow triangle so the loop is
 * seamless; tweak SUN_CYCLE_S for speed and SUN_R for the disc size. */
#define SUN_CYCLE_S      16.0f    /* seconds for a full rise+set sweep       */
#define SUN_R            3.4f     /* sun disc radius, in pixels              */
#define SUN_GLOW         5.0f     /* halo falloff distance beyond the disc   */
#define SKY_MID          0.55f    /* height (0=top,1=bottom) of the mid stop */
#define OUT_CAP          56       /* per-channel ceiling -> bounds current   */

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

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float lerpf(float a, float b, float t)
{
    return a + (b - a) * t;
}

/* Vertical sky gradient, full-intensity (0..255). t: 0 at the top of the panel,
 * 1 at the horizon. Indigo zenith -> magenta mid-sky -> warm orange horizon. */
static void sky_color(float t, float *r, float *g, float *b)
{
    if (t < SKY_MID) {
        float k = t / SKY_MID;
        *r = lerpf(18.0f,  120.0f, k);   /* zenith -> mid */
        *g = lerpf(14.0f,   38.0f, k);
        *b = lerpf(64.0f,   96.0f, k);
    } else {
        float k = (t - SKY_MID) / (1.0f - SKY_MID);
        *r = lerpf(120.0f, 255.0f, k);   /* mid -> horizon */
        *g = lerpf( 38.0f, 120.0f, k);
        *b = lerpf( 96.0f,  42.0f, k);
    }
}

/* Scale a full-intensity RGB triple down to the panel's brightness budget and
 * push it to the strip, clamping per channel to keep total current in check. */
static inline void put_px(led_strip_handle_t s, int x, int y, float r, float g, float b)
{
    const float k = (float)MATRIX_BRIGHTNESS / 255.0f;
    int R = (int)clampf(r * k, 0.0f, (float)OUT_CAP);
    int G = (int)clampf(g * k, 0.0f, (float)OUT_CAP);
    int B = (int)clampf(b * k, 0.0f, (float)OUT_CAP);
    led_strip_set_pixel(s, xy_to_index(x, y), R, G, B);
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

    const float sun_cx = (MATRIX_WIDTH - 1) / 2.0f;   /* horizontal centre */

    while (1) {
        /* Triangle phase p: 0 (sun below horizon) -> 1 (high) -> 0, seamless. */
        float ts = (float)esp_timer_get_time() / 1e6f;
        float pr = fmodf(ts / SUN_CYCLE_S, 1.0f);
        float p  = (pr < 0.5f) ? (pr * 2.0f) : ((1.0f - pr) * 2.0f);

        /* Pre-dawn is dim; the whole sky brightens as the sun climbs. */
        float day = 0.22f + 0.78f * p;
        /* Sun centre travels from just under the bottom edge up near the top. */
        float sun_cy = lerpf((float)MATRIX_HEIGHT + 1.5f, 2.0f, p);

        for (int y = 0; y < MATRIX_HEIGHT; y++) {
            float th = (float)y / (float)(MATRIX_HEIGHT - 1);   /* 0 top..1 horizon */
            float base_r, base_g, base_b;
            sky_color(th, &base_r, &base_g, &base_b);
            base_r *= day; base_g *= day; base_b *= day;

            for (int x = 0; x < MATRIX_WIDTH; x++) {
                float dx = (float)x - sun_cx;
                float dy = (float)y - sun_cy;
                float d  = sqrtf(dx * dx + dy * dy);

                float r = base_r, g = base_g, b = base_b;

                if (d <= SUN_R) {
                    /* Disc: hot white-gold core fading to orange at the rim,
                     * and redder overall while the sun is low on the horizon. */
                    float f      = d / SUN_R;                 /* 0 core..1 rim */
                    float redden = clampf(1.0f - p, 0.0f, 1.0f);
                    float cr = 255.0f;
                    float cg = lerpf(238.0f, 150.0f, f) - redden * 70.0f;
                    float cb = lerpf(190.0f,  40.0f, f) - redden * 20.0f;
                    float hot = 2.0f - f;                     /* brighten centre */
                    r = cr * hot;
                    g = clampf(cg, 0.0f, 255.0f) * hot;
                    b = clampf(cb, 0.0f, 255.0f) * hot;
                } else {
                    /* Warm halo bleeding into the sky around the disc. */
                    float glow = clampf(1.0f - (d - SUN_R) / SUN_GLOW, 0.0f, 1.0f);
                    glow *= glow;
                    r += 255.0f * glow * 0.9f;
                    g += 130.0f * glow * 0.9f;
                    b +=  40.0f * glow * 0.7f;
                }

                put_px(strip, x, y, r, g, b);
            }
        }

        ESP_ERROR_CHECK(led_strip_refresh(strip));
        vTaskDelay(pdMS_TO_TICKS(33));   /* ~30 fps */
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
