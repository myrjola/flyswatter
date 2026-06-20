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
/* LED matrix (animated thunderstorm, runs as its own task)                 */
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

/* Thunderstorm animation: wind-blown rain over a dark sky, lit by random
 * lightning. Tune the rain density, wind lean, and strike frequency here. */
#define RAIN_DROPS       46       /* number of falling rain particles        */
#define RAIN_STREAK      3        /* length of each drop's motion-blur tail  */
#define RAIN_VMIN        0.55f    /* slowest fall speed, pixels/frame        */
#define RAIN_VMAX        1.30f    /* fastest fall speed, pixels/frame        */
#define RAIN_SLANT       0.35f    /* tail lean per step (matches the wind)   */
#define WIND             0.14f    /* horizontal drift, pixels/frame          */
#define LIGHTNING_P      0.012f   /* per-frame chance to start a strike      */
#define FLASH_DECAY      0.55f    /* flash brightness retained each frame    */
#define OUT_CAP          72       /* per-channel ceiling -> bounds current   */

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

/* Cheap xorshift PRNG -- deterministic, no extra component needed. Seeded once
 * from the timer so the storm differs run to run. */
static uint32_t s_rng = 0x1234567u;
static inline uint32_t rnd_u32(void)
{
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return s_rng;
}
static inline float rnd_f(void)   /* uniform 0..1 */
{
    return (float)(rnd_u32() & 0xFFFFFFu) / (float)0x1000000u;
}

/* Linear-light accumulation buffer; the storm composites into this (rain and
 * lightning add over the sky) and then it is flushed to the strip. */
static float g_fb[MATRIX_PIXELS][3];

static inline void fb_add(int x, int y, float r, float g, float b)
{
    if (x < 0 || x >= MATRIX_WIDTH || y < 0 || y >= MATRIX_HEIGHT) return;
    int i = y * MATRIX_WIDTH + x;
    g_fb[i][0] += r; g_fb[i][1] += g; g_fb[i][2] += b;
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

    /* Seed the PRNG from the boot clock so each power-up storms differently. */
    s_rng = (uint32_t)esp_timer_get_time() | 1u;

    /* Rain particles: position and per-drop fall speed. */
    float drop_x[RAIN_DROPS], drop_y[RAIN_DROPS], drop_v[RAIN_DROPS];
    for (int i = 0; i < RAIN_DROPS; i++) {
        drop_x[i] = rnd_f() * MATRIX_WIDTH;
        drop_y[i] = rnd_f() * MATRIX_HEIGHT;
        drop_v[i] = lerpf(RAIN_VMIN, RAIN_VMAX, rnd_f());
    }

    float flash      = 0.0f;          /* whole-sky lightning glow, 1 -> 0     */
    int   bolt_timer = 0;             /* frames the drawn bolt stays visible  */
    int   bolt_x[MATRIX_HEIGHT];      /* jagged column of the current bolt    */
    int   bolt_len = 0;

    while (1) {
        /* --- Lightning: maybe start a strike, or flicker an active one. --- */
        if (flash < 0.03f) {
            if (rnd_f() < LIGHTNING_P) {
                flash      = 1.0f;
                bolt_timer = 2 + (int)(rnd_u32() % 2);
                bolt_len   = 7 + (int)(rnd_u32() % (MATRIX_HEIGHT - 6));
                int bx = 2 + (int)(rnd_u32() % (MATRIX_WIDTH - 4));
                for (int y = 0; y < bolt_len; y++) {
                    bolt_x[y] = bx;
                    bx += (int)(rnd_u32() % 3) - 1;          /* wander -1/0/+1 */
                    if (bx < 0) bx = 0;
                    if (bx >= MATRIX_WIDTH) bx = MATRIX_WIDTH - 1;
                }
            }
        } else if (flash < 0.5f && rnd_f() < 0.30f) {
            flash = 0.9f;                                     /* re-strike flicker */
        }

        /* --- Sky: dark storm gradient, lighter cloud deck up top. --- */
        for (int y = 0; y < MATRIX_HEIGHT; y++) {
            float t = (float)y / (float)(MATRIX_HEIGHT - 1); /* 0 top..1 bottom */
            float r = lerpf(34.0f,  6.0f, t);
            float g = lerpf(40.0f,  9.0f, t);
            float b = lerpf(54.0f, 20.0f, t);
            float fr = r, fg = g, fb = b;
            for (int x = 0; x < MATRIX_WIDTH; x++) {
                int i = y * MATRIX_WIDTH + x;
                g_fb[i][0] = fr; g_fb[i][1] = fg; g_fb[i][2] = fb;
            }
        }

        /* --- Rain: a bright head with a wind-leaned, fading tail. --- */
        for (int i = 0; i < RAIN_DROPS; i++) {
            for (int k = 0; k < RAIN_STREAK; k++) {
                int yy = (int)drop_y[i] - k;
                int xx = (int)(drop_x[i] - k * RAIN_SLANT);
                float w = 1.0f - (float)k / RAIN_STREAK;     /* head brightest */
                fb_add(xx, yy, 90.0f * w, 130.0f * w, 200.0f * w);
            }
        }

        /* --- Lightning composite: whole-sky cool flash + the bright bolt. --- */
        if (flash > 0.0f) {
            float wash = flash * flash;                      /* punchier ramp */
            for (int i = 0; i < MATRIX_PIXELS; i++) {
                g_fb[i][0] += 170.0f * wash;
                g_fb[i][1] += 185.0f * wash;
                g_fb[i][2] += 230.0f * wash;
            }
        }
        if (bolt_timer > 0) {
            for (int y = 0; y < bolt_len; y++) {
                fb_add(bolt_x[y],     y, 255.0f, 255.0f, 255.0f);
                fb_add(bolt_x[y] - 1, y,  70.0f,  80.0f, 120.0f);   /* glow */
                fb_add(bolt_x[y] + 1, y,  70.0f,  80.0f, 120.0f);
            }
        }

        /* --- Flush the buffer to the panel. --- */
        for (int y = 0; y < MATRIX_HEIGHT; y++) {
            for (int x = 0; x < MATRIX_WIDTH; x++) {
                int i = y * MATRIX_WIDTH + x;
                put_px(strip, x, y, g_fb[i][0], g_fb[i][1], g_fb[i][2]);
            }
        }
        ESP_ERROR_CHECK(led_strip_refresh(strip));

        /* --- Advance the simulation for the next frame. --- */
        if (flash > 0.0f) { flash *= FLASH_DECAY; if (flash < 0.02f) flash = 0.0f; }
        if (bolt_timer > 0) bolt_timer--;
        for (int i = 0; i < RAIN_DROPS; i++) {
            drop_y[i] += drop_v[i];
            drop_x[i] += WIND;
            if (drop_x[i] >= MATRIX_WIDTH) drop_x[i] -= MATRIX_WIDTH;
            if (drop_y[i] - RAIN_STREAK > MATRIX_HEIGHT) {   /* fell off bottom */
                drop_y[i] = -(rnd_f() * 3.0f);
                drop_x[i] = rnd_f() * MATRIX_WIDTH;
                drop_v[i] = lerpf(RAIN_VMIN, RAIN_VMAX, rnd_f());
            }
        }

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
