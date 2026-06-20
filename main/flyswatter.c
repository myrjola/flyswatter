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
/* LED matrix (sunrise -> day -> sunset -> starry night, as its own task)   */
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

/* A full day/night cycle: dawn -> day -> a long sunset -> dusk -> a starry
 * night, then it loops seamlessly back through dawn. Stretch SKY_CYCLE_S to
 * slow the whole thing down. */
#define SKY_CYCLE_S      64.0f    /* seconds for one full day/night loop      */
#define SUN_R            3.2f     /* sun disc radius, in pixels               */
#define SUN_GLOW         5.5f     /* halo falloff distance beyond the disc    */
#define NSTARS           26       /* twinkling stars in the night sky         */
#define OUT_CAP          56       /* per-channel ceiling -> bounds current    */

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

/* xorshift PRNG for one-time star placement; seeded from the boot clock. */
static uint32_t s_rng = 0x2545F491u;
static inline uint32_t rnd_u32(void)
{
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return s_rng;
}
static inline float rnd_f(void) { return (float)(rnd_u32() & 0xFFFFFFu) / (float)0x1000000u; }

/* Linear-light accumulation buffer: the sky fills it, then the sun and the
 * stars add their light over the top before it is flushed to the strip. */
static float g_fb[MATRIX_PIXELS][3];
static inline void fb_add(int x, int y, float r, float g, float b)
{
    if (x < 0 || x >= MATRIX_WIDTH || y < 0 || y >= MATRIX_HEIGHT) return;
    int i = y * MATRIX_WIDTH + x;
    g_fb[i][0] += r; g_fb[i][1] += g; g_fb[i][2] += b;
}

/* Timeline keyframes, all on one u-grid (u in [0,1) is the time of day). Node
 * order: night, dawn, day, golden, sunset, dusk, night -- so u=1 == u=0. */
#define NKEY 7
static const float KEY_U[NKEY]      = { 0.00f, 0.12f, 0.30f, 0.45f, 0.55f, 0.68f, 1.00f };
static const float KEY_TOP[NKEY][3] = {   /* zenith (top row) colour */
    {  0.0f,   1.0f,   6.0f },   /* night  */
    { 45.0f,  55.0f, 120.0f },   /* dawn   */
    { 60.0f, 115.0f, 195.0f },   /* day    */
    { 50.0f,  50.0f, 120.0f },   /* golden */
    { 80.0f,  35.0f,  95.0f },   /* sunset */
    {  4.0f,   5.0f,  20.0f },   /* dusk   */
    {  0.0f,   1.0f,   6.0f },   /* night  */
};
static const float KEY_HOR[NKEY][3] = {   /* horizon (bottom row) colour */
    {  1.0f,   2.0f,   9.0f },
    {255.0f, 160.0f,  80.0f },
    {150.0f, 180.0f, 225.0f },
    {255.0f, 170.0f,  70.0f },
    {255.0f,  80.0f,  30.0f },
    { 24.0f,  15.0f,  34.0f },
    {  1.0f,   2.0f,   9.0f },
};
static const float KEY_SUNX[NKEY]   = { 2.0f,  4.0f, 11.0f, 15.0f, 18.0f, 20.0f, 21.0f };
static const float KEY_SUNY[NKEY]   = { 15.0f, 8.0f,  2.5f,  6.0f, 10.5f, 15.0f, 16.0f };

static float key_scalar(float u, const float v[])
{
    if (u <= KEY_U[0])      return v[0];
    if (u >= KEY_U[NKEY-1]) return v[NKEY-1];
    for (int i = 0; i < NKEY - 1; i++)
        if (u < KEY_U[i+1]) {
            float k = (u - KEY_U[i]) / (KEY_U[i+1] - KEY_U[i]);
            return lerpf(v[i], v[i+1], k);
        }
    return v[NKEY-1];
}
static void key_vec3(float u, const float c[][3], float out[3])
{
    if (u <= KEY_U[0])      { out[0]=c[0][0]; out[1]=c[0][1]; out[2]=c[0][2]; return; }
    if (u >= KEY_U[NKEY-1]) { int n=NKEY-1; out[0]=c[n][0]; out[1]=c[n][1]; out[2]=c[n][2]; return; }
    for (int i = 0; i < NKEY - 1; i++)
        if (u < KEY_U[i+1]) {
            float k = (u - KEY_U[i]) / (KEY_U[i+1] - KEY_U[i]);
            for (int j = 0; j < 3; j++) out[j] = lerpf(c[i][j], c[i+1][j], k);
            return;
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

    /* Scatter the stars once across the upper sky (the bottom rows stay clear
     * for the horizon glow). They persist for the life of the task. */
    s_rng = (uint32_t)esp_timer_get_time() | 1u;
    int   star_x[NSTARS], star_y[NSTARS];
    float star_ph[NSTARS], star_sp[NSTARS], star_w[NSTARS];
    for (int i = 0; i < NSTARS; i++) {
        star_x[i]  = (int)(rnd_f() * MATRIX_WIDTH);
        star_y[i]  = (int)(rnd_f() * (MATRIX_HEIGHT - 3));
        star_ph[i] = rnd_f() * 6.2832f;             /* twinkle phase   */
        star_sp[i] = lerpf(1.5f, 4.0f, rnd_f());    /* twinkle rate    */
        star_w[i]  = lerpf(0.5f, 1.0f, rnd_f());    /* base brightness */
    }

    while (1) {
        float ts = (float)esp_timer_get_time() / 1e6f;
        float u  = fmodf(ts / SKY_CYCLE_S, 1.0f);   /* time of day, 0..1 */

        float top[3], hor[3];
        key_vec3(u, KEY_TOP, top);
        key_vec3(u, KEY_HOR, hor);
        float sun_cx = key_scalar(u, KEY_SUNX);
        float sun_cy = key_scalar(u, KEY_SUNY);

        /* The sun reddens as it nears the horizon and fades out below it. */
        float redden = clampf((sun_cy - 2.5f) / 8.0f, 0.0f, 1.0f);
        float sun_up = clampf((13.5f - sun_cy) / 4.0f, 0.0f, 1.0f);

        /* Stars fade in once the zenith goes dark (low top-of-sky luminance). */
        float tmax = top[0] > top[1] ? (top[0] > top[2] ? top[0] : top[2])
                                     : (top[1] > top[2] ? top[1] : top[2]);
        float star_vis = clampf((46.0f - tmax) / 34.0f, 0.0f, 1.0f);

        /* Sky gradient (top->horizon) plus the sun, composited into the buffer. */
        for (int y = 0; y < MATRIX_HEIGHT; y++) {
            float t  = (float)y / (float)(MATRIX_HEIGHT - 1);
            float sr = lerpf(top[0], hor[0], t);
            float sg = lerpf(top[1], hor[1], t);
            float sb = lerpf(top[2], hor[2], t);
            for (int x = 0; x < MATRIX_WIDTH; x++) {
                float r = sr, g = sg, b = sb;
                float dx = (float)x - sun_cx;
                float dy = (float)y - sun_cy;
                float d  = sqrtf(dx * dx + dy * dy);

                if (d <= SUN_R) {
                    float f   = d / SUN_R;                    /* 0 core..1 rim */
                    float cr  = 255.0f;
                    float cg  = lerpf(238.0f, 150.0f, f) - redden * 80.0f;
                    float cb  = lerpf(190.0f,  40.0f, f) - redden * 30.0f;
                    float hot = (2.0f - f) * sun_up;          /* dim as it sets */
                    r = cr * hot;
                    g = clampf(cg, 0.0f, 255.0f) * hot;
                    b = clampf(cb, 0.0f, 255.0f) * hot;
                } else {
                    /* Warm halo, redder at sunset, gone once the sun is down. */
                    float glow = clampf(1.0f - (d - SUN_R) / SUN_GLOW, 0.0f, 1.0f);
                    glow = glow * glow * sun_up;
                    r += 255.0f                       * glow * 0.9f;
                    g += lerpf(170.0f, 110.0f, redden) * glow * 0.9f;
                    b += lerpf( 90.0f,  35.0f, redden) * glow * 0.7f;
                }

                int i = y * MATRIX_WIDTH + x;
                g_fb[i][0] = r; g_fb[i][1] = g; g_fb[i][2] = b;
            }
        }

        /* Twinkling stars on top, brightening as night deepens. */
        if (star_vis > 0.0f) {
            for (int i = 0; i < NSTARS; i++) {
                float tw = 0.45f + 0.55f * sinf(star_ph[i] + ts * star_sp[i]);
                if (tw < 0.0f) tw = 0.0f;
                float a = star_vis * star_w[i] * tw;
                fb_add(star_x[i], star_y[i], 230.0f * a, 240.0f * a, 255.0f * a);
            }
        }

        /* Flush the composed frame to the panel. */
        for (int y = 0; y < MATRIX_HEIGHT; y++) {
            for (int x = 0; x < MATRIX_WIDTH; x++) {
                int i = y * MATRIX_WIDTH + x;
                put_px(strip, x, y, g_fb[i][0], g_fb[i][1], g_fb[i][2]);
            }
        }
        ESP_ERROR_CHECK(led_strip_refresh(strip));
        vTaskDelay(pdMS_TO_TICKS(40));   /* ~25 fps */
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
