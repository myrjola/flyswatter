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
/* LED matrix (hello-world rainbow, runs as its own task)                   */
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

/* Layout: a throbbing heart up top, the BPM readout in the bottom rows. */
#define HEART_ROWS       7        /* top rows reserved for the heart        */
#define DIGIT_ROWS       5        /* bottom rows for the 3x5 BPM glyphs      */
#define HEART_CX         10.5f    /* horizontal centre (width 22 => 0..21)  */
#define HEART_CY         3.0f     /* row where the heart's math y == 0       */
#define HEART_BASE_SX    4.2f     /* horizontal scale at rest                */
#define HEART_BASE_SY    2.1f     /* vertical scale at rest                  */
#define HEART_PULSE_AMP  0.30f    /* how much the heart grows on a beat      */
#define HEART_PULSE_TAU  0.16f    /* throb decay time constant (seconds)     */
#define HEART_FLARE      40       /* extra HSV value added at the beat peak  */

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

/* Published by the heartbeat detector (app_main), consumed by matrix_task.
 * Plain 32-bit/64-bit loads and stores on the C3 are good enough here -- a
 * one-frame tear on the timestamp would be invisible. */
static volatile int     g_bpm          = 0;   /* smoothed beats per minute   */
static volatile int64_t g_last_beat_us = 0;   /* esp_timer time of last beat */

/* 3x5 pixel font for digits 0-9; bit 2 (0x4) is the leftmost column. */
static const uint8_t FONT3x5[10][5] = {
    {0x7, 0x5, 0x5, 0x5, 0x7},  /* 0 */
    {0x2, 0x6, 0x2, 0x2, 0x7},  /* 1 */
    {0x7, 0x1, 0x7, 0x4, 0x7},  /* 2 */
    {0x7, 0x1, 0x7, 0x1, 0x7},  /* 3 */
    {0x5, 0x5, 0x7, 0x1, 0x1},  /* 4 */
    {0x7, 0x4, 0x7, 0x1, 0x7},  /* 5 */
    {0x7, 0x4, 0x7, 0x5, 0x7},  /* 6 */
    {0x7, 0x1, 0x2, 0x2, 0x2},  /* 7 */
    {0x7, 0x5, 0x7, 0x5, 0x7},  /* 8 */
    {0x7, 0x5, 0x7, 0x1, 0x7},  /* 9 */
};

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

/* Paint the BPM readout (or "--" before the first beat) centred in the bottom
 * DIGIT_ROWS rows, using a soft pink so it reads apart from the red heart. */
static void draw_bpm(led_strip_handle_t strip, int bpm)
{
    char buf[4];
    int  n;
    if (bpm > 0) {
        if (bpm > 999) bpm = 999;
        n = snprintf(buf, sizeof buf, "%d", bpm);
    } else {
        buf[0] = '-'; buf[1] = '-'; buf[2] = '\0'; n = 2;
    }

    int width = n * 3 + (n - 1);                 /* 3px glyphs + 1px gaps */
    int ox    = (MATRIX_WIDTH - width) / 2;
    int oy    = MATRIX_HEIGHT - DIGIT_ROWS;      /* bottom DIGIT_ROWS rows */

    for (int i = 0; i < n; i++) {
        char c = buf[i];
        for (int row = 0; row < 5; row++) {
            uint8_t bits = (c >= '0' && c <= '9') ? FONT3x5[c - '0'][row]
                                                  : (row == 2 ? 0x7 : 0x0); /* '-' */
            for (int col = 0; col < 3; col++) {
                if (bits & (0x4 >> col)) {
                    led_strip_set_pixel_hsv(strip,
                        xy_to_index(ox + i * 4 + col, oy + row),
                        330, 200, MATRIX_BRIGHTNESS);
                }
            }
        }
    }
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

    while (1) {
        /* Throb envelope: 1.0 at the instant of a beat, decaying to 0. */
        int64_t now   = esp_timer_get_time();
        float   dt    = (float)(now - g_last_beat_us) / 1e6f;   /* s since beat */
        float   pulse = expf(-dt / HEART_PULSE_TAU);
        float   scale = 1.0f + HEART_PULSE_AMP * pulse;         /* grow on beat */
        float   sx    = HEART_BASE_SX * scale;
        float   sy    = HEART_BASE_SY * scale;
        int     val   = MATRIX_BRIGHTNESS + (int)(pulse * HEART_FLARE);

        /* Blank the panel, then paint the heart and the readout fresh. */
        for (int i = 0; i < MATRIX_PIXELS; i++) {
            led_strip_set_pixel(strip, i, 0, 0, 0);
        }

        /* Heart via the implicit curve (x^2+y^2-1)^3 - x^2*y^3 <= 0, with y up
         * so the two lobes sit at the top and the point hangs below. */
        for (int y = 0; y < HEART_ROWS; y++) {
            for (int x = 0; x < MATRIX_WIDTH; x++) {
                float fx = ((float)x - HEART_CX) / sx;
                float fy = (HEART_CY - (float)y) / sy;
                float a  = fx * fx + fy * fy - 1.0f;
                if (a * a * a - fx * fx * fy * fy * fy <= 0.0f) {
                    led_strip_set_pixel_hsv(strip, xy_to_index(x, y), 0, 255, val);
                }
            }
        }

        draw_bpm(strip, g_bpm);

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
    float bpm_smooth = 0.0f;     /* EMA of the inter-beat rate for the panel */

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
            int inst = last_beat_us ? (int)(60000000LL / (now - last_beat_us)) : 0;
            last_beat_us = now;
            led_off_at_us = now + HB_FLASH_MS * 1000;
            g_last_beat_us = now;        /* drives the matrix throb */
            if (inst >= 30 && inst <= 220) {
                bpm_smooth = (bpm_smooth <= 0.0f)
                             ? (float)inst
                             : bpm_smooth * 0.7f + (float)inst * 0.3f;
                g_bpm = (int)(bpm_smooth + 0.5f);
                ESP_LOGI(TAG, "beat  ~%d bpm", g_bpm);
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
