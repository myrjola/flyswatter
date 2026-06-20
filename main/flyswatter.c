#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"             /* for esp_rom_delay_us() */
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "led_strip.h"

/* ======================================================================== */
/* LED matrix (red stick figure dancing to a 4/4 disco beat, as its own task)*/
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

/* A red stick figure grooving on a dark-green floor to a steady 4/4 beat,
 * cycling smoothly through a handful of disco moves. Tempo and proportions
 * below; the choreography lives in the move functions further down. */
#define BPM            112.0f   /* moderate disco tempo (beats per minute)    */
#define MOVE_BARS      4        /* bars (4 beats each) a move holds before    */
                                /* it crossfades into the next                */
#define XFADE_BEATS    1.0f     /* length of that crossfade, in beats         */
#define OUT_CAP        56       /* per-channel ceiling -> bounds current      */

/* Figure geometry, in matrix pixels (22 wide x 12 tall). The dancer is built
 * by forward kinematics from a pelvis anchor each frame. */
#define DANCE_PI    3.14159265f
#define CX          10.5f       /* horizontal centre of the stage             */
#define PELVIS_Y    7.0f        /* pelvis height (y grows downward)           */
#define TORSO_LEN   3.2f        /* pelvis -> neck                             */
#define UPPER_ARM   1.8f
#define FOREARM     1.7f
#define THIGH       2.0f
#define SHIN        2.0f
#define SHOULDER_W  1.1f        /* half-width of the shoulder line            */
#define HIP_W       0.9f        /* half-width of the hip line                 */
#define NECK_HEAD   0.6f        /* neck -> head-centre gap (beyond HEAD_R)     */
#define HEAD_R      0.8f        /* head disc solid radius, px                 */
#define LIMB_HALF   0.25f       /* limb solid half-thickness, px              */
#define LIMB_AA     0.55f       /* anti-aliased feather beyond the solid core */

/* Colours, authored at full intensity (0-255); put_px dims to the panel's
 * brightness budget. Dark green stage, hot red dancer. */
#define BG_R    0.0f
#define BG_G   55.0f
#define BG_B   12.0f
#define FIG_R 255.0f
#define FIG_G  18.0f
#define FIG_B  14.0f

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

/* ------------------------------------------------------------------------ */
/* HC-SR04 ultrasonic distance                                               */
/* ------------------------------------------------------------------------ */
#define ENABLE_SONAR       1
#define SONAR_TRIG_GPIO    GPIO_NUM_6
#define SONAR_ECHO_GPIO    GPIO_NUM_7      /* via 1k/2k divider -- Echo idles at 5V */
#define SONAR_PING_MS      100             /* 10 Hz; HC-SR04 needs >60 ms between pings */
#define SONAR_TIMEOUT_MS   30              /* ~5 m ceiling -> report no-target on timeout */
#define SONAR_CM_PER_US    (1.0f / 58.0f)  /* round-trip speed of sound */
#define SONAR_MIN_US       100             /* reject implausibly short echoes (noise) */
#define SONAR_MAX_US       30000
#define SONAR_PLOT         0               /* stream "dist,<cm>"; collides with HB_PLOT, enable one */
#define SONAR_DRIVES_VISUALS 1             /* blue-particle fountain below; 0 = pure dancer */

/* Particle-fountain tunables (used only when SONAR_DRIVES_VISUALS). Blue sparks
 * rise from the bottom toward the top; the closer the target, the more of them. */
#define SONAR_NEAR_CM      5.0f            /* <= this -> peak spawn rate */
#define SONAR_FAR_CM       50.0f           /* >= this (or no target) -> no spawns */
#define PARTICLE_MAX       96              /* pool size (static, off the task stack) */
#define PARTICLE_RATE      55.0f           /* spawns/sec at full closeness */
#define PARTICLE_VY_MIN    0.18f           /* upward speed, rows/frame */
#define PARTICLE_VY_MAX    0.45f
#define PARTICLE_DRIFT     0.06f           /* sideways wander, rows/frame */
#define PARTICLE_ALPHA     0.30f           /* EMA smoothing of closeness (anti-jitter) */

static const char *TAG = "flyswatter";

/* Latest ultrasonic distance in cm; <0 means "no target / out of range".
 * Written by sonar_task, read by matrix_task. One writer, one reader, 32-bit
 * aligned volatile float -> no tearing, no mutex needed on this MCU. */
static volatile float g_distance_cm = -1.0f;

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

/* Linear-light accumulation buffer: the floor fills it, then the dancer's
 * limbs add their light over the top before it is flushed to the strip. */
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

#if SONAR_DRIVES_VISUALS
/* Tiny xorshift PRNG for the particle fountain (seeded once from the boot clock). */
static uint32_t s_rng = 0x2545F491u;
static inline uint32_t rnd_u32(void)
{
    s_rng ^= s_rng << 13; s_rng ^= s_rng >> 17; s_rng ^= s_rng << 5;
    return s_rng;
}
static inline float rnd_f(void) { return (rnd_u32() >> 8) * (1.0f / 16777216.0f); } /* [0,1) */

/* Blue sparks rising bottom->top. Inactive when life <= 0. Pool lives in BSS,
 * not on matrix_task's stack. */
typedef struct { float x, y, vx, vy, life; } Particle;
static Particle g_parts[PARTICLE_MAX];
#endif

/* ------------------------------------------------------------------------ */
/* Dancer: pose -> skeleton -> anti-aliased bones                            */
/* ------------------------------------------------------------------------ */

/* A pose is just the joint angles of the figure plus a few whole-body offsets.
 * Angles are absolute, measured clockwise from straight up (0 = up, +PI/2 =
 * screen-right, -PI/2 = screen-left, PI = straight down). Each move below is a
 * function of the running beat count that fills one of these in. */
typedef struct {
    float bob;        /* vertical lift, px (+ = up)                          */
    float hipShift;   /* pelvis x offset, px                                 */
    float lean;       /* torso lean from vertical, rad (+ = toward screen-R) */
    float armL, foreL;  /* left  arm: shoulder & forearm angle               */
    float armR, foreR;  /* right arm: shoulder & forearm angle               */
    float legL, shinL;  /* left  leg: hip & shin angle                       */
    float legR, shinR;  /* right leg: hip & shin angle                       */
} Pose;

typedef struct { float x, y; } Pt;

/* Step from p by len along an absolute clock-from-up angle. */
static inline Pt pt_step(Pt p, float len, float ang)
{
    Pt q = { p.x + len * sinf(ang), p.y - len * cosf(ang) };
    return q;
}

typedef struct {
    Pt pelvis, neck, head;
    Pt shL, elbowL, handL,  shR, elbowR, handR;
    Pt hipL, kneeL, footL,  hipR, kneeR, footR;
} Skel;

/* Forward kinematics: turn a pose into world-space joint positions. */
static void build_skeleton(const Pose *p, Skel *s)
{
    Pt pelvis = { CX + p->hipShift, PELVIS_Y - p->bob };
    /* torso "up" and a perpendicular pointing toward screen-right */
    float ux = sinf(p->lean), uy = -cosf(p->lean);
    float px = cosf(p->lean), py =  sinf(p->lean);

    Pt neck = { pelvis.x + TORSO_LEN * ux, pelvis.y + TORSO_LEN * uy };
    Pt head = { neck.x + (HEAD_R + NECK_HEAD) * ux,
                neck.y + (HEAD_R + NECK_HEAD) * uy };

    s->pelvis = pelvis;  s->neck = neck;  s->head = head;
    s->shR = (Pt){ neck.x + SHOULDER_W * px, neck.y + SHOULDER_W * py };
    s->shL = (Pt){ neck.x - SHOULDER_W * px, neck.y - SHOULDER_W * py };
    s->hipR = (Pt){ pelvis.x + HIP_W * px, pelvis.y + HIP_W * py };
    s->hipL = (Pt){ pelvis.x - HIP_W * px, pelvis.y - HIP_W * py };

    s->elbowR = pt_step(s->shR, UPPER_ARM, p->armR);
    s->handR  = pt_step(s->elbowR, FOREARM, p->foreR);
    s->elbowL = pt_step(s->shL, UPPER_ARM, p->armL);
    s->handL  = pt_step(s->elbowL, FOREARM, p->foreL);

    s->kneeR = pt_step(s->hipR, THIGH, p->legR);
    s->footR = pt_step(s->kneeR, SHIN, p->shinR);
    s->kneeL = pt_step(s->hipL, THIGH, p->legL);
    s->footL = pt_step(s->kneeL, SHIN, p->shinL);
}

/* Rasterise a capsule (line segment of half-thickness `half`) additively into
 * g_fb, with a ~1px anti-aliased edge from distance-to-segment coverage. A
 * degenerate segment (a == b) yields a filled disc -- that's how the head is
 * drawn. */
static void draw_bone(Pt a, Pt b, float half, float r, float g, float bl)
{
    float pad = half + LIMB_AA + 1.0f;
    int x0 = (int)floorf(fminf(a.x, b.x) - pad);
    int x1 = (int)ceilf (fmaxf(a.x, b.x) + pad);
    int y0 = (int)floorf(fminf(a.y, b.y) - pad);
    int y1 = (int)ceilf (fmaxf(a.y, b.y) + pad);
    float vx = b.x - a.x, vy = b.y - a.y;
    float vv = vx * vx + vy * vy;
    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            float wx = (float)x - a.x, wy = (float)y - a.y;
            float t  = vv > 1e-4f ? clampf((wx * vx + wy * vy) / vv, 0.0f, 1.0f) : 0.0f;
            float dx = wx - t * vx, dy = wy - t * vy;
            float d  = sqrtf(dx * dx + dy * dy);
            float cov = clampf(1.0f - (d - half) / LIMB_AA, 0.0f, 1.0f);
            if (cov > 0.0f) fb_add(x, y, r * cov, g * cov, bl * cov);
        }
    }
}

/* Draw the whole dancer in figure red. */
static void draw_figure(const Skel *s)
{
    const float h = LIMB_HALF, R = FIG_R, G = FIG_G, B = FIG_B;
    draw_bone(s->pelvis, s->neck, h, R, G, B);            /* spine            */
    draw_bone(s->shL,    s->shR,  h * 0.8f, R, G, B);     /* shoulders        */
    draw_bone(s->hipL,   s->hipR, h * 0.8f, R, G, B);     /* hips             */
    draw_bone(s->shR, s->elbowR, h, R, G, B);  draw_bone(s->elbowR, s->handR, h, R, G, B);
    draw_bone(s->shL, s->elbowL, h, R, G, B);  draw_bone(s->elbowL, s->handL, h, R, G, B);
    draw_bone(s->hipR, s->kneeR, h, R, G, B);  draw_bone(s->kneeR, s->footR, h, R, G, B);
    draw_bone(s->hipL, s->kneeL, h, R, G, B);  draw_bone(s->kneeL, s->footL, h, R, G, B);
    draw_bone(s->head, s->head, HEAD_R, R, G, B);         /* head disc        */
}

/* ------------------------------------------------------------------------ */
/* Choreography: each move is a function of the running beat count `b`        */
/* (so fmodf(b,1) is the phase within the current quarter note, and a bar is  */
/* 4 beats). Moves share the per-beat bounce added in matrix_task; here they  */
/* only express their own style. They are blended pairwise to cross-fade.     */
/* ------------------------------------------------------------------------ */
typedef void (*MoveFn)(float b, Pose *p);

/* A relaxed default stance the legs mostly keep; individual moves override. */
static void legs_stand(Pose *p, float spread)
{
    p->legR = DANCE_PI - spread;  p->shinR = DANCE_PI - spread * 0.5f;
    p->legL = DANCE_PI + spread;  p->shinL = DANCE_PI + spread * 0.5f;
}

/* 1. Saturday Night Fever point: one straight arm stabs up-diagonally, the
 *    other rests low at the hip; the active side flips every two beats. */
static void mv_point(float b, Pose *p)
{
    float seg  = floorf(b / 2.0f);                 /* switch sides every 2 beats */
    int   right = ((int)fmodf(seg, 2.0f)) == 0;
    float beatp = b - floorf(b);
    float ext   = 0.18f * sinf(DANCE_PI * beatp);  /* a little stab on the beat  */
    if (right) {
        p->armR =  0.85f - ext;  p->foreR =  0.85f - ext;   /* up-right point */
        p->armL =  DANCE_PI + 0.35f;  p->foreL = DANCE_PI + 0.15f;
    } else {
        p->armL = -(0.85f - ext); p->foreL = -(0.85f - ext); /* up-left point */
        p->armR =  DANCE_PI - 0.35f;  p->foreR = DANCE_PI - 0.15f;
    }
    legs_stand(p, 0.22f);
    p->lean     = right ?  0.12f : -0.12f;
    p->hipShift = right ?  0.5f  : -0.5f;
}

/* 2. Raise the roof: both forearms punch upward on every beat. */
static void mv_roof(float b, Pose *p)
{
    float beatp = b - floorf(b);
    float push  = 0.5f + 0.5f * cosf(2.0f * DANCE_PI * beatp);  /* peak on beat */
    p->armR =  0.7f;  p->foreR =  0.22f - 0.22f * push;   /* forearm -> vertical */
    p->armL = -0.7f;  p->foreL = -0.22f + 0.22f * push;
    legs_stand(p, 0.18f);
    p->lean = 0.0f;  p->hipShift = 0.0f;  p->bob = 0.55f * push;
}

/* 3. Hip sway / side-step: weight rolls side to side, arms swing counter. */
static void mv_sway(float b, Pose *p)
{
    float s = sinf(DANCE_PI * b);                   /* one sway per two beats */
    p->hipShift = 1.4f * s;
    p->lean     = 0.18f * s;
    p->armR = DANCE_PI - 0.6f - 0.5f * s;  p->foreR = DANCE_PI - 0.5f - 0.4f * s;
    p->armL = DANCE_PI + 0.6f - 0.5f * s;  p->foreL = DANCE_PI + 0.5f - 0.4f * s;
    p->legR = DANCE_PI - 0.25f - 0.15f * s;  p->shinR = DANCE_PI - 0.12f;
    p->legL = DANCE_PI + 0.25f - 0.15f * s;  p->shinL = DANCE_PI + 0.12f;
}

/* 4. Sprinkler: a straight arm sweeps slowly across, the other tucks behind
 *    the head. */
static void mv_sprinkler(float b, Pose *p)
{
    float sweep = 0.9f * sinf(DANCE_PI * b * 0.5f); /* slow front sweep */
    p->armR = 0.2f + sweep;  p->foreR = 0.2f + sweep;
    p->armL = -0.3f;         p->foreL = -1.4f;       /* bent behind head */
    legs_stand(p, 0.2f);
    p->lean     = 0.10f * sinf(DANCE_PI * b * 0.5f);
    p->hipShift = 0.4f  * sinf(DANCE_PI * b * 0.5f);
}

/* 5. Shimmy bounce: knees bend on the beat while arms stick out and shake. */
static void mv_shimmy(float b, Pose *p)
{
    float beatp = b - floorf(b);
    float bend  = 0.5f + 0.5f * cosf(2.0f * DANCE_PI * beatp);
    float sh    = 0.25f * sinf(4.0f * DANCE_PI * b);            /* fast shake */
    p->armR =  DANCE_PI * 0.5f + 0.1f + sh;  p->foreR =  DANCE_PI * 0.5f - 0.2f + sh;
    p->armL = -DANCE_PI * 0.5f - 0.1f + sh;  p->foreL = -DANCE_PI * 0.5f + 0.2f + sh;
    p->legR = DANCE_PI - 0.28f;  p->shinR = DANCE_PI - 0.28f - 0.3f * bend;
    p->legL = DANCE_PI + 0.28f;  p->shinL = DANCE_PI + 0.28f + 0.3f * bend;
    p->lean = 0.0f;  p->hipShift = 0.0f;  p->bob = -0.6f * bend;
}

/* 6. Overhead V punch: both arms throw up into a V on each downbeat. */
static void mv_v(float b, Pose *p)
{
    float beatp = b - floorf(b);
    float punch = 0.5f + 0.5f * cosf(2.0f * DANCE_PI * beatp);
    float spread = 0.6f - 0.25f * punch;            /* V narrows as it punches */
    p->armR =  spread;  p->foreR =  spread;
    p->armL = -spread;  p->foreL = -spread;
    p->legR = DANCE_PI - 0.2f - 0.1f * punch;  p->shinR = DANCE_PI - 0.1f;
    p->legL = DANCE_PI + 0.2f + 0.1f * punch;  p->shinL = DANCE_PI + 0.1f;
    p->lean = 0.0f;  p->hipShift = 0.0f;  p->bob = 0.5f * punch;
}

/* 7. Bodybuilder flex: upper arms held straight out to the sides, forearms
 *    curling vertical -- one up, one down -- then smoothly swapping. */
static void mv_flex(float b, Pose *p)
{
    float a = 0.5f + 0.5f * sinf(DANCE_PI * b);      /* 0..1, swaps each beat */
    p->armR =  DANCE_PI * 0.5f;   /* upper arm out to screen-right, horizontal */
    p->armL = -DANCE_PI * 0.5f;   /* upper arm out to screen-left,  horizontal */
    p->foreR = lerpf(DANCE_PI, 0.0f, a);   /* a=1 -> up,   a=0 -> down */
    p->foreL = lerpf(0.0f, DANCE_PI, a);   /* a=1 -> down, a=0 -> up   */
    legs_stand(p, 0.2f);
    p->lean     = 0.12f * (2.0f * a - 1.0f);   /* lean toward the raised arm */
    p->hipShift = 0.0f;
}

/* 8. Moonwalk: glide all the way across, entering and leaving fully off-stage,
 *    while the feet do a back-shuffle on the beat. Recreates the same per-move
 *    phase sequence_pose() uses so the glide spans exactly one move slot, eased
 *    so it dwells off-screen at both ends (crossfades land while invisible). */
static void mv_moonwalk(float b, Pose *p)
{
    float movelen = (float)MOVE_BARS * 4.0f;
    float u = b / movelen - floorf(b / movelen);    /* 0..1 across the slot */
    float s = u * u * (3.0f - 2.0f * u);            /* ease in/out */
    p->hipShift = lerpf(-16.0f, 16.0f, s);          /* off-left -> off-right */

    float beatp = b - floorf(b);
    float step  = sinf(2.0f * DANCE_PI * beatp);    /* foot shuffle on the beat */
    p->legR  = DANCE_PI - 0.15f + 0.45f * step;
    p->shinR = DANCE_PI - 0.05f + 0.30f * step;
    p->legL  = DANCE_PI + 0.15f - 0.45f * step;
    p->shinL = DANCE_PI + 0.05f - 0.30f * step;

    p->lean  = -0.18f;                               /* lean back into the glide */
    p->armR = DANCE_PI - 0.5f - 0.4f * step;  p->foreR = DANCE_PI - 0.4f - 0.3f * step;
    p->armL = DANCE_PI + 0.5f + 0.4f * step;  p->foreL = DANCE_PI + 0.4f + 0.3f * step;
}

static const MoveFn MOVES[] = {
    mv_point, mv_roof, mv_flex, mv_sway, mv_sprinkler, mv_shimmy, mv_v, mv_moonwalk,
};
#define NMOVES ((int)(sizeof(MOVES) / sizeof(MOVES[0])))

#define POSE_BLEND(field) out->field = lerpf(a.field, c.field, t)
static void blend_pose(Pose a, Pose c, float t, Pose *out)
{
    POSE_BLEND(bob);   POSE_BLEND(hipShift); POSE_BLEND(lean);
    POSE_BLEND(armL);  POSE_BLEND(foreL);    POSE_BLEND(armR);  POSE_BLEND(foreR);
    POSE_BLEND(legL);  POSE_BLEND(shinL);    POSE_BLEND(legR);  POSE_BLEND(shinR);
}
#undef POSE_BLEND

/* Pick the current move from the running beat count, cross-fading into the
 * next one over the final XFADE_BEATS of each move so the loop never snaps. */
static void sequence_pose(float b, Pose *out)
{
    float movelen = (float)MOVE_BARS * 4.0f;        /* beats per move */
    float mi  = b / movelen;
    int   idx = (int)floorf(mi);
    float frac = mi - (float)idx;                   /* 0..1 within the move */

    Pose cur = (Pose){0};
    MOVES[((idx % NMOVES) + NMOVES) % NMOVES](b, &cur);

    float xf = XFADE_BEATS / movelen;               /* fraction spent fading */
    if (frac > 1.0f - xf) {
        float w = (frac - (1.0f - xf)) / xf;        /* 0..1 across the fade */
        w = w * w * (3.0f - 2.0f * w);              /* smoothstep */
        Pose nxt = (Pose){0};
        MOVES[(((idx + 1) % NMOVES) + NMOVES) % NMOVES](b, &nxt);
        blend_pose(cur, nxt, w, out);
    } else {
        *out = cur;
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
    ESP_LOGI(TAG, "%dx%d matrix ready on GPIO%d (%d LEDs) -- disco dancer @ %d bpm",
             MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX_GPIO, MATRIX_PIXELS, (int)BPM);

#if SONAR_DRIVES_VISUALS
    s_rng ^= (uint32_t)esp_timer_get_time() | 1u;   /* seed the particle PRNG */
#endif

    while (1) {
        float ts = (float)esp_timer_get_time() / 1e6f;
        float b  = ts * (BPM / 60.0f);              /* beats elapsed */
        float beatp = b - floorf(b);                /* phase within the beat */

        /* Dark-green floor with a subtle pulse that swells on each beat. */
        float pulse = 0.5f + 0.5f * cosf(2.0f * DANCE_PI * beatp);
        float gr = BG_R;
        float gg = BG_G * (1.0f + 0.40f * pulse);
        float gb = BG_B * (1.0f + 0.40f * pulse);
        for (int i = 0; i < MATRIX_PIXELS; i++) {
            g_fb[i][0] = gr; g_fb[i][1] = gg; g_fb[i][2] = gb;
        }

        /* Choreograph, add the shared per-beat knee-dip bounce, draw. */
        Pose pose;
        sequence_pose(b, &pose);
        pose.bob += -0.5f * (0.5f + 0.5f * cosf(2.0f * DANCE_PI * beatp));
        Skel sk;
        build_skeleton(&pose, &sk);
        draw_figure(&sk);

#if SONAR_DRIVES_VISUALS
        /* Blue-particle fountain: sparks rise from the bottom toward the top,
         * and the closer the target the more we spawn. Composited additively
         * over the dancer; put_px still gates the brightness/current. */
        {
            static float close_s = 0.0f;     /* smoothed closeness 0..1 */
            static float spawn_acc = 0.0f;   /* fractional spawn carry */
            const float dt = 0.033f;         /* matches the ~30 fps frame delay */

            float d = g_distance_cm;          /* single volatile read */
            float close = (d < 0.0f) ? 0.0f
                : clampf((SONAR_FAR_CM - d) / (SONAR_FAR_CM - SONAR_NEAR_CM), 0.0f, 1.0f);
            close_s += PARTICLE_ALPHA * (close - close_s);   /* EMA anti-jitter */

            /* Spawn from the bottom edge in proportion to closeness. */
            spawn_acc += close_s * PARTICLE_RATE * dt;
            while (spawn_acc >= 1.0f) {
                spawn_acc -= 1.0f;
                for (int i = 0; i < PARTICLE_MAX; i++) {
                    if (g_parts[i].life <= 0.0f) {
                        g_parts[i].x  = rnd_f() * MATRIX_WIDTH;
                        g_parts[i].y  = MATRIX_HEIGHT - 1.0f;
                        g_parts[i].vx = (rnd_f() - 0.5f) * 2.0f * PARTICLE_DRIFT;
                        g_parts[i].vy = lerpf(PARTICLE_VY_MIN, PARTICLE_VY_MAX, rnd_f());
                        g_parts[i].life = 1.0f;
                        break;
                    }
                }
            }

            /* Advance and draw every live spark as a bright core + faint tail. */
            for (int i = 0; i < PARTICLE_MAX; i++) {
                if (g_parts[i].life <= 0.0f) continue;
                g_parts[i].y  -= g_parts[i].vy;
                g_parts[i].x  += g_parts[i].vx;
                if (g_parts[i].y < -1.0f) { g_parts[i].life = 0.0f; continue; }

                float fade = clampf(g_parts[i].y / 2.0f, 0.0f, 1.0f);  /* dim near the top */
                int xi = (int)(g_parts[i].x + 0.5f);
                int y0 = (int)floorf(g_parts[i].y);
                float fy = g_parts[i].y - y0;                          /* sub-pixel split */
                /* blue spark: low red, some green, full blue */
                float cr = 30.0f * fade, cg = 90.0f * fade, cb = 255.0f * fade;
                fb_add(xi, y0,     cr * (1.0f - fy), cg * (1.0f - fy), cb * (1.0f - fy));
                fb_add(xi, y0 + 1, cr * fy,          cg * fy,          cb * fy);
                fb_add(xi, y0 + 2, cr * 0.25f,       cg * 0.25f,       cb * 0.25f); /* tail */
            }
        }
#endif

        /* Flush the composed frame to the panel. */
        for (int y = 0; y < MATRIX_HEIGHT; y++) {
            for (int x = 0; x < MATRIX_WIDTH; x++) {
                int i = y * MATRIX_WIDTH + x;
                put_px(strip, x, y, g_fb[i][0], g_fb[i][1], g_fb[i][2]);
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

#if ENABLE_SONAR
/* ------------------------------------------------------------------------ */
/* HC-SR04 ultrasonic                                                        */
/* ------------------------------------------------------------------------ */
/* Non-blocking: a GPIO edge ISR timestamps the echo edges (preempts tasks,
 * so the width stays accurate), and sonar_task *blocks* on a task notification
 * instead of busy-spinning -- keeping the core free for the 30 fps dancer and
 * the 50 Hz heartbeat sampler. */
static TaskHandle_t s_sonar_task;

static void IRAM_ATTR echo_isr(void *arg)
{
    static int64_t t_rise;
    if (gpio_get_level(SONAR_ECHO_GPIO)) {              /* rising edge */
        t_rise = esp_timer_get_time();
    } else {                                            /* falling edge */
        uint32_t width = (uint32_t)(esp_timer_get_time() - t_rise);
        BaseType_t hpw = pdFALSE;
        xTaskNotifyFromISR(s_sonar_task, width, eSetValueWithOverwrite, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}

static void sonar_task(void *arg)
{
    gpio_config_t trig = { .pin_bit_mask = 1ULL << SONAR_TRIG_GPIO,
                           .mode = GPIO_MODE_OUTPUT };
    gpio_config(&trig);
    gpio_set_level(SONAR_TRIG_GPIO, 0);

    gpio_config_t echo = { .pin_bit_mask = 1ULL << SONAR_ECHO_GPIO,
                           .mode = GPIO_MODE_INPUT, .intr_type = GPIO_INTR_ANYEDGE };
    gpio_config(&echo);

    s_sonar_task = xTaskGetCurrentTaskHandle();         /* set BEFORE adding the ISR */
    gpio_install_isr_service(0);                        /* INVALID_STATE if already installed -> ok */
    gpio_isr_handler_add(SONAR_ECHO_GPIO, echo_isr, NULL);

    ESP_LOGI(TAG, "sonar: HC-SR04 trig GPIO%d, echo GPIO%d", SONAR_TRIG_GPIO, SONAR_ECHO_GPIO);

    while (1) {
        gpio_set_level(SONAR_TRIG_GPIO, 1);             /* 10 us trigger pulse */
        esp_rom_delay_us(10);                           /* the ONLY busy-wait, and it's tiny */
        gpio_set_level(SONAR_TRIG_GPIO, 0);

        uint32_t width = 0;
        if (xTaskNotifyWait(0, 0, &width, pdMS_TO_TICKS(SONAR_TIMEOUT_MS)) == pdTRUE
            && width >= SONAR_MIN_US && width <= SONAR_MAX_US) {
            g_distance_cm = width * SONAR_CM_PER_US;
        } else {
            g_distance_cm = -1.0f;                       /* timeout: out of range / no target */
        }
#if SONAR_PLOT
        printf("dist,%d\n", (int)g_distance_cm);
#endif
        vTaskDelay(pdMS_TO_TICKS(SONAR_PING_MS));
    }
}
#endif /* ENABLE_SONAR */

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

#if ENABLE_SONAR
    xTaskCreate(sonar_task, "sonar", 3072, NULL, 5, NULL);
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
