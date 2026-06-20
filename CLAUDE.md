# CLAUDE.md

Guidance for working effectively in this repo. Read this before building, flashing, or editing animations.

## What this is

An **ESP-IDF (v6.0.1) firmware** for an **ESP32-C3**. Two things run concurrently:

1. **An LED-matrix animation** (`matrix_task`) — the part that gets iterated on most. Past approaches: sunrise, thunderstorm, full day→night cycle with stars.
2. **A KY-039 heartbeat detector** (in `app_main`) — samples an analog pulse sensor and blinks the onboard LED per beat. Independent of the matrix; leave it alone unless asked.

All of it lives in a single file: **`main/flyswatter.c`**.

## Hardware / pinout

| Function | Pin | Notes |
|---|---|---|
| WS2812 matrix data (DIN) | GPIO1 | 22×12 = 264 LEDs, GRB, driven via RMT (no DMA on C3) |
| Heartbeat sensor (KY-039 S) | GPIO0 / ADC1_CH0 | 12 dB atten; `+`→3V3, `-`→GND |
| Onboard LED | GPIO8 | **active-low** (drive LOW to light) |

The matrix is **22 wide × 12 tall**. `MATRIX_SERPENTINE` selects boustrophedon vs progressive wiring — the user has it on `0`; always route pixel writes through `xy_to_index()` so the choice is respected.

## Build & flash (important — the environment is finicky)

`idf.py` is **not on PATH by default**. Source the export script first:

```bash
. /Users/personal/.espressif/v6.0.1/esp-idf/export.sh
```

If that errors with "Python virtual environment ... not found", create it once:

```bash
/Users/personal/.espressif/v6.0.1/esp-idf/install.sh esp32c3
```

**Always build into a dedicated dir, NOT `build/`:**

```bash
idf.py -B build_verify build
```

- `build/` is **CLion's** build directory. Driving it from the command line (especially raw `ninja`) desyncs CMake's configuration and leaves it in a broken state (symptoms: ldgen `objdump` failures on empty `libmbed-builtin.a`, missing mbedtls symbols at link). **Don't run raw `ninja` in `build/`.** Let CLion own it.
- `build_verify/` is gitignored, so it's safe to create and leave around.

**Flash** (board enumerates as USB-serial-JTAG; confirm the port first):

```bash
ls /dev/cu.usbmodem*                # find the port
idf.py -B build_verify -p /dev/cu.usbmodem1101 flash
idf.py -B build_verify -p /dev/cu.usbmodem1101 monitor   # Ctrl-] to exit
```

Flashing is an outward hardware action — only do it when the user asks.

## Matrix animation conventions

Everything visual is one self-contained `while(1)` loop in `matrix_task()`, refreshing at ~25–30 fps via `vTaskDelay`. Reusable helpers sit above it — **keep these when swapping animations**:

- **`xy_to_index(x,y)`** — maps logical coords to LED index, handles serpentine wiring.
- **`clampf`, `lerpf`** — float helpers.
- **`put_px(strip, x, y, r, g, b)`** — the brightness gate. Author colors in **full intensity (0–255)**; `put_px` scales by `MATRIX_BRIGHTNESS/255` and clamps each channel to `OUT_CAP`. **Never bypass it** — 264 LEDs at full white is ~15 A. Tune brightness via `MATRIX_BRIGHTNESS` / `OUT_CAP`, not by hand-dimming colors.
- **`g_fb[MATRIX_PIXELS][3]` + `fb_add`** — a linear-light framebuffer. When layering (sky + sun + stars, or rain + lightning), composite additively into `g_fb`, then flush each cell through `put_px`. For a single flat layer you can call `put_px` directly.

Other patterns used:
- **Clock:** `esp_timer_get_time()` (µs). Convert to seconds for animation phase.
- **Randomness:** a tiny `xorshift` PRNG (`rnd_u32`/`rnd_f`), seeded once from the boot clock. No `rand()` / extra components needed.
- **Keyframed timelines:** for multi-stage animations, a shared `KEY_U[]` grid plus per-quantity value tables interpolated by `key_scalar`/`key_vec3`. Make the first and last keyframe equal so the loop is seamless.

**Swapping in a new animation:** replace the render loop and its config `#define`s, and **delete now-unused helpers/defines** — the build treats unused-static warnings as errors, so leftovers (e.g. an old `sky_color`) will fail the build.

## Heartbeat detector (usually leave as-is)

In `app_main`: ADC one-shot sampling at 50 Hz, DC-removal + dynamic-threshold beat detection, onboard-LED flash per beat. Tunables are the `HB_*` defines. `HB_PLOT 1` streams `raw,acf,thr` to serial each sample for the Arduino-style plotter (handy when tuning sensitivity). The matrix and detector are decoupled — don't reintroduce a dependency unless asked.

## Git workflow

- **`main` is ambiguous** with the `main/` source directory. Disambiguate refs explicitly: `git log refs/heads/main`, `git rev-parse refs/heads/main`.
- The user **tries different approaches as commits on a linear `main`** (`feat: sunrise`, `feat: thunderstorm`, `feat: day-night cycle...`). To save a new approach, commit it on top of `main`. Old approaches stay recoverable: `git checkout <sha> -- main/flyswatter.c`.
- Local `main` may be **ahead of `origin/main`**; push only when asked.
- Gitignored: `build/`, `build_verify/`, `sdkconfig`, `managed_components/`, `dependencies.lock`, `.idea/`.
- Match the existing commit style (`feat:` / `fix:` prefixes). Don't `rm -rf` build directories — the user prefers to manage those.
