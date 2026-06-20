# CLAUDE.md

Guidance for working effectively in this repo. Read this before building, flashing, or editing animations.

## What this is

An **ESP-IDF (v6.0.1) firmware** for an **ESP32-C3**. Three things run concurrently:

1. **An LED-matrix animation** (`matrix_task`) — the part that gets iterated on most.
   Past approaches (each a commit on `main`): sunrise, thunderstorm, full day→night cycle
   with stars. Current: a red disco stick-figure dancer.
2. **A KY-039 heartbeat detector** (in `app_main`) — samples an analog pulse sensor and
   blinks the onboard LED per beat. Independent of the matrix; leave it alone unless asked.
3. **An HC-SR04 ultrasonic rangefinder** (`sonar_task`) — pings at 10 Hz and publishes the
   latest distance in `g_distance_cm`. The matrix *optionally* reads it to drive a reactive
   overlay (currently a blue particle fountain, gated by `SONAR_DRIVES_VISUALS`). The echo
   timing is non-blocking by design — leave that path alone unless asked.

All of it lives in a single file: **`main/flyswatter.c`**.

## Hardware / pinout

| Function | Pin | Notes |
|---|---|---|
| WS2812 matrix data (DIN) | GPIO1 | 22×12 = 264 LEDs, GRB, driven via RMT (no DMA on C3) |
| Heartbeat sensor (KY-039 S) | GPIO0 / ADC1_CH0 | 12 dB atten; `+`→3V3, `-`→GND |
| Onboard LED | GPIO8 | **active-low** (drive LOW to light) |
| HC-SR04 Trig | GPIO6 | output; 3.3 V drive triggers the sensor fine |
| HC-SR04 Echo | GPIO7 | input **via a 1 kΩ/2 kΩ divider** — Echo idles at 5 V and the C3 is **not** 5 V-tolerant |

Power: matrix and HC-SR04 run from **5 V** (USB passthrough); KY-039 runs from **3V3**;
all grounds common. A full connection diagram lives in `wiring.svg` / the README.

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

## Concurrency model (single core — keep work non-blocking)

`matrix_task` (prio 5) and `sonar_task` (prio 5) are FreeRTOS tasks; the heartbeat detector
runs in `app_main`'s own loop. There is one RISC-V core, so nothing may busy-wait:

- **The sonar's echo timing is intentional.** `echo_isr` (a GPIO edge ISR) timestamps the
  echo edges in interrupt context, and `sonar_task` *blocks* on `xTaskNotifyWait` instead of
  spinning. A busy-wait / `pulseIn`-style poll would stall the 30 fps matrix and the 50 Hz
  heartbeat, and would also corrupt the pulse width when another task preempts mid-wait.
  Don't reintroduce polling.
- **Shared state:** `g_distance_cm` — one writer (`sonar_task`), one reader (`matrix_task`),
  a 32-bit aligned `volatile float`, no mutex. Don't add locking, and don't add other
  cross-task globals without the same one-writer/one-reader discipline.

## Matrix animation conventions

Everything visual is one self-contained `while(1)` loop in `matrix_task()`, refreshing at ~25–30 fps via `vTaskDelay`. Reusable helpers sit above it — **keep these when swapping animations**:

- **`xy_to_index(x,y)`** — maps logical coords to LED index, handles serpentine wiring.
- **`clampf`, `lerpf`** — float helpers.
- **`put_px(strip, x, y, r, g, b)`** — the brightness gate. Author colors in **full intensity (0–255)**; `put_px` scales by `MATRIX_BRIGHTNESS/255` and clamps each channel to `OUT_CAP`. **Never bypass it** — 264 LEDs at full white is ~15 A. Tune brightness via `MATRIX_BRIGHTNESS` / `OUT_CAP`, not by hand-dimming colors.
- **`g_fb[MATRIX_PIXELS][3]` + `fb_add`** — a linear-light framebuffer. When layering (e.g. floor + figure + particle fountain, or sky + sun + stars), composite additively into `g_fb`, then flush each cell through `put_px`. For a single flat layer you can call `put_px` directly.

Other patterns used:
- **Clock:** `esp_timer_get_time()` (µs). Convert to seconds for animation phase.
- **Randomness:** a tiny `xorshift` PRNG (`rnd_u32`/`rnd_f`), seeded once from the boot clock. No `rand()` / extra components needed. (The particle fountain uses it.)
- **Keyframed timelines:** for multi-stage animations, a shared `KEY_U[]` grid plus per-quantity value tables interpolated by `key_scalar`/`key_vec3`. Make the first and last keyframe equal so the loop is seamless.

**Swapping in a new animation:** replace the render loop and its config `#define`s, and **delete now-unused helpers/defines** — the build treats unused-static warnings as errors, so leftovers (e.g. an old `sky_color`, or the particle pool if you drop the sonar overlay) will fail the build.

## Heartbeat detector (usually leave as-is)

In `app_main`: ADC one-shot sampling at 50 Hz, DC-removal + dynamic-threshold beat detection, onboard-LED flash per beat. Tunables are the `HB_*` defines. `HB_PLOT 1` streams `raw,acf,thr` to serial each sample for the Arduino-style plotter (handy when tuning sensitivity). The matrix and detector are decoupled — don't reintroduce a dependency unless asked.

## HC-SR04 sonar (leave the timing path as-is)

`ENABLE_SONAR` gates the whole subsystem; `SONAR_DRIVES_VISUALS` gates the matrix overlay
only. `SONAR_PLOT 1` streams `dist,<cm>` for the plotter — but it shares stdout with
`HB_PLOT`, so **enable only one plotter at a time**. Tune behaviour via the `SONAR_*` defines
(ping rate, distance window, particle look). The `echo_isr` + `sonar_task` structure is the
non-blocking design described under *Concurrency model* — tune around it, don't replace it
with a polling loop.

## Git workflow

- **`main` is ambiguous** with the `main/` source directory. Disambiguate refs explicitly: `git log refs/heads/main`, `git rev-parse refs/heads/main`.
- The user **tries different approaches as commits on a linear `main`** (`feat: sunrise`, `feat: thunderstorm`, `feat: day-night cycle...`). To save a new approach, commit it on top of `main`. Old approaches stay recoverable: `git checkout <sha> -- main/flyswatter.c`.
- Local `main` may be **ahead of `origin/main`**; push only when asked.
- Gitignored: `build/`, `build_verify/`, `sdkconfig`, `managed_components/`, `dependencies.lock`, `.idea/`.
- Match the existing commit style (`feat:` / `fix:` prefixes). Don't `rm -rf` build directories — the user prefers to manage those.