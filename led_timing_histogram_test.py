#!/usr/bin/env python3
"""
Closed-loop LED control timing test -- full per-sample histogram variant.

Same core measurement as led_timing_test_raw.py (command a toggle, time how
long until the camera detects the new state, repeat) -- but where that
script only ever reported mean/std/min/max at the end, this one keeps every
single toggle's latency sample, saves the raw arrays to results/*.npz, and
renders a histogram (plus a percentile summary) instead of throwing the
per-sample detail away after computing aggregate stats.

Drives LED_PIN=22 (BCM), not the older LED_PIN=14 used by
led_timing_test_raw.py/led_calibrate_raw.py -- GPIO22 is the pin
camera_view_tool.py's heartbeat blink now uses (see that script's
HEARTBEAT_LED_PIN), and this test's LED was physically repositioned for
that pin, not the old one. Camera scope is single-camera (camera 0) only
-- camera 1 (i2c@80000) isn't currently enumerating this boot (its
already-documented intermittent ribbon-seating fault, see CLAUDE.md's
"Hardware status" section), so a dual-camera version of this test isn't
possible against current hardware without reseating it first.

ROI and THRESHOLD are auto-calibrated at startup instead of hardcoded --
the LED moved to a new physical position for GPIO22, so the old scripts'
fixed ROI=(300,220,40,40)/THRESHOLD=2.51 (calibrated for GPIO14's old
position) can't be trusted here. Calibration: average several frames with
the LED held OFF, several with it held ON, diff them, and center a fixed-
size box on the pixel of peak positive difference -- robust to not knowing
the LED's new screen position in advance, same spirit as led_calibrate_raw.py's
"move ROI onto LED, check the number" workflow but automated instead of
requiring a human to eyeball a live window. Saves a debug PNG
(led_timing_histogram_roi_debug.png) with the detected box overlaid so the
calibration can be sanity-checked after the fact.

Real finding from the actual bench run this LED/camera setup produced
(2026-11-19): the LED floods the ENTIRE sensor to full saturation (65280,
confirmed: on_frame.min() == on_frame.max() == 65280 across every single
pixel) when lit, rather than appearing as a localized bright spot --
confirmed by user observation ("the whole frame gets brighter") and a
direct capture check. This does NOT hurt the latency measurement -- if
anything a full-frame flood is a MORE robust toggle signal than a small
point source, since detection can't be thrown off by imprecise ROI
placement -- but it does mean the debug PNG is rendered from the OFF-state
frame (the only one with real spatial texture to show) rather than ON, and
that "ROI centered on the LED" is really "ROI centered wherever the OFF
frame happened to be darkest," not a meaningful point-source location.
Deliberately left as-is rather than tuning exposure/gain down to avoid
saturation (a live option, not pursued) -- the per-sample latency data
this script produces is valid either way.

Defaults to MODE_640_100_ROI at its validated ~880fps/1050us floor (CLAUDE.md
"quarter-tier ROI modes"), NOT led_timing_test_raw.py's old stock-mode
200fps/5000us baseline -- the whole point of this characterization is
whether the camera is fast enough for real closed-loop beam tracking
(CLAUDE.md's 10ms-loop-latency / 10-20Hz-disturbance-band target), which a
conservative baseline inherited from an older, slower-mode test can't
answer. See CLAUDE.md for the ~5ms-period-baseline results kept as a
reference/comparison point, not the headline number.

Usage:
  python3 led_timing_histogram_test.py [duration_s] [frame_duration_us] [raw_size] [random_delay]
    duration_s         total test window, default 10.0 (longer than
                        led_timing_test_raw.py's 5.0s default -- a histogram
                        wants more samples than a single mean/std needs to
                        be trustworthy)
    frame_duration_us   overrides FORCE_FRAME_DURATION_US (default 1050,
                        MODE_640_100_ROI's validated floor) -- also useful
                        for testing whether the histogram's peak structure
                        scales with the camera's frame period (it does, see
                        CLAUDE.md's frame-cadence-quantization finding)
    raw_size            overrides RAW_SIZE (default 640x100), e.g. 640x200
                        for this project's other fast binned ROI mode
    random_delay        pass the literal string "random" to insert a random
                        delay (0 to RANDOM_DELAY_PERIODS frame periods)
                        before each command instead of firing immediately
                        after detecting the previous toggle -- decorrelates
                        command phase from the camera's own clock, giving a
                        more representative "how long after an arbitrary
                        real disturbance does the camera first see it"
                        distribution than the default immediate-re-command
                        mode (which is entrained to the camera's clock, see
                        CLAUDE.md). Any other value (or omitting it) keeps
                        the default immediate-re-command behavior.

Install:  pip install lgpio opencv-python numpy matplotlib
          (picamera2 is pre-installed on RPi OS Bookworm)
"""

import random
import statistics
import sys
import time
from datetime import datetime, timezone

import lgpio
import matplotlib
matplotlib.use("Agg")  # headless -- this Pi has no need to pop a plot
                         # window open, just save the PNG
import matplotlib.pyplot as plt
import numpy as np
from picamera2 import Picamera2

# ── Config ────────────────────────────────────────────────────────────────
LED_PIN       = 22             # BCM -- camera_view_tool.py's heartbeat pin,
                                  # NOT led_timing_test_raw.py's old LED_PIN=14
GPIOCHIP      = 4              # /dev/gpiochip4 on RPi 5
RAW_SIZE      = (640, 100)     # MODE_640_100_ROI, this project's fastest
                                  # validated binned mode (CLAUDE.md) --
                                  # NOT led_timing_test_raw.py's old stock
                                  # (640,400) mode. The whole point of this
                                  # test is characterizing whether the
                                  # camera is fast enough for real beam
                                  # tracking (CLAUDE.md's 10ms-loop-latency,
                                  # 10-20Hz-disturbance-band target) -- a
                                  # conservative 200fps baseline inherited
                                  # from an older, slower-mode test doesn't
                                  # answer that question, only the fastest
                                  # mode actually in use for tracking does.
RAW_FORMAT    = "R8"
EXPOSURE_US   = 1500           # same settled config as the rest of this
ANALOGUE_GAIN = 4.0            # project's LED timing scripts
FORCE_FRAME_DURATION_US = 1050  # MODE_640_100_ROI's own validated clean
                                  # floor (~880fps, CLAUDE.md's "quarter-tier
                                  # ROI modes" section) -- re-confirmed live
                                  # at startup below via FrameDurationLimits
                                  # rather than trusted blindly, since this
                                  # script has never run at this mode before

ROI_BOX_SIZE   = 40   # pixels, matches led_calibrate_raw.py's ROI size
CALIB_FRAMES   = 15   # frames averaged per LED state during auto-calibration
CALIB_SETTLE_S = 0.2  # settle time after toggling the LED before sampling
MIN_SEPARATION = 0.20  # same relative-separation sanity check as
                         # led_calibrate_raw.py: (on_mean-off_mean)/off_mean
                         # must clear this or calibration is rejected

TIMEOUT_S = 1.0  # safety: max wait for a single detection before forcing
                   # a flip anyway (matches led_timing_test_raw.py)

DEFAULT_DURATION_S = 10.0

RANDOM_DELAY_PERIODS = 3  # when enabled (see RANDOM_DELAY_MS arg below), the
                            # random pre-command delay is drawn uniformly
                            # from [0, this many frame periods) -- needs to
                            # span at least one full period for every phase
                            # relative to the camera's internal clock to get
                            # equal coverage; a few periods of margin makes
                            # that coverage more even without costing much
                            # extra test time at these frame rates.

def plot_histogram(lat_ms, directions, timeouts, n, test_duration_s, led_pin,
                    random_delay_enabled, png_path):
    """Shared by the live-test path below and --replot PATH (regenerates a
    PNG from an already-saved npz without touching the camera/GPIO at all)
    -- kept as one function so a plot-layout fix only ever needs to happen
    in one place."""
    lat_arr = np.array(lat_ms)
    directions = np.array(directions)
    on_lat = lat_arr[directions == 1]
    off_lat = lat_arr[directions == 0]

    fig, ax = plt.subplots(figsize=(9, 5.5))
    bins = min(50, max(10, n // 10))
    ax.hist(lat_arr, bins=bins, color="#4C72B0", alpha=0.75, label=f"all (n={n})")
    if len(on_lat) and len(off_lat):
        ax.hist(on_lat, bins=bins, color="#55A868", alpha=0.5, label=f"OFF->ON (n={len(on_lat)})")
        ax.hist(off_lat, bins=bins, color="#C44E52", alpha=0.5, label=f"ON->OFF (n={len(off_lat)})")
    ax.set_xlabel("Command -> detection latency (ms)")
    ax.set_ylabel("Count")
    title_tag = "random pre-command delay" if random_delay_enabled else "immediate re-command"
    ax.set_title(f"LED (GPIO{led_pin}) toggle-to-camera-detection latency, n={n}, "
                 f"{test_duration_s:.0f}s window, {title_tag}")
    # Pinned to opposite corners (not ax.legend()'s auto-placed "best",
    # which picked upper-right in at least one real run and collided with
    # the stats box also at upper-right -- found live, both became
    # illegible) so the two never fight for the same space regardless of
    # the data's own shape.
    ax.legend(loc="upper left")
    ax.grid(alpha=0.3)

    lat_sorted = sorted(lat_ms)

    def pct(p):
        return lat_sorted[min(int(p * n), n - 1)]

    stats_text = (
        f"mean={statistics.mean(lat_ms):.2f}ms  median={statistics.median(lat_ms):.2f}ms\n"
        f"p95={pct(0.95):.2f}ms  p99={pct(0.99):.2f}ms  max={lat_sorted[-1]:.2f}ms\n"
        f"timeouts={timeouts}"
    )
    ax.text(0.98, 0.97, stats_text, transform=ax.transAxes, ha="right", va="top",
            fontsize=9, bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))
    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    plt.close(fig)


# ── Args ─────────────────────────────────────────────────────────────────
if len(sys.argv) > 2 and sys.argv[1] == "--replot":
    # Regenerate a PNG from an already-saved npz, no camera/GPIO touched at
    # all -- e.g. after a plot-layout fix, so every past run's image can be
    # refreshed without re-running the hardware test.
    _npz_path = sys.argv[2]
    _d = np.load(_npz_path)
    _lat_ms = list(_d["latencies_s"] * 1000)
    _png_path = _npz_path.rsplit(".", 1)[0] + ".png"
    if "test_duration_s" in _d:
        _test_duration_s = float(_d["test_duration_s"])
    elif len(_lat_ms):
        # Older npz files (saved before test_duration_s was recorded
        # directly) -- approximate from the real command/detect span
        # instead, close enough for the title text, never used for math.
        _test_duration_s = float(_d["detect_times"][-1] - _d["command_times"][0])
    else:
        _test_duration_s = 0.0
    plot_histogram(
        lat_ms=_lat_ms,
        directions=_d["directions"],
        timeouts=int(_d["timeouts"]),
        n=len(_lat_ms),
        test_duration_s=_test_duration_s,
        led_pin=int(_d["led_pin"]),
        random_delay_enabled=bool(_d["random_delay_enabled"]) if "random_delay_enabled" in _d else False,
        png_path=_png_path,
    )
    print(f"Replotted {_npz_path} -> {_png_path}")
    raise SystemExit(0)

TEST_DURATION_S = float(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_DURATION_S
if len(sys.argv) > 2:
    FORCE_FRAME_DURATION_US = int(sys.argv[2])  # overrides the module-level
                                                    # default above -- added
                                                    # to test whether the
                                                    # histogram's two-peak
                                                    # separation tracks the
                                                    # frame period (it does,
                                                    # see CLAUDE.md)
if len(sys.argv) > 3:
    _w, _h = sys.argv[3].lower().split("x")
    RAW_SIZE = (int(_w), int(_h))  # overrides the module-level default
                                      # above -- lets this script be rerun
                                      # at any of this project's other
                                      # validated raw sizes without editing
                                      # the file
RANDOM_DELAY_ENABLED = len(sys.argv) > 4 and sys.argv[4] == "random"
# Default (no 4th arg, or anything other than "random"): each new command
# fires immediately after detecting the previous toggle -- this ties
# command timing to the camera's own clock (confirmed: produces the sharp
# two-peak latency structure documented in CLAUDE.md), which is a real
# artifact of THIS TEST LOOP's structure, not necessarily representative of
# how an unpredictable real-world disturbance (the actual 10-20Hz beacon
# wobble this project targets) would land relative to the camera's frame
# clock. With "random" as the 4th arg, a random delay (see
# RANDOM_DELAY_PERIODS) is inserted before each command instead of firing
# immediately -- decorrelates command phase from the camera's clock, giving
# a more representative "how long after an arbitrary real event does the
# camera first see it" distribution.

# ── Setup ────────────────────────────────────────────────────────────────
gpio = lgpio.gpiochip_open(GPIOCHIP)
lgpio.gpio_claim_output(gpio, LED_PIN, 0)

cam = Picamera2(0)
config = cam.create_video_configuration(
    main={"size": (64, 48), "format": "RGB888"},  # required by the API,
                                                     # never actually pulled
    raw={"size": RAW_SIZE, "format": RAW_FORMAT},
    buffer_count=2,
)
cam.configure(config)
cam.start()

min_us, max_us, default_us = cam.camera_controls["FrameDurationLimits"]
print(f"FrameDurationLimits supported range: {min_us}-{max_us}us (default {default_us}us)")
print(f"Requesting frame duration: {FORCE_FRAME_DURATION_US}us "
      f"(~{1_000_000/FORCE_FRAME_DURATION_US:.1f}fps)\n")

controls = {
    "FrameDurationLimits": (FORCE_FRAME_DURATION_US, FORCE_FRAME_DURATION_US),
    "AeEnable": False,
    "NoiseReductionMode": 0,
    "ExposureTime": EXPOSURE_US,
    "AnalogueGain": ANALOGUE_GAIN,
}
unsupported = [k for k in controls if k not in cam.camera_controls]
for k in unsupported:
    print(f"  (skipping control not advertised on this sensor: {k})")
    del controls[k]
cam.set_controls(controls)
time.sleep(1.0)  # let exposure/frame-duration settle before anything below
                   # trusts a captured frame


def set_led(state: int):
    lgpio.gpio_write(gpio, LED_PIN, state)


def capture_mean_frame(n):
    """Average n consecutive raw captures -- cuts per-frame sensor noise
    out of the calibration pass without needing a longer single exposure."""
    acc = None
    for _ in range(n):
        # capture_array("raw") returns the buffer as flat uint8 bytes, but
        # this pipeline always delivers raw frames as 16-bit-per-pixel words
        # (Picamera2 negotiates "R16" even when "R8" is requested, confirmed
        # directly: without .view() this comes back shape (400,1280) uint8,
        # not (400,640) uint16) -- .view() reinterprets consecutive byte
        # pairs as the real pixel values instead of interleaved data/padding
        # bytes. See camera_preview.py's docstring for the same finding.
        f = cam.capture_array("raw").view(np.uint16).astype(np.float64)
        acc = f if acc is None else acc + f
    return acc / n


# ── Auto-calibrate ROI + THRESHOLD ──────────────────────────────────────
print(f"Auto-calibrating ROI/threshold for GPIO{LED_PIN} "
      f"({CALIB_FRAMES} frames/state)...")

set_led(0)
time.sleep(CALIB_SETTLE_S)
off_frame = capture_mean_frame(CALIB_FRAMES)

set_led(1)
time.sleep(CALIB_SETTLE_S)
on_frame = capture_mean_frame(CALIB_FRAMES)

set_led(0)  # park off while we finish setting up

diff = on_frame - off_frame
peak_y, peak_x = np.unravel_index(np.argmax(diff), diff.shape)

half = ROI_BOX_SIZE // 2
h, w = off_frame.shape
rx = int(np.clip(peak_x - half, 0, w - ROI_BOX_SIZE))
ry = int(np.clip(peak_y - half, 0, h - ROI_BOX_SIZE))
rw = rh = ROI_BOX_SIZE
ROI = (rx, ry, rw, rh)

off_mean = float(off_frame[ry:ry+rh, rx:rx+rw].mean())
on_mean = float(on_frame[ry:ry+rh, rx:rx+rw].mean())
THRESHOLD = (off_mean + on_mean) / 2

print(f"  peak diff pixel: (x={peak_x}, y={peak_y}), diff={diff[peak_y, peak_x]:.2f}")
print(f"  ROI: {ROI}")
print(f"  OFF mean={off_mean:.2f}  ON mean={on_mean:.2f}  THRESHOLD={THRESHOLD:.2f}")

if off_mean <= 0 or (on_mean - off_mean) / max(off_mean, 1e-9) < MIN_SEPARATION:
    print(f"\nWARNING: ON/OFF separation is poor ({off_mean:.2f} -> {on_mean:.2f}) "
          f"-- check the LED is actually in frame, wiring, exposure/gain, and "
          f"ambient light before trusting results from this run.\n")
else:
    print("  Separation looks good.\n")

# Debug snapshot -- same purpose as led_calibrate_raw.py's roi_debug_raw.png,
# confirms after the fact where the auto-detected box landed in the real
# scene. Rendered from off_frame, NOT on_frame: this physical setup floods
# the ENTIRE sensor to full saturation (65280, confirmed: on_frame.min() ==
# on_frame.max() == 65280, every single pixel) when the LED is on, rather
# than showing a localized bright blob -- a uniformly-saturated frame has
# no spatial structure left to normalize/display at all, so on_frame would
# just render as flat/blank. off_frame still has real background texture,
# so it's the only one of the two actually useful for confirming the ROI
# box's real-scene location. This doesn't affect the timing measurement
# itself -- see the module docstring's note on why full-frame flooding is
# if anything a MORE robust toggle signal than a small point source would
# be, just not a visually distinct "LED" to box in a debug image.
debug = np.empty_like(off_frame, dtype=np.uint8)
cv_norm_src = off_frame.astype(np.uint16)
import cv2  # local import -- only this one spot in the script needs cv2,
             # the rest is plain numpy/matplotlib
cv2.normalize(cv_norm_src, debug, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
debug_bgr = cv2.cvtColor(debug, cv2.COLOR_GRAY2BGR)
cv2.rectangle(debug_bgr, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
cv2.imwrite("led_timing_histogram_roi_debug.png", debug_bgr)
print("Saved led_timing_histogram_roi_debug.png (OFF-state frame, since ON "
      "saturates the whole sensor -- see script comment) -- confirm the "
      "green box's position looks reasonable in the real scene.\n")


def brightness() -> float:
    frame = cam.capture_array("raw").view(np.uint16)  # see capture_mean_frame's
                                                          # comment above -- same
                                                          # byte-interleaving fix,
                                                          # needed on every raw
                                                          # capture in this script
    return float(np.mean(frame[ry:ry+rh, rx:rx+rw]))


def is_on(b: float) -> bool:
    return b > THRESHOLD


# ── Closed-loop toggle test -- keep every sample ────────────────────────
latencies_s = []      # command -> detection, seconds, one per toggle
command_times = []    # absolute monotonic time each toggle was commanded
detect_times = []     # absolute monotonic time each toggle was detected
directions = []        # 1 = commanded ON, 0 = commanded OFF -- kept so
                        # ON-latency and OFF-latency can be compared
                        # separately, not just pooled
timeouts = 0

RANDOM_DELAY_MAX_S = RANDOM_DELAY_PERIODS * FORCE_FRAME_DURATION_US / 1e6

target = 1  # start by commanding ON
set_led(target)
command_time = time.monotonic()

test_start = time.monotonic()
mode_tag = (f"RANDOM pre-command delay enabled (0-{RANDOM_DELAY_MAX_S*1000:.2f}ms, "
            f"{RANDOM_DELAY_PERIODS}x frame period) -- decorrelating command "
            f"phase from the camera's clock" if RANDOM_DELAY_ENABLED else
            "immediate re-command (default) -- command phase stays "
            "entrained to the camera's own clock, see CLAUDE.md")
print(f"Running closed-loop toggle test for {TEST_DURATION_S}s, "
      f"threshold={THRESHOLD:.2f}, keeping every sample")
print(f"  {mode_tag}\n")

try:
    while True:
        now = time.monotonic()
        if now - test_start >= TEST_DURATION_S:
            break

        b = brightness()
        detected = is_on(b)

        if detected == bool(target):
            detect_time = time.monotonic()
            latency = detect_time - command_time
            latencies_s.append(latency)
            command_times.append(command_time)
            detect_times.append(detect_time)
            directions.append(target)

            target ^= 1
            if RANDOM_DELAY_ENABLED:
                # Deliberately AFTER stamping detect_time/logging the
                # latency above, so this delay never contaminates the
                # measurement itself -- only command_time (stamped below,
                # after the delay) is affected, which is exactly the point:
                # the NEXT latency sample should be measured from a command
                # whose phase relative to the camera's clock is random, not
                # whatever phase happens to follow immediately after a
                # detection.
                time.sleep(random.uniform(0, RANDOM_DELAY_MAX_S))
            set_led(target)
            command_time = time.monotonic()

        elif now - command_time > TIMEOUT_S:
            timeouts += 1
            print(f"  WARNING: timed out waiting for {'ON' if target else 'OFF'} "
                  f"(last brightness={b:.1f}) -- forcing flip")
            target ^= 1
            set_led(target)
            command_time = time.monotonic()

except KeyboardInterrupt:
    pass

finally:
    n = len(latencies_s)
    lat_ms = sorted(x * 1000 for x in latencies_s)

    def pct(p):
        # Only ever called below when n > 0 -- both call sites are guarded
        # by "if n:".
        return lat_ms[min(int(p * n), n - 1)]

    print("\n── Summary ──────────────────────────────")
    print(f"  Toggles detected: {n}  (timeouts: {timeouts})")
    if n:
        print(f"  Latency (ms): mean={statistics.mean(lat_ms):.3f}  "
              f"median={statistics.median(lat_ms):.3f}  "
              f"std={statistics.pstdev(lat_ms) if n > 1 else 0.0:.3f}  "
              f"min={lat_ms[0]:.3f}  p95={pct(0.95):.3f}  "
              f"p99={pct(0.99):.3f}  max={lat_ms[-1]:.3f}")

    if len(detect_times) > 1:
        periods = [detect_times[i+1] - detect_times[i] for i in range(len(detect_times)-1)]
        mean_p = statistics.mean(periods)
        freq = 1.0 / mean_p if mean_p else float("nan")
        print(f"  Effective closed-loop toggle frequency: {freq:.2f} Hz "
              f"(full ON+OFF cycle: {2*mean_p*1000:.1f} ms)")
    print("─────────────────────────────────────────")

    cam.stop()
    set_led(0)
    lgpio.gpiochip_close(gpio)

    if n:
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        tag = "_random" if RANDOM_DELAY_ENABLED else ""
        npz_path = f"results/led_timing_histogram{tag}_{ts}.npz"
        png_path = f"results/led_timing_histogram{tag}_{ts}.png"

        np.savez(
            npz_path,
            latencies_s=np.array(latencies_s),
            command_times=np.array(command_times),
            detect_times=np.array(detect_times),
            directions=np.array(directions),
            timeouts=timeouts,
            led_pin=LED_PIN,
            roi=np.array(ROI),
            threshold=THRESHOLD,
            raw_size=np.array(RAW_SIZE),
            frame_duration_us=FORCE_FRAME_DURATION_US,
            exposure_us=EXPOSURE_US,
            analogue_gain=ANALOGUE_GAIN,
            random_delay_enabled=RANDOM_DELAY_ENABLED,
            test_duration_s=TEST_DURATION_S,
        )
        print(f"Saved raw samples: {npz_path}")

        plot_histogram(
            lat_ms=lat_ms,
            directions=directions,
            timeouts=timeouts,
            n=n,
            test_duration_s=TEST_DURATION_S,
            led_pin=LED_PIN,
            random_delay_enabled=RANDOM_DELAY_ENABLED,
            png_path=png_path,
        )
        print(f"Saved histogram: {png_path}")
    else:
        print("No samples collected -- nothing saved.")
