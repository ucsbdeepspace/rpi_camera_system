#!/usr/bin/env python3
"""
Exposure-time sweep -- RAW stream.

Runs the closed-loop toggle test at a fixed frame duration but across a
range of EXPOSURE_US values, to characterize the latency/frequency vs.
exposure tradeoff fairly -- same exposure list as led_exposure_sweep_rgb.py,
same frame duration, same gain, so the two streams can be compared on
equal footing at every exposure point instead of cherry-picked settings.

At each exposure step:
  1. Set ExposureTime, let it settle.
  2. Take one ON/OFF calibration sample, compute a per-exposure THRESHOLD
     as the midpoint (brightness scale shifts with exposure, so a single
     fixed threshold across the whole sweep would be wrong).
  3. Run the closed-loop toggle test for SWEEP_TEST_DURATION_S seconds.
  4. Record exposure, calibration samples, threshold, and all timing
     stats to a row in the output CSV.

FORCE_FRAME_DURATION_US is fixed for the whole sweep -- only exposure
varies. Exposures are kept comfortably below the frame duration so each
one is achievable without exceeding the frame period.

CAUTION: this only sweeps exposure, not frame duration, so it does not
carry the same "Camera frontend has timed out" risk that pushing
FrameDurationLimits toward the sensor's floor does. Still run in a fresh
process per the usual caution, and if that error does appear, you must
restart the process -- it cannot recover itself.

Install:  pip install lgpio numpy
          (picamera2 is pre-installed on RPi OS Bookworm)
"""

import csv
import time
import statistics
import lgpio
import numpy as np
from picamera2 import Picamera2

# ── Config ────────────────────────────────────────────────────────────────────
LED_PIN          = 14
GPIOCHIP         = 4
ROI              = (300, 220, 40, 40)
RAW_SIZE         = (640, 400)
RAW_FORMAT       = "R8"
ANALOGUE_GAIN    = 4.0
FORCE_FRAME_DURATION_US = 6000   # fixed for the whole sweep -- confirmed stable
                                  # on both pipelines in prior matched-duration runs

# Exposures to sweep, in microseconds. All comfortably below
# FORCE_FRAME_DURATION_US to stay achievable within the frame period.
# IDENTICAL list used in led_exposure_sweep_rgb.py for a fair comparison.
EXPOSURE_SWEEP_US = [500, 1000, 1500, 2000, 3000, 4000, 5000]

SWEEP_TEST_DURATION_S = 3.0   # per-exposure test window (kept shorter than the
                               # earlier 5.0s single-point runs since this repeats
                               # per exposure value)
TIMEOUT_S         = 1.0
SETTLE_S          = 0.3       # after changing ExposureTime, before recalibrating
OUTPUT_CSV        = "exposure_sweep_raw.csv"

# ── Setup ─────────────────────────────────────────────────────────────────────
gpio = lgpio.gpiochip_open(GPIOCHIP)
lgpio.gpio_claim_output(gpio, LED_PIN, 0)

cam = Picamera2(0)
config = cam.create_video_configuration(
    main={"size": (64, 48), "format": "RGB888"},
    raw={"size": RAW_SIZE, "format": RAW_FORMAT},
    buffer_count=2,
)
cam.configure(config)
cam.start()

min_us, max_us, default_us = cam.camera_controls["FrameDurationLimits"]
target_us = FORCE_FRAME_DURATION_US
print(f"FrameDurationLimits supported range: {min_us}-{max_us}us (default {default_us}us)")
print(f"Fixed frame duration for entire sweep: {target_us}us (~{1_000_000/target_us:.1f}fps)")

base_controls = {
    "FrameDurationLimits": (target_us, target_us),
    "AeEnable": False,
    "NoiseReductionMode": 0,
}
if ANALOGUE_GAIN is not None:
    base_controls["AnalogueGain"] = ANALOGUE_GAIN

unsupported = [k for k in base_controls if k not in cam.camera_controls]
for k in unsupported:
    print(f"  (skipping control not advertised on this sensor: {k})")
    del base_controls[k]

cam.set_controls(base_controls)
time.sleep(1.0)

exp_min, exp_max, exp_default = cam.camera_controls.get("ExposureTime", (None, None, None))
if exp_min is not None:
    print(f"ExposureTime supported range: {exp_min}-{exp_max}us (default {exp_default}us)")
    sweep_values = [e for e in EXPOSURE_SWEEP_US if exp_min <= e <= exp_max]
    skipped = [e for e in EXPOSURE_SWEEP_US if e not in sweep_values]
    if skipped:
        print(f"  Skipping out-of-range exposures: {skipped}")
else:
    sweep_values = list(EXPOSURE_SWEEP_US)
print()

rx, ry, rw, rh = ROI


def set_led(state: int):
    lgpio.gpio_write(gpio, LED_PIN, state)


def brightness() -> float:
    frame = cam.capture_array("raw")
    return float(np.mean(frame[ry:ry+rh, rx:rx+rw]))


def run_one_exposure(exposure_us: float) -> dict:
    cam.set_controls({"ExposureTime": int(exposure_us)})
    time.sleep(SETTLE_S)

    set_led(1)
    time.sleep(0.2)
    on_sample = brightness()
    set_led(0)
    time.sleep(0.2)
    off_sample = brightness()

    threshold = (on_sample + off_sample) / 2.0
    valid = off_sample < threshold < on_sample

    def is_on(b):
        return b > threshold

    latencies, detect_times = [], []
    timeouts = 0

    target = 1
    set_led(target)
    command_time = time.monotonic()
    test_start = time.monotonic()

    while True:
        now = time.monotonic()
        if now - test_start >= SWEEP_TEST_DURATION_S:
            break

        b = brightness()
        detected = is_on(b)

        if detected == bool(target):
            detect_time = time.monotonic()
            latencies.append(detect_time - command_time)
            detect_times.append(detect_time)
            target ^= 1
            set_led(target)
            command_time = time.monotonic()
        elif now - command_time > TIMEOUT_S:
            timeouts += 1
            target ^= 1
            set_led(target)
            command_time = time.monotonic()

    set_led(0)

    n = len(latencies)
    mean_lat = statistics.mean(latencies) * 1000 if n else float("nan")
    std_lat = (statistics.pstdev(latencies) if n > 1 else 0.0) * 1000
    min_lat = min(latencies) * 1000 if n else float("nan")
    max_lat = max(latencies) * 1000 if n else float("nan")

    if len(detect_times) > 1:
        periods = [detect_times[i+1] - detect_times[i] for i in range(len(detect_times)-1)]
        mean_p = statistics.mean(periods)
        freq = 1.0 / mean_p if mean_p else float("nan")
    else:
        freq = float("nan")

    return {
        "exposure_us": exposure_us,
        "on_brightness": round(on_sample, 3),
        "off_brightness": round(off_sample, 3),
        "threshold": round(threshold, 3),
        "threshold_valid": valid,
        "toggles": n,
        "timeouts": timeouts,
        "mean_latency_ms": round(mean_lat, 3) if n else "",
        "std_latency_ms": round(std_lat, 3) if n else "",
        "min_latency_ms": round(min_lat, 3) if n else "",
        "max_latency_ms": round(max_lat, 3) if n else "",
        "freq_hz": round(freq, 3) if n > 1 else "",
    }


print(f"Sweeping {len(sweep_values)} exposure values, "
      f"{SWEEP_TEST_DURATION_S}s each (~{len(sweep_values)*SWEEP_TEST_DURATION_S:.0f}s total)\n")

rows = []
try:
    for exposure_us in sweep_values:
        print(f"-- ExposureTime={exposure_us}us --")
        row = run_one_exposure(exposure_us)
        rows.append(row)
        ok = "ok" if row["threshold_valid"] else "POOR SEPARATION"
        print(f"   ON={row['on_brightness']}  OFF={row['off_brightness']}  "
              f"threshold={row['threshold']} ({ok})")
        print(f"   toggles={row['toggles']} timeouts={row['timeouts']}  "
              f"latency mean={row['mean_latency_ms']}ms std={row['std_latency_ms']}ms  "
              f"freq={row['freq_hz']}Hz\n")
except KeyboardInterrupt:
    print("Sweep interrupted -- writing partial results.\n")

finally:
    if rows:
        fieldnames = list(rows[0].keys())
        with open(OUTPUT_CSV, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        print(f"Saved {len(rows)} rows to {OUTPUT_CSV}")
    else:
        print("No rows collected -- nothing written.")

    cam.stop()
    set_led(0)
    lgpio.gpiochip_close(gpio)
