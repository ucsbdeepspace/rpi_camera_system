#!/usr/bin/env python3
"""
Closed-loop LED control timing test -- RAW capture + centroid variant.

Same closed-loop ON/OFF detection test as led_timing_test_raw.py, but each
frame also computes the intensity-weighted centroid of the ROI (not just
the mean brightness used for ON/OFF detection). The point of this script
is to answer: does the per-frame numpy centroid math eat into the timing
win the raw stream gave us, or is frame capture still the dominant cost?

Every frame, capture time and compute time are timed separately with
time.perf_counter() and accumulated independently, so the summary reports
them as two distinct numbers instead of one combined latency. ON/OFF
detection still uses ROI mean brightness vs THRESHOLD, same as before --
the centroid is extra work computed alongside it every frame, to measure
realistic overhead if you were to use it for closed-loop position control.

THRESHOLD and EXPOSURE_US/ANALOGUE_GAIN are carried over from
led_timing_test_raw.py's calibration. Recalibrate if ROI, exposure, gain,
or lighting conditions change.

CAUTION: same rules as before -- run in a fresh process, step
FORCE_FRAME_DURATION_US down gradually across separate runs, and if you
hit a "Camera frontend has timed out" error you must restart the process
before retrying (it cannot recover itself).

Install:  pip install lgpio opencv-python numpy
          (picamera2 is pre-installed on RPi OS Bookworm)
"""

import time
import statistics
import lgpio
import numpy as np
from picamera2 import Picamera2

# ── Config ────────────────────────────────────────────────────────────────────
LED_PIN          = 14          # GPIO pin driving the LED (BCM)
GPIOCHIP         = 4           # /dev/gpiochip4 on RPi 5
ROI              = (300, 220, 40, 40)   # (x, y, width, height)
RAW_SIZE         = (640, 400)  # native sensor resolution
RAW_FORMAT       = "R8"        # unpacked 8-bit mono, no bit-unpacking needed in Python
THRESHOLD        = 2.51        # calibrated for raw stream, ROI mean brightness scale
TEST_DURATION_S  = 5.0         # total test window
TIMEOUT_S        = 1.0         # safety: max wait for a single detection
EXPOSURE_US      = 1500
ANALOGUE_GAIN    = 4.0
FORCE_FRAME_DURATION_US = 5000  # ~200fps -- same starting point as led_timing_test_raw.py

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
target_us = FORCE_FRAME_DURATION_US if FORCE_FRAME_DURATION_US is not None else min_us
print(f"FrameDurationLimits supported range: {min_us}-{max_us}us (default {default_us}us)")
print(f"Requesting frame duration: {target_us}us (~{1_000_000/target_us:.1f}fps)\n")

controls = {
    "FrameDurationLimits": (target_us, target_us),
    "AeEnable": False,
    "NoiseReductionMode": 0,
}
if EXPOSURE_US is not None:
    controls["ExposureTime"] = EXPOSURE_US
if ANALOGUE_GAIN is not None:
    controls["AnalogueGain"] = ANALOGUE_GAIN

unsupported = [k for k in controls if k not in cam.camera_controls]
for k in unsupported:
    print(f"  (skipping control not advertised on this sensor: {k})")
    del controls[k]

cam.set_controls(controls)
time.sleep(1.0)

rx, ry, rw, rh = ROI

# Precomputed ONCE, outside the loop -- coordinate grids for the weighted
# centroid. Recomputing these every frame would be exactly the kind of
# avoidable per-frame Python overhead that could eat into the time saved
# by switching to the raw stream.
xs, ys = np.meshgrid(np.arange(rw, dtype=np.float64),
                      np.arange(rh, dtype=np.float64))


def set_led(state: int):
    lgpio.gpio_write(gpio, LED_PIN, state)


def measure():
    """
    Capture one raw frame and compute both ROI mean brightness (for ON/OFF
    detection, same as the non-centroid script) and the intensity-weighted
    centroid within the ROI. Returns (brightness, cx, cy, capture_s, compute_s)
    with capture_s/compute_s timed separately via time.perf_counter().
    """
    t0 = time.perf_counter()
    frame = cam.capture_array("raw")
    t1 = time.perf_counter()

    roi = frame[ry:ry+rh, rx:rx+rw].astype(np.float64)
    total = roi.sum()
    brightness = total / roi.size

    if total > 0:
        cx = float((roi * xs).sum() / total)
        cy = float((roi * ys).sum() / total)
    else:
        cx = cy = float("nan")

    t2 = time.perf_counter()
    return brightness, cx, cy, (t1 - t0), (t2 - t1)


def is_on(b: float) -> bool:
    return b > THRESHOLD


# ── Calibration check before the timed test ───────────────────────────────────
print("Raw-stream calibration check:")
set_led(1)
time.sleep(0.2)
on_b, on_cx, on_cy, _, _ = measure()
print(f"  LED ON  raw brightness: {on_b:.2f}  centroid=({on_cx:.1f}, {on_cy:.1f}) within ROI")
set_led(0)
time.sleep(0.2)
off_b, off_cx, off_cy, _, _ = measure()
print(f"  LED OFF raw brightness: {off_b:.2f}  centroid=({off_cx:.1f}, {off_cy:.1f}) within ROI "
      f"(meaningless when LED is off -- just confirms the math runs)")
print(f"  Current THRESHOLD={THRESHOLD}  "
      f"({'looks fine' if off_b < THRESHOLD < on_b else 'ADJUST THRESHOLD before trusting results'})\n")

# ── Closed-loop toggle test ───────────────────────────────────────────────────
latencies = []
detect_times = []
capture_times = []   # seconds, every frame
compute_times = []   # seconds, every frame
on_centroids = []    # (cx, cy) recorded at each successful ON detection
timeouts = 0

target = 1
set_led(target)
command_time = time.monotonic()
test_start = time.monotonic()
print(f"Running closed-loop toggle + centroid test for {TEST_DURATION_S}s, threshold={THRESHOLD}\n")

try:
    while True:
        now = time.monotonic()
        if now - test_start >= TEST_DURATION_S:
            break

        b, cx, cy, cap_dt, comp_dt = measure()
        capture_times.append(cap_dt)
        compute_times.append(comp_dt)
        detected = is_on(b)

        if detected == bool(target):
            detect_time = time.monotonic()
            latency = detect_time - command_time
            latencies.append(latency)
            detect_times.append(detect_time)
            if target == 1:
                on_centroids.append((cx, cy))

            target ^= 1
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
    print("\n── Summary ──────────────────────────────")
    n = len(latencies)
    print(f"  Toggles detected: {n}  (timeouts: {timeouts})")
    if n:
        mean_lat = statistics.mean(latencies)
        std_lat = statistics.pstdev(latencies) if n > 1 else 0.0
        print(f"  Per-transition latency: mean={mean_lat*1000:.1f} ms  std={std_lat*1000:.1f} ms  "
              f"min={min(latencies)*1000:.1f} ms  max={max(latencies)*1000:.1f} ms")

    if len(detect_times) > 1:
        periods = [detect_times[i+1] - detect_times[i] for i in range(len(detect_times)-1)]
        mean_p = statistics.mean(periods)
        std_p = statistics.pstdev(periods) if len(periods) > 1 else 0.0
        freq = 1.0 / mean_p if mean_p else float("nan")
        print(f"  Inter-detection period: mean={mean_p*1000:.1f} ms  std={std_p*1000:.1f} ms")
        print(f"  Effective closed-loop toggle frequency: {freq:.2f} Hz "
              f"(full ON+OFF cycle: {2*mean_p*1000:.1f} ms)")

    if capture_times:
        nf = len(capture_times)
        mean_cap = statistics.mean(capture_times) * 1000
        std_cap = (statistics.pstdev(capture_times) if nf > 1 else 0.0) * 1000
        mean_comp = statistics.mean(compute_times) * 1000
        std_comp = (statistics.pstdev(compute_times) if nf > 1 else 0.0) * 1000
        total_cap = sum(capture_times) * 1000
        total_comp = sum(compute_times) * 1000
        comp_share = 100.0 * total_comp / (total_cap + total_comp) if (total_cap + total_comp) else 0.0
        print(f"\n  Per-frame timing breakdown ({nf} frames):")
        print(f"    Capture: mean={mean_cap:.3f} ms  std={std_cap:.3f} ms")
        print(f"    Compute (brightness+centroid): mean={mean_comp:.4f} ms  std={std_comp:.4f} ms")
        print(f"    Compute share of total per-frame time: {comp_share:.2f}%")

    if on_centroids:
        cxs = [c[0] for c in on_centroids]
        cys = [c[1] for c in on_centroids]
        print(f"\n  ON-state centroid (within {rw}x{rh} ROI, n={len(on_centroids)}):")
        print(f"    x: mean={statistics.mean(cxs):.2f}  std={statistics.pstdev(cxs):.2f}")
        print(f"    y: mean={statistics.mean(cys):.2f}  std={statistics.pstdev(cys):.2f}")
    print("─────────────────────────────────────────")

    cam.stop()
    set_led(0)
    lgpio.gpiochip_close(gpio)
