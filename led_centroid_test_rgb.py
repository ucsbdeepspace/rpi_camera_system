#!/usr/bin/env python3
"""
Closed-loop LED control timing test -- RGB888 capture + centroid variant.

Counterpart to led_centroid_test_raw.py, using the ISP-processed RGB888
main stream instead of the raw R8 stream. Same instrumentation: capture
time and compute time (brightness + centroid) are timed separately every
frame via time.perf_counter(), so the summary reports the compute-share
percentage the same way the raw version does -- this is the number to
compare against the raw run's 1.94% to see whether the centroid math
itself is more expensive on this stream (e.g. due to the extra channel
dimension) or whether it's still negligible either way.

ON/OFF detection still uses ROI mean brightness vs THRESHOLD (averaged
across the 3 color channels, matching the original RGB888 calibration
script's approach) -- not a separate measurement from the centroid.

THRESHOLD/EXPOSURE_US/ANALOGUE_GAIN/FORCE_FRAME_DURATION_US carried over
from led_timing_test3.py's matched-duration comparison run (6000us).
Recalibrate if ROI, exposure, gain, or lighting conditions change.

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
FRAME_SIZE       = (640, 400)  # matches the sensor's native raw mode -- avoids an
                                # extra ISP scaling step that (640, 480) requires
THRESHOLD        = 5.0         # ROI mean brightness (averaged across RGB channels)
TEST_DURATION_S  = 5.0         # total test window
TIMEOUT_S        = 1.0         # safety: max wait for a single detection
EXPOSURE_US      = 5000
ANALOGUE_GAIN    = 4.0
FORCE_FRAME_DURATION_US = 6000  # same value used for the matched raw-vs-RGB888 comparison

# ── Setup ─────────────────────────────────────────────────────────────────────
gpio = lgpio.gpiochip_open(GPIOCHIP)
lgpio.gpio_claim_output(gpio, LED_PIN, 0)

cam = Picamera2(0)

# buffer_count=2 keeps the capture queue shallow so capture_array() returns
# the most recent frame rather than one that was already in flight before
# the LED was toggled.
cam.configure(cam.create_video_configuration(
    main={"size": FRAME_SIZE, "format": "RGB888"}, buffer_count=2))
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

# Precomputed ONCE, outside the loop -- same reasoning as the raw version.
xs, ys = np.meshgrid(np.arange(rw, dtype=np.float64),
                      np.arange(rh, dtype=np.float64))


def set_led(state: int):
    lgpio.gpio_write(gpio, LED_PIN, state)


def measure():
    """
    Capture one RGB888 frame and compute both ROI mean brightness (channel-
    averaged, for ON/OFF detection) and the intensity-weighted centroid
    within the ROI. Returns (brightness, cx, cy, capture_s, compute_s).
    """
    t0 = time.perf_counter()
    frame = cam.capture_array()
    t1 = time.perf_counter()

    roi = frame[ry:ry+rh, rx:rx+rw].astype(np.float64).mean(axis=2)  # collapse R/G/B
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
print("RGB888-stream calibration check:")
set_led(1)
time.sleep(0.2)
on_b, on_cx, on_cy, _, _ = measure()
print(f"  LED ON  brightness: {on_b:.2f}  centroid=({on_cx:.1f}, {on_cy:.1f}) within ROI")
set_led(0)
time.sleep(0.2)
off_b, off_cx, off_cy, _, _ = measure()
print(f"  LED OFF brightness: {off_b:.2f}  centroid=({off_cx:.1f}, {off_cy:.1f}) within ROI "
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
