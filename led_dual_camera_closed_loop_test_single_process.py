#!/usr/bin/env python3
"""
Closed-loop dual-camera LED timing test -- SINGLE PROCESS, SINGLE THREAD.

Same measurement and same settings as the multiprocess version settled on
(3400us frame duration, 1500us exposure, gain 4.0, full-frame ROI, both
cameras must confirm before flipping) -- but both cameras are opened in
one process and polled sequentially in one loop, no threading or
multiprocessing at all. This exists to answer: how much did going
multiprocess actually buy us, apples-to-apples, at the best known
settings?

Expectation going in: sequential capture_array() calls block one after
another, so each loop iteration pays cam0's wait AND cam1's wait
back-to-back instead of overlapping them. At the achieved ~282fps/3.5ms
actual frame period from the multiprocess run, naive stacking predicts
roughly double the latency and roughly half the frequency of the
concurrent version -- this run tests whether that prediction holds.

Install:  pip install lgpio numpy
          (picamera2 is pre-installed on RPi OS Bookworm)
"""

import sys
import time
import statistics
import lgpio
import numpy as np
from picamera2 import Picamera2

# ── Config ────────────────────────────────────────────────────────────────────
LED_PIN          = 14
GPIOCHIP         = 4

ROI_CAM0         = (0, 0, 640, 400)
ROI_CAM1         = (0, 0, 640, 400)

RAW_SIZE         = (640, 400)
RAW_FORMAT       = "R8"
FORCE_FRAME_DURATION_US = int(sys.argv[1]) if len(sys.argv) > 1 else 3400  # best value found
EXPOSURE_US      = 1500
ANALOGUE_GAIN    = 4.0

TEST_DURATION_S  = 5.0
TIMEOUT_S        = 1.0

print(f"FORCE_FRAME_DURATION_US={FORCE_FRAME_DURATION_US}  (single process, single thread)")

# ── Setup ─────────────────────────────────────────────────────────────────────
gpio = lgpio.gpiochip_open(GPIOCHIP)
lgpio.gpio_claim_output(gpio, LED_PIN, 0)


def make_camera(index):
    cam = Picamera2(index)
    config = cam.create_video_configuration(
        main={"size": (64, 48), "format": "RGB888"},
        raw={"size": RAW_SIZE, "format": RAW_FORMAT},
        buffer_count=2,
    )
    cam.configure(config)
    cam.start()

    controls = {
        "FrameDurationLimits": (FORCE_FRAME_DURATION_US, FORCE_FRAME_DURATION_US),
        "AeEnable": False,
        "NoiseReductionMode": 0,
        "ExposureTime": EXPOSURE_US,
        "AnalogueGain": ANALOGUE_GAIN,
    }
    unsupported = [k for k in controls if k not in cam.camera_controls]
    for k in unsupported:
        del controls[k]
    cam.set_controls(controls)
    return cam


print("Opening both cameras sequentially in one process...")
cam0 = make_camera(0)
cam1 = make_camera(1)
time.sleep(1.0)


def set_led(state: int):
    lgpio.gpio_write(gpio, LED_PIN, state)


def roi_brightness(cam, roi):
    rx, ry, rw, rh = roi
    frame = cam.capture_array("raw")
    return float(np.mean(frame[ry:ry+rh, rx:rx+rw]))


# ── Per-camera calibration ──────────────────────────────────────────────────
print("Per-camera calibration check:")
set_led(1)
time.sleep(0.2)
on0, on1 = roi_brightness(cam0, ROI_CAM0), roi_brightness(cam1, ROI_CAM1)
set_led(0)
time.sleep(0.2)
off0, off1 = roi_brightness(cam0, ROI_CAM0), roi_brightness(cam1, ROI_CAM1)

threshold0 = (on0 + off0) / 2.0
threshold1 = (on1 + off1) / 2.0
print(f"  cam0: ON={on0:.2f} OFF={off0:.2f} threshold={threshold0:.2f} "
      f"({'ok' if off0 < threshold0 < on0 else 'POOR SEPARATION'})")
print(f"  cam1: ON={on1:.2f} OFF={off1:.2f} threshold={threshold1:.2f} "
      f"({'ok' if off1 < threshold1 < on1 else 'POOR SEPARATION'})\n")

rx0, ry0, rw0, rh0 = ROI_CAM0
rx1, ry1, rw1, rh1 = ROI_CAM1

# ── Closed-loop scheduler -- sequential polling, no concurrency at all ─────
lat0_list, lat1_list, skew_list = [], [], []
command_times = []
cam0_frame_count = 0
cam1_frame_count = 0
timeouts = 0

target = 0
print(f"Running {TEST_DURATION_S}s closed-loop single-process test\n")
test_start = time.monotonic()

try:
    while time.monotonic() - test_start < TEST_DURATION_S:
        target ^= 1
        set_led(target)
        command_time = time.monotonic()
        command_times.append(command_time)

        detected0 = detected1 = False
        detect_time0 = detect_time1 = None
        deadline = command_time + TIMEOUT_S

        while not (detected0 and detected1):
            if time.monotonic() > deadline:
                break

            if not detected0:
                frame = cam0.capture_array("raw")
                t = time.monotonic()
                cam0_frame_count += 1
                b = float(np.mean(frame[ry0:ry0+rh0, rx0:rx0+rw0]))
                if (b > threshold0) == bool(target):
                    detected0 = True
                    detect_time0 = t

            if not detected1:
                frame = cam1.capture_array("raw")
                t = time.monotonic()
                cam1_frame_count += 1
                b = float(np.mean(frame[ry1:ry1+rh1, rx1:rx1+rw1]))
                if (b > threshold1) == bool(target):
                    detected1 = True
                    detect_time1 = t

        if detected0 and detected1:
            lat0_list.append((detect_time0 - command_time) * 1000.0)
            lat1_list.append((detect_time1 - command_time) * 1000.0)
            skew_list.append((detect_time0 - detect_time1) * 1000.0)
        else:
            timeouts += 1

except KeyboardInterrupt:
    pass

finally:
    test_end = time.monotonic()
    set_led(0)
    cam0.stop()
    cam1.stop()
    lgpio.gpiochip_close(gpio)

    print("── Summary ──────────────────────────────")
    n = len(skew_list)
    print(f"  Transitions with both cameras confirming: {n}  (timeouts: {timeouts})")

    elapsed = test_end - test_start
    print(f"  cam0 achieved during closed loop: {cam0_frame_count} frames in {elapsed:.2f}s  ->  "
          f"{cam0_frame_count/elapsed:.2f} fps")
    print(f"  cam1 achieved during closed loop: {cam1_frame_count} frames in {elapsed:.2f}s  ->  "
          f"{cam1_frame_count/elapsed:.2f} fps")

    if n:
        for name, lats in [("cam0", lat0_list), ("cam1", lat1_list)]:
            mean_l = statistics.mean(lats)
            std_l = statistics.pstdev(lats) if n > 1 else 0.0
            print(f"  {name} latency: mean={mean_l:.3f} ms  std={std_l:.3f} ms  "
                  f"min={min(lats):.3f} ms  max={max(lats):.3f} ms")

        mean_skew = statistics.mean(skew_list)
        std_skew = statistics.pstdev(skew_list) if n > 1 else 0.0
        print(f"\n  Inter-camera skew (cam0 - cam1): mean={mean_skew:+.3f} ms  "
              f"std={std_skew:.3f} ms  min={min(skew_list):+.3f} ms  max={max(skew_list):+.3f} ms")
        print(f"  |skew| max: {max(abs(s) for s in skew_list):.3f} ms")

        if len(command_times) > 1:
            periods = [command_times[i+1] - command_times[i] for i in range(len(command_times)-1)]
            mean_p = statistics.mean(periods)
            freq = 1.0 / mean_p if mean_p else float("nan")
            print(f"\n  Effective closed-loop toggle frequency (both-confirmed): {freq:.2f} Hz")
    print("─────────────────────────────────────────")
