#!/usr/bin/env python3
"""
Closed-loop dual-camera LED timing test -- single shared LED.

Same structure as the single-camera closed-loop tests: command a state,
wait for detection, flip, repeat as fast as possible. The difference is
the wait condition -- each cycle waits until BOTH cameras have
independently confirmed the new state before flipping again. That gives:

  - per-camera latency (command -> that camera's own detection), same
    stats as the single-camera tests, for direct comparison
  - skew per transition (cam0's detection time - cam1's detection time),
    measured directly against a shared command timestamp instead of
    correlated afterward from two free-running logs

Each camera runs its own capture thread, continuously checking incoming
frames against the current shared target state. The main thread drives
the LED and blocks until both per-camera "detected" events fire (or a
per-transition timeout), then immediately flips and continues.

CAVEAT: both capture threads run in the same process under the GIL --
this measures latency/skew with that threading overhead included. If
numbers look unexpectedly large or noisy compared to the single-camera
baselines, that's the first thing to suspect; the fix would be splitting
into two separate processes.

Install:  pip install lgpio numpy
          (picamera2 is pre-installed on RPi OS Bookworm)
"""

import threading
import time
import statistics
import lgpio
import numpy as np
from picamera2 import Picamera2

# ── Config ────────────────────────────────────────────────────────────────────
LED_PIN          = 14
GPIOCHIP         = 4

ROI_CAM0         = (0, 0, 640, 400)   # full frame -- dark box means no background
ROI_CAM1         = (0, 0, 640, 400)   # to dilute the mean, so this is safe as the real setting

RAW_SIZE         = (640, 400)
RAW_FORMAT       = "R8"
FORCE_FRAME_DURATION_US = 6000
EXPOSURE_US      = 1500
ANALOGUE_GAIN    = 4.0

TEST_DURATION_S  = 5.0
TIMEOUT_S        = 1.0      # per-transition safety: force flip if either/both cameras never confirm
POLL_SLEEP_S     = 0.0001   # main-thread wait granularity while polling for both detections

# ── Camera discovery ─────────────────────────────────────────────────────────
print("Detected cameras:")
for info in Picamera2.global_camera_info():
    print(f"  {info}")
print()

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
    }
    if EXPOSURE_US is not None:
        controls["ExposureTime"] = EXPOSURE_US
    if ANALOGUE_GAIN is not None:
        controls["AnalogueGain"] = ANALOGUE_GAIN
    unsupported = [k for k in controls if k not in cam.camera_controls]
    for k in unsupported:
        del controls[k]
    cam.set_controls(controls)
    return cam


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

# ── Shared state between the scheduler (main thread) and capture threads ───
target = 0                 # current commanded state, written only by main thread
detected = [threading.Event(), threading.Event()]
detect_time = [None, None]
running = True


def capture_loop(index, cam, roi, threshold):
    rx, ry, rw, rh = roi
    while running:
        frame = cam.capture_array("raw")
        t = time.monotonic()
        b = float(np.mean(frame[ry:ry+rh, rx:rx+rw]))
        state = b > threshold
        if state == bool(target) and not detected[index].is_set():
            detect_time[index] = t
            detected[index].set()


t0 = threading.Thread(target=capture_loop, args=(0, cam0, ROI_CAM0, threshold0))
t1 = threading.Thread(target=capture_loop, args=(1, cam1, ROI_CAM1, threshold1))
t0.start()
t1.start()

# ── Closed-loop scheduler: command -> wait for BOTH -> flip -> repeat ──────
lat0_list, lat1_list, skew_list = [], [], []
command_times = []
timeouts = 0
both_timeouts = 0

print(f"Running {TEST_DURATION_S}s closed-loop dual-camera test\n")
test_start = time.monotonic()

try:
    while time.monotonic() - test_start < TEST_DURATION_S:
        target ^= 1
        detected[0].clear()
        detected[1].clear()
        detect_time[0] = None
        detect_time[1] = None
        set_led(target)
        command_time = time.monotonic()
        command_times.append(command_time)

        deadline = command_time + TIMEOUT_S
        while not (detected[0].is_set() and detected[1].is_set()):
            if time.monotonic() > deadline:
                break
            time.sleep(POLL_SLEEP_S)

        if detected[0].is_set() and detected[1].is_set():
            lat0 = (detect_time[0] - command_time) * 1000.0
            lat1 = (detect_time[1] - command_time) * 1000.0
            skew = (detect_time[0] - detect_time[1]) * 1000.0
            lat0_list.append(lat0)
            lat1_list.append(lat1)
            skew_list.append(skew)
        else:
            timeouts += 1
            if not detected[0].is_set() and not detected[1].is_set():
                both_timeouts += 1

except KeyboardInterrupt:
    pass

finally:
    running = False
    t0.join()
    t1.join()
    set_led(0)

    print("── Summary ──────────────────────────────")
    n = len(skew_list)
    print(f"  Transitions with both cameras confirming: {n}  "
          f"(timeouts: {timeouts}, of which both-missed: {both_timeouts})")

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

    cam0.stop()
    cam1.stop()
    lgpio.gpiochip_close(gpio)
