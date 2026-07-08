#!/usr/bin/env python3
"""
Dual-camera closed-loop test -- per-iteration parallel capture via thread pool.

Answers two things:
  1. How long does capture_array() actually take, per camera, per call?
  2. Does parallelizing just the two capture calls (not the whole loop) with
     threads help, versus the fully sequential version?

Each loop iteration submits both cameras' capture+check work to a 2-worker
ThreadPoolExecutor and waits for both results before proceeding -- so the
two ~4-5ms blocking hardware waits can genuinely overlap (capture_array()
releases the GIL while blocked on hardware I/O), without the sustained
free-running thread contention that caused the earlier threaded test's
erratic skew.

NOTE: this uses a persistent thread pool (2 workers, created once, reused
every iteration) rather than literally spawning and joining OS threads on
every pass -- true spawn/teardown overhead every iteration would likely
eat the benefit. This still matches the spirit of "grab both frames in
parallel, don't keep threads running the whole time" without that cost.

Install:  pip install lgpio numpy
          (picamera2 is pre-installed on RPi OS Bookworm)
"""

import time
import statistics
from concurrent.futures import ThreadPoolExecutor
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
FORCE_FRAME_DURATION_US = 4000
EXPOSURE_US      = 1500
ANALOGUE_GAIN    = 4.0

TEST_DURATION_S  = 5.0
TIMEOUT_S        = 1.0

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


# ── Calibration ──────────────────────────────────────────────────────────────
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


def capture_and_check(cam, roi, threshold):
    """Runs in a pool worker. Returns (state, detect_wall_time, capture_duration_s)."""
    rx, ry, rw, rh = roi
    t_req = time.perf_counter()
    frame = cam.capture_array("raw")
    t_got = time.perf_counter()
    detect_wall_time = time.monotonic()
    b = float(np.mean(frame[ry:ry+rh, rx:rx+rw]))
    state = b > threshold
    return state, detect_wall_time, (t_got - t_req)


pool = ThreadPoolExecutor(max_workers=2)

# ── Closed loop, parallel capture per iteration ─────────────────────────────
target = 1
confirmed0 = confirmed1 = False
detect_time0 = detect_time1 = None
set_led(target)
command_time = time.monotonic()

lat0_list, lat1_list, skew_list = [], [], []
cap_dur0_list, cap_dur1_list = [], []
iteration_wall_list = []
command_times = []
timeouts = 0

print(f"Running {TEST_DURATION_S}s dual-camera closed loop, parallel capture via thread pool\n")
test_start = time.monotonic()

try:
    while time.monotonic() - test_start < TEST_DURATION_S:
        iter_start = time.perf_counter()

        fut0 = pool.submit(capture_and_check, cam0, ROI_CAM0, threshold0)
        fut1 = pool.submit(capture_and_check, cam1, ROI_CAM1, threshold1)
        state0, t0, cap_dur0 = fut0.result()
        state1, t1, cap_dur1 = fut1.result()

        iteration_wall_list.append(time.perf_counter() - iter_start)
        cap_dur0_list.append(cap_dur0 * 1000.0)
        cap_dur1_list.append(cap_dur1 * 1000.0)

        if state0 == bool(target) and not confirmed0:
            confirmed0 = True
            detect_time0 = t0
        if state1 == bool(target) and not confirmed1:
            confirmed1 = True
            detect_time1 = t1

        if confirmed0 and confirmed1:
            lat0_list.append((detect_time0 - command_time) * 1000.0)
            lat1_list.append((detect_time1 - command_time) * 1000.0)
            skew_list.append((detect_time0 - detect_time1) * 1000.0)
            command_times.append(command_time)

            target ^= 1
            confirmed0 = confirmed1 = False
            detect_time0 = detect_time1 = None
            set_led(target)
            command_time = time.monotonic()

        elif time.monotonic() - command_time > TIMEOUT_S:
            timeouts += 1
            target ^= 1
            confirmed0 = confirmed1 = False
            detect_time0 = detect_time1 = None
            set_led(target)
            command_time = time.monotonic()

except KeyboardInterrupt:
    pass

finally:
    pool.shutdown(wait=True)
    set_led(0)
    cam0.stop()
    cam1.stop()
    lgpio.gpiochip_close(gpio)

    print("── Raw capture timing (answers 'how long to get a frame') ──")
    print(f"  cam0 capture_array() duration: mean={statistics.mean(cap_dur0_list):.3f}ms  "
          f"std={statistics.pstdev(cap_dur0_list):.3f}ms")
    print(f"  cam1 capture_array() duration: mean={statistics.mean(cap_dur1_list):.3f}ms  "
          f"std={statistics.pstdev(cap_dur1_list):.3f}ms")
    print(f"  Per-iteration wall time (parallel, both cameras): "
          f"mean={statistics.mean(iteration_wall_list)*1000:.3f}ms  "
          f"std={statistics.pstdev(iteration_wall_list)*1000:.3f}ms")
    print(f"  (compare: sequential would be roughly the SUM of both capture durations above)\n")

    print("── Closed-loop summary ──────────────────────────────")
    n = len(skew_list)
    print(f"  Transitions with both cameras confirming: {n}  (timeouts: {timeouts})")
    if n:
        for name, lats in [("cam0", lat0_list), ("cam1", lat1_list)]:
            mean_l = statistics.mean(lats)
            std_l = statistics.pstdev(lats) if n > 1 else 0.0
            print(f"  {name} latency: mean={mean_l:.3f}ms  std={std_l:.3f}ms  "
                  f"min={min(lats):.3f}ms  max={max(lats):.3f}ms")

        mean_skew = statistics.mean(skew_list)
        std_skew = statistics.pstdev(skew_list) if n > 1 else 0.0
        print(f"\n  Inter-camera skew (cam0 - cam1): mean={mean_skew:+.3f}ms  "
              f"std={std_skew:.3f}ms  min={min(skew_list):+.3f}ms  max={max(skew_list):+.3f}ms")
        print(f"  |skew| max: {max(abs(s) for s in skew_list):.3f}ms")

        if len(command_times) > 1:
            periods = [command_times[i+1] - command_times[i] for i in range(len(command_times)-1)]
            mean_p = statistics.mean(periods)
            freq = 1.0 / mean_p if mean_p else float("nan")
            print(f"\n  Effective closed-loop toggle frequency: {freq:.2f} Hz")
    print("─────────────────────────────────────────")
