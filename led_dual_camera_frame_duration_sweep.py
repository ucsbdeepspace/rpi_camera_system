#!/usr/bin/env python3
"""
Dual-camera closed-loop frame-duration sweep.

Same closed-loop structure as led_dual_camera_closed_loop_test.py (command
LED, wait for BOTH cameras to confirm, flip, repeat) but run repeatedly
across a range of FORCE_FRAME_DURATION_US values, to chart latency/skew/
frequency vs frame duration and find where the dual-camera pipeline's
real speed ceiling sits.

IMPORTANT SAFETY NOTE -- READ BEFORE EXTENDING THE SWEEP RANGE:
Requesting a frame duration below what the hardware pipeline can actually
sustain triggers a "Camera frontend has timed out" failure that is NOT a
catchable Python exception -- the process can simply hang, with no way
for this script to recover or even report it. That's why this sweep
stops at a conservative floor (FRAME_DURATION_SWEEP_US below) well above
the single-camera raw-stream floor measured earlier (3228us) -- the
dual-camera combined floor is unknown and could plausibly be HIGHER due
to shared CSI/memory bandwidth across two simultaneous raw streams.

Do not extend this list below its current minimum and run it unattended.
If you want to explore lower values, do it the same way as before: one
value at a time, in a fresh process, watching the terminal, ready to
kill/restart if it hangs.

Results are written to CSV incrementally (flushed after every step), so
a hang on a later step doesn't lose data from steps that already
completed.

Install:  pip install lgpio numpy
          (picamera2 is pre-installed on RPi OS Bookworm)
"""

import csv
import threading
import time
import statistics
import lgpio
import numpy as np
from picamera2 import Picamera2

# ── Config ────────────────────────────────────────────────────────────────────
LED_PIN          = 14
GPIOCHIP         = 4

ROI_CAM0         = (0, 0, 640, 400)   # full frame, confirmed good in the dark-box setup
ROI_CAM1         = (0, 0, 640, 400)

RAW_SIZE         = (640, 400)
RAW_FORMAT       = "R8"
EXPOSURE_US      = 1500     # stays fixed and below every swept frame duration -- no clipping
ANALOGUE_GAIN    = 4.0

# Conservative automated range -- see safety note above. 4000us leaves real
# margin above the known single-camera floor (3228us); do not lower this
# minimum without manual, fresh-process, one-value-at-a-time testing first.
FRAME_DURATION_SWEEP_US = [6000, 5500, 5000, 4500, 4000]

SWEEP_TEST_DURATION_S = 3.0
TIMEOUT_S         = 1.0
SETTLE_S          = 0.3
POLL_SLEEP_S      = 0.0001
OUTPUT_CSV        = "dual_camera_frame_duration_sweep.csv"

# ── Camera discovery / setup ────────────────────────────────────────────────
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
    return cam


cam0 = make_camera(0)
cam1 = make_camera(1)

min0, max0, def0 = cam0.camera_controls["FrameDurationLimits"]
min1, max1, def1 = cam1.camera_controls["FrameDurationLimits"]
print(f"cam0 FrameDurationLimits: {min0}-{max0}us")
print(f"cam1 FrameDurationLimits: {min1}-{max1}us")

sweep_values = [d for d in FRAME_DURATION_SWEEP_US if d >= max(min0, min1)]
skipped = [d for d in FRAME_DURATION_SWEEP_US if d not in sweep_values]
if skipped:
    print(f"  Skipping values below a camera's reported minimum: {skipped}")
print()


def set_frame_duration(cam, duration_us):
    controls = {
        "FrameDurationLimits": (duration_us, duration_us),
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


def set_led(state: int):
    lgpio.gpio_write(gpio, LED_PIN, state)


def roi_brightness(cam, roi):
    rx, ry, rw, rh = roi
    frame = cam.capture_array("raw")
    return float(np.mean(frame[ry:ry+rh, rx:rx+rw]))


def run_one_duration(duration_us):
    set_frame_duration(cam0, duration_us)
    set_frame_duration(cam1, duration_us)
    time.sleep(SETTLE_S)

    set_led(1)
    time.sleep(0.2)
    on0, on1 = roi_brightness(cam0, ROI_CAM0), roi_brightness(cam1, ROI_CAM1)
    set_led(0)
    time.sleep(0.2)
    off0, off1 = roi_brightness(cam0, ROI_CAM0), roi_brightness(cam1, ROI_CAM1)

    threshold0 = (on0 + off0) / 2.0
    threshold1 = (on1 + off1) / 2.0
    valid0 = off0 < threshold0 < on0
    valid1 = off1 < threshold1 < on1

    target = 0
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

    th0 = threading.Thread(target=capture_loop, args=(0, cam0, ROI_CAM0, threshold0))
    th1 = threading.Thread(target=capture_loop, args=(1, cam1, ROI_CAM1, threshold1))
    th0.start()
    th1.start()

    lat0_list, lat1_list, skew_list = [], [], []
    command_times = []
    timeouts = 0

    test_start = time.monotonic()
    while time.monotonic() - test_start < SWEEP_TEST_DURATION_S:
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
            lat0_list.append((detect_time[0] - command_time) * 1000.0)
            lat1_list.append((detect_time[1] - command_time) * 1000.0)
            skew_list.append((detect_time[0] - detect_time[1]) * 1000.0)
        else:
            timeouts += 1

    running = False
    th0.join()
    th1.join()
    set_led(0)

    n = len(skew_list)
    row = {
        "frame_duration_us": duration_us,
        "cam0_on": round(on0, 3), "cam0_off": round(off0, 3), "cam0_threshold": round(threshold0, 3),
        "cam1_on": round(on1, 3), "cam1_off": round(off1, 3), "cam1_threshold": round(threshold1, 3),
        "threshold_valid": valid0 and valid1,
        "transitions": n,
        "timeouts": timeouts,
        "cam0_lat_mean_ms": round(statistics.mean(lat0_list), 3) if n else "",
        "cam0_lat_std_ms": round(statistics.pstdev(lat0_list), 3) if n > 1 else "",
        "cam1_lat_mean_ms": round(statistics.mean(lat1_list), 3) if n else "",
        "cam1_lat_std_ms": round(statistics.pstdev(lat1_list), 3) if n > 1 else "",
        "skew_mean_ms": round(statistics.mean(skew_list), 3) if n else "",
        "skew_std_ms": round(statistics.pstdev(skew_list), 3) if n > 1 else "",
        "skew_abs_max_ms": round(max(abs(s) for s in skew_list), 3) if n else "",
        "freq_hz": "",
    }
    if len(command_times) > 1:
        periods = [command_times[i+1] - command_times[i] for i in range(len(command_times)-1)]
        mean_p = statistics.mean(periods)
        row["freq_hz"] = round(1.0 / mean_p, 3) if mean_p else ""
    return row


print(f"Sweeping {len(sweep_values)} frame durations, "
      f"{SWEEP_TEST_DURATION_S}s each (~{len(sweep_values)*SWEEP_TEST_DURATION_S:.0f}s total)\n")

rows = []
fieldnames = None
f = open(OUTPUT_CSV, "w", newline="")
writer = None

try:
    for duration_us in sweep_values:
        print(f"-- FrameDurationLimits={duration_us}us --")
        row = run_one_duration(duration_us)
        rows.append(row)

        if writer is None:
            fieldnames = list(row.keys())
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
        writer.writerow(row)
        f.flush()

        ok = "ok" if row["threshold_valid"] else "POOR SEPARATION"
        print(f"   cam0 ON={row['cam0_on']} OFF={row['cam0_off']}  "
              f"cam1 ON={row['cam1_on']} OFF={row['cam1_off']}  ({ok})")
        print(f"   transitions={row['transitions']} timeouts={row['timeouts']}  "
              f"cam0_lat={row['cam0_lat_mean_ms']}ms cam1_lat={row['cam1_lat_mean_ms']}ms  "
              f"skew_mean={row['skew_mean_ms']}ms |skew|max={row['skew_abs_max_ms']}ms  "
              f"freq={row['freq_hz']}Hz\n")

except KeyboardInterrupt:
    print("Sweep interrupted -- partial results already flushed to CSV.\n")

finally:
    f.close()
    print(f"Saved {len(rows)} rows to {OUTPUT_CSV}")

    cam0.stop()
    cam1.stop()
    set_led(0)
    lgpio.gpiochip_close(gpio)
