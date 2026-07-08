#!/usr/bin/env python3
"""
Two-process skew test -- DRIVER (cam0 + GPIO).

Run this alongside led_skew_test_logger_cam1.py, in a separate terminal,
started at roughly the same time (doesn't need to be precise).

This process is a single-camera closed-loop test, same as
led_timing_test_raw.py -- it doesn't know or care about the second
camera at all. It just drives the LED as fast as it can against its own
detections and logs every command/detect timestamp pair to CSV.

Being a completely separate OS process (not a thread), it has its own
Python interpreter and GIL -- no contention with the logger process, and
no "wait for both" coupling between the two cameras' timing.

Install:  pip install lgpio numpy
          (picamera2 is pre-installed on RPi OS Bookworm)
"""

import csv
import time
import statistics
import lgpio
import numpy as np
from picamera2 import Picamera2

# ── Config -- MUST match led_skew_test_logger_cam1.py's camera settings ────
CAMERA_INDEX     = 0
LED_PIN          = 14
GPIOCHIP         = 4
ROI              = (0, 0, 640, 400)   # full frame
RAW_SIZE         = (640, 400)
RAW_FORMAT       = "R8"
FORCE_FRAME_DURATION_US = 4000   # best result from the earlier sweep
EXPOSURE_US      = 1500
ANALOGUE_GAIN    = 4.0

TEST_DURATION_S  = 8.0    # bumped up to match the logger's longer window
TIMEOUT_S        = 1.0
OUTPUT_CSV       = "skew_test_driver_cam0.csv"

# ── Setup ─────────────────────────────────────────────────────────────────────
gpio = lgpio.gpiochip_open(GPIOCHIP)
lgpio.gpio_claim_output(gpio, LED_PIN, 0)

cam = Picamera2(CAMERA_INDEX)
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
time.sleep(1.0)

rx, ry, rw, rh = ROI


def set_led(state: int):
    lgpio.gpio_write(gpio, LED_PIN, state)


def brightness() -> float:
    frame = cam.capture_array("raw")
    return float(np.mean(frame[ry:ry+rh, rx:rx+rw]))


# ── Calibration ──────────────────────────────────────────────────────────────
print("Driver (cam0) calibration:")
set_led(1)
time.sleep(0.2)
on_sample = brightness()
set_led(0)
time.sleep(0.2)
off_sample = brightness()
threshold = (on_sample + off_sample) / 2.0
print(f"  ON={on_sample:.2f} OFF={off_sample:.2f} threshold={threshold:.2f} "
      f"({'ok' if off_sample < threshold < on_sample else 'POOR SEPARATION'})\n")


def is_on(b: float) -> bool:
    return b > threshold


# ── Closed-loop, logging every transition ───────────────────────────────────
rows = []
target = 1
set_led(target)
command_time = time.monotonic()
test_start = time.monotonic()
timeouts = 0

print(f"Running {TEST_DURATION_S}s single-camera closed loop, logging every transition\n")

try:
    while time.monotonic() - test_start < TEST_DURATION_S:
        b = brightness()
        now = time.monotonic()

        if is_on(b) == bool(target):
            detect_time = time.monotonic()
            rows.append({
                "index": len(rows),
                "target_state": target,
                "command_time": command_time,
                "detect_time": detect_time,
                "latency_ms": (detect_time - command_time) * 1000.0,
            })
            target ^= 1
            set_led(target)
            command_time = time.monotonic()
        elif now - command_time > TIMEOUT_S:
            timeouts += 1
            target ^= 1
            set_led(target)
            command_time = time.monotonic()

except KeyboardInterrupt:
    pass

finally:
    set_led(0)
    with open(OUTPUT_CSV, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["index", "target_state", "command_time", "detect_time", "latency_ms"])
        w.writeheader()
        w.writerows(rows)

    print(f"Logged {len(rows)} transitions (timeouts: {timeouts}) to {OUTPUT_CSV}")
    if rows:
        lats = [r["latency_ms"] for r in rows]
        print(f"  latency: mean={statistics.mean(lats):.3f}ms  "
              f"std={(statistics.pstdev(lats) if len(lats)>1 else 0):.3f}ms  "
              f"min={min(lats):.3f}ms  max={max(lats):.3f}ms")

    cam.stop()
    lgpio.gpiochip_close(gpio)
