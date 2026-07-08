#!/usr/bin/env python3
"""
Two-process skew test -- LOGGER (cam1 only, no GPIO).

Run this alongside led_skew_test_driver_cam0.py, in a separate terminal,
started at roughly the same time (doesn't need to be precise -- transitions
accumulate over the whole run regardless of small startup offset).

This process never touches the LED. It free-runs capture on its own
camera and logs every (timestamp, brightness) frame to CSV. Threshold
for ON/OFF is computed AFTER the run from the full log's min/max --
since the driver process keeps the LED toggling continuously throughout
the run, both true ON and true OFF brightness will show up many times
in this camera's log too, without needing any explicit coordinated
calibration phase between the two processes.

Install:  pip install numpy
          (picamera2 is pre-installed on RPi OS Bookworm)
"""

import csv
import time
import numpy as np
from picamera2 import Picamera2

# ── Config -- MUST match led_skew_test_driver_cam0.py's camera settings ────
CAMERA_INDEX     = 1
ROI              = (0, 0, 640, 400)   # full frame
RAW_SIZE         = (640, 400)
RAW_FORMAT       = "R8"
FORCE_FRAME_DURATION_US = 4000
EXPOSURE_US      = 1500
ANALOGUE_GAIN    = 4.0

TEST_DURATION_S  = 8.0    # bumped up so there's still a solid overlap window
                            # despite the startup delay below
STARTUP_DELAY_S  = 3.0     # wait for the driver process to finish acquiring its
                            # camera first -- both cameras share one pipeline
                            # handler (rpi/pisp on RP1), which appears to briefly
                            # lock exclusively during acquisition; opening both
                            # at the exact same instant made BOTH fail last time
OUTPUT_CSV       = "skew_test_logger_cam1.csv"

# ── Setup ─────────────────────────────────────────────────────────────────────
print(f"Waiting {STARTUP_DELAY_S}s before opening the camera, so this doesn't "
      f"collide with the driver process acquiring its own camera at the same instant...")
time.sleep(STARTUP_DELAY_S)

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

print(f"Free-running capture on cam{CAMERA_INDEX} for {TEST_DURATION_S}s, logging every frame\n")

log = []
test_start = time.monotonic()
while time.monotonic() - test_start < TEST_DURATION_S:
    frame = cam.capture_array("raw")
    t = time.monotonic()
    b = float(np.mean(frame[ry:ry+rh, rx:rx+rw]))
    log.append((t, b))

with open(OUTPUT_CSV, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["timestamp", "brightness"])
    w.writerows(log)

print(f"Logged {len(log)} frames to {OUTPUT_CSV}")
cam.stop()
