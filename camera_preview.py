#!/usr/bin/env python3
"""
Simple live preview -- one or two cameras, raw mono stream.

Diagnostic/sanity-check tool for after the OS reinstall: shows what each
camera actually sees, using the same raw R8 stream settings that worked
throughout this project (no debayering needed -- these are OV9281 mono
sensors, not color). Press 'q' in any window to quit.

If no cameras are detected, this is almost certainly a config/hardware
issue, not a code issue -- check (in order):
  1. Camera interface enabled: sudo raspi-config -> Interface Options -> Camera
  2. picamera2 installed: sudo apt install -y python3-picamera2
  3. Outside Python entirely: libcamera-hello --list-cameras
     (if this shows nothing, it's not a Python problem)
  4. Ribbon cable seating after reassembly

Install:  pip install opencv-python numpy
          (picamera2 is pre-installed on RPi OS Bookworm)
"""

import cv2
import numpy as np
from picamera2 import Picamera2

RAW_SIZE = (640, 400)
RAW_FORMAT = "R8"
EXPOSURE_US = 1500
ANALOGUE_GAIN = 4.0
FORCE_FRAME_DURATION_US = 6000  # conservative, known-stable value -- not the
                                  # 3400us floor found through careful step-down

# ── Detect available cameras ────────────────────────────────────────────────
info = Picamera2.global_camera_info()
print(f"Detected {len(info)} camera(s):")
for i, cam_info in enumerate(info):
    print(f"  [{i}] {cam_info}")

if not info:
    print("\nNo cameras detected -- this is a config/hardware issue, not a "
          "script issue. See the checklist in this file's docstring.")
    raise SystemExit(1)

indices = list(range(len(info)))  # opens all detected cameras, 1 or 2


def make_camera(index):
    cam = Picamera2(index)
    config = cam.create_video_configuration(
        main={"size": (64, 48), "format": "RGB888"},  # required by the API,
                                                         # never displayed
        raw={"size": RAW_SIZE, "format": RAW_FORMAT},
        buffer_count=2,  # confirmed better than 1 -- see project notes
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
        print(f"  (skipping control not advertised on camera {index}: {k})")
        del controls[k]
    cam.set_controls(controls)
    return cam


print(f"\nOpening {len(indices)} camera(s)...")
cams = [make_camera(i) for i in indices]

window_names = [f"Camera {i} -- press q to quit" for i in indices]
for name in window_names:
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, RAW_SIZE[0], RAW_SIZE[1])

print("Streaming -- press 'q' in any window to quit.\n")

try:
    while True:
        for i, cam in zip(indices, cams):
            frame = cam.capture_array("raw")

            # Normalize for display only -- raw sensor values often sit in a
            # low range that reads as solid black otherwise. This never
            # touches the underlying data, just stretches it for viewing.
            display = np.empty_like(frame, dtype=np.uint8)
            cv2.normalize(frame, display, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            display = cv2.cvtColor(display, cv2.COLOR_GRAY2BGR)

            cv2.imshow(window_names[i], display)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cv2.destroyAllWindows()
    for cam in cams:
        cam.stop()
