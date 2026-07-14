#!/usr/bin/env python3
"""
Live dual-camera demo for the runtime-movable ROI (set_selection) feature.
Shows both cameras' ROI-mode feeds side by side and lets the user move the
shared ROI position live, while both cameras are actively streaming --
exercising the exact mid-stream set_selection path validated in CLAUDE.md
("runtime-movable ROI" section), not just a before-start move.

Controls (click a video window to give it keyboard focus first):
  w   move ROI up   (decrease y_start)
  s   move ROI down (increase y_start)
  r   reset to y_start=0
  q   quit

Usage:
  python3 roi_live_demo.py                  # 640x200, step=20 sensor rows
  python3 roi_live_demo.py 1280x400 40       # mode, step size
"""
import sys

import cv2
import numpy as np
from picamera2 import Picamera2

from roi_set_selection import MAX_Y_START, set_roi_y_start

RAW_FORMAT = "R8"
EXPOSURE_US = 1500
ANALOGUE_GAIN = 4.0
FRAME_DURATION_US = 6000

if len(sys.argv) > 1:
    w, h = sys.argv[1].lower().split("x")
    RAW_SIZE = (int(w), int(h))
else:
    RAW_SIZE = (640, 200)
STEP = int(sys.argv[2]) if len(sys.argv) > 2 else 20

info = Picamera2.global_camera_info()
if not info:
    print("No cameras detected.")
    raise SystemExit(1)
indices = list(range(len(info)))


def make_camera(index):
    cam = Picamera2(index)
    config = cam.create_video_configuration(
        main={"size": (64, 48), "format": "RGB888"},
        raw={"size": RAW_SIZE, "format": RAW_FORMAT},
        buffer_count=2,
    )
    cam.configure(config)
    actual_raw = cam.camera_configuration()["raw"]
    if tuple(actual_raw["size"]) != RAW_SIZE:
        print(f"WARNING camera {index}: requested {RAW_SIZE} got {actual_raw['size']} "
              f"-- the ROI mode was not selected as expected.")
    cam.start()
    controls = {
        "FrameDurationLimits": (FRAME_DURATION_US, FRAME_DURATION_US),
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


print(f"Opening {len(indices)} camera(s)... RAW_SIZE={RAW_SIZE} step={STEP} rows")
cams = [make_camera(i) for i in indices]

window_names = [f"Camera {i} -- ROI {RAW_SIZE[0]}x{RAW_SIZE[1]}" for i in indices]
for name in window_names:
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, max(RAW_SIZE[0], 480), max(RAW_SIZE[1], 200))

y_start = 0
print(f"y_start=0  (range 0-{MAX_Y_START})")
print("Controls: w = ROI up, s = ROI down, r = reset, q = quit")

try:
    while True:
        for i, cam in zip(indices, cams):
            frame = cam.capture_array("raw").view(np.uint16)
            display = np.empty_like(frame, dtype=np.uint8)
            cv2.normalize(frame, display, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            display = cv2.cvtColor(display, cv2.COLOR_GRAY2BGR)
            cv2.putText(display, f"y_start={y_start}  (w/s move, r reset, q quit)",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                        cv2.LINE_AA)
            cv2.imshow(window_names[i], display)

        key = cv2.waitKey(1) & 0xFF
        requested = None
        if key == ord('q'):
            break
        elif key == ord('w'):
            requested = y_start - STEP
        elif key == ord('s'):
            requested = y_start + STEP
        elif key == ord('r'):
            requested = 0

        if requested is not None:
            for i in indices:
                actual = set_roi_y_start(i, requested)
            y_start = actual  # driver-applied value (post-clamp), same for both cams
            print(f"y_start={y_start}")

finally:
    cv2.destroyAllWindows()
    for cam in cams:
        cam.stop()
