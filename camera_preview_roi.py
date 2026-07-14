#!/usr/bin/env python3
"""
Live preview for the experimental sensor-level ROI modes (patched ov9282
driver): MODE_1280_400_ROI and MODE_640_200_ROI. Opens both cameras at the
requested raw size and shows what's actually being read out -- confirms the
crop is real image data (not garbage/tearing/wrong offset) and lets you
visually check framing against what the ROI *should* see.

Capture always runs at full requested rate; only the display is throttled
(--skip-n), so a slow cv2.imshow doesn't starve the capture loop or make the
preview misleadingly laggy relative to actual camera throughput.

Usage:
  python3 camera_preview_roi.py                    # 640x200, skip-n=5
  python3 camera_preview_roi.py 1280x400            # MODE_1280_400_ROI size
  python3 camera_preview_roi.py 640x200 6000 10     # size, frame_duration_us, skip-n

Prints the actual negotiated raw configuration and each camera's sensor
modes on startup -- confirms Picamera2 actually picked the requested ROI
crop mode rather than silently falling back to something else (e.g. cropping
a larger mode in software, which would defeat the point of the driver patch).
Requires the patched ov9282 module to be loaded (see CLAUDE.md) --
`rpicam-hello --list-cameras` should list the requested size as an R8 mode
before running this. Press 'q' in any window to quit.

Install:  pip install opencv-python numpy
          (picamera2 is pre-installed on RPi OS Bookworm)
"""

import sys

import cv2
import numpy as np
from picamera2 import Picamera2

DEFAULT_RAW_SIZE = (640, 200)
DEFAULT_FRAME_DURATION_US = 6000  # conservative -- floor not characterized
                                    # for either ROI mode, unlike the 3400us
                                    # floor found for stock 640x400
DEFAULT_SKIP_N = 5  # display every Nth captured frame

RAW_FORMAT = "R8"
EXPOSURE_US = 1500
ANALOGUE_GAIN = 4.0

if len(sys.argv) > 1:
    w, h = sys.argv[1].lower().split("x")
    RAW_SIZE = (int(w), int(h))
else:
    RAW_SIZE = DEFAULT_RAW_SIZE
FORCE_FRAME_DURATION_US = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_FRAME_DURATION_US
SKIP_N = int(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_SKIP_N

# ── Detect available cameras ────────────────────────────────────────────────
info = Picamera2.global_camera_info()
print(f"Detected {len(info)} camera(s):")
for i, cam_info in enumerate(info):
    print(f"  [{i}] {cam_info}")

if not info:
    print("\nNo cameras detected -- check the patched module is loaded "
          "(see CLAUDE.md) and re-run `rpicam-hello --list-cameras`.")
    raise SystemExit(1)

indices = list(range(len(info)))  # opens all detected cameras, 1 or 2


def make_camera(index):
    cam = Picamera2(index)

    print(f"\nCamera {index} sensor modes:")
    for mode in cam.sensor_modes:
        print(f"  {mode}")

    config = cam.create_video_configuration(
        main={"size": (64, 48), "format": "RGB888"},  # required by the API,
                                                         # never displayed
        raw={"size": RAW_SIZE, "format": RAW_FORMAT},
        buffer_count=2,
    )
    cam.configure(config)

    actual_raw = cam.camera_configuration()["raw"]
    print(f"Camera {index} negotiated raw config: {actual_raw}")
    if tuple(actual_raw["size"]) != RAW_SIZE:
        print(f"  WARNING: requested {RAW_SIZE} but got {actual_raw['size']} "
              f"-- the requested ROI mode was NOT selected as expected.")

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


print(f"\nOpening {len(indices)} camera(s)... RAW_SIZE={RAW_SIZE} "
      f"frame_duration={FORCE_FRAME_DURATION_US}us skip_n={SKIP_N}")
cams = [make_camera(i) for i in indices]

window_names = [f"Camera {i} -- ROI {RAW_SIZE[0]}x{RAW_SIZE[1]} -- press q to quit" for i in indices]
for name in window_names:
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, RAW_SIZE[0], RAW_SIZE[1])

print("\nStreaming -- press 'q' in any window to quit.\n")

frame_count = 0
try:
    while True:
        frame_count += 1
        show = (frame_count % SKIP_N == 0)
        for i, cam in zip(indices, cams):
            # capture_array("raw") returns the buffer as flat uint8 bytes,
            # but this pipeline always delivers raw frames as 16-bit-per-pixel
            # words (Picamera2 negotiates "R16" even when "R8" is requested)
            # -- .view() reinterprets consecutive byte pairs as the real
            # pixel values instead of interleaved data/padding bytes.
            frame = cam.capture_array("raw").view(np.uint16)

            if not show:
                continue

            # Normalize for display only -- raw sensor values often sit in a
            # low range that reads as solid black otherwise. This never
            # touches the underlying data, just stretches it for viewing.
            display = np.empty_like(frame, dtype=np.uint8)
            cv2.normalize(frame, display, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            display = cv2.cvtColor(display, cv2.COLOR_GRAY2BGR)

            cv2.imshow(window_names[i], display)

        if show and cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cv2.destroyAllWindows()
    for cam in cams:
        cam.stop()
