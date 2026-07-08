#!/usr/bin/env python3
"""
LED brightness calibration tool -- RAW mono stream version.

Shows the live camera feed (raw R8 mono, not RGB888) with the ROI
highlighted, toggles the LED at ~1 Hz, and prints brightness in the
raw 0-255 scale so you can pick a THRESHOLD for led_timing_test_raw.py.

This exists because raw R8 pixel values are not guaranteed to be on the
same brightness scale as the RGB888-averaged values from the original
calibration tool -- THRESHOLD=5 (tuned for RGB888) may be wrong here.

Install:  pip install lgpio opencv-python numpy
          (picamera2 is pre-installed on RPi OS Bookworm)

Usage:    python led_calibrate_raw.py
          Press 'q' in the window to quit; a summary with a suggested
          threshold prints at the end.
"""

import time
import lgpio
import numpy as np
import cv2
from picamera2 import Picamera2

# ── Config ────────────────────────────────────────────────────────────────────
LED_PIN     = 14
GPIOCHIP    = 4
ROI         = (300, 220, 40, 40)   # (x, y, width, height) -- move this onto the LED
RAW_SIZE    = (640, 400)           # native sensor resolution
RAW_FORMAT  = "R8"                 # unpacked 8-bit mono
TOGGLE_HZ   = 1.0
EXPOSURE_US = 1500
ANALOGUE_GAIN = 4.0
FORCE_FRAME_DURATION_US = 5000     # same value already confirmed stable in prior tests

# ── Setup ─────────────────────────────────────────────────────────────────────
gpio = lgpio.gpiochip_open(GPIOCHIP)
lgpio.gpio_claim_output(gpio, LED_PIN, 0)

cam = Picamera2(0)
config = cam.create_video_configuration(
    # picamera2 requires a 'main' stream entry to produce a valid pipeline
    # config even when raw is what you care about -- but we never call
    # capture_array("main") below, so it's never actually pulled or processed
    # per-frame. Kept tiny so it costs as little as possible.
    main={"size": (320, 200), "format": "RGB888"},
    raw={"size": RAW_SIZE, "format": RAW_FORMAT},
    buffer_count=2,
)
cam.configure(config)
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

# The raw R8 stream is a single-channel Bayer mosaic (one of R/Gr/Gb/B per
# pixel position), not grayscale image data -- demosaicing it is what turns
# it into a recognizable picture instead of a faint checkerboard texture.
# BGGR is the common Bayer order for Pi camera sensors (OV5647/IMX219/IMX477)
# but if the displayed colors look swapped/wrong, try GBR2BGR, RGGB2BGR, or
# GRBG2BGR instead.
BAYER_CODE = cv2.COLOR_BayerBG2BGR

WINDOW_NAME = "RAW LED calibration -- press q to quit"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW_NAME, RAW_SIZE[0], RAW_SIZE[1])
cv2.moveWindow(WINDOW_NAME, 100, 100)

# Debug snapshot so ROI alignment can be checked even if the live window
# isn't visible for some reason.
debug_frame = cam.capture_array("raw")
print(f"Initial ROI raw brightness: {float(np.mean(debug_frame[ry:ry+rh, rx:rx+rw])):.2f}")
print(f"Initial frame min/max: {debug_frame.min()} / {debug_frame.max()}")

# Demosaic the Bayer mosaic into a real BGR image, then normalize for
# *viewing* only -- this never touches the values used for brightness
# measurement / stats below, which still read the raw single-channel frame
# directly. Color balance will look off (no AWB applied) but the image
# should be recognizable now instead of a faint mosaic texture.
debug_color = cv2.cvtColor(debug_frame, BAYER_CODE)
debug_bgr = np.empty_like(debug_color, dtype=np.uint8)
cv2.normalize(debug_color, debug_bgr, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
cv2.rectangle(debug_bgr, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
cv2.imwrite("roi_debug_raw.png", debug_bgr)
print(f"Saved roi_debug_raw.png -- confirm the green box is over the LED.\n")

led_state = 0
last_toggle = time.monotonic()

stats = {0: [0, 0.0], 1: [0, 0.0]}  # 0=OFF, 1=ON -- running mean per state


def update_stats(state, value):
    n, mean = stats[state]
    n += 1
    mean += (value - mean) / n
    stats[state] = [n, mean]


print("ROI raw brightness while LED toggles at 1 Hz.  Press 'q' in the window to quit.\n")
print(f"  {'LED':<6} {'Raw brightness':>15}")
print(f"  {'-'*6} {'-'*15}")

try:
    while True:
        frame = cam.capture_array("raw")
        now = time.monotonic()

        if now - last_toggle >= 1.0 / TOGGLE_HZ:
            led_state ^= 1
            lgpio.gpio_write(gpio, LED_PIN, led_state)
            last_toggle = now

        brightness = float(np.mean(frame[ry:ry+rh, rx:rx+rw]))
        label = "ON " if led_state else "OFF"
        update_stats(led_state, brightness)
        print(f"\r  {label:<6} {brightness:>15.2f}", end="", flush=True)

        # Single capture per loop -- demosaic + normalize the same raw frame
        # we already measured brightness from, for display only.
        color = cv2.cvtColor(frame, BAYER_CODE)
        display = np.empty_like(color, dtype=np.uint8)
        cv2.normalize(color, display, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        cv2.rectangle(display, (rx, ry), (rx+rw, ry+rh), (0, 255, 0), 2)
        cv2.putText(display, f"LED {label}  raw brightness: {brightness:.2f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow(WINDOW_NAME, display)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    print()
    off_n, off_mean = stats[0]
    on_n, on_mean = stats[1]
    print("\n── Summary ──────────────────────────────")
    print(f"  OFF: mean={off_mean:.2f}  (n={off_n} samples)")
    print(f"  ON:  mean={on_mean:.2f}  (n={on_n} samples)")
    if off_n and on_n:
        threshold = (off_mean + on_mean) / 2
        print(f"  Suggested THRESHOLD for led_timing_test_raw.py: {threshold:.2f}")
        # Relative check instead of an absolute one -- the raw stream's
        # values live in a much lower range (often single digits to low
        # tens) than the RGB888 0-255 scale this check was originally
        # written for, so a fixed "< 2.0" gap is a false alarm here even
        # when ON is genuinely ~40-50% brighter than OFF.
        if off_mean > 0 and (on_mean - off_mean) / off_mean < 0.20:
            print("  WARNING: ON/OFF nearly identical -- check ROI alignment "
                  "(see roi_debug_raw.png), wiring, or exposure/gain settings.")
    print("─────────────────────────────────────────")

    cv2.destroyAllWindows()
    cam.stop()
    lgpio.gpio_write(gpio, LED_PIN, 0)
    lgpio.gpiochip_close(gpio)
