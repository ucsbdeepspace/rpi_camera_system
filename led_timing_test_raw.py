#!/usr/bin/env python3
"""
Closed-loop LED control timing test -- RAW capture variant.

Same measurement as led_timing_test.py (command a toggle, time how long
until the camera detects the new state, repeat), but reads brightness
from the raw R8 stream instead of the RGB888 main stream. The point is
to see whether skipping the ISP debayer/scale pass on every frame buys
a meaningfully faster closed-loop toggle frequency.

  1. Flip the target state (ON <-> OFF) and record the command timestamp.
  2. Capture raw frames as fast as possible until ROI brightness crosses
     THRESHOLD in the expected direction.
  3. Record the detection timestamp -> latency = detect_time - command_time.
  4. Repeat for TEST_DURATION_S seconds.

At the end, reports:
  - total toggles detected
  - mean / std of per-transition latency (command -> detection)
  - mean / std of inter-detection period (detection -> next detection),
    which approximates your effective max closed-loop toggle frequency

THRESHOLD below (2.51) came from led_calibrate_raw.py run with the box
closed at EXPOSURE_US=1500 / ANALOGUE_GAIN=4.0 (OFF mean ~2.05, ON mean
~2.98). Recalibrate if ROI, exposure, gain, or lighting conditions change.

CAUTION: same rules as the RGB888 version -- run in a fresh process,
step FORCE_FRAME_DURATION_US down gradually across separate runs, and
if you hit a "Camera frontend has timed out" error you must restart
the process before retrying (it cannot recover itself).

Install:  pip install lgpio opencv-python numpy
          (picamera2 is pre-installed on RPi OS Bookworm)
"""

import time
import statistics
import lgpio
import numpy as np
from picamera2 import Picamera2

# ── Config ────────────────────────────────────────────────────────────────────
LED_PIN          = 14          # GPIO pin driving the LED (BCM)
GPIOCHIP         = 4           # /dev/gpiochip4 on RPi 5
ROI              = (300, 220, 40, 40)   # (x, y, width, height)
RAW_SIZE         = (640, 400)  # native sensor resolution
RAW_FORMAT       = "R8"        # unpacked 8-bit mono, no bit-unpacking needed in Python
THRESHOLD        = 2.51        # calibrated for raw stream -- see docstring above; not the
                                # same scale as the old RGB888 THRESHOLD
TEST_DURATION_S  = 5.0         # total test window
TIMEOUT_S        = 1.0         # safety: max wait for a single detection
EXPOSURE_US      = 1500        # matches led_calibrate_raw.py calibration run
ANALOGUE_GAIN    = 4.0         # matches led_calibrate_raw.py calibration run

# IMPORTANT: cam.camera_controls["FrameDurationLimits"] reports the SENSOR's raw
# readout limit, not the limit of the full pipeline you're actually running.
# Asking for that sensor-only minimum can stall the hardware capture pipeline
# entirely (a "Camera frontend has timed out" error) -- which is NOT a Python
# exception you can catch; the only recovery is killing the script and
# starting a fresh process. Don't request the raw minimum directly -- start
# conservative and binary-search down across separate runs, watching for
# that error each time.
FORCE_FRAME_DURATION_US = 5000  # ~200fps -- already confirmed stable in the raw
                                 # calibration script; step down from here once
                                 # this value is confirmed stable for this test too

# ── Setup ─────────────────────────────────────────────────────────────────────
gpio = lgpio.gpiochip_open(GPIOCHIP)
lgpio.gpio_claim_output(gpio, LED_PIN, 0)

cam = Picamera2(0)

print("Available sensor modes (size / fps range):")
for m in cam.sensor_modes:
    print(f"  {m['size']}  fps={m.get('fps')}  format={m.get('format')}")
print()

# picamera2 requires a 'main' stream entry for a valid pipeline config even
# though we never call capture_array("main") below -- kept tiny so it costs
# as little as possible. buffer_count=2 keeps the capture queue shallow so
# capture_array() returns the most recent frame rather than one that was
# already in flight before the LED was toggled.
config = cam.create_video_configuration(
    main={"size": (64, 48), "format": "RGB888"},
    raw={"size": RAW_SIZE, "format": RAW_FORMAT},
    buffer_count=2,
)
cam.configure(config)
cam.start()

# Query the fastest frame duration this mode actually supports, then request it.
min_us, max_us, default_us = cam.camera_controls["FrameDurationLimits"]
target_us = FORCE_FRAME_DURATION_US if FORCE_FRAME_DURATION_US is not None else min_us
print(f"FrameDurationLimits supported range: {min_us}-{max_us}us (default {default_us}us)")
print(f"Requesting frame duration: {target_us}us (~{1_000_000/target_us:.1f}fps)\n")

controls = {
    "FrameDurationLimits": (target_us, target_us),
    "AeEnable": False,
    "NoiseReductionMode": 0,  # Off
}
if EXPOSURE_US is not None:
    controls["ExposureTime"] = EXPOSURE_US
if ANALOGUE_GAIN is not None:
    controls["AnalogueGain"] = ANALOGUE_GAIN

# Mono sensors don't advertise color controls such as AwbEnable -- filter to
# only what libcamera actually exposes so a missing control on a given
# sensor doesn't crash the whole run.
unsupported = [k for k in controls if k not in cam.camera_controls]
for k in unsupported:
    print(f"  (skipping control not advertised on this sensor: {k})")
    del controls[k]

cam.set_controls(controls)
time.sleep(1.0)  # let exposure/frame-duration settle before the timed test starts

rx, ry, rw, rh = ROI


def set_led(state: int):
    lgpio.gpio_write(gpio, LED_PIN, state)


def brightness() -> float:
    frame = cam.capture_array("raw")
    return float(np.mean(frame[ry:ry+rh, rx:rx+rw]))


def is_on(b: float) -> bool:
    return b > THRESHOLD


# ── Calibration check before the timed test ───────────────────────────────────
print("Raw-stream calibration check:")
set_led(1)
time.sleep(0.2)
on_sample = brightness()
print(f"  LED ON  raw brightness: {on_sample:.2f}")
set_led(0)
time.sleep(0.2)
off_sample = brightness()
print(f"  LED OFF raw brightness: {off_sample:.2f}")
print(f"  Current THRESHOLD={THRESHOLD}  "
      f"({'looks fine' if off_sample < THRESHOLD < on_sample else 'ADJUST THRESHOLD before trusting results'})\n")

# ── Closed-loop toggle test ───────────────────────────────────────────────────
latencies = []     # command -> detection, seconds
detect_times = []  # absolute monotonic time of each successful detection
timeouts = 0

target = 1  # start by commanding ON
set_led(target)
command_time = time.monotonic()

test_start = time.monotonic()
print(f"Running closed-loop toggle test for {TEST_DURATION_S}s, threshold={THRESHOLD}\n")

try:
    while True:
        now = time.monotonic()
        if now - test_start >= TEST_DURATION_S:
            break

        b = brightness()
        detected = is_on(b)

        if detected == bool(target):
            detect_time = time.monotonic()
            latency = detect_time - command_time
            latencies.append(latency)
            detect_times.append(detect_time)

            target ^= 1
            set_led(target)
            command_time = time.monotonic()

        elif now - command_time > TIMEOUT_S:
            timeouts += 1
            print(f"  WARNING: timed out waiting for {'ON' if target else 'OFF'} "
                  f"(last brightness={b:.1f}) -- forcing flip")
            target ^= 1
            set_led(target)
            command_time = time.monotonic()

except KeyboardInterrupt:
    pass

finally:
    print("\n── Summary ──────────────────────────────")
    n = len(latencies)
    print(f"  Toggles detected: {n}  (timeouts: {timeouts})")
    if n:
        mean_lat = statistics.mean(latencies)
        std_lat  = statistics.pstdev(latencies) if n > 1 else 0.0
        print(f"  Per-transition latency: mean={mean_lat*1000:.1f} ms  std={std_lat*1000:.1f} ms  "
              f"min={min(latencies)*1000:.1f} ms  max={max(latencies)*1000:.1f} ms")

    if len(detect_times) > 1:
        periods = [detect_times[i+1] - detect_times[i] for i in range(len(detect_times)-1)]
        mean_p = statistics.mean(periods)
        std_p  = statistics.pstdev(periods) if len(periods) > 1 else 0.0
        freq   = 1.0 / mean_p if mean_p else float("nan")
        print(f"  Inter-detection period: mean={mean_p*1000:.1f} ms  std={std_p*1000:.1f} ms")
        print(f"  Effective closed-loop toggle frequency: {freq:.2f} Hz "
              f"(full ON+OFF cycle: {2*mean_p*1000:.1f} ms)")
    print("─────────────────────────────────────────")

    cam.stop()
    set_led(0)
    lgpio.gpiochip_close(gpio)
