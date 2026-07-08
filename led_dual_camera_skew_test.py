#!/usr/bin/env python3
"""
Dual-camera inter-camera skew test -- single shared LED, open-loop schedule.

Neither camera drives the LED and neither reacts to the other -- the LED
just toggles on a fixed timer, independent of anything either camera sees.
Both cameras run free-running capture loops (their own thread each) that
just log (timestamp, brightness) for every frame as fast as they can.

After the test, transitions are detected independently in each camera's
log by threshold-crossing, then transition k from camera 0 is diffed
against transition k from camera 1 to get the inter-camera detection
skew -- i.e. when both cameras "see" the same physical LED edge, how far
apart in time do they actually register it.

CAVEAT: both capture loops are Python threads in the same process, under
the GIL. That's an upper bound on real hardware skew, not a clean
measurement of it -- thread scheduling/contention could add skew that
two truly independent processes wouldn't have. If results look
suspiciously large or noisy, the next step is splitting this into two
separate processes (e.g. writing to two files, correlated afterward)
to remove that confound.

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

# ROI is per-camera since the two cameras may be mounted at different
# distances/angles to the single LED -- adjust after checking each
# camera's debug frame if detection looks off.
ROI_CAM0         = (300, 220, 40, 40)
ROI_CAM1         = (300, 220, 40, 40)

RAW_SIZE         = (640, 400)
RAW_FORMAT       = "R8"
FORCE_FRAME_DURATION_US = 6000   # same matched-comparison value used in earlier tests
EXPOSURE_US      = 1500
ANALOGUE_GAIN    = 4.0

TOGGLE_PERIOD_S  = 0.1    # LED flips every 100ms, fixed schedule -- ~10Hz
TEST_DURATION_S  = 5.0
OUTPUT_CSV_CAM0  = "skew_test_cam0.csv"
OUTPUT_CSV_CAM1  = "skew_test_cam1.csv"

# ── Camera discovery ─────────────────────────────────────────────────────────
print("Detected cameras:")
for info in Picamera2.global_camera_info():
    print(f"  {info}")
print()

gpio = lgpio.gpiochip_open(GPIOCHIP)
lgpio.gpio_claim_output(gpio, LED_PIN, 0)


def make_camera(index, roi):
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


cam0 = make_camera(0, ROI_CAM0)
cam1 = make_camera(1, ROI_CAM1)
time.sleep(1.0)  # let both settle


def set_led(state: int):
    lgpio.gpio_write(gpio, LED_PIN, state)


def roi_brightness(cam, roi):
    rx, ry, rw, rh = roi
    frame = cam.capture_array("raw")
    return float(np.mean(frame[ry:ry+rh, rx:rx+rw]))


# ── Per-camera calibration (independent thresholds) ────────────────────────
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

# ── Free-running capture threads (no LED logic, just log everything) ───────
log0 = []  # (timestamp, brightness)
log1 = []
stop_event = threading.Event()
start_event = threading.Event()


def capture_loop(cam, roi, log):
    rx, ry, rw, rh = roi
    start_event.wait()
    while not stop_event.is_set():
        frame = cam.capture_array("raw")
        t = time.monotonic()
        b = float(np.mean(frame[ry:ry+rh, rx:rx+rw]))
        log.append((t, b))


def toggle_scheduler():
    start_event.wait()
    commands = []  # (scheduled_time, target_state) for reference
    target = 1
    set_led(target)
    commands.append((time.monotonic(), target))
    next_toggle = time.monotonic() + TOGGLE_PERIOD_S
    test_end = time.monotonic() + TEST_DURATION_S
    while time.monotonic() < test_end:
        now = time.monotonic()
        if now >= next_toggle:
            target ^= 1
            set_led(target)
            commands.append((time.monotonic(), target))
            next_toggle += TOGGLE_PERIOD_S
        else:
            time.sleep(max(0.0, next_toggle - now) * 0.5)
    stop_event.set()
    return commands


t0 = threading.Thread(target=capture_loop, args=(cam0, ROI_CAM0, log0))
t1 = threading.Thread(target=capture_loop, args=(cam1, ROI_CAM1, log1))
t0.start()
t1.start()

print(f"Running {TEST_DURATION_S}s open-loop skew test, "
      f"LED toggling every {TOGGLE_PERIOD_S*1000:.0f}ms\n")
start_event.set()
commands = toggle_scheduler()
t0.join()
t1.join()
set_led(0)

print(f"Logged {len(log0)} frames on cam0, {len(log1)} frames on cam1\n")

# ── Save raw per-frame logs ─────────────────────────────────────────────────
for path, log in [(OUTPUT_CSV_CAM0, log0), (OUTPUT_CSV_CAM1, log1)]:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "brightness"])
        w.writerows(log)
print(f"Saved {OUTPUT_CSV_CAM0} and {OUTPUT_CSV_CAM1}\n")


# ── Detect transitions independently in each log, then diff ────────────────
def detect_transitions(log, threshold):
    """Returns list of (timestamp, new_state) for every threshold crossing."""
    transitions = []
    if not log:
        return transitions
    prev_state = log[0][1] > threshold
    for t, b in log[1:]:
        state = b > threshold
        if state != prev_state:
            transitions.append((t, state))
            prev_state = state
    return transitions


trans0 = detect_transitions(log0, threshold0)
trans1 = detect_transitions(log1, threshold1)
print(f"Detected {len(trans0)} transitions on cam0, {len(trans1)} on cam1 "
      f"({len(commands)-1} commanded)\n")

n = min(len(trans0), len(trans1))
if n < min(len(trans0), len(trans1)):
    print("  WARNING: transition counts mismatch -- one camera missed some "
          "edges. Only the first min(n0,n1) are compared; check threshold/ROI.\n")

skews_ms = []
for k in range(n):
    t0_, s0 = trans0[k]
    t1_, s1 = trans1[k]
    if s0 != s1:
        print(f"  WARNING: transition {k} direction mismatch (cam0={s0}, cam1={s1}) "
              f"-- skipping, likely a missed edge upstream")
        continue
    skews_ms.append((t0_ - t1_) * 1000.0)

print("── Summary ──────────────────────────────")
if skews_ms:
    mean_skew = statistics.mean(skews_ms)
    std_skew = statistics.pstdev(skews_ms) if len(skews_ms) > 1 else 0.0
    print(f"  Matched transitions compared: {len(skews_ms)}")
    print(f"  Inter-camera skew (cam0 - cam1): mean={mean_skew:+.3f} ms  "
          f"std={std_skew:.3f} ms  min={min(skews_ms):+.3f} ms  max={max(skews_ms):+.3f} ms")
    print(f"  |skew| max: {max(abs(s) for s in skews_ms):.3f} ms")
else:
    print("  No matched transitions to compare.")
print("─────────────────────────────────────────")

cam0.stop()
cam1.stop()
lgpio.gpiochip_close(gpio)
