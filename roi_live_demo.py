#!/usr/bin/env python3
"""
Live dual-camera demo for the runtime-movable ROI (set_selection) feature.
Shows both cameras' ROI-mode feeds side by side, with live achieved fps,
and lets the user move each camera's ROI position AND toggle binning
independently, live -- exercising the mid-stream set_selection path
validated in CLAUDE.md ("runtime-movable ROI" section) plus a full
stop/reconfigure/start cycle for binning changes (a separate, heavier
operation -- see CLAUDE.md's "Height/size changes are excluded" note on
why ROI position and binning are handled differently).

Four ROI modes are supported, in two "tiers" by real (pre-bin) crop-window
depth -- 'b' toggles binned<->unbinned within whichever tier a camera is
currently in, it doesn't cross tiers:
  half tier    (400 real rows): 640x200 binned <-> 1280x400 unbinned
  quarter tier (200 real rows): 640x100 binned <-> 1280x200 unbinned (fastest)

Capture runs at each mode's validated floor duration every loop iteration
(see FRAME_DURATION_US_BY_SIZE below); the on-screen image and fps readout
only redraw at DISPLAY_INTERVAL_S (~15fps) so GTK/imshow overhead never
gates capture speed -- the fps counter reflects true capture rate, not
display rate.

Controls (keys apply to the "active" camera, shown highlighted in its
window's overlay -- switch which one is active with the number keys):
  1/2   make camera 0/1 the active camera
  w     move the active camera's ROI up   (decrease y_start)
  s     move the active camera's ROI down (increase y_start)
  r     reset the active camera to y_start=0
  a     reset ALL cameras to y_start=0
  b     toggle binning on the active camera, within its current tier
  q     quit

To change binning on both cameras, press 1, b, 2, b. (A bulk "toggle all"
shortcut on Shift+B was tried and dropped: this app's cv2/GTK key capture
doesn't reliably deliver the shifted keycode -- it silently arrives as
plain lowercase 'b' instead, so Shift+B was actually just re-toggling
whichever camera was already active. Per-camera-only avoids that trap and
matches this app's existing independent-per-camera control model anyway.)

Usage:
  python3 roi_live_demo.py                  # start at 640x200 (binned), step=20 sensor rows
  python3 roi_live_demo.py 1280x400 40       # start mode, step size
  python3 roi_live_demo.py 640x100 20        # fastest validated mode (~880fps ceiling)
"""
import sys
import time
from collections import deque

import cv2
import numpy as np
from picamera2 import Picamera2

from roi_set_selection import get_max_y_start, set_roi_y_start

RAW_FORMAT = "R8"
EXPOSURE_US = 1500
ANALOGUE_GAIN = 4.0
FPS_WINDOW = 30  # frames averaged for the displayed fps
DISPLAY_INTERVAL_S = 1 / 15  # redraw each window at ~15fps regardless of capture rate

# Binned<->unbinned partners, grouped by real (pre-bin) crop-window depth --
# 'b' toggles within a pair, never across tiers. See CLAUDE.md.
TIER_PAIRS = [
    ((640, 200), (1280, 400)),  # half tier:    400 real rows
    ((640, 100), (1280, 200)),  # quarter tier: 200 real rows -- fastest
]


def _tier_partner(size):
    for a, b in TIER_PAIRS:
        if size == a:
            return b
        if size == b:
            return a
    raise ValueError(f"no binning partner known for size {size}")


# Validated per-mode floors from CLAUDE.md's throughput sweeps -- NOT the
# 6000us placeholder the first cut of this script used, which capped
# capture at ~166fps regardless of mode and made the ROI's whole point
# (real speed win) invisible. 1800us/640x200 and 1050us/640x100 are the
# confirmed-clean floors for the two binned modes (~527fps / ~854-880fps
# respectively; 1750us and 1000us hang). 3400us/1280x400 and 1775us/1280x200
# are the two unbinned floors (1280x400 never beat stock, see "Throughput
# result -- NEGATIVE" section; 1280x200 plateaus rather than hanging).
FRAME_DURATION_US_BY_SIZE = {
    (640, 200): 1800,
    (1280, 400): 3400,
    (640, 100): 1050,
    (1280, 200): 1775,
}

def _is_binned(size):
    return size[0] == 640  # both binned modes are 640-wide, both unbinned are 1280-wide


if len(sys.argv) > 1:
    w, h = sys.argv[1].lower().split("x")
    START_SIZE = (int(w), int(h))
else:
    START_SIZE = (640, 200)
STEP = int(sys.argv[2]) if len(sys.argv) > 2 else 20

info = Picamera2.global_camera_info()
if not info:
    print("No cameras detected.")
    raise SystemExit(1)
indices = list(range(len(info)))


def configure_and_start(cam, index, raw_size):
    config = cam.create_video_configuration(
        main={"size": (64, 48), "format": "RGB888"},
        raw={"size": raw_size, "format": RAW_FORMAT},
        buffer_count=2,
    )
    cam.configure(config)
    actual_raw = cam.camera_configuration()["raw"]
    if tuple(actual_raw["size"]) != raw_size:
        print(f"WARNING camera {index}: requested {raw_size} got {actual_raw['size']} "
              f"-- the ROI mode was not selected as expected.")
    cam.start()
    frame_duration_us = FRAME_DURATION_US_BY_SIZE[raw_size]
    controls = {
        "FrameDurationLimits": (frame_duration_us, frame_duration_us),
        "AeEnable": False,
        "NoiseReductionMode": 0,
        "ExposureTime": EXPOSURE_US,
        "AnalogueGain": ANALOGUE_GAIN,
    }
    unsupported = [k for k in controls if k not in cam.camera_controls]
    for k in unsupported:
        del controls[k]
    cam.set_controls(controls)


def make_camera(index):
    cam = Picamera2(index)
    configure_and_start(cam, index, START_SIZE)
    return cam


def apply_y_start(index, target):
    """Push index's y_start and verify it actually landed, retrying briefly.

    Needed for two reasons found by testing, not just theory: (1) cam.start()
    can return before the driver has fully settled into a freshly-selected
    pad format, so a set_selection pushed immediately after can land in the
    driver's "not a currently-adjustable mode" fallback and silently echo
    back y_start=0 instead of erroring; (2) reconfiguring ONE camera's mode
    was found to also silently reset the OTHER (untouched, already-streaming)
    camera's roi_y_start back to 0 -- confirmed reproducible in both
    directions (toggling cam0 clobbers cam1's already-applied position and
    vice versa), most likely a side effect of both sensors sharing one
    media-graph pad-format validation pass in the CFE bridge driver even
    though their CSI/register paths are otherwise independent. Because of
    (2), toggle_binning re-applies EVERY camera's y_start after any single
    camera's reconfigure, not just the one that was toggled.
    """
    max_y_start = get_max_y_start(index)
    expected = max(0, min(target, max_y_start))
    for attempt in range(10):
        y_starts[index] = set_roi_y_start(index, target)
        if y_starts[index] == expected:
            return
        time.sleep(0.05)
    print(f"WARNING camera {index}: y_start did not settle at {target} "
          f"after retries, landed at {y_starts[index]}")


def toggle_binning(index):
    """Stop, reconfigure to this camera's binning partner within its current
    tier, restart, and re-apply every camera's y_start (a tier's two modes
    share the same real crop-window height and max y_start, so positions
    carry over unchanged within a tier) -- see apply_y_start() for why ALL
    cameras get re-applied, not just this one.
    """
    new_size = _tier_partner(raw_sizes[index])
    cams[index].stop()
    configure_and_start(cams[index], index, new_size)
    raw_sizes[index] = new_size
    for i in indices:
        apply_y_start(i, y_starts[i])
    fps_history[index].clear()
    cv2.resizeWindow(window_names[index], max(new_size[0], 480), max(new_size[1], 200))
    w, h = new_size
    kind = "binned" if _is_binned(new_size) else "unbinned"
    print(f"cam {index}: binning={kind} {w}x{h}  y_start={y_starts[index]}")


print(f"Opening {len(indices)} camera(s)... start={START_SIZE} step={STEP} rows")
cams = [make_camera(i) for i in indices]

window_names = [f"Camera {i}" for i in indices]
for i, name in zip(indices, window_names):
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, max(START_SIZE[0], 480), max(START_SIZE[1], 200))

raw_sizes = {i: START_SIZE for i in indices}
y_starts = {i: 0 for i in indices}
fps_history = {i: deque(maxlen=FPS_WINDOW) for i in indices}
last_display_time = {i: 0.0 for i in indices}
last_waitkey_time = 0.0
active = indices[0]
digit_keys = {ord(str(i + 1)): i for i in indices if i < 9}

print(f"y_start=0 for all cameras (range 0-{get_max_y_start(indices[0])} at {START_SIZE})")
print(f"Active camera: {active}")
print("Controls: 1/2 select camera, w/s move it, r reset it, a reset all, "
      "b toggle its binning, q quit")

try:
    while True:
        for i, cam in zip(indices, cams):
            # Capture every iteration, unconditionally -- this is what lets
            # capture run at the mode's real rate. Display work below (the
            # actual bottleneck this app used to have: normalize/cvtColor/
            # imshow every single frame) only runs on a throttled cadence,
            # so the fps counter reflects true capture speed, not display
            # speed, while still refreshing the on-screen image and its fps
            # readout often enough to watch live.
            frame = cam.capture_array("raw").view(np.uint16)
            now = time.monotonic()
            fps_history[i].append(now)

            if now - last_display_time[i] < DISPLAY_INTERVAL_S:
                continue
            last_display_time[i] = now

            display = np.empty_like(frame, dtype=np.uint8)
            cv2.normalize(frame, display, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            display = cv2.cvtColor(display, cv2.COLOR_GRAY2BGR)
            is_active = (i == active)
            tag = " [ACTIVE -- w/s/r/b apply here]" if is_active else "  (press %d to control)" % (i + 1)
            color = (0, 255, 0) if is_active else (160, 160, 160)
            hist = fps_history[i]
            fps = (len(hist) - 1) / (hist[-1] - hist[0]) if len(hist) > 1 and hist[-1] != hist[0] else 0.0
            mode = "binned" if _is_binned(raw_sizes[i]) else "unbinned"
            w, h = raw_sizes[i]
            cv2.putText(display, f"cam {i}: {w}x{h} {mode}  y_start={y_starts[i]}  {fps:.1f}fps{tag}",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
                        cv2.LINE_AA)
            cv2.imshow(window_names[i], display)

        # waitKey's nominal "1ms" wait is not what it costs in practice --
        # measured ~2.2ms/call with two windows open on this GTK backend,
        # which at every-capture-iteration cadence was eating ~69% of the
        # loop (this was the actual fps bottleneck, not display rendering
        # or GIL contention between the two cameras -- both measured and
        # ruled out separately). Throttling it to the same cadence as the
        # display redraw keeps keyboard response well under human reaction
        # time while no longer capping capture throughput.
        now = time.monotonic()
        if now - last_waitkey_time < DISPLAY_INTERVAL_S:
            continue
        last_waitkey_time = now

        # Drain every pending key this throttle window, not just one --
        # cv2.waitKey() only ever returns a single queued key per call, so
        # with waitKey now polled at ~15Hz instead of every capture, two
        # keys pressed faster than ~67ms apart used to strand the second
        # one until the next poll (confirmed by testing: two rapid 's'
        # presses only moved the ROI once). Looping until the queue is
        # empty (0xFF = no key) fixes that without giving up the fps win.
        quit_requested = False
        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == 0xFF:
                break
            if key == ord('q'):
                quit_requested = True
                break
            elif key in digit_keys:
                active = digit_keys[key]
                print(f"active camera: {active}")
            elif key == ord('w'):
                y_starts[active] = set_roi_y_start(active, y_starts[active] - STEP)
                print(f"cam {active}: y_start={y_starts[active]}")
            elif key == ord('s'):
                y_starts[active] = set_roi_y_start(active, y_starts[active] + STEP)
                print(f"cam {active}: y_start={y_starts[active]}")
            elif key == ord('r'):
                y_starts[active] = set_roi_y_start(active, 0)
                print(f"cam {active}: y_start={y_starts[active]}")
            elif key == ord('a'):
                for i in indices:
                    y_starts[i] = set_roi_y_start(i, 0)
                print(f"all cameras reset: {y_starts}")
            elif key == ord('b'):
                toggle_binning(active)
        if quit_requested:
            break

finally:
    cv2.destroyAllWindows()
    for cam in cams:
        cam.stop()
