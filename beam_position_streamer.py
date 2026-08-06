#!/usr/bin/env python3
"""
Headless companion to camera_view_tool.py: continuously detects the beam
centroid and streams it to an STM32 Nucleo over I2C via nucleo_i2c_sender.py.
No display, no GTK -- just capture -> detect -> send, as fast as
find_beam_blob allows. Measured live on this bench (no display, no ROI
writes -- both cost this script doesn't have): ~335fps for 640x200, no
artificial throttle needed (unlike camera_view_tool.py's auto-track, which
was dominated by its subprocess-based ROI-repositioning cost, not
detection itself -- this script doesn't reposition the ROI at all).

Beam detection (find_beam_blob, the confidence-gated raw-value approach) is
intentionally duplicated from camera_view_tool.py rather than imported --
that script has no `if __name__ == "__main__":` guard, so importing it
would execute its live viewer instead of just exposing the function.
Matches this project's existing convention of small, focused, standalone
scripts anyway (see CLAUDE.md's script list). If the confidence-gate
constants (CONTRAST_CONFIDENCE_K etc.) are retuned in one script, mirror
the change here.

Coordinates sent are REAL, ABSOLUTE SENSOR pixels (full-sensor row/column,
0-800 / 0-1280), not frame-local or mode-relative -- so the Nucleo doesn't
need to know anything about which ROI window is active. Both axes account
for the 640-wide modes' 2:1 binning in BOTH dimensions (see
camera_view_tool.py's V_BIN_RATIO_BY_SIZE for the empirical confirmation
this is real, not assumed).

Defaults to full-sensor (1280x800) -- y_start is always 0 and the bin
ratio is always 1 there, so no ROI/binning conversion is needed at all;
simplest possible starting point. Pass a WxH arg for one of the faster
binned ROI modes once higher update rates matter, plus --y-start to
bracket wherever the beam actually is.

IMPORTANT: y_start can NOT be inherited from an earlier roi_set_selection.py
session -- every fresh configure()+start() resets it to 0 (confirmed live:
an earlier version of this script tried to read back a previously-set
position and got 0, because its own mode-select had already reset it by
the time it read). This script sets its own y_start (via --y-start) after
its own start(), following the same retry-verify pattern used elsewhere in
this project for the same "cam.start() can return before the driver
settles" reason.

Usage:
  python3 beam_position_streamer.py [WxH] [--y-start N] [--dry-run]
    WxH        raw sensor size, default 1280x800 (full sensor). The two
               binned ROI modes (640x200, 640x100) also work.
    --y-start  real sensor row to center the ROI window's top on, for the
               binned modes only (ignored/inapplicable for full sensor).
               Omit and the window defaults to y_start=0 -- fine for full
               sensor, but for a binned mode the beam needs to actually be
               within [0, height*2) or nothing will be detected.
    --dry-run  skip the actual I2C send, print what would be sent instead
               -- use this before the I2C bus is enabled / before a Nucleo
               is wired up, to validate detection alone.

Not yet tested against real hardware -- no Nucleo in this session to send
to. --dry-run mode IS validated live (see CLAUDE.md).
"""
import sys
import time

import cv2
import numpy as np
from picamera2 import Picamera2

from roi_set_selection import get_max_y_start, set_roi_y_start

EXPOSURE_US = 1500
ANALOGUE_GAIN = 4.0
CAM_INDEX = 0

FRAME_DURATION_US_BY_SIZE = {
    (1280, 800): 6000,
    (640, 200): 1800,
    (640, 100): 1050,
}
V_BIN_RATIO_BY_SIZE = {
    (1280, 800): 1,
    (640, 200): 2,
    (640, 100): 2,
}

MIN_BLOB_AREA_PX = 15
CONTRAST_CONFIDENCE_K = 5.0
MASK_THRESH_K = 3.0

STATUS_INTERVAL = 200  # print a rate summary every this many cycles


def find_beam_blob(frame):
    """Duplicated from camera_view_tool.py -- see its module docstring's
    "Beam detection note" for why this works on raw values with a
    confidence gate instead of Otsu-on-normalized-8-bit."""
    median = float(np.median(frame))
    std = float(frame.std())
    peak = float(frame.max())
    if std == 0 or (peak - median) < CONTRAST_CONFIDENCE_K * std:
        return None

    mask = (frame >= median + MASK_THRESH_K * std).astype(np.uint8) * 255
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    peak_y, peak_x = np.unravel_index(np.argmax(frame), frame.shape)
    blob = next((c for c in contours
                 if cv2.pointPolygonTest(c, (int(peak_x), int(peak_y)), False) >= 0),
                max(contours, key=cv2.contourArea))
    if cv2.contourArea(blob) < MIN_BLOB_AREA_PX:
        return None

    _, radius = cv2.minEnclosingCircle(blob)
    bx, by, bw, bh = cv2.boundingRect(blob)
    blob_mask = np.zeros((bh, bw), dtype=np.uint8)
    cv2.drawContours(blob_mask, [blob], -1, 255, thickness=cv2.FILLED, offset=(-bx, -by))
    roi = frame[by:by+bh, bx:bx+bw].astype(np.float64) * (blob_mask > 0)
    total = roi.sum()
    if total <= 0:
        return None
    ys_idx, xs_idx = np.indices((bh, bw), dtype=np.float64)
    cx = bx + float((roi * xs_idx).sum() / total)
    cy = by + float((roi * ys_idx).sum() / total)
    return cx, cy, radius


def parse_args():
    raw_args = sys.argv[1:]
    dry_run = "--dry-run" in raw_args
    remaining = [a for a in raw_args if a != "--dry-run"]

    y_start_arg = None
    positional = []
    i = 0
    while i < len(remaining):
        if remaining[i] == "--y-start":
            y_start_arg = int(remaining[i + 1])
            i += 2
        else:
            positional.append(remaining[i])
            i += 1

    if positional:
        w, h = positional[0].lower().split("x")
        raw_size = (int(w), int(h))
    else:
        raw_size = (1280, 800)
    if raw_size not in FRAME_DURATION_US_BY_SIZE:
        print(f"Unsupported size {raw_size}. Supported: "
              f"{list(FRAME_DURATION_US_BY_SIZE)}")
        raise SystemExit(1)
    return raw_size, dry_run, y_start_arg


def apply_y_start(target):
    """Push y_start and verify it actually landed, retrying briefly.

    Needed because cam.start() can return before the driver has fully
    settled into a freshly-selected pad format, so a set_selection pushed
    immediately after can silently no-op -- same issue documented and
    worked around in camera_view_tool.py/roi_live_demo.py's apply_y_start.
    """
    max_y_start = get_max_y_start(CAM_INDEX)
    expected = max(0, min(target, max_y_start))
    expected -= expected % 4  # driver rounds down to a 4-row boundary
    landed = None
    for _ in range(10):
        landed = set_roi_y_start(CAM_INDEX, target)
        if landed == expected:
            return landed
        time.sleep(0.05)
    print(f"WARNING: y_start did not settle at {target}, landed at {landed}")
    return landed


def main():
    raw_size, dry_run, y_start_arg = parse_args()
    v_bin = V_BIN_RATIO_BY_SIZE[raw_size]

    cam = Picamera2(CAM_INDEX)
    config = cam.create_video_configuration(raw={"size": raw_size, "format": "R8"}, buffer_count=2)
    cam.configure(config)
    cam.start()
    frame_duration_us = FRAME_DURATION_US_BY_SIZE[raw_size]
    cam.set_controls({
        "FrameDurationLimits": (frame_duration_us, frame_duration_us),
        "AeEnable": False,
        "NoiseReductionMode": 0,
        "ExposureTime": EXPOSURE_US,
        "AnalogueGain": ANALOGUE_GAIN,
    })

    # IMPORTANT: y_start cannot be "read from a previous session" -- every
    # fresh configure()+start() resets it to 0 (ov9282_set_pad_format's own
    # behavior, documented in CLAUDE.md), which happens above, before this
    # point. Confirmed live: an earlier version of this script tried to
    # read back whatever roi_set_selection.py had set in a PRIOR, separate
    # process and got 0 instead of the expected value, because this
    # script's own mode-select had already reset it by the time it read.
    # This script must set its own y_start (if any) after start(), not
    # inherit one externally.
    if raw_size == (1280, 800):
        y_start = 0  # full sensor isn't ROI-adjustable at all
    elif y_start_arg is not None:
        y_start = apply_y_start(y_start_arg)
    else:
        y_start = 0
        print("WARNING: no --y-start given for a binned ROI mode -- window "
              f"defaults to y_start=0 (real rows 0-{raw_size[1] * v_bin}). "
              "If the beam isn't in that range, no detection will happen; "
              "pass --y-start <row> to bracket where it actually is (check "
              "with camera_view_tool.py or a full-sensor --dry-run first).")

    print(f"raw_size={raw_size} y_start={y_start} (pre-bin) v_bin={v_bin} "
          f"dry_run={dry_run}")

    link = None
    if not dry_run:
        from nucleo_i2c_sender import NucleoLink
        link = NucleoLink()

    last_abs_x, last_abs_y = 0, 0
    n_sent = 0
    n_send_errors = 0
    t_start = time.monotonic()
    try:
        while True:
            frame = cam.capture_array("raw").view(np.uint16)
            found = find_beam_blob(frame)
            if found is not None:
                cx, cy, _radius = found
                # Real, absolute sensor coordinates -- both axes scaled by
                # the same 2:1 binning ratio these modes use in width and
                # height alike (empirically confirmed, see
                # camera_view_tool.py's V_BIN_RATIO_BY_SIZE). Kept as float,
                # NOT rounded to int here -- find_beam_blob's centroid is
                # sub-pixel, and NucleoLink.send_position does its own
                # fixed-point scaling (POSITION_SCALE) to preserve that
                # precision on the wire; rounding to a whole pixel here
                # would throw it away before it ever got there.
                last_abs_x = cx * v_bin
                last_abs_y = y_start + cy * v_bin
                valid = True
            else:
                valid = False  # keep last_abs_x/y -- Nucleo decides what
                                 # to do with a stale-but-flagged position

            if dry_run:
                print(f"x={last_abs_x:.1f} y={last_abs_y:.1f} valid={valid}")
            else:
                try:
                    link.send_position(last_abs_x, last_abs_y, valid=valid)
                except OSError as e:
                    # e.g. the TimeoutError smbus2 raises when the Nucleo
                    # doesn't ACK -- expected while it's held in reset or
                    # mid-reflash on the laptop. Don't let a send failure
                    # kill this process; this script is meant to be left
                    # running unattended as the Pi's one telemetry source.
                    n_send_errors += 1
                    if n_send_errors == 1 or n_send_errors % STATUS_INTERVAL == 0:
                        print(f"WARNING: I2C send to Nucleo failed ({e}) -- "
                              f"{n_send_errors} failures so far, still "
                              f"retrying every frame.")

            n_sent += 1
            if n_sent % STATUS_INTERVAL == 0:
                elapsed = time.monotonic() - t_start
                print(f"{n_sent} sent ({n_sent / elapsed:.1f}/s average, "
                      f"{n_send_errors} send failures)")
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        cam.close()
        if link is not None:
            link.close()


if __name__ == "__main__":
    main()
