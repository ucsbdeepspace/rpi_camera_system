#!/usr/bin/env python3
"""
Runtime ROI (vertical position) control for the patched ov9282 driver's
`set_selection` support -- see CLAUDE.md's "runtime-movable ROI" section for
the driver-side design and validation history.

Picamera2 has no high-level API for arbitrary sensor crop, so this shells
out to `v4l2-ctl` subdev ioctls -- the same approach used by the one-off
test scripts that validated this design (pre-stream moves, and moves while
a capture is already running, on both cameras and both windowed ROI modes).

Only `y_start` (vertical crop position) is adjustable, and only for the two
windowed-crop ROI modes (MODE_1280_400_ROI, MODE_640_200_ROI) -- both use
the same real sensor crop window, 1280 wide x 400 rows, before any binning.
Width, output height, and binning are NOT adjustable here (see CLAUDE.md for
why: changing them needs a full reconfigure, not a live crop move, and there
is no hardware-verified register formula for horizontal panning).

The driver clamps out-of-range requests silently (validated: values past the
max, or below 0, land on the nearest valid value rather than erroring) --
`set_roi_y_start` always reads the position back after writing and returns
what actually landed, which may differ from what was requested.

Usage as a script:
  python3 roi_set_selection.py <cam_index>              # report current y_start
  python3 roi_set_selection.py <cam_index> <y_start>     # move to a new y_start

Usage as a module:
  from roi_set_selection import get_roi_y_start, set_roi_y_start
  actual = set_roi_y_start(0, 200)
"""
import subprocess
import sys

from picamera2 import Picamera2

CROP_WIDTH = 1280
CROP_HEIGHT = 400  # real sensor crop window for both ROI modes -- see CLAUDE.md
TOP_BASE = 8  # fixed ISP offset, independent of y_start (same at every stock mode)
MAX_Y_START = 800 - CROP_HEIGHT  # driver's own valid range is [0, MAX_Y_START]


def _subdev_for_camera(cam_index):
    """Map a Picamera2 camera index to its V4L2 subdev path.

    Mapping confirmed via Picamera2.global_camera_info(): index 0 is always
    i2c@88000 -> /dev/v4l-subdev5, index 1 is always i2c@80000 ->
    /dev/v4l-subdev2, on this specific board/media-graph enumeration.
    """
    info = Picamera2.global_camera_info()
    cam_id = info[cam_index]["Id"]
    if "i2c@88000" in cam_id:
        return "/dev/v4l-subdev5"
    if "i2c@80000" in cam_id:
        return "/dev/v4l-subdev2"
    raise ValueError(f"unrecognized camera Id, can't map to subdev: {cam_id}")


def _parse_top(v4l2ctl_output):
    for tok in v4l2ctl_output.split(","):
        tok = tok.strip()
        if tok.startswith("Top"):
            return int(tok.split()[1])
    raise ValueError(f"couldn't parse 'Top' from v4l2-ctl output: {v4l2ctl_output!r}")


def get_roi_y_start(cam_index):
    """Return the currently active y_start (real sensor rows) for this camera."""
    subdev = _subdev_for_camera(cam_index)
    out = subprocess.run(
        ["v4l2-ctl", "-d", subdev, "--get-subdev-selection=pad=0,stream=0,target=crop"],
        capture_output=True, text=True, check=True,
    ).stdout
    return _parse_top(out) - TOP_BASE


def set_roi_y_start(cam_index, y_start):
    """Move the ROI to a new y_start. Safe to call before cam.start() or while
    actively streaming (validated clean in both cases on both cameras and
    both ROI modes -- see CLAUDE.md).

    Returns the y_start the driver actually applied, which may differ from
    the request if it was outside the valid range -- the driver clamps
    rather than erroring, EXCEPT for negative values: a negative y_start
    sent straight to the driver wraps around (confirmed via direct v4l2-ctl
    test, e.g. requesting top=-42 lands at the MAX position, 408, not the
    min, 8 -- looks like unsigned-arithmetic underflow in the driver's own
    clamp, not something this wrapper can fix on the kernel side). So
    negative/over-range values are clamped here in Python before ever
    reaching the driver, rather than relying on its clamp for the low end.
    """
    y_start = max(0, min(y_start, MAX_Y_START))
    subdev = _subdev_for_camera(cam_index)
    top = TOP_BASE + y_start
    subprocess.run(
        ["v4l2-ctl", "-d", subdev, "--set-subdev-selection",
         f"pad=0,stream=0,target=crop,top={top},left=8,"
         f"width={CROP_WIDTH},height={CROP_HEIGHT}"],
        capture_output=True, text=True, check=True,
    )
    return get_roi_y_start(cam_index)


if __name__ == "__main__":
    index = int(sys.argv[1])
    if len(sys.argv) > 2:
        requested = int(sys.argv[2])
        actual = set_roi_y_start(index, requested)
        note = "" if actual == requested else f" (clamped from requested {requested})"
        print(f"camera {index}: y_start={actual}{note}")
    else:
        print(f"camera {index}: y_start={get_roi_y_start(index)}")
