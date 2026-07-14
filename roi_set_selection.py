#!/usr/bin/env python3
"""
Runtime ROI (vertical position) control for the patched ov9282 driver's
`set_selection` support -- see CLAUDE.md's "runtime-movable ROI" section for
the driver-side design and validation history.

Picamera2 has no high-level API for arbitrary sensor crop, so this shells
out to `v4l2-ctl` subdev ioctls -- the same approach used by the one-off
test scripts that validated this design (pre-stream moves, and moves while
a capture is already running, on both cameras and both windowed ROI modes).

Only `y_start` (vertical crop position) is adjustable, and only for the four
windowed-crop ROI modes (MODE_1280_400_ROI, MODE_640_200_ROI,
MODE_1280_200_ROI, MODE_640_100_ROI). The real (pre-bin) sensor crop window
height differs by mode -- 400 rows for the first two, 200 for the newer
pair -- so the valid y_start range differs too; this module queries the
live crop height from the driver on every call rather than hardcoding it,
so it works for whichever mode is currently configured without needing to
know about it in advance. Width, output height, and binning are NOT
adjustable here (see CLAUDE.md for why: changing them needs a full
reconfigure, not a live crop move, and there is no hardware-verified
register formula for horizontal panning).

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

PIXEL_ARRAY_HEIGHT = 800  # full sensor height every ROI mode crops within
TOP_BASE = 8  # fixed ISP offset, independent of y_start (same at every stock mode)


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


def _get_crop(cam_index):
    """Read the subdev's live crop rect (Left/Top/Width/Height), fresh every
    call. Height is the real, mode-dependent sensor window -- 400 for
    MODE_1280_400_ROI/MODE_640_200_ROI, 200 for MODE_1280_200_ROI/
    MODE_640_100_ROI -- so this is what lets the rest of this module avoid
    hardcoding which mode is active.
    """
    subdev = _subdev_for_camera(cam_index)
    out = subprocess.run(
        ["v4l2-ctl", "-d", subdev, "--get-subdev-selection=pad=0,stream=0,target=crop"],
        capture_output=True, text=True, check=True,
    ).stdout
    # v4l2-ctl's output has an "ioctl: ..." line before the data line and a
    # trailing "Flags: " field with no value, both of which contain commas/
    # tokens that aren't clean "Key value" pairs -- only pick out the four
    # fields this module actually needs, same defensive approach the
    # original single-field _parse_top used.
    crop = {}
    for tok in out.split(","):
        tok = tok.strip()
        for key in ("Left", "Top", "Width", "Height"):
            if tok.startswith(key + " "):
                crop[key] = int(tok.split()[1])
    return crop


def get_max_y_start(cam_index):
    """Valid y_start upper bound for whichever mode is currently configured
    on this camera -- 400 for the two original ROI modes, 600 for the newer
    pair (their real crop window is 200 rows, not 400)."""
    return PIXEL_ARRAY_HEIGHT - _get_crop(cam_index)["Height"]


def get_roi_y_start(cam_index):
    """Return the currently active y_start (real sensor rows) for this camera."""
    return _get_crop(cam_index)["Top"] - TOP_BASE


def set_roi_y_start(cam_index, y_start):
    """Move the ROI to a new y_start. Safe to call before cam.start() or while
    actively streaming (validated clean in both cases on both cameras and
    all four ROI modes -- see CLAUDE.md).

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
    crop = _get_crop(cam_index)
    max_y_start = PIXEL_ARRAY_HEIGHT - crop["Height"]
    y_start = max(0, min(y_start, max_y_start))
    subdev = _subdev_for_camera(cam_index)
    top = TOP_BASE + y_start
    subprocess.run(
        ["v4l2-ctl", "-d", subdev, "--set-subdev-selection",
         f"pad=0,stream=0,target=crop,top={top},left={crop['Left']},"
         f"width={crop['Width']},height={crop['Height']}"],
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
        print(f"camera {index}: y_start={get_roi_y_start(index)} "
              f"(max={get_max_y_start(index)})")
