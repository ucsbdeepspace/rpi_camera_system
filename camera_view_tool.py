#!/usr/bin/env python3
"""
Full-frame / windowed-ROI live camera viewer with beam centroid tracking --
one or two cameras, raw mono stream. Built for bench alignment work (e.g.
watching a beam through a beamsplitter): starts on the full sensor field of
view, and 'h' steps down through the patched driver's BINNED windowed ROI
modes for a tighter, faster-updating view of the beam.

Uses the binned ROI modes (MODE_640_200_ROI / MODE_640_100_ROI), not the
unbinned ones (MODE_1280_400_ROI / MODE_1280_200_ROI) this tool originally
used -- the unbinned modes were found to invert bright point sources under
sustained streaming at their validated floor durations (a continuously-
active internal sensor auto-calibration engine with no register-level fix;
see CLAUDE.md's "unbinned ROI modes invert bright point sources" section).
The binned modes are confirmed clean under the same sustained-run test and
are also faster, so there's no tradeoff in switching.

Each frame's raw pixel values (not the display-normalized 8-bit copy) are
checked for a confident beam peak -- (max - median) must clear a multiple
of the frame's own noise std, so a weak/no-signal frame (beam moved out of
this crop, momentarily dim, etc.) is correctly reported as "no beam this
frame" instead of the detector locking onto whatever background speckle
happens to be relatively brightest. When a peak is confident, the frame is
masked at median + k*std and the contour actually containing the frame's
brightest pixel is taken as the beam blob (not just "largest contour by
area" -- background speckle can form a larger low-contrast blob than the
real, smaller, much-brighter spot, see the "Beam detection" note below for
the live bench test that found this the hard way). Overlaid with a reticle
(ring + tick marks + center dot) at the blob's intensity-weighted centroid
(raw pixel values, not display-normalized). A live per-camera fps counter
(rolling average of true capture timestamps, not display rate -- see the
throttled-display note below) is shown in the corner.

Beam detection note (found 2026-07-15 testing 'h' on the actual bench
signal): the first cut of this thresholded on the display-normalized 8-bit
image via Otsu. That worked for the full-sensor view (a small, genuinely
much brighter spot against a mostly-dark background is a clean bimodal
split) but broke as soon as 'h' cropped down to a windowed ROI mode: the
background within that narrower vertical band was already fairly bright/
uniform speckle (median ~3748, max only ~5888 -- nowhere near the full
frame's ~65280 peak, i.e. the true peak happened to not be very prominent
within that particular crop at that moment), so Otsu split the frame
roughly in half and centroided on the larger, low-contrast speckle mass
instead of a real beam feature -- confirmed via saved raw frames and an
offline comparison, not just guessed at. The raw-value/confidence-gated
approach above was verified against both the good (full-sensor, strong
peak) and bad (windowed, weak peak) captured frames before being adopted
here: it finds the same centroid as before on the good frame, and correctly
reports no detection at all on the weak one instead of drawing a
misleading reticle.

Controls:
  h   cycle mode: 1280x800 (full sensor) -> 640x200 (binned) -> 640x100
      (binned) -> back to full sensor. Each step is a full stop/reconfigure/
      start (a different sensor mode, not a live crop move -- see CLAUDE.md's
      "Height/size changes are excluded" note on why). The two binned tiers
      use the patched driver's MODE_640_200_ROI / MODE_640_100_ROI, both
      runtime-repositionable via set_selection -- every time you cycle into
      one, this script immediately centers the new (narrower) window on
      wherever that camera's beam centroid was last seen, no manual
      re-aiming needed. Full sensor isn't ROI-adjustable, so cycling back to
      it is just a mode switch with no centering step.
  t   toggle auto-track: while on, the ROI is re-centered on the current
      centroid at most every ANALYSIS_INTERVAL_S (a live set_selection
      push, same mechanism as the initial mode-switch centering, just
      repeated continuously instead of once). Has no effect in full-sensor
      mode (nothing to move). Detection and recentering both run on their
      own ~20Hz throttle, decoupled from both the raw capture rate
      (find_beam_blob is too expensive to run on every one of 500-900
      frames/sec) and the ~15Hz display throttle (tying auto-track to that
      was the original "why is this so slow" bug). Each recenter blocks the
      capture loop for ~7-10ms (a v4l2-ctl subprocess call), so tracking
      does cost some fps while active -- measured ~525fps -> ~220-230fps at
      the old 50Hz rate; 20Hz keeps that hit much smaller. See
      ANALYSIS_INTERVAL_S if you want to trade responsiveness for fps or
      vice versa.
  q   quit

Capture runs unconditionally every loop iteration (this is what the fps
counter reflects); the heavier per-frame work -- normalize, threshold,
contour/centroid, imshow, waitKey -- is throttled to ~15Hz so a slow
display never caps real capture throughput (same fix applied to
roi_live_demo.py after its fps turned out to be display-bound -- see
CLAUDE.md's "runtime-movable ROI" section, item 12).

Requires the patched ov9282 module (MODE_640_200_ROI / MODE_640_100_ROI)
to be loaded -- see CLAUDE.md.

Usage:
  python3 camera_view_tool.py

Install:  pip install opencv-python numpy
          (picamera2 is pre-installed on RPi OS Bookworm)
"""
import time
from collections import deque

import cv2
import numpy as np
from picamera2 import Picamera2

from roi_set_selection import get_max_y_start, set_roi_y_start

RAW_FORMAT = "R8"
EXPOSURE_US = 1500
ANALOGUE_GAIN = 4.0

PIXEL_ARRAY_HEIGHT = 800  # full sensor height, in real (pre-bin) sensor rows

# 'h' cycle order: full sensor -> half-tier binned -> quarter-tier binned -> full.
# Binned (640-wide), not unbinned (1280-wide) -- the unbinned windowed-crop
# modes invert bright point sources under sustained streaming (see CLAUDE.md),
# the binned ones are confirmed clean and are also faster.
SIZES = [(1280, 800), (640, 200), (640, 100)]
# The 640-wide modes bin 2:1 in BOTH dimensions, not just horizontally --
# confirmed empirically (not assumed): rpicam-hello itself reports 640x200's
# real sensor crop as 1280x400 (double the output in each axis), and a
# direct y_start-shift test measured output-row displacement at ~half the
# requested pre-bin row shift for both 640x200 and 640x100 (ratio 1.94 and
# 2.00 respectively). This matters because y_start (from roi_set_selection)
# is in pre-bin sensor rows while a detected centroid's row is in the
# captured (post-bin) frame's coordinate space -- converting between them
# needs this ratio, or centering math silently mismatches units by 2x.
V_BIN_RATIO_BY_SIZE = {
    (1280, 800): 1,  # full sensor, not ROI-adjustable, ratio unused
    (640, 200): 2,
    (640, 100): 2,
}
FRAME_DURATION_US_BY_SIZE = {
    (1280, 800): 6000,  # full sensor -- conservative, floor not characterized
    (640, 200): 1800,   # MODE_640_200_ROI validated floor (CLAUDE.md)
    (640, 100): 1050,   # MODE_640_100_ROI validated floor (CLAUDE.md)
}

MIN_BLOB_AREA_PX = 15  # ignore contours smaller than this -- rejects single
                         # hot-pixel/noise specks so they don't get circled
                         # as though they were the beam
CONTRAST_CONFIDENCE_K = 5.0  # require (max - median) > this * std to call a
                               # frame's peak "confident" -- see the "Beam
                               # detection note" in the module docstring;
                               # calibrated against one real strong-peak frame
                               # (passed easily) and one real weak-peak frame
                               # (correctly rejected) from the actual bench
                               # signal, not picked arbitrarily
MASK_THRESH_K = 3.0  # once a frame passes the confidence gate, flag pixels
                       # >= median + this * std as part of the beam blob
FPS_WINDOW = 30
DISPLAY_INTERVAL_S = 1 / 15  # redraw + poll keys at ~15Hz regardless of capture rate
ANALYSIS_INTERVAL_S = 0.05  # beam detection + auto-track recenter run at most
                              # this often (~20Hz) -- gated on wall-clock time,
                              # decoupled from both the raw capture rate
                              # (500-900fps -- find_beam_blob's median/std/
                              # contour work is NOT cheap enough to run on
                              # every one of those frames; measured live:
                              # doing so dropped 640x200 from ~530fps to
                              # ~339fps on its own) and from the ~15Hz
                              # DISPLAY_INTERVAL_S (tying auto-track to the
                              # display throttle was the original "why is
                              # this so slow" bug -- ~333ms/correction).
                              #
                              # This is a real throughput/responsiveness
                              # trade-off, not a free decoupling: each
                              # recenter blocks the single capture loop for
                              # set_roi_y_start's measured ~7-10ms subprocess
                              # cost. At 50Hz (0.02s) that's up to ~45% of
                              # the loop's time when the beam is tracked
                              # continuously -- measured live, it dropped
                              # 640x200 from ~525fps to ~220-230fps. 20Hz
                              # keeps the fps hit much smaller while still
                              # recentering ~16x faster than the original
                              # ~333ms bug -- plenty responsive for a bench
                              # alignment/monitoring tool. Lower this if
                              # faster tracking matters more than fps for a
                              # given use, higher if the reverse.

# ── Detect available cameras ────────────────────────────────────────────────
info = Picamera2.global_camera_info()
print(f"Detected {len(info)} camera(s):")
for i, cam_info in enumerate(info):
    print(f"  [{i}] {cam_info}")

if not info:
    print("\nNo cameras detected -- this is a config/hardware issue, not a "
          "script issue. Check `rpicam-hello --list-cameras` outside Python.")
    raise SystemExit(1)

indices = list(range(len(info)))  # opens all detected cameras, 1 or 2


def configure_and_start(cam, index, raw_size):
    config = cam.create_video_configuration(
        main={"size": (64, 48), "format": "RGB888"},  # required by the API,
                                                         # never displayed
        raw={"size": raw_size, "format": RAW_FORMAT},
        buffer_count=2,  # confirmed better than 1 -- see project notes
    )
    cam.configure(config)

    actual_raw = cam.camera_configuration()["raw"]
    if tuple(actual_raw["size"]) != raw_size:
        print(f"WARNING camera {index}: requested {raw_size} got {actual_raw['size']} "
              f"-- the requested mode was NOT selected as expected.")

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
        print(f"  (skipping control not advertised on camera {index}: {k})")
        del controls[k]
    cam.set_controls(controls)


def make_camera(index):
    cam = Picamera2(index)
    configure_and_start(cam, index, SIZES[0])
    return cam


def apply_y_start(index, target):
    """Push index's y_start and verify it actually landed, retrying briefly.

    Needed because cam.start() can return before the driver has fully
    settled into a freshly-selected pad format, so a set_selection pushed
    immediately after can silently no-op (same issue found and worked
    around in roi_live_demo.py's apply_y_start).
    """
    max_y_start = get_max_y_start(index)
    expected = max(0, min(target, max_y_start))
    expected -= expected % 4  # driver rounds down to a 4-row boundary --
                                # match that here, or a non-aligned target
                                # (the normal case for a centroid-derived
                                # value) never matches `expected` and this
                                # burns all 10 retries (~500ms) every single
                                # call. Harmless as an occasional one-off
                                # (cycle_height calls this once per mode
                                # switch) but catastrophic once auto-track
                                # started calling it continuously -- found
                                # live, tanked fps from ~530 to ~6.
    for _ in range(10):
        y_starts[index] = set_roi_y_start(index, target)
        if y_starts[index] == expected:
            return
        time.sleep(0.05)
    print(f"WARNING camera {index}: y_start did not settle at {target}, "
          f"landed at {y_starts[index]}")


def cycle_height():
    """Step to the next mode/size, reconfiguring every camera, then --
    for the two ROI-adjustable tiers -- center each camera's new window
    on wherever ITS OWN beam centroid was last seen (falling back to the
    sensor center if no beam has been detected yet this session)."""
    global size_idx
    size_idx = (size_idx + 1) % len(SIZES)
    new_size = SIZES[size_idx]
    new_height = new_size[1]

    for i, cam in zip(indices, cams):
        cam.stop()
        configure_and_start(cam, i, new_size)
        raw_sizes[i] = new_size
        cv2.resizeWindow(window_names[i], max(new_size[0], 480), max(new_size[1], 200))
        fps_history[i].clear()
        last_analysis_time[i] = 0.0
        last_found[i] = None

    for i in indices:
        if new_height == PIXEL_ARRAY_HEIGHT:
            y_starts[i] = 0  # full sensor -- nothing to center
            continue
        center = last_centroid_abs_y[i] if last_centroid_abs_y[i] is not None \
            else PIXEL_ARRAY_HEIGHT // 2
        # Half the PRE-BIN crop height, not half the output height -- the
        # window spans new_height * V_BIN_RATIO real sensor rows even
        # though only new_height rows come out the other end.
        pre_bin_height = new_height * V_BIN_RATIO_BY_SIZE[new_size]
        apply_y_start(i, int(round(center - pre_bin_height / 2)))

    tag = "full sensor" if new_height == PIXEL_ARRAY_HEIGHT else "centered on last beam position"
    print(f"mode -> {new_size[0]}x{new_size[1]} ({tag})  y_starts={y_starts}")


def find_beam_blob(frame):
    """Return (cx, cy, radius) in frame-local pixel coords for the beam
    spot, or None if this frame has no confident peak. See the module
    docstring's "Beam detection note" for why this works on raw values with
    a confidence gate instead of Otsu-on-normalized-8-bit."""
    median = float(np.median(frame))
    std = float(frame.std())
    peak = float(frame.max())
    if std == 0 or (peak - median) < CONTRAST_CONFIDENCE_K * std:
        return None

    mask = (frame >= median + MASK_THRESH_K * std).astype(np.uint8) * 255
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # The contour containing the frame's single brightest pixel, not just
    # "largest by area" -- a broad, low-contrast speckle patch can outsize
    # the real (smaller, much brighter) beam blob in raw pixel count.
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


def draw_reticle(img, center, radius):
    """Ring + four scope-style tick marks + a center dot, each stroke drawn
    with a black outline underneath so it stays legible against both the
    black background and the bright blob itself -- more useful (and more
    interesting to look at) than a single plain circle."""
    cx, cy = center
    r = max(int(round(radius)), 8)

    def stroke(draw_fn, *pts, color, thickness):
        draw_fn(img, *pts, (0, 0, 0), thickness + 2, cv2.LINE_AA)
        draw_fn(img, *pts, color, thickness, cv2.LINE_AA)

    ring_color = (255, 255, 0)   # cyan
    tick_color = (0, 255, 255)   # yellow
    dot_color = (0, 0, 255)      # red

    stroke(cv2.circle, (cx, cy), r, color=ring_color, thickness=2)

    tick_len = max(int(r * 0.5), 6)
    gap = 3
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        p1 = (cx + dx * (r + gap), cy + dy * (r + gap))
        p2 = (cx + dx * (r + gap + tick_len), cy + dy * (r + gap + tick_len))
        stroke(cv2.line, p1, p2, color=tick_color, thickness=2)

    stroke(cv2.circle, (cx, cy), 3, color=dot_color, thickness=-1)


print(f"\nOpening {len(indices)} camera(s)... RAW_SIZE={SIZES[0]}")
cams = [make_camera(i) for i in indices]

window_names = [f"Camera {i} -- h: cycle mode, t: auto-track, q: quit" for i in indices]
for i, name in zip(indices, window_names):
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, *SIZES[0])

size_idx = 0
raw_sizes = {i: SIZES[0] for i in indices}
y_starts = {i: 0 for i in indices}
last_centroid_abs_y = {i: None for i in indices}  # real sensor row, updated
                                                     # whenever a blob is found
fps_history = {i: deque(maxlen=FPS_WINDOW) for i in indices}
last_display_time = {i: 0.0 for i in indices}
last_waitkey_time = 0.0
auto_track = False
last_analysis_time = {i: 0.0 for i in indices}
last_found = {i: None for i in indices}  # most recent find_beam_blob() result
                                            # per camera, so the (throttled)
                                            # display block always has
                                            # something to draw even on
                                            # iterations where detection
                                            # itself didn't run

print("Streaming -- 'h' cycles mode (1280x800 -> 640x200 -> 640x100 -> 1280x800), "
      "'t' toggles auto-track, 'q' quits.\n")

try:
    while True:
        for i, cam in zip(indices, cams):
            # Capture every iteration, unconditionally, so the fps counter
            # reflects true capture speed -- the display/analysis work below
            # is throttled instead (see module docstring).
            frame = cam.capture_array("raw").view(np.uint16)
            now = time.monotonic()
            fps_history[i].append(now)

            # Beam detection + auto-track recenter run on their own ~20Hz
            # throttle (ANALYSIS_INTERVAL_S) -- fast enough to keep up with
            # a moving beam, but decoupled from both the raw capture rate
            # (find_beam_blob is too expensive to run on every one of
            # 500-900 frames/sec, measured live) and the ~15Hz display
            # throttle below (tying auto-track to that was the original
            # "why is this so slow" bug: ~333ms/correction).
            if now - last_analysis_time[i] >= ANALYSIS_INTERVAL_S:
                last_analysis_time[i] = now
                last_found[i] = find_beam_blob(frame)
                if last_found[i] is not None:
                    cx, cy, radius = last_found[i]
                    # cy is in the captured (post-bin) frame's row space;
                    # y_starts[i] is in real pre-bin sensor rows. The
                    # 640-wide modes bin 2:1 vertically as well as
                    # horizontally (confirmed empirically, see
                    # V_BIN_RATIO_BY_SIZE), so cy must be scaled up to
                    # pre-bin rows before adding -- treating them as the
                    # same unit was a real bug (silently off by 2x) until
                    # this was checked.
                    v_bin = V_BIN_RATIO_BY_SIZE[raw_sizes[i]]
                    last_centroid_abs_y[i] = y_starts[i] + cy * v_bin

                    if auto_track and raw_sizes[i][1] != PIXEL_ARRAY_HEIGHT:
                        pre_bin_height = raw_sizes[i][1] * v_bin
                        target = int(round(last_centroid_abs_y[i] - pre_bin_height / 2))
                        apply_y_start(i, target)

            if now - last_display_time[i] < DISPLAY_INTERVAL_S:
                continue
            last_display_time[i] = now

            # Normalize for display only -- raw sensor values often sit in a
            # low range that reads as solid black otherwise. This never
            # touches the underlying data, just stretches it for viewing.
            norm = np.empty_like(frame, dtype=np.uint8)
            cv2.normalize(frame, norm, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            display = cv2.cvtColor(norm, cv2.COLOR_GRAY2BGR)

            if last_found[i] is not None:
                cx, cy, radius = last_found[i]
                marker_pt = (int(round(cx)), int(round(cy)))
                draw_reticle(display, marker_pt, radius)
                cv2.putText(display, f"({cx:.1f}, {cy:.1f})",
                            (marker_pt[0] + 12, marker_pt[1] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

            hist = fps_history[i]
            fps = (len(hist) - 1) / (hist[-1] - hist[0]) if len(hist) > 1 and hist[-1] != hist[0] else 0.0
            h = raw_sizes[i][1]
            roi_tag = "full sensor" if h == PIXEL_ARRAY_HEIGHT else f"y_start={y_starts[i]}"
            track_tag = "  [TRACK]" if auto_track and h != PIXEL_ARRAY_HEIGHT else ""
            cv2.putText(display, f"cam {i}: {raw_sizes[i][0]}x{h}  {roi_tag}  {fps:.1f}fps{track_tag}",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

            cv2.imshow(window_names[i], display)

        # waitKey's nominal "1ms" wait costs much more in practice on this
        # GTK backend (measured ~2.2ms/call with two windows open, ~69% of
        # the loop when called every capture iteration -- see CLAUDE.md item
        # 12). Throttling it to the same cadence as the display redraw keeps
        # keyboard response well under human reaction time without capping
        # capture throughput.
        now = time.monotonic()
        if now - last_waitkey_time < DISPLAY_INTERVAL_S:
            continue
        last_waitkey_time = now

        # Drain every pending key this throttle window, not just one --
        # waitKey() only ever returns a single queued key per call, so at
        # ~15Hz cadence two keys pressed faster than ~67ms apart would
        # otherwise strand the second one until the next poll.
        quit_requested = False
        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == 0xFF:
                break
            if key == ord('q'):
                quit_requested = True
                break
            elif key == ord('h'):
                cycle_height()
            elif key == ord('t'):
                auto_track = not auto_track
                for i in indices:
                    last_analysis_time[i] = 0.0
                print(f"auto-track {'ON' if auto_track else 'OFF'}")
        if quit_requested:
            break

finally:
    cv2.destroyAllWindows()
    for cam in cams:
        cam.stop()
