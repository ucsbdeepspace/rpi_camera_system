#!/usr/bin/env python3
"""
Pure camera throughput test -- no LED, no detection, just raw capture rate.

Measures actual achieved frames-per-second for one or both cameras over a
fixed window, at a given requested frame duration. No GPIO, no closed-loop
logic -- just capture_array("raw") in a tight loop and count frames.

The point: compare SOLO achieved fps (one camera running alone) against
DUAL achieved fps (both cameras running at once, same settings) at the
same requested frame duration. If dual is meaningfully lower than solo,
that's direct evidence of a shared bandwidth/pipeline ceiling -- not an
artifact of the "wait for both" closed-loop test's order statistics.

Usage:
  python3 camera_throughput_test.py 0                  # solo, camera 0
  python3 camera_throughput_test.py 1                  # solo, camera 1
  python3 camera_throughput_test.py 01                  # both, concurrently
  python3 camera_throughput_test.py 01 4000              # both, at 4000us frame duration
  python3 camera_throughput_test.py 01 2000 640x200     # both, ROI mode, 2000us

Install:  pip install numpy
          (picamera2 is pre-installed on RPi OS Bookworm)
"""

import multiprocessing as mp
import sys
import time

RAW_FORMAT = "R8"
EXPOSURE_US = 1500
ANALOGUE_GAIN = 4.0
TEST_DURATION_S = 5.0

which = sys.argv[1] if len(sys.argv) > 1 else "01"
duration_us = int(sys.argv[2]) if len(sys.argv) > 2 else 4000
if len(sys.argv) > 3:
    w, h = sys.argv[3].lower().split("x")
    RAW_SIZE = (int(w), int(h))
else:
    RAW_SIZE = (640, 400)
indices = [int(c) for c in which]


def worker(index, duration_us, result_queue):
    from picamera2 import Picamera2

    cam = Picamera2(index)
    config = cam.create_video_configuration(
        main={"size": (64, 48), "format": "RGB888"},
        raw={"size": RAW_SIZE, "format": RAW_FORMAT},
        buffer_count=2,
    )
    cam.configure(config)
    cam.start()

    controls = {
        "FrameDurationLimits": (duration_us, duration_us),
        "AeEnable": False,
        "NoiseReductionMode": 0,
        "ExposureTime": EXPOSURE_US,
        "AnalogueGain": ANALOGUE_GAIN,
    }
    unsupported = [k for k in controls if k not in cam.camera_controls]
    for k in unsupported:
        del controls[k]
    cam.set_controls(controls)
    time.sleep(1.0)

    count = 0
    start = time.monotonic()
    while time.monotonic() - start < TEST_DURATION_S:
        cam.capture_array("raw")
        count += 1
    elapsed = time.monotonic() - start

    cam.stop()
    result_queue.put((index, count, elapsed))


def main():
    label = "SOLO" if len(indices) == 1 else "DUAL (concurrent)"
    print(f"{label} throughput test -- camera(s) {indices}, RAW_SIZE={RAW_SIZE}, "
          f"requested frame duration {duration_us}us, {TEST_DURATION_S}s each\n")

    ctx = mp.get_context("fork")
    result_queue = ctx.Queue()
    procs = []
    for idx in indices:
        p = ctx.Process(target=worker, args=(idx, duration_us, result_queue))
        p.start()
        procs.append(p)

    results = {}
    for _ in indices:
        idx, count, elapsed = result_queue.get()
        results[idx] = (count, elapsed)

    for p in procs:
        p.join(timeout=10.0)

    print("── Results ──────────────────────────────")
    for idx in indices:
        count, elapsed = results[idx]
        fps = count / elapsed if elapsed else float("nan")
        period_ms = 1000.0 / fps if fps else float("nan")
        print(f"  cam{idx}: {count} frames in {elapsed:.2f}s  ->  "
              f"{fps:.2f} fps  ({period_ms:.3f} ms/frame actual)")
    print("─────────────────────────────────────────")


if __name__ == "__main__":
    main()
