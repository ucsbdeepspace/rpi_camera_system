#!/usr/bin/env python3
"""
Frame-duration sweep for camera_throughput_test.py -- SUBPROCESS orchestrator.

Same rationale as led_dual_camera_sweep_subprocess.py: run each duration in a
completely fresh OS process rather than looping in-process, because a
too-low frame duration can hang camera capture with no catchable Python
exception. A real OS process can be killed from outside with
subprocess.run(timeout=...); an in-process hang cannot be recovered at all.

Default target is MODE_640_200_ROI (640x200) -- its rated ceiling
(588.93fps R8, ~1698us native period) is well below anything in this repo's
frame-duration history, which previously only characterized the stock/
1280x400-family floor (stable at 3400us, hangs at 3228us). This sweep exists
to find this mode's equivalent floor.

Stops at the first timeout or non-clean run and does NOT attempt lower
values afterward -- camera driver state is uncertain after a hang/kill, and
a reboot is recommended before trusting any subsequent runs (manual or
scripted).

Run this from the same directory as camera_throughput_test.py.

Install:  pip install numpy
          (picamera2 is pre-installed on RPi OS Bookworm)
"""

import csv
import re
import subprocess
import sys
import time

# ── Config ───────────────────────────────────────────────────────────────────
# All three overridable via CLI so this same sweep can target any ROI mode:
#   python3 camera_throughput_sweep_subprocess.py [WxH] [durations_csv] [output_csv]
# Defaults below reproduce the original MODE_640_200_ROI-only behavior exactly.
SCRIPT_PATH = "camera_throughput_test.py"
WHICH = "01"  # dual-concurrent, matches the existing 281.8/283.6fps comparisons
RAW_SIZE_ARG = sys.argv[1] if len(sys.argv) > 1 else "640x200"

# Starting from the already-known-stable 3400us point (matches prior modes'
# comparison baseline), stepping down toward this mode's ~1698us native
# ceiling. Smaller steps near the bottom, same philosophy as the LED sweep's
# step pattern -- that's where a request is most likely to fail outright.
if len(sys.argv) > 2:
    FRAME_DURATION_SWEEP_US = [int(v) for v in sys.argv[2].split(",")]
else:
    FRAME_DURATION_SWEEP_US = [
        3400, 3200, 3000, 2800, 2600, 2400, 2200,
        2000, 1900, 1800, 1750, 1725, 1700,
    ]

RUN_TIMEOUT_S = 20.0       # generous margin over the 5s test + camera init
GAP_BETWEEN_RUNS_S = 2.0   # let camera hardware fully release between processes
OUTPUT_CSV = sys.argv[3] if len(sys.argv) > 3 else f"camera_throughput_sweep_{RAW_SIZE_ARG}.csv"

FPS_RE = re.compile(
    r"cam(\d): (\d+) frames in ([\d.]+)s\s+->\s+([\d.]+) fps\s+\(([\d.]+) ms/frame actual\)"
)

ALL_FIELDS = [
    "frame_duration_us", "requested_fps", "run_ok",
    "cam0_fps", "cam0_frames", "cam0_ms_per_frame",
    "cam1_fps", "cam1_frames", "cam1_ms_per_frame",
    "note",
]


def run_one(duration_us: int) -> dict:
    row = {k: "" for k in ALL_FIELDS}
    row["frame_duration_us"] = duration_us
    row["requested_fps"] = round(1_000_000 / duration_us, 2)

    try:
        result = subprocess.run(
            [sys.executable, SCRIPT_PATH, WHICH, str(duration_us), RAW_SIZE_ARG],
            capture_output=True, text=True, timeout=RUN_TIMEOUT_S,
        )
    except subprocess.TimeoutExpired:
        row["run_ok"] = False
        row["note"] = "process timed out and was killed -- treat subsequent runs with suspicion"
        print(f"  TIMEOUT -- process killed after {RUN_TIMEOUT_S}s. "
              f"Camera driver state after this is uncertain.")
        return row

    combined = result.stdout + "\n" + result.stderr
    matches = FPS_RE.findall(combined)
    for idx, frames, secs, fps, ms in matches:
        row[f"cam{idx}_fps"] = float(fps)
        row[f"cam{idx}_frames"] = int(frames)
        row[f"cam{idx}_ms_per_frame"] = float(ms)

    row["run_ok"] = result.returncode == 0 and len(matches) == len(WHICH)
    if result.returncode != 0:
        row["note"] = f"subprocess exited with code {result.returncode}"
    elif len(matches) != len(WHICH):
        row["note"] = "ran but expected fps lines not found (parsing failed or run aborted early)"

    return row


def main():
    print(f"Sweeping {len(FRAME_DURATION_SWEEP_US)} frame durations for RAW_SIZE={RAW_SIZE_ARG} "
          f"via fresh subprocesses\n")

    rows = []
    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=ALL_FIELDS)
        writer.writeheader()

        for duration_us in FRAME_DURATION_SWEEP_US:
            print(f"-- FrameDurationLimits={duration_us}us "
                  f"(requested {1_000_000/duration_us:.1f}fps, fresh process) --")
            row = run_one(duration_us)
            rows.append(row)
            writer.writerow(row)
            f.flush()

            if row["run_ok"]:
                print(f"   achieved: cam0={row['cam0_fps']}fps  cam1={row['cam1_fps']}fps\n")
            else:
                print(f"   FAILED: {row['note']}")
                print(f"   Stopping sweep here -- lower durations not attempted. "
                      f"Camera driver state is uncertain after a failure; reboot before "
                      f"trying lower values manually.\n")
                break

            time.sleep(GAP_BETWEEN_RUNS_S)

    print(f"Saved {len(rows)} rows to {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
