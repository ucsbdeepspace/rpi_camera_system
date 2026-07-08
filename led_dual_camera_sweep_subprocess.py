#!/usr/bin/env python3
"""
Dual-camera frame-duration sweep -- SUBPROCESS orchestrator.

Runs led_dual_camera_closed_loop_test_mp.py as a completely fresh OS
process for each frame duration value, instead of trying to cycle
through durations inside one long-lived process. This exists because
that in-process approach was tried twice (live set_controls(), then a
full stop/reconfigure/start cycle) and failed identically both times on
the very first duration change -- round 1 (the object's first-ever
configuration) always worked, every subsequent reconfiguration of the
same Picamera2 object within the same process did not. A fresh process
means every run IS "round 1" from the camera's perspective, which is the
only state that's been reliable so far.

Bonus: a real OS process can actually be killed from outside if it hangs
(subprocess.run(timeout=...) + terminate/kill), unlike an in-process
camera call that hangs the whole interpreter with no catchable
exception. This doesn't guarantee the underlying camera driver recovers
cleanly after being killed mid-capture -- if a run times out, treat
subsequent runs with suspicion and consider a full reboot before
continuing.

Run this from the same directory as led_dual_camera_closed_loop_test_mp.py.

Install:  pip install lgpio numpy
          (picamera2 is pre-installed on RPi OS Bookworm)
"""

import csv
import re
import subprocess
import sys
import time

# ── Config ────────────────────────────────────────────────────────────────────
SCRIPT_PATH = "led_dual_camera_closed_loop_test_mp.py"

# Extended toward the known single-camera floor (3228us). Smaller steps
# near the bottom since that's where a request can fail outright -- if a
# run times out (see RUN_TIMEOUT_S below), stop the sweep there rather
# than continuing further down; the fresh-process design means a failed
# run doesn't corrupt earlier data, but it doesn't guarantee the camera
# driver is clean for whatever comes after it either.
FRAME_DURATION_SWEEP_US = [6000, 5500, 5000, 4500, 4000, 3800, 3600, 3400, 3228]

RUN_TIMEOUT_S = 30.0   # generous margin over the ~5s test + camera init overhead
GAP_BETWEEN_RUNS_S = 2.0  # let camera hardware fully release between processes
OUTPUT_CSV = "dual_camera_subprocess_sweep.csv"

# ── Parsing ──────────────────────────────────────────────────────────────────
CALIB_RE = re.compile(
    r"cam(\d): ON=([\d.]+) OFF=([\d.]+) threshold=([\d.]+) \((ok|POOR SEPARATION)\)"
)
TRANSITIONS_RE = re.compile(
    r"Transitions with both cameras confirming: (\d+)\s+\(timeouts: (\d+)\)"
)
LATENCY_RE = re.compile(
    r"cam(\d) latency: mean=([-\d.]+) ms\s+std=([-\d.]+) ms\s+"
    r"min=([-\d.]+) ms\s+max=([-\d.]+) ms"
)
SKEW_RE = re.compile(
    r"Inter-camera skew \(cam0 - cam1\): mean=([+\-\d.]+) ms\s+std=([\d.]+) ms\s+"
    r"min=([+\-\d.]+) ms\s+max=([+\-\d.]+) ms"
)
SKEW_MAX_RE = re.compile(r"\|skew\| max: ([\d.]+) ms")
FREQ_RE = re.compile(r"Effective closed-loop toggle frequency \(both-confirmed\): ([\d.]+) Hz")
ACHIEVED_FPS_RE = re.compile(
    r"cam(\d) achieved during closed loop: (\d+) frames in ([\d.]+)s\s+->\s+([\d.]+) fps"
)


def parse_output(text: str) -> dict:
    row = {}
    for m in CALIB_RE.finditer(text):
        idx, on, off, thresh, status = m.groups()
        row[f"cam{idx}_on"] = float(on)
        row[f"cam{idx}_off"] = float(off)
        row[f"cam{idx}_threshold"] = float(thresh)
        row[f"cam{idx}_valid"] = (status == "ok")

    m = TRANSITIONS_RE.search(text)
    if m:
        row["transitions"] = int(m.group(1))
        row["timeouts"] = int(m.group(2))

    for m in LATENCY_RE.finditer(text):
        idx, mean, std, lmin, lmax = m.groups()
        row[f"cam{idx}_lat_mean_ms"] = float(mean)
        row[f"cam{idx}_lat_std_ms"] = float(std)
        row[f"cam{idx}_lat_min_ms"] = float(lmin)
        row[f"cam{idx}_lat_max_ms"] = float(lmax)

    m = SKEW_RE.search(text)
    if m:
        mean, std, smin, smax = m.groups()
        row["skew_mean_ms"] = float(mean)
        row["skew_std_ms"] = float(std)
        row["skew_min_ms"] = float(smin)
        row["skew_max_ms"] = float(smax)

    m = SKEW_MAX_RE.search(text)
    if m:
        row["skew_abs_max_ms"] = float(m.group(1))

    for m in ACHIEVED_FPS_RE.finditer(text):
        idx, frames, secs, fps = m.groups()
        row[f"cam{idx}_achieved_fps"] = float(fps)
        row[f"cam{idx}_achieved_frames"] = int(frames)

    m = FREQ_RE.search(text)
    if m:
        row["freq_hz"] = float(m.group(1))

    return row


ALL_FIELDS = [
    "frame_duration_us", "run_ok",
    "cam0_on", "cam0_off", "cam0_threshold", "cam0_valid",
    "cam1_on", "cam1_off", "cam1_threshold", "cam1_valid",
    "transitions", "timeouts",
    "cam0_achieved_fps", "cam0_achieved_frames",
    "cam1_achieved_fps", "cam1_achieved_frames",
    "cam0_lat_mean_ms", "cam0_lat_std_ms", "cam0_lat_min_ms", "cam0_lat_max_ms",
    "cam1_lat_mean_ms", "cam1_lat_std_ms", "cam1_lat_min_ms", "cam1_lat_max_ms",
    "skew_mean_ms", "skew_std_ms", "skew_min_ms", "skew_max_ms", "skew_abs_max_ms",
    "freq_hz", "note",
]


def run_one(duration_us: int) -> dict:
    row = {k: "" for k in ALL_FIELDS}
    row["frame_duration_us"] = duration_us

    try:
        result = subprocess.run(
            [sys.executable, SCRIPT_PATH, str(duration_us)],
            capture_output=True, text=True, timeout=RUN_TIMEOUT_S,
        )
    except subprocess.TimeoutExpired:
        row["run_ok"] = False
        row["note"] = "process timed out and was killed -- treat subsequent runs with suspicion"
        print(f"  TIMEOUT -- process killed after {RUN_TIMEOUT_S}s. "
              f"Camera driver state after this is uncertain.")
        return row

    combined = result.stdout + "\n" + result.stderr
    parsed = parse_output(combined)
    row.update(parsed)
    row["run_ok"] = result.returncode == 0 and "freq_hz" in parsed
    if result.returncode != 0:
        row["note"] = f"subprocess exited with code {result.returncode}"
    elif "freq_hz" not in parsed:
        row["note"] = "ran but summary line not found (parsing failed or run aborted early)"

    return row


def main():
    print(f"Sweeping {len(FRAME_DURATION_SWEEP_US)} frame durations via fresh subprocesses\n")

    rows = []
    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=ALL_FIELDS)
        writer.writeheader()

        for duration_us in FRAME_DURATION_SWEEP_US:
            print(f"-- FrameDurationLimits={duration_us}us (fresh process) --")
            row = run_one(duration_us)
            rows.append(row)
            writer.writerow(row)
            f.flush()

            if row["run_ok"]:
                print(f"   achieved: cam0={row['cam0_achieved_fps']}fps cam1={row['cam1_achieved_fps']}fps "
                      f"(vs requested {1_000_000/duration_us:.1f}fps)")
                print(f"   cam0_lat mean={row['cam0_lat_mean_ms']}ms max={row['cam0_lat_max_ms']}ms  "
                      f"cam1_lat mean={row['cam1_lat_mean_ms']}ms max={row['cam1_lat_max_ms']}ms")
                print(f"   skew_mean={row['skew_mean_ms']}ms |skew|max={row['skew_abs_max_ms']}ms  "
                      f"freq={row['freq_hz']}Hz\n")
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
