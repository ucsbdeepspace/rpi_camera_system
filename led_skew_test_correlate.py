#!/usr/bin/env python3
"""
Two-process skew test -- CORRELATE (run after both driver and logger finish).

Reads skew_test_driver_cam0.csv (transitions cam0 detected, with its own
command/detect timestamps) and skew_test_logger_cam1.csv (every frame
cam1 saw, raw), computes cam1's threshold from the full log's min/max,
detects cam1's own transitions independently, then matches each driver
transition to the nearest same-direction logger transition to get skew
per transition -- this time with no shared-process threading involved.

Usage: python3 led_skew_test_correlate.py
"""

import csv
import statistics

DRIVER_CSV = "skew_test_driver_cam0.csv"
LOGGER_CSV = "skew_test_logger_cam1.csv"
MATCH_WINDOW_S = 0.05   # don't match transitions more than 50ms apart -- guards
                         # against pairing unrelated events if one camera missed one

# ── Load driver transitions ─────────────────────────────────────────────────
driver_rows = []
with open(DRIVER_CSV, newline="") as f:
    for row in csv.DictReader(f):
        driver_rows.append({
            "target_state": int(row["target_state"]),
            "detect_time": float(row["detect_time"]),
            "latency_ms": float(row["latency_ms"]),
        })
print(f"Loaded {len(driver_rows)} driver (cam0) transitions")

# ── Load logger frames, compute threshold, detect transitions ──────────────
logger_log = []
with open(LOGGER_CSV, newline="") as f:
    for row in csv.DictReader(f):
        logger_log.append((float(row["timestamp"]), float(row["brightness"])))
print(f"Loaded {len(logger_log)} logger (cam1) frames")

brightness_vals = [b for _, b in logger_log]
lo, hi = min(brightness_vals), max(brightness_vals)
threshold = (lo + hi) / 2.0
print(f"cam1 brightness range: {lo:.2f}-{hi:.2f}  threshold={threshold:.2f}\n")


def detect_transitions(log, threshold):
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


logger_transitions = detect_transitions(logger_log, threshold)
print(f"Detected {len(logger_transitions)} transitions in cam1 log\n")

# ── Match each driver transition to the nearest same-direction logger one ──
skews_ms = []
unmatched = 0

logger_by_state = {True: [], False: []}
for t, state in logger_transitions:
    logger_by_state[state].append(t)

for d in driver_rows:
    target_state = bool(d["target_state"])
    candidates = logger_by_state[target_state]
    if not candidates:
        unmatched += 1
        continue
    nearest = min(candidates, key=lambda t: abs(t - d["detect_time"]))
    if abs(nearest - d["detect_time"]) > MATCH_WINDOW_S:
        unmatched += 1
        continue
    skews_ms.append((d["detect_time"] - nearest) * 1000.0)

print("── Summary ──────────────────────────────")
print(f"  Matched transitions: {len(skews_ms)}  (unmatched/out-of-window: {unmatched})")
if skews_ms:
    mean_skew = statistics.mean(skews_ms)
    std_skew = statistics.pstdev(skews_ms) if len(skews_ms) > 1 else 0.0
    print(f"  Skew (cam0 - cam1): mean={mean_skew:+.3f}ms  std={std_skew:.3f}ms  "
          f"min={min(skews_ms):+.3f}ms  max={max(skews_ms):+.3f}ms")
    print(f"  |skew| max: {max(abs(s) for s in skews_ms):.3f}ms")

    driver_lats = [d["latency_ms"] for d in driver_rows]
    print(f"\n  For reference, cam0 solo closed-loop latency (from driver CSV): "
          f"mean={statistics.mean(driver_lats):.3f}ms")
else:
    print("  No matched transitions -- check that both CSVs are from the same run "
          "and MATCH_WINDOW_S is reasonable.")
print("─────────────────────────────────────────")
