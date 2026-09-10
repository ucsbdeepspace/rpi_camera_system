"""
Order-independence / thermal-history-contamination check for the
low-side (dac_y=1200) vs high-side (dac_y=3900) thermal/current-limit
oscillation thresholds found earlier today.

Runs 4 trials in the order: low, high, high, low -- with a full cooldown
(idle at dac=2048, amp off) between each -- so that:
  - trial 2 (high, right after low) vs trial 3 (high, right after high)
    tests whether High's verdict depends on what ran immediately before it
  - trial 1 (low, cold start) vs trial 4 (low, after two High trials)
    tests whether Low's verdict depends on residual heat from High trials

Same detector as the original v1 test: 12s hold, range>8px = OSCILLATING.

Usage: python thermal_asymmetry_recheck.py
"""
import sys
import time
import threading

import numpy as np

sys.path.insert(0, r"C:\Users\Bryan\Documents\GitHub\rpi_camera_system")
import fta_calibration_vcp as m

import serial

HOLD_S = 12.0
COOLDOWN_S = 90.0
TRIALS = [("low", 1200), ("high", 3900), ("high", 3900), ("low", 1200)]

port = m.find_fta_port()
print(f"connecting {port}")
ser = serial.Serial(port, m.FTA_BAUD, timeout=0.2)
time.sleep(2)
ser.reset_input_buffer()

print(m.send_command(ser, "clear_estop"))
print(m.send_command(ser, "set_mode open_loop"))
st = m.get_status(ser)
amp_was_enabled = bool(st[2])
if not amp_was_enabled:
    print(m.send_command(ser, "amp_enable"))


def idle():
    for cmd in ("set_mode open_loop", "set_x 2048", "set_y 2048"):
        try:
            m.send_command(ser, cmd)
        except Exception as e:
            print(f"WARNING: {cmd!r} failed: {e}")


def run_trial(dac_value):
    idle()
    time.sleep(0.3)
    print(f"  driving y={dac_value} (x parked at 2048)...")
    m.send_command(ser, "set_x 2048")
    m.send_command(ser, f"set_y {dac_value}")
    time.sleep(0.3)

    records = []
    stop_event = threading.Event()

    def reader():
        while not stop_event.is_set():
            try:
                raw = ser.readline()
            except Exception:
                continue
            if not raw or not raw.startswith(b"seq="):
                continue
            line = raw.decode(errors="replace").strip()
            status_m = m.STATUS_TOKEN_RE.search(line)
            if not status_m or not (int(status_m.group(1)) & 1):
                continue
            x_m = m.X_TOKEN_RE.search(line)
            if not x_m:
                continue
            records.append(float(x_m.group(1)))

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    time.sleep(HOLD_S)
    stop_event.set()
    th.join(timeout=1.0)

    idle()

    if len(records) < 20:
        return None, None, f"only {len(records)} samples -- inconclusive"

    coord = np.array(records)
    std = float(np.std(coord))
    rng = float(np.max(coord) - np.min(coord))
    verdict = "OSCILLATING" if rng > 8.0 else "clean"
    return std, rng, verdict


results = []
for i, (side, dac) in enumerate(TRIALS, 1):
    print(f"\n=== trial {i}/4: {side} (dac_y={dac}) ===")
    std, rng, verdict = run_trial(dac)
    print(f"  std={std} range={rng} -> {verdict}")
    results.append((i, side, dac, std, rng, verdict))
    if i < len(TRIALS):
        print(f"  cooling down {COOLDOWN_S:.0f}s (idle, amp {'stays enabled' if amp_was_enabled else 'staying on but zero-drive'})...")
        time.sleep(COOLDOWN_S)

idle()
if not amp_was_enabled:
    try:
        print(m.send_command(ser, "amp_disable"))
    except Exception as e:
        print(f"WARNING: amp_disable failed: {e}")
ser.close()

print("\n=== SUMMARY ===")
print(f"{'trial':>5} {'side':>5} {'dac':>5} {'std':>8} {'range':>8}  verdict")
for i, side, dac, std, rng, verdict in results:
    std_s = f"{std:.2f}" if std is not None else "n/a"
    rng_s = f"{rng:.2f}" if rng is not None else "n/a"
    print(f"{i:>5} {side:>5} {dac:>5} {std_s:>8} {rng_s:>8}  {verdict}")
