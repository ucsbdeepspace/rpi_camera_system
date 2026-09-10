"""
Static-hold thermal/current-limiting oscillation test.

Holds ONE axis at a fixed DAC value (open loop, no control loop involved)
with the amp enabled, while the OTHER axis is parked at 2048 (center,
near-zero drive on this inverting amp -- avoids confounding this test
with the already-found idle-at-floor thermal bug). Records ~N seconds of
telemetry and reports std/range of the corresponding measured coordinate
-- a clean hold should show only normal detection noise (std well under
1px over a several-second window, per this session's baseline
measurements); the square-wave thermal-cycling artifact found earlier
showed std ~30-35px, range ~75-135px, so a >8px range threshold is a
robust, unambiguous detector with huge margin either way.

Usage: python thermal_hold_test.py AXIS DAC_VALUE [HOLD_S]
  AXIS: x or y (which DAC to hold at the test value)
  DAC_VALUE: the count to test
  HOLD_S: seconds to hold and record, default 12
"""
import sys
import time
import threading

import numpy as np

sys.path.insert(0, r"C:\Users\Bryan\Documents\GitHub\rpi_camera_system")
import fta_calibration_vcp as m

import serial

axis = sys.argv[1].lower()
dac_value = int(sys.argv[2])
hold_s = float(sys.argv[3]) if len(sys.argv) > 3 else 12.0
assert axis in ("x", "y")

# old-board V(DAC) fit, measured new-board resistance, for a power estimate
d = np.load(r"C:\Users\Bryan\Documents\GitHub\rpi_camera_system\results\fta_amp_voltage_calibration.npz")
r_new = 1.6  # measured directly on today's board, not assumed
slope, intercept = (float(d["x_slope"]), float(d["x_intercept"])) if axis == "x" else \
                    (float(d["y_slope"]), float(d["y_intercept"]))
v_est = slope * dac_value + intercept
p_est_mw = v_est ** 2 / r_new * 1000

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

# park the OTHER axis at center, drive the test axis to the target value
if axis == "y":
    print(m.send_command(ser, "set_x 2048"))
    print(m.send_command(ser, f"set_y {dac_value}"))
else:
    print(m.send_command(ser, "set_y 2048"))
    print(m.send_command(ser, f"set_x {dac_value}"))
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
        y_m = m.Y_TOKEN_RE.search(line)
        if not x_m or not y_m:
            continue
        records.append((float(x_m.group(1)), float(y_m.group(1))))


th = threading.Thread(target=reader, daemon=True)
th.start()
print(f"holding {axis}={dac_value} for {hold_s:.0f}s (other axis parked at 2048)...")
time.sleep(hold_s)
stop_event.set()
th.join(timeout=1.0)

for cmd in ("set_mode open_loop", "set_y 2048", "set_x 2048"):
    try:
        print(m.send_command(ser, cmd))
    except Exception as e:
        print(f"WARNING: {cmd!r} failed: {e}")
if not amp_was_enabled:
    try:
        print(m.send_command(ser, "amp_disable"))
    except Exception as e:
        print(f"WARNING: amp_disable failed: {e}")
ser.close()

if len(records) < 20:
    print(f"only {len(records)} samples -- inconclusive")
    sys.exit(1)

xs = np.array([r[0] for r in records])
ys = np.array([r[1] for r in records])
coord = xs if axis == "y" else ys  # dac_y drives cx, dac_x drives cy
coord_name = "cx" if axis == "y" else "cy"
std = float(np.std(coord))
rng = float(np.max(coord) - np.min(coord))
verdict = "OSCILLATING" if rng > 8.0 else "clean"

print(f"\n{axis}={dac_value}  n={len(records)}  {coord_name} std={std:.2f}px range={rng:.2f}px  "
      f"est. V={v_est:.3f}  est. P={p_est_mw:.0f}mW  VERDICT: {verdict}")
