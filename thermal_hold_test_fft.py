"""
Static-hold thermal/current-limiting test, v2 -- addresses two real gaps
in the first version (found via direct user pushback):

1. The original test only checked position RANGE against an 8px
   threshold -- if the same on/off cycling happens everywhere but its
   position swing scales with |commanded DAC - 2048|, an 8px threshold
   would miss it near center even if it's still happening. This version
   saves raw telemetry and FFTs the detrended residual, checking for any
   real spectral peak in the 0.3-3Hz band (where the confirmed-oscillating
   cases showed their square-wave period, ~0.6-0.8s) well above the noise
   floor -- not just a time-domain range check.
2. If this is a genuine THERMAL (heat buildup over many seconds) effect
   rather than an instantaneous current-limit trip, a 12s hold may be too
   short to reveal onset at "safer" DAC values. This version holds much
   longer (default 60s) for points being re-checked.

Usage: python thermal_hold_test_fft.py AXIS DAC_VALUE [HOLD_S]
"""
import sys
import time
import threading

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, r"C:\Users\Bryan\Documents\GitHub\rpi_camera_system")
import fta_calibration_vcp as m

import serial

axis = sys.argv[1].lower()
dac_value = int(sys.argv[2])
hold_s = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0
assert axis in ("x", "y")

TICK_RE = m.STATUS_TOKEN_RE  # reuse module's compiled regexes where possible
import re
TICK_TOKEN_RE = re.compile(r"\btick=(\d+)")

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
        tick_m = TICK_TOKEN_RE.search(line)
        if not x_m or not y_m or not tick_m:
            continue
        records.append((int(tick_m.group(1)), float(x_m.group(1)), float(y_m.group(1))))


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

if len(records) < 50:
    print(f"only {len(records)} samples -- inconclusive")
    sys.exit(1)

tick_ms = np.array([r[0] for r in records], dtype=np.float64)
xs = np.array([r[1] for r in records])
ys = np.array([r[2] for r in records])
coord = xs if axis == "y" else ys
coord_name = "cx" if axis == "y" else "cy"

t = (tick_ms - tick_ms[0]) / 1000.0
dt_median = np.median(np.diff(t))
std = float(np.std(coord))
rng = float(np.max(coord) - np.min(coord))
print(f"\n{axis}={dac_value}  n={len(records)} over {t[-1]:.1f}s  "
      f"{coord_name} std={std:.3f}px range={rng:.2f}px  (median dt={dt_median*1000:.2f}ms)")

# Resample onto a uniform grid, detrend, FFT -- same established technique
# as scratch_buzz_fft.py earlier this project.
t_uniform = np.arange(0, t[-1], dt_median)
x_uniform = np.interp(t_uniform, t, coord)
x_detrended = x_uniform - x_uniform.mean()
n = len(x_detrended)
window = np.hanning(n)
spectrum = np.fft.rfft(x_detrended * window)
freqs = np.fft.rfftfreq(n, d=dt_median)
mag = np.abs(spectrum)

# Known oscillation band from confirmed-bad points: ~0.6-0.8s period -> ~1.25-1.7Hz.
# Check a generous 0.3-3Hz band for any real peak.
band = (freqs > 0.3) & (freqs < 3.0)
band_freqs, band_mag = freqs[band], mag[band]
peak_idx = np.argmax(band_mag)
peak_freq, peak_mag = band_freqs[peak_idx], band_mag[peak_idx]
noise_floor = np.median(mag[(freqs > 3.0) & (freqs < freqs.max() * 0.8)])
snr = peak_mag / noise_floor if noise_floor > 0 else float("inf")

print(f"0.3-3Hz band peak: {peak_freq:.2f}Hz, magnitude={peak_mag:.2f}  "
      f"(high-freq noise floor median={noise_floor:.2f}, SNR={snr:.1f}x)")
verdict = "REAL PERIODIC SIGNAL" if snr > 5 else "no significant peak (looks like noise)"
print(f"VERDICT: {verdict}")

ts = time.strftime("%Y%m%dT%H%M%S")
out = f"results/scratch_thermal_fft_{axis}{dac_value}_{ts}"
np.savez(f"{out}.npz", t=t, coord=coord, freqs=freqs, mag=mag,
         peak_freq=peak_freq, peak_mag=peak_mag, noise_floor=noise_floor, snr=snr)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), dpi=150)
ax1.plot(t, coord - coord.mean(), color="#2a78d6", linewidth=0.8)
ax1.set_xlabel("time (s)")
ax1.set_ylabel(f"{coord_name} deviation from mean (px)")
ax1.set_title(f"{axis}={dac_value} hold, {t[-1]:.0f}s, std={std:.3f}px")
ax2.plot(freqs, mag, color="#eb6834", linewidth=1.0)
ax2.set_xlim(0, 5)
ax2.axvline(peak_freq, color="#898781", linewidth=0.8, linestyle="--")
ax2.text(peak_freq, mag.max() * 0.9, f"{peak_freq:.2f}Hz\nSNR={snr:.1f}x", fontsize=8)
ax2.set_xlabel("frequency (Hz)")
ax2.set_ylabel("|FFT|")
ax2.set_title("spectrum, 0-5Hz")
fig.tight_layout()
fig.savefig(f"{out}.png")
print(f"saved {out}.png / .npz")
