"""
Characterizes the persistent, small-amplitude background buzz seen on
axis 1's cx trace even at rest (no step, holding a fixed target) --
visible in results/scratch_axis1_wiresback_baseline25.png both before
AND after the step, present the whole recording. Not previously
explained -- present regardless of Ki, present holding steady.

Method: engage closed_loop with target_x == current cx (zero commanded
step, real hold), record for several seconds using the same tick-based
real-time timestamps every other script in this project trusts (immune
to host clock jitter), then FFT the detrended residual after resampling
to a uniform grid (irregular real sample times, ~280Hz nominal but not
perfectly uniform -- linear interpolation onto a uniform grid at the
median real rate is accurate enough for identifying a buzz frequency
somewhere in the 1-140Hz range, well below any interpolation artifact).

Usage: python3 scratch_buzz_fft.py [KP_MILLI] [KI_MILLI] [--axis2]
--axis2: check axis 2 (dac_x <- cy) instead of axis 1, using its own
kp2/ki2/target_y -- added 2026-09-03 to compare against axis 1's buzz.
"""
import sys
import time
import threading

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, r"C:\Users\Bryan\Documents\GitHub\rpi_camera_system")
import fta_closed_loop_step_response_vcp as step_mod

send_command = step_mod.send_command
get_status = step_mod.get_status
find_fta_port = step_mod.find_fta_port
FTA_BAUD = step_mod.FTA_BAUD
_reader_thread = step_mod._reader_thread

RECORD_S = 8.0
BASE_DAC_Y = 2048

BLUE = "#2a78d6"
ORANGE = "#eb6834"
MUTED = "#898781"
GRID = "#e1e0d9"


def main():
    axis2 = "--axis2" in sys.argv
    pos_args = [a for a in sys.argv[1:] if a != "--axis2"]
    kp_milli = int(pos_args[0]) if len(pos_args) > 0 else 1750
    ki_milli = int(pos_args[1]) if len(pos_args) > 1 else 200000

    import serial
    port = find_fta_port()
    print(f"connecting {port}")
    ser = serial.Serial(port, FTA_BAUD, timeout=0.2)
    time.sleep(2)
    ser.reset_input_buffer()

    print(send_command(ser, "clear_estop"))
    print(send_command(ser, "set_mode open_loop"))
    st = get_status(ser)
    if st["tel_age_ms"] > 500:
        print("ABORT: telemetry stale.")
        ser.close()
        return
    amp_was_enabled = bool(st["amp"])
    if not amp_was_enabled:
        print(send_command(ser, "amp_enable"))

    if axis2:
        print(send_command(ser, f"set_x {BASE_DAC_Y}"))
        time.sleep(0.5)
        st = get_status(ser)
        # set_mode closed_loop has always required set_target_x first (a
        # real safety guard from before axis 2 existed) -- harmless here
        # since Kp/Ki stay 0 for axis 1 the whole time (see the
        # scratch_axis2_step_response.py entry in CLAUDE.md for the full
        # story of this trap).
        print(send_command(ser, f"set_target_x {round(st['tel_x'])}"))
        raw = send_command(ser, "get_status")
        import re
        m = re.search(r"tel_y=(-?[\d.]+)", raw or "")
        target = round(float(m.group(1)))
        print(f"holding target_y={target} (no step), Kp2={kp_milli/1000} Ki2={ki_milli/1000}")
        print(send_command(ser, f"set_target_y {target}"))
        print(send_command(ser, f"set_kp2 {kp_milli}"))
        print(send_command(ser, f"set_ki2 {ki_milli}"))
        print(send_command(ser, "set_kd2 0"))
    else:
        print(send_command(ser, f"set_y {BASE_DAC_Y}"))
        time.sleep(0.5)
        st = get_status(ser)
        baseline_cx = st["tel_x"]
        target = round(baseline_cx)
        print(f"holding target_x={target} (no step), Kp={kp_milli/1000} Ki={ki_milli/1000}")
        print(send_command(ser, f"set_target_x {target}"))
        print(send_command(ser, f"set_kp {kp_milli}"))
        print(send_command(ser, f"set_ki {ki_milli}"))
        print(send_command(ser, "set_kd 0"))

    print(send_command(ser, "set_mode closed_loop"))
    time.sleep(0.3)

    records = []
    stop_event = threading.Event()
    t0 = time.monotonic()
    reader = threading.Thread(target=_reader_thread, args=(ser, t0, records, stop_event), daemon=True)
    reader.start()
    time.sleep(RECORD_S)
    stop_event.set()
    reader.join(timeout=1.0)

    print(send_command(ser, "set_mode open_loop"))
    print(send_command(ser, "set_y 95"))
    print(send_command(ser, "set_x 95"))
    if not amp_was_enabled:
        print(send_command(ser, "amp_disable"))
    ser.close()

    if len(records) < 50:
        print(f"only {len(records)} samples, aborting analysis")
        return

    tick_ms = np.array([r[3] for r in records], dtype=np.float64)
    x = np.array([r[2] for r in records]) if axis2 else np.array([r[1] for r in records])
    t = (tick_ms - tick_ms[0]) / 1000.0
    dt_median = np.median(np.diff(t))
    fs = 1.0 / dt_median
    print(f"{len(records)} samples over {t[-1]:.2f}s, median dt={dt_median*1000:.2f}ms (fs~{fs:.1f}Hz)")

    # Resample onto a uniform grid via linear interpolation (real sample
    # times are only approximately uniform -- see this file's own
    # docstring for why this is accurate enough for a buzz well below fs/2)
    t_uniform = np.arange(0, t[-1], dt_median)
    x_uniform = np.interp(t_uniform, t, x)
    x_detrended = x_uniform - x_uniform.mean()

    n = len(x_detrended)
    window = np.hanning(n)
    spectrum = np.fft.rfft(x_detrended * window)
    freqs = np.fft.rfftfreq(n, d=dt_median)
    mag = np.abs(spectrum)

    # Report the top few peaks above 1Hz (DC/very-low-freq drift isn't the buzz)
    mask = freqs > 1.0
    peak_order = np.argsort(mag[mask])[::-1][:8]
    peak_freqs = freqs[mask][peak_order]
    peak_mags = mag[mask][peak_order]
    print("\nTop spectral peaks (>1Hz):")
    for f, m in zip(peak_freqs, peak_mags):
        print(f"  {f:6.2f} Hz   magnitude={m:.2f}")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), dpi=150)
    for a in (ax1, ax2):
        a.set_facecolor("white")
        for spine in ("top", "right"):
            a.spines[spine].set_visible(False)
        for spine in ("left", "bottom"):
            a.spines[spine].set_color(GRID)
        a.tick_params(colors=MUTED, labelsize=9, length=3)
        a.grid(True, color=GRID, linewidth=0.6)

    coord = "cy" if axis2 else "cx"
    axis_desc = "axis 2 (dac_x->cy)" if axis2 else "axis 1 (dac_y->cx)"
    ax1.plot(t, x - x.mean(), color=BLUE, linewidth=0.8)
    ax1.set_xlabel("time (s)", fontsize=9.5, color=MUTED)
    ax1.set_ylabel(f"{coord} deviation from mean (px)", fontsize=9.5, color=MUTED)
    ax1.set_title(f"{axis_desc} at-rest hold, Kp={kp_milli/1000}/Ki={ki_milli/1000}, no step", fontsize=10.5, color="#1A1A2E", loc="left")

    ax2.plot(freqs, mag, color=ORANGE, linewidth=1.0)
    ax2.set_xlim(0, min(140, fs / 2))
    ax2.set_xlabel("frequency (Hz)", fontsize=9.5, color=MUTED)
    ax2.set_ylabel("|FFT|", fontsize=9.5, color=MUTED)
    ax2.set_title("spectrum of the at-rest residual", fontsize=10.5, color="#1A1A2E", loc="left")
    for f in peak_freqs[:3]:
        ax2.axvline(f, color=MUTED, linewidth=0.8, linestyle=(0, (2, 2)))
        ax2.text(f, mag.max() * 0.9, f"{f:.1f}Hz", fontsize=8, color=MUTED, rotation=90, va="top")

    fig.tight_layout()
    ts = time.strftime("%Y%m%dT%H%M%S")
    axis_tag = "axis2" if axis2 else "axis1"
    out = f"results/scratch_buzz_fft_{axis_tag}_kp{kp_milli}_ki{ki_milli}_{ts}.png"
    fig.savefig(out, facecolor="white")
    print(f"\nsaved {out}")
    np.savez(f"results/scratch_buzz_fft_{axis_tag}_kp{kp_milli}_ki{ki_milli}_{ts}.npz",
             t=t, x=x, freqs=freqs, mag=mag, peak_freqs=peak_freqs, peak_mags=peak_mags)


if __name__ == "__main__":
    main()
