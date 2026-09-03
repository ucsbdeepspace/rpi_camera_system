"""
Cross-axis coupling test (2026-09-03). FEA predicts the two flexure axes
are totally independent modes -- this drives that prediction against real
hardware rather than trusting it blind, per the user's explicit request
("FEA indicates they are totally independent, but we should run the
test").

Method: drive ONE axis's open-loop sine at (near) its own measured
resonance (already characterized: dac_y->cx peaks ~40Hz,
dac_x->cy peaks ~47-50Hz -- see fta_open_loop_bode_test.py's summaries),
and record BOTH cx and cy from the same telemetry stream at once (every
relay packet already carries both, regardless of which axis is driven).
Fit BOTH traces against the SAME known drive frequency (w=2*pi*f_drive,
exact -- no need to fit a reference/commanded trace to recover it, unlike
the Bode test, since the firmware's own sine generator holds f exactly).
The DRIVEN coordinate's fitted amplitude is the "on-axis" response
(already known from the Bode sweep, here as a sanity check); the OTHER
coordinate's fitted amplitude is the "cross-axis" response -- if the FEA
independence prediction holds, this should be indistinguishable from
this same coordinate's own noise floor (measured separately, amp on, no
sine driving, at rest).

Usage: python3 scratch_cross_axis_coupling_test.py
"""
import sys
import time

sys.path.insert(0, r"C:\Users\Bryan\Documents\GitHub\rpi_camera_system")
import fta_open_loop_bode_test as bode

send_command = bode.send_command
get_status = bode.get_status
find_fta_port = bode.find_fta_port
FTA_BAUD = bode.FTA_BAUD
CORE_STATUS_FIELDS = bode.CORE_STATUS_FIELDS
verify_open_sine_state = bode.verify_open_sine_state
ensure_stopped = bode.ensure_stopped
emergency_cleanup = bode.emergency_cleanup
fit_sine_component = bode.fit_sine_component
TELEMETRY_RE = bode.TELEMETRY_RE

import threading
import math
import numpy as np


def _both_axes_reader(ser, records, stop_event):
    """Unlike bode._reader_thread (which only keeps one axis' measured
    value), this keeps BOTH x and y from every confident packet, since
    the whole point here is comparing the driven axis' response against
    the OTHER axis' response from the exact same samples."""
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except Exception:
            continue
        if not raw:
            continue
        m = TELEMETRY_RE.match(raw.decode(errors="replace").strip())
        if not m:
            continue
        status = int(m.group(2))
        if not (status & 1):
            continue
        x = float(m.group(3))
        y = float(m.group(4))
        tick_ms = int(m.group(8))
        records.append((tick_ms, x, y))


def record_quiet_baseline(ser, duration=3.0):
    """amp on, no sine driving, hardware at rest -- measures each axis'
    own noise floor so the cross-axis result can be judged against a real
    reference instead of an assumed threshold."""
    print(f"\n--- quiet baseline ({duration:.0f}s, amp on, no drive) ---")
    records = []
    stop_event = threading.Event()
    reader = threading.Thread(target=_both_axes_reader, args=(ser, records, stop_event), daemon=True)
    reader.start()
    time.sleep(duration)
    stop_event.set()
    reader.join(timeout=1.0)
    if len(records) < 10:
        print("  too few samples for a baseline, skipping")
        return None
    x = np.array([r[1] for r in records])
    y = np.array([r[2] for r in records])
    print(f"  {len(records)} samples -- x std={x.std():.3f}px range={x.ptp():.2f}px, "
          f"y std={y.std():.3f}px range={y.ptp():.2f}px")
    return {"x_std": float(x.std()), "y_std": float(y.std())}


def run_coupling_point(ser, drive_axis, freq, amplitude_counts, base_dac, duration):
    """drive_axis: 'y' drives dac_y (near-resonance ~40Hz, watch cy for
    coupling into cx's pathway); 'x' drives dac_x (near-resonance
    ~47-50Hz, watch cx for coupling into cy's pathway)."""
    print(f"\n--- driving axis={drive_axis} @ {freq:.1f}Hz, watching both cx and cy ---")
    axis_num = 1 if drive_axis == "y" else 0
    set_cmd = "set_y" if drive_axis == "y" else "set_x"
    send_command(ser, f"{set_cmd} {base_dac}")
    time.sleep(0.4)

    freq_millihz = round(freq * 1000)
    confirmed = False
    for attempt in range(5):
        reply = send_command(ser, f"start_open_sine {freq_millihz} {amplitude_counts} {base_dac} {axis_num}")
        print(reply)
        confirmed = verify_open_sine_state(ser, expect_active=True, expect_freq_millihz=freq_millihz,
                                            expect_axis=axis_num)
        if confirmed:
            break
        print(f"  not confirmed (attempt {attempt+1}/5), retrying...")
        ensure_stopped(ser)
        time.sleep(0.3)
    if not confirmed:
        print("ERR: could not confirm start_open_sine after 5 attempts, skipping this point.")
        ensure_stopped(ser)
        return None

    records = []
    stop_event = threading.Event()
    reader = threading.Thread(target=_both_axes_reader, args=(ser, records, stop_event), daemon=True)
    reader.start()
    time.sleep(duration)
    stop_event.set()
    reader.join(timeout=1.0)

    ensure_stopped(ser)
    send_command(ser, f"{set_cmd} {base_dac}")

    if len(records) < 10:
        print(f"  only {len(records)} samples, skipping")
        return None

    tick_ms = np.array([r[0] for r in records], dtype=np.float64)
    x = np.array([r[1] for r in records])
    y = np.array([r[2] for r in records])
    t = (tick_ms - tick_ms[0]) / 1000.0
    w = 2.0 * math.pi * freq

    Ax, Bx, _ = fit_sine_component(t, x, w)
    Ay, By, _ = fit_sine_component(t, y, w)
    amp_x = float(np.hypot(Ax, Bx))
    amp_y = float(np.hypot(Ay, By))

    on_axis_amp = amp_x if drive_axis == "y" else amp_y
    cross_axis_amp = amp_y if drive_axis == "y" else amp_x
    on_axis_name = "cx" if drive_axis == "y" else "cy"
    cross_axis_name = "cy" if drive_axis == "y" else "cx"

    print(f"  {len(records)} samples over {t[-1]:.2f}s -- "
          f"on-axis ({on_axis_name}) fitted amplitude={on_axis_amp:.3f}px, "
          f"cross-axis ({cross_axis_name}) fitted amplitude={cross_axis_amp:.3f}px "
          f"({100.0*cross_axis_amp/on_axis_amp:.1f}% of on-axis)")

    return {
        "drive_axis": drive_axis, "freq": freq,
        "on_axis_name": on_axis_name, "on_axis_amp": on_axis_amp,
        "cross_axis_name": cross_axis_name, "cross_axis_amp": cross_axis_amp,
        "t": t, "x": x, "y": y,
    }


def main():
    import serial
    port = find_fta_port()
    print(f"Connecting to {port} @ {FTA_BAUD}")
    ser = serial.Serial(port, FTA_BAUD, timeout=0.2)
    time.sleep(2)
    ser.reset_input_buffer()

    send_command(ser, "clear_estop")
    send_command(ser, "set_mode open_loop")

    st = get_status(ser, required=CORE_STATUS_FIELDS)
    if st["tel_age_ms"] > 500:
        print(f"ERR: telemetry stale ({st['tel_age_ms']}ms) -- Pi not streaming, aborting.")
        ser.close()
        raise SystemExit(1)

    amp_was_enabled = bool(st["amp"])
    if not amp_was_enabled:
        send_command(ser, "amp_enable")
        st = get_status(ser, required=CORE_STATUS_FIELDS)
        if not st["amp"]:
            print("ERR: amp_enable didn't take, aborting.")
            ser.close()
            raise SystemExit(1)

    results = []
    baseline = None
    try:
        baseline = record_quiet_baseline(ser, duration=3.0)

        # Primary axis resonance ~40Hz (fresh Bode sweep, grid-limited
        # 35/40/44Hz around the peak) -- drive dac_y, watch cy for coupling.
        r1 = run_coupling_point(ser, drive_axis="y", freq=40.0, amplitude_counts=150,
                                 base_dac=2048, duration=5.0)
        if r1:
            results.append(r1)

        # Second axis resonance ~47-50Hz (grid-limited) -- drive dac_x,
        # watch cx for coupling. Midpoint of the bracket used as a
        # reasonable single representative point.
        r2 = run_coupling_point(ser, drive_axis="x", freq=48.5, amplitude_counts=150,
                                 base_dac=2048, duration=5.0)
        if r2:
            results.append(r2)
    finally:
        emergency_cleanup(ser)
        ser.close()

    print("\n\n=== Cross-axis coupling summary ===")
    if baseline:
        print(f"quiet baseline noise floor: cx std={baseline['x_std']:.3f}px, cy std={baseline['y_std']:.3f}px")
    for r in results:
        # A cross-axis fitted AMPLITUDE (not std) below ~2-3x the quiet
        # baseline's std for that same coordinate is not distinguishable
        # from noise -- a real coupling signal should clearly exceed it.
        base_std = baseline["y_std"] if r["cross_axis_name"] == "cy" else baseline["x_std"] if baseline else None
        verdict = "N/A (no baseline)"
        if base_std is not None:
            ratio = r["cross_axis_amp"] / base_std if base_std > 1e-6 else float("inf")
            verdict = f"{ratio:.1f}x baseline noise std -- " + (
                "ABOVE noise, real coupling signal" if ratio > 3.0 else "within/near noise floor, no clear coupling")
        print(f"drive={r['drive_axis']}@{r['freq']:.1f}Hz: on-axis {r['on_axis_name']}={r['on_axis_amp']:.3f}px, "
              f"cross-axis {r['cross_axis_name']}={r['cross_axis_amp']:.3f}px -> {verdict}")

    ts = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    import numpy as _np
    for r in results:
        out_path = f"results/scratch_cross_axis_{r['drive_axis']}drive_{ts}.npz"
        _np.savez(out_path, t=r["t"], x=r["x"], y=r["y"], drive_axis=r["drive_axis"], freq=r["freq"],
                  on_axis_amp=r["on_axis_amp"], cross_axis_amp=r["cross_axis_amp"])
        print(f"saved {out_path}")


if __name__ == "__main__":
    main()
