"""
Axis-2 (dac_x -> cy) closed-loop step response -- reuses
fta_closed_loop_step_response_vcp.py's helpers (send_command, get_status,
analyze_step, _reader_thread -- the latter already records BOTH x and y
per sample, so no new reader needed) but drives axis 2's own
independent gains (set_kp2/set_ki2/set_kd2, added 2026-09-03) and its own
setpoint (set_target_y, also added 2026-09-03) instead of axis 1's.

Axis 2 has never been tuned on its own merits before today -- it only
ever mirrored axis 1's gains. Its measured open-loop resonance is a
gentler ~8x DC gain (vs. axis 1's ~11x, see CLAUDE.md's axis-2 Bode
entry), so it may tolerate faster tuning than axis 1 ever could.

Usage: python3 scratch_axis2_step_response.py KP_MILLI KI_MILLI [KD_MILLI] [label]
"""
import sys
import time
import threading

import numpy as np

sys.path.insert(0, r"C:\Users\Bryan\Documents\GitHub\rpi_camera_system")
import fta_closed_loop_step_response_vcp as step_mod

send_command = step_mod.send_command
get_status = step_mod.get_status
find_fta_port = step_mod.find_fta_port
FTA_BAUD = step_mod.FTA_BAUD
analyze_step = step_mod.analyze_step
_reader_thread = step_mod._reader_thread

BASE_DAC_X = 2048  # matches axis 1's own established clean operating point (2048);
                    # axis 2 has no equivalent independently-established "clean region"
                    # yet, used as a reasonable starting point, not a validated one.
STEP_PX = -25.0
PRE_S = 0.5
POST_S = 3.0
SETTLE_TOL_PX = 2.0


def run_trial(ser, kp_milli, ki_milli, kd_milli, label="", step_px=None):
    if step_px is None:
        step_px = STEP_PX
    print(f"\n=== TRIAL {label}: Kp2={kp_milli/1000:.3f} Ki2={ki_milli/1000:.3f} Kd2={kd_milli/1000:.3f} ===")

    send_command(ser, "clear_estop")
    send_command(ser, "set_mode open_loop")

    st = get_status(ser)
    if st["tel_age_ms"] > 500:
        print("ABORT: telemetry stale, Pi not streaming.")
        return None

    amp_was_enabled = bool(st["amp"])
    if not amp_was_enabled:
        send_command(ser, "amp_enable")
        st = get_status(ser)
        if not st["amp"]:
            print("ABORT: amp_enable did not take.")
            return None

    send_command(ser, f"set_x {BASE_DAC_X}")
    time.sleep(0.5)

    st = get_status(ser)
    baseline_cy = st["tel_y"] if "tel_y" in st else None
    if baseline_cy is None:
        # get_status()'s STATUS_FIELD_RE (imported from the base module)
        # doesn't include tel_y by default -- read it directly via a raw
        # status line instead of extending the shared dict for one field.
        raw = send_command(ser, "get_status")
        import re
        m = re.search(r"tel_y=(-?[\d.]+)", raw or "")
        baseline_cy = float(m.group(1)) if m else None
    if baseline_cy is None:
        print("ABORT: could not read tel_y.")
        return None

    target_from = round(baseline_cy)
    target_to = round(baseline_cy + step_px)
    print(f"baseline cy={baseline_cy:.1f}  target {target_from} -> {target_to} (step {step_px:+.1f}px)")

    # set_mode closed_loop has ALWAYS required set_target_x first (a real
    # safety guard from before axis 2 existed, g_target_x_set) -- with
    # Kp/Ki left at 0 (never set in this axis-2-only script), axis 1's
    # correction is always exactly 0 regardless of target_x, so this is
    # a harmless no-op for axis 1, purely to satisfy the guard. Found by
    # direct debugging 2026-09-03: set_mode closed_loop was being silently
    # REFUSED (not lost in transit) on every single attempt without this.
    st_cx = get_status(ser)
    send_command(ser, f"set_target_x {round(st_cx['tel_x'])}")

    send_command(ser, f"set_target_y {target_from}")
    send_command(ser, f"set_kp2 {kp_milli}")
    send_command(ser, f"set_ki2 {ki_milli}")
    send_command(ser, f"set_kd2 {kd_milli}")

    # Explicit confirmation, not a single trust-the-reply send -- set_mode
    # replies are lost under load routinely enough elsewhere in this
    # project that a bare send_command() isn't reliable evidence either
    # way (see this file's own docstring notes on VCP reliability).
    import re
    mode_confirmed = False
    for _ in range(6):
        send_command(ser, "set_mode closed_loop")
        time.sleep(0.05)
        raw = send_command(ser, "get_status")
        m = re.search(r"mode=(\S+)", raw or "")
        if m and m.group(1) == "closed_loop":
            mode_confirmed = True
            break
    if not mode_confirmed:
        print("ABORT: closed_loop mode never confirmed engaged after retries.")
        return None
    time.sleep(0.3)

    records = []
    stop_event = threading.Event()
    t0 = time.monotonic()
    reader = threading.Thread(target=_reader_thread, args=(ser, t0, records, stop_event), daemon=True)
    reader.start()

    time.sleep(PRE_S)
    for ch in f"set_target_y {target_to}\n":
        ser.write(ch.encode("ascii"))
        time.sleep(0.02)
    t_step_host = time.monotonic() - t0
    time.sleep(POST_S)

    stop_event.set()
    reader.join(timeout=1.0)

    st = get_status(ser)
    raw = send_command(ser, "get_status")
    import re
    m = re.search(r"target_y=(-?[\d.]+)", raw or "")
    target_landed = m and round(float(m.group(1))) == target_to
    if not target_landed:
        print(f"  WARNING: target_y didn't land as expected, step command likely dropped.")

    send_command(ser, "set_mode open_loop")
    send_command(ser, "set_x 95")
    if not amp_was_enabled:
        send_command(ser, "amp_disable")

    if len(records) < 6:
        print(f"  only {len(records)} samples, discarding trial")
        return None

    host_arr = np.array([r[0] for r in records])
    y = np.array([r[2] for r in records])
    tick_ms = np.array([r[3] for r in records], dtype=np.float64)
    a, b = np.polyfit(host_arr, tick_ms, 1)
    tick_ms_at_step = a * t_step_host + b
    t = (tick_ms - tick_ms[0]) / 1000.0
    t_step = (tick_ms_at_step - tick_ms[0]) / 1000.0

    metrics = analyze_step(t, y, t_step, SETTLE_TOL_PX)
    max_dev = float(np.max(np.abs(y - baseline_cy)))

    if not target_landed and max_dev < 3.0:
        verdict = "NO-RESPONSE (step likely didn't land)"
    elif max_dev > abs(step_px) * 4:
        verdict = "DIVERGED"
    else:
        os_pct = metrics["overshoot_pct"] if metrics and metrics["overshoot_pct"] is not None else None
        settle = metrics["settling_time_s"] if metrics else None
        if os_pct is not None and os_pct > 25:
            verdict = f"RINGING (overshoot {os_pct:.1f}%)"
        else:
            verdict = f"CLEAN (overshoot {os_pct if os_pct is not None else 'n/a'}%, " \
                      f"settle {settle*1000 if settle else 'n/a'}ms)"

    print(f"  max_dev={max_dev:.1f}px  VERDICT: {verdict}")

    ts = time.strftime("%Y%m%dT%H%M%S")
    out_path = f"results/scratch_axis2_step_{label}_{ts}.npz"
    np.savez(out_path, t=t, y=y, t_step=t_step, kp_milli=kp_milli, ki_milli=ki_milli, kd_milli=kd_milli,
             target_from=target_from, target_to=target_to)

    return {"verdict": verdict, "max_dev": max_dev, "metrics": metrics, "out_path": out_path}


def safe_idle(ser):
    send_command(ser, "set_mode open_loop")
    send_command(ser, "amp_disable")
    send_command(ser, "set_x 95")
    send_command(ser, "set_y 95")


def main():
    import serial
    port = find_fta_port()
    print(f"connecting {port}")
    ser = serial.Serial(port, FTA_BAUD, timeout=0.2)
    time.sleep(2)
    ser.reset_input_buffer()

    # --step-px=N accepted anywhere on the command line (default -25.0,
    # matches STEP_PX) -- pulled out before positional parsing so the
    # existing KP_MILLI KI_MILLI [KD_MILLI] [label] contract is unchanged.
    args = [a for a in sys.argv[1:] if not a.startswith("--step-px=")]
    step_px_args = [a for a in sys.argv[1:] if a.startswith("--step-px=")]
    step_px = float(step_px_args[0].split("=", 1)[1]) if step_px_args else STEP_PX

    kp_milli = int(args[0])
    ki_milli = int(args[1])
    kd_milli = int(args[2]) if len(args) > 2 else 0
    label = args[3] if len(args) > 3 else f"kp{kp_milli}_ki{ki_milli}"

    try:
        run_trial(ser, kp_milli, ki_milli, kd_milli, label=label, step_px=step_px)
    finally:
        safe_idle(ser)
        ser.close()


if __name__ == "__main__":
    main()
