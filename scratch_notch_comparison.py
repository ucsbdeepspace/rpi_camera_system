"""
Scratch driver for the notch vs. no-notch max-speed comparison
(2026-08-19). Reuses fta_closed_loop_step_response_vcp's helpers but adds
one thing that script doesn't do: explicitly VERIFY set_mode closed_loop
actually took (parses the raw "mode=" field, which get_status()'s
structured dict doesn't expose) and retries if not, rather than trusting
a send_command reply. Root cause for adding this: two live runs this
session got a `None` reply to `set_mode closed_loop` and the actuator
never moved at all post-step (std ~0.09px for the whole recorded window)
-- consistent with the command itself not landing, not real instability.

Also prints explicit amp-on/amp-off timestamps so what happens on the
bench can be correlated with the log.
"""
import re
import sys
import time

sys.path.insert(0, r"C:\Users\Bryan\Documents\GitHub\rpi_camera_system")
import fta_closed_loop_step_response_vcp as step_mod

send_command = step_mod.send_command
get_status = step_mod.get_status
find_fta_port = step_mod.find_fta_port
FTA_BAUD = step_mod.FTA_BAUD

MODE_RE = re.compile(r"mode=(\S+)")


def raw_mode(ser):
    reply = send_command(ser, "get_status")
    if reply is None:
        return None
    m = MODE_RE.search(reply)
    return m.group(1) if m else None


def ensure_mode(ser, target_mode, retries=4):
    for attempt in range(retries):
        send_command(ser, f"set_mode {target_mode}")
        time.sleep(0.05)
        m = raw_mode(ser)
        if m == target_mode:
            return True
        print(f"  set_mode {target_mode}: attempt {attempt+1} read back mode={m}, retrying...")
    return False


def safe_idle(ser, note=""):
    ensure_mode(ser, "open_loop")
    send_command(ser, "amp_disable")
    send_command(ser, "set_x 95")
    send_command(ser, "set_y 95")
    st = get_status(ser)
    print(f"[{note}] idle check: mode={raw_mode(ser)} amp={st['amp']} dac_x={st['dac_x']} dac_y={st['dac_y']}")
    return st


import threading
import numpy as np

KM_RE = re.compile(r"kp_milli=(-?\d+)")
KI_RE = re.compile(r"ki_milli=(-?\d+)")


def run_trial(ser, kp_milli, ki_milli, notch_freq_milli, notch_q_milli=3000,
              ctrl_rate_milli=200000, axis2=1, step_px=-25.0, base_dac_y=2048,
              pre_s=0.5, post_s=3.0, label=""):
    print(f"\n=== TRIAL {label}: Kp_milli={kp_milli} Ki_milli={ki_milli} "
          f"notch={'OFF' if notch_freq_milli is None else f'{notch_freq_milli/1000:.1f}Hz'} ===")

    st = get_status(ser)
    if st["tel_age_ms"] > 500:
        print("ABORT: telemetry stale, Pi not streaming.")
        return None

    send_command(ser, "amp_enable")
    t_amp_on = time.strftime("%H:%M:%S")
    st = get_status(ser)
    if not st["amp"]:
        print("ABORT: amp_enable did not take.")
        return None
    print(f"AMP ON @ {t_amp_on}")

    send_command(ser, f"set_y {base_dac_y}")
    time.sleep(0.5)
    st = get_status(ser)
    baseline_cx = st["tel_x"]
    target_from = round(baseline_cx)
    target_to = round(baseline_cx + step_px)

    send_command(ser, f"set_target_x {target_from}")
    send_command(ser, f"set_kp {kp_milli}")
    send_command(ser, f"set_ki {ki_milli}")
    send_command(ser, "set_kd 0")
    send_command(ser, f"set_ctrl_rate {ctrl_rate_milli}")
    send_command(ser, f"set_axis2 {axis2}")
    if notch_freq_milli is None:
        send_command(ser, "notch_off")
    else:
        send_command(ser, f"set_notch {notch_freq_milli} {notch_q_milli}")

    # Verify gains/notch actually landed (raw regex -- structured dict
    # doesn't expose kp_milli/ki_milli).
    raw = send_command(ser, "get_status")
    km = KM_RE.search(raw); kim = KI_RE.search(raw)
    got_kp = int(km.group(1)) if km else None
    got_ki = int(kim.group(1)) if kim else None
    if got_kp != kp_milli or got_ki != ki_milli:
        print(f"  WARNING: gain readback kp={got_kp} ki={got_ki} != requested "
              f"kp={kp_milli} ki={ki_milli}, retrying once...")
        send_command(ser, f"set_kp {kp_milli}")
        send_command(ser, f"set_ki {ki_milli}")

    if not ensure_mode(ser, "closed_loop"):
        print("ABORT: closed_loop mode never confirmed engaged after retries.")
        safe_idle(ser, f"trial {label} abort")
        return None
    print(f"  closed_loop confirmed engaged, baseline cx={baseline_cx:.1f} "
          f"target {target_from}->{target_to}")
    time.sleep(0.3)

    records = []
    stop_event = threading.Event()
    t0 = time.monotonic()
    reader = threading.Thread(target=step_mod._reader_thread, args=(ser, t0, records, stop_event), daemon=True)
    reader.start()
    time.sleep(pre_s)
    for ch in f"set_target_x {target_to}\n":
        ser.write(ch.encode("ascii"))
        time.sleep(0.02)
    t_step_host = time.monotonic() - t0
    time.sleep(post_s)
    stop_event.set()
    reader.join(timeout=1.0)

    st = get_status(ser)
    target_landed = round(st["target_x"]) == target_to
    if not target_landed:
        print(f"  WARNING: post-run target_x={st['target_x']}, step command likely dropped.")

    t_amp_off = time.strftime("%H:%M:%S")
    safe_idle(ser, f"trial {label} cleanup")
    print(f"AMP OFF @ {t_amp_off}")

    if len(records) < 6:
        print("  too few samples, discarding trial")
        return None

    host_arr = np.array([r[0] for r in records])
    x = np.array([r[1] for r in records])
    tick_ms = np.array([r[3] for r in records], dtype=np.float64)
    a, b = np.polyfit(host_arr, tick_ms, 1)
    tick_ms_at_step = a * t_step_host + b
    t = (tick_ms - tick_ms[0]) / 1000.0
    t_step = (tick_ms_at_step - tick_ms[0]) / 1000.0

    pre_mask = t < t_step
    post_mask = ~pre_mask
    pre_x = x[pre_mask]
    post_x = x[post_mask]
    post_t = t[post_mask]

    reached = bool(np.any(np.abs(post_x - target_to) < 3.0))
    half = len(post_x) // 2
    early_std = post_x[:half].std() if half > 5 else float("nan")
    late_std = post_x[half:].std() if len(post_x) - half > 5 else float("nan")
    max_dev = float(np.max(np.abs(post_x - baseline_cx)))

    if not target_landed or not reached and max_dev < 3.0:
        verdict = "NO-RESPONSE (mode/step likely didn't land -- discard)"
    elif late_std > early_std * 1.5 and late_std > 1.0:
        verdict = "GROWING / UNSTABLE"
    elif max_dev > abs(step_px) * 3:
        verdict = "DIVERGED"
    else:
        metrics = step_mod.analyze_step(t, x, t_step, 2.0)
        os_pct = metrics["overshoot_pct"] if metrics and metrics["overshoot_pct"] is not None else None
        settle = metrics["settling_time_s"] if metrics else None
        if os_pct is not None and os_pct > 25:
            verdict = f"RINGING (overshoot {os_pct:.1f}%)"
        else:
            verdict = f"CLEAN (overshoot {os_pct if os_pct is not None else 'n/a'}%, " \
                      f"settle {settle*1000 if settle else 'n/a'}ms)"

    print(f"  reached_target={reached} max_dev={max_dev:.1f}px early_std={early_std:.2f} "
          f"late_std={late_std:.2f} target_landed={target_landed}")
    print(f"  VERDICT: {verdict}")

    ts = time.strftime("%Y%m%dT%H%M%S")
    out_path = f"results/scratch_notch_cmp_{label}_{ts}.npz"
    np.savez(out_path, t=t, x=x, t_step=t_step, kp_milli=kp_milli, ki_milli=ki_milli,
             notch_freq_milli=notch_freq_milli or 0, target_from=target_from, target_to=target_to)

    return {"verdict": verdict, "max_dev": max_dev, "early_std": early_std,
            "late_std": late_std, "reached": reached, "target_landed": target_landed,
            "out_path": out_path}


def main():
    import serial
    port = find_fta_port()
    print(f"connecting {port}")
    ser = serial.Serial(port, FTA_BAUD, timeout=0.2)
    time.sleep(2)
    ser.reset_input_buffer()
    send_command(ser, "clear_estop")
    safe_idle(ser, "startup")

    import sys as _sys
    if len(_sys.argv) > 1 and _sys.argv[1] == "trial":
        # scratch_notch_comparison.py trial <ki_milli> <notch_milli_or_0> <label>
        ki_milli = int(_sys.argv[2])
        notch_milli = int(_sys.argv[3])
        label = _sys.argv[4] if len(_sys.argv) > 4 else "t"
        run_trial(ser, 1750, ki_milli, None if notch_milli == 0 else notch_milli, label=label)
        safe_idle(ser, "final")

    ser.close()


if __name__ == "__main__":
    main()
