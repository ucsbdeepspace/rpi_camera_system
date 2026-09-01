"""
Scratch driver (2026-09-01): does every incoming CONFIDENT I2C packet
actually get a completed PID loop, or does the single-slot g_latest_beam
"mailbox" (overwritten by a new ISR arrival before the main loop drains
g_new_packet_ready -- not a queue) silently drop some before they ever
reach run_closed_loop_step()?

Uses two new firmware-level counters (main.c, 2026-09-01):
  - g_confident_packet_count (confident_pkts=): ISR-level, unconditional
    of mode, increments once per checksum-valid packet with status bit0
    set.
  - g_ctrl_step_seq (cseq=): main-loop-level, increments once per real
    run_closed_loop_step() firing (already added 2026-08-27 for a
    different but related measurement).

Both are plain monotonic counters read via two get_status calls
bracketing a test window -- diffing them is airtight ground truth,
immune to the ~4-8% VCP telemetry LINE loss already found and
root-caused separately (that affects the per-packet relay print, not
these counters, which live purely in firmware state until read).
"""
import re
import sys
import time

FTA_BAUD = 460800
REPLY_RE = re.compile(r"^(OK|ERR|STATUS|WARN)\b")
FIELD_RE = {
    "confident_pkts": re.compile(r"confident_pkts=(\d+)"),
    "cseq": re.compile(r"cseq=(\d+)"),
    "pkts": re.compile(r"pkts=(\d+)"),
    "mode": re.compile(r"mode=(\S+)"),
    "amp": re.compile(r"amp=(\d+)"),
    "tel_x": re.compile(r"tel_x=(-?[\d.]+)"),
}


def find_fta_port():
    from serial.tools import list_ports
    candidates = [p for p in list_ports.comports()
                  if any(t in (p.description or "") for t in ("STLink", "ST-Link", "STMicroelectronics"))]
    return candidates[0].device if candidates else None


def send_command(ser, cmd, char_delay=0.02, reply_timeout=2.0, retries=5):
    for _ in range(retries):
        ser.reset_input_buffer()
        for ch in cmd + "\n":
            ser.write(ch.encode("ascii"))
            time.sleep(char_delay)
        deadline = time.monotonic() + reply_timeout
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if REPLY_RE.match(line):
                return line
    return None


def get_status(ser, retries=6):
    for _ in range(retries):
        reply = send_command(ser, "get_status", retries=1)
        if reply is None or not reply.startswith("STATUS"):
            continue
        m = {k: rx.search(reply) for k, rx in FIELD_RE.items()}
        if all(m.values()):
            return {"confident_pkts": int(m["confident_pkts"].group(1)),
                    "cseq": int(m["cseq"].group(1)),
                    "pkts": int(m["pkts"].group(1)),
                    "mode": m["mode"].group(1),
                    "amp": int(m["amp"].group(1)),
                    "tel_x": float(m["tel_x"].group(1))}
    raise RuntimeError("get_status failed after retries")


def run_trial(ser, label, kp_milli, ki_milli, ctrl_rate_millihz, axis2, duration_s, base_dac_y=2048):
    print(f"\n=== {label} ===")
    print(send_command(ser, f"set_y {base_dac_y}"))
    time.sleep(0.4)
    st = get_status(ser)
    baseline = round(st["tel_x"])
    print(send_command(ser, f"set_target_x {baseline}"))
    print(send_command(ser, f"set_kp {kp_milli}"))
    print(send_command(ser, f"set_ki {ki_milli}"))
    print(send_command(ser, "set_kd 0"))
    print(send_command(ser, f"set_ctrl_rate {ctrl_rate_millihz}"))
    print(send_command(ser, f"set_axis2 {axis2}"))
    print(send_command(ser, "set_mode closed_loop"))
    time.sleep(0.2)

    st1 = get_status(ser)
    t1 = time.monotonic()
    time.sleep(duration_s)
    st2 = get_status(ser)
    t2 = time.monotonic()

    print(send_command(ser, "set_mode open_loop"))
    print(send_command(ser, "set_y 95"))

    d_confident = st2["confident_pkts"] - st1["confident_pkts"]
    d_cseq = st2["cseq"] - st1["cseq"]
    d_pkts = st2["pkts"] - st1["pkts"]
    dt = t2 - t1

    fraction = 100.0 * d_cseq / d_confident if d_confident else float("nan")
    missing = d_confident - d_cseq

    print(f"[{label}] window={dt:.2f}s  raw_pkts={d_pkts} ({d_pkts/dt:.1f}/s)  "
          f"confident_pkts={d_confident} ({d_confident/dt:.1f}/s)  "
          f"ctrl_steps_fired={d_cseq} ({d_cseq/dt:.1f}/s)")
    print(f"[{label}] fraction of confident packets that got a completed PID loop: "
          f"{fraction:.2f}%  ({'ALL' if missing == 0 else f'{missing} MISSING'})")
    return {"d_confident": d_confident, "d_cseq": d_cseq, "d_pkts": d_pkts,
            "dt": dt, "fraction": fraction, "missing": missing}


def main():
    import serial
    label_prefix = sys.argv[1] if len(sys.argv) > 1 else "run"
    n_trials = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    axis2 = int(sys.argv[3]) if len(sys.argv) > 3 else 1

    port = find_fta_port()
    if port is None:
        print("No ST-Link port found.")
        raise SystemExit(1)
    print(f"Connecting to {port} @ {FTA_BAUD}")
    ser = serial.Serial(port, FTA_BAUD, timeout=0.2)
    time.sleep(2)
    ser.reset_input_buffer()

    print(send_command(ser, "clear_estop"))
    print(send_command(ser, "set_mode open_loop"))
    st = get_status(ser)
    amp_was_enabled = bool(st["amp"])
    if not amp_was_enabled:
        print(send_command(ser, "amp_enable"))

    results = []
    for i in range(n_trials):
        for attempt in range(3):
            try:
                r = run_trial(ser, f"{label_prefix} trial {i+1}", 1750, 15000, 0, axis2, 4.0)
                results.append(r)
                break
            except RuntimeError as e:
                print(f"[{label_prefix} trial {i+1}] attempt {attempt+1} failed ({e}), retrying...")
                time.sleep(0.5)
        time.sleep(1)

    if not amp_was_enabled:
        for _ in range(5):
            send_command(ser, "amp_disable")
            if not get_status(ser)["amp"]:
                break
            time.sleep(0.2)
    final = get_status(ser)
    print("final:", final)
    if final["amp"]:
        print("WARNING: amp still enabled after cleanup retries -- check hardware manually.")
    ser.close()

    fracs = [r["fraction"] for r in results]
    total_confident = sum(r["d_confident"] for r in results)
    total_cseq = sum(r["d_cseq"] for r in results)
    total_missing = sum(r["missing"] for r in results)
    print(f"\n{label_prefix} SUMMARY: per-trial fractions={[f'{f:.2f}%' for f in fracs]}")
    print(f"pooled: {total_cseq}/{total_confident} confident packets got a completed PID loop "
          f"({100.0*total_cseq/total_confident:.2f}%), {total_missing} missing")


if __name__ == "__main__":
    main()
