"""
Scratch driver (2026-08-27) for a clean double-vs-float / -O2-vs-precision
throughput+jitter measurement, replacing two flawed proxies used earlier
the same day:
  1. Detecting dac_y VALUE CHANGES as a firing proxy -- goes blind whenever
     a real firing's correction rounds to the same integer (near-settled
     periods), silently undercounting real firings.
  2. Comparing the firmware's EMA-smoothed meas_ctrl_rate_millihz (a
     near-instantaneous recent-rate snapshot) directly against a true
     window-averaged raw packet rate -- different statistics over
     different effective windows, produced a nonsensical 102% reading.

Uses the new g_ctrl_step_seq counter (main.c) instead: a plain,
monotonic, unconditional count of real run_closed_loop_step() firings,
relayed per-packet as cseq= alongside tick=. No proxy, no averaging --
firing count over any window is just last_cseq-first_cseq, and true
per-firing intervals come directly from consecutive increments' tick=
deltas.
"""
import re
import sys
import threading
import time

import numpy as np

FTA_BAUD = 460800
REPLY_RE = re.compile(r"^(OK|ERR|STATUS|WARN)\b")
TELEMETRY_RE = re.compile(
    r"^seq=\s*(\d+)\s+status=(\d+)\s+x=(-?\d+\.\d)\s+y=(-?\d+\.\d)\s+"
    r"tgt=(-?\d+\.\d)\s+dac_y=(-?\d+)\s+tick=(\d+)\s+pkts=(\d+)\s+errs=(\d+)\s+cseq=(\d+)$")
STATUS_FIELD_RE = {
    "amp": re.compile(r"amp=(\d+)"),
    "tel_x": re.compile(r"tel_x=(-?[\d.]+)"),
    "mode": re.compile(r"mode=(\S+)"),
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


def get_status(ser, retries=5):
    for _ in range(retries):
        reply = send_command(ser, "get_status", retries=1)
        if reply is None or not reply.startswith("STATUS"):
            continue
        m = {k: rx.search(reply) for k, rx in STATUS_FIELD_RE.items()}
        if all(m.values()):
            return {"amp": int(m["amp"].group(1)), "tel_x": float(m["tel_x"].group(1)),
                    "mode": m["mode"].group(1)}
    raise RuntimeError("get_status failed after retries")


def reader_thread(ser, records, stop_event):
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
        pi_seq, status, cseq, tick = int(m.group(1)), int(m.group(2)), int(m.group(10)), int(m.group(7))
        records.append((tick, pi_seq, status, cseq))


def run_condition(ser, label, ctrl_rate_millihz, kp_milli, ki_milli, step_px, duration_s):
    print(f"\n=== {label} ===")
    print(send_command(ser, "set_y 2048"))
    time.sleep(0.4)
    st = get_status(ser)
    baseline = round(st["tel_x"])
    target = round(baseline + step_px)
    print(send_command(ser, f"set_target_x {baseline}"))
    print(send_command(ser, f"set_kp {kp_milli}"))
    print(send_command(ser, f"set_ki {ki_milli}"))
    print(send_command(ser, "set_kd 0"))
    print(send_command(ser, f"set_ctrl_rate {ctrl_rate_millihz}"))
    print(send_command(ser, "set_mode closed_loop"))
    time.sleep(0.3)

    records = []
    stop_event = threading.Event()
    reader = threading.Thread(target=reader_thread, args=(ser, records, stop_event), daemon=True)
    ser.reset_input_buffer()
    reader.start()

    time.sleep(0.3)
    for ch in f"set_target_x {target}\n":
        ser.write(ch.encode("ascii"))
        time.sleep(0.02)
    time.sleep(duration_s)

    stop_event.set()
    reader.join(timeout=1.0)

    print(send_command(ser, "set_mode open_loop"))
    print(send_command(ser, "set_y 95"))

    print(f"captured {len(records)} lines")
    return records


def analyze(records, label):
    tick = np.array([r[0] for r in records], dtype=np.int64)
    pi_seq = np.array([r[1] for r in records], dtype=np.int64)
    status = np.array([r[2] for r in records], dtype=np.int64)
    cseq = np.array([r[3] for r in records], dtype=np.int64)

    # Reject corrupted/torn lines before trusting anything derived from
    # tick or pi_seq: a line that passes TELEMETRY_RE syntactically can
    # still be semantically garbage if two lines got concatenated at a
    # field boundary during a lost/torn serial read. tick is HAL_GetTick()
    # (1ms resolution) -- a real inter-sample gap at ~200-500Hz telemetry
    # is never anywhere close to 500ms, so any dtick that large is a torn
    # line, not a real timing event. Drop the LATER sample of any such
    # pair (cseq/pi_seq from a torn line are equally untrustworthy).
    dtick_raw = np.diff(tick)
    bad = np.concatenate([[False], dtick_raw > 500])
    n_bad = int(bad.sum())
    if n_bad:
        print(f"[{label}] dropped {n_bad} corrupted/torn line(s) (tick delta > 500ms)")
    tick, pi_seq, status, cseq = tick[~bad], pi_seq[~bad], status[~bad], cseq[~bad]

    # Raw-packet-count denominator: pi_seq (the ORIGINAL Pi-sent seq byte,
    # relayed end-to-end) unwrapped, not a count of captured lines --
    # robust to host-side VCP line loss (a real, separate effect measured
    # here as "lost"), since pi_seq's own value in whatever lines DID
    # arrive already reflects the true cumulative count. Confirmed real
    # this session: 83-163 of ~2000 lines/trial were lost in transit
    # (TX queue depth / host read timing), which silently undercounted a
    # captured-line-based denominator enough to read >100% before this fix.
    dseq = np.diff(pi_seq)
    dseq_wrapped = np.where(dseq < 0, dseq + 256, dseq)
    raw_pkt_count = int(dseq_wrapped.sum())
    lost = int((dseq_wrapped - 1).clip(min=0).sum())

    firing_count = int(cseq[-1] - cseq[0])
    fraction = 100.0 * firing_count / raw_pkt_count if raw_pkt_count else float("nan")

    # real per-firing intervals: wherever cseq increments between
    # consecutive (surviving) samples, the tick delta between those two
    # samples IS the true firing interval (no proxy, no dac_y gating).
    dcseq = np.diff(cseq)
    dtick = np.diff(tick)
    fired_mask = dcseq >= 1
    intervals = dtick[fired_mask].astype(np.float64)

    cv = 100 * intervals.std() / intervals.mean() if len(intervals) else float("nan")
    print(f"[{label}] n_samples={len(records)-n_bad} raw_pkt_count={raw_pkt_count} "
          f"firing_count={firing_count} fraction={fraction:.1f}% "
          f"pi_seq_lost={lost} ({100*lost/raw_pkt_count:.1f}% of raw)")
    print(f"[{label}] real firing intervals: n={len(intervals)} "
          f"mean={intervals.mean():.2f}ms std={intervals.std():.2f}ms CV={cv:.1f}%")
    return {"fraction": fraction, "firing_count": firing_count, "raw_pkt_count": raw_pkt_count,
            "lost": lost, "intervals": intervals}


def main():
    import serial
    label = sys.argv[1] if len(sys.argv) > 1 else "run"
    n_trials = int(sys.argv[2]) if len(sys.argv) > 2 else 3

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
    print(send_command(ser, "set_axis2 1"))

    all_intervals = []
    fractions = []
    for i in range(n_trials):
        records = run_condition(ser, f"{label} trial {i+1}", 0, 1750, 15000, -25.0, 4.0)
        result = analyze(records, f"{label} trial {i+1}")
        fractions.append(result["fraction"])
        all_intervals.append(result["intervals"])
        time.sleep(1)

    print(send_command(ser, "set_ctrl_rate 0"))
    if not amp_was_enabled:
        print(send_command(ser, "amp_disable"))
    print("final:", get_status(ser))
    ser.close()

    pooled = np.concatenate(all_intervals)
    cv = 100 * pooled.std() / pooled.mean()
    print(f"\n{label} SUMMARY: fractions={[f'{f:.1f}%' for f in fractions]} "
          f"pooled_intervals n={len(pooled)} mean={pooled.mean():.2f}ms "
          f"std={pooled.std():.2f}ms CV={cv:.1f}%")

    np.savez(f"results/scratch_cseq_{label}.npz",
             fractions=np.array(fractions), pooled_intervals=pooled)


if __name__ == "__main__":
    main()
