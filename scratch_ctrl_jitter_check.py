#!/usr/bin/env python3
"""
Ad hoc (not committed-quality) check: does throttling the control loop to
200Hz actually make control-STEP timing more regular, or just less
frequent? Reuses telemetry's own dac_y= field as a firing-detector: in
closed_loop mode, apply_dac() is only ever called from run_closed_loop_step,
so dac_y only changes value on a real control-step firing. Detecting
dac_y value-changes in the telemetry stream (timestamped via firmware
tick=, never a host clock) reconstructs the real achieved control-step
interval distribution, for direct comparison between full-rate and
throttled-200Hz operation.

Two conditions, each a real step response using each condition's own
already-validated-stable gains (not the same gains for both -- the goal
here is characterizing the control-step TIMING pattern, not comparing
control performance):
  - full rate (ctrl_rate=0), Kp=1.75/Ki=15 (known stable at full rate)
  - throttled 200Hz, Kp=1.75/Ki=200 (known stable when throttled)
"""
import re
import threading
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FTA_BAUD = 460800
REPLY_RE = re.compile(r"^(OK|ERR|STATUS|WARN)\b")
TELEMETRY_RE = re.compile(
    r"^seq=\s*(\d+)\s+status=(\d+)\s+x=(-?\d+\.\d)\s+y=(-?\d+\.\d)\s+"
    r"tgt=(-?\d+\.\d)\s+dac_y=(-?\d+)\s+tick=(\d+)\s+pkts=(\d+)\s+errs=(\d+)$")
STATUS_FIELD_RE = {
    "dac_y": re.compile(r"dac_y=(-?\d+)"),
    "amp": re.compile(r"amp=(\d+)"),
    "tel_x": re.compile(r"tel_x=(-?[\d.]+)"),
    "tel_age_ms": re.compile(r"tel_age_ms=(\d+)"),
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
            return {k: (int(v.group(1)) if k != "tel_x" else float(v.group(1))) for k, v in m.items()}
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
        if not (int(m.group(2)) & 1):
            continue
        tick_ms = int(m.group(7))
        dac_y = int(m.group(6))
        records.append((tick_ms, dac_y))


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
    print(send_command(ser, f"set_kd 0"))
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

    tick_ms = np.array([r[0] for r in records], dtype=np.int64)
    dac_y = np.array([r[1] for r in records], dtype=np.int64)
    print(f"captured {len(records)} samples")
    return tick_ms, dac_y


def firing_intervals(tick_ms, dac_y):
    """Collapse consecutive equal-dac_y samples into runs; the first tick
    of each new run is a real control-step firing (apply_dac only ever
    called from run_closed_loop_step in closed_loop mode). Returns the
    tick-to-tick intervals between consecutive firings."""
    change_mask = np.concatenate([[True], np.diff(dac_y) != 0])
    firing_ticks = tick_ms[change_mask]
    return np.diff(firing_ticks).astype(np.float64)


def main():
    import serial
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

    t_full, dac_full = run_condition(ser, "FULL RATE (ctrl_rate=0), Kp=1.75/Ki=15",
                                      0, 1750, 15000, -25.0, 4.0)
    t_thr, dac_thr = run_condition(ser, "THROTTLED 200Hz, Kp=1.75/Ki=200",
                                    200000, 1750, 200000, -25.0, 4.0)

    print(send_command(ser, "set_ctrl_rate 0"))  # leave firmware at default full-rate
    if not amp_was_enabled:
        print(send_command(ser, "amp_disable"))
    print("final:", get_status(ser))
    ser.close()

    iv_full = firing_intervals(t_full, dac_full)
    iv_thr = firing_intervals(t_thr, dac_thr)

    print(f"\nFULL RATE firing intervals: n={len(iv_full)} mean={iv_full.mean():.2f}ms "
          f"std={iv_full.std():.2f}ms  CV={100*iv_full.std()/iv_full.mean():.1f}%  "
          f"median={np.median(iv_full):.2f}ms")
    print(f"THROTTLED  firing intervals: n={len(iv_thr)} mean={iv_thr.mean():.2f}ms "
          f"std={iv_thr.std():.2f}ms  CV={100*iv_thr.std()/iv_thr.mean():.1f}%  "
          f"median={np.median(iv_thr):.2f}ms")

    np.savez("results/fta_ctrl_jitter_check.npz",
             iv_full=iv_full, iv_thr=iv_thr,
             t_full=t_full, dac_full=dac_full, t_thr=t_thr, dac_thr=dac_thr)

    BLUE = "#2a78d6"
    ORANGE = "#eb6834"
    MUTED = "#898781"
    GRID = "#e1e0d9"

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), dpi=150)
    for ax, iv, color, label in (
        (axes[0], iv_full, BLUE, "Full rate (~465Hz telemetry, ctrl fires every confident packet)"),
        (axes[1], iv_thr, ORANGE, "Throttled to 200Hz (5ms gate)"),
    ):
        ax.set_facecolor("white")
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        for spine in ("left", "bottom"):
            ax.spines[spine].set_color(GRID)
        ax.tick_params(colors=MUTED, labelsize=9, length=3)
        bins = np.arange(0, max(iv.max(), 10) + 1, 0.5)
        ax.hist(iv, bins=bins, color=color, alpha=0.85, edgecolor="white", linewidth=0.3)
        ax.set_xlabel("interval between control-step firings (ms)", fontsize=9, color=MUTED)
        ax.set_ylabel("count", fontsize=9, color=MUTED)
        cv = 100 * iv.std() / iv.mean()
        ax.set_title(f"{label}\nmean={iv.mean():.2f}ms  std={iv.std():.2f}ms  CV={cv:.1f}%  n={len(iv)}",
                     fontsize=9.5, color="#0b0b0b")

    fig.suptitle("Real achieved control-step timing: full-rate vs. throttled-200Hz",
                 fontsize=13, fontweight="bold")
    fig.tight_layout()
    fig.savefig("results/fta_ctrl_jitter_check.png", facecolor="white")
    print("Saved results/fta_ctrl_jitter_check.png")


if __name__ == "__main__":
    main()
