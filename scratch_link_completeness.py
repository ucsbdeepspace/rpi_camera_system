"""
Scratch driver (2026-09-01): does every I2C send the Pi's own code
considers successful (i.e. every capture-enumerated `seq` increment --
see camera_view_tool.py's streaming loop, which calls send_position()
unconditionally on every capture_array() call before any detection/send
outcome is known) actually reach and get clocked in by the Nucleo's I2C1
peripheral?

Ground truth, both sides already exist, no new firmware/Pi-side code:
  - pi_seq (relayed `seq=` field, u8 wraps): the Pi's own capture-
    enumerated send counter, unwrapped across the relay stream.
  - pkts= + errs=: the Nucleo's ISR-level counters for every I2C
    transaction it actually completed (checksum-valid + checksum-invalid
    respectively) -- process_beam_packet() increments one or the other
    for every HAL_I2C_SlaveRxCpltCallback firing, so their sum is "every
    transaction the Nucleo's I2C peripheral registered," regardless of
    content validity.

If pi_seq's reconstructed delta > (pkts+errs) delta over the same window,
that gap is real physical-layer loss: a send the Pi's own code believes
happened (no exception from send_position()/smbus2), but the Nucleo's
I2C1 peripheral never completed a transaction for at all -- invisible to
everything measured so far (VCP relay loss, dac_y-change proxy, cseq
firing-vs-confident-packet checks) since none of those ever looked
upstream of "did the Nucleo's ISR fire in the first place."

Runs in open_loop mode, amp off -- purely a link-completeness check, no
actuator motion needed.
"""
import re
import threading
import time

FTA_BAUD = 460800
REPLY_RE = re.compile(r"^(OK|ERR|STATUS|WARN)\b")
TELEMETRY_RE = re.compile(
    r"^seq=\s*(\d+)\s+status=(\d+)\s+x=(-?\d+\.\d)\s+y=(-?\d+\.\d)\s+"
    r"tgt=(-?\d+\.\d)\s+dac_y=(-?\d+)\s+tick=(\d+)\s+pkts=(\d+)\s+errs=(\d+)\s+cseq=(\d+)$")


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
        pi_seq = int(m.group(1))
        pkts = int(m.group(8))
        errs = int(m.group(9))
        records.append((pi_seq, pkts, errs))


def main():
    import sys
    import serial

    duration_s = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0

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
    print(send_command(ser, "amp_disable"))

    records = []
    stop_event = threading.Event()
    reader = threading.Thread(target=reader_thread, args=(ser, records, stop_event), daemon=True)
    ser.reset_input_buffer()
    reader.start()
    time.sleep(duration_s)
    stop_event.set()
    reader.join(timeout=1.0)
    ser.close()

    print(f"captured {len(records)} lines")
    if len(records) < 20:
        print("Not enough samples.")
        return

    pi_seq = [r[0] for r in records]
    pkts = [r[1] for r in records]
    errs = [r[2] for r in records]

    # Unwrap pi_seq (u8, wraps 0-255) across the whole stream.
    unwrapped = [pi_seq[0]]
    for prev, cur in zip(pi_seq, pi_seq[1:]):
        d = cur - prev
        if d < -128:  # wrapped forward
            d += 256
        elif d > 128:  # went backwards further than a wrap would explain (corrupt line) -- keep as-is, flagged below
            pass
        unwrapped.append(unwrapped[-1] + d)

    pi_attempts_delta = unwrapped[-1] - unwrapped[0]
    nucleo_delta = (pkts[-1] - pkts[0]) + (errs[-1] - errs[0])

    print(f"\nPi-side capture-enumerated sends (seq, unwrapped): {pi_attempts_delta}")
    print(f"Nucleo-side I2C transactions registered (pkts+errs delta): {nucleo_delta}")
    gap = pi_attempts_delta - nucleo_delta
    if gap <= 0:
        print(f"No gap (Nucleo registered {-gap} MORE than the Pi's seq delta -- expected noise from "
              f"window-edge effects, not evidence of anything).")
    else:
        pct = 100.0 * gap / pi_attempts_delta if pi_attempts_delta else float("nan")
        print(f"GAP: {gap} sends ({pct:.2f}%) the Pi's own code believed succeeded, "
              f"but the Nucleo's I2C1 peripheral never registered a transaction for at all.")


if __name__ == "__main__":
    main()
