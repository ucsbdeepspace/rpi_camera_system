#!/usr/bin/env python3
"""
Interactive manual DAC control for camera_centroid_receiver -- for visually
confirming the FTA actuator physically moves, independent of any scripted
test. Written after fta_step_response_test_vcp.py's first real run
(2026-08-04) measured essentially zero movement (delta ~0.06px, pure
noise) despite the script sending amp_enable first. That confirms the
Nucleo's PA12 amp-gate signal went out, but NOT that the physical amp
board has power or that the actuator is actually wired up on this bench --
software can't see past the GPIO pin. This tool exists to let a human
watch the actuator directly while jogging the DAC by hand.

Not a scripted test -- a REPL. Runs entirely from the laptop over the
Nucleo's VCP; the Pi doesn't need to be involved at all for this (no
camera/telemetry dependency -- pure open-loop DAC control + status).
A background thread continuously drains the high-rate I2C-relay
telemetry/heartbeat lines so the serial buffer never backs up, but only
command replies (OK/ERR/STATUS) are shown -- same filtering pattern as
fta_step_response_test_vcp.py's reader thread.

Commands:
  x N        set DAC-x to N (clamped to [95, 4000] by firmware)
  y N        set DAC-y to N
  center     shortcut for x 95 / y 95 (idle floor, no motion)
  amp on     enable the amp gate (PA12) -- does NOT confirm the physical
             amp board is powered, only that this signal was sent
  amp off    disable it
  status     print get_status (mode, amp, estop, dac_x/y, relayed
             telemetry + age, error counts, uptime)
  estop      send the bare '!' emergency-stop byte
  clear      clear_estop (releases the latch estop sets)
  raw TEXT   send TEXT verbatim as a command line (escape hatch for
             anything not covered above)
  quit       restore idle state (x=95 y=95 amp off) and exit

A dropped character in a command (see CLAUDE.md's 2026-08-04 bench-test
notes -- the VCP occasionally loses a byte under live I2C telemetry load)
shows up as an unexpected ERR reply -- just retype the command, nothing
is left in a bad state by a rejected line.

Usage: python3 fta_manual_control.py [--port PORT]
"""
import argparse
import queue
import re
import threading
import time

FTA_BAUD = 115200  # camera_centroid_receiver's USART2 rate, NOT the old
                    # "FTA Controller"'s 460800.
REPLY_RE = re.compile(r"^(OK|ERR|STATUS)\b")

HELP_TEXT = """\
  x N        set DAC-x to N (95-4000)
  y N        set DAC-y to N
  center     x 95 / y 95 (idle floor)
  amp on     enable amp gate (PA12)
  amp off    disable amp gate
  status     print get_status
  estop      send bare '!' e-stop byte
  clear      clear_estop
  raw TEXT   send TEXT verbatim
  h          this help
  quit       restore idle state and exit"""


def find_fta_port():
    """Auto-detect the Nucleo's USB-serial port by USB description -- same
    tags used in the other fta_*.py scripts."""
    from serial.tools import list_ports
    candidates = [
        p for p in list_ports.comports()
        if any(tag in (p.description or "") for tag in ("STLink", "ST-Link", "STMicroelectronics"))
    ]
    if not candidates:
        return None
    return candidates[0].device


def reader_thread(ser, reply_q, stop_event):
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except Exception:
            continue
        if not raw:
            continue
        line = raw.decode(errors="replace").strip()
        if REPLY_RE.match(line):
            reply_q.put(line)
        # else: telemetry ("seq=...") or heartbeat line -- silently drained
        # so the reader keeps up and the serial buffer never backs up.


def send(ser, reply_q, cmd, timeout=2.0):
    """Drains any stale queued reply first (e.g. a heartbeat-timed OK from
    a command whose reply we already gave up on), then sends cmd and waits
    for the next OK/ERR/STATUS line."""
    while not reply_q.empty():
        try:
            reply_q.get_nowait()
        except queue.Empty:
            break
    ser.write((cmd + "\n").encode("ascii"))
    try:
        return reply_q.get(timeout=timeout)
    except queue.Empty:
        return None


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None)
    args = parser.parse_args()

    import serial

    port = args.port or find_fta_port()
    if port is None:
        print("No ST-Link serial port found -- pass --port explicitly, or "
              "check the Nucleo's USB cable is connected to this machine.")
        raise SystemExit(1)
    print(f"Connecting to {port} @ {FTA_BAUD}")
    ser = serial.Serial(port, FTA_BAUD, timeout=0.2)
    time.sleep(2)
    ser.reset_input_buffer()

    reply_q = queue.Queue()
    stop_event = threading.Event()
    reader = threading.Thread(target=reader_thread, args=(ser, reply_q, stop_event), daemon=True)
    reader.start()

    print(send(ser, reply_q, "get_status") or "(no reply -- check the serial link/firmware)")
    print("\nManual DAC control. Amp starts however it was left -- check the")
    print("status line above. Type 'h' for commands, 'quit' to exit safely.\n")

    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()

            if cmd in ("quit", "exit", "q"):
                break
            elif cmd in ("h", "help", "?"):
                print(HELP_TEXT)
            elif cmd == "x" and len(parts) == 2:
                print(send(ser, reply_q, f"set_x {parts[1]}") or "(no reply)")
            elif cmd == "y" and len(parts) == 2:
                print(send(ser, reply_q, f"set_y {parts[1]}") or "(no reply)")
            elif cmd == "center" and len(parts) == 1:
                print(send(ser, reply_q, "set_x 95") or "(no reply)")
                print(send(ser, reply_q, "set_y 95") or "(no reply)")
            elif cmd == "amp" and len(parts) == 2 and parts[1] in ("on", "off"):
                print(send(ser, reply_q, "amp_enable" if parts[1] == "on" else "amp_disable") or "(no reply)")
            elif cmd == "status" and len(parts) == 1:
                print(send(ser, reply_q, "get_status") or "(no reply)")
            elif cmd == "estop" and len(parts) == 1:
                ser.write(b"!")
                time.sleep(0.1)
                print(send(ser, reply_q, "get_status") or "(sent -- no status reply)")
            elif cmd == "clear" and len(parts) == 1:
                print(send(ser, reply_q, "clear_estop") or "(no reply)")
            elif cmd == "raw" and len(parts) >= 2:
                print(send(ser, reply_q, line[4:]) or "(no reply)")
            else:
                print(f"Unrecognized command '{line}' -- type 'h' for help.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("Restoring idle state (x=95 y=95 amp off)...")
        send(ser, reply_q, "set_x 95")
        send(ser, reply_q, "set_y 95")
        send(ser, reply_q, "amp_disable")
        stop_event.set()
        reader.join(timeout=1.0)
        ser.close()


if __name__ == "__main__":
    main()
