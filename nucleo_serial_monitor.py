#!/usr/bin/env python3
"""
Reads and timestamps the debug line stream printed by camera_centroid_receiver
(the Nucleo firmware in this project's STM32CubeIDE workspace) over its
USART2 -> ST-Link VCP UART, one line per successfully checksummed I2C packet
received from nucleo_i2c_sender.py / beam_position_streamer.py on the Pi:

    seq= 12 status=1 x=   412 y=   289 pkts=13 errs=0

Run this on the laptop (not the Pi) while the Nucleo is flashed and connected
via its single USB cable, and while the Pi-side sender is running and wired
to the Nucleo over I2C.

Usage:
    python nucleo_serial_monitor.py            # auto-detect the ST-Link port
    python nucleo_serial_monitor.py COM5       # use an explicit port

Requires pyserial: pip install pyserial
"""
import sys
import time

import serial
from serial.tools import list_ports

BAUD = 115200


def find_stlink_port():
    """Best-effort auto-detect -- ST-Link's VCP normally identifies itself
    with "STMicroelectronics"/"ST-Link" somewhere in its USB description.
    Returns None (not an error) if that doesn't pan out, so the caller can
    fall back to listing everything for the user to pick from by hand."""
    candidates = [
        p for p in list_ports.comports()
        if any(tag in (p.description or "") for tag in ("STLink", "ST-Link", "STMicroelectronics"))
    ]
    if len(candidates) == 1:
        return candidates[0].device
    if len(candidates) > 1:
        print("Multiple ST-Link-looking ports found -- re-run with one explicitly:")
        for p in candidates:
            print(f"  {p.device}  -- {p.description}")
        sys.exit(1)
    return None


def main():
    if len(sys.argv) > 1:
        port_name = sys.argv[1]
    else:
        port_name = find_stlink_port()
        if port_name is None:
            print("No ST-Link VCP port auto-detected. Available ports:")
            for p in list_ports.comports():
                print(f"  {p.device}  -- {p.description}")
            print("\nRe-run with an explicit port, e.g.: python nucleo_serial_monitor.py COM5")
            sys.exit(1)

    print(f"Opening {port_name} @ {BAUD} -- Ctrl+C to stop")
    with serial.Serial(port_name, BAUD, timeout=1) as ser:
        idle_seconds = 0
        try:
            while True:
                line = ser.readline()
                if not line:
                    # readline's 1s timeout expired with nothing received --
                    # print an occasional heartbeat so "no data yet" is
                    # distinguishable from the script having hung.
                    idle_seconds += 1
                    if idle_seconds % 5 == 0:
                        print(f"... no data received in {idle_seconds}s")
                    continue
                idle_seconds = 0
                ts = time.strftime("%H:%M:%S")
                print(f"[{ts}] {line.decode('ascii', errors='replace').rstrip()}")
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
