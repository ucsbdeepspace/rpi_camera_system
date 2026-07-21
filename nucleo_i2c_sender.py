#!/usr/bin/env python3
"""
Streams the latest beam centroid to an STM32 Nucleo over I2C, Pi as master /
Nucleo as slave -- Pi's I2C controllers have weak/awkward slave-mode support,
so master-writes-to-Nucleo is the natural direction, not the reverse.

Uses a small register-mapped packet (mirrors the same "register pointer +
data" convention this project already uses to talk to the OV9281 sensors --
see kernel_patch/ov9282/ov9282.c's OV9282_REG_* pattern):

  reg 0x00        seq       (u8)  increments every send, wraps 0-255 --
                                    lets the Nucleo detect a stale/stuck link
  reg 0x01        status    (u8)  bit0 = beam confidently detected this cycle
  reg 0x02-0x03   x         (s16, little-endian) centroid column * POSITION_SCALE
  reg 0x04-0x05   y         (s16, little-endian) centroid row * POSITION_SCALE
  reg 0x06        checksum  (u8)  additive sum of regs 0x00-0x05, mod 256

x/y are fixed-point, not raw integer pixels: find_beam_blob() computes a
sub-pixel (intensity-weighted) centroid, and sending it pre-rounded to the
nearest whole pixel silently threw that precision away. Scaling by
POSITION_SCALE (10) before packing into the same s16 field preserves one
decimal digit without widening the packet -- the sensor's 1280x800 extent
scales to +-12800/+-8000, comfortably inside s16's +-32767 range.
**Firmware must divide the received x/y by POSITION_SCALE to recover real
pixel coordinates** -- this is a wire-format change, old firmware reading
these fields as raw pixels will see values 10x too large.

7 data bytes + 1 register-pointer byte = 8 bytes/write, well under any I2C
transaction size limit and fast even at Standard Mode (100kHz). No
request/response handshake needed beyond this -- I2C's own ACK per byte is
the only "handshake" a plain streaming write requires; `seq` and the stale
check on the Nucleo side are what stand in for "did this actually update
recently," not a protocol-level requirement.

Uses smbus2 directly (not a v4l2-ctl subprocess, unlike roi_set_selection.py
in this repo) -- that subprocess overhead, measured ~7-10ms/call elsewhere
in this project, is fine for an occasional ROI move but not for a per-frame
tracking send.

Requires a GENERAL-PURPOSE i2c bus, not the camera control buses
(i2c@88000/i2c@80000) -- those are owned by the kernel camera driver and
this must not contend with it. On this Pi 5, the header I2C bus is enabled
via `dtparam=i2c_arm=on` (already uncommented, top-of-file/global scope,
in /boot/firmware/config.txt) and confirmed live at `/dev/i2c-1` (backed
by RP1's `i2c@74000` controller, per
`/proc/device-tree/aliases/i2c1` -- despite RP1 renumbering the camera
buses to i2c-10/11, the header bus kept the classic `i2c-1` number).
Confirmed 2026-07-15 with `sudo i2cdetect -y 1`: bus responds, scans clean
(no devices -- expected with no Nucleo wired up yet).

NUCLEO_I2C_BUS=1 and NUCLEO_I2C_ADDR=0x42 are both confirmed correct
against the real Nucleo firmware (camera_centroid_receiver, see
CLAUDE.md) -- not placeholders.

End-to-end link confirmed live 2026-07-21 (seq/pkts in lockstep, zero
checksum errors) against the PRE-fixed-point protocol (whole-pixel x/y).
The POSITION_SCALE change above has not yet been validated against real
firmware -- the Nucleo side must be updated to divide by POSITION_SCALE
before that's true again.
"""
import struct
import time

from smbus2 import SMBus, i2c_msg

NUCLEO_I2C_BUS = 1       # confirmed 2026-07-15 -- header bus, see module docstring
NUCLEO_I2C_ADDR = 0x42   # confirmed against the real Nucleo firmware
                           # (camera_centroid_receiver), not a placeholder

REG_POINTER = 0x00  # single fixed packet shape, no real addressable
                      # register file on the Nucleo side -- kept for
                      # convention-consistency with the OV9281 protocol,
                      # not because anything reads this back

POSITION_SCALE = 10  # x/y are sent as round(real_pixel_value * POSITION_SCALE)
                       # -- see module docstring's "x/y are fixed-point" note.
                       # Firmware must divide by this to recover real pixels.


def _checksum(payload):
    """Additive checksum over the 6 data bytes (seq, status, x_lo, x_hi,
    y_lo, y_hi), mod 256. I2C's per-byte ACK only proves each byte was
    clocked in, not that the packet as a whole is uncorrupted -- this
    catches noise on the physical link the ACK alone wouldn't."""
    return sum(payload) & 0xFF


class NucleoLink:
    """Holds the open bus handle and the running sequence counter. Open
    once at process start and reuse -- don't reopen per-send, that would
    add real overhead at tracking-loop rates."""

    def __init__(self, bus_number=NUCLEO_I2C_BUS, addr=NUCLEO_I2C_ADDR):
        self.bus = SMBus(bus_number)
        self.addr = addr
        self._seq = 0

    def send_position(self, x, y, valid=True):
        """Push the latest centroid. `x`/`y` are REAL PIXEL coordinates
        (float or int, e.g. a sub-pixel centroid straight out of
        find_beam_blob) in whatever coordinate space the Nucleo firmware
        expects (e.g. full-sensor rows/columns) -- pick one convention and
        keep both sides consistent, this module doesn't care which. This
        method scales by POSITION_SCALE and rounds to the nearest int16
        internally, so callers should NOT pre-round -- doing so throws away
        the sub-pixel precision this scaling exists to preserve. Safe to
        call every tracking cycle: each write is 9 bytes total, well under
        1ms at Fast Mode (400kHz).

        `valid=False` still sends a packet (with whatever x/y is passed,
        e.g. the last known position) rather than skipping the send -- the
        Nucleo needs an explicit "beam not found" signal to react
        immediately (e.g. hold position), not silence it can only
        interpret after timing out, which would make a real "beam lost"
        indistinguishable from a dead link until that timeout passes.
        """
        self._seq = (self._seq + 1) % 256
        status = 1 if valid else 0
        x_scaled = int(round(x * POSITION_SCALE))
        y_scaled = int(round(y * POSITION_SCALE))
        payload = struct.pack('<BB', self._seq, status) + struct.pack('<hh', x_scaled, y_scaled)
        checksum = _checksum(payload)
        write = i2c_msg.write(self.addr, [REG_POINTER] + list(payload) + [checksum])
        self.bus.i2c_rdwr(write)

    def close(self):
        self.bus.close()


if __name__ == "__main__":
    # Manual smoke test: sends a slowly-orbiting fake position so the
    # Nucleo side can be checked without a real beam. Ctrl-C to stop.
    import math

    link = NucleoLink()
    print(f"Sending to bus {NUCLEO_I2C_BUS} addr 0x{NUCLEO_I2C_ADDR:02x} -- Ctrl-C to stop")
    t = 0.0
    try:
        while True:
            x = int(400 + 100 * math.sin(t))
            y = int(300 + 50 * math.cos(t))
            link.send_position(x, y, valid=True)
            print(f"sent x={x} y={y} seq={link._seq}")
            t += 0.1
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
