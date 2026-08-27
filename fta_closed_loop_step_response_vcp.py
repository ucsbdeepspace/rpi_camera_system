#!/usr/bin/env python3
"""
Characterizes the CLOSED-LOOP step response of the dac_y -> cx PID pathway
implemented in camera_centroid_receiver (MODE_CLOSED_LOOP, added
2026-08-13) -- rise time, overshoot, settling time of the measured pixel
error following a target_x step, the same three metrics
fta_step_response_test_vcp.py already computes for the OPEN-LOOP plant.
That script characterizes the actuator itself; this one characterizes the
actual control loop that will run in real operation.

Only the dac_y->cx pathway exists in firmware right now (the other axis
was never implemented -- see CLAUDE.md's "Architecture DECISION v2" and
the 2026-08-12 axis-choice section), so there's no --axis option the way
the open-loop script has one.

Bench findings this session (2026-08-13, see CLAUDE.md) this script's
defaults are built on:
  - dac_y=2048 is the cleanest, most linear region found on this rig this
    session (minor-loop hysteresis gap ~0.04px, consistent slope both
    directions) -- default --base-dac-y.
  - Kp=1.75 counts/px, Ki=200 counts/(px*s) give a clean, single-
    transition step response in ~141ms (rise and settling both), used as
    this script's own first real result. An EARLIER interactive escalation
    search (crude ~3-4Hz terminal polling, not this script) claimed no
    overshoot existed up to Ki=300 -- that claim turned out to be a
    measurement-resolution artifact, not a real absence of overshoot: this
    script's own high-rate (~135Hz) logging found real, visible ringing at
    Ki=400 (15% overshoot), and the interactive search's polling rate was
    too slow to resolve oscillation on that timescale. Raising Kp instead
    of Ki was tried and made things WORSE (more ringing, no faster
    settling) -- Ki was the actual bottleneck the whole time, not Kp.
    See CLAUDE.md for the full comparison table.
  - A single ser.write() burst of a whole VCP command line reliably loses
    bytes at the Pi's current high telemetry rate (~150-200Hz) -- every
    SETUP command here is sent via send_command(), which paces the write
    at ~20ms/char and confirms the reply, exactly the workaround found
    this session. The STEP command itself (the one whose exact timing
    this script measures) is sent as a single fast burst instead, to
    preserve precise step-onset timing the way fta_step_response_test_vcp.py
    does -- but verified via a paced get_status AFTER recording stops, so
    a silently-dropped step (which would otherwise just look like "the
    actuator didn't respond") is caught and reported instead of trusted.

Usage:
  python3 fta_closed_loop_step_response_vcp.py [--base-dac-y N]
      [--step-px N] [--kp-milli N] [--ki-milli N] [--pre-s SEC]
      [--post-s SEC] [--settle-tol-px PX] [--port PORT] [--out PATH]

    --base-dac-y N     open-loop pre-position before engaging the loop,
                        default 2048 (see above).
    --step-px N         target_x step size in pixels, default -25
                        (matches this session's tested convention; sign
                        chooses direction, magnitude should stay in the
                        small-step regime this project has established
                        elsewhere -- large steps haven't been
                        characterized for this control pathway).
    --kp-milli N        Kp * 1000 (firmware's own units), default 1750.
    --ki-milli N        Ki * 1000, default 200000.
    --pre-s SEC         seconds recorded BEFORE the step, holding at
                        target=baseline under closed-loop control (not
                        open-loop -- this captures the loop's own hold
                        noise/behavior, not just plant baseline), default
                        0.5.
    --post-s SEC        seconds recorded AFTER the step, default 3.0.
    --settle-tol-px PX  settling-band tolerance, default 2.0px (6.0um).
    --port PORT         Nucleo VCP serial port, default auto-detect.
    --out PATH          raw (t, x, y) npz path -- default
                        results/fta_closed_loop_step_response_vcp_<UTC
                        timestamp>.npz. A PNG plot is saved alongside it
                        (same path, .png extension).

Requires the Pi to already be streaming telemetry, same precondition as
fta_step_response_test_vcp.py -- checked via get_status's tel_age_ms
before starting.
"""
import argparse
import re
import threading
import time
from datetime import datetime, timezone

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FTA_BAUD = 460800  # camera_centroid_receiver's USART2 rate -- raised from
                    # 115200 back to 460800 on 2026-08-13, now matching
                    # the old "FTA Controller"'s rate again. Needed the
                    # whole project's clock tree raised too (4MHz -> 16MHz
                    # HSI) since 460800 was unreachable at 4MHz -- see
                    # CLAUDE.md for the full story.
MICRONS_PER_PIXEL = 3.0  # OV9281 pixel pitch, same constant used throughout this project.

REPLY_RE = re.compile(r"^(OK|ERR|STATUS|WARN)\b")
TELEMETRY_RE = re.compile(
    r"^seq=\s*(\d+)\s+status=(\d+)\s+x=(-?\d+\.\d)\s+y=(-?\d+\.\d)\s+"
    r"tgt=(-?\d+\.\d)\s+dac_y=(-?\d+)\s+tick=(\d+)\s+pkts=(\d+)\s+errs=(\d+)$")
STATUS_FIELD_RE = {
    "dac_x": re.compile(r"dac_x=(-?\d+)"),
    "dac_y": re.compile(r"dac_y=(-?\d+)"),
    "amp": re.compile(r"amp=(\d+)"),
    "tel_x": re.compile(r"tel_x=(-?[\d.]+)"),
    "tel_age_ms": re.compile(r"tel_age_ms=(\d+)"),
    "target_x": re.compile(r"target_x=(-?[\d.]+)"),
    "target_x_set": re.compile(r"target_x_set=(\d+)"),
    "notch": re.compile(r"notch=(\d+)"),
    "notch_freq_millihz": re.compile(r"notch_freq_millihz=(-?\d+)"),
    "notch_q_milli": re.compile(r"notch_q_milli=(-?\d+)"),
    "lead": re.compile(r"lead=(\d+)"),
    "lead_fz_millihz": re.compile(r"lead_fz_millihz=(-?\d+)"),
    "lead_fp_millihz": re.compile(r"lead_fp_millihz=(-?\d+)"),
    "out_limit": re.compile(r"out_limit=(-?\d+)"),
    "ctrl_rate_millihz": re.compile(r"ctrl_rate_millihz=(-?\d+)"),
    "ctrl_interval_ms": re.compile(r"ctrl_interval_ms=(\d+)"),
    "smoothing": re.compile(r"smoothing=(\d+)"),
    "axis2": re.compile(r"axis2=(\d+)"),
}

BLUE = "#2a78d6"
TARGET_COLOR = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"


def find_fta_port():
    from serial.tools import list_ports
    candidates = [
        p for p in list_ports.comports()
        if any(tag in (p.description or "") for tag in ("STLink", "ST-Link", "STMicroelectronics"))
    ]
    return candidates[0].device if candidates else None


def send_command(ser, cmd, char_delay=0.02, reply_timeout=2.0):
    """Paces the write at ~20ms/char rather than one ser.write() burst --
    found necessary this session (2026-08-13, see CLAUDE.md and this
    module's docstring): a burst write of a whole command line reliably
    loses bytes at the Pi's current high telemetry rate. Returns the
    first OK/ERR/STATUS/WARN reply line seen, or None on timeout.

    Clears stale input right before writing (2026-08-19 fix): at the
    ~465Hz telemetry rate, a command whose reply never arrives leaves
    ~2s of already-buffered telemetry sitting unread. Without this reset,
    the NEXT command's reply-matching window gets spent draining that
    stale backlog instead of watching for a genuinely fresh reply --
    confirmed directly to cascade into repeated timeouts (get_status
    itself failing 5/5 retries) even though get_status called in
    isolation right afterward succeeds instantly."""
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
    """Paced get_status with field parsing, retried on a corrupted/missing
    reply rather than trusting the first attempt."""
    for _ in range(retries):
        reply = send_command(ser, "get_status")
        if reply is None or not reply.startswith("STATUS"):
            continue
        matches = {k: rx.search(reply) for k, rx in STATUS_FIELD_RE.items()}
        if all(matches.values()):
            return {
                "dac_x": int(matches["dac_x"].group(1)),
                "dac_y": int(matches["dac_y"].group(1)),
                "amp": int(matches["amp"].group(1)),
                "tel_x": float(matches["tel_x"].group(1)),
                "tel_age_ms": int(matches["tel_age_ms"].group(1)),
                "target_x": float(matches["target_x"].group(1)),
                "target_x_set": int(matches["target_x_set"].group(1)),
                "notch": int(matches["notch"].group(1)),
                "notch_freq_millihz": int(matches["notch_freq_millihz"].group(1)),
                "notch_q_milli": int(matches["notch_q_milli"].group(1)),
                "lead": int(matches["lead"].group(1)),
                "lead_fz_millihz": int(matches["lead_fz_millihz"].group(1)),
                "lead_fp_millihz": int(matches["lead_fp_millihz"].group(1)),
                "out_limit": int(matches["out_limit"].group(1)),
                "ctrl_rate_millihz": int(matches["ctrl_rate_millihz"].group(1)),
                "ctrl_interval_ms": int(matches["ctrl_interval_ms"].group(1)),
                "smoothing": int(matches["smoothing"].group(1)),
                "axis2": int(matches["axis2"].group(1)),
            }
    raise RuntimeError("No parseable get_status reply after several attempts -- check the serial link/firmware.")


def analyze_step(t, primary, t_step, settle_tol_px):
    """Based on fta_step_response_test_vcp.py's analyze_step (duplicated
    per this project's established convention -- see that function's own
    docstring), but with a real bug fixed: that version's first_crossing
    switched from `frac >= target` to `frac <= target` when delta<0, on
    the assumption a falling signal needs the comparison flipped. It
    doesn't -- frac = (v - baseline) / delta is already sign-normalized
    by the division, so it goes 0 -> 1 as the signal moves from baseline
    to final REGARDLESS of whether the raw value is rising or falling
    (verified against this script's own real step data, 2026-08-13: a
    -25px step's frac trace climbs 0->1 monotonically like any other,
    confirmed with a hand-checked synthetic example too). The old
    conditional made first_crossing(0.90) trigger on the very first
    post-step sample for any falling step (any frac below 0.90, including
    near-zero noise, satisfied `frac <= 0.90`), landing t90 before t10 and
    silently reporting "rise time: could not be determined" every single
    time -- not a fluke, this would have hit every closed-loop test this
    session, all of which stepped in the negative direction. Only
    first_crossing itself needed the fix; everything else in this
    function (baseline/final/overshoot/settling) was unaffected since
    those don't depend on frac's crossing direction."""
    pre_mask = t < t_step
    post_mask = ~pre_mask
    if pre_mask.sum() < 3 or post_mask.sum() < 3:
        return None

    baseline = float(np.mean(primary[pre_mask]))
    tail_mask = post_mask & (t > t[-1] - 0.2 * (t[-1] - t_step))
    final = float(np.mean(primary[tail_mask])) if tail_mask.sum() >= 3 else float(primary[post_mask][-1])

    delta = final - baseline
    if delta == 0:
        return {"baseline": baseline, "final": final, "delta": delta,
                "rise_time_s": None, "overshoot_pct": None, "settling_time_s": None,
                "note": "no net change detected"}

    post_t = t[post_mask] - t_step
    post_v = primary[post_mask]
    frac = (post_v - baseline) / delta

    def first_crossing(target_frac):
        idx = np.where(frac >= target_frac)[0]
        return post_t[idx[0]] if len(idx) else None

    t10 = first_crossing(0.10)
    t90 = first_crossing(0.90)
    rise_time = (t90 - t10) if (t10 is not None and t90 is not None and t90 >= t10) else None

    # Same sign-normalization argument as first_crossing above: frac already
    # reads 0->1(+) regardless of delta's sign, so the peak (overshoot) is
    # always just its max -- the old delta<0 branch (`-np.min(frac)`) measured
    # something unrelated to overshoot and could silently under-report it.
    max_frac = float(np.max(frac))
    overshoot_pct = max(0.0, (max_frac - 1.0) * 100.0)

    within_tol = np.abs(post_v - final) <= settle_tol_px
    settling_time = None
    for i in range(len(post_t)):
        if np.all(within_tol[i:]):
            settling_time = post_t[i]
            break

    return {
        "baseline": baseline, "final": final, "delta": delta,
        "rise_time_s": rise_time, "overshoot_pct": overshoot_pct,
        "settling_time_s": settling_time,
        "note": None if settling_time is not None else
                f"never stayed within {settle_tol_px}px of final for the rest "
                "of the recorded window -- widen --post-s to get a real number",
    }


def _reader_thread(ser, t0, records, stop_event):
    """Sole reader of the serial port for the whole recording window --
    same discipline as fta_step_response_test_vcp.py: the main thread only
    writes (the single burst step command) while this thread is running,
    never reads, so lines never get split across two concurrent readers.

    Records BOTH a host arrival timestamp and the firmware's own tick=
    field per sample (2026-08-19 fix): host arrival timestamps alone get
    batched into ~15-16ms bursts by Windows thread-scheduling granularity
    (confirmed directly -- 85% of consecutive samples landed on the exact
    same host timestamp in one recorded run), the same bug already fixed
    in fta_ringdown_test.py and fta_closed_loop_onboard_sine_test.py.
    Unlike those two, this script's t_step is measured on the HOST clock
    (the step command has no firmware-reported echo the way the sine
    generator's tgt= field provides), so main() fits an affine mapping
    between host time and firmware tick using every recorded sample
    (least-squares over thousands of points averages out the per-sample
    jitter) rather than just switching wholesale to tick-based time."""
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except Exception:
            continue
        if not raw:
            continue
        host_now = time.monotonic() - t0
        m = TELEMETRY_RE.match(raw.decode(errors="replace").strip())
        if not m:
            continue
        status = int(m.group(2))
        if not (status & 1):
            continue
        x = float(m.group(3))
        y = float(m.group(4))
        tick_ms = int(m.group(7))
        records.append((host_now, x, y, tick_ms))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base-dac-y", type=int, default=2048)
    parser.add_argument("--step-px", type=float, default=-25.0)
    parser.add_argument("--kp-milli", type=int, default=1750)
    parser.add_argument("--ki-milli", type=int, default=200000)
    parser.add_argument("--kd-milli", type=int, default=0)
    parser.add_argument("--fc-milli", type=int, default=None,
                         help="derivative filter cutoff, milli-Hz (e.g. 5000 = 5.0Hz); "
                              "omit to leave firmware's current fc unchanged")
    parser.add_argument("--out-limit", type=int, default=None,
                         help="symmetric +-limit (DAC counts) passed to PIDController's "
                              "setOutputLimits(), e.g. 500; omit to leave firmware's current "
                              "limit unchanged (defaults to +-3905, the full DAC span, at boot)")
    parser.add_argument("--ctrl-rate-milli", type=int, default=None,
                         help="throttle the control loop to this rate, milli-Hz (e.g. 200000 = "
                              "200Hz), and set the PID's ts_s to match, in one firmware command; "
                              "0 disables the throttle (full telemetry-driven rate, the default); "
                              "omit to leave firmware's current setting unchanged")
    parser.add_argument("--smoothing", type=int, default=None, choices=[0, 1],
                         help="0/1: feed the PID the mean of every confident sample since the "
                              "last control step (boxcar anti-aliasing pre-filter) instead of "
                              "just the latest raw sample; independent of --ctrl-rate-milli; "
                              "omit to leave firmware's current setting unchanged")
    parser.add_argument("--axis2", type=int, default=None, choices=[0, 1],
                         help="0/1: enable/disable the second control axis (dac_x <- cy) -- "
                              "0 leaves dac_x fixed at its bumpless-transfer base instead of "
                              "correcting, for A/B comparison; omit to leave firmware's current "
                              "setting unchanged")
    parser.add_argument("--notch-freq-milli", type=int, default=None,
                         help="resonance notch filter center freq, milli-Hz (e.g. 38500 = 38.5Hz); "
                              "omit to leave the notch in its current state (disabled by default)")
    parser.add_argument("--notch-q-milli", type=int, default=3000,
                         help="notch filter Q*1000 (default 3000 = Q of 3.0), only used if "
                              "--notch-freq-milli is given")
    parser.add_argument("--notch-off", action="store_true",
                         help="explicitly disable the notch filter before this run (sends "
                              "notch_off) -- use this rather than just omitting "
                              "--notch-freq-milli if a PREVIOUS run may have left it enabled, "
                              "since the firmware's notch state persists across runs")
    parser.add_argument("--lead-fz-milli", type=int, default=None,
                         help="lead compensator zero freq, milli-Hz (e.g. 9600 = 9.6Hz); "
                              "must be paired with --lead-fp-milli (fz < fp required by firmware); "
                              "omit to leave the lead filter in its current state (disabled by "
                              "default). Uses whatever ctrl_rate is CURRENTLY set when this runs, "
                              "so pass --ctrl-rate-milli in the same invocation if changing both.")
    parser.add_argument("--lead-fp-milli", type=int, default=None,
                         help="lead compensator pole freq, milli-Hz (e.g. 65000 = 65Hz); "
                              "must be paired with --lead-fz-milli")
    parser.add_argument("--lead-off", action="store_true",
                         help="explicitly disable the lead compensator before this run (sends "
                              "lead_off) -- same reasoning as --notch-off: firmware state "
                              "persists across runs")
    parser.add_argument("--pre-s", type=float, default=0.5)
    parser.add_argument("--post-s", type=float, default=3.0)
    parser.add_argument("--settle-tol-px", type=float, default=2.0)
    parser.add_argument("--port", default=None)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    import serial

    port = args.port or find_fta_port()
    if port is None:
        print("No ST-Link serial port found -- pass --port explicitly.")
        raise SystemExit(1)
    print(f"Connecting to {port} @ {FTA_BAUD}")
    ser = serial.Serial(port, FTA_BAUD, timeout=0.2)
    time.sleep(2)
    ser.reset_input_buffer()

    print(send_command(ser, "clear_estop"))
    print(send_command(ser, "set_mode open_loop"))

    st = get_status(ser)
    if st["tel_age_ms"] > 500:
        print(f"ERR: last relayed I2C telemetry is {st['tel_age_ms']}ms old -- "
              "nothing appears to be streaming from the Pi. Start "
              "camera_view_tool.py or beam_position_streamer.py there first.")
        ser.close()
        raise SystemExit(1)

    amp_was_enabled = bool(st["amp"])
    if not amp_was_enabled:
        print(send_command(ser, "amp_enable"))
        st = get_status(ser)
        if not st["amp"]:
            print("ERR: sent amp_enable but get_status still reports amp=off -- aborting.")
            ser.close()
            raise SystemExit(1)

    print(f"Pre-positioning dac_y={args.base_dac_y} (open_loop)...")
    print(send_command(ser, f"set_y {args.base_dac_y}"))
    time.sleep(0.5)

    st = get_status(ser)
    baseline_cx = st["tel_x"]
    target_from = round(baseline_cx)
    target_to = round(baseline_cx + args.step_px)
    print(f"baseline cx={baseline_cx:.1f}  target_from={target_from}  "
          f"target_to={target_to} (step {args.step_px:+.1f}px)  "
          f"Kp_milli={args.kp_milli} Ki_milli={args.ki_milli} Kd_milli={args.kd_milli}")

    print(send_command(ser, f"set_target_x {target_from}"))
    print(send_command(ser, f"set_kp {args.kp_milli}"))
    print(send_command(ser, f"set_ki {args.ki_milli}"))
    print(send_command(ser, f"set_kd {args.kd_milli}"))
    if args.fc_milli is not None:
        print(send_command(ser, f"set_fc {args.fc_milli}"))
    if args.out_limit is not None:
        print(send_command(ser, f"set_out_limit {args.out_limit}"))
    if args.ctrl_rate_milli is not None:
        print(send_command(ser, f"set_ctrl_rate {args.ctrl_rate_milli}"))
    if args.smoothing is not None:
        print(send_command(ser, f"set_smoothing {args.smoothing}"))
    if args.axis2 is not None:
        print(send_command(ser, f"set_axis2 {args.axis2}"))
    if args.notch_off:
        print(send_command(ser, "notch_off"))
    elif args.notch_freq_milli is not None:
        print(send_command(ser, f"set_notch {args.notch_freq_milli} {args.notch_q_milli}"))
    # Lead compensator sent AFTER ctrl_rate above -- cmd_set_lead configures
    # its filter coefficients using whatever g_ctrl_rate_millihz is set to
    # AT THAT MOMENT (see main.c's lead_filter_t comment on why it's tied
    # to the real current rate rather than a hardcoded constant, unlike the
    # notch's known stale-457.5Hz bug) -- so ctrl_rate must land first if
    # both are being changed in the same invocation.
    if args.lead_off:
        print(send_command(ser, "lead_off"))
    elif args.lead_fz_milli is not None and args.lead_fp_milli is not None:
        print(send_command(ser, f"set_lead {args.lead_fz_milli} {args.lead_fp_milli}"))
    elif args.lead_fz_milli is not None or args.lead_fp_milli is not None:
        print("ERR: --lead-fz-milli and --lead-fp-milli must both be given together -- aborting.")
        ser.close()
        raise SystemExit(1)
    # Ground truth, not just an echo of the CLI args: the firmware's notch/
    # lead state persists in RAM across runs, so a run that passes none of
    # these flags could still be running with a filter some EARLIER run
    # left enabled. Read it back rather than assume.
    notch_st = get_status(ser)
    notch_active = bool(notch_st["notch"])
    notch_freq_hz = notch_st["notch_freq_millihz"] / 1000.0
    notch_q = notch_st["notch_q_milli"] / 1000.0
    lead_active = bool(notch_st["lead"])
    lead_fz_hz = notch_st["lead_fz_millihz"] / 1000.0
    lead_fp_hz = notch_st["lead_fp_millihz"] / 1000.0
    out_limit = notch_st["out_limit"]
    ctrl_rate_millihz = notch_st["ctrl_rate_millihz"]
    ctrl_interval_ms = notch_st["ctrl_interval_ms"]
    smoothing = bool(notch_st["smoothing"])
    axis2 = bool(notch_st["axis2"])
    print(f"notch: {'ON @ ' + format(notch_freq_hz, '.1f') + 'Hz Q=' + format(notch_q, '.1f') if notch_active else 'OFF'}")
    print(f"lead: {'ON  fz=' + format(lead_fz_hz, '.1f') + 'Hz fp=' + format(lead_fp_hz, '.1f') + 'Hz' if lead_active else 'OFF'}")
    print(f"ctrl_rate: {ctrl_rate_millihz/1000.0:.1f}Hz (throttle interval {ctrl_interval_ms}ms, "
          f"0=unthrottled)")
    print(f"out_limit: +-{out_limit} counts")
    print(f"smoothing (boxcar pre-filter): {'ON' if smoothing else 'OFF'}")
    print(f"axis2 (dac_x <- cy): {'ON' if axis2 else 'OFF (dac_x held fixed)'}")
    print(send_command(ser, "set_mode closed_loop"))
    time.sleep(0.3)  # let it settle at the zero-error hold before recording

    records = []
    stop_event = threading.Event()
    t0 = time.monotonic()
    reader = threading.Thread(target=_reader_thread, args=(ser, t0, records, stop_event), daemon=True)
    reader.start()

    time.sleep(args.pre_s)
    # The measured step. A single burst write here was tried first and
    # DID lose bytes on the very first live run of this script (caught by
    # the post-recording target_x verification below, not just a
    # theoretical risk) -- paced instead, matching send_command's write
    # side. Deliberately does NOT read a reply here: the reader thread
    # owns ser.readline() for the whole recording window, and having two
    # threads read the same Serial object concurrently would race and
    # could corrupt the telemetry stream itself. t_step is stamped after
    # the last character is sent, not after a confirmed reply -- still
    # far more precise than risking the whole step going missing.
    for ch in f"set_target_x {target_to}\n":
        ser.write(ch.encode("ascii"))
        time.sleep(0.02)
    t_step_host = time.monotonic() - t0
    time.sleep(args.post_s)

    stop_event.set()
    reader.join(timeout=1.0)

    # Verify the step command actually landed -- a burst write CAN still
    # lose bytes (that's exactly why setup commands above are paced); this
    # confirms the recorded data reflects a real step, not a dropped one.
    st = get_status(ser)
    if round(st["target_x"]) != target_to:
        print(f"WARNING: post-recording get_status reports target_x={st['target_x']}, "
              f"not the intended {target_to} -- the step command was likely corrupted "
              "in flight. Treat this run's data as suspect; rerun.")

    print(send_command(ser, "set_mode open_loop"))
    print(send_command(ser, "set_y 95"))
    if not amp_was_enabled:
        print(send_command(ser, "amp_disable"))
    print("final:", get_status(ser))
    ser.close()

    if len(records) < 6:
        print(f"Only {len(records)} usable telemetry samples -- not enough to analyze.")
        return

    host_arr = np.array([r[0] for r in records])
    x = np.array([r[1] for r in records])
    y = np.array([r[2] for r in records])
    tick_ms = np.array([r[3] for r in records], dtype=np.float64)
    # Fit an affine mapping from host time to firmware tick (ms) using
    # every sample -- least squares over thousands of points averages out
    # the ~15-16ms OS-scheduling jitter on host_arr, giving a far more
    # precise t_step than trusting the host-clock value directly would
    # (see _reader_thread's docstring for why host timestamps alone can't
    # be trusted here).
    a, b = np.polyfit(host_arr, tick_ms, 1)
    tick_ms_at_step = a * t_step_host + b
    t = (tick_ms - tick_ms[0]) / 1000.0
    t_step = (tick_ms_at_step - tick_ms[0]) / 1000.0
    span = t[-1] - t[0]
    print(f"Captured {len(records)} telemetry samples ({span:.3f}s span, "
          f"~{len(records) / span if span > 0 else 0:.0f}/s average).")

    metrics = analyze_step(t, x, t_step, args.settle_tol_px)
    um = MICRONS_PER_PIXEL
    if metrics is None:
        print("Not enough samples before/after the step to compute metrics.")
    else:
        print(f"baseline={metrics['baseline']:.2f}px ({metrics['baseline']*um:.1f}um)  "
              f"final={metrics['final']:.2f}px ({metrics['final']*um:.1f}um)  "
              f"delta={metrics['delta']:.2f}px ({metrics['delta']*um:.1f}um)")
        if metrics["rise_time_s"] is not None:
            print(f"rise time (10%-90%): {metrics['rise_time_s'] * 1000:.1f}ms")
        else:
            print("rise time: could not be determined")
        if metrics["overshoot_pct"] is not None:
            print(f"overshoot: {metrics['overshoot_pct']:.1f}%")
        if metrics["settling_time_s"] is not None:
            print(f"settling time (within {args.settle_tol_px}px / "
                  f"{args.settle_tol_px*um:.1f}um): {metrics['settling_time_s'] * 1000:.1f}ms")
        if metrics["note"]:
            print(f"NOTE: {metrics['note']}")

    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_path = args.out or f"results/fta_closed_loop_step_response_vcp_{ts}.npz"
    np.savez(out_path, t=t, x=x, y=y, t_step=t_step,
              base_dac_y=args.base_dac_y, step_px=args.step_px,
              target_from=target_from, target_to=target_to,
              kp_milli=args.kp_milli, ki_milli=args.ki_milli, kd_milli=args.kd_milli,
              notch_active=notch_active, notch_freq_hz=notch_freq_hz, notch_q=notch_q,
              lead_active=lead_active, lead_fz_hz=lead_fz_hz, lead_fp_hz=lead_fp_hz,
              out_limit=out_limit, ctrl_rate_millihz=ctrl_rate_millihz,
              ctrl_interval_ms=ctrl_interval_ms, smoothing=smoothing, axis2=axis2)
    print(f"Saved raw time series to {out_path}")

    # --- plot --- two panels: cx (the driven axis) on top, cy (the OTHER
    # axis -- what axis2, when enabled, is trying to hold steady) below,
    # sharing the time axis, so a Y-step test directly shows whether the
    # second axis controller visibly changes cy's behavior.
    fig, (ax, ax_y) = plt.subplots(2, 1, figsize=(9, 6.5), dpi=150, sharex=True,
                                    gridspec_kw={"height_ratios": [1.4, 1]})
    for a in (ax, ax_y):
        a.set_facecolor("white")
        for spine in ("top", "right"):
            a.spines[spine].set_visible(False)
        for spine in ("left", "bottom"):
            a.spines[spine].set_color(GRID)
        a.tick_params(colors=MUTED, labelsize=9, length=3)

    ax.axhline(target_from, color=TARGET_COLOR, linewidth=1.0, linestyle=(0, (2, 2)), alpha=0.7)
    ax.plot([t_step, t[-1]], [target_to, target_to], color=TARGET_COLOR, linewidth=1.2,
            linestyle=(0, (2, 2)), label="target_x")
    ax.plot(t, x, color=BLUE, linewidth=1.4, label="measured cx")
    ax.axvline(t_step, color=MUTED, linewidth=0.8, linestyle=(0, (1, 2)))

    sec = ax.secondary_yaxis("right", functions=(lambda px: px * um, lambda v: v / um))
    sec.tick_params(colors=MUTED, labelsize=9, length=3)
    sec.set_ylabel("µm", fontsize=9, color=MUTED)

    ax.set_ylabel("cx (px)", fontsize=9.5, color=MUTED)
    ax.legend(frameon=False, fontsize=9, loc="upper right")

    ax_y.plot(t, y, color="#c9962c", linewidth=1.2, label="measured cy (other axis)")
    ax_y.axvline(t_step, color=MUTED, linewidth=0.8, linestyle=(0, (1, 2)))
    ax_y.set_xlabel("time (s)", fontsize=9.5, color=MUTED)
    ax_y.set_ylabel("cy (px)", fontsize=9.5, color=MUTED)
    y_std = y.std()
    y_range = y.max() - y.min()
    ax_y.text(0.02, 0.95, f"cy std={y_std:.2f}px  range={y_range:.2f}px", transform=ax_y.transAxes,
              fontsize=8.5, color="#0b0b0b", va="top", ha="left",
              bbox=dict(facecolor="white", edgecolor=GRID, alpha=0.9, pad=3))
    ax_y.legend(frameon=False, fontsize=8.5, loc="upper right")

    if metrics is not None:
        parts = [f"step: {args.step_px:+.1f}px @ dac_y={args.base_dac_y}",
                 f"Kp={args.kp_milli/1000:.2f} Ki={args.ki_milli/1000:.2f} Kd={args.kd_milli/1000:.2f}"]
        if out_limit < 3905:  # firmware default is +-3905 (full DAC span) -- only worth
            parts.append(f"out_limit: ±{out_limit} counts (tightened anti-windup)")  # flagging when tightened
        if ctrl_interval_ms > 0:
            parts.append(f"ctrl_rate: {ctrl_rate_millihz/1000.0:.0f}Hz (throttled, "
                          f"{ctrl_interval_ms}ms gate)")
        if smoothing:
            parts.append("smoothing: boxcar ON")
        if metrics["rise_time_s"] is not None:
            parts.append(f"rise: {metrics['rise_time_s']*1000:.0f}ms")
        if metrics["overshoot_pct"] is not None:
            parts.append(f"overshoot: {metrics['overshoot_pct']:.1f}%")
        if metrics["settling_time_s"] is not None:
            parts.append(f"settling ({args.settle_tol_px}px): {metrics['settling_time_s']*1000:.0f}ms")
        else:
            parts.append("settling: not reached in window")
        ax.text(0.02, 0.03, "\n".join(parts), transform=ax.transAxes, fontsize=8.5,
                color="#0b0b0b", va="bottom", ha="left",
                bbox=dict(facecolor="white", edgecolor=GRID, alpha=0.9, pad=4))

    # Notch-filter badge -- always shown (not just when active), high-
    # contrast, so a viewer glancing at a saved PNG can't mistake a
    # notch-filtered run for a plain-PID one or vice versa. Ground-truth
    # (from get_status), not just an echo of whatever CLI args were passed.
    if notch_active:
        notch_label = f"NOTCH ON: {notch_freq_hz:.1f}Hz  Q={notch_q:.1f}"
        notch_box = dict(facecolor="#fff3cd", edgecolor="#c9962c", alpha=0.95, pad=5)
        notch_color = "#7a5b00"
    else:
        notch_label = "notch: off"
        notch_box = dict(facecolor="white", edgecolor=GRID, alpha=0.8, pad=4)
        notch_color = MUTED
    ax.text(0.02, 0.97, notch_label, transform=ax.transAxes, fontsize=9,
            fontweight=("bold" if notch_active else "normal"), color=notch_color,
            va="top", ha="left", bbox=notch_box)

    # Lead-compensator badge -- stacked directly under the notch badge
    # (same top-left corner, same "always shown, ground-truth" reasoning).
    if lead_active:
        lead_label = f"LEAD ON: fz={lead_fz_hz:.1f}Hz  fp={lead_fp_hz:.1f}Hz"
        lead_box = dict(facecolor="#d9ecff", edgecolor="#2a78d6", alpha=0.95, pad=5)
        lead_color = "#0b3d73"
    else:
        lead_label = "lead: off"
        lead_box = dict(facecolor="white", edgecolor=GRID, alpha=0.8, pad=4)
        lead_color = MUTED
    ax.text(0.02, 0.89, lead_label, transform=ax.transAxes, fontsize=9,
            fontweight=("bold" if lead_active else "normal"), color=lead_color,
            va="top", ha="left", bbox=lead_box)

    # Control-rate throttle badge -- top-right (notch's badge owns top-left),
    # only shown when active: throttling is a deliberate deviation from real
    # operating conditions (not a normal tuning knob like Kp/Ki), so a
    # throttled run should never be mistaken for a full-rate one at a glance.
    if ctrl_interval_ms > 0:
        ax.text(0.98, 0.97, f"THROTTLED: {ctrl_rate_millihz/1000.0:.0f}Hz "
                f"(gate {ctrl_interval_ms}ms)", transform=ax.transAxes, fontsize=9,
                fontweight="bold", color="#8a1f1f", va="top", ha="right",
                bbox=dict(facecolor="#fde2e2", edgecolor="#b33a3a", alpha=0.95, pad=5))

    # Smoothing badge -- bottom-right (notch owns top-left, throttle owns
    # top-right), only shown when active for the same "never mistaken at
    # a glance" reasoning as the other two.
    if smoothing:
        ax.text(0.98, 0.03, "BOXCAR SMOOTHING ON", transform=ax.transAxes, fontsize=9,
                fontweight="bold", color="#1f6b3a", va="bottom", ha="right",
                bbox=dict(facecolor="#e3f5e8", edgecolor="#3a9c5c", alpha=0.95, pad=5))

    # axis2 badge -- on the cy panel itself, since that's the axis it
    # controls. Always shown (on AND off), since "was axis2 active" is
    # exactly the thing an A/B comparison plot needs to be unambiguous
    # about at a glance.
    axis2_label = "AXIS2 ON (dac_x correcting cy)" if axis2 else "axis2 OFF (dac_x held fixed)"
    axis2_box = (dict(facecolor="#e3f5e8", edgecolor="#3a9c5c", alpha=0.95, pad=4) if axis2
                 else dict(facecolor="white", edgecolor=GRID, alpha=0.85, pad=4))
    ax_y.text(0.98, 0.95, axis2_label, transform=ax_y.transAxes, fontsize=8.5,
              fontweight=("bold" if axis2 else "normal"),
              color=("#1f6b3a" if axis2 else MUTED), va="top", ha="right", bbox=axis2_box)

    title = "Closed-loop step response, dac_y → cx (+ cy, axis2 "
    title += "ON)" if axis2 else "OFF)"
    if notch_active:
        title += f"  (notch @ {notch_freq_hz:.1f}Hz)"
    if ctrl_interval_ms > 0:
        title += f"  (throttled {ctrl_rate_millihz/1000.0:.0f}Hz)"
    if smoothing:
        title += "  (boxcar smoothing)"
    fig.suptitle(title, fontsize=13, fontweight="bold")
    fig.tight_layout()
    png_path = out_path.rsplit(".", 1)[0] + ".png"
    fig.savefig(png_path, facecolor="white")
    plt.close(fig)
    print(f"Saved plot to {png_path}")


if __name__ == "__main__":
    main()
