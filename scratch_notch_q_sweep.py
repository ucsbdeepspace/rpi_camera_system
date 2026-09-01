"""
Scratch driver: does a MUCH tighter notch (Q well above the Q=3 tried on
2026-08-19/2026-08-27) recover the Ki ceiling that Q=3 lost, vs. no notch
at all?

Motivation (see rpi_camera_system CLAUDE.md, 2026-08-27 entry "Found and
fixed: the notch filter's sample-rate bug..."): once the notch's own
sample-rate bug was fixed so it genuinely centers on the real resonance
(38.5Hz at the time; this session's fresh open-loop Bode sweep -- see
results/fta_open_loop_bode_20260901_summary.png -- puts the peak closer to
40Hz, used here), the CORRECTLY-centered Q=3 notch had a WORSE Ki ceiling
(~350) than no notch at all (~500-550) -- never explained. A quick
continuous-domain loop-gain simulation (this session, scratchpad-only)
found the 10-20Hz passband cost of a much tighter notch is negligible
(Q=30 barely moves the passband gain at all vs Q=3), but that same
simulation's absolute stability predictions can't be trusted here -- it
predicted instability at a Kp=1.75/Ki=15 operating point that's already
confirmed clean on real hardware, so this script exists to get a real
answer directly instead of trusting more simulation.

Reuses scratch_notch_comparison.py's run_trial()/safe_idle() harness
unmodified (paced writes, explicit set_mode/gain readback verification,
CLEAN/RINGING/DIVERGED/GROWING classification, emergency-safe idle in
finally) -- see that file's own docstring for why each of those exists.

Usage: python3 scratch_notch_q_sweep.py
Sweeps Q in {8, 12, 20} x Ki in {200000, 400000, 600000, 800000} milli-
units, all at Kp=1.75 (1750 milli), notch centered at 40Hz (40000 milli),
throttled to 200Hz control rate (matches every other notch test this
project has run), axis2 on. Prints a summary table at the end and leaves
hardware idle regardless of outcome.
"""
import sys
import time

sys.path.insert(0, r"C:\Users\Bryan\Documents\GitHub\rpi_camera_system")
import scratch_notch_comparison as base

run_trial = base.run_trial
safe_idle = base.safe_idle
send_command = base.send_command
find_fta_port = base.find_fta_port
FTA_BAUD = base.FTA_BAUD

NOTCH_FREQ_MILLI = 40000  # 40Hz -- this session's fresh open-loop Bode peak
Q_LIST = [8, 12, 20]
KI_LIST = [200000, 400000, 600000, 800000]
KP_MILLI = 1750


def main():
    import serial

    port = find_fta_port()
    print(f"connecting {port}")
    ser = serial.Serial(port, FTA_BAUD, timeout=0.2)
    time.sleep(2)
    ser.reset_input_buffer()
    send_command(ser, "clear_estop")
    safe_idle(ser, "startup")

    results = []
    try:
        for q in Q_LIST:
            for ki in KI_LIST:
                label = f"Q{q}_Ki{ki//1000}"
                r = run_trial(
                    ser,
                    KP_MILLI,
                    ki,
                    NOTCH_FREQ_MILLI,
                    notch_q_milli=q * 1000,
                    label=label,
                )
                results.append((q, ki, r))
                time.sleep(0.5)  # let hardware settle between trials
    finally:
        safe_idle(ser, "final")
        ser.close()

    print("\n\n=== SUMMARY: Q x Ki notch sweep, notch @ 40Hz, Kp=1.75, throttled 200Hz ===")
    print(f"{'Q':>4} {'Ki':>8}  verdict")
    for q, ki, r in results:
        v = r["verdict"] if r else "TRIAL FAILED (discarded)"
        print(f"{q:>4} {ki/1000:>8.0f}  {v}")

    # last-clean-Ki-per-Q, for quick eyeballing of where the ceiling landed
    print("\n=== Ceiling per Q (last CLEAN Ki before RINGING/DIVERGED/GROWING) ===")
    for q in Q_LIST:
        last_clean = None
        for ki_v, r in [(ki, r) for (qq, ki, r) in results if qq == q]:
            if r and r["verdict"].startswith("CLEAN"):
                last_clean = ki_v
        print(f"Q={q}: last clean Ki = {last_clean}")


if __name__ == "__main__":
    main()
