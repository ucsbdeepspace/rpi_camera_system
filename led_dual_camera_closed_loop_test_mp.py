#!/usr/bin/env python3
"""
Closed-loop dual-camera LED timing test -- MULTIPROCESS variant.

Same measurement as led_dual_camera_closed_loop_test.py (command LED,
wait for BOTH cameras to independently confirm, flip, repeat) but each
camera runs in its own OS process instead of a thread in the same
process -- removing the GIL as a possible source of the erratic,
non-monotonic skew seen in the threaded frame-duration sweep. If the
skew pattern cleans up here, that confirms GIL contention was the
culprit; if it doesn't, the noise is coming from somewhere else
(camera/kernel/driver scheduling) and frame duration tuning alone won't
fix it.

Architecture:
  - The main process owns the GPIO/LED and all coordination. It never
    touches a camera directly -- Picamera2 + multiprocessing (fork) can
    be fragile if the parent process has already opened a camera before
    forking, so the parent stays camera-free entirely.
  - Each camera runs in its own child process (worker()), which opens
    its own Picamera2 instance, does its own ON/OFF calibration sample
    (coordinated with the main process via multiprocessing.Event, since
    only the main process can toggle the LED), then enters the same
    detect-and-flag loop as the threaded version -- just using
    multiprocessing.Value/Event instead of a plain shared variable.

CAUTION: same frame-duration safety rules as before -- this defaults to
6000us (the original baseline value) so results are directly comparable
to the first threaded dual-camera run. Don't push frame duration lower
in this script without the same fresh-process, one-value-at-a-time
caution used everywhere else.

Install:  pip install lgpio numpy
          (picamera2 is pre-installed on RPi OS Bookworm)
"""

import multiprocessing as mp
import sys
import time
import statistics
import lgpio
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────
LED_PIN          = 14
GPIOCHIP         = 4

ROI_CAM0         = (0, 0, 640, 400)
ROI_CAM1         = (0, 0, 640, 400)

RAW_SIZE         = (640, 400)
RAW_FORMAT       = "R8"
FORCE_FRAME_DURATION_US = int(sys.argv[1]) if len(sys.argv) > 1 else 6000
EXPOSURE_US      = 1500
ANALOGUE_GAIN    = 4.0

TEST_DURATION_S  = 5.0
CALIB_TIMEOUT_S  = 3.0
TIMEOUT_S        = 1.0
POLL_SLEEP_S     = 0.0001


def worker(index, roi, led_on_event, led_off_event,
           calib_on_ready, calib_off_ready, on_val, off_val,
           threshold_val, thresholds_ready, start_event,
           target, detected, detect_time, stop_event, frame_count):
    """Runs entirely in its own process -- opens its own camera, never
    touches the parent's state except through the shared multiprocessing
    primitives passed in."""
    from picamera2 import Picamera2  # imported here, not at module level, to
                                       # keep camera setup entirely inside the
                                       # child process post-fork

    cam = Picamera2(index)
    config = cam.create_video_configuration(
        main={"size": (64, 48), "format": "RGB888"},
        raw={"size": RAW_SIZE, "format": RAW_FORMAT},
        buffer_count=2,
    )
    cam.configure(config)
    cam.start()

    controls = {
        "FrameDurationLimits": (FORCE_FRAME_DURATION_US, FORCE_FRAME_DURATION_US),
        "AeEnable": False,
        "NoiseReductionMode": 0,
    }
    if EXPOSURE_US is not None:
        controls["ExposureTime"] = EXPOSURE_US
    if ANALOGUE_GAIN is not None:
        controls["AnalogueGain"] = ANALOGUE_GAIN
    unsupported = [k for k in controls if k not in cam.camera_controls]
    for k in unsupported:
        del controls[k]
    cam.set_controls(controls)
    time.sleep(1.0)

    rx, ry, rw, rh = roi

    def brightness():
        frame = cam.capture_array("raw")
        return float(np.mean(frame[ry:ry+rh, rx:rx+rw]))

    # ── Calibration: wait for main process to command LED ON, sample, then OFF ──
    led_on_event.wait()
    on_val.value = brightness()
    calib_on_ready.set()

    led_off_event.wait()
    off_val.value = brightness()
    calib_off_ready.set()

    thresholds_ready.wait()
    threshold = threshold_val.value

    # ── Main detect loop ─────────────────────────────────────────────────────
    start_event.wait()
    local_count = 0
    while not stop_event.is_set():
        frame = cam.capture_array("raw")
        t = time.monotonic()
        local_count += 1
        b = float(np.mean(frame[ry:ry+rh, rx:rx+rw]))
        state = b > threshold
        if state == bool(target.value) and not detected.is_set():
            detect_time.value = t
            detected.set()
    frame_count.value = local_count

    cam.stop()


def main():
    print(f"FORCE_FRAME_DURATION_US={FORCE_FRAME_DURATION_US}")
    gpio = lgpio.gpiochip_open(GPIOCHIP)
    lgpio.gpio_claim_output(gpio, LED_PIN, 0)

    def set_led(state: int):
        lgpio.gpio_write(gpio, LED_PIN, state)

    ctx = mp.get_context("fork")

    led_on_event = ctx.Event()
    led_off_event = ctx.Event()
    calib_on_ready = [ctx.Event(), ctx.Event()]
    calib_off_ready = [ctx.Event(), ctx.Event()]
    on_val = [ctx.Value("d", 0.0), ctx.Value("d", 0.0)]
    off_val = [ctx.Value("d", 0.0), ctx.Value("d", 0.0)]
    threshold_val = [ctx.Value("d", 0.0), ctx.Value("d", 0.0)]
    thresholds_ready = ctx.Event()
    start_event = ctx.Event()
    stop_event = ctx.Event()
    target = ctx.Value("i", 0)
    detected = [ctx.Event(), ctx.Event()]
    detect_time = [ctx.Value("d", 0.0), ctx.Value("d", 0.0)]
    frame_count = [ctx.Value("i", 0), ctx.Value("i", 0)]

    rois = [ROI_CAM0, ROI_CAM1]
    procs = []
    for i in range(2):
        p = ctx.Process(target=worker, args=(
            i, rois[i], led_on_event, led_off_event,
            calib_on_ready[i], calib_off_ready[i], on_val[i], off_val[i],
            threshold_val[i], thresholds_ready, start_event,
            target, detected[i], detect_time[i], stop_event, frame_count[i],
        ))
        p.start()
        procs.append(p)

    print("Started camera worker processes, waiting for camera init...")
    time.sleep(2.0)  # let both children finish opening/configuring their cameras

    # ── Calibration handshake ───────────────────────────────────────────────
    print("Per-camera calibration check:")
    set_led(1)
    time.sleep(0.2)
    led_on_event.set()
    for e in calib_on_ready:
        if not e.wait(timeout=CALIB_TIMEOUT_S):
            print("  WARNING: calibration ON sample timed out for a camera")

    set_led(0)
    time.sleep(0.2)
    led_off_event.set()
    for e in calib_off_ready:
        if not e.wait(timeout=CALIB_TIMEOUT_S):
            print("  WARNING: calibration OFF sample timed out for a camera")

    on0, off0 = on_val[0].value, off_val[0].value
    on1, off1 = on_val[1].value, off_val[1].value
    threshold0 = (on0 + off0) / 2.0
    threshold1 = (on1 + off1) / 2.0
    threshold_val[0].value = threshold0
    threshold_val[1].value = threshold1
    thresholds_ready.set()

    print(f"  cam0: ON={on0:.2f} OFF={off0:.2f} threshold={threshold0:.2f} "
          f"({'ok' if off0 < threshold0 < on0 else 'POOR SEPARATION'})")
    print(f"  cam1: ON={on1:.2f} OFF={off1:.2f} threshold={threshold1:.2f} "
          f"({'ok' if off1 < threshold1 < on1 else 'POOR SEPARATION'})\n")

    # ── Closed-loop scheduler ───────────────────────────────────────────────
    lat0_list, lat1_list, skew_list = [], [], []
    command_times = []
    timeouts = 0

    print(f"Running {TEST_DURATION_S}s closed-loop dual-process test\n")
    start_event.set()
    test_start = time.monotonic()

    try:
        while time.monotonic() - test_start < TEST_DURATION_S:
            target.value ^= 1
            detected[0].clear()
            detected[1].clear()
            set_led(target.value)
            command_time = time.monotonic()
            command_times.append(command_time)

            deadline = command_time + TIMEOUT_S
            while not (detected[0].is_set() and detected[1].is_set()):
                if time.monotonic() > deadline:
                    break
                time.sleep(POLL_SLEEP_S)

            if detected[0].is_set() and detected[1].is_set():
                t0 = detect_time[0].value
                t1 = detect_time[1].value
                lat0_list.append((t0 - command_time) * 1000.0)
                lat1_list.append((t1 - command_time) * 1000.0)
                skew_list.append((t0 - t1) * 1000.0)
            else:
                timeouts += 1

    except KeyboardInterrupt:
        pass

    finally:
        test_end = time.monotonic()
        stop_event.set()
        for p in procs:
            p.join(timeout=5.0)
        set_led(0)

        print("── Summary ──────────────────────────────")
        n = len(skew_list)
        print(f"  Transitions with both cameras confirming: {n}  (timeouts: {timeouts})")

        elapsed = test_end - test_start
        for idx in (0, 1):
            fc = frame_count[idx].value
            fps = fc / elapsed if elapsed else float("nan")
            print(f"  cam{idx} achieved during closed loop: {fc} frames in {elapsed:.2f}s  ->  {fps:.2f} fps")

        if n:
            for name, lats in [("cam0", lat0_list), ("cam1", lat1_list)]:
                mean_l = statistics.mean(lats)
                std_l = statistics.pstdev(lats) if n > 1 else 0.0
                print(f"  {name} latency: mean={mean_l:.3f} ms  std={std_l:.3f} ms  "
                      f"min={min(lats):.3f} ms  max={max(lats):.3f} ms")

            mean_skew = statistics.mean(skew_list)
            std_skew = statistics.pstdev(skew_list) if n > 1 else 0.0
            print(f"\n  Inter-camera skew (cam0 - cam1): mean={mean_skew:+.3f} ms  "
                  f"std={std_skew:.3f} ms  min={min(skew_list):+.3f} ms  max={max(skew_list):+.3f} ms")
            print(f"  |skew| max: {max(abs(s) for s in skew_list):.3f} ms")

            if len(command_times) > 1:
                periods = [command_times[i+1] - command_times[i] for i in range(len(command_times)-1)]
                mean_p = statistics.mean(periods)
                freq = 1.0 / mean_p if mean_p else float("nan")
                print(f"\n  Effective closed-loop toggle frequency (both-confirmed): {freq:.2f} Hz")
        print("─────────────────────────────────────────")

        lgpio.gpiochip_close(gpio)


if __name__ == "__main__":
    main()
