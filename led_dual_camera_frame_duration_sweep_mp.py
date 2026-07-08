#!/usr/bin/env python3
"""
Dual-camera closed-loop frame-duration sweep -- MULTIPROCESS variant.

Same measurement as led_dual_camera_frame_duration_sweep.py, but built on
top of led_dual_camera_closed_loop_test_mp.py's two-process architecture
instead of threads, to see whether the worst-case latency tail actually
shrinks as frame duration drops (the question left open after comparing
the threaded vs. multiprocess single-point results: multiprocessing
fixed the skew bias/std but not the ~13-14ms tail, suggesting frame
duration itself -- not threading -- is the tail's real driver).

Rather than restarting two fresh processes per duration step (slow, and
each camera open/close cycle adds its own risk), the two worker
processes stay alive for the whole sweep and step through a sequence of
"rounds" -- one per frame-duration value -- coordinated via
multiprocessing Events/Values:

  1. Main publishes the next frame duration; each worker applies it to
     its own camera.
  2. Same ON/OFF calibration handshake as the single-point script,
     repeated per round (brightness can shift slightly with duration).
  3. Closed-loop measurement for SWEEP_TEST_DURATION_S, same "wait for
     both cameras" logic as before.
  4. Workers signal round completion, main resets shared state, moves
     to the next duration.

SAFETY NOTE: same conservative floor as the threaded sweep script
(4000us) -- the dual-camera combined hardware floor is still unknown,
and low-duration failures can hang the process without a catchable
exception. Don't lower this range without manual, fresh-process,
one-value-at-a-time testing first.

Install:  pip install lgpio numpy
          (picamera2 is pre-installed on RPi OS Bookworm)
"""

import csv
import multiprocessing as mp
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
EXPOSURE_US      = 1500
ANALOGUE_GAIN    = 4.0

# Same conservative range as the threaded sweep -- see safety note above.
FRAME_DURATION_SWEEP_US = [6000, 5500, 5000, 4500, 4000]

SWEEP_TEST_DURATION_S = 3.0
CALIB_TIMEOUT_S  = 3.0
TIMEOUT_S        = 1.0
SETTLE_S         = 1.5   # workers now do a full stop/reconfigure/start per round
                          # (their own internal 1.0s settle already included) --
                          # this extra margin is on top of that
POLL_SLEEP_S     = 0.0001
OUTPUT_CSV       = "dual_camera_mp_frame_duration_sweep.csv"


def worker(index, roi, stop_event, duration_ready, duration_val,
           led_on_event, led_off_event, calib_on_ready, calib_off_ready,
           on_val, off_val, threshold_val, thresholds_ready,
           round_start_event, round_stop_event, round_done,
           target, detected, detect_time):
    from picamera2 import Picamera2

    cam = Picamera2(index)
    config = cam.create_video_configuration(
        main={"size": (64, 48), "format": "RGB888"},
        raw={"size": RAW_SIZE, "format": RAW_FORMAT},
        buffer_count=2,
    )
    cam.configure(config)
    cam.start()
    time.sleep(1.0)

    rx, ry, rw, rh = roi

    def brightness():
        frame = cam.capture_array("raw")
        return float(np.mean(frame[ry:ry+rh, rx:rx+rw]))

    while True:
        duration_ready.wait()
        duration_ready.clear()
        if stop_event.is_set():
            break

        d = duration_val.value

        # Full stop/reconfigure/start per round instead of a live
        # set_controls() change on an already-running stream -- this
        # mirrors exactly what worked for the very first configuration
        # (transition away from defaults), since changing
        # FrameDurationLimits on an already-fixed-exposure stream turned
        # out not to reliably re-settle within a short sleep.
        cam.stop()
        config = cam.create_video_configuration(
            main={"size": (64, 48), "format": "RGB888"},
            raw={"size": RAW_SIZE, "format": RAW_FORMAT},
            buffer_count=2,
        )
        cam.configure(config)
        cam.start()

        controls = {
            "FrameDurationLimits": (d, d),
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
        time.sleep(1.0)  # same settle time given to the very first configuration

        # ── Calibration for this round ──
        led_on_event.wait()
        on_val.value = brightness()
        calib_on_ready.set()

        led_off_event.wait()
        off_val.value = brightness()
        calib_off_ready.set()

        thresholds_ready.wait()
        threshold = threshold_val.value

        # ── Detection loop for this round ──
        round_start_event.wait()
        while not round_stop_event.is_set():
            frame = cam.capture_array("raw")
            t = time.monotonic()
            b = float(np.mean(frame[ry:ry+rh, rx:rx+rw]))
            state = b > threshold
            if state == bool(target.value) and not detected.is_set():
                detect_time.value = t
                detected.set()

        round_done.set()

    cam.stop()


def main():
    gpio = lgpio.gpiochip_open(GPIOCHIP)
    lgpio.gpio_claim_output(gpio, LED_PIN, 0)

    def set_led(state: int):
        lgpio.gpio_write(gpio, LED_PIN, state)

    ctx = mp.get_context("fork")

    stop_event = ctx.Event()
    duration_ready = ctx.Event()
    duration_val = ctx.Value("i", 0)
    led_on_event = ctx.Event()
    led_off_event = ctx.Event()
    calib_on_ready = [ctx.Event(), ctx.Event()]
    calib_off_ready = [ctx.Event(), ctx.Event()]
    on_val = [ctx.Value("d", 0.0), ctx.Value("d", 0.0)]
    off_val = [ctx.Value("d", 0.0), ctx.Value("d", 0.0)]
    threshold_val = [ctx.Value("d", 0.0), ctx.Value("d", 0.0)]
    thresholds_ready = ctx.Event()
    round_start_event = ctx.Event()
    round_stop_event = ctx.Event()
    round_done = [ctx.Event(), ctx.Event()]
    target = ctx.Value("i", 0)
    detected = [ctx.Event(), ctx.Event()]
    detect_time = [ctx.Value("d", 0.0), ctx.Value("d", 0.0)]

    rois = [ROI_CAM0, ROI_CAM1]
    procs = []
    for i in range(2):
        p = ctx.Process(target=worker, args=(
            i, rois[i], stop_event, duration_ready, duration_val,
            led_on_event, led_off_event, calib_on_ready[i], calib_off_ready[i],
            on_val[i], off_val[i], threshold_val[i], thresholds_ready,
            round_start_event, round_stop_event, round_done[i],
            target, detected[i], detect_time[i],
        ))
        p.start()
        procs.append(p)

    print("Started camera worker processes, waiting for camera init...")
    time.sleep(2.5)

    rows = []
    f = open(OUTPUT_CSV, "w", newline="")
    writer = None

    try:
        for duration_us in FRAME_DURATION_SWEEP_US:
            print(f"-- FrameDurationLimits={duration_us}us --")

            duration_val.value = duration_us
            duration_ready.set()
            time.sleep(SETTLE_S)

            led_on_event.clear()
            led_off_event.clear()
            calib_on_ready[0].clear()
            calib_on_ready[1].clear()
            calib_off_ready[0].clear()
            calib_off_ready[1].clear()
            thresholds_ready.clear()

            set_led(1)
            time.sleep(0.2)
            led_on_event.set()
            calib_on_ok = all(e.wait(timeout=CALIB_TIMEOUT_S) for e in calib_on_ready)

            set_led(0)
            time.sleep(0.2)
            led_off_event.set()
            calib_off_ok = all(e.wait(timeout=CALIB_TIMEOUT_S) for e in calib_off_ready)

            if not (calib_on_ok and calib_off_ok):
                print(f"   WARNING: calibration handshake timed out at {duration_us}us "
                      f"(camera didn't confirm in time) -- skipping this duration\n")
                round_stop_event.set()  # release workers from any lingering wait state
                for e in round_done:
                    e.wait(timeout=CALIB_TIMEOUT_S)
                row = {"frame_duration_us": duration_us, "threshold_valid": False,
                       "transitions": 0, "timeouts": 0, "note": "calibration handshake timeout"}
                rows.append(row)
                if writer is None:
                    writer = csv.DictWriter(f, fieldnames=list(row.keys()))
                    writer.writeheader()
                writer.writerow(row)
                f.flush()
                continue

            on0, off0 = on_val[0].value, off_val[0].value
            on1, off1 = on_val[1].value, off_val[1].value
            threshold0 = (on0 + off0) / 2.0
            threshold1 = (on1 + off1) / 2.0
            threshold_val[0].value = threshold0
            threshold_val[1].value = threshold1
            valid0 = off0 < threshold0 < on0
            valid1 = off1 < threshold1 < on1
            thresholds_ready.set()

            round_start_event.clear()
            round_stop_event.clear()
            round_done[0].clear()
            round_done[1].clear()
            target.value = 0
            detected[0].clear()
            detected[1].clear()

            lat0_list, lat1_list, skew_list = [], [], []
            command_times = []
            timeouts = 0

            round_start_event.set()
            test_start = time.monotonic()
            while time.monotonic() - test_start < SWEEP_TEST_DURATION_S:
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

            round_stop_event.set()
            set_led(0)
            for e in round_done:
                e.wait(timeout=CALIB_TIMEOUT_S)

            n = len(skew_list)
            row = {
                "frame_duration_us": duration_us,
                "cam0_on": round(on0, 3), "cam0_off": round(off0, 3), "cam0_threshold": round(threshold0, 3),
                "cam1_on": round(on1, 3), "cam1_off": round(off1, 3), "cam1_threshold": round(threshold1, 3),
                "threshold_valid": valid0 and valid1,
                "transitions": n,
                "timeouts": timeouts,
                "cam0_lat_mean_ms": round(statistics.mean(lat0_list), 3) if n else "",
                "cam0_lat_std_ms": round(statistics.pstdev(lat0_list), 3) if n > 1 else "",
                "cam0_lat_max_ms": round(max(lat0_list), 3) if n else "",
                "cam1_lat_mean_ms": round(statistics.mean(lat1_list), 3) if n else "",
                "cam1_lat_std_ms": round(statistics.pstdev(lat1_list), 3) if n > 1 else "",
                "cam1_lat_max_ms": round(max(lat1_list), 3) if n else "",
                "skew_mean_ms": round(statistics.mean(skew_list), 3) if n else "",
                "skew_std_ms": round(statistics.pstdev(skew_list), 3) if n > 1 else "",
                "skew_abs_max_ms": round(max(abs(s) for s in skew_list), 3) if n else "",
                "freq_hz": "",
            }
            if len(command_times) > 1:
                periods = [command_times[i+1] - command_times[i] for i in range(len(command_times)-1)]
                mean_p = statistics.mean(periods)
                row["freq_hz"] = round(1.0 / mean_p, 3) if mean_p else ""
            rows.append(row)

            if writer is None:
                writer = csv.DictWriter(f, fieldnames=list(row.keys()))
                writer.writeheader()
            writer.writerow(row)
            f.flush()

            ok = "ok" if row["threshold_valid"] else "POOR SEPARATION"
            print(f"   cam0 ON={row['cam0_on']} OFF={row['cam0_off']}  "
                  f"cam1 ON={row['cam1_on']} OFF={row['cam1_off']}  ({ok})")
            print(f"   transitions={n} timeouts={timeouts}  "
                  f"cam0_lat mean={row['cam0_lat_mean_ms']}ms max={row['cam0_lat_max_ms']}ms  "
                  f"cam1_lat mean={row['cam1_lat_mean_ms']}ms max={row['cam1_lat_max_ms']}ms  "
                  f"skew_mean={row['skew_mean_ms']}ms |skew|max={row['skew_abs_max_ms']}ms  "
                  f"freq={row['freq_hz']}Hz\n")

    except KeyboardInterrupt:
        print("Sweep interrupted -- partial results already flushed to CSV.\n")

    finally:
        f.close()
        print(f"Saved {len(rows)} rows to {OUTPUT_CSV}")

        stop_event.set()
        duration_ready.set()  # wake children so they see stop_event and exit
        for p in procs:
            p.join(timeout=5.0)
        set_led(0)
        lgpio.gpiochip_close(gpio)


if __name__ == "__main__":
    main()
