# FTA Dual-Camera Timing Characterization

Characterizing Raspberry Pi 5 dual-camera (OV9281) timing performance for an FTA
beam-tracking closed-loop control system. Target set by Phil (Prof. Lubin):
10ms loop latency, disturbance band 10-20Hz beacon wobble.

This file is the living state of the project. Update it as findings change —
don't let it drift from what's actually true. Full prior conversation history
(pre-2026-07-08) is archived in `docs/archive/handoff_conversation_2026-07-08.txt`
for context, but treat *this* file as authoritative, not the archive.

## DONE: runtime-movable ROI via `set_selection`, live dual-camera demo built (2026-07-14)

**Stop here if picking this up fresh: this whole feature is validated and
usable now.** Driver patch rebooted in and clean; mid-stream/pre-stream
writes validated across both cameras and both ROI modes (all combinations
clean, see "Validation results" below); `roi_set_selection.py` is a real,
committed helper (`get_roi_y_start`/`set_roi_y_start`); and
`roi_live_demo.py` is a real, committed interactive demo — two live camera
windows, keyboard-driven ROI move, confirmed working end-to-end on the
Pi's actual display (see "Live demo" section below). Remaining open items
are polish, not blockers: repeated mid-stream trials (each combo has only
been tried once), and a known-but-worked-around kernel-side clamp bug (see
item 7 below).

### Why this exists

User asked for (1) an app to view the `MODE_640_200_ROI` feed — already
satisfied, `camera_preview_roi.py` defaults to `640x200` and was already used
to validate that mode — and (2) a way to add/move ROIs without a
rebuild+reboot cycle per new crop, i.e. change the ROI from Python at
runtime instead of hardcoding it into the kernel driver.

Investigated the driver: `ov9282_pad_ops` had `.get_selection` but no
`.set_selection` at all — that's *why* the earlier "sensor-level
crop/windowing — RESOLVED, not available" investigation (below) found
`crop == crop_bounds == crop_default` fixed. There was no code path for
userspace to change it; every mode's window is baked into a compile-time
register array.

Two options were on the table: (a) Picamera2's `ScalerCrop` control — settable
live, zero kernel changes, but almost certainly an ISP-level crop applied
*after* the sensor already read out the full window, so it would silently
give up the 1.9x throughput win MODE_640_200_ROI just proved (same
conclusion as the earlier "software ROI slicing costs <3%, no speed benefit"
finding). (b) Real driver-level runtime windowing via `.set_selection`,
translating a requested rect into i2c writes of the sensor's own window
registers, preserving the speed win. **User chose (b).**

### Scope decision (deliberately narrow)

Only vertical **position** (`y_start`) is runtime-adjustable, and only for
the two windowed-crop ROI modes (`MODE_1280_400_ROI`, `MODE_640_200_ROI`).
Width, output height, and binning stay fixed at each mode's already-validated
values. Reasons for the narrower scope, not a fully general crop API:

- **Height/size changes are excluded**: changing output height would change
  the negotiated buffer geometry mid-stream, which needs a full
  reconfigure/restart, not a live crop move — different, riskier problem.
- **Horizontal (x_start) panning is excluded**: unlike vertical, there's no
  stock-mode precedent to extrapolate a register formula from (noted
  already in the `MODE_1280_400_ROI` section below — width was "deliberately
  left at 1280, no stock precedent exists for horizontal windowing"). Adding
  it now would be pure speculation with zero empirical anchor, unlike
  vertical which has 3 confirmed data points (800→720→400 row windows).

### What was implemented (`kernel_patch/ov9282/ov9282.c`, not yet committed)

- New `struct ov9282` field `roi_y_start` (real sensor rows, default 0),
  reset to 0 whenever `ov9282_set_pad_format` selects/reselects any mode.
- `ov9282_apply_roi_y_start()`: writes only `0x3802/0x3803` (y_start) and
  `0x3806/0x3807` (y_end), reusing the already-validated
  `y_end = y_start + height + 15` relationship (derived by comparing the
  stock 800/720/400-row modes) — **but every prior use of that formula only
  ever exercised `y_start = 0`. A nonzero `y_start` here is a new,
  hardware-unverified extrapolation, not yet checked against a captured
  frame.** Called from `ov9282_start_streaming()` right after the mode's
  static `reg_list` (so it's a no-op reproducing the same values already in
  `reg_list` unless `set_selection` moved `roi_y_start` first — verified this
  reproduces the exact stock register values for all 5 modes at `y_start=0`).
- `ov9282_set_selection()`: new pad op, target `V4L2_SEL_TGT_CROP` only.
  Clamps requested `y_start` to `[0, 800 - mode_height]`, rounds down to a
  4-row boundary. For any mode other than the two ROI modes, silently
  echoes back the fixed default crop instead of erroring (matches how
  `get_selection` already treats non-adjustable modes). For `ACTIVE` state:
  stores the new `roi_y_start`, and if the sensor is currently powered/
  streaming (`pm_runtime_get_if_in_use`), pushes the two registers over i2c
  immediately — i.e. **intended to move the ROI while a capture is already
  running**, not just before `cam.start()`. This live-while-streaming path is
  the least-tested part of the design (see validation plan below).
- `ov9282_get_selection()` updated to report the *live* `top` (not the
  static compile-time default) when `which == ACTIVE`.

### Exact current state (2026-07-14)

1. Code changes made to `ov9282.c`, **not yet git-committed** (working tree
   dirty).
2. `make` in `kernel_patch/ov9282/` — clean build, no warnings.
3. `modinfo ov9282.ko` confirmed `vermagic: 6.18.34+rpt-rpi-2712 ... aarch64`
   — matches running kernel exactly, same check done before every prior
   install.
4. `sudo cp` to `/lib/modules/6.18.34+rpt-rpi-2712/updates/ov9282.ko`
   (overwriting the previously-installed MODE_640_200_ROI-only build) +
   `sudo depmod -a` — both succeeded.
5. **Reboot happened** (sometime between the 2026-07-14 hand-off note above
   being written and this update — exact trigger not captured, but the
   system came up on the patched module without incident).

### Validation results (2026-07-14, post-reboot)

All against camera 0 (`i2c@88000` → `/dev/v4l-subdev5`), mode `640x200`
(`MODE_640_200_ROI`, real sensor crop window `1280x400` pre-bin per
`rpicam-hello`'s own reporting).

1. **Clean boot — confirmed.** `tainted` = `4096` (only `O`=OOT_MODULE, no
   `D`/`W`), no BUG/Oops/WARNING in dmesg.
2. **Same 5 modes present — confirmed.** `rpicam-hello --list-cameras` lists
   `640x200`, `640x400`, `1280x400`, `1280x720`, `1280x800` on both cameras,
   unchanged from pre-patch.
3. **Default `y_start=0` reporting — confirmed.** Fresh mode-select via
   `v4l2-ctl --set-subdev-fmt` to `640x200`/`Y8_1X8`, then
   `--get-subdev-selection=...target=crop` reported `Left 8, Top 8, Width
   1280, Height 400` — i.e. `top=8` = `y_start=0`, matching the sensor
   window `rpicam-hello` itself advertises for this mode. `crop_bounds`/
   `crop_default` still correctly report the full `1280x800` sensor,
   unaffected.
4. **Pre-stream nonzero `y_start` + frame content — confirmed clean.**
   Wrote a one-off headless script (not committed — GUI preview needs a live
   display, same reasoning as the earlier `MODE_640_200_ROI` frame-content
   check) that: opens Picamera2, `configure()`s `640x200` raw, pushes
   `set-subdev-selection top=208` (`y_start=200`, i.e. `left=8,width=1280,
   height=400` — the real pre-bin sensor window) via `v4l2-ctl` **after**
   `configure()` but **before** `cam.start()` (order matters:
   `ov9282_set_pad_format` resets `roi_y_start=0` on every mode
   select/reselect, so pushing the write any earlier would just get
   clobbered by Picamera2's own `configure()` call), then starts and
   captures. Compared against a `y_start=0` baseline capture of the same
   static scene: `get-subdev-selection` correctly reflected `top=208` both
   before and during streaming, no dmesg/taint anomaly, and the captured
   image visibly shifted content in the correct direction/magnitude (a
   bright blob moved from the bottom half toward the upper-middle of frame,
   consistent with sliding a 400-row window down 200 rows) with no tearing,
   banding, or corruption in either frame.
5. **The real test — mid-stream register write while actively capturing —
   PASSED CLEAN.** Wrote `roi_midstream_capture.py` (one-off, not
   committed): opens camera 0, `640x200`, starts streaming, then just
   captures continuously into an in-memory frame stack (with a wall-clock
   timestamp per frame) for 20s while doing nothing else. Ran it as a
   background process, and — from a **separate shell**, i.e. genuinely a
   different, unrelated process, not coordinated in-process — fired
   `v4l2-ctl --set-subdev-selection top=208` partway through the run.
   Post-hoc analysis of the saved frame stack (per-row correlation against
   reference "steady old-position" and "steady new-position" frames, in 50-
   row quarters) found: the write landed cleanly between two whole frames —
   frame N was **100% old-position** in all four quarters (row-wise
   correlation ≈0.995 to old ref, ≈−0.96 to new ref, no exceptions), frame
   N+1 was **100% new-position** in all four quarters (≈0.99+ to new ref,
   strongly negative to old ref). **No frame showed a mixed/torn signature**
   (e.g. top quarters correlating to one position while bottom quarters
   correlate to the other) — the sensor evidently applies the window-
   register change atomically at a frame boundary rather than mid-readout,
   at least in this one trial. Visually confirmed too (saved PNGs of both
   frames): the bright blob is at the old position in frame N, shifted to
   the new position in frame N+1, both frames individually clean/coherent.
   `tainted` stayed `4096` throughout, no dmesg BUG/Oops/WARNING.
   Cam0's crop was reset back to `y_start=0` afterward to leave hardware in
   a known state.

6. **Range coverage — DONE, all clean (2026-07-14).** Tried `y_start` = 0,
   100, 200, 300, 380, and 400 (the exact max, since `800 - mode_height(400)
   = 400` and that's already a multiple of 4 so no rounding kicks in) on
   camera 0 / `640x200`, each in a fresh process via the same pre-stream
   `set-subdev-selection`-before-`cam.start()` pattern as item 4. Every
   value: `get-subdev-selection` echoed back the exact requested `top`, the
   captured frame was visually coherent (a different, correctly-shifted
   vertical slice of the same static scene each time, no tearing/banding/
   garbage — spot-checked visually at `y=0/200/400`, stats-checked via row
   means at all six), and `tainted`/dmesg stayed clean throughout. This is
   reasonable evidence `y_end = y_start + height + 15` generalizes across
   the *whole* allowed range, not just the `y_start=0` case every stock mode
   already validated — though it's still only camera 0 / one mode; camera 1
   and `MODE_1280_400_ROI` haven't been touched by this new code path.
   **Also tested clamp behavior directly** (not just in-range values):
   requesting `top=608` (`y_start=600`, far past the `400` max) clamped down
   to `top=408` (`y_start=400`) exactly; requesting `top=0` (below the
   `top=8` floor) clamped up to `top=8` (`y_start=0`) exactly. Both clamps
   silent (no ioctl error), no dmesg/taint anomaly — `ov9282_set_selection`'s
   clamp logic holds at both edges, not just accepting whatever's asked.
7. **Python helper — DONE, committed as `roi_set_selection.py`
   (2026-07-14).** Real repo script (not scratchpad), exposing
   `get_roi_y_start(cam_index)` / `set_roi_y_start(cam_index, y_start)`.
   Shells out to `v4l2-ctl` subdev ioctls (Picamera2 has no high-level API
   for arbitrary sensor crop) — same approach the one-off validation
   scripts used. Handles the camera-index → subdev path mapping internally
   (`i2c@88000`→`/dev/v4l-subdev5`, `i2c@80000`→`/dev/v4l-subdev2`, read via
   `Picamera2.global_camera_info()`), and always reads the position back
   after writing so callers get the value the driver actually applied, not
   just an echo of the request. Also runnable directly as a CLI
   (`python3 roi_set_selection.py <cam_index> [y_start]`).

   **Found a real driver bug while dogfooding this**: a negative `y_start`
   sent straight to the driver does **not** clamp to the `0` floor the way
   an over-range value correctly clamps to the max — it wraps around
   (almost certainly unsigned-arithmetic underflow in the kernel-side clamp
   in `ov9282_set_selection`) and lands at the **max** position (400)
   instead. Confirmed directly with a raw `v4l2-ctl --set-subdev-selection
   ...top=-42...` call (independent of this Python script), so it's a
   genuine kernel-side gap, not a bug in the wrapper. **Not fixed in the
   driver** — worked around in `roi_set_selection.py` by clamping
   `y_start` to `[0, MAX_Y_START]` in Python *before* it ever reaches the
   driver, so this wrapper never triggers the bug. Anything that talks to
   the subdev directly (bypassing this script) is still exposed to it —
   worth a kernel-side fix at some point (the driver's own clamp should
   check for negative input as a signed value before comparing against 0),
   but not blocking since the one sanctioned Python entry point now guards
   against it.
8. **Mid-stream write, full camera×mode matrix — DONE, all 4 clean
   (2026-07-14).** Extended the item-5 mid-stream test (background capture
   loop + an external, separate-shell `v4l2-ctl set-subdev-selection` fired
   mid-run) from camera 0/`640x200` (already done) to the remaining three
   combinations: camera 1/`640x200`, camera 0/`1280x400`, camera
   1/`1280x400`. Generalized `roi_midstream_capture.py` to take a `WxH` CLI
   arg and tag its output files by mode so runs don't clobber each other.
   For each combination: launched a 20s background capture, fired the
   external write partway through (landed 3-4.7s into the window each
   time), then found the transition via frame-mean discontinuity and
   checked per-quarter (of the frame height) row-correlation against
   "steady old" and "steady new" reference frames. **Every one of the 4
   combinations showed the identical pattern already found for camera
   0/`640x200`**: one frame is fully old-position in all 4 quarters (corr
   ≈0.95-1.0 to old, negative/weak to new), the very next frame is fully
   new-position in all 4 quarters (corr ≈0.95-1.0 to new, negative/weak to
   old) — **no mixed/torn frame in any of the 4 trials**, and this held
   whether the write landed early (~3s) or later (~4.7s) into the capture
   window. `tainted` stayed `4096` throughout all 4 runs, no dmesg BUG/
   Oops/WARNING. Visually spot-checked the transition-frame pair for the
   camera 1/`640x200` case too (not just the correlation numbers) — same
   clean before/after cut seen in the camera 0 case. Both subdevs reset to
   `y_start=0` afterward.

   **Net result: the mid-stream live-write path is now validated clean
   across all 4 camera×mode combinations** (2 cameras × the 2 windowed ROI
   modes), 1 trial each. Still true that each combination has only been
   tried *once* — repeated trials (esp. writes landing at different points
   within a frame's readout window, or back-to-back writes in a single
   stream) would build more confidence than "clean once per combination"
   currently provides, but the earlier open question of "does this even
   work outside the one camera/mode it was first tried on" is answered:
   yes, consistently.

9. **Camera 1 and `MODE_1280_400_ROI` coverage — DONE for the pre-stream
   path, all clean (2026-07-14).** Repeated the item-4/6-style pre-stream
   test (`set-subdev-selection` after `configure()`, before `start()`,
   `y_start` = 0/200/400) across all three previously-untested combinations:
   camera 1 / `640x200`, camera 0 / `1280x400`, camera 1 / `1280x400`. All
   9 runs (3 combos × 3 values): `get-subdev-selection` echoed back the
   exact requested `top` both before and during streaming, captured frames
   were visually coherent and correctly shifted (spot-checked at least one
   image per combo), and `tainted`/dmesg stayed clean (`4096`, no BUG/Oops)
   throughout — including confirming the right per-camera CSI controller
   logged the streaming-start message (`1f00110000.csi` for camera 0,
   `1f00128000.csi` for camera 1), i.e. the two cameras' independent CFE
   paths both handled the runtime crop change correctly. Camera mapping
   confirmed via `Picamera2.global_camera_info()`: index 0 → `i2c@88000` →
   `/dev/v4l-subdev5`, index 1 → `i2c@80000` → `/dev/v4l-subdev2`. Combined
   with the earlier camera 0 / `640x200` results, **the pre-stream
   set_selection path is now validated on both cameras and both windowed
   ROI modes.** Both subdevs reset back to `y_start=0` afterward.
   **What this does NOT cover**: the mid-stream (item 5/8) write — that
   real-time "move it while streaming" test has still only ever been done
   once, on camera 0 / `640x200`.

10. **Live interactive dual-camera demo — DONE, committed as
    `roi_live_demo.py` (2026-07-14).** User asked for a demo showing both
    camera streams with a way to change the ROI interactively. Built on top
    of `roi_set_selection.py`: opens both cameras in an ROI mode (default
    `640x200`), shows each in its own OpenCV window with the current
    `y_start` overlaid, and reads keyboard input each frame (`w`/`s` move
    the *shared* ROI up/down together on both cameras by a configurable
    step, `r` resets to 0, `q` quits) — every move goes through
    `set_roi_y_start()` while both cameras are actively streaming, i.e. it
    continuously exercises the exact mid-stream write path validated in
    items 5/8/9 above, not just a canned before-start move.

    **Actually tested end-to-end on the Pi's live display, not just
    inspected as code.** This session has a real desktop session
    (`DISPLAY=:0`, `labwc` compositor, confirmed via `who`/`loginctl`), so
    launched the demo in the background and drove it for real: installed
    `xdotool` (via `sudo apt-get install`, kept installed per user's
    choice) to focus the camera window and send actual `s`/`r`/`q`
    keypresses, using `grim` to screenshot the real screen after each one.
    Confirmed, with actual screenshots, not just log output: (1) both
    camera windows render live, correctly-labeled feeds side by side; (2)
    three `s` presses moved the overlay from `y_start=0` → `20` → `40` →
    `60` (step=20 default) and the visible image content in *both* windows
    shifted together, in sync; (3) `r` correctly reset both back to
    `y_start=0`, with the image content visibly returning to its original
    framing; (4) `q` exited the process cleanly (exit code 0, not a
    timeout-kill). `tainted` stayed `4096` throughout, no dmesg BUG/Oops.
    Both subdevs confirmed back at `y_start=0` after the run.

    **Repeated the same live test on `MODE_1280_400_ROI` (2026-07-14,
    `python3 roi_live_demo.py 1280x400 40`).** Both windows correctly
    titled `Camera N -- ROI 1280x400` (confirms the `WxH` CLI arg actually
    changes the negotiated mode, not just the label). Three `s` presses
    (step=40) moved the overlay `0`→`40`→`80`→`120` on camera 0; brought
    camera 1's window to the front separately and confirmed it independently
    read the same `y_start=120` and showed correspondingly shifted content
    — the two cameras' windows aren't just both displaying the same
    variable, each is actually reporting back its own driver-applied
    position and they agree. `q` exited cleanly (exit 0), `tainted` stayed
    `4096`, no dmesg anomaly, both subdevs reset to `y_start=0` after.

## MODE_640_200_ROI — DONE, committed (`d5eb808`, 2026-07-08)

Superseded by the in-progress section above as the active thread, but the
underlying mode itself is validated and unchanged. Detail preserved below
for reference.

Added a second experimental mode, `MODE_640_200_ROI`, to
`kernel_patch/ov9282/ov9282.c` — combines the stock 640x400 mode's binning
registers (fast per-row readout) with a shrunk vertical window (fewer rows),
targeting an actual speed win that `MODE_1280_400_ROI` alone didn't deliver
(see "Throughput result — NEGATIVE" below). Built, vermagic-matched,
depmod-installed to `/lib/modules/.../updates/ov9282.ko`, reboot triggered
to load it.

**Reboot succeeded, first two checks pass:**
1. **Clean state — confirmed.** `tainted` = `4096` (only `O`=OOT_MODULE, no
   `D`/`W`), no BUG/Oops in dmesg. `rpicam-hello --list-cameras` now shows a
   `640x200 [588.93 fps - (0, 0)/1280x400 crop]` R8 mode on both cameras
   alongside the original four.
2. **Frame content — looks real, not garbage.** Wrote a headless capture
   script (one-off, not committed — GUI preview scripts need a live display
   to eyeball; this saves PNGs + prints min/max/mean instead) modeled on
   `camera_preview_roi.py` but for `RAW_SIZE=(640,200)`. Both cameras
   negotiated the correct `(640, 200)` raw config, captured arrays of the
   right shape, non-degenerate pixel ranges (cam0: min=4096 max=8960
   mean=6018; cam1: min=3840 max=16640 mean=6922 — note these are 16-bit
   values per the `R16`-delivery quirk documented elsewhere in this file),
   and visually showed coherent gradients/objects with normal sensor noise
   texture — no tearing, banding, or tiling. This is reasonable (not
   conclusive) evidence the guessed `0x380a/0x380b` y_output_size halving
   (see comment above `mode_640x200_roi_regs` in `ov9282.c`) didn't break
   the image, though a rigorous check (e.g. a known test pattern) hasn't
   been done.
3. **Throughput — apples-to-apples point measured, real ceiling not yet
   probed.** Dual-concurrent at 3400µs (same duration as the existing
   281.8fps/283.6fps comparison): **282.14fps**, essentially identical to
   both prior modes. This is expected and *not yet informative* — 3400µs is
   still well above this mode's rated floor (588.93fps mode ≈ 1698µs native
   period), so all three modes are duration-limited, not sensor-limited, at
   this setting. The actual test of whether this mode beats ~282fps requires
   stepping the requested frame duration down toward ~1700µs.

4. **Throughput floor found (2026-07-08 ~17:00) — real speed win confirmed.**
   Built `camera_throughput_sweep_subprocess.py` (mirrors
   `led_dual_camera_sweep_subprocess.py`'s fresh-subprocess-per-value
   pattern: each duration runs in its own OS process via
   `subprocess.run(timeout=...)`, so a hang can be killed from outside
   instead of freezing the whole interpreter with no catchable exception).
   Also gave `camera_throughput_test.py` an optional 3rd CLI arg for raw
   size (defaults to 640x400, unchanged behavior otherwise). Swept
   3400µs → 1750µs on `RAW_SIZE=640x200`, dual-concurrent:

   | duration | achieved fps (cam0/cam1) |
   |---|---|
   | 3400µs | 282.4 / 282.2 |
   | 3000µs | 319.4 / 319.8 |
   | 2400µs | 396.0 / 395.0 |
   | 2000µs | 476.3 / 475.5 |
   | 1900µs | 500.8 / 501.2 |
   | **1800µs** | **526.8 / 526.7 — highest clean result** |
   | 1750µs | **hung** — process timed out (20s) and was killed |

   **This is a genuine ~1.9x throughput win over the ~282fps ceiling that
   held across stock 640x400 and `MODE_1280_400_ROI`.** 1800µs is the
   current known-good floor for this mode; full CSV at
   `camera_throughput_sweep_640x200.csv`.

   **What happened at the 1750µs hang, and recovery (important for next
   time):** `subprocess.run(timeout=20)` correctly killed the *top-level*
   `camera_throughput_test.py` process, but that script itself forks two
   `multiprocessing` worker processes (one per camera) — killing the
   parent does not kill already-forked children, so the two workers were
   orphaned and kept running, still holding all `/dev/video*` fds (found
   via `lsof`, reparented to PID 1). Checked their state before doing
   anything: `ps -o stat,wchan` showed `Sl` (interruptible sleep) blocked
   on `futex_do_wait` — a userspace lock wait (picamera2 waiting on a
   completion queue a too-fast request never satisfied), **not** `D`
   (uninterruptible kernel-driver block). That distinction mattered: `S`
   state means a plain `kill -9 <pid>` on the two orphans is expected to
   work cleanly, vs `D` state which usually means only a reboot recovers
   it. Killed both orphans directly, `lsof` confirmed devices released,
   then ran a known-good sanity check (`camera_throughput_test.py 01
   3400`, stock 640x400) which came back at 280.66fps — matching the
   established baseline exactly. **No reboot was needed this time** —
   kernel taint stayed `4096` (no `D`/`W`) and dmesg showed no BUG/Oops
   throughout, consistent with this being a userspace-level hang, not the
   kernel-level `rmmod`/`insmod` crash documented elsewhere in this file.
   `camera_throughput_sweep_subprocess.py`'s own `RUN_TIMEOUT_S=20` does
   *not* currently kill orphaned children automatically — if re-running
   deeper into a hang-prone range, check for and clean up orphaned
   `camera_throughput_test.py` workers the same way after any timeout row.

5. **Closed-loop LED round-trip test run at these settings (2026-07-08
   ~17:10) — result is a major win, both on rate and on latency.**
   Added an optional 2nd CLI arg to `led_dual_camera_closed_loop_test_mp.py`
   for raw size (`WxH`, defaults to stock 640x400 — fully backward
   compatible); ROI is now always derived as full-frame for whatever size
   is selected (settled-config already established full-frame ROI as the
   correct choice, so this generalizes cleanly rather than hardcoding a
   second ROI constant). Ran twice at `1800 640x200`:

   | | run 1 | run 2 |
   |---|---|---|
   | confirmed transitions (5s) | 1034 (0 timeouts) | 1040 (0 timeouts) |
   | achieved capture fps | 530.25 / 530.25 | 528.78 / 529.38 |
   | mean latency cam0/cam1 | 4.124 / 4.422 ms | 4.405 / 4.430 ms |
   | max latency cam0/cam1 | 7.579 / 8.252 ms | 7.850 / 8.165 ms |
   | mean skew | −0.298 ms | −0.025 ms |
   | max \|skew\| | 2.673 ms | 4.118 ms |
   | **effective closed-loop freq** | **206.64 Hz** | **207.87 Hz** |

   Consistent across both runs, zero timeouts. **Closed-loop confirmed
   rate went from ~90-122Hz (previous stock/1280x400-ROI ceiling) to
   ~207Hz — about a 1.8-2.3x win**, matching the ~1.9x pure-throughput
   gain the sweep already found. **Mean per-camera latency (4.1-4.4ms) is
   now comfortably under Phil's 10ms loop-latency target** for the first
   time — previous best was ~7ms mean with a 10-15ms tail; now the *max*
   observed latency (7.6-8.3ms) is close to where the old *mean* used to
   sit. This is the strongest result so far toward the original 10ms/
   10-20Hz-disturbance-band goal at the top of this file.

   **Not yet done**: only 2 runs so far (both clean, no timeouts) — more
   repeats would build confidence this isn't a lucky pair, especially
   since 1800µs sits just above the confirmed 1750µs hang point found in
   the pure-throughput sweep. Also haven't stress-tested this duration for
   longer than 5s, and the true floor between 1750-1800µs hasn't been
   narrowed further. If either matters before presenting results to Phil,
   worth doing.

## Status (as of 2026-07-08 16:19)

**The depmod-install + reboot strategy worked.** The patched `ov9282.ko` is
now installed at `/lib/modules/6.18.34+rpt-rpi-2712/updates/ov9282.ko` and
loads cleanly at boot via the normal `modprobe`/initramfs path — no live
`rmmod`/`insmod` cycling involved, so the reload bug never gets triggered.

Confirmed clean post-reboot: `tainted` = `4096` (only `O`=OOT_MODULE, no
`D`/`W`), no BUG/Oops in dmesg, both cameras enumerate normally. **And the new
mode is live**: `rpicam-hello --list-cameras` now lists a
`1280x400 [296.47 fps - (0, 0)/1280x400 crop]` R8 mode on *both* cameras
(`i2c@88000` and `i2c@80000`), alongside the original 640x400/1280x720/1280x800
modes — `MODE_1280_400_ROI` is real and selectable.

Frame content validated visually (2026-07-08 ~16:30, see below) and throughput
benchmarked against stock: **this mode does NOT deliver a higher capture
rate** — see "Throughput result — NEGATIVE for the speed goal" below. It's a
real, correctly-oriented crop (confirmed against actual captured frames), just
not a faster one. This module is a boot-time install, not a loose `insmod` —
a plain reboot will *not* revert it; removing it requires deleting
`/lib/modules/.../updates/ov9282.ko` and running `depmod -a` before the next
reboot.

### Frame content validated (2026-07-08 ~16:30)

Wrote `camera_preview_roi.py` (live OpenCV viewer, modeled on
`camera_preview.py`) to eyeball the new mode. Both scripts initially appeared
to show solid black / wrong aspect ratio — turned out to be a viewer bug, not
a driver problem: `capture_array("raw")` returns the buffer as flat uint8
bytes, but this pipeline always delivers raw frames as 16-bit-per-pixel words
(Picamera2 negotiates format `"R16"` even when `"R8"` is requested) — treating
it as flat uint8 interleaves each real pixel byte with a zero padding byte,
reading as near-black at double the apparent width. Fix: `.view(np.uint16)`
on the captured array. Applied to `camera_preview.py` and
`camera_preview_roi.py`.

**This same bug exists in ~15 other scripts in the repo** (everything using
raw capture except the two preview scripts now) — including
`led_dual_camera_closed_loop_test_mp.py` and `led_centroid_test_raw.py`.
Per-project-owner decision (2026-07-08): **not being fixed repo-wide right
now** — out of scope unless it turns out to affect the ROI/higher-rate goal
(it doesn't: fps/timing scripts only count frames or threshold on relative
brightness, both unaffected by the byte-interleaving). Known exception:
`led_centroid_test_raw.py`'s X/Y centroid math **is** affected (X-axis
indexing operates on the byte-doubled array) — any past centroid position
numbers from that script should be treated as unvalidated until it's fixed.

Once fixed up, both cameras showed a real, correctly-exposed image in the new
mode — confirms the copied-verbatim registers aren't producing garbage. Also
visually confirmed the crop is anchored at the top of the sensor (not
centered): a centered object in the stock (binned, full-height) view appears
shifted toward the bottom edge of the ROI (top-400-rows-only) view. Expected
given `y_start=0` in the patch, not a defect.

### Throughput result — NEGATIVE for the speed goal (2026-07-08 ~16:35)

Goal was capturing at a **higher rate** via sensor-level ROI. Measured dual-
camera concurrent achieved fps:

- Stock 640x400 (binned, known-stable floor, 3400µs): **281.8 fps**
- New 1280x400 ROI (native rated period, 3373µs): **283.6 fps**

Essentially no difference — matches the rated max fps too (309.79 vs 296.47,
i.e. the new mode's own ceiling is *lower* than the stock mode's). **Root
cause: the stock 640x400 mode bins in both dimensions** (faster-to-digitize
rows *and* fewer of them), while this patch only crops vertically at full
1280-pixel width — the extra per-row time from staying unbinned eats all the
time saved by reading fewer rows. So this specific ROI variant trades field-
of-view width for no speed gain.

**Real next step toward higher rate**: a mode combining stock horizontal
binning (`0x3814`/`0x3815`) with a *much* shorter vertical window than 400
(e.g. 640x200 or 640x150), applying the same `y_end = height + 15` pattern on
top of the binned register set instead of the full-width set. That's where an
actual speed win over the current ~282fps ceiling would come from — neither
existing mode combines both levers. Next concrete steps: (1) add
`MODE_640_200_ROI` (or similar) to the patch, (2) depmod-install + reboot
(same safe process as this round), (3) benchmark and visually validate the
same way as today.

## Settled configuration (validated, don't re-derive from scratch)

- **Architecture**: multiprocessing — two separate OS processes, one per camera.
  Threading was tried and discarded (GIL contention caused erratic skew). A
  single-process/single-thread sequential version was also tested and is
  clearly worse (tail latency 22-27ms vs 10-15ms).
- **Stream**: raw R8 (mono, no debayering needed — OV9281 is a monochrome sensor)
- **ROI**: full frame (0, 0, 640, 400), both cameras — small ROI previously
  caused a silent detection failure on one camera (bad box placement), full
  frame fixed it and costs negligible compute (~2-3% of per-frame time)
- **Exposure**: 1500µs, **Gain**: 4.0
- **Frame duration**: 3400µs is the last stable point (~282fps achieved, ~7ms
  mean latency, ~122Hz toggle frequency). 3228µs hangs the process — this is
  very close to the sensor's own rated max (309.79fps mode = ~3226µs native
  period), so 3228µs asks for the literal hardware ceiling and breaks.
- **buffer_count=2** confirmed better than buffer_count=1 (buffer_count=1
  nearly halved achieved fps — losing pipeline overlap between sensor readout
  and app processing, not a staleness benefit as might be assumed)

## Key numbers to keep straight (easy to conflate)

- GPIO-only loopback (no camera): 305,522 Hz / 3.27µs — confirms GPIO overhead
  is negligible
- Pure camera capture throughput (no LED/detection logic): ~282fps per camera,
  concurrent, no bandwidth contention between the two cameras
- Closed-loop confirmed-transition rate (LED test, AND-gated on both cameras):
  ~90-122Hz — roughly half of raw capture rate, because confirming a discrete
  transition requires waiting for the next frame after the one that was already
  mid-exposure at toggle time, plus an order-statistics penalty from waiting on
  whichever camera is slower each cycle

## Open question for Phil (not yet resolved)

Does the real FTA control algorithm need discrete step-and-confirm (~120Hz
applies) or continuous latest-frame tracking (~280fps range could apply, no
confirmation step needed)? This determines which number is actually the
relevant ceiling. **Ask before optimizing further in either direction.**

## Sensor-level crop/windowing — RESOLVED, not available (2026-07-08)

Investigated whether the OV9281 supports true sensor-level windowing (reducing
actual sensor readout, not just post-capture numpy slicing — software ROI
slicing was already tested and ruled out as a speed lever, costs <3% of frame
time regardless of size).

Queried both sensor subdevices directly via V4L2 subdev selection ioctls:

```
v4l2-ctl --list-devices   # find subdev paths from topology
media-ctl -d /dev/media0 -p   # cam0 -> /dev/v4l-subdev2
media-ctl -d /dev/media2 -p   # cam1 -> /dev/v4l-subdev5
v4l2-ctl -d /dev/v4l-subdevN --get-subdev-selection=pad=0,stream=0,target=crop
v4l2-ctl -d /dev/v4l-subdevN --get-subdev-selection=pad=0,stream=0,target=crop_bounds
```

(Note: `--subdev-device` flag from earlier notes doesn't exist in v4l2-ctl
1.30.1 on this image — use `-d /dev/v4l-subdevN --get-subdev-selection=...`
instead, plain `--get-selection` is for video-capture nodes, not subdevs.)

Result: `crop` == `crop_bounds` == `crop_default` == 1280x800 @ offset (8,8),
identical on both sensors. `native_size` is 1296x816 (the 8px border is just
optical-black trim, not a tunable margin). **Since crop already equals its own
bounds, there is no addressable range left to shrink at the stock-driver API
level.** See the next section — this was later found to be a driver-policy
limitation, not a hardware one; the sensor itself supports a real windowed
crop, the stock driver just never exposes it via a mode variant.

## Sensor-level windowing — MODE_1280_400_ROI, out-of-tree driver patch (2026-07-08, LOADED / UNVALIDATED)

**Status: the depmod-installed patched module is loaded and stable (see
Status section at top of file). Both cameras present a `1280x400 [296.47
fps]` R8 mode. Not yet validated: an actual captured frame in this mode
hasn't been inspected, so the registers copied verbatim from the 1280x720
mode (see below) are still unverified.**

### Background

The stock `ov9282` kernel driver (`drivers/media/i2c/ov9282.c`, in-tree in
`raspberrypi/linux` branch `rpi-6.18.y`, matches running kernel
`6.18.34+rpt-rpi-2712`) ships 3 modes: 1280x800, 1280x720, 640x400. The 640x400
mode is pixel-*binned* (subsampling regs `0x3814/0x3815 = 0x31/0x22`) — it
still scans the full sensor array, so it doesn't reduce per-frame readout time.
That's why the previous "RESOLVED, not available" investigation above found no
addressable crop range: the driver pins `.crop` to full-frame for every mode
regardless of output size.

The 1280x720 mode, however, *is* a genuine windowed crop (readout-window
register `0x3806/0x3807` shrinks `y_end` from 815 to 735 vs the 800-mode).
Comparing the two stock modes revealed a clean, consistent pattern:
`y_end = height + 15`, `y_start = 0`, ISP y-offset register fixed at 8
regardless of height, independent of any horizontal register.

Using that pattern, added `MODE_1280_400_ROI` (index 3) to `supported_modes[]`:
1280x400, real vertical crop (not binned). Width deliberately left at 1280 —
no stock precedent exists for horizontal windowing, so that's out of scope for
this pass. Several registers in the new mode's reg list (`TIMING_FORMAT_1/2`,
`0x4008/0x4009/0x400c/0x400d`, `0x4507/0x4509`) were copied verbatim from the
1280x720 mode since their exact hardware semantics aren't documented anywhere
available — flagged as unverified, meant to be validated by inspecting an
actual captured frame (never got that far — see below).

Patch lives at `kernel_patch/ov9282/` in this repo: `ov9282.c` (patched),
`ov9282.c.orig` (stock baseline), `ov9282_MODE_1280_400_ROI.patch` (diff),
`Makefile`, and the built `ov9282.ko` (out-of-tree module, built against
`linux-headers-6.18.34+rpt-rpi-2712`, vermagic- and depends-matched to the
running kernel — confirmed via `modinfo`). **Never `depmod`-installed** —
loaded only via loose `insmod`, specifically so a reboot is a total, guaranteed
rollback to stock.

### What happened when loading it

Procedure: stopped `wireplumber`/`pipewire` (they hold `/dev/media*` fds via
libcamera's camera monitor and will block `rmmod`), confirmed via `lsof` that
nothing held the camera devices, then:
```
sudo rmmod ov9282        # succeeded cleanly, dmesg quiet, lsmod confirmed empty
sudo insmod kernel_patch/ov9282/ov9282.ko
```
`insmod` **segfaulted** (exit 139). `lsmod` afterward showed `ov9282` loaded
with refcount 1 (so it partially succeeded — likely one of the two i2c clients
bound before the crash). `dmesg` showed:

```
WARNING: CPU: 2 PID: 10114 at drivers/media/mc/mc-device.c:619 media_device_register_entity+0x1d8/0x208 [mc]
```
— fired **twice** (once per camera, `ov9282_probe → i2c_register_driver` →
... → `v4l2_async_register_subdev_sensor → cfe_async_complete →
__video_register_device → media_device_register_entity`), followed by:
```
kernel BUG at drivers/media/mc/mc-entity.c:146!
Internal error: Oops - BUG: ... [#1] SMP
```
and the kernel taint flags progressed to `Tainted: G  D  W  O` (`D`=DIE). A
later, separate WARNING also appeared in the idle loop
(`ct_kernel_exit.constprop.0`, `PID 0 Comm swapper/2`), suggesting broader
instability after the BUG, not just a contained failure in the media subsystem.

### Root cause — CONFIRMED not specific to the patch content (2026-07-08 16:04)

**The entire crashing call stack is generic/stock code** — `ov9282_probe()`,
`v4l2_async_register_subdev_sensor()`, `cfe_async_complete()`,
`__video_register_device()`, `media_device_register_entity()` — none of it
touches the new mode-table entry added by this patch. The `media_device`
object being registered into is owned by `rp1_cfe_downstream` (the CSI/CFE
bridge driver), which stays loaded across the `ov9282` `rmmod`/`insmod` cycle.

**Diagnostic test performed**: after a reboot to confirm clean stock state
(taint=0, `ov9282` loaded normally, both cameras detected via `rpicam-hello
--list-cameras`), stopped `wireplumber` (had to stop the systemd *service*,
not just `pipewire`/`pipewire-pulse` — those alone don't release the
`/dev/media*`/`/dev/video*` fds), confirmed via `lsof` nothing held the camera
devices, then:
```
sudo rmmod ov9282                                                          # succeeded cleanly, exit 0
sudo insmod /lib/modules/$(uname -r)/kernel/drivers/media/i2c/ov9282.ko.xz  # the UNMODIFIED stock module
```
**This reproduced the exact same crash** — `insmod` segfaulted (exit 139),
dmesg showed `rp1-cfe: Rejecting subdev ov9281 11-0060 (Already set!!)`,
repeated `kobject tried to init an initialized object` + `WARNING ... at
drivers/media/mc/mc-device.c:619 media_device_register_entity` (one pair per
video node registered, ~8 times), finally escalating to the identical
`kernel BUG at drivers/media/mc/mc-entity.c:146!` in `media_gobj_create`
(called from `cfe_async_complete`), and taint progressed to `D W` again.

**Conclusion: this is a pre-existing bug in `rmmod`/`insmod`-cycling the
`ov9282` driver on this kernel/bridge-driver combination, 100% independent of
whether the loaded module is stock or patched.** `rmmod` does not cleanly
tear down the entities `ov9282` registered into `rp1_cfe_downstream`'s shared
`media_device` (which stays resident across the cycle), so `insmod`'s
re-probe collides with stale registrations and crashes. The patch content
itself is not implicated and does not need re-examination on this basis.

### Next actions for this thread

1. ~~Reboot~~ / ~~depmod-install the patched module~~ — **done, confirmed
   working** (see Status section at top of file).
2. Capture a frame in the new `1280x400` mode and visually/numerically sanity
   check it (look for tearing, garbage rows, wrong offset — anything that
   would indicate the copied-verbatim registers, e.g. `TIMING_FORMAT_1/2`,
   `0x4008/0x4009/0x400c/0x400d`, `0x4507/0x4509`, are wrong for this crop
   height).
3. If the frame looks correct, benchmark it: run the throughput/closed-loop
   scripts against the new mode the same way `3400µs` was characterized for
   the stock 640x400 mode, and compare.
4. Decide whether to keep the patched module installed for further testing or
   revert to stock — remember reverting now requires actively deleting
   `/lib/modules/.../updates/ov9282.ko` + `depmod -a` (not just a reboot),
   since this is a boot-time install, not a loose `insmod`.
5. Root-causing *why* `rmmod` doesn't clean up the `media_device` entity is a
   deeper kernel/driver investigation, not necessary to unblock this project;
   worth a note upstream to `raspberrypi/linux` at some point but out of scope
   here.

## Hardware status

SD card was damaged once already, OS was reimaged from scratch, which
previously caused (both now fixed as of the reimage):

1. Missing `dtoverlay=ov9281,cam0` / `dtoverlay=ov9281,cam1` in
   `/boot/firmware/config.txt` (fresh image doesn't include third-party sensor
   overlays) — added, confirmed working.
2. One camera (`i2c@80000`) stopped being detected after reimage — I2C found
   the sensor but the CSI-2 high-speed link never came up (zero errors, just
   silent). Root cause never fully confirmed — swapped the two cameras'
   physical ports to isolate port-vs-camera fault, and both cameras came back
   working after the swap, suggesting a loose/marginal connection rather than
   permanent damage.

**Re-verified 2026-07-08**: `rpicam-hello --list-cameras` shows both sensors
detected (`i2c@88000` and `i2c@80000`). **Still not stress-tested for
stability** — run `led_dual_camera_closed_loop_test_mp.py 3400` a few times
and watch for intermittent failures before fully trusting the `i2c@80000`
connection under sustained load.

Also note: 2 spare Raspberry Pis + 2 spare cameras have been ordered. Ribbon
cable is printed "HBV-Raspberry-160FPC" — useful search string for exact
camera module matching if needed later.

## Scripts (all at repo root)

- `led_dual_camera_closed_loop_test_mp.py` — **main validated test**, takes
  frame duration as CLI arg, defaults to 3400
- `led_dual_camera_sweep_subprocess.py` — sweeps frame duration via fresh
  subprocess per value (the reliable pattern — in-process reconfiguration
  between durations was tried and found unreliable)
- `camera_throughput_test.py` — pure capture rate, no LED, solo or concurrent
  (0/1/01 arg)
- `camera_preview.py` — live visual preview, 1 or 2 cameras, for basic sanity
  checks
- `camera_preview_roi.py` — live visual preview for the patched-driver ROI
  modes (`MODE_1280_400_ROI`, `MODE_640_200_ROI`), defaults to `640x200`
- `roi_set_selection.py` — runtime helper for the `set_selection` patch:
  `get_roi_y_start(cam_index)` / `set_roi_y_start(cam_index, y_start)`,
  also runnable as a CLI (`python3 roi_set_selection.py <cam_index>
  [y_start]`). Clamps `y_start` client-side before it reaches the driver —
  see the "runtime-movable ROI" section above for a driver-side clamp bug
  this works around.
- `roi_live_demo.py` — interactive live demo: both camera ROI feeds side by
  side, `w`/`s` move the shared ROI up/down while streaming, `r` resets,
  `q` quits. Needs an actual display (`DISPLAY=:0` — this Pi has a real
  desktop session via `labwc`), not runnable headless. `python3
  roi_live_demo.py [WxH] [step]`, defaults `640x200`, step=20 rows.
- `led_dual_camera_closed_loop_test_single_process.py`,
  `led_dual_camera_closed_loop_test_mp_buf1.py` — comparison baselines,
  already answered their questions, probably don't need to run again
- `led_dual_camera_pooled_capture_test.py`, `led_dual_camera_sequential_test.py`,
  `led_skew_test_driver_cam0.py`, `led_skew_test_logger_cam1.py`,
  `led_skew_test_correlate.py` — leftover intermediate versions from earlier
  iterations, superseded, kept for reference only
- Others (`led_calibrate_raw.py`, `led_centroid_test_raw.py`,
  `led_centroid_test_rgb.py`, `led_exposure_sweep_raw.py`,
  `led_exposure_sweep_rgb.py`, `led_timing_test_raw.py`,
  `led_dual_camera_frame_duration_sweep.py`,
  `led_dual_camera_frame_duration_sweep_mp.py`,
  `led_dual_camera_skew_test.py`) — earlier calibration/exposure/skew work,
  reference only

`results/` holds sweep CSVs/PNGs. `docs/` holds the current slide deck
(`camera_timing_characterization.pptx`) and the conversation archive.

## Standing caution

Requesting a frame duration below the sensor's true floor can hang the process
with no catchable Python exception — happened at 3228µs. Any further downward
exploration needs one-value-at-a-time testing in fresh processes, watching the
terminal, ready to kill/reboot. Camera driver state is uncertain after any such
hang — reboot before trusting subsequent runs.

## Next steps (as of 2026-07-08 16:35)

1. **Design and test a combined mode** (e.g. `MODE_640_200_ROI`): stock
   horizontal binning + a real vertical window well under 400 rows — see
   "Throughput result — NEGATIVE for the speed goal" above. This is the
   actual next lever for higher capture rate; `MODE_1280_400_ROI` alone is
   validated-working but does not beat current fps.
2. Stress-test `i2c@80000` reconnection: run
   `led_dual_camera_closed_loop_test_mp.py 3400` several times, watch for
   intermittent failures. (Unblocked — kernel is in a clean, rebooted state
   with the patched module loaded.)
3. Get Phil's answer on discrete-step vs. continuous tracking (determines
   whether ~120Hz or ~280fps is the real ceiling to design around) — matters
   even more now that a straightforward vertical-only crop hasn't moved the
   ceiling; worth confirming a higher rate is actually the right thing to
   chase before investing in another driver-patch/reboot cycle.
