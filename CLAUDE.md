# FTA Dual-Camera Timing Characterization

Characterizing Raspberry Pi 5 dual-camera (OV9281) timing performance for an FTA
beam-tracking closed-loop control system. Target set by Phil (Prof. Lubin):
10ms loop latency, disturbance band 10-20Hz beacon wobble.

This file is the living state of the project. Update it as findings change —
don't let it drift from what's actually true. Full prior conversation history
(pre-2026-07-08) is archived in `docs/archive/handoff_conversation_2026-07-08.txt`
for context, but treat *this* file as authoritative, not the archive.

**Repo now lives on GitHub, multi-device work starts here (2026-07-16)**:
`https://github.com/ucsbdeepspace/rpi_camera_system` (pushed from this Pi,
`master` branch). This Pi has a deploy key with write access
(`~/.ssh/id_ed25519_deepspace`, configured for `Host github.com` in
`~/.ssh/config`) — a **different** device (e.g. the laptop doing Nucleo
firmware work below) needs its own key/credential added separately, this
one doesn't transfer. Claude Code conversations and memory are local to
each device/directory, not synced — this file is the intended hand-off
mechanism between them. Anyone picking this project up on a new machine
should clone the repo and start fresh here, not expect prior-conversation
context to carry over.

**Laptop is now set up (2026-07-21)**: repo cloned there. Push access from
this laptop works via HTTPS + Git Credential Manager (`credential.helper =
manager`), confirmed — no separate deploy key was needed, unlike the
original assumption below.

**Nucleo firmware bring-up done from the laptop (2026-07-21)** — see the
"IN PROGRESS" section below for full detail. Summary: CubeIDE project
`camera_centroid_receiver` configures I2C1 as a slave at address `0x42` and
receives/checksums the 8-byte packet from `nucleo_i2c_sender.py`, with a
USART2→ST-Link VCP debug print added for visibility. **Folded into this
repo (2026-08-04)**: originally lived only in the laptop's local
`STM32CubeIDE/workspace_1.14.0/camera_centroid_receiver` workspace (a real
gap — if the laptop was lost/reimaged, that firmware work would have been
gone, unlike every other artifact this file tracks); now tracked at
`nucleo_firmware/camera_centroid_receiver/` in this repo, build-output
dirs (`Debug/`/`Release/`) gitignored. The laptop's CubeIDE workspace still
needs to be re-pointed at the new path (see "Firmware phase 1" section
below) — the physical files were moved, not copied, so the old workspace
location no longer exists.

Physical wiring between the Pi's header I2C1 (physical pin 5=SCL/GPIO3,
pin 3=SDA/GPIO2, plus GND) and the Nucleo's I2C1 pins (board silkscreen
labels `D5`=PB6=SCL, `D4`=PB7=SDA, confirmed against CubeMX's own PB6/PB7
pin assignment) is **done** — no level shifter needed, both boards are
3.3V logic, Pi's I2C1 header pins have built-in pull-ups.

**End-to-end validation — CONFIRMED (2026-07-21).** With
`nucleo_i2c_sender.py`'s fake-orbit smoke test running on the Pi and
`nucleo_serial_monitor.py` watching the Nucleo's VCP output on the
laptop: `seq` incremented 1:1 with the running `pkts` counter, `status=1`
throughout, `x`/`y` smoothly traced the sender's sine/cosine orbit, and
`errs` stayed at 0 the entire time — a real, clean round trip, not just
both sides individually looking plausible. See "Nucleo firmware built,
wiring done" below for the full firmware detail; that section's earlier
"not yet hardware-validated" caveat is now resolved.

## RESOLVED (2026-07-29): I2C1 bus/controller scare was a wedged controller state, cleared by reboot — NOT permanent Pi-side damage; amp board remains the sole real fault

Later the same day as the amp-board finding and the auto-track fps fix
below: the Nucleo now fails to answer **whether or not it's plugged into
the amplifier board** (previously it only failed when connected to that
board — see "Hardware note" below) — a regression from earlier in this
same session. `sudo i2cdetect -y 1` is slow again too (was confirmed fast,
~0.03s, immediately after the fps-fix testing above), and **the user
reports it stays slow even with the I2C lines physically disconnected
from anything** — which is the key, suspicious data point: a healthy bus
with nothing wired to it should scan fast (NACK immediately), not hang.

Confirmed directly, not just by user report: `time timeout 15 sudo
i2cdetect -y 1` took the full 15s and only got partway through the
address range before being killed. `dmesg` shows the real cause —
repeated, hard controller-level failures, not a userspace/driver-level
slowness:
```
i2c_designware 1f00074000.i2c: controller timed out
```
firing roughly once per second, one per probed address — `1f00074000.i2c`
is the RP1 I2C1 controller backing `/dev/i2c-1`, the same bus
`nucleo_i2c_sender.py` uses (see that module's docstring for the
`i2c@74000`/`/dev/i2c-1` mapping).

**Why this is a bigger deal than "the Nucleo is broken" again**: if the
I2C lines are genuinely disconnected and it's still timing out on every
address, the fault can't be in the Nucleo or the amp board anymore — it
has to be upstream, on the Pi side. Two live hypotheses, not yet
distinguished:
1. The RP1 controller got wedged into a bad internal state by whatever
   electrical event happened on the amp board (a stuck-low SDA/SCL line
   mid-transaction can do this to many I2C controllers) — recoverable by
   a plain reboot, since that resets the controller's hardware state.
2. Actual damage to the Pi's I2C1 hardware itself (GPIO2/GPIO3 pins or
   the RP1 controller silicon) from whatever backfed through the Nucleo
   while it was connected to the amp board — would NOT be fixed by a
   reboot, and would mean the amp-board fault damaged the Pi, not just
   the Nucleo(s).

**Post-reboot result — hypothesis 1 confirmed (2026-07-29).** `sudo
i2cdetect -y 1` now cleanly finds the Nucleo ACKing at `0x42` when it is
NOT plugged into the amplifier board — fast, clean scan, real device
found, not a timeout. This rules out hypothesis 2 (permanent Pi-side
I2C1/RP1 damage): a genuinely damaged controller or header pins would
not suddenly detect a real device correctly after nothing but a reboot.
The controller was simply wedged into a bad internal state by whatever
electrical event happened via the amp board, and a plain reboot cleared
it, exactly as hypothesis 1 predicted.

**Net effect: this whole scare collapses back into the already-known
amp-board fault** documented in "Hardware note" below — the Nucleo works
correctly on this bus when isolated from the amp board, consistent with
that section's finding (a fresh Nucleo also worked everywhere except
plugged into the amp board). No new Pi-side hardware issue. Safe to
resume treating the amp board as the sole suspect; no multimeter check
needed. Not yet re-tested: whether `i2cdetect` still times out
specifically when the Nucleo *is* connected to the amp board post-reboot
(expected, per the amp-board finding, but not re-confirmed this session).

**Re-tested, connected to the amp board — confirms amp-board fault, and
reveals it's intermittent, not a fixed hard fault.** First check (user):
hung/timed out, as expected. Immediately after (Claude, via Bash, same
physical setup, ~seconds later): three consecutive clean scans, ~0.024-
0.030s each, no device found at any address (not `0x42`, not a timeout).
Re-run by the user again right after that: also clean/fast, no device
found. So on the same physical connection, in short succession, the
fault showed up as a hard timeout once and as a silent "nothing there"
several times — **this flip-flopping is itself informative**: a solid
short or a permanently dead pin would misbehave the same way every time,
whereas symptoms that change between a hang and a clean-but-empty scan
without anything being deliberately altered points at **marginal/
intermittent contact** (a loose connector, a cracked solder joint, a wire
just barely making contact) rather than a fixed dead short. Either way,
the Nucleo never once ACKed at `0x42` while connected to the amp board
across any of these checks — consistent with the "Hardware note" finding
below regardless of which failure signature shows up on a given attempt.

**Next step**: to actually localize this (vs. just re-confirming "it's
still broken somehow"), try gently wiggling/reseating the amp board's
Nucleo connector while repeatedly running `i2cdetect` (e.g. a tight loop
printing timestamps) and watch whether hangs/clean-empty-scans correlate
with physical movement — that would confirm a loose mechanical contact
specifically, as opposed to a component-level fault on the board that
just happens to present inconsistently. A continuity check on the
connector's SDA/SCL/GND pins (multimeter, board unpowered) would be the
more definitive version of the same test.

**Localized further, and the fault is NOT a loose/marginal connector after
all — it's power-dependent and pin-specific (2026-07-29).** Confirmed the
fault isn't unique to one physical board: plugging the Nucleo into **any
of 3 amp boards on hand** stops I2C from working, ruling out a single
bad/damaged board as the explanation (points instead at something common
to this amp board design/wiring, not a one-off defect). Then, on the
primary board, connected the amp board's pins to the Nucleo **one pin at
a time** instead of via the full connector — this isolates the fault to a
**single specific pin: pin 23, one of the ADC pins going to the
amplifier**. Critically, pin 23 only causes the I2C failure **when the
amp board is powered on** — unpowered, connecting that same pin is fine.

This changes the diagnosis from the "marginal/intermittent contact"
theory above (which fit the earlier hang-vs-clean-scan flip-flopping) to
something more specific and reproducible: a powered ADC line on the amp
board is interfering with the Pi↔Nucleo I2C bus, most likely either (a) a
direct short/leakage path from pin 23 to SDA or SCL somewhere in the
harness/connector, or (b) noise/coupling injected onto the I2C lines by
that ADC pin only when it's carrying a live signal (unpowered = no
signal = no interference). **Not yet distinguished which** — the earlier
suggested continuity check (multimeter, board unpowered, pin 23 vs.
SDA/SCL/GND) is still the right next diagnostic for (a); confirming (b)
would need e.g. scoping pin 23 and the I2C lines simultaneously while
powered. Either way, this is real, useful narrowing: the fault is one
identified pin, not "something loose somewhere on 3 different boards."

**Pin-23/powered-only finding no longer reproducing on retest — real
observation at the time, but the picture has since changed (2026-07-29).**
The pin-23-only-when-powered result above did happen as described. On
retest, though, it stopped reproducing: one-pin-at-a-time connection (pin
23 included) no longer breaks I2C, powered or not. What *does* now
reliably break it: plugging in the **whole Nucleo connector** (all pins
via the actual connector, not one at a time) — and this happens
**regardless of whether the amp board is powered**, unlike the
powered-only behavior seen with pin 23 in isolation. Net effect: no
longer have a clean single-pin/single-cause explanation — whatever's
happening now depends on the full connector being mated (not any one pin
alone) and isn't gated by board power. The "marginal/intermittent
contact" theory from before the pin-23 detour is worth taking seriously
again given this — full-connector-only, one-pin-only-clean is at least
consistent with something connector-level (cumulative loading across many
simultaneously-connected pins, a marginal pin not covered by the
one-at-a-time test, or genuine mechanical contact quality at the
connector) rather than a single component-level fault. Next diagnostic
still stands: a continuity/short check across the full connector's pins
(multimeter, board unpowered) is more informative right now than per-pin
isolation, since isolation stopped reproducing the fault.

## RESOLVED (2026-08-03): amp-board I2C fault was the Nucleo-32's own on-board solder bridges (SB16/SB18) tying A4/A5 to D4/D5 — confirmed fixed by removing them

Picked the amp-board fault back up with an electrical approach (scope the
line, reason from what's on the wire) instead of more mechanical
reseating/isolation trials, since the last entry above left off without a
clean single-cause explanation.

**SCL scoped at only ~1.8V high with the amp board connected, vs. a clean
~3V high on the breadboard.** Informative on its own: 1.8V is close enough
to (or below) typical VIH thresholds for 3.3V CMOS I/O (commonly ~0.7xVDD ~
2.3V, sometimes lower in TTL-compatible mode) to plausibly explain the
flip-flopping symptom documented above (hard timeout on one attempt,
clean-but-empty scan on the next, same physical connection) — a borderline
logic high is exactly the kind of fault that reads differently scan to scan.
**Confirmed the line is still toggling, not clamped flat** — rules out a
low-impedance driver pinning SCL to a fixed level; this is a *loading*
effect (something on the amp board competing with the Pi's ~1.8k-ohm
pull-up), not a hard short.

**Ruled out rail sag**: VDD/3.3V measured directly at the Nucleo came back
identical (3.3V) whether on the breadboard or plugged into the amp board —
the pull-up's supply rail itself isn't the problem.

**Ruled out the SDA/SCL pins themselves being the coupling path into the amp
board.** Physically cut the SDA/SCL pins on the bottom of the Nucleo so they
no longer connect to the amp board's connector at all (I2C re-wired directly
to the top of the same pins instead, straight to the Pi); power and ground
were left connected. **No change** — SCL still capped at ~1.8V high. Key
negative result: whatever's loading the line down, it isn't reaching SCL
through the SDA/SCL pins on the amp-board connector, since that specific
path no longer physically exists.

**Root cause candidate found: the NUCLEO-L432KC (Nucleo-32 form factor)
ships with two solder bridges, SB16 and SB18, BOTH ON by default.**
Confirmed directly from ST's UM1956 (Nucleo-32 user manual) — fetched and
read the actual PDF (board bottom layout, Figure 4; solder-bridges table,
Table 8), not just secondhand text:
- **SB16** ties **D5 (PB6) to A5 (PA6)**
- **SB18** ties **D4 (PB7) to A4 (PA5)**

i.e. on this board, as shipped, D4/D5 — the I2C1 SCL/SDA pins this
project's whole Pi-to-Nucleo I2C link is built on (see "Nucleo firmware
built, wiring done" below) — are electrically the *same net*, inside the
Nucleo's own PCB, as the analog pins A4/A5. **This directly explains the
"no change" result above**: if the amp board's harness touches A4 or A5 for
any reason (not yet confirmed which — plausibly related to the already-
documented pin-23/ADC finding, or something else on the connector), that
connection rides straight back onto the SDA/SCL net through the internal
bridge, *upstream* of the D4/D5 header pins that were physically cut.
Cutting those pins couldn't have isolated anything, because the short way
back to A4/A5 lives inside the Nucleo board itself, not on the external
harness.

**Physical location**: bottom side of the Nucleo board, silkscreen-labeled
directly on the PCB ("SB16"/"SB18" printed right at the pads) — positioned
just below connector CN4, next to the crystal footprint (X2), grouped with
SB11/SB12.

**Confirmed via the STM32L4's official alternate-function table (fetched
and read directly, not CubeMX-guessed) that there is no alternate I2C pin
pair available on this board at all** — closes off "move I2C to different
pins" as an alternative to dealing with the bridges:
- A2/A3 (PA3/PA4, floated as a candidate this session) have **zero** I2C
  alternate function on any AF slot (AF4 is the I2C1/I2C2/I2C3 category;
  empty for both pins in the chip's real AF0-AF15 mux table). CubeMX can't
  offer I2C there because the silicon doesn't route it — no board-level
  workaround exists for this the way there is for the A4/A5 case.
- Scanned every pin actually broken out on this board's two headers
  (CN3+CN4, all of D0-D13/A0-A7) against the same table: **PB6/PB7 (D5/D4,
  already in use) are the ONLY pins on this entire board with real I2C
  SCL/SDA capability.** The only other I2C-related AF anywhere on the header
  is PB5 = `I2C1_SMBA` (SMBus alert line only, not usable as SDA/SCL). There
  is no fallback pin pair to move to even if desired.

**Net conclusion**: removing/desoldering SB16 and SB18 is not just the
preferred fix, it's the *only* available path to keep I2C working on this
board while breaking whatever's coupling in via A4/A5 — no firmware change
needed (PA5/PA6 default to input-floating on reset, which is what the
bridge-off state requires anyway), no rewiring of the already-validated
Pi-to-Nucleo D4/D5 harness.

**Removed SB16/SB18 — confirmed fixed (2026-08-03).** Immediately after
desoldering the bridges, I2C appeared to be down even on the plain
breadboard setup (previously the known-good baseline) — alarming at first,
since electrically the bridges shouldn't have touched the D4/D5 path to the
Pi at all (D4/D5 wire straight to the MCU's PB6/PB7 pads independently of
the bridge; only A4/A5 run through it). Treated this as likely rework
collateral damage rather than a real electrical dependency — SB16/SB18 sit
in a cramped cluster right next to the crystal (X2) and SB11, easy to
nick/bridge/lift something adjacent while working pads that small. After
cleaning up the rework, I2C came back — **and, critically, now works with
the Nucleo plugged into the amp board**, which is the fault this whole
thread was chasing. This confirms the SB16/SB18 diagnosis was correct: A4/A5
(tied to D4/D5 via the bridges) was the real coupling path from the amp
board onto the I2C bus, not the D4/D5 connector pins themselves, not ground,
not rail sag, and not anything else investigated earlier in this file. Amp
board's exact connection point on A4/A5 was never directly confirmed (no
continuity trace was done before the fix), but the outcome — fault present
with bridges in, gone with bridges out, on the same physical hardware
otherwise unchanged — is strong enough evidence to close this thread.

**Net result: the amp-board I2C fault documented across this whole file
since 2026-07-29 (bus/controller scare, pin-23 detour, full-connector-only
finding, marginal-contact theory, and finally this SB16/SB18 root cause) is
resolved.** Nucleo I2C is confirmed working with the amp board connected.
**Full pipeline re-validated end-to-end (2026-08-04).** Ran
`camera_view_tool.py` for real (default settings: camera 0, `640x200`,
streaming on, live I2C send to the Nucleo now plugged into the amp board) —
streamed 1600+ packets in ~9s, **0 send failures throughout**, `valid=True`
and a stable centroid (~962-966, ~568-572) the entire run. Confirms the
whole camera->centroid->I2C->Nucleo path is genuinely working through the
amp board, not just bare bus connectivity. (Run was stopped via an external
SIGINT rather than the `q` keypress, which produced a benign
`KeyboardInterrupt` traceback mid-frame-capture at shutdown — not a bug,
just an artifact of how the test was stopped; not a graceful-exit path
issue in the script itself.) Verified clean afterward: `tainted` stayed
`4096` (no D/W), no dmesg BUG/Oops, no lingering `/dev/video*`/`/dev/media*`
handles beyond the normal pipewire/wireplumber holders, and `i2cdetect`
still cleanly finds the Nucleo at `0x42` post-run. This closes out the
amp-board I2C fault thread for real — hardware fix confirmed, software path
confirmed, bus left in a known-good state.

## FIXED (2026-07-29): `camera_view_tool.py` auto-track collapsed 640x200 streaming fps from ~527fps to ~50-57fps — root cause was `roi_set_selection.py` re-resolving the subdev path from scratch on every call, not the I2C link

After the amp-board rewiring below got I2C genuinely working again (Nucleo
now ACKs at `0x42`, confirmed via `i2cdetect`), the user reported
`camera_view_tool.py` still only hitting ~30fps at 640x200, a mode
previously validated at ~527fps. Isolated step by step, ruling out causes
in order: (1) `NucleoLink.send_position()` itself — a direct 200-call
timing loop against real hardware gave median 1.02ms/call, mean 1.15ms,
only 2/200 fast-NACK failures — nowhere near slow enough to explain this;
(2) `beam_position_streamer.py` at the same `640x200` mode, real I2C send,
hit ~227-249/s and climbing — so the *link* clearly supports much higher
throughput than 30fps; the problem was specific to `camera_view_tool.py`.

Found it: `auto_track = STREAM_ENABLED` (`camera_view_tool.py`) means
auto-track recentering is ON by default whenever streaming is on, which it
is by default. Toggling it off live (`t` key) jumped the *instantaneous*
rate (the printed average is cumulative-since-start, which hid this) from
~50-57/s to ~240-250/s — auto-track was the bottleneck, but this was a much
bigger hit than the ~530→220-230fps already documented elsewhere in this
file for the same feature, so something about the recentering path itself
had gotten slower, not just "auto-track is inherently this costly."

Root cause: `roi_set_selection.py`'s `_subdev_for_camera()` — rewritten
2026-07-28 to fix a real boot-order bug (see that section further down) —
resolves the Picamera2-index-to-`/dev/v4l-subdevN` mapping by calling
`Picamera2.global_camera_info()` (spins up a full libcamera camera_manager
internally) plus a `media-ctl` + `readlink` subprocess per `/dev/media*`
device, **on every single call**, replacing what used to be a cheap
hardcoded dict lookup. Measured directly: ~30ms/call. `set_roi_y_start()`
calls it 2-3 times internally (clamp read, the write, the readback), and
`camera_view_tool.py`'s `apply_y_start()` (used for auto-track recentering)
adds another for `get_max_y_start()` — so one recenter event cost ~120ms+
before any retries, against a ~50ms/20Hz throttle interval it was supposed
to fit inside. That 120ms+ blocks the single-threaded capture loop
synchronously, which is what actually collapsed throughput.

**Fix**: added a module-level `_subdev_cache` dict in `roi_set_selection.py`,
keyed by `cam_index`, populated on first resolution and reused after.
Safe to cache for a whole process's lifetime — the physical wiring this
resolves is fixed hardware and can only change across a *reboot* (different
probe order), which always starts a fresh process anyway, so a per-process
cache can never see a stale mapping; this preserves the 2026-07-28 fix's
correctness while removing the repeated cost. Verified in isolation: first
`_subdev_for_camera(0)` call 31.85ms, next three calls 0.00ms;
`get_roi_y_start(0)` dropped to 3.36ms (just the remaining real
`v4l2-ctl` subprocess call). **Verified end-to-end**: fresh
`camera_view_tool.py` process, default settings (streaming + auto-track
both on, 640x200), stable **~187-190/s** for 30+ seconds, zero I2C send
failures — up from ~50-57/s before the fix, and now close to the ~220-230fps
previously documented for auto-track+detection alone (the remaining small
gap is plausibly the per-frame I2C send cost, ~1ms, which wasn't part of
that older benchmark).

**Committed**: `b87dc73` (`roi_set_selection.py` + this file). A separate,
unrelated stray edit sitting in `camera_view_tool.py`'s working tree (a
one-character docstring typo, "hUses" → "Uses", plus an accidental
executable-bit change) was cleaned up the same session — fixing the typo
and reverting the permission bit brought the file back to *exactly*
matching the last commit, so there was nothing new to commit there.
Re-ran the same speed check afterward on a fresh process to confirm the
cleanup didn't regress anything: stable **~187/s**, zero I2C send
failures, 30+ seconds — matches the fix verification above exactly.

## Hardware note (2026-07-29): the Nucleo physically mounted on/near the amplifier board has a real fault — a second, freshly-flashed Nucleo works everywhere EXCEPT plugged into that amp board

While chasing the "camera_view_tool.py can't reach the Nucleo over I2C"
finding from 2026-07-28 (still current — Nucleo was flashed with "FTA
Controller" serial firmware, not an I2C slave), the user swapped in a
second, newly-flashed Nucleo board and rewired it in. Result: **the new
board works fine everywhere except when actually plugged into the
amplifier board** — i.e. this isn't (only) the already-documented
firmware mismatch, there's a real hardware fault on/around the amplifier
board itself (bad connector, a short, a pin driving something it
shouldn't, etc.) that breaks a Nucleo once connected to it. Not yet
root-caused further — needs a continuity/short check on the amp board's
Nucleo connector before trusting that board with hardware again.

## fps root-cause (2026-07-29): `beam_position_streamer.py` "really slow" (~45fps) at default settings — NOT an I2C problem, it's full-sensor detection cost

After the rewiring above, the user reported `beam_position_streamer.py`
(run with no args, so full-sensor `1280x800` default) streaming at only
~45-46/s and asked why, suspecting the I2C link. **Root-caused via a
synthetic-frame benchmark of `find_beam_blob` in isolation (no camera
needed)**: at `1280x800` it costs **~22.55ms/call (~44 calls/s)**; at
`640x200` (the binned ROI mode, same one `camera_view_tool.py` streams by
default) it costs **~2.32ms/call (~430 calls/s)** — a ~10x difference from
contour-finding/centroid math scaling with pixel count, nothing to do with
I2C at all. The ~44fps ceiling this implies matches the user's observed
45-46/s almost exactly — **full-sensor detection cost, by itself, is the
entire bottleneck**, independent of whether the I2C send succeeds, fails
fast, or hangs.

Confirmed by actually re-running the script at `640x200`
(`python3 beam_position_streamer.py 640x200 --y-start N`, real I2C send,
not dry-run): **~227-249/s and still climbing** in a 5s sample, zero I2C
errors logged. This also confirms the amp-board rewiring fixed the I2C
side specifically (no `[Errno 121]` failures this time, unlike the
2026-07-28 finding) — the *remaining* gap from this to the ~335fps
documented elsewhere for this exact script/mode is most likely just this
sample being too short to reach steady-state (the average is cumulative
since start), not a new problem.

**Practical takeaway**: never run `beam_position_streamer.py` with no
size argument for real work — full-sensor is a detection-cost trap, not
just a slower camera mode. Always pass a binned ROI size (`640x200` or
`640x100`) plus a `--y-start` that actually brackets the beam (find it
first via `camera_view_tool.py` or a full-sensor `--dry-run`).

## IN PROGRESS: FTA position calibration — DAC setpoint → camera centroid, `fta_calibration.py` built, not yet run against hardware (2026-07-23)

Motivating idea (user): to close the loop out to an actual servo, sweep the
FTA over a grid of actuator setpoints, record where each one puts the beam
centroid, fit a matrix that captures the transform (including offset), then
invert it to command a desired centroid position. Settled on a 3x3
homogeneous affine (2x2 gain block + offset) for a single camera/2-axis
actuator, not a 4x4 — a 2-camera version (4 observed numbers, 2 actuator
inputs, overdetermined least-squares) was discussed as a later "average out
per-camera noise" idea, not built yet.

**Wrong turn, corrected**: initially assumed the FTA-driving Nucleo would
need a *new* I2C actuator-command packet, and started extending
`nucleo_i2c_sender.py` (`REG_POINTER_ACTUATOR`, `NucleoLink.set_actuator()`)
based on the wrong reference repo (`ucsbdeepspace/fta_calibration`, an old
pyboard-driven rig). **Fully reverted** (`git checkout --`) once the actual
reference repo was found: `ucsbdeepspace/7-element-array`, branch
`lock_in_2` (private). `nucleo_i2c_sender.py` is unchanged from before this
session.

**What that repo actually shows** (an "FTA Controller" STM32L432 CubeIDE
project — matches this project's NUCLEO-L432KC): the FTA is driven over
**USB-serial, not I2C** — an ASCII line-command protocol at **460800 baud**
(matches `FTA_GUI_PID.py`, an existing PyQt host GUI already using this
exact protocol) straight to two `DAC1` channels, 12-bit (`0`-`4095`,
firmware's own default safety clamp is **95-4000**, not the full range).
Direct commands `set_x <n>` / `set_y <n>` snap to an absolute setpoint
instantly; there's also a full PID framework (`set_kp_x`/`ki_x`/`kd_x`
etc., currently driving a lock-in/photodiode dither-and-gradient-ascent
tracker — matches the `lock_in_2` branch name, a different sensing
modality than this project's camera). Most useful find: an **already-working
`grid_scan x1 y1 x2 y2` command** that smoothly microsteps through a grid
(deliberately avoiding backlash/shock — NOT the same as repeated
instant `set_x`/`set_y` jumps) and streams one `SD <adc> <x_center>
<y_center>\n` line per sampled point (sampled only on the downward Y pass,
to avoid up/down hysteresis asymmetry). The `<adc>` field is that
firmware's own onboard photodiode reading — a different physical sensor
than this project's camera, not used by our fit but logged in case
cross-referencing it is ever useful. There's also an ISR-level hard
emergency-stop: sending a bare `!` byte (bypasses the line parser
entirely) sets `stop_requested` immediately.

**Repo access**: this repo's existing deploy key
(`~/.ssh/id_ed25519_deepspace`) could not be reused — GitHub only allows a
given public key to be a deploy key on one repo, and this one was already
attached to `rpi_camera_system`. Generated a second keypair,
`~/.ssh/id_ed25519_7element_array`, with a matching `~/.ssh/config` `Host
github.com-7element-array` alias, added as a **read-only** deploy key on
`ucsbdeepspace/7-element-array`. Cloned `lock_in_2` (shallow) into
scratchpad for reference only — not part of this repo, not committed here.

**Clipboard wrinkle worth remembering**: pasting the deploy-key public key
into GitHub's web UI silently failed via `xsel` even though the command
reported success — this Pi's desktop (`labwc`/Wayland) runs Firefox as a
**native Wayland client**, and `xsel` only reaches X11/XWayland clients, so
it was setting a clipboard Firefox never saw. Fix: `sudo apt-get install -y
wl-clipboard`, then `wl-copy` — that's the one that actually landed in
Firefox's paste. `xdotool` (used successfully elsewhere in this project,
e.g. `roi_live_demo.py` validation) has the same X11-only limitation and
also could not reach Firefox.

**Built `fta_calibration.py`** (committed `4827072`, pushed to
`origin/master`): reuses the firmware's own `grid_scan` command as-is (no
firmware changes) rather than driving individual `set_x`/`set_y` calls
per point from the Pi — this was a deliberate choice (user-confirmed) to
inherit the firmware's already-validated smooth/backlash-aware travel and
settle timing instead of reinventing it. Listens for `SD` lines purely as
a per-grid-point capture trigger; on each one, captures and averages
`--frames-per-point` (default 3) camera centroids via `find_beam_blob`
(duplicated from `camera_view_tool.py`/`beam_position_streamer.py` per
this project's established convention, not imported). After the sweep,
fits the 3x3 affine via least-squares — **forward direction (DAC →
centroid)**, deliberately: DAC setpoints are commanded exactly (noise-free
independent variable) while centroids are noisy measurements, matching
ordinary least-squares' assumption of where the noise lives. Inverts the
fitted matrix for future centroid → DAC lookups, warns if the 2x2 gain
block is near-singular (axes not independently moving the centroid) or if
RMS residual is large (nonlinearity/hysteresis/bad point), and saves the
raw sweep + both matrices to `results/fta_calibration_<UTC
timestamp>.npz`. Sends the firmware's `!` emergency-stop byte on
Ctrl-C/any abort so an interrupted run doesn't leave the FTA mid-scan.
`--dry-run` captures/detects repeatedly without touching the Nucleo, to
validate the camera pipeline alone. **Fit math validated against synthetic
data only** (recovers a known matrix to within injected noise, inverse
round-trips exactly) — script has never been run against real hardware.

**Open architecture question, not yet resolved**: the FTA Controller
Nucleo's USB will plug directly into this Pi (user-confirmed, no laptop
hop for this link) — but whether that's the *same* physical Nucleo
already wired for I2C centroid-receiving (running `camera_centroid_receiver`
firmware, see below) or a second board is undecided. A single MCU can only
run one firmware image at a time, so if it's the same board,
`camera_centroid_receiver`'s and "FTA Controller"'s functionality will
eventually need consolidating into one firmware image (or the I2C
centroid-streaming role gets dropped in favor of driving the FTA directly
from a Pi-computed setpoint) — flagging so it isn't forgotten, not
decided yet.

**Control law, not yet resolved**: once a trusted calibration matrix
exists, the natural next step is a PI feedback loop using the fitted gain
as the pixel-error → actuator-command conversion — NOT a single
open-loop inverse-and-command, which wouldn't reject the 10-20Hz beacon
disturbance this project's spec targets and wouldn't correct for
calibration drift/hysteresis over time. Not designed or implemented yet.

**`fta_calibration.py --dry-run` run for real — CONFIRMED (2026-07-28).**
`python3 fta_calibration.py 95 95 4000 4000 --dry-run` (grid corners unused
in dry-run, just argparse-required): camera 0 opened at full-sensor
1280x800, detected a real beam at ~(554, 32), stable across 5 captures
(554.0-554.2, 32.2-32.4 — well under 1px jitter). Capture/detection
pipeline confirmed working end-to-end; the Nucleo/serial side is still
untouched by this test (that's the point of `--dry-run`).

**Firmware/connection state re-confirmed (2026-07-28)**: same physical
board (`SER=0666FF515152827187143833`), alive at 460800 baud, `get_status`
→ `MODE_IDLE`, `amp_enabled=0`, `drv_enabled=0`, all error counters zero —
still running "FTA Controller", healthy idle boot.

**Does `fta_calibration.py` need to send `amp_enable` itself? Checked the
firmware source directly (not assumed) — no.** `grid_scan_roi()` in `FTA
Controller/Core/Src/main.c` (the routine the calibration sweep actually
drives) enables the amp GPIO itself at the top and disables it again when
the scan completes (`HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, ...)`,
~line 1171/1240) — self-contained, no client-side enable step needed.
Separately confirmed `drv_enable`/`drv_disable` (GPIO_PIN_9) is unrelated
to the X/Y DAC path entirely — it only gates `move_z()`, the Z-stage
stepper motor driver — so enabling it during the 2026-07-23 step-response
debugging session was a red herring, not part of what actually fixed that
issue (the physically unplugged amp cable). The `amp_enabled=0` idle
default seen in `get_status` is expected and fine; only `set_x`/`set_y`
(used by `fta_step_response_test.py`/`fta_serial_latency_test.py`, NOT by
`fta_calibration.py`) skip the auto-gate and need a manual `amp_enable`.

**First real sweep run — `fta_calibration.py 200 200 3900 3900` (2026-07-28)
— completed but result is NOT trustworthy, correctly self-flagged.**
462/1444 expected grid points recorded, saved to
`results/fta_calibration_20260728T211217Z.npz`. Fit came back RMS residual
**86.66px** (script's own sanity threshold is 5px) and a near-singular 2x2
gain block (det = -3.658e-06, close to the script's 1e-6 warning
threshold) — both warnings fired, calibration correctly rejected by the
script's own checks. Root cause is two distinct real problems, not a
script bug:

1. **Beam leaves this camera's field of view above DAC-x ≈ 900-1000.** For
   dac_x 200-900 the centroid moves substantially and sensibly (cy sweeps
   ~4px to ~200px as dac_x rises, matching the already-confirmed DAC-x↔
   pixel-y axis rotation). At dac_x ≥ ~1000 the centroid **freezes** at
   ~(555, 30-40) — <1px drift for the rest of the DAC-x range (up to 3900)
   and fully unresponsive to the entire dac_y sweep. That frozen value is
   suspiciously close to the beam's pre-sweep idle rest position measured
   in the same session's `--dry-run` (~554, 32) — best explanation: past
   dac_x≈900-1000 the beam physically leaves this camera's FOV (or hits a
   mechanical/optical limit) and the detector locks onto some other
   static bright feature instead of the real beam, not real tracking data.
2. **Real Pi-side serial data loss during the sweep.** Around
   dac=(1100,500) a line arrived corrupted (`SSD 572 3600 400` — fails
   the parser regex, silently dropped) and the log then jumps straight to
   dac_x=3600 — DAC columns 1200-3500 (~24 columns, ~900 expected points)
   never got recorded, with no corresponding time gap, meaning the
   firmware kept scanning at full pace while Python's serial reads
   desynced for a long stretch. Likely cause: `capture_centroid()` blocks
   on 3 camera frames per `SD` line while `grid_scan` streams with no
   flow control — if Python falls behind, the OS serial receive buffer
   overflows. Not yet root-caused further or fixed.

Fitting DAC 200-3900 as one linear affine averaged across two totally
different regimes (real motion + a frozen artifact) is why RMS blew up —
expected, not surprising, given the above.

**Next steps**: (1) re-run a NARROWER sweep confined to where real beam
motion was actually observed this session, e.g. dac_x 200-900ish (find
the real upper bound more precisely first, e.g. with manual `set_x` steps
around 900-1100), to get a trustworthy fit; (2) separately investigate
the serial data-loss issue (e.g. whether `capture_centroid` needs to be
faster, or the firmware needs pacing/flow control) before trusting a
larger/full-range sweep; (3) design/implement the PI control law once a
trusted calibration matrix exists.

### Serial-vs-I2C latency estimated, then a real hardware check found the Nucleo needs reflashing first (2026-07-23)

Asked to estimate the closed-loop performance hit of USB-serial (the FTA
Controller's actual link) vs. I2C (this project's existing
`nucleo_i2c_sender.py` link) before committing to the architecture.
Physics-based estimate (not yet measured): raw wire-bit-time is
comparable either way (~0.2-0.3ms), but USB-CDC's extra layers (1ms USB
Full-Speed frame interval, `cdc_acm` driver buffering) likely add
**~1-4ms of round-trip latency I2C doesn't have** — probably still inside
Phil's 10ms budget if the real-time path is fire-and-forget (no waiting
for the firmware's per-command ack line), but no longer "basically free"
the way I2C was.

Built `fta_serial_latency_test.py` (not yet committed) to replace that
estimate with real numbers: three non-destructive modes —
`ping` (round trip via read-only `get_status`), `setpos` (round trip via
`set_x` re-sent with the FTA's OWN current position, so nothing actually
moves, but exercises the real wait-for-ack command shape), and `burst`
(fires N `set_x` commands back-to-back with zero waiting, then diffs the
firmware's own `cmdq_stats` drop counter — a real 64-deep command queue,
`CMD_Q_SIZE`/`cmd_q_dropped` in `main.c` — before vs. after, to check for
genuine command loss at that send rate instead of guessing).

**Tried to run it — the physical Nucleo now connected to this Pi's USB is
still running `camera_centroid_receiver`, not "FTA Controller."**
Verified directly, not assumed: at 460800 baud (the FTA Controller's
rate), `get_status` produced garbage bytes — a baud-rate mismatch
signature, not silence. Dropping to 115200 baud immediately revealed the
old firmware's `heartbeat uptime=Ns pkts=N errs=N` line, exactly as
documented in the "Nucleo firmware built" section further below. **This
also resolves the one-board-vs-two-board question above: it's the same
physical board** — flashing "FTA Controller" onto it will retire its I2C
centroid-receiving role until the two are consolidated or it's reflashed
back.

**Handoff to the laptop — DONE (2026-07-23).**
1. ~~Unplug the Nucleo's USB from this Pi, plug it into the laptop.~~ Done.
2. ~~Clone (or pull) `ucsbdeepspace/7-element-array`, branch
   `lock_in_2`~~ — turned out **already done**, from an earlier,
   unrelated session: already cloned on the laptop, already on
   `lock_in_2`, even 2 commits ahead of `origin` (a prior local build).
   No cloning or credential work needed after all.
3. ~~Import the existing "FTA Controller" project~~ — also **already
   done**: it (and a sibling `PWM FTA Driver` project) already showed up
   in the same STM32CubeIDE workspace used for `camera_centroid_receiver`
   (`workspace_1.14.0`), registered from that same earlier session. No
   File → Import needed.
4. **Built and flashed with Run (not Debug) — done, no issues.**
5. **Sanity-checked — confirmed alive and healthy.** Sent `get_status\n`
   at 460800 baud, got back `status:0,0,0,0,0,39269,0,0,0,0,0`. Decoded
   against the field order in `FTA Controller/Core/Src/main.c` (~line
   1058 — `controller_mode,x,y,amp_enabled,drv_enabled,uptime_ms,
   adc_stale_cnt,adc_total_cnt,isr_overrun_cnt,isr_max_cycles,
   cmd_q_dropped`): `MODE_IDLE`, amp/driver both off (safe default),
   `uptime_ms=39269` consistent with a board freshly flashed and running,
   every error/drop counter at zero. A genuinely healthy idle boot, not
   garbage bytes or a stale/crashed reply.
6. **Nucleo's USB moved back to this Pi.** Not yet run from here:
   `python3 fta_serial_latency_test.py --mode ping` and
   `python3 fta_calibration.py --dry-run` — both still the actual next
   step, nothing beyond the sanity check above has been exercised yet.

**Board is now running "FTA Controller" firmware, not
`camera_centroid_receiver`** — the I2C centroid-receiving role documented
in the section below is retired until the two get consolidated or it's
reflashed back (see "one-board-vs-two-board question," now resolved,
above). While `camera_centroid_receiver` was still open on the laptop (but
**before** reflashing over it), a real bug was fixed in its *source* —
see "sub-pixel precision fix needs a matching firmware update" below —
but that fix was **deliberately never flashed**, since the board was
about to be overwritten with "FTA Controller" anyway and reflashing twice
in a row would have been pure waste. **If `camera_centroid_receiver` is
ever put back on this board, remember it needs a rebuild+reflash first**
— the currently-installed image on any board running that firmware
predates the `POSITION_SCALE` divide-by-10 fix.

### `fta_serial_latency_test.py` run for real — first attempt still showed the OLD firmware, reflash confirmed, then real numbers (2026-07-23)

First re-check from the Pi after the laptop's flash still showed the old
`camera_centroid_receiver` heartbeat at 115200 baud (same board serial
number, so genuinely the same physical unit, just not actually
reprogrammed yet) — turned out the first CubeIDE action taken had only
**built**, not flashed. Re-done properly (Run, not just Build); re-checked
from the Pi and confirmed a clean `status:...` reply at 460800 baud before
trusting any timing numbers.

**Real results, all three `fta_serial_latency_test.py` modes, committed
firmware version, single run (not yet repeated across multiple sessions
or actuator loads):**

| mode | n | min | mean | median | p95 | p99 | max |
|---|---|---|---|---|---|---|---|
| `ping` (`get_status`, wait-for-reply) | 300/300 | 1.924ms | 2.141ms | 2.001ms | 2.935ms | 3.004ms | 5.527ms |
| `setpos` (`set_x`, wait-for-ack) | 300/300 | 0.994ms | 1.261ms | 1.070ms | 1.937ms | 2.027ms | 2.138ms |
| `burst` (500× `set_x`, zero waiting) | — | — | — | — | — | — | sent at **2824 cmds/sec**, but **257/500 (51%) dropped** |

`setpos` (the command shape real control actually uses) came in *faster*
than `ping` — likely just `get_status`'s longer reply line (11 comma-
fields vs. `set_x`'s short ack) costing more polling-mode
`HAL_UART_Transmit` time on the firmware side, not something specific to
the command type.

**Interpretation — what's actually known vs. still estimated:**
- **I2C itself was never round-trip measured in this project** — the
  `nucleo_i2c_sender.py` link was only ever validated for *correctness*
  (seq/pkts lockstep, zero checksum errors against a fake orbit), never
  benchmarked for latency. So "how much faster is I2C" is still a real
  number (serial) being compared against an *estimate* (I2C,
  ~0.2-1ms), not two measured numbers side by side.
- **Taking the I2C estimate at face value**, serial's real wait-for-ack
  cost (~1.3-2.1ms) is roughly **1-2ms slower per round trip** — a real
  but modest gap. Stacked on top of the ~3.5-4ms mean per-camera
  detection latency already measured for the best closed-loop
  configuration (`MODE_640_100_ROI`), that's ~5-6ms total, still inside
  Phil's 10ms budget with a few ms of margin — **probably doesn't matter
  for round-trip latency alone**, if the real-time path avoids
  waiting on acks.
- **The throughput/reliability question — RESOLVED (2026-07-23), a real
  safe ceiling found.** Added a `sweep` mode to
  `fta_serial_latency_test.py`: paced fire-and-forget `set_x` at fixed
  candidate rates (100-2500Hz), diffing `cmdq_stats`' drop counter at
  each rate to find where drops actually start, instead of only knowing
  "too fast" from the unthrottled `burst` result above. First run hit a
  transient "no `cmdq_stats` reply" failure partway through (root cause
  not fully pinned down — possibly a residual ack from the prior rate's
  burst still in flight) that killed the whole sweep; fixed by adding
  retries to `get_cmdq_stats` and making `run_sweep` skip a failed rate
  instead of aborting. Re-run completed cleanly, all 12 rates:

  | req Hz | achieved Hz | dropped | drop % |
  |---|---|---|---|
  | 100 – 1600 | matches request | **0** | **0.0%** |
  | 1800 | 1796.4 | 36 | 12.0% |
  | 2000 | 1999.2 | 72 | 24.0% |
  | 2500 | 2496.4 | 116 | 38.7% |

  **Safe fire-and-forget ceiling: ~1600Hz with zero drops**, degrading
  sharply past that (a real cliff, not a gradual falloff). This is
  comfortably above the fastest camera-side rate achieved anywhere in
  this project (~880fps, `MODE_640_100_ROI`) — **throughput is not a
  bottleneck for serial at any camera mode currently in use.** Combined
  with the latency finding above (round-trip cost is small relative to
  the 10ms budget), there's no longer a clear performance argument for
  keeping the I2C link over serial for the real-time control path — the
  remaining reasons to keep both (if any) would be architectural
  (e.g. not wanting the Pi to be a single point of failure for actuation)
  rather than performance-driven.

**Full raw data (for slides) — test conditions**: 2026-07-23, Nucleo
serial `SER=0666FF515152827187143833`, "FTA Controller" firmware
(`ucsbdeepspace/7-element-array`, `lock_in_2`) freshly flashed, single
run each (not repeated across sessions), `x_center` held at 95 throughout
(the firmware's own default `roi_x_min` floor) — nothing physically
moved for any of these numbers, all four tests re-send/query the FTA's
own current position.

Round-trip latency (`--trials 300` each):

| mode | command | n | min (ms) | mean (ms) | median (ms) | p95 (ms) | p99 (ms) | max (ms) |
|---|---|---|---|---|---|---|---|---|
| ping | `get_status` (read-only) | 300/300 | 1.924 | 2.141 | 2.001 | 2.935 | 3.004 | 5.527 |
| setpos | `set_x 95` (wait-for-ack) | 300/300 | 0.994 | 1.261 | 1.070 | 1.937 | 2.027 | 2.138 |

Unthrottled burst (`--mode burst --burst-n 500`, zero pacing):

| n sent | elapsed | achieved send rate | dropped | drop % |
|---|---|---|---|---|
| 500 | 177.1ms | 2824 cmds/sec | 257 | 51.4% |

Rate-paced sweep (`--mode sweep`, default rate list, `--sweep-n 300` per rate):

| requested Hz | achieved Hz | dropped (of 300) | drop % |
|---|---|---|---|
| 100 | 100.3 | 0 | 0.0% |
| 200 | 200.7 | 0 | 0.0% |
| 400 | 401.3 | 0 | 0.0% |
| 600 | 601.9 | 0 | 0.0% |
| 800 | 802.4 | 0 | 0.0% |
| 1000 | 1003.0 | 0 | 0.0% |
| 1200 | 1203.6 | 0 | 0.0% |
| 1400 | 1404.1 | 0 | 0.0% |
| 1600 | 1604.5 | 0 | 0.0% |
| 1800 | 1796.4 | 36 | 12.0% |
| 2000 | 1999.2 | 72 | 24.0% |
| 2500 | 2496.4 | 116 | 38.7% |

### PI control law designed (roughly); step-response characterization built, but hits a real hardware gap — the FTA isn't moving this beam at all (2026-07-23)

**Design discussed for the closed-loop control law** (not yet implemented
as a script): since the target is basically fixed (keep the beacon
centered) and the thing being fought is a disturbance (the 10-20Hz
wobble), this is a regulator problem, not a moving-target-tracking one.
Proposed structure: (1) a one-time feedforward jump using
`fta_calibration.py`'s fitted `M_inv` to get near the right operating
point immediately, then (2) a continuous PI trim every camera frame after
that. Pixel error is converted into decoupled DAC-space error via the
calibration's 2x2 gain block inverse (`A_inv = inv(M[:2,:2])`) *before*
running two independent scalar PI loops — this reuses the calibration's
already-captured cross-axis coupling instead of re-solving it. Key
details flagged: use measured `dt` (not assumed-constant) for the
integral term; anti-windup guard against the DAC clamp (the firmware
clamps `set_x`/`set_y` independently, so the Python-side integral must
also stop accumulating once saturated, or it overshoots when the
disturbance reverses); freeze (don't integrate) on a lost-beam frame;
start with PI, not PID, since a derivative term would amplify the
centroid measurement's own pixel noise. **Gains (`Kp`/`Ki`) can't be
derived analytically from the calibration alone** — that only captures
the actuator's *static* gain, not its dynamic response (speed,
overshoot, resonance), which is what actually limits how aggressively
the loop can be tuned. A rough bandwidth sanity check from the latency
numbers above: total loop delay (~3.5-4ms camera + ~1-2ms serial ≈
5-6ms) puts a rule-of-thumb usable closed-loop bandwidth ceiling
(`~1/(2π·delay)`) around 29Hz — rejecting a 10-20Hz disturbance looks
plausible but without huge margin, reinforcing that the fastest safe
camera+serial combination matters for this spec, not just as a nice-to-have.

**Built `fta_step_response_test.py`** (not yet committed) to get the
missing actuator-dynamics data before picking real gains: commands a
step in one DAC axis via serial, captures camera frames at full speed
spanning the step (reusing the `find_beam_blob` duplication convention),
and computes rise time (10%-90%), overshoot, and settling time (within a
configurable pixel tolerance) from the logged centroid-vs-time trace.
Saves the raw time series to `results/` regardless of whether the
computed metrics come out clean.

**Tried to run it for real — found a hardware gap, not a script bug.**
Before trusting the full timed test, sanity-checked with simple manual
steps: a real beam IS visible and stably detected by camera 0 (~(541,
214), consistent across many frames). But sweeping `set_x`/`set_y`
across the *entire* safe DAC range (95 → 4000 → 95, both axes) produced
**zero detectable centroid movement** — position stayed pinned within
noise (&lt;0.3px) throughout. Checked whether the firmware's
`amp_enable`/`drv_enable` gates (an amplifier/driver stage between the
DAC and the physical actuator, off by default per `send_status_auto`'s
own reported state) were the missing piece — enabled both explicitly
(`amp_enable` → "Amplifier Enabled", `drv_enable` → "Driver enabled",
confirmed via a clean `get_status` reparse: `amp_enabled=1
drv_enabled=1`) and re-swept the DAC range again: **still zero
movement.** This rules out "just needed to enable the amp/driver" and
points at something further down the physical chain — the FTA actuator
not actually connected/wired to whatever's steering this beam, or this
camera not viewing the beam path this particular FTA actually steers.
Not something software can diagnose further; needs a physical check of
the actuator/optics setup.

**Next steps**: (1) check the physical FTA-to-beam optical/mechanical
path — is the actuator connected, powered, and actually in the beam's
path this camera sees; (2) once real movement is confirmed, run
`fta_step_response_test.py` for real to get rise time/settling
time/overshoot; (3) only then pick real `Kp`/`Ki` and implement the PI
control law described above.

### Root cause of the "hardware gap" — voice coil amplifier was unplugged; real step-response data now in hand (2026-07-23)

Turned out to be exactly that simple: the voice coil's amplifier was
physically unplugged. Re-checked with the same manual sweep used to find
the gap (`set_x`/`set_y` across 95→4000→95, both axes): **real movement
this time** — e.g. DAC-x 95→4000 moved the centroid from (658, 12) to
(701, 325). Confirms the FTA is now genuinely steering this beam.

**Confirmed a real ~90° axis rotation between the actuator and the
camera**: DAC-x mostly drives pixel-**y** (not pixel-x), and DAC-y
mostly drives pixel-**x**. Not a bug — a physical mounting/alignment
fact about this rig. This is exactly why the control law design above
decouples via the full 2x2 calibration gain block rather than assuming
DAC-x↔pixel-x — that assumption would have been wrong here.

**Bug found and fixed in `fta_step_response_test.py` itself**: the
analysis picked `cx` as the "primary" trace for an x-axis step (and `cy`
for a y-axis step) — given the axis rotation just confirmed, that's
frequently the WRONG, weakly-responding pixel axis. First run's reported
metrics (rise time 54ms, overshoot 78.7%, settling 132ms) were computed
against `cx`'s 26px delta while `cy` moved 148px for the same step —
analyzing noise, not signal. Fixed: now auto-picks whichever of cx/cy
actually moved more between the pre- and post-step windows and reports
which one it chose. All numbers below are post-fix.

**Real step-response results, `--axis x`, full-sensor 1280x800:**

Converted px → microns using the OV9281's real pixel pitch, confirmed
live via `Picamera2(0).camera_properties["UnitCellSize"]` = `(3000,
3000)` nanometers = **3.0µm/px** — not a datasheet guess, read directly
off the running sensor. Applies directly here since these runs used
full-sensor 1280x800 (`v_bin=1`, no binning correction needed); see
`MICRONS_PER_PIXEL` in `fta_step_response_test.py`.

| step | dominant axis | delta (px) | delta (µm) | rise time (10-90%) | overshoot | settling (2px / 6µm tol) |
|---|---|---|---|---|---|---|
| 95 → 2000 (large, 1905 counts) | cy | 161.1px | 483µm | unresolved (see below) | 22.4% | **793ms** |
| 2000 → 2100 (small, 100 counts) | cy | 9.4px | 28µm | unresolved | 42.3% | **45ms** |
| 2100 → 2000 (small, reverse) | cy | -8.1px | -24µm | unresolved | 0.0% | **67ms** |
| 2000 → 2100 (small, repeat) | cy | 8.2px | 25µm | unresolved | 21.4% | **80ms** |

**Rise time reads as ~0ms every time — a real measurement limitation, not
a real result.** Effective valid-frame capture rate during these runs was
only ~47-49fps (well under the ~143fps this raw size supports) — almost
certainly because the beam moves fast/blurs during the sharpest part of
a transition, dropping `find_beam_blob`'s confidence gate for exactly
those frames. The result: sampling jumps straight from a pre-step frame
to an already-mostly-settled post-step frame, with the truly fast part
of the rise invisible to this method. The settling-time numbers are more
trustworthy (measured during the slower tail, where detection is
reliable), but rise time needs a different measurement approach (e.g. a
faster/binned camera mode, or a photodiode-based approach) to resolve —
not attempted yet.

**The big finding: settling time depends heavily on step size, and this
matters a lot for the control law.** The huge first step (1905 DAC
counts) settled in ~793ms; three small steps (100 counts, closer to what
a real disturbance-rejection correction would actually look like each
cycle) settled in 45-80ms instead — over an order of magnitude faster.
Reads as slew-rate/large-signal limiting on big moves, not a fundamentally
slow actuator. **This is good news for the project's goal**: a ~45-80ms
small-signal settling time implies a dominant time constant on the order
of 10-20ms, i.e. an actuator bandwidth roughly comparable to (not
dramatically worse than) the ~29Hz ceiling already estimated from pure
loop latency — meaning the actuator's own dynamics probably aren't the
single dominant bottleneck for rejecting the 10-20Hz disturbance, though
neither leaves a lot of margin. Overshoot at small-signal is noisy
(0-42% across 3 repeats) — expected, since a few pixels of detection
noise is a much larger fraction of an ~8px step than of a ~161px one, so
these percentages shouldn't be over-trusted individually, only as "yes,
there's real overshoot, roughly tens of percent."

**Practical implication for gain tuning (not yet done)**: use the
small-step numbers as the basis for `Kp`/`Ki`, not the big-step numbers —
the small-step regime is what real closed-loop disturbance rejection
actually looks like. Expect to need real overshoot damping (via `Ki`
tuned conservatively, or eventually `Kd` despite its noise-sensitivity
cost) given the consistent nonzero overshoot observed. Repeat with more
trials once implementing the real controller, since 3 small-step repeats
is not a lot of statistical confidence on the overshoot number
specifically.

**Repeated on `--axis y`, same protocol — pattern holds (2026-07-23):**

| step | dominant axis | delta (px) | delta (µm) | rise time | overshoot | settling (2px / 6µm tol) |
|---|---|---|---|---|---|---|
| 95 → 2000 (large, 1905 counts) | cx | -121.3px | -364µm | unresolved | 0.0% | **1183ms** |
| 2000 → 2100 (small, 100 counts) | cx | -6.5px | -20µm | unresolved | 0.0% | **83ms** |
| 2100 → 2000 (small, reverse) | cx | 5.4px | 16µm | unresolved | 34.4% | **144ms** |
| 2000 → 2100 (small, repeat) | cx | -5.4px | -16µm | unresolved | 0.0% | **22ms** |

Dominant axis confirms `cx` for DAC-y steps, as expected from the ~90°
rotation (not a fluke of testing DAC-x only). Same big-vs-small-step
pattern holds: large step settled >10x slower (1183ms) than any small
step (22-144ms) — corroborating evidence across both actuator axes that
the slow big-step numbers are slew-rate/large-signal limiting, not a
fundamental actuator speed limit. **One new wrinkle**: y-axis small-step
settling times spread wider (22-144ms) than x-axis's tighter 45-80ms
cluster — worth another look (more repeats, or check whether y's
mechanical path has different damping/mass) before assuming both axes
can share one `Kp`/`Ki` pair; may need per-axis gains rather than a
single symmetric tuning.

Combined 8-run dataset (4 per axis) published as an interactive artifact
with hover tooltips, both axes' panels, and the full data table — see
`fta_step_response.html` build (not checked into this repo, published via
Claude's Artifact feature; regenerate from the `results/fta_step_response_*.npz`
files if needed).

### Architecture DECISION v2 (2026-08-04, supersedes the "dumb DAC" decision below): the PID controller runs ON the Nucleo — Pi is a pure centroid sensor streaming telemetry over I2C, laptop sends setpoint/tuning commands over VCP

After talking to Peter, reversed the 2026-07-28 "dumb DAC" decision
immediately below this one. That decision kept all control math in Python
specifically to avoid a CubeIDE rebuild-and-reflash cycle per gain change —
a real cost, but it's outweighed by a bigger concern surfaced this session:
the loop-bandwidth margin against the 10-20Hz disturbance band is tight
(~29Hz ceiling estimated from pure latency, with real closed-loop step
response showing overshoot at every step size tested), and a PID running in
Python is subject to OS/interpreter scheduling jitter in a way a bare-metal
MCU loop isn't. This is the more standard embedded-control pattern anyway:
fast deterministic inner loop on the MCU, slow supervisory setpoint updates
from a host. The reflash-per-tune cost is deliberately offset below by
making gains (and the calibration gain-block) live-settable over VCP,
instead of hardcoded.

**Roles, strictly separated:**
- **Pi**: camera capture + centroid detection only. Streams telemetry to the
  Nucleo over I2C. Never receives anything back, never makes a decision
  based on feedback, never issues a command to anything. This boundary was
  deliberately tightened mid-design after an inconsistency got caught: an
  earlier version of this plan had the Pi watching its own centroid to
  decide when a hypothetical future grid-scan target was "reached" — that's
  a control decision, which violates "Pi is just a sensor." Any future
  logic that needs to react to position (e.g. a target-position grid scan)
  belongs on the laptop, reading the Nucleo's relayed telemetry via
  `get_status`, not on the Pi.
- **Nucleo**: runs the actual PI control loop (pixel error, decoupled via
  the calibration's 2x2 gain-block inverse, into per-axis P+I with
  anti-windup), drives the DAC, owns amp-enable safety gating. Also
  supports a raw open-loop DAC passthrough mode for bench characterization
  (see below — this is not optional, several existing tools depend on it).
- **Laptop**: sends target-position setpoints and gain/calibration updates
  over the existing ST-Link VCP link (USART2, PA2/PA15 — already wired for
  `camera_centroid_receiver`'s heartbeat print, no new hardware). Also
  where any future closed-loop sequencing logic (e.g. a target-position
  grid scan) would live, per the Pi-boundary note above.

**I2C wire format does NOT need to change.** Under this architecture the
Pi's job is exactly what `nucleo_i2c_sender.py`/`camera_centroid_receiver`
were originally built for — stream centroid telemetry, nothing else. The
existing `seq/status/x/y/checksum` packet is already the right shape. Only
outstanding fix: the `POSITION_SCALE` divide-by-10 (sub-pixel precision,
flagged in an earlier session, never flashed — see "camera_view_tool.py
gains built-in full-speed I2C streaming" below) still needs to land in
firmware.

**DAC pins are now conflict-free.** DAC1_OUT1/OUT2 are PA4/PA5 — PA5 is the
same physical pin as this board's "A4" header position, which used to be
hard-wired to D4 (I2C1_SDA) via solder bridge SB18 (see the amp-board I2C
fault section above). With SB16/SB18 removed, PA4/PA5 are fully independent
of I2C1 now — no shared-net concern between the DAC output and the
telemetry input.

**Two firmware modes, not one — this is the part that's easy to get wrong.**
`fta_step_response_test.py` and `fta_calibration.py` both work by exciting
the raw actuator directly and watching the plant's own response — that's
not a convenience, it's required: you need open-loop actuator dynamics
*before* you can pick sensible gains, and calibration measures the raw
DAC-to-centroid transform, which a closed loop would corrupt by correcting
for it while you're trying to measure it. So:
- **`open_loop`**: raw DAC passthrough (`set_x`/`set_y`, same shape as
  today's "FTA Controller" commands), PID inactive, I2C telemetry received
  but ignored. Used for `fta_step_response_test.py` and
  `fta_calibration.py`.
- **`closed_loop`**: `set_target_x`/`set_target_y` sets the PID's setpoint;
  the loop runs continuously against streamed telemetry. Real operation.

**`grid_scan` is being dropped, not ported.** Its only real job was smooth,
backlash-avoiding travel between points for the (different, lock-in-based)
firmware it came from. Under this architecture, `fta_calibration.py`'s
sweep becomes a plain sequence of `set_x`/`set_y` calls in `open_loop`
mode — no firmware travel logic needed — *as long as* the grid step size
stays in the small-step regime already characterized (~45-80ms clean
settling, none of the slew-rate ringing seen on the one large 1905-count
step). This aligns with the earlier finding that small-step dynamics, not
big-step dynamics, are the right basis for gain tuning anyway. **Not yet
verified**: whether this actuator has real position-dependent hysteresis
(different reading approaching a point from opposite directions) — if so,
calibration should approach points from a consistent direction even with
plain jumps (trivial to arrange in Python), but this hasn't been tested. A
*separate*, future closed-loop grid scan (visiting known target positions
for e.g. tracking-repeatability testing) is a different tool entirely, and
per the Pi-boundary note above would live on the laptop, not the Pi.

**Command set (VCP, laptop<->Nucleo):**

| Command | Mode | Purpose |
|---|---|---|
| `set_mode open_loop\|closed_loop` | either | switch control mode |
| `set_x N` / `set_y N` | open_loop | raw DAC setpoint, bypasses PID |
| `set_target_x N` / `set_target_y N` | closed_loop | PID setpoint |
| `set_kp_x/ki_x/kp_y/ki_y N` | closed_loop | live gain tuning, no reflash |
| `set_gain_matrix ...` | closed_loop | load the calibration 2x2 gain-block (inverse) without a reflash |
| `amp_enable` / `amp_disable` | either | manual override; auto-gated by telemetry freshness in closed_loop (manual disable always wins) |
| `get_status` | either | mode, amp state, last raw/target value, last relayed telemetry (x/y + age), packet/checksum-error counters, uptime |
| `!` (bare byte, bypasses line parser) | either | ISR-level emergency stop, carried forward as-is from "FTA Controller" |

**Firmware function breakdown** (subsystem / where it runs — ISRs stay
short, matching the existing `camera_centroid_receiver` convention):
- *I2C1 (ISR)*: `HAL_I2C_SlaveRxCpltCallback()` — parse/checksum/apply
  `POSITION_SCALE`, store into `g_latest_telemetry` + timestamp;
  `HAL_I2C_ErrorCallback()` re-arms, unchanged from today.
- *VCP (ISR buffers only, parsed in main loop)*: RX ISR pushes to a ring
  buffer; `process_command_line()` dispatches to `cmd_set_mode()`,
  `cmd_set_x/y()`, `cmd_set_target_x/y()`, `cmd_set_kp_x/ki_x/kp_y/ki_y()`,
  `cmd_set_gain_matrix()`, `cmd_amp_enable/disable()`, `cmd_get_status()`;
  `handle_estop_byte()` stays ISR-level, bypasses the line buffer.
- *Control loop (fires on each fresh valid telemetry packet, not a
  timer)*: `run_control_step(x_meas, y_meas, dt)` — only in closed_loop
  mode, `dt` measured from the actual telemetry interval; calls
  `pixel_error_to_dac_error()` (applies the gain-block inverse) then
  `pi_update_axis()` per axis (P+I, anti-windup against the DAC clamp).
  `check_telemetry_staleness()` runs every main-loop tick; past a timeout,
  freezes both integrators and forces `amp_disable()`.
- *DAC output*: `apply_dac(axis, value)` — the only function that ever
  writes the DAC registers, clamps to [95, 4000], called by both
  `cmd_set_x/y` (open-loop) and `run_control_step` (closed-loop).
- *Amp/safety*: `amp_enable()`/`amp_disable()` (GPIOA12, shared by manual
  commands and the automatic staleness trigger); `estop()` disables amp,
  holds the DAC, latches a fault requiring an explicit clear.
- *Housekeeping*: `print_heartbeat()` (periodic VCP status, extended with
  mode/telemetry-age/amp-state vs. today's version); `main()` services the
  USART ring buffer, calls `check_telemetry_staleness()`, drives the
  heartbeat on its own cadence.

**Not yet decided/done**: exact staleness-timeout value for the fail-safe
amp gate; whether hysteresis needs directional-approach discipline in
calibration (see above); this is still firmware work requiring the laptop
(same one-board-one-firmware-at-a-time constraint as ever — this retires
"FTA Controller" as the flashed image again); `fta_calibration.py` and
`fta_step_response_test.py` will need updates to target this firmware's
`open_loop`-mode command set (likely minimal, since the command shapes are
deliberately kept the same as "FTA Controller"'s existing `set_x`/`set_y`).

### Firmware phase 1 (everything except PID) implemented in `camera_centroid_receiver`, build-verified, NOT yet flashed (2026-08-04)

Per the user's explicit sequencing ("get everything else working first, then
put a PID controller in last"), built every non-PID piece of the v2
architecture above into the existing `camera_centroid_receiver` CubeIDE
project (at the time, still only in the laptop's local
`STM32CubeIDE/workspace_1.14.0/camera_centroid_receiver` workspace, not this
git repo -- since folded in, see "Folded `camera_centroid_receiver` into
this repo" below). Deferred to the PID pass, per that sequencing: `run_control_step`/
`pixel_error_to_dac_error`/`pi_update_axis`, and the VCP setters that only a
running PID would consume (`set_target_x/y`, `set_kp_x/ki_x/kp_y/ki_y`,
`set_gain_matrix`). `set_mode closed_loop` is explicitly rejected (`ERR
closed_loop not yet implemented`) rather than silently stubbed, per the
user's call.

**What's built:**
- **DAC1** (`MX_DAC1_Init`, PA4=OUT1=x, PA5=OUT2=y) and **`apply_dac(axis,
  value)`** — the single choke point for DAC writes, clamps to
  `[95, 4000]` (same floor/ceiling as "FTA Controller"'s own default
  clamp). PA4/PA5 are conflict-free with I2C1 now that SB16/SB18 are gone
  (see the amp-board I2C fault thread above).
- **PA12 amp-enable gate** (active high, default LOW at boot) plus
  `amp_enable()`/`amp_disable()`/`estop()`. `estop()` disables the amp and
  latches `g_estop_latched`, which blocks `amp_enable()` until an explicit
  clear — **`clear_estop`, a command not named in the v2 command-set table
  above**, was added since the latch needs some way to release; flagging
  in case a different name/flow was intended.
- **VCP command link**: USART2 RX interrupt (single-byte
  `HAL_UART_Receive_IT`, re-armed each byte, same one-shot/re-arm
  convention the I2C reception already used) feeding a line buffer, parsed
  by `process_command_line()`. Implemented commands: `set_mode
  open_loop|closed_loop` (closed_loop rejected, see above), `set_x N` /
  `set_y N`, `amp_enable`, `amp_disable`, `clear_estop`, `get_status`. The
  bare `!` e-stop byte is handled at ISR level in
  `HAL_UART_RxCpltCallback`, bypassing the line parser, matching "FTA
  Controller"'s convention.
- **`get_status`** reports mode, amp/estop state, last commanded DAC x/y,
  the last *relayed* I2C telemetry x/y + its age in ms, telemetry
  packet/checksum-error counts, and uptime — **not the same reply format
  "FTA Controller" used** (that was a fixed positional CSV list; this is
  keyed `key=value` text). `fta_calibration.py`/`fta_step_response_test.py`
  will need their status-parsing updated to match, not just their DAC
  command calls, when they're adapted for this firmware (still open per
  the "Not yet decided/done" note above).
- Heartbeat (`print_heartbeat`, still free-running every 1s regardless of
  traffic) extended with `mode=`/`amp=`/`estop=`, for a glance-and-go bench
  check without needing `get_status`.
- I2C1 receive path (`process_beam_packet`) unchanged in wire format, only
  addition is `g_latest_beam_tick = HAL_GetTick()` per packet, feeding
  `get_status`'s telemetry-age field.

**Sourcing the DAC HAL driver**: this project's `Drivers/` only ever
contained the HAL source files CubeMX generates for the peripherals
actually selected in the `.ioc` (I2C1, USART2) — `stm32l4xx_hal_dac[.c/.h]`
and the `_ex` pair didn't exist here at all. Copied them from the cached
`STM32Cube_FW_L4_V1.18.2` package
(`~/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.2`, matching this project's
`ProjectManager.FirmwarePackage` exactly — not a different/newer version)
rather than hand-authoring a HAL driver from memory. Also flipped
`HAL_DAC_MODULE_ENABLED` on in `stm32l4xx_hal_conf.h`.

**Deliberately NOT added to the `.ioc`.** DAC1/PA4/PA5/PA12 and the USART2
NVIC interrupt exist only as hand-written code inside `USER CODE` marker
blocks in `main.c`/`stm32l4xx_hal_msp.c`/`stm32l4xx_it.c` — same tradeoff
already made for the PB3 LED GPIO earlier in this project (see
`MX_GPIO_Init`'s own comment). This means a future CubeMX "Generate Code"
from the `.ioc` will **not** delete any of this (USER CODE blocks always
survive regeneration) but **will** silently re-comment-out
`HAL_DAC_MODULE_ENABLED` in `stm32l4xx_hal_conf.h`, since that file isn't
USER-CODE-protected and is driven straight off the `.ioc`'s IP list — if
this project is ever regenerated, re-enable that line by hand.

**Build-verified, not hardware-verified.** STM32CubeIDE was already open
with this workspace (so a headless CubeIDE build would have fought the
workspace lock) — instead compiled every project source file directly with
the project's own bundled `arm-none-eabi-gcc` (13.3.rel1) against the real
include paths/defines/target flags, then linked the result against the
project's actual `STM32L432KCUX_FLASH.ld`: clean build, zero warnings even
under `-Wall -Wextra`, zero linker errors. **Never built inside CubeIDE
itself and never flashed** — that, plus a real bench pass (`get_status`,
`set_x`/`set_y` moving the actuator, `amp_enable`/`amp_disable`,
`clear_estop`, and the bare `!` e-stop), is the actual next step before
trusting this on hardware.

### Folded `camera_centroid_receiver` into this repo (2026-08-04)

Closed the tracking gap flagged at the top of this file since 2026-07-21:
moved (not copied) the CubeIDE project from the laptop's local
`STM32CubeIDE/workspace_1.14.0/camera_centroid_receiver` into
`nucleo_firmware/camera_centroid_receiver/` in this repo. `.gitignore`
already had `nucleo_firmware/*/Debug/` etc. rules waiting for this (unclear
who added them or when — they predate this move), so build-output
exclusion needed no extra work. Committed 104 files (source, `.cproject`/
`.project`/`.settings`, the `.ioc`, linker script, launch config); `Debug/`
correctly excluded.

**The already-open STM32CubeIDE instance on the laptop still points at the
old, now-nonexistent path** — moving the physical folder doesn't update
Eclipse's workspace metadata, which is separate from the project's own
files. Needs manual fixup next time that IDE window is used: remove the
stale `camera_centroid_receiver` project reference (right-click → Delete →
**uncheck** "delete contents", since the files are already gone from that
location anyway) and re-import from `nucleo_firmware/camera_centroid_receiver/`
in this repo (File → Import → Existing Projects into Workspace). Not done
yet as of this entry.

### Firmware phase 1 flashed and bench-tested — CONFIRMED working (2026-08-04)

Flashed the phase-1 build (previous section) to real hardware from the
laptop, bypassing CubeIDE entirely (it was closed by request, so this used
the bundled `STM32_Programmer_CLI.exe` directly against the `.elf` built
earlier): `STM32_Programmer_CLI -c port=SWD sn=066FFF515152827187153930 -w
firmware.elf -v -rst` — download verified, MCU reset. **Board identity
confirmed by the user**, not assumed: three ST-Link-capable Nucleos were
physically connected to the laptop at once (Device Manager showed three
distinct serials, one of which — `0666FF515152827187143833` — matches this
file's own on-record "FTA Controller" board serial from the 2026-07-23
latency-test session); with CubeIDE closed, `STM32_Programmer_CLI --list`
only detected one (`066FFF515152827187153930`, a different serial, board
type `NUCLEO-L432KC`), and the user confirmed that's the
`camera_centroid_receiver` target.

**Bench-tested over the VCP (COM4, 115200 baud) directly, no laptop-side
Python driver yet — just raw command/response.** Confirmed live, in this
order: heartbeat free-running (`mode=open_loop amp=0 estop=0`); real I2C
telemetry actively streaming from the Pi the whole time (thousands of
packets, 0 checksum errors — `camera_view_tool.py` or similar was already
running on the Pi during this test); `get_status` reporting correctly;
`amp_enable` → `OK amp_enabled`; `set_x`/`set_y` → `OK x=N`/`OK y=N` and
confirmed via a follow-up `get_status` (`dac_x`/`dac_y` updated); bare `!`
→ amp drops within one heartbeat tick, `estop=1` latches, DAC value held
(not zeroed) exactly as designed; `amp_enable` correctly refused
(`ERR amp latched by estop, clear_estop first`) until `clear_estop` →
`OK estop cleared` → `amp_enable` now succeeds; `set_mode closed_loop` →
`ERR closed_loop not yet implemented`, confirming the deliberate rejection
works; `set_x 99999` → `OK x=4000`, confirming the `[95, 4000]` clamp;
unknown commands correctly rejected. **Every phase-1 function behaves
exactly as designed.**

**Real finding: the VCP occasionally drops a character out of a command
line under live I2C load — not a parser bug, an interrupt-priority
artifact.** Saw it 3 times in the first ~19 commands sent while the Pi's
telemetry stream was running (`get_status`→`ge_status`,
`bogus_command`→`bogus_mand`, `set_y 95`→`set_y95`), then again on a later
retry (`se_y` before a clean `set_y 95` landed) — roughly 1 in 5-6 commands
during this session, always while telemetry was flowing, never observed
against a quiet bus. Root cause, not yet confirmed by instrumentation but
consistent with every symptom: I2C1's NVIC priority (0) is higher than
USART2's (1) (see `stm32l4xx_hal_msp.c`'s `USART2_MspInit 1` comment,
deliberate at the time — telemetry was judged more time-critical than
occasional VCP commands). Under the Pi's high packet rate, the CPU spends
enough time inside the I2C ISR that the UART's single-byte
receive/re-arm cycle occasionally misses a byte before the next one
arrives, corrupting whatever line was mid-transmission. **Not fixed in
firmware** (would mean either raising USART2's priority, which risks
delaying I2C servicing under the exact conditions this was originally
tuned to avoid, or moving to a more robust framing/ack scheme — neither
attempted yet, flagging as open). **Not a safety concern**: corrupted
lines are never silently misinterpreted as a *different* valid command —
`process_command_line`'s exact-match dispatch means a mangled token either
matches nothing (`ERR unknown command`) or, if the front-loaded byte drop
happens to land, occasionally reproducing a truncated form of a real
command name is possible in principle but wasn't observed. Worked around
on the host side instead (see below) — a corrupted line always gets an
`ERR` reply, so simply retrying is safe.

**Fixed `fta_step_response_test.py`** (commit `a1e0725`) to actually target
this firmware — it was still written against the old "FTA Controller"
protocol on two real points, not just the already-known `get_status`
format concern: (1) `get_current_position()`'s regex matched the old
positional `status:x,y,...` reply, updated to this firmware's keyed
`dac_x=N dac_y=N`; (2) `FTA_BAUD` was still `460800` (the old firmware's
rate) against this firmware's actual `115200` — would have produced pure
baud-mismatch garbage, the same failure signature this file already
documented once before (2026-07-23) for the reverse mix-up. Also added a
5-attempt retry around `get_current_position()`'s request/reply, given the
byte-loss finding above — confirmed this actually matters in practice: a
live retry during this same bench session needed 2 attempts before
`set_y 95` landed cleanly. `fta_calibration.py` and
`fta_serial_latency_test.py` almost certainly have the same two
incompatibilities (old status format assumption, `FTA_BAUD` at `460800`)
and haven't been checked yet — do that before running either against this
firmware.

**Corrected same session: the "needs the Pi" framing above was wrong.**
Initially assumed rerunning the step-response test required moving the
Nucleo's USB to the Pi (`fta_step_response_test.py` captures camera frames
directly via Picamera2). User pushback: under the v2 architecture the Pi's
only job is streaming centroids over I2C — already running, already
confirmed live in this bench session — so nothing else needs to happen
there. The missing piece was noticing that `camera_centroid_receiver`
already prints a line for *every* relayed I2C packet over its VCP (the
`seq=... x=... y=...` lines seen throughout this bench session), at the
same rate the Pi streams — a ready-made high-rate position feed over the
exact link already used to send `set_x`/`set_y`. Wrote
`fta_step_response_test_vcp.py`: same `analyze_step` math, but sources
`(t, x, y)` from that relay stream via a background reader thread instead
of `Picamera2.capture_array()`, so it runs entirely from the laptop.
Tradeoff vs. the camera-direct version: time resolution is bounded by the
relay/VCP rate (~176/s measured), not raw camera fps, plus a small
roughly-constant relay latency — fine for rerunning the existing
small-step characterization, not a replacement if sub-frame precision is
ever needed again.

**First real run (`axis x, 95→2000`) measured ~0.06px delta — essentially
no movement**, the same "actuator isn't moving the beam" signature this
file documented once before (2026-07-23, that time root-caused to an
unplugged voice-coil amplifier). The script's own `amp_enable` call
confirmed the PA12 signal went out, but that's as far as software can see
— it can't confirm the physical amp board has power or the actuator is
wired up. Wrote `fta_manual_control.py` (interactive REPL: `x`/`y` jog,
`amp on`/`off`, `status`, `estop`/`clear`) so a human can watch the
actuator directly while driving it by hand, instead of trusting a
scripted pixel-delta measurement for this specific question. Physical
check not yet done as of this entry.

**Second real finding, this time from the user's own interactive session**:
`fta_manual_control.py` immediately corrupted its first two live commands
(`amp_enable`→`amp_eble`, `get_status`→`getstatus`) — worse than the ~1-in-
5-6 rate measured earlier in this same file, up to 100% of the first few
commands in that session. Prompted actually fixing the byte-loss issue
instead of just working around it. **Root cause confirmed and fixed**:
I2C1 was NVIC priority 0 (above USART2's 1) — backwards, given I2C1 has
`NoStretchMode` disabled (so a preempting UART ISR only costs the Pi's
master a few µs of protocol-legal clock stretching) while UART RX has no
equivalent tolerance (no hardware flow control — a missed byte is gone for
good). Swapped priorities in `stm32l4xx_hal_msp.c` (USART2 → 0, I2C1 EV/ER
→ 1), rebuilt, reflashed. **Verified on hardware**: 30/30 commands clean
over the VCP with I2C telemetry actively streaming throughout (`errs=0`),
where the same test previously corrupted roughly 1 in 5-6. Neither NVIC
change lives in the `.ioc` — flagged inline in both spots to reapply by
hand if this project is ever regenerated from it.

**Current state**: firmware flashed with the priority fix, board left idle
(`amp=0 estop=0 dac_x=95 dac_y=95`). Still not done: the physical
amp/actuator check via `fta_manual_control.py` (why the step-response test
measured no movement — signal-reaches-the-pin vs. actuator-actually-moves
is still an open question), and re-running
`fta_step_response_test_vcp.py`'s full original protocol (95→2000, small
100-count steps both directions, both axes) once real movement is
confirmed.

### Architecture DECISION v1, SUPERSEDED 2026-08-04 (see above): Nucleo becomes a dumb I2C-driven external DAC, all control logic stays in Python on the Pi (2026-07-28)

Trigger: `camera_view_tool.py`'s default-on I2C streaming collapsed capture
fps to ~0.9fps (from >100fps) because the physical Nucleo wired to the
Pi's I2C1 header is currently flashed with "FTA Controller" firmware,
which has no I2C slave role at all — every per-frame I2C send was blocking
on the kernel's I2C timeout against a dead link (confirmed: `sudo
i2cdetect -y 1` took >120s and found zero devices, vs. a healthy bus
scanning near-instantly). Immediate fix was `--no-stream`; this section is
the real architectural resolution discussed afterward.

**Re-checked 2026-07-29, same root cause confirmed but the failure signature
is different this time — worth knowing if this gets re-tested again.**
`sudo i2cdetect -y 1` now returns in ~0.03s (not >120s) and still finds zero
devices — a fast, clean "nothing answering" scan, not a hung one. Re-ran the
actual fps comparison (camera 0, only camera enumerating this boot, `1280x800`
full-sensor is `--no-stream`'s default so `--roi 640x200` was used for a fair
apples-to-apples check against streaming's `640x200` default):

| config | fps |
|---|---|
| `--no-stream --roi 640x200` (baseline, matches the validated ~527fps floor) | **520.1fps** |
| default (streaming to the dead I2C link) | **~57-63fps** (climbing slowly: 60.9/s → 57.4/s over ~13,400 frames) |

Still a real, large collapse (~9x, not ~0.9fps-scale total collapse) — same
root cause (Nucleo has no I2C slave firmware loaded right now, still "FTA
Controller"), but each failed send now returns fast (`OSError: [Errno 121]
Remote I/O error`) instead of blocking on a multi-second kernel timeout, so
the cap lands at ~60fps instead of ~1fps. Not chased further why the failure
mode itself changed (kernel/driver state, bus wiring, or something about how
the Nucleo's GPIO pins float under "FTA Controller" firmware vs. whatever
state it was in on 2026-07-28) — not necessary to unblock anything, since the
fix either way is the same already-decided one below (give
`camera_centroid_receiver` DAC output and reflash), not a driver-level chase.
`--no-stream` remains the correct workaround until that firmware work is
done.

**Two firmware-consolidation options were compared, then a third,
better one emerged:**
1. Port I2C-receive into "FTA Controller" (so it gains a fast centroid
   input), with the PI loop running either onboard or still in Python —
   rejected as more complex than necessary: it means merging a real
   command parser + PID framework + grid_scan travel logic into a new
   I2C receive path, and firmware-side PID work doesn't reuse the
   Python-side PI design already sketched above.
2. **Chosen instead (user's idea): flip it — add DAC-output capability TO
   `camera_centroid_receiver` (the I2C firmware), converting the Nucleo
   into a pure external I2C-controlled DAC.** All control logic —
   centroid detection, the calibration matrix, the PI loop itself —
   stays in Python on the Pi, unchanged from the design already sketched
   in the "PI control law designed" section above. The firmware's only
   real-time job becomes: receive a checksummed I2C packet carrying an
   (x, y) DAC setpoint, clamp it to a safe range, ensure amp/driver are
   enabled, and write directly to the DAC1 output channels. No onboard
   PID, no serial ASCII command parsing needed for the real-time control
   path at all.

**Why this is clean**: reuses the I2C slave transport
(`camera_centroid_receiver`'s already-validated register-pointer +
checksum packet convention) instead of building something new: I2C has
lower overhead than USB-serial (no USB stack, no per-command ACK line to
parse) and this project already proved the transport works end-to-end.
Keeping all math in Python means the control loop stays easy to iterate
on (no CubeIDE rebuild-and-reflash cycle just to tune a gain), and it's
the same design already on paper — nothing about the PI/calibration plan
above needs to change, only how the final setpoint gets to the actuator.

**Confirmed via the firmware source (this session, read-only clone of
`ucsbdeepspace/7-element-array` `lock_in_2`) that this is mechanically
plausible**: "FTA Controller"'s own `.ioc`/`main.c` never touch I2C1 or
pins PB6/PB7 at all — no conflict to work around. The actual DAC-write
logic to port over is "FTA Controller"'s own `set_x`/`set_y`,
`amp_enable`/`amp_disable`, `drv_enable`/`drv_disable` functions (see the
"Serial-vs-I2C latency" section above for where these live in `main.c`).

**Real open items, not yet resolved:**
- **Same physical board, one firmware at a time** — moving to this
  architecture means re-flashing back toward a `camera_centroid_receiver`
  descendant (with DAC support added), retiring "FTA Controller" as the
  flashed image again. Nothing about the underlying one-MCU constraint
  changes; this is a different resolution of it than either option
  floated on 2026-07-23, not a way around it.
- **New I2C packet needed.** The existing packet
  (`nucleo_i2c_sender.py`'s `seq/status/x/y/checksum` format) was designed
  for camera→Nucleo *telemetry* (a centroid, for logging). A DAC setpoint
  is a *command*, semantically different — decide on the laptop whether to
  reinterpret the same wire format or add a distinct register-pointer
  value, so the two meanings can't be confused if both roles are ever
  needed at once.
- **`fta_calibration.py` currently depends on "FTA Controller"'s own
  `grid_scan`** for smooth, backlash-aware microstep travel between grid
  points — that capability doesn't exist in `camera_centroid_receiver`.
  Under this architecture, either Python has to replicate smooth
  microstepping itself (many small I2C setpoints instead of one jump), or
  calibration sweeps become direct jumps (simpler, but should be checked
  against the actuator's real step-response behavior above before
  assuming it's fine).
- **`fta_serial_latency_test.py` / `fta_step_response_test.py`** are
  built entirely around the serial link and "FTA Controller"'s
  `get_status`/`cmdq_stats`/`set_x` commands — once that firmware is no
  longer flashed, these scripts stop working against real hardware as-is
  and would need an I2C-based equivalent (or "FTA Controller" needs to
  stay available to reflash back for occasional re-benchmarking).
- **Worth noting explicitly**: an earlier session (2026-07-23, see the
  "Wrong turn, corrected" note in the FTA calibration section above)
  started down a similar-looking path — adding an I2C actuator-command
  packet — and reverted it, but that was because it was based on the
  *wrong reference repo* (an old pyboard-driven rig), before "FTA
  Controller" and its serial protocol were even discovered. This is a
  different, deliberate decision made *with* full knowledge of what "FTA
  Controller" actually does — not the same mistake being repeated.

**Next step**: this is firmware work, needs CubeIDE — **move to the
laptop** for `camera_centroid_receiver`'s actual code changes (add DAC1
init, port the amp/driver-enable + DAC-write logic, define the new
setpoint packet, build, flash, bench-test). This Pi's `7-element-array`
deploy key is deliberately read-only (see the "FTA position calibration"
section above); the laptop is the one with any real push access to that
repo, not independently confirmed this session. Once firmware exists:
extend `nucleo_i2c_sender.py` with a `set_dac(x, y)`-style send method,
and update `fta_calibration.py`/step-response/latency scripts to match.

**Fold `camera_centroid_receiver` into this repo (user-decided, 2026-07-28)
— DONE 2026-08-04.** It lived ONLY as an uncommitted local CubeIDE
workspace on the laptop (see the top-of-file note) until then, a real
data-loss risk. Prepped from the Pi side back on 2026-07-28 (couldn't touch
the laptop's files directly at the time): `.gitignore` got a
`nucleo_firmware/*/{Debug,Release,*.o,...}` block, mirroring the existing
`kernel_patch/` pattern (source/project tracked, per-build output not).
Completed from the laptop 2026-08-04: moved the project folder from
`STM32CubeIDE/workspace_1.14.0/camera_centroid_receiver` into this repo at
`nucleo_firmware/camera_centroid_receiver/`, committed. See "Folded
`camera_centroid_receiver` into this repo" further below for the full
detail, including the still-open follow-up (the laptop's already-running
CubeIDE instance needs re-pointing at the new path).

### Optional pre-reflash cross-check: `fta_closed_loop_fps_test.py` built to measure the REAL combined capture+detect+serial-send loop rate, before the I2C-DAC decision above makes it moot (2026-07-28)

Written in a separate, concurrent session (before that session had seen the
Architecture DECISION above — cross-referenced and reconciled here after the
fact, not proposing an alternative to it). Motivation: the "Serial-vs-I2C
latency estimated" section's "no longer a clear performance argument for
keeping I2C" conclusion was arithmetic on two separately-measured numbers
(camera-only detection latency + serial-only round-trip latency via
`fta_serial_latency_test.py`, never run with a camera in the loop at all) —
not a real combined measurement. That gap is still real and still
unmeasured, but **the DECISION above already supersedes it as the reason to
prefer I2C** — the chosen architecture wasn't purely a latency argument (see
"Why this is clean" above: avoiding a firmware PID merge, no CubeIDE
rebuild-per-gain-tune), so a good serial number wouldn't reverse it.

**Value this still has**: a real, measured "what serial actually achieves"
number is a useful data point to have on record regardless of which link
gets used for real, and — importantly — **this is a closing window**: it can
only be measured while the board still runs "FTA Controller" firmware,
which the DECISION above is about to retire in favor of the I2C-DAC image.
Once that reflash happens, `fta_serial_latency_test.py`,
`fta_step_response_test.py`, and this script all stop working against real
hardware as-is (same point already flagged above). If it doesn't get run
now, getting this number later means reflashing "FTA Controller" back
temporarily just to re-benchmark it.

**Built `fta_closed_loop_fps_test.py`** (committed, pushed to
`origin/master`, **not yet run against real hardware** — written on a
device with no camera/Nucleo attached). Each loop iteration: capture one raw
frame, run `find_beam_blob`, then (per `--send-mode`) send the FTA's own
CURRENT position back over serial for BOTH axes — nothing physically moves
(same non-destructive convention as `fta_serial_latency_test.py`), but it
pays the real 2-axis wire cost every iteration alongside real camera work,
which no prior test did together. Three modes, run separately and compared:
- `none` — capture+detect only, no serial at all (pure camera-side ceiling,
  measured with this exact script so it's apples-to-apples with the other two)
- `wait_ack` (default) — `set_x` then `set_y`, each a full write-then-wait-
  for-ack round trip before the next axis — notably **two sequential round
  trips per loop, not one** (double what the existing single-axis latency
  test measured)
- `fire_and_forget` — both axes written back-to-back with zero waiting, plus
  a `cmdq_stats` drop-counter check to confirm nothing is silently lost at
  the achieved rate

Reports per-stage min/mean/median/p95/p99/max (capture, detect, send, and
total achieved loop period → Hz), beam-detected fraction, and saves raw
per-iteration timing arrays to `results/fta_closed_loop_fps_<mode>_<size>_<UTC
timestamp>.npz`. **One unverified assumption flagged in the docstring**: the
`set_y` ack line is assumed to read `"y_center set to N"`, symmetric to the
confirmed `set_x` ack (`"x_center set to N"`) — `fta_serial_latency_test.py`
only ever validated the x ack. If this assumption is wrong, `wait_ack` mode
will fail LOUDLY (near-zero successful sends in the report), not silently
mismeasure, since a sample only counts on an actual regex match.

**If there's time before the reflash, run this on the Pi (camera + "FTA
Controller" Nucleo still attached) — otherwise skip straight to the DECISION
section's next step (laptop firmware work) without regret:**
```
cd ~/rpi_camera_system   # or wherever this repo is cloned on the Pi -- confirm path
git pull
python3 fta_closed_loop_fps_test.py --send-mode none        --raw-size 640x100 --y-start N
python3 fta_closed_loop_fps_test.py --send-mode wait_ack    --raw-size 640x100 --y-start N
python3 fta_closed_loop_fps_test.py --send-mode fire_and_forget --raw-size 640x100 --y-start N
```
Replace `--y-start N` with whatever real sensor row currently brackets the
beam for `MODE_640_100_ROI`'s 100-row window (this session's `--dry-run`
found the beam near row 32 at full-sensor 1280x800 y_start=0 — that maps to
roughly y_start≈0 for a 100-row window too, but confirm live via
`roi_live_demo.py` rather than assuming). If unsure, start with `--y-start 0`
and check the printed "Beam detected: N/N" fraction — a low fraction means
the window doesn't currently bracket the beam, not that the script is
broken. Note this session also found `camera_view_tool.py`'s default-on I2C
streaming currently HANGS for ~120s against this same "FTA Controller"
firmware (no I2C slave role) — irrelevant to this script (it doesn't use
`nucleo_i2c_sender.py` at all), but don't run `camera_view_tool.py` with
default streaming enabled on this same board in the meantime.

**What to do with the results**: just record the three achieved-loop-rate Hz
numbers (from each run's "Achieved loop period" line) here in this file for
the record. This does NOT gate the next step — proceed to the DECISION
section's laptop firmware work regardless of what these numbers show, unless
they reveal something surprising enough to reopen the architecture question
(e.g. serial turning out to have some other problem beyond raw speed).

## IN PROGRESS: streaming beam position to an STM32 Nucleo over I2C — camera_view_tool.py now streams real centroids by default, sub-pixel precision fix needs a matching firmware update, live end-to-end not yet reconfirmed (2026-07-21)

First step past pure characterization: closing the loop out to an external
controller. **Pi-side hardware now exists**: a NUCLEO-L432KC board and a
laptop (for flashing firmware via STM32CubeIDE) are available as of
2026-07-16 — the earlier "no Nucleo in this session" blocker is gone, but
firmware work is expected to happen from the laptop, not this Pi (no
CubeIDE here). Two Pi-side scripts, both committed but still not run
against the real board (that validation is the very next step, likely
easiest done once someone is at the laptop + Nucleo together):

- **`beam_position_streamer.py`** — headless companion to
  `camera_view_tool.py`: capture → detect beam centroid → send, no display,
  no GTK. Reuses `roi_set_selection.py`'s `apply_y_start`-retry pattern.
  Beam detection (`find_beam_blob`) is intentionally duplicated from
  `camera_view_tool.py` rather than imported (that script has no
  `__main__` guard, so importing it would launch its live viewer) — if the
  confidence-gate constants are retuned in one script, mirror the change
  in the other. Measured live on the bench (no display, no ROI writes):
  **~335fps at 640x200**, no artificial throttle needed — faster than
  `camera_view_tool.py`'s auto-track mode, which was dominated by its
  subprocess-based ROI-repositioning cost rather than detection itself.
  Coordinates sent are real, absolute full-sensor pixels (0-1280/0-800),
  scaled by each mode's binning ratio, so the Nucleo doesn't need to know
  which ROI window is active. `--dry-run` mode (detect + print, skip the
  actual I2C send) **is validated live**; the real-send path is not.
- **`nucleo_i2c_sender.py`** — the I2C sender: Pi as master, Nucleo as
  slave (Pi's I2C controllers have weak/awkward slave-mode support, so
  this direction was chosen deliberately). Small fixed register-mapped
  packet mirroring the OV9281 driver's own "register pointer + data"
  convention: `seq` (u8, wraps, lets the Nucleo detect a stale link),
  `status` (u8, bit0 = beam confidently detected this cycle), `x`/`y`
  (s16 each), `checksum` (u8, additive sum mod 256 — catches link noise
  the per-byte I2C ACK wouldn't). `valid=False` still sends a packet
  (last-known position) rather than going silent, so "beam lost" is an
  explicit signal, not something the Nucleo can only infer after a
  timeout. Uses `smbus2` directly, not a `v4l2-ctl` subprocess (that
  subprocess overhead, ~7-10ms/call elsewhere in this project, is fine for
  an occasional ROI move but not a per-frame tracking send).

**Header I2C bus — RESOLVED, already enabled, no reboot needed
(2026-07-15).** The earlier note here ("no header I2C bus is enabled
yet") turned out to be stale — `dtparam=i2c_arm=on` was already
uncommented (top-of-file, global scope) in `/boot/firmware/config.txt`,
and the bus is live *right now*: `/dev/i2c-1` exists, backed by RP1's
`i2c@74000` controller (confirmed via
`/proc/device-tree/aliases/i2c1`), and `sudo i2cdetect -y 1` scans clean
(responds, no devices — expected with no Nucleo wired up). Despite RP1
renumbering the camera buses to i2c-10/11, the header bus kept the
classic `i2c-1` number. `NUCLEO_I2C_BUS = 1` in `nucleo_i2c_sender.py` is
now confirmed correct, not a placeholder — updated in-file.
`NUCLEO_I2C_ADDR` is still a placeholder pending real Nucleo firmware.

**Next steps (2026-07-16)**, expected to happen from the laptop side using
the NUCLEO-L432KC + STM32CubeIDE:
1. **Write the Nucleo firmware** — does not exist yet at all, this is the
   real blocker. Needs: CubeMX I2C1 config as a **slave** (not the
   default master template), a chosen 7-bit slave address (update
   `NUCLEO_I2C_ADDR` in `nucleo_i2c_sender.py` on the Pi side to match —
   currently placeholder `0x42`), and a receive handler that parses the
   fixed 7-byte packet defined in `nucleo_i2c_sender.py`'s docstring
   (`seq, status, x_lo, x_hi, y_lo, y_hi, checksum` after the register-
   pointer byte — `x`/`y` are little-endian s16, checksum is an additive
   sum of the 6 data bytes mod 256) and verifies the checksum before
   trusting a packet.
2. **Wire it up**: Nucleo I2C slave pins (L432KC default I2C1: PB6=SCL,
   PB7=SDA, but confirm against whatever CubeMX pinout is actually used)
   to the Pi's header I2C1 (physical pins 5=SCL/GPIO3, 3=SDA/GPIO2), plus
   a shared ground. Bus is already confirmed live on the Pi side at
   `/dev/i2c-1` (see below) — no Pi-side config left to do.
3. **Smoke-test the link without the camera**: run `nucleo_i2c_sender.py`
   directly on the Pi (its `__main__` sends a slowly-orbiting fake
   position) and confirm the Nucleo firmware receives sane, checksum-valid
   values — e.g. blink an LED or print over USB-serial back to the laptop.
4. **Only then** run `beam_position_streamer.py` for real — `--dry-run`
   first to reconfirm detection still works on whatever hardware state
   the cameras are in, then live against the Nucleo.

### Nucleo firmware built, wiring done, end-to-end validated (2026-07-21)

Steps 1-3 above are now done and confirmed live (step 4, the real
camera-driven streamer, is still not started — only the fake-orbit smoke
test in step 3 has been run). Built via STM32CubeIDE on the laptop,
project `camera_centroid_receiver` (at the time, not part of this git
repo — folded in 2026-08-04, see "Folded `camera_centroid_receiver` into
this repo" further below; now at `nucleo_firmware/camera_centroid_receiver/`).

**CubeMX config**: I2C1 as slave, PB6/PB7 (SCL/SDA, matches
`nucleo_i2c_sender.py`'s expectation), address `0x42`, NVIC event+error
interrupts enabled, clock stretching allowed (`NoStretchMode` disabled).
USART2 for the VCP debug print: PA2=TX, PA15=RX (PA15 is silkscreen-
labeled `PA15 (JTDI)` — a JTAG debug pin by default; reassigning it to
USART2_RX required giving up the JTAG debug pins, harmless since this
Nucleo's ST-Link uses SWD, not full JTAG).

**Three real bugs found and fixed during bring-up, worth remembering:**

1. **I2C address field expects the raw address, not pre-shifted.**
   CubeMX's "Primary Slave Address" field (I2C1 Parameter Settings) wants
   the plain 7-bit address (`0x42`) — entering the pre-shifted value
   (`0x84`) gets silently rejected/reset to 0 by the GUI's own validation.
   CubeMX then correctly emits the shifted value itself
   (`hi2c1.Init.OwnAddress1 = 132`, i.e. `0x84`) into generated code —
   confirmed by inspecting `main.c`. This matters because STM32's
   `HAL_I2C_Init()` writes `OwnAddress1` straight into the `OAR1` register
   with no shift of its own (bit 0 is reserved in 7-bit mode), so *some*
   layer has to apply the `<<1` — it's CubeMX's generator, not HAL, and
   the GUI field expects un-shifted input. Get this backwards and the
   slave silently never ACKs the right address.
2. **Setting a peripheral's pins in the Pinout view is not the same as
   enabling the peripheral.** Trying to fix the PA15 pin assignment by
   clicking pins directly left USART2 pin *labels* in place
   (`PA2.Signal=USART2_TX` etc.) while the peripheral's own `Mode` was
   unset and it dropped out of `Mcu.IP` entirely — silently deleting
   `huart2`/`MX_USART2_UART_Init()` from generated code (build error:
   `'huart2' undeclared`). Made worse by a stray USB device peripheral
   (PA11/PA12) getting enabled at the same time, apparently from a
   misclick during the same pin-reassignment attempt. Fix: always set
   `Mode` on the peripheral itself in the left-hand **Connectivity** list
   (e.g. USART2 → Asynchronous), not just by clicking pins in the diagram.
3. **CubeIDE's Debug launch halts at program entry until Resume is
   pressed.** After a clean flash, zero heartbeat output looked identical
   to a hang — turned out to just be the debugger paused right after
   reset. Use **Run** (not Debug) for straightforward flash-and-observe
   testing, or remember to hit Resume.

**Firmware behavior**: `HAL_I2C_Slave_Receive_IT` arms an 8-byte one-shot
reception matching `nucleo_i2c_sender.py`'s exact wire format
(`reg_ptr, seq, status, x_lo, x_hi, y_lo, y_hi, checksum`).
`HAL_I2C_SlaveRxCpltCallback` validates the additive checksum, drops the
packet silently on mismatch (counted in `g_checksum_error_count`, never
trusted into `g_latest_beam`), and re-arms for the next transaction;
`HAL_I2C_ErrorCallback` also re-arms so a NACK/bus glitch can't leave the
slave stuck waiting forever. PB3 (LD3, per the L432KC user manual —
**not independently hardware-verified**, board uses a bare MCU selection
not a board file) toggles on every valid packet. A free-running 1Hz
heartbeat print (`heartbeat uptime=Ns pkts=N errs=N`), independent of any
I2C activity, was added specifically to separate "is the firmware/VCP
link alive at all" from "is I2C actually working" while debugging — kept
in place since it's cheap and useful. The ISR itself stays short (just
parses, checksums, sets a flag); the actual VCP prints happen from the
main loop polling that flag, avoiding a blocking UART call from interrupt
context.

**Physical wiring — done**: Pi header physical pin 5 (SCL/GPIO3) ↔ Nucleo
`D5` (PB6/SCL); Pi physical pin 3 (SDA/GPIO2) ↔ Nucleo `D4` (PB7/SDA); a
Pi GND pin ↔ a Nucleo GND pin. No level shifter or external pull-ups
needed (3.3V both sides, Pi header already has pull-ups).

**Laptop-side tooling**: `nucleo_serial_monitor.py` (this repo) opens the
Nucleo's ST-Link VCP port (auto-detected via `pyserial`'s `list_ports`,
matching on "STLink"/"STMicroelectronics" in the USB description),
timestamps each line, and prints an idle heartbeat (`... no data received
in Ns`) if nothing arrives for a while so a quiet bus is distinguishable
from a hung script. Needs `pyserial` installed into *whichever* Python
interpreter actually runs it — on this laptop that tripped a
`ModuleNotFoundError` once because `pip install pyserial` and the VS Code
run button resolved to different Python installs (3.11 vs. the Windows
Store 3.10 vs. an Anaconda env); fixed via `py -3.11 -m pip install
pyserial`. Worth remembering if this bites again on another machine.

**End-to-end validation — CONFIRMED (2026-07-21).** With
`nucleo_i2c_sender.py` running on the Pi and `nucleo_serial_monitor.py`
watching the laptop: `seq` incremented 1:1 with `pkts`, `status=1`
throughout, `x`/`y` smoothly traced the sender's sine/cosine orbit, and
`errs` stayed at 0 for hundreds of packets. Real, clean round trip.

**Next step**: run `beam_position_streamer.py` for real (item 4 in the
numbered list above) — `--dry-run` first to reconfirm beam detection
still works on whatever hardware state the cameras are in, then live
against the Nucleo. Not started yet — superseded in practice by
`camera_view_tool.py` gaining its own streaming path, below, which is what
actually got exercised first.

### `camera_view_tool.py` gains built-in full-speed I2C streaming; sub-pixel centroid precision fixed (2026-07-21)

Prompted by a real user session: running `camera_view_tool.py` against the
wired-up Nucleo and seeing nothing arrive on `nucleo_serial_monitor.py` —
turned out streaming was never wired into that script at all, only into
`beam_position_streamer.py` (which had never actually been run for real).
Rather than chase that one gap, gave `camera_view_tool.py` — the tool
actually being used on the bench — its own streaming path directly,
independent of `beam_position_streamer.py`:

- **Full-speed streaming, decoupled from the display.** Beam detection
  normally throttles to ~20Hz (`ANALYSIS_INTERVAL_S`) to save fps for a
  merely-displayed camera; for whichever camera is streaming, that
  throttle is bypassed entirely — `find_beam_blob` and the I2C send run on
  every captured frame (the same ~339fps cost `beam_position_streamer.py`
  already pays and accepts), while the on-screen view still only redraws
  at ~15Hz (`DISPLAY_INTERVAL_S`). Auto-track recentering (if `t` is also
  on) stays gated at the slower ~20Hz cadence regardless, since its real
  cost is the `set_roi_y_start` subprocess call, not detection —
  recentering on every streamed frame would reintroduce the original
  auto-track-at-50Hz fps-collapse bug documented elsewhere in this file.
- **Streaming is ON BY DEFAULT (camera 0)**, no flag needed — first cut
  made it opt-in (`--stream`), then flipped to default-on after exactly
  the "why isn't anything arriving" confusion above. `--no-stream` goes
  back to pure bench-viewer mode (no `NucleoLink`/`smbus2` touched at
  all); `--stream-cam N` picks a different camera; `--dry-run` computes
  and reports without actually sending.
- **A missing/unresponsive Nucleo is now a warning, not a crash.** Both
  opening the link and each individual send are wrapped: a failed
  `NucleoLink()` construction, or a per-frame `OSError` (e.g. the
  `TimeoutError` smbus2 raises on a dead/unwired bus — the exact symptom
  hit earlier this session during Nucleo bring-up), prints `WARNING: I2C
  send to Nucleo failed (...)` with a running failure count and keeps the
  viewer running — detection/display are unaffected, only the send itself
  is best-effort. Worth knowing: a failing send blocks on the kernel's I2C
  timeout, so a dead link will also visibly cap capture fps until it
  recovers, not just silently drop sends.
- **Fixed a real precision bug: sub-pixel centroids were being thrown away
  before they reached the wire.** `find_beam_blob` computes an
  intensity-weighted (sub-pixel) centroid, but both `camera_view_tool.py`
  and `beam_position_streamer.py` were rounding to a whole pixel *before*
  calling `NucleoLink.send_position` — the fractional part never had a
  chance to be sent. Fixed by moving the rounding into
  `NucleoLink.send_position` itself: it now takes real (float) pixel
  coordinates and scales by a new `POSITION_SCALE = 10` before packing
  into the existing `s16` field — no packet growth, no float-on-the-wire
  endianness/alignment fragility (max real coordinate 1280/800 scales to
  ±12800/±8000, comfortably inside `s16`'s ±32767 ceiling). Verified the
  encode/decode math directly (mocking the I2C write, no hardware
  involved): `585.37` sent → decodes back to `585.4`.
  **The Nucleo firmware (outside this repo, not yet updated) must divide
  received x/y by `POSITION_SCALE` (10) to recover real pixels — this is a
  wire-format change, not backward compatible with the firmware described
  above as-is.**
- Committed and pushed to `origin/master`: `c67013e` (opt-in `--stream`),
  `7c60805` (default-on + failure resilience + `POSITION_SCALE` fix).
- **Not yet confirmed**: no live report of `camera_view_tool.py`
  successfully streaming real detected positions to the Nucleo and being
  seen on `nucleo_serial_monitor.py`. The default-on flip addresses the
  original "nothing arrives" symptom, but that fix plus the
  `POSITION_SCALE` change together haven't been confirmed live against the
  real Nucleo — and can't be, fully, until the firmware's own divide-by-10
  update is made on the laptop side.

## RESOLVED (practically, not root-caused to a fix): unbinned ROI modes invert bright point sources — root cause pinpointed, use binned modes instead (2026-07-15)

**Skip to "Practical resolution" below for the bottom line: use
`MODE_640_200_ROI`/`MODE_640_100_ROI` (binned), not the unbinned ROI modes,
for all real work.** The mechanism is now understood (a continuously-active
internal sensor auto-calibration engine, triggered by locked frame duration
near the unbinned windowed-crop modes' rated ceiling) but is not fixable via
register override — see "Root cause pinpointed" and "Practical resolution"
below for the full investigation. Original bug report preserved below for
history.

**While using `camera_view_tool.py` on the bench against a real laser beam
(through a beamsplitter), the user noticed every mode except full-sensor
looked visibly inverted and noisier.** Confirmed and isolated with a direct
diagnostic (captured full-sensor + each ROI mode, both at `y_start=0` and
at a `y_start` moved to bracket the beam's known row, cam 0 only —
`i2c@88000`/cam 1 unavailable this session, see "Hardware status"):

- **`MODE_640_200_ROI` and `MODE_640_100_ROI` (binned, 640-wide) are
  clean** even with `y_start` moved onto the beam: the beam shows up
  correctly as a bright peak (max 65280 / 60416, matching the full-sensor
  peak scale) at the column its binned (2:1) position predicts (~col
  426-427, i.e. full-sensor col ~852 halved) — no inversion, background
  stays at a normal, flat level.
- **`MODE_1280_400_ROI` and `MODE_1280_200_ROI` (unbinned, 1280-wide) show
  a real inversion once the window is moved onto the beam's actual row**:
  the column where the beam physically sits (confirmed via full-sensor
  capture: peak at row 409, col 851) becomes the **darkest** feature in the
  frame instead of the brightest — e.g. `1280x200` at `y_start=288`: that
  column's mean drops to 448 while the surrounding background sits around
  4057. At `y_start=0` (beam not in that window) no such feature appears,
  so this needed a moved window bracketing the real beam row to catch —
  the earlier per-mode "frame content validated clean" checks (see the
  `MODE_1280_400_ROI` and quarter-tier sections below) used gradient/room
  scenes and LED blinks, never a genuinely saturating focused point
  source, which is almost certainly why this was never caught until now.
  Background noise (`std`) is also elevated relative to a true flat
  background in these two modes, matching the user's "noisier" observation.
- **Root cause not yet found** — prime suspects are the ISP registers
  already flagged as "copied verbatim from stock `1280x720`, unverified"
  when these two modes were first added (`TIMING_FORMAT_1/2`,
  `0x4008/0x4009`, `0x400c/0x400d`, `0x4507/0x4509` — see the
  `MODE_1280_400_ROI` section below), most likely a black-level-clamp or
  defect-pixel-correction register misbehaving on a genuinely saturating
  highlight in the unbinned (not binned) readout path specifically. Not
  yet root-caused to a specific register or fixed.

**Practical implication until this is fixed**: prefer `MODE_640_200_ROI` /
`MODE_640_100_ROI` (binned) over `MODE_1280_400_ROI` / `MODE_1280_200_ROI`
(unbinned) for any real beam-tracking work — the binned modes are
confirmed clean against an actual bright point source, the unbinned ones
are not. All four modes' throughput/closed-loop numbers documented
elsewhere in this file are still accurate as *timing* measurements (LED
on/off transitions aren't affected by this pixel-value inversion), but the
unbinned modes' "frame content validated clean" claims should be
considered superseded by this finding until root-caused.

Diagnostic scripts/frames are in the scratchpad, not committed
(`diag_inversion.py`/`diag_inversion2.py`, `inv_*.npy`) — rerun against
cam 0 if re-investigating; cam 1 not yet checked at all for this issue.

### Attempted fix #1 (`TIMING_FORMAT_2` re-latch) — TRIED, CONFIRMED NOT SUFFICIENT (2026-07-15)

Working theory was a stale internal calibration (auto black-level or similar)
keyed to the window's position at last full mode-select. `ov9282_apply_roi_y_start()`
was changed (`kernel_patch/ov9282/ov9282.c`, **still uncommitted**) to
read-modify-write `OV9282_REG_TIMING_FORMAT_2` (unchanged value, just
re-latched) immediately after every `y_start`/`y_end` write, on both the
initial-stream path and the live mid-stream `set_selection` path. This was
based on a manual raw-i2c bisection (bypassing the driver, against an
already-running stream) that appeared to clear the inversion by rewriting
that one register.

Built, vermagic/srcversion-matched, depmod-installed, **rebooted in**.
`tainted` stayed `4096` throughout, no dmesg BUG/Oops.

**Result: does NOT fix it.** User re-tested live with `camera_view_tool.py`
(which only ever exercises the two unbinned 1280-wide modes via its `h`
cycle — it has no binned-mode option at all, so "all ROI sizes" in that
tool *are* the two known-bad modes) and still saw the inversion. Verified
independently and quantitatively (headless, `i2c@80000` — the only camera
enumerating this boot, `i2c@88000` failed to probe again, see "Hardware
status"): with the exact settled-config controls (`ExposureTime=1500`,
`AnalogueGain=4.0`, `AeEnable=False`, `NoiseReductionMode=0` — **matching
these exactly turned out to matter**, see caveat below), captured full
sensor (peak: row 389, col 852, max 49920), then centered `1280x400`
(`y_start=188`) and `1280x200` (`y_start=288`) on that row:

| mode | y_start | col 852 mean | frame median | frame max |
|---|---|---|---|---|
| `1280x400` | 188 | 2368 | 3840 | 5632 |
| `1280x200` | 288 | 938 | 3840 | 5632 |

Both **still inverted** (the beam column reads darker than background) —
essentially the same signature as the original finding (`y_start=288`
matches the original report exactly; col 852 vs. the original's col 851).
Fix confirmed ineffective, not just "unconfirmed."

**Methodology caveat, worth remembering for next attempt**: an initial
version of this same headless recheck left camera controls at their
Picamera2 defaults (auto-exposure on) instead of the settled
`ExposureTime`/`AnalogueGain`/`AeEnable=False` config, and came back
"bright (ok)" for both modes — a false negative. Only after matching the
exact controls `camera_view_tool.py` actually uses did the inversion
reproduce. **Any future recheck of this bug must fix exposure/gain/AE
exactly as above** — auto-exposure alone is enough to mask it.

**Not yet tried** (superseded by the root-cause reframing below, see
"Attempt #2" and "Root cause reframed" — kept here for the historical
record of what attempt #1 left open): the other flagged-unverified registers
(`TIMING_FORMAT_1`, `0x4008/0x4009`, `0x400c/0x400d`, `0x4507/0x4509`);
an edge-triggered write (0 then back to original, rather than a same-value
rewrite) in case whatever internal routine needs a transition, not just a
rewrite; and re-confirming the original manual raw-i2c bisection that
seemed to implicate `TIMING_FORMAT_2` in the first place, in case that was
a confound rather than a real signal. A working, fast, quantitative
headless repro now exists (this recheck script, scratchpad-only, rebuild
per the pattern above) — future bisection attempts should use it instead
of relying on live visual inspection through `camera_view_tool.py`, and
must include the exact control set noted above. `i2c@88000` (the camera
the original bug was found on) has still never been tested against this
fix at all — only `i2c@80000` has, since that's the only camera that
enumerated this boot.

### Attempted fix #2 (`TIMING_FORMAT_1` re-latch, isolated from `_2`) — TRIED, CONFIRMED NOT SUFFICIENT (2026-07-15)

Found already implemented in the working tree at the start of this session
(uncommitted, `kernel_patch/ov9282/ov9282.c` ~line 1299,
`ov9282_apply_roi_y_start()`): replaced the `TIMING_FORMAT_2` relatch from
attempt #1 with a read-modify-write of `OV9282_REG_TIMING_FORMAT_1` instead,
same call site, same "re-latch after every y_start/y_end write" idea. Built,
vermagic/srcversion-matched, depmod-installed, **rebooted in** (this is the
build that was live when this session started). `tainted` stayed `4096`
throughout, no dmesg BUG/Oops. `i2c@88000` (the camera the bug was
originally found on) still fails to probe this boot (`fail to write
MIPI_CTRL00` at ~3.3s in dmesg) — this fix has still never been tested
against that camera, only `i2c@80000`.

**User re-tested live with `camera_view_tool.py` after rebooting and still
saw the inversion.** Initial quantitative headless rechecks (single capture
1-2 frames after a pre-stream or mid-stream `y_start` move, same pattern as
attempt #1's recheck) came back clean (bright, not inverted) on both
`1280x400` and `1280x200` — an apparent contradiction with the live
observation. Screenshotting the actual live `camera_view_tool.py` process
(launched on the real display, driven with `xdotool`, same method used to
validate `roi_live_demo.py`) resolved it: the live tool's window showed a
clear dark blob (the beam, inverted) at `1280x400`/`y_start=200` — the bug
was real and reproducing, the headless single-frame recheck was just not
long-running enough to catch it. See "Root cause reframed" below for what
actually explains the discrepancy.

### Root cause reframed: NOT about `y_start`/window position at all (2026-07-15)

Both attempted fixes were built on the theory that something is "stale,
keyed to the window's position at last full mode-select" — i.e. that
*moving* the crop is what breaks the calibration. **That theory is now
disproven.** Isolating the discrepancy between the clean single-frame
headless recheck and the inverted live-tool screenshot (both against the
identical attempt-#2-patched build, camera 0/`i2c@80000`) found:

- The inversion is **not immediate** — it's a transient that develops over
  a handful of frames. Capturing continuously after a mode/ROI change:
  frames 0-4 read correctly bright (matching the full-sensor peak), then by
  frame ~5 the signal collapses to near-background and **stays collapsed**
  for as long as capture continues (tested to 40 consecutive frames).
- **The trigger is a locked `FrameDurationLimits` control** (`(N, N)`,
  i.e. min==max), not the ROI move. Isolated with paired tests (same mode,
  same beam, only this one control toggled): unbinned windowed-crop modes
  stay clean indefinitely (tested to 60 consecutive frames) with
  `FrameDurationLimits` left unset (free-running), and reliably collapse by
  frame ~5 with it locked to the mode's own validated floor duration
  (`3400µs` for `1280x400`, matching what `camera_view_tool.py` and every
  throughput/closed-loop test script in this project actually sets). The
  presence/absence of a second (`main`) ISP-processed stream alongside the
  raw stream made no difference either way — ruled out as a factor.
- **Moving `y_start` is not required to trigger it at all.** Confirmed
  directly: `1280x400` left at its untouched compile-time-default
  `y_start=0` (which already brackets the bench beam's actual row, ~386-389,
  since 0 < 400) — no `set_selection` call made, ever — still collapses by
  frame ~5 once `FrameDurationLimits` is locked (35/40 frames inverted in
  one run). This is the same signature, same magnitude, as every
  `y_start`-moved case previously documented. The entire "keyed to last
  mode-select position" framing behind both attempted fixes was addressing
  a mechanism that isn't actually what's happening.
- **Confirmed specific to the windowed-crop unbinned modes, not "any
  unbinned mode."** The stock, uncropped `1280x800` mode (fully unbinned,
  full sensor height, no shrunk-window register set) does **not** collapse
  even after ~2s/hundreds of frames with `FrameDurationLimits` locked —
  ruling out "unbinned readout in general" as the trigger. This points back
  at something specific to the windowed-crop register set shared by
  `MODE_1280_400_ROI`/`MODE_1280_200_ROI` (the same registers flagged
  "copied verbatim from `1280x720`, unverified" when these modes were first
  added), not at unbinned readout as a general concept.
- **Binned modes remain genuinely clean, now under a much more sustained
  check than before.** `MODE_640_200_ROI` at its own validated floor
  (`1800µs`, locked `FrameDurationLimits`) stayed bright/correct across 60
  consecutive frames, 0 inverted — the existing "prefer binned modes"
  guidance holds and is now validated against sustained running, not just
  the original single-frame-after-a-move check.
- Attempt #2's `TIMING_FORMAT_1` relatch does **not** prevent this — it was
  active (installed, loaded) throughout every test above that showed the
  collapse.

**Practical implication, updated**: the "prefer binned modes" guidance from
the original bug-found note stands, now on stronger evidence. For anyone
touching the unbinned ROI modes: the failure mode is specifically "run it
for more than a few frames at a locked/fixed frame duration" — a quick
single-frame sanity check (as most of this project's earlier "frame content
validated clean" checks were) will **not** catch this; a sustained
multi-frame check with `FrameDurationLimits` actually locked is required to
see it at all. Since essentially every real usage in this project (any
throughput sweep, any closed-loop test, `camera_view_tool.py`,
`roi_live_demo.py`) locks `FrameDurationLimits` to hit a target fps and runs
for much longer than 5 frames, **this bug has almost certainly been present
in every unbinned-ROI-mode run in this project's history**, silently, just
never caught because those tests only check LED on/off timing deltas, never
absolute frame content.

### Root cause pinpointed via register-diff diagnostic — mechanism identified, NOT fixable by register override (2026-07-15)

Rather than continue guessing which register to rewrite (both prior
attempts did exactly that and failed), built a diagnostic that reads the
sensor's own register values via raw i2c (bypassing the driver, `smbus2`
burst reads on bus 11 addr 0x60 — `i2c@80000`) before and after the
collapse, and diffs them. Any register that changes value on its own is
either the mechanism or directly downstream of it — this is real signal, not
a guess.

**Methodology pitfall found and fixed first**: an initial version of this
diff was contaminated — the "before" snapshot was taken too soon after
`cam.start()`/reconfigure, catching the sensor still mid mode-transition
(the already-documented "`cam.start()` can return before the driver
finishes settling into the new pad format" quirk, same one
`roi_live_demo.py`'s `apply_y_start` retry-loop works around). That first
diff showed ~22 "changed" registers, almost all of which were just old-mode
vs. new-mode differences, not genuine runtime drift. Fixed by polling
`0x380a`/`0x380b` (output height) until it actually read back the new
mode's value before trusting the "before" dump. With that fix:

- **None of the previously-flagged registers actually drift.**
  `TIMING_FORMAT_1`/`_2`, `0x4008/0x4009`, `0x400c/0x400d`, `0x4507/0x4509`
  hold constant between the genuinely-settled "before" state and the
  collapsed "after" state — continuing to bisect that list would have been
  chasing artifacts of the settling bug, not the real mechanism. (They also
  never appeared as real drift once decontaminated, which is consistent
  with the two full attempts of re-latching `TIMING_FORMAT_2`/`_1` doing
  nothing.)
- **The real drift**: `0x380e`/`0x380f` (VTS, frame length) drops from
  `0x038e` (910, matching the mode's native/rated-max-fps value) to `0x01bc`
  (444) partway through streaming — consistent with `FrameDurationLimits`
  landing a few frames after `start()` (normal libcamera control-queueing
  lag) and the driver/ISP recomputing VTS to match. `TIMING_FORMAT_1`/`_2`
  and `0x3830`/`0x3831` change alongside it, most likely as an automatic
  side effect of VTS changing (same bit pattern shift in both timing-format
  registers), not independently significant.
- **The actual smoking gun**: `0x4061, 0x4063, 0x4065, 0x4067, 0x4069,
  0x406b, 0x406d, 0x406f` (and `0x4073`) go from all-zero to populated with
  real, non-zero values at exactly the same moment the image collapses —
  the signature of an internal auto-calibration engine (black-level or
  defect-pixel correction, matching the sensor's own product-brief language)
  computing and writing its output for the first time.

**Tested whether this is fixable by direct override — it is not**:

- **`0x4061` is read-only in practice.** Wrote `0x00` to it while streaming;
  the *immediate* readback (before another frame could even elapse) already
  showed `0x40` again — confirmed this isn't a timing artifact by also
  checking one frame later (still `0x40`). The sensor's internal engine is
  continuously re-driving this register's value; it is a live status/output
  register, not a configuration register writable via i2c poke.
- **`0x380e`/`0x380f` (VTS) writes DO land and hold** (confirmed: write
  `0x038e`, immediate and one-frame-later readback both `0x038e`) — but
  forcibly pinning VTS to its native value does **not** prevent the
  collapse (tested standalone and combined with zeroing the correction
  block; image still collapsed at frame ~5 in every combination). VTS
  changing is a correlated symptom, not the lever.
- Net: the mechanism is a genuine, continuously-active internal
  auto-calibration engine specific to the windowed-crop unbinned register
  set, triggered by running near the mode's rated frame-duration ceiling,
  and its output cannot be overridden from outside — there would need to be
  a separate enable/disable bit for the engine itself, and nothing in the
  scanned register blocks (`0x3800-0x38ff`, `0x4000-0x40ff`,
  `0x4500-0x45ff`) is it. Finding such a bit with no datasheet would mean an
  untargeted scan of the sensor's full register space with no more specific
  leads than "somewhere else, unknown" — materially worse odds than the
  targeted search just completed. **This is the point of diminishing
  returns for a register-level fix**, absent a real OmniVision datasheet or
  vendor contact.

### Practical resolution: safe-duration envelope exists but isn't competitive with binned modes — use binned modes (2026-07-15)

Since the trigger is specifically "locked frame duration near the mode's
rated ceiling," swept `MODE_1280_200_ROI` (the only unbinned ROI mode that
was ever a genuine speed win — `MODE_1280_400_ROI` was already documented
as no faster than stock) across a range of locked durations, checking 20
consecutive frames for inversion at each:

| duration | rated fps | 20-frame result |
|---|---|---|
| 1775µs (documented floor) | 563.4 | **collapsed from frame 0** (20/20) |
| 2000µs | 500.0 | clean |
| 2200-6000µs | 454.5-166.7 | clean |

So a genuinely safe operating point does exist (≥2000µs) — but it's not
useful in practice: real measured throughput at 2000µs (`camera_throughput_test.py`,
solo) is **478.26fps**, *slower* than the already-clean, already-validated
`MODE_640_200_ROI` (binned) at its own floor (**~527-531fps achieved**,
1800µs). The entire value proposition of the unbinned quarter-tier mode —
being faster than the binned alternative — evaporates once it's restricted
to a duration slow enough to avoid this bug. There is no setting where
`MODE_1280_200_ROI` is simultaneously bug-free and faster than
`MODE_640_200_ROI`.

**Verdict**: don't chase a safe-but-competitive unbinned setting further.
Use the binned modes (`MODE_640_200_ROI`, `MODE_640_100_ROI`) for all real
beam-tracking work — they're faster *and* clean, with no caveats needed.
The unbinned windowed-crop modes (`MODE_1280_400_ROI`, `MODE_1280_200_ROI`)
should be treated as validated-broken-at-useful-speeds and not used for
real work; kept in the driver for reference/future investigation only.

Diagnostic scripts from this session (register-diff, intervention tests,
duration sweeps) are scratchpad-only, not committed — rerun against camera
0 (`i2c@80000`) if re-investigating; `i2c@88000` still not tested at all
(fails to probe this boot).

## DONE: quarter-tier ROI modes (`MODE_1280_200_ROI`, `MODE_640_100_ROI`) — validated, NOT YET COMMITTED (2026-07-14)

Pushed the `y_end = height + 15` windowing trick one step past the existing
400-real-row ROI modes: two new modes added to `supported_modes[]` in
`kernel_patch/ov9282/ov9282.c`, each cloned register-for-register from its
400-row sibling with only `y_end` (`0x3806/0x3807`) and `y_output_size`
(`0x380a/0x380b`) changed, same "unverified until checked against a captured
frame" caveat as every mode added this way so far:

- `MODE_1280_200_ROI` — unbinned, 200 real rows, rated 542.59fps
- `MODE_640_100_ROI` — binned, 200 real pre-bin rows → 100 output rows,
  rated 1071.81fps

`ov9282_set_selection()`'s mode whitelist was extended to include both new
modes (so runtime `y_start` moves work on them too, same as the existing two
ROI modes) — explicit whitelist, not a `crop.height != PIXEL_ARRAY_HEIGHT`
inference (`MODE_1280_720` also has a non-800 crop height and must stay
non-adjustable).

Built, vermagic/srcversion-matched, `depmod`-installed to
`/lib/modules/.../updates/ov9282.ko`, and **rebooted in already** — running
now. `tainted` = `4096` throughout every test below, no dmesg BUG/Oops.

**Frame content — validated clean, both modes, both cameras.** Headless
capture (no live display needed): all 4 combinations negotiated the correct
raw size, non-degenerate pixel stats, and visually coherent gradient/noise
texture with no tearing/banding/garbage (spot-checked all 4 saved PNGs).

**Throughput — real, substantial speed wins over every previously
characterized mode, floors found for both:**

| mode | rated ceiling | clean floor found | achieved fps at floor | prior best (400-row sibling) |
|---|---|---|---|---|
| `MODE_1280_200_ROI` | 542.59fps | ~1600-1825us plateau, no hang found down to 1600us | ~511-514fps | `MODE_1280_400_ROI`: ~282fps (unbinned) |
| `MODE_640_100_ROI` | 1071.81fps | clean @1050us (**~880fps**), hangs @1000us | **~854-880fps** | `MODE_640_200_ROI`: ~527fps (binned) |

`MODE_640_100_ROI` is now the fastest mode found in this entire project —
~1.7x the previous ~527fps ceiling. `MODE_1280_200_ROI` didn't hang anywhere
in the tested range (1600-3400us) — it plateaus around ~511-514fps instead
(short of its 542.59fps rated ceiling) rather than crashing, unlike every
other mode swept so far; true hang floor (if one exists above the sensor's
absolute limit) not found, not chased further since the plateau already
answers "is there a speed win" (yes). Full sweep CSVs:
`camera_throughput_sweep_1280x200.csv` (+ `..._floor.csv`, `..._floor2.csv`)
and `camera_throughput_sweep_640x100.csv` (+ `..._probe.csv`,
`..._probe2.csv`).

**One real hang hit and recovered from cleanly, same pattern as the
original 640x200 floor-sweep hang**: `640x100` @ 1000us timed out (20s) and
was killed by the sweep orchestrator, orphaning two `camera_throughput_test.py`
multiprocessing workers (parent killed, forked children survive). Checked
state before acting: `ps -o stat,wchan` showed `Sl`/`futex_do_wait`
(userspace lock wait, not kernel `D`-state), so `kill -9` on both was
expected-safe — confirmed via `lsof` that all `/dev/video*`/`/dev/media*`
fds were released after. Ran a known-good sanity check
(`camera_throughput_test.py 01 3400`, stock 640x400) afterward: 281.95/281.19fps,
matching the established baseline exactly. **No reboot needed.** `tainted`
stayed `4096`, no dmesg BUG/Oops throughout.

`camera_throughput_sweep_subprocess.py` was generalized to take optional CLI
args (`[WxH] [durations_csv] [output_csv]`) so the same sweep script works
for any mode — defaults reproduce the original `MODE_640_200_ROI`-only
behavior exactly, fully backward compatible.

~~1. Commit `ov9282.c`/`ov9282.ko`~~ — **done**, commit `3343434`.

**Closed-loop LED round-trip test — DONE, both new modes, both a major win
(2026-07-14).** Same `led_dual_camera_closed_loop_test_mp.py` pattern used
for `MODE_640_200_ROI` (2nd CLI arg selects raw size, ROI always
full-frame). Two runs each, at each mode's validated clean floor:

| | `1280x200` @ 1775µs, run 1 | run 2 | `640x100` @ 1050µs, run 1 | run 2 |
|---|---|---|---|---|
| confirmed transitions (5s) | 1172 (0 timeouts) | 1136 (0 timeouts) | 1194 (0 timeouts) | 1160 (0 timeouts) |
| achieved capture fps | 496.31 / 495.71 | 491.84 / 492.64 | 860.59 / 860.79 | 858.32 / 861.92 |
| mean latency cam0/cam1 | 3.381 / 3.297 ms | 3.392 / 3.289 ms | 3.501 / 3.540 ms | 3.724 / 3.645 ms |
| max latency cam0/cam1 | 29.568 / 9.179 ms | 27.332 / 13.583 ms | 12.697 / 7.705 ms | 15.593 / 21.900 ms |
| max \|skew\| | 25.386 ms | 23.683 ms | 8.693 ms | 17.828 ms |
| **effective closed-loop freq** | **234.38 Hz** | **227.18 Hz** | **238.79 Hz** | **232.06 Hz** |

**`MODE_640_100_ROI` is now the best closed-loop result in the project**:
~232-239Hz effective toggle rate (vs. the prior best ~207Hz at
`MODE_640_200_ROI`/1800µs), mean per-camera latency 3.5-3.7ms (comfortably
under Phil's 10ms target, same as the prior best), and *tighter* tails than
`1280x200` (max latency 7.7-21.9ms vs. 9.2-29.6ms) despite running at a much
higher fps — consistent with it being the mode with the most margin below
its own hang point (1050µs floor vs. a 1000µs hang, a comfortable 5% gap)
whereas `1280x200`'s floor was chosen from a plateau, not a hang boundary,
so it isn't actually "close to the edge" in the same sense.
`MODE_1280_200_ROI`'s occasional latency/skew outliers (e.g. the 29.568ms
max in run 1) are still well within the *closed-loop* success criterion (0
timeouts across all 4 runs) — flagged as worth another look before
presenting to Phil, not as a failure. `tainted` stayed `4096` throughout all
4 runs, no dmesg BUG/Oops. Calibration `POOR SEPARATION` appeared once
(cam0, `1280x200` run 2) — the same ambient-light calibration-noise wrinkle
already documented in "Hardware status" below, not a new finding.

**Still not yet done**:
1. Mid-stream `set_selection` write validation (the item-5/8 pattern from
   the runtime-ROI section below) hasn't been repeated on these two new
   modes specifically — only the original two ROI modes were covered there.
2. `camera_preview_roi.py`/`roi_live_demo.py` haven't been driven against
   these sizes on the live display yet (the CLI already accepts arbitrary
   `WxH` so this should just work, but hasn't been exercised).

## DONE: runtime-movable ROI via `set_selection`, live dual-camera demo built (2026-07-14)

**Stop here if picking this up fresh: this whole feature is validated and
usable now.** Driver patch rebooted in and clean; mid-stream/pre-stream
writes validated across both cameras and both ROI modes (all combinations
clean, see "Validation results" below); `roi_set_selection.py` is a real,
committed helper (`get_roi_y_start`/`set_roi_y_start`); and
`roi_live_demo.py` is a real, committed interactive demo — two live camera
windows with a live fps overlay, keyboard-driven ROI move, and a per-camera
binning toggle, confirmed working end-to-end on the Pi's actual display
(see "Live demo" section below, item 11 in particular for the binning
toggle and the two real bugs found validating it). The item-7 kernel-side
negative-`y_start` clamp bug is now fixed (not just worked around) — see
that item for details. Remaining open item is repeated mid-stream trials
(each camera×mode combo has only been tried once) — polish, not a blocker.

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
   sent straight to the driver did **not** clamp to the `0` floor the way
   an over-range value correctly clamped to the max — it wrapped around
   (unsigned-arithmetic underflow in the kernel-side clamp in
   `ov9282_set_selection`) and landed at the **max** position (400)
   instead. Confirmed directly with a raw `v4l2-ctl --set-subdev-selection
   ...top=-42...` call (independent of this Python script), so it was a
   genuine kernel-side gap, not a bug in the wrapper. Worked around in
   `roi_set_selection.py` by clamping `y_start` to `[0, MAX_Y_START]` in
   Python *before* it ever reaches the driver, so this wrapper never
   triggered the bug.

   **Kernel-side fix — DONE, committed (2026-07-14).** Fixed the clamp
   itself in `ov9282_set_selection()`: the old code compared/subtracted
   `sel->r.top` (signed) against `OV9282_PIXEL_ARRAY_TOP` (unsigned
   literal) directly, so a negative `top` silently promoted to a huge
   unsigned value before the `> OV9282_PIXEL_ARRAY_TOP` check, and that
   huge value then clamped down to `max_y_start` instead of `0`. Fix does
   the subtraction in a signed `s32` temporary first, then clamps into the
   unsigned `y_start` only after checking the sign. Built, vermagic-
   matched, depmod-installed, reboot done. **Validated directly against
   the live subdev** (camera 0, bypassing the Python wrapper entirely, raw
   `v4l2-ctl --set-subdev-selection ...top=-42...`): now correctly lands
   at `top=8` (`y_start=0`), not the old `top=408` (`y_start=400`) wrap.
   Regression-checked the existing over-range-positive clamp still works
   (`top=608` → clamps to `top=408`/`y_start=400`, unchanged). `tainted`
   stayed `4096`, no dmesg BUG/Oops during either test. Camera 0 reset
   back to `y_start=0` afterward. The Python-side clamp in
   `roi_set_selection.py` is now redundant (driver enforces both ends
   correctly) but left in place as defense-in-depth / documentation of the
   valid range — no reason to remove it.
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
    `y_start` overlaid — every move goes through `set_roi_y_start()` while
    both cameras are actively streaming, i.e. it continuously exercises the
    exact mid-stream write path validated in items 5/8/9 above, not just a
    canned before-start move.

    **First version moved both cameras' ROI together (shared `y_start`)**;
    validated as described just below, then **the user asked for
    independent per-camera control instead**, so it was redesigned:
    `1`/`2` pick which camera is "active" (highlighted green in its own
    overlay, e.g. `cam 0: y_start=60 [ACTIVE -- w/s/r apply here]`, vs. gray
    `cam 1: y_start=40  (press 2 to control)` for the inactive one), `w`/`s`
    move only the active camera, `r` resets only the active camera, `a`
    resets all cameras, `q` quits. Each camera keeps its own `y_start` in a
    dict, independent of the others.

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

    **Independent-control redesign validated the same way (2026-07-14,
    `python3 roi_live_demo.py 640x200 20`).** Confirmed via direct
    `v4l2-ctl --get-subdev-selection` reads on both subdevs after each
    keypress (not just the on-screen overlay), which is the ground truth:
    with camera 0 active, three `s` presses moved only `/dev/v4l-subdev5`
    to `top=68` (`y_start=60`) while `/dev/v4l-subdev2` (camera 1) stayed at
    `top=8` (`y_start=0`) — untouched. Pressed `2` to switch active to
    camera 1, then `s` twice: `/dev/v4l-subdev2` moved to `top=48`
    (`y_start=40`) while camera 0 stayed exactly at `y_start=60` — i.e. the
    two cameras ended up at **genuinely different, independently-held
    positions** (60 vs. 40), not just independently-driven toward the same
    value. Switched back to camera 0, moved it further (three more `s`) to
    `y_start=60`+more while camera 1 stayed at 40. Screenshotted both
    windows and confirmed the overlay tagging matches reality: the active
    camera's text renders green with `[ACTIVE -- w/s/r apply here]`, the
    inactive one gray with `(press N to control)`. `q` exited cleanly (exit
    0), `tainted` stayed `4096` throughout, no dmesg anomaly, both subdevs
    reset to `y_start=0` after. One caveat noted during this test, not a
    bug: a `timeout`-based kill (SIGTERM, as opposed to pressing `q`) does
    **not** run the script's `finally` cleanup (`cv2.destroyAllWindows()` /
    `cam.stop()`) — Python's default SIGTERM handling doesn't raise
    anything catchable the way Ctrl-C/SIGINT does. Checked after one such
    kill happened by accident (this session's own 90s test timeout expired
    mid-test): no lingering process, no held `/dev/video*`/`/dev/media*`
    fds via `lsof`, taint stayed clean — so OS-level process teardown was
    sufficient in practice, but this means the demo's own cleanup code is
    *not* what's relied on if it's ever killed by something other than `q`.

11. **fps overlay + live binning toggle — DONE, committed (2026-07-14).**
    User asked for a test app showing both feeds with fps and letting them
    adjust ROI *and* toggle binning per camera. Added to `roi_live_demo.py`:
    a rolling 30-frame fps counter per camera in the overlay (`{fps:.1f}fps`,
    computed from wall-clock timestamps of the app's own capture loop — an
    honest "what this app is actually achieving," not an isolated hardware
    benchmark), and a `b` key that toggles the *active* camera between the
    two windowed ROI modes (`640x200` binned <-> `1280x400` unbinned) via a
    real `cam.stop()` / reconfigure / `cam.start()` cycle — a materially
    different, heavier operation than the mid-stream `y_start` move (see the
    "Scope decision" note near the top of this section on why binning/size
    changes need a full reconfigure).

    **Two real bugs found and fixed while validating this on the live
    display, not just written and assumed correct:**

    - **Shift+key shortcuts don't work through this app's input path.**
      First cut had `B` (shift+b) as a "toggle binning on all cameras"
      shortcut. Testing via `xdotool key shift+b` (the same driving method
      used for all prior live-demo validation) showed cv2's `waitKey()` on
      this GTK backend never receives the shifted keycode — it silently
      delivers plain lowercase `b` instead, confirmed by logging the raw
      key values received. This meant `B` was actually just re-toggling
      whichever camera was already active, not operating on both — wrong
      silently, not wrong loudly. Fixed by dropping the bulk shortcut
      entirely rather than chasing a fragile modifier-key path: `b` now
      only ever affects the active camera (select the other with `1`/`2`
      and press `b` again for both) — also a better fit for this app's
      already-established independent-per-camera control model than a
      bulk "do both" shortcut would have been anyway.
    - **Reconfiguring one camera silently resets the OTHER (untouched)
      camera's `roi_y_start` back to 0.** Found by testing the full
      sequence live: set cam0 to `y_start=60` and cam1 to `y_start=40`
      (both confirmed via `v4l2-ctl --get-subdev-selection`), toggled
      cam1's binning (cam1's own position correctly preserved at 40,
      verified immediately after) — and cam0, never touched, had silently
      dropped to `y_start=0`. Reproduced in both directions (toggling
      either camera clobbers the other's already-applied position) and
      confirmed NOT just an app-side bookkeeping bug: the driver itself
      reports the wrong value on direct `v4l2-ctl` reads of the untouched
      camera's own subdev, so this is a genuine kernel/driver-level side
      effect, most likely the two sensors sharing one media-graph
      pad-format validation pass in the CFE bridge driver during any
      single camera's `cam.configure()`, even though their CSI/register
      paths are otherwise fully independent (matches the shared
      `media_device` object noted in the `MODE_1280_400_ROI` crash
      investigation elsewhere in this file, though that was a much more
      severe rmmod/insmod-only bug — this is a much milder, non-crashing
      side effect of routine reconfigure). **Not a kernel patch fix** —
      worked around at the application level: `toggle_binning()` now
      re-applies *every* camera's stored `y_start` (with the existing
      retry-verify logic, itself added after finding cam.start() can also
      return before the driver's pad-format selection has fully settled,
      causing an immediate set_selection to silently no-op) after any
      single camera's reconfigure, not just the one that was toggled.
      Validated after the fix: toggled cam1 then cam0 in sequence, both
      held their distinct positions (60 vs. 40) both immediately and 5s
      later; `tainted` stayed `4096`, no dmesg anomaly throughout. Worth
      flagging as a real, reproducible driver quirk if anyone digs into
      the CFE bridge driver later — out of scope to root-cause further
      here now that there's a working per-app-level guard against it.

    End-to-end re-validated on the Pi's live display after both fixes:
    fps overlay renders correctly on both cameras, `b` toggles only the
    active camera's binning with its window resizing to match (confirmed
    via screenshot), `q` exits cleanly (process gone, no lingering
    `/dev/video*`/`/dev/media*` fds via `lsof`), both subdevs reset to
    `y_start=0` afterward.

12. **fps was capped at ~130fps instead of the validated ~527fps ceiling —
    root-caused and fixed, DONE (2026-07-14).** User noticed the demo's
    displayed fps (~130) was roughly half the ~280fps stock ceiling and
    nowhere near the ~527fps `640x200`-binned ceiling this whole ROI effort
    exists to demonstrate, and asked specifically whether the display was
    the bottleneck and whether capture/display could be decoupled. Found
    **two separate real bugs**, isolated one at a time with targeted
    diagnostics rather than guessing:

    - **`FRAME_DURATION_US` was hardcoded to `6000`** (inherited from the
      original ROI-move-only demo, never revisited when binning was added)
      — a hard ~166fps ceiling regardless of mode, already below what was
      actually being observed. Fixed: per-mode duration from the already-
      validated floors elsewhere in this file (`1800us`/binned,
      `3400us`/unbinned), looked up by raw size in `configure_and_start()`.
    - **Even after that fix, only ~274-306fps** (measured via a throttled-
      display version and a display-free repro respectively) — short of
      the ~527fps binned ceiling. Bisected with two standalone diagnostic
      scripts (not the demo itself, to isolate variables): (1) a bare
      dual-camera capture loop with **zero** cv2/display code got
      **~494fps/camera** — ruling out single-process/GIL contention
      between the two cameras as a meaningful cost (matches solo-camera
      capture at ~493-496fps almost exactly, i.e. running both cameras in
      one process barely costs anything by itself); (2) that same loop
      with `cv2.namedWindow`s created and a **throttled** display added
      back showed `cv2.waitKey(1)` — called every capture iteration, as
      the demo originally did — costing **~2.24ms/call measured** (not the
      nominal "1ms"), consuming **~69% of total loop time** with two
      windows open on this GTK/Wayland backend. That was the dominant
      bottleneck, not GTK image rendering (already ruled out by the
      throttled-display test in item 11) and not GIL contention. Fixed:
      `cv2.waitKey()` is now called only at the same ~15Hz throttle as the
      display redraw (`DISPLAY_INTERVAL_S`), not every capture iteration —
      keyboard response stays well under human reaction time (~67ms worst
      case) while no longer capping capture throughput.

    **Re-validated on the live display after the fix**: `640x200` binned
    showed **524.6fps / 531.6fps** (cam1/cam0) — matching the ~527fps
    floor-sweep ceiling documented in the `MODE_640_200_ROI` section below.
    `1280x400` unbinned showed a similar jump, cross-checked against a
    display-free raw-capture measurement at the same duration (**262.2fps
    both cameras**, matching the ~280fps documented ceiling for that mode
    within normal run-to-run variance). Keyboard controls (`s`/`r`/`q`)
    confirmed still fully responsive after the fix via direct subdev
    reads, not just visually. `tainted` stayed `4096` throughout every
    test in this investigation, no dmesg anomaly. Both subdevs reset to
    `y_start=0` afterward.

    **Follow-up found and fixed a second-order regression from the fps
    fix itself, DONE (2026-07-14).** User asked to re-test on
    `1280x400` with the display visible; while re-validating fps there
    (confirmed **241.1fps / 241.4fps** cam0/cam1, matching the ~262fps
    display-free ceiling within normal display overhead), a rapid 2x `s`
    keypress test only moved the ROI once instead of twice. Root cause:
    throttling `cv2.waitKey()` to ~15Hz means each poll only pops a
    *single* queued key (`waitKey` never drains a queue, just returns one
    key per call) — before this session's fps fix, `waitKey` ran on every
    capture (~500+/sec), so this was never an issue; throttling it
    exposed a real gap. Fixed by draining all pending keys in a `while`
    loop each time the throttle window opens (loop until `waitKey`
    returns "no key"/`0xFF`), instead of processing just one. Verified
    with `xdotool key --delay 30 s s s s` (4 presses ~30ms apart): still
    only 3 of 4 registered even after the drain-loop fix — traced this to
    X11/GTK itself coalescing keydown events faster than ~30-100ms apart
    before they ever reach `cv2.waitKey()`, not something app-level
    draining can address. At `--delay 120` (a realistic human typing
    cadence), all 4 presses registered correctly every time. Net: the
    drain-loop fix closes the real gap (2 rapid presses: 1→2 registered)
    that throttling introduced; the remaining synthetic-burst-speed edge
    case is a lower-level input-stack limit below any real human typing
    speed, not a practical concern for this demo. fps re-confirmed
    unchanged after the drain-loop fix (240.7fps / 236.7fps). `tainted`
    stayed `4096` throughout, no dmesg anomaly, both subdevs reset to
    `y_start=0` afterward.

    **Binning toggle re-validated starting from `1280x400`, DONE
    (2026-07-14).** All prior binning-toggle validation (item 11) started
    from the `640x200` default; user asked to also confirm starting from
    unbinned. Launched `roi_live_demo.py 1280x400 40`, set cam0/cam1 to
    distinct positions (`y_start=120`/`80`) while both unbinned, toggled
    each camera's binning to `640x200` individually (`1`/`b` then `2`/`b`)
    — both landed at the ~527fps ceiling (**532.2fps cam0, 529.7fps
    cam1**, read directly off the overlay) with positions correctly
    preserved (confirmed both via the app's own log and independently via
    `v4l2-ctl` reads on both subdevs — the cross-camera clobbering fix
    from item 11 held with `1280x400` as the starting mode too, not just
    `640x200`). Completed the full round trip by toggling both cameras
    back to unbinned: both subdevs correctly reported `Width 1280, Height
    400` again with positions still exactly `y_start=120`/`80` unchanged.
    `tainted` stayed `4096` throughout, no dmesg anomaly, `q` exited
    cleanly (no lingering `/dev/video*`/`/dev/media*` fds), both subdevs
    reset to `y_start=0` afterward.

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
detected (`i2c@88000` and `i2c@80000`).

**Stress-tested 2026-07-14 — `i2c@80000` connection is solid.** Ran
`led_dual_camera_closed_loop_test_mp.py 3400` 8 times back-to-back (fresh
process each run, stock 640x400). Every run: both cameras initialized
cleanly and sustained their normal ~280-282fps raw capture rate for the
full 5s, `tainted` stayed `4096` throughout, no dmesg BUG/Oops/WARNING at
any point. **No sign of the CSI-2-link-silently-not-coming-up failure mode
that motivated this test** — camera 1 never failed to init or dropped out
across 8 consecutive runs.

One separate, pre-existing wrinkle surfaced during this stress test, **not**
a camera/driver reliability finding: runs 5 and 7 (of 8) showed a collapsed
confirmed-transition rate (2.24Hz and 8.78Hz vs. the normal ~140-230Hz) with
real timeouts (5 each), and 3 of the 8 runs logged `POOR SEPARATION` on the
script's own ON/OFF brightness calibration check. This tracks as LED-
detection/ambient-light noise in the test harness's calibration step, not
an `i2c@80000`-specific hardware fault: `POOR SEPARATION` hit *both*
cameras' calibration simultaneously in every affected run (an asymmetric,
camera-1-only failure would be the expected signature of a flaky physical
connection, not a symmetric one), and run 7 had timeouts despite *both*
cameras calibrating "ok" — so the correlation with the calibration warning
isn't even clean. Worth knowing about if someone reruns this test and sees
a low toggle-frequency outlier, but doesn't call the hardware connection
into question — that's now considered validated under sustained load.

Also note: 2 spare Raspberry Pis + 2 spare cameras have been ordered. Ribbon
cable is printed "HBV-Raspberry-160FPC" — useful search string for exact
camera module matching if needed later.

**New finding (2026-07-15): camera 0 (`i2c@88000`) failed to probe at this
boot.** While testing `camera_view_tool.py` on the bench, `rpicam-hello
--list-cameras` and `Picamera2.global_camera_info()` both showed only one
camera (`i2c@80000`, the previously-flaky one — now enumerating fine and
renumbered to index 0 in its absence). `dmesg` showed, from very early in
this boot (~4s in, i.e. not triggered by anything this session did):
`ov9282 10-0060: fail to write MIPI_CTRL00` / `failed to power-on the
sensor` / `probe with driver ov9282 failed with error -5` — an I2C
communication failure at probe time for the sensor on the `i2c@88000`
controller. `tainted` stayed `4096` throughout (no BUG/Oops), so this is
almost certainly the same class of marginal-connection issue documented
above for `i2c@80000` (loose/marginal ribbon seating), just affecting the
other port/camera this time. Not yet re-seated or rebooted to confirm
recovery — do that before trusting any single-camera test result as "only
one camera works now" rather than "only one camera came up THIS boot."

## `roi_set_selection.py` subdev lookup fixed — hardcoded CFE media-device mapping was wrong (2026-07-28)

`camera_view_tool.py` crashed on startup: `RuntimeError: no ov9281 entity
found in /dev/media0's topology for camera 0`. Root cause: only one camera
(`i2c@88000`) enumerated this boot (the already-documented intermittent
dropout, see "Hardware status"), and with that camera population,
`i2c@88000` fed CFE device `/dev/media3` (confirmed via libcamera's own
startup log: `Registered camera .../i2c@88000/... to CFE device
/dev/media3 and ISP device /dev/media0`) — but `roi_set_selection.py`'s
`_subdev_for_camera()` had a hardcoded `{"i2c@88000": "/dev/media0", ...}`
table, on the theory (stated explicitly in a since-removed comment) that
the CFE media-device *number* was a stable physical association, unlike
the already-known-unstable `/dev/v4l-subdevN` node number. That theory was
wrong — `/dev/media0` this boot was the ISP (`pispbe`) device, not a CFE
device at all, so it had no `ov9281` entity to find. Same class of
boot-order-dependent numbering bug the function already existed to work
around, one layer up.

**Fixed properly, not by hardcoding a different guess**: `_subdev_for_camera()`
now scans every `/dev/media*` device's topology for an `ov9281 <bus>-<addr>`
entity (e.g. `ov9281 10-0060`), and for each one resolves which devicetree
i2c label that Linux i2c bus number actually corresponds to via sysfs
(`readlink -f /sys/bus/i2c/devices/10-0060/of_node` → a path containing
`i2c@88000`) — genuinely fixed hardware wiring, unlike any `/dev/mediaN`
or `/dev/v4l-subdevN` enumeration order. Verified directly:
`python3 roi_set_selection.py 0` now correctly reports `y_start=0 (max=400)`
against this boot's actual `/dev/media3`/`/dev/v4l-subdev2`. Not yet
re-confirmed against a two-camera boot, but the fix removes the
boot-order assumption entirely rather than adding a second special case.

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
  modes (`MODE_1280_400_ROI`, `MODE_640_200_ROI`, and the newer
  `MODE_1280_200_ROI`/`MODE_640_100_ROI`), defaults to `640x200`
- `camera_throughput_sweep_subprocess.py` — frame-duration sweep for
  `camera_throughput_test.py`, fresh subprocess per value (same
  crash-recovery rationale as `led_dual_camera_sweep_subprocess.py`). Takes
  optional `[WxH] [durations_csv] [output_csv]` CLI args — defaults
  reproduce the original `MODE_640_200_ROI`-only sweep.
- `roi_set_selection.py` — runtime helper for the `set_selection` patch:
  `get_roi_y_start(cam_index)` / `set_roi_y_start(cam_index, y_start)`,
  also runnable as a CLI (`python3 roi_set_selection.py <cam_index>
  [y_start]`). Clamps `y_start` client-side before it reaches the driver —
  see the "runtime-movable ROI" section above for a driver-side clamp bug
  this fixed.
- `camera_view_tool.py` — bench alignment viewer: full-sensor (1280x800) or
  windowed-ROI (1280x400/1280x200) live view with beam centroid tracking.
  Detects the beam via a raw-value adaptive threshold with a confidence
  gate (NOT Otsu-on-normalized-8-bit -- that broke down in the narrower ROI
  crops on the real bench signal, see "Beam detection note" in the script's
  own docstring for the full story), overlays a reticle (ring + tick marks
  + center dot) at the intensity-weighted centroid, and shows a live
  per-camera fps counter. `h` cycles ROI height (800 -> 400 -> 200 -> 800,
  each a full stop/reconfigure/start) and auto-centers each new window on
  wherever that camera's beam was last confidently seen. `q` quits. **Also
  streams camera 0's centroid to the Nucleo over I2C by default** (full
  capture speed, decoupled from the ~15Hz display -- see the "gains
  built-in full-speed I2C streaming" section above); `--no-stream` for
  pure bench-viewer mode, `--stream-cam N`, `--dry-run`. Live end-to-end
  streaming not yet reconfirmed (see that section).
- `roi_live_demo.py` — interactive live demo: both camera ROI feeds side by
  side with a live fps overlay, independent per-camera control (`1`/`2`
  picks the active camera, `w`/`s` move its ROI while streaming, `r`
  resets it, `a` resets all ROIs, `b` toggles its binning between
  `640x200` binned and `1280x400` unbinned via a full reconfigure, `q`
  quits — no bulk "toggle all binning" shortcut, see item 11 in the
  "runtime-movable ROI" section for why). Needs an actual display
  (`DISPLAY=:0` — this Pi has a real desktop session via `labwc`), not
  runnable headless. `python3 roi_live_demo.py [WxH] [step]`, defaults
  `640x200`, step=20 rows.
- `beam_position_streamer.py` — headless capture -> detect -> stream to an
  STM32 Nucleo over I2C, no display. `python3 beam_position_streamer.py
  [WxH] [--y-start N] [--dry-run]`, defaults to full-sensor 1280x800. The
  fake-orbit I2C link itself is confirmed live end-to-end (see "streaming
  beam position" section above), but this script's own real camera-driven
  send has still never been run for real — `camera_view_tool.py`'s
  built-in streaming got exercised first instead.
- `nucleo_i2c_sender.py` — `NucleoLink` class used by both
  `beam_position_streamer.py` and `camera_view_tool.py` to send the
  register-mapped position packet over I2C; also runnable standalone for
  a smoke test (sends a fake orbiting position). `send_position(x, y)`
  takes real (float) pixel coordinates and internally scales by
  `POSITION_SCALE` (10) to preserve one decimal digit of sub-pixel
  centroid precision in the wire's `s16` field — the Nucleo firmware
  (outside this repo) must divide by `POSITION_SCALE` to match, not yet
  done. See "gains built-in full-speed I2C streaming" above.
- `fta_calibration.py` — sweeps the FTA over a grid of DAC setpoints via
  the (separate, USB-serial-driven) "FTA Controller" Nucleo's own
  `grid_scan` command, pairs each sampled point with a camera centroid,
  and fits a 3x3 DAC↔centroid affine (plus its inverse). `python3
  fta_calibration.py X1 Y1 X2 Y2 [--grid-step N] [--raw-size WxH]
  [--y-start N] [--port PORT] [--frames-per-point N] [--out PATH]
  [--dry-run]`. Talks over serial (460800 baud), NOT the I2C link
  `nucleo_i2c_sender.py`/`NucleoLink` use — see "FTA position calibration"
  section above. Not yet run against real hardware.
- `fta_serial_latency_test.py` — measures real round-trip latency
  (`--mode ping`/`setpos`), fire-and-forget burst throughput at max speed
  (`--mode burst`), and a rate-paced sweep to find the actual safe
  fire-and-forget ceiling (`--mode sweep`) — all via the firmware's own
  `cmdq_stats` drop counter, not a synthetic estimate. All modes
  non-destructive. Run for real 2026-07-23, see "Serial-vs-I2C latency"
  section above for the full results table.
- `fta_step_response_test.py` — commands a step in one FTA DAC axis over
  serial and logs camera centroid vs. time spanning the step, computing
  rise time/overshoot/settling time — the actuator DYNAMICS data the
  static `fta_calibration.py` matrix can't provide, needed before tuning
  the PI control law's gains. `python3 fta_step_response_test.py
  --step-to N [--axis x|y] [--step-from N] [--raw-size WxH] [--y-start N]
  [--pre-s SEC] [--post-s SEC] [--settle-tol-px PX] [--port PORT]
  [--out PATH]`. Built but not yet producing real data — see "PI control
  law designed" section above: the FTA isn't currently moving the beam
  this camera sees at all, a hardware gap unrelated to this script.
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

## Next steps (as of 2026-07-14)

1. **Ask Phil**: discrete step-and-confirm (~120-230Hz range, see closed-loop
   numbers throughout this file) vs. continuous latest-frame tracking
   (~280-527fps range, mode-dependent) — this determines which ceiling is
   actually the one to design/optimize around. Blocking further speed work.
2. ~~Stress-test `i2c@80000` reconnection~~ — **done, solid** (2026-07-14, see
   "Hardware status" above): 8 back-to-back runs, no init failures, no
   dmesg anomalies, consistent ~280fps raw capture every time.
3. **Repeat mid-stream ROI-move trials** for more confidence: each of the 4
   camera×mode combinations for the live `set_selection` write (see
   "runtime-movable ROI" section) has only been tried once. More repeats —
   especially writes landing at different points within a frame's readout
   window, or back-to-back writes in a single stream — would build more
   confidence than "clean once per combination" currently provides.
