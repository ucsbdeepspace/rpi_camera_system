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

## HANDOFF (2026-08-06): moving to the laptop to build the v2 firmware — Pi side is ready and running

Picking up right after "Architecture DECISION v2" below (Nucleo runs the
PID, Pi is telemetry-only, laptop sends setpoints/tuning over VCP) got
logged, and after the SB16/SB18 amp-board I2C fix was confirmed end-to-end.
This note is the pickup point for the laptop session doing the actual
CubeIDE work — full design detail lives in "Architecture DECISION v2"
further down, this is just "what's already done, what to do next."

**Not yet done, and blocking the firmware work**: `camera_centroid_receiver`
still needs to be folded into this repo from the laptop's local CubeIDE
workspace (`STM32CubeIDE/workspace_1.14.0/camera_centroid_receiver`) —
this was decided back on 2026-07-28 and prepped from the Pi side (the
`.gitignore` block for build artifacts already exists, see below) but
never actually done. **Do this first**, as its own commit, before starting
any new firmware changes — see "Before editing it: fold
`camera_centroid_receiver` into this repo" further down for the exact
steps (copy the project folder into `nucleo_firmware/camera_centroid_receiver/`,
commit as an honest "import as-is" baseline, push, confirm push access
still works).

**Pi-side prep done this session, and left running**:
`beam_position_streamer.py` (not `camera_view_tool.py` — that's an
interactive bench GUI, not meant to run unattended) was hardened to
survive I2C send failures without crashing (commit `821cb1d`) — needed
because the Nucleo will be reset/reflashed repeatedly during firmware
bring-up, which previously would have killed the script outright on the
first failed send. It's meant to be started once and left running for the
whole laptop session; every reset/reflash on the laptop side will just
show up as a burst of `WARNING: I2C send to Nucleo failed` lines on the
Pi, then recover on its own once new firmware boots and starts ACKing
again. Start it (from this Pi) with whichever mode/`--y-start` currently
brackets the beam:
```
python3 beam_position_streamer.py 640x200 --y-start N
```

**Suggested build order once `camera_centroid_receiver` is folded in and
new firmware work starts** (from "Architecture DECISION v2"'s function
list): add DAC1 (PA4/PA5) + the amp-enable GPIO (GPIOA12) to the `.ioc`
first and get `open_loop`-mode `set_x`/`set_y` + `get_status` working —
bench-testable with just a multimeter, no Pi or camera needed. Then the
mode switch and heartbeat extension. PID + the I2C telemetry-staleness
fail-safe last, since that's the only part that actually needs
`beam_position_streamer.py`'s real telemetry to test against — which is
exactly why it's already running.

**Two things flagged as worth confirming early, not assumed**: (1) which
physical DAC channel (PA4 vs PA5) drives which axis in "FTA Controller"'s
existing `main.c`, so `apply_dac()` wires to the right channel; (2) that
`camera_centroid_receiver`'s current `.ioc` doesn't already claim
PA4/PA5/PA12 for anything (shouldn't, since that firmware never touched
them, but cheap to check before adding DAC1 + the amp-enable output).

**SUPERSEDED — this handoff note was stale by the time it was pulled.**
Written from the Pi without visibility into laptop-side work that had
already happened in parallel: `camera_centroid_receiver` was folded into
this repo, the full non-PID "firmware phase 1" was built, bench-tested on
real hardware, and a real VCP byte-loss bug found and fixed (NVIC
priority swap) — see "Firmware phase 1 flashed and bench-tested", "Folded
`camera_centroid_receiver` into this repo", and everything after them,
all further down this file. Step-response and sine-tracking
characterization was also done on top of that. **Current actual state**:
firmware built and hardware-verified through everything except the PID
loop itself (still deliberately not implemented — see "Firmware phase 1
(everything except PID) implemented"); open questions are the axis-
calibration invalidation and the step/sine dynamics findings documented
in the later sections, not anything this note describes as blocking.

**One item from this note *was* still worth checking, and got checked**:
confirmed directly against `7-element-array`'s `FTA Controller/Core/Src/main.c`
(`write_x_cmd_from_float`/`write_y_cmd_from_float`, ~line 741) — `DAC_CHANNEL_1`
is x, `DAC_CHANNEL_2` is y, no swap. `camera_centroid_receiver`'s
`apply_dac()` uses the identical mapping. **This rules out a DAC-channel
software swap as the explanation for the axis-coupling-flip finding**
(step-response section, "Actuator confirmed physically moving") — that
finding stands as a real physical change, not a wiring/software bug on
this end.

**Real, useful info from this note, worth keeping**: `beam_position_streamer.py`
was hardened on the Pi (commit `821cb1d`) to survive I2C send failures
(catch-log-continue instead of crashing) rather than dying every time the
Nucleo resets/reflashes, and is meant to be left running unattended as the
Pi's telemetry source through repeated laptop-side firmware work —
relevant context for future bench sessions, since earlier sessions this
file documents weren't necessarily running that hardened version.

Two coordination gaps this surfaced, worth naming so they don't repeat:
this file is the hand-off mechanism between the Pi and laptop (per the
note at the very top of this file), but two sessions clearly ran on
different machines without pulling each other's commits first — worth
pulling before writing a new handoff note, not just before starting work.

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

### Actuator confirmed physically moving; full step-response protocol rerun on hardware (2026-08-04)

User confirmed via `fta_manual_control.py` that `amp on` does something
physically observable at the amp board and the actuator visibly moves —
resolves the open question above. `amp_enable`'s PA12 signal was always
reaching the board; whatever the null result on the very first
`fta_step_response_test_vcp.py` run was, it wasn't "amp board unpowered/
disconnected." Not root-caused further (not necessary once a real
non-zero response was confirmed on the retest below).

**Full original protocol rerun** (95→2000 large step, then 2000↔2100 small
steps ×3, both axes; amp enabled once for the whole batch rather than
re-toggled per run) — real motion confirmed on every single run this time:

| axis | step | dominant (2026-07-23 → now) | delta (px) | overshoot | settling (2026-07-23 → now) |
|---|---|---|---|---|---|
| x | 95→2000 | cy → **x** | -148.6 | 0.0% | 793ms → 922ms |
| x | 2000→2100 | cy → **x** | -19.8 | 0.0% | 45ms → **469ms** |
| x | 2100→2000 | cy → **x** | +5.3 | 79.1% | 67ms → 31ms |
| x | 2000→2100 (repeat) | cy → **x** | -7.1 | 0.0% | 80ms → 125ms |
| y | 95→2000 | cx → **y** | -124.2 | 0.0% | 1183ms → 765ms |
| y | 2000→2100 | cx → **y** | -11.5 | 0.0% | 83ms → 250ms |
| y | 2100→2000 | cx → **y** | +3.4 | 94.1% | 144ms → 156ms |
| y | 2000→2100 (repeat) | cx → **y** | -5.7 | 0.0% | 22ms → 234ms |

Raw data: `results/fta_step_response_vcp_{x,y}_20260804T2326-2327*.npz`.

**Finding 1 — axis coupling flipped, real and consistent across all 8
runs, not noise**: pre-fix, DAC-x visibly drove pixel-*y* (cy) and DAC-y
drove pixel-*x* (cx) — a rotated rig. Now each axis drives its own
matching pixel coordinate directly (DAC-x → cx, DAC-y → cy). Consistent
direction across every run rules out measurement noise as the explanation.
**Physical cause not identified** — plausibly the camera or actuator got
reoriented during the SB16/SB18 rework or the bench reconfiguration for
laptop-based testing, but not confirmed by inspection. **Practical
consequence: any old `fta_calibration.py` fit matrix is now invalid** — a
fresh calibration sweep is required before that matrix can be trusted
again, independent of anything else in this thread.

**Finding 2 — small-step settling times are larger and noisier than
pre-fix (125-469ms vs. 22-144ms), large-step settling is comparable
(765-922ms vs. 793-1183ms)**: not yet distinguished between two
explanations — (a) a real dynamics change from the rework (extra parasitic
mass/damping from rework, different mechanical preload after
reorientation), or (b) a measurement-resolution artifact of this specific
retest method: `fta_step_response_test_vcp.py` samples at ~170-190/s (the
VCP relay rate), well below whatever camera-direct rate the original
2026-07-23 numbers used, and `analyze_step`'s settling-time criterion
(every remaining sample must stay within tolerance) is more sensitive to a
single noisy tail sample at a lower sample rate. **Not yet resolved** —
more repeats, or a camera-direct rerun via `fta_step_response_test.py` for
a resolution-matched comparison, would distinguish these.

**Next steps, not yet decided/done**: (1) re-run `fta_calibration.py`
(needs the same protocol/status-format/baud fixes already applied to the
step-response script, not yet done) given Finding 1; (2) more repeats of
the small-step sequence, or a camera-direct comparison run, to resolve
Finding 2 before trusting these settling times for gain tuning; (3) only
after both are settled, pick real `Kp`/`Ki` and implement the PID loop
(the piece deliberately deferred through all of "Firmware phase 1").

### RESOLVED (2026-08-06): the "large phase lag" below was a sign-unaware analysis bug, not a real actuator problem — real lag is ~41ms, consistent with a fixed pipeline delay

**Root cause of the whole "large phase lag" thread below**: `fit_sine()`
reported `amplitude = sqrt(A^2+B^2)` (always positive) and
`phase = atan2(B, A)`, but the step-response data already on record in
this file showed DAC increases move axis-x's pixel value *down* — a real,
normal sign convention (however the actuator/optics happen to be
oriented), not a bug. A negative gain is mathematically indistinguishable
from a 180-degree phase shift, so it showed up as phase ≈ ±180° at *every*
tested frequency regardless of any real dynamics — exactly the "roughly
constant phase, not constant time delay" pattern noted below as
unresolved, misread as an alarmingly large lag present even near DC.

**Two other real mistakes compounded this before it got sorted out**: (1)
the very first 0.1Hz run showed ~0 amplitude — not a frequency-response
finding at all, just the amp actually being off (a fire-and-forget
`amp_enable` had silently failed to take effect; fixed by having both
scripts verify via `get_status` and abort if it didn't take, rather than
proceeding to collect data against a de-energized actuator). (2) The
retest after that ran with the beam not confidently detected at all
(`tel_status=0`, 0 usable samples) — a Pi-side camera/ROI issue, resolved
by checking on the Pi and restarting its streaming script.

**Fix**: `fit_sine()` now picks whichever reference (0° or 180°) puts the
residual phase within ±90°, returning a *signed gain* (the direction)
separately from that residual (the real, small, frequency-dependent lag).
Reran 0.1/0.5/1/2Hz clean with this fix plus the amp-verification fix:

| freq | gain (px / 200 DAC counts) | lag (ms) |
|---|---|---|
| 0.1 Hz | -19.2 | 161 |
| 0.5 Hz | -17.8 | 68 |
| 1 Hz | -16.6 | 59 |
| 2 Hz | -15.2 | 47 |

Fitting lag-in-degrees vs. frequency gives a near-perfect line through
near-zero (residuals under 1.5° across the whole range) — the signature
of a roughly **constant ~41ms time delay**, not frequency-dependent
actuator dynamics (which would curve/plateau, not stay linear all the way
down to 0.1Hz). ~41ms is a plausible total for this test's own
measurement pipeline (camera capture → I2C → Nucleo print → USB VCP →
Python read) — a real closed-loop PID running on the Nucleo reads
telemetry directly at ISR level and would not pay most of this cost. Gain
is consistent across all 4 frequencies (mildly declining, -15 to -19px),
no sign of distortion or tracking failure at any rate tested so far.

**Net effect**: the actuator's own dynamics look faster/cleaner than the
original (mistaken) reading suggested — closer to good news than bad.
Doesn't yet answer the real question (10-20Hz, still untested) but the
analysis is trustworthy now, which it wasn't before. `docs/session_results_2026-08-04.pptx`'s
3 sine-tracking slides were rebuilt with the corrected numbers/story —
**rebuilt from scratch in one pass**, not edited in place:
`python-pptx`'s slide-removal trick (manipulating `_sldIdLst` directly)
leaves orphaned XML parts in the zip, which produced a file with
duplicate zip entries on the first attempt — caught via `zipfile`
integrity check before it was committed, worth remembering if slides ever
need removing from this deck again.

### Fresh calibration sweep run — `fta_calibration_vcp.py` built, real data collected, RMS higher than the script's own threshold but explained (2026-08-06)

Needed regardless of the sine-lag correction above: the step-response
axis-coupling-flip finding (see "Actuator confirmed physically moving")
already invalidated any old `fta_calibration.py` fit matrix, and no
replacement existed yet.

**Old `fta_calibration.py` doesn't work against this firmware** — drove
its sweep via "FTA Controller"'s `grid_scan` command, which
`camera_centroid_receiver` doesn't have (dropped rather than ported, per
the architecture decision), and captured centroids via `Picamera2`
directly on the Pi, which the v2 architecture doesn't need for this kind
of thing anymore. Built `fta_calibration_vcp.py` instead: same
laptop-only VCP architecture as the step/sine scripts — plain `set_x`/
`set_y` jumps with a settle wait, position read from the Nucleo's I2C-relay
print, no Pi access needed. Grid points swept in serpentine
(boustrophedon) row order (hysteresis not yet characterized — this at
least keeps travel direction consistent within a row).

**First real sweep, 200→3800 both axes, step 300 (169 points)**:
**169/169 captured, zero skipped** — the beam stayed in view across the
*entire* range, unlike the old pre-rework sweep documented earlier in
this file (982/1444 points lost past `dac_x≈900-1000`). Real, welcome
confirmation that whatever changed during the SB16/SB18 rework didn't
shrink the usable field of view.

**RMS residual 9.1px, above the script's own 5px warning threshold** —
diagnosed rather than dismissed or blindly trusted:
- Plotted `cx`/`cy` vs `dac_x`/`dac_y` directly: both look genuinely
  linear per axis, no visible kink or saturation region.
- Tested whether excluding the near-floor corner (`dac < 500`, close to
  the firmware's 95-count clamp) helped: modestly (9.1px → 7.6-8.0px),
  not enough to be the whole story.
- Tested whether more per-point averaging helped (`--capture-s` 0.15 →
  0.4, roughly doubling samples/point): barely (9.1px → 8.9px) — rules
  out plain measurement noise as the dominant cause.
- **Conclusion**: most likely real, smooth nonlinearity across this wide
  a sweep (~90% of the actuator's full clamped range), not a bug, not
  noise, not a sharp localized defect. Not surprising for a mechanical
  actuator swept across nearly its whole travel rather than around one
  small operating point — matches this project's own established
  "small-step regime is what real control looks like" lesson from the
  step-response work.
- 2x2 gain block determinant (-0.0004455) is small but **not** below the
  script's own 1e-6 near-singular threshold — the inverse map is usable,
  just not tight.

**Practical takeaway**: this wide-range matrix is a reasonable first-pass
decoupling estimate, not a final one. Once a real closed-loop operating
point exists, a narrower sweep centered there should fit much tighter —
same logic as why gain tuning should use small-step step-response
numbers, not the large-step ones. Both sweeps kept
(`results/fta_calibration_vcp_202608061912*.npz`) since together they
show the diagnostic story (200-3800/capture=0.15 first pass +
400-3800/capture=0.4 corner-and-noise check), not just the final matrix.

### Pushed sine tracking to 5/10/15/20Hz — real gain rolloff found, likely a plant-bandwidth problem for the 10-20Hz spec (2026-08-06)

Direct follow-up to the "RESOLVED" sine-tracking correction above: with a
trustworthy 0.1-2Hz analysis in hand, pushed frequency toward the actual
10-20Hz disturbance band this project needs to reject. Also raised
`--update-rate` (to 400Hz) as frequency increased, to keep the commanded
trajectory from degrading into a coarse staircase.

**Hit a second, related but distinct sign-ambiguity bug at 5Hz.**
`fit_sine()`'s auto-detected sign (whichever of 0°/180° puts the residual
phase within ±90°) misdetected axis x's known-negative gain as positive,
symptom: an impossible negative "lag" (a response can't precede its own
command). Root cause: real actuator dynamics stacking on top of the
~41ms pipeline delay established above pushed total phase past the ±90°
auto-detection boundary — expected, once frequency is high enough,
regardless of how the sign is computed. Fix: since a linear system's
static-gain *sign* is a fixed physical property (confirmed independently
by the step-response data and every 0.1-2Hz sine run — negative on this
axis, always), added `--gain-sign` to fix that reference instead of
re-deriving it per frequency — valid up to a full ±180° of real lag, not
just ±90°.

**Real result once fixed:**

| freq | gain (px / ±200 DAC counts) | lag |
|---|---|---|
| 0.1 Hz | -19.2 | 161ms |
| 0.5 Hz | -17.8 | 68ms |
| 1 Hz | -16.6 | 59ms |
| 2 Hz | -15.2 | 47ms |
| 5 Hz | -10.3 | 50ms |
| 10 Hz | -9.2 | unreliable — phase at the ±180° wraparound boundary |
| 15 Hz | -2.6 | unreliable |
| 20 Hz | -4.1 | unreliable |

**Gain drops to ~15-20% of its low-frequency value by 15-20Hz** — not a
gentle rolloff, a real collapse right at the frequencies that matter most
for this project. Lag above ~5-10Hz is explicitly not reported as a
number: at 10Hz the phase reaches the *fundamental* single-frequency
wraparound ambiguity (a real mathematical limit — a single test frequency
alone can't distinguish a lag from `lag ± n·period`), and by 15-20Hz the
raw traces (`docs/session_results_2026-08-04.pptx`, "Pushing toward the
10-20Hz disturbance band" slide) visibly stop looking like clean sinusoids
— signal is small enough that noise likely dominates whatever the fit
returns.

**Why this matters more than a tuning detail**: this is evidence about
the *plant* (actuator + optics), not the controller. If the actuator
itself can only produce ~15-20% of its low-frequency response at 10-20Hz,
no PID gain choice fixes that — the physical bandwidth ceiling sits below
where this project needs to operate. Not yet confirmed as a hard
blocker (only axis x tested; single-tone sine at fixed amplitude, not
the real 10-20Hz beacon wobble; camera-direct verification never done to
rule out any remaining test-method attenuation) — but real enough to
flag as a project-level open question before investing further in gain
selection, not just a footnote to route around.

**Not yet done**: axis y at these frequencies; confirming this isn't
still a residual test-method artifact (e.g. via a camera-direct sine test
removing the VCP relay, same rationale as `fta_step_response_test.py`
being kept as ground truth); deciding whether this changes the control
architecture (e.g. accepting partial rejection, revisiting the actuator/
optics hardware, or re-scoping the disturbance-rejection target) before
proceeding to PID gain selection and firmware implementation.

### Corrected to true displacement vector; fine 3-12Hz sweep finds real resonance/anti-resonance behavior, not simple rolloff (2026-08-06)

User question that triggered this: was the gain-rolloff analysis assuming
DAC-x displacement shows up as pure x-pixel motion, or measuring the
actual (rotated) displacement direction? Answer was the former — a real
gap. Every gain number up to this point was the driven axis's own pixel
projection only, ignoring the confirmed cross-axis coupling.

**Fixed**: `fta_sine_response_test_vcp.py` now also reports the true
displacement vector, `|vector| = sqrt(gain_x^2 + gain_y^2)` and its
direction `atan2(gain_y, gain_x)`, not just the driven axis's own
component. Correction is small at 0.1-2Hz (~1%, cross-coupling is modest
there) but 6-14% at 5-20Hz — doesn't overturn the rolloff conclusion, but
surfaces a real new finding: **the vector's angle rotates sharply between
2-3Hz** (steady ~-171.5° from 0.1-2Hz → ~-150 to -160° from 3Hz onward,
holding roughly steady through 20Hz) — a step change, not a gradual
drift. Evidence the x and y mechanical axes have measurably different
dynamics, not identical ones.

**Ran a finer 3-12Hz sweep (1Hz steps)** specifically to locate what's
happening there, instead of inferring it from 4 widely-spaced points
(0.1/0.5/1/2 then jumping to 5/10/15/20). Reused the 5Hz/10Hz slots with
fresh runs under identical conditions to the rest of the fine sweep
(same amplitude/update-rate) rather than mixing them with the earlier
separately-run data.

**Result is genuinely not a simple rolloff**: magnitude plateaus 6-10Hz
(~11-12px, barely below the 5Hz value), **dips sharply at 11Hz** (~8.4px),
**partially recovers at 12Hz** (~11.4px), then collapses by 15Hz. See
`docs/session_results_2026-08-04.pptx`'s "Fine sweep" slide for both
charts (magnitude and angle vs. frequency). A plateau-dip-recovery shape
is not what a simple single-pole (first-order low-pass) actuator would
produce — it's more consistent with resonance/anti-resonance interaction
from a higher-order or two-mode mechanical system, e.g. the x and y
flexure axes having close but distinct resonant frequencies whose
combined response creates constructive/destructive interference at
specific frequencies (matching the angle-rotation finding: different
axes, different dynamics).

**On whether a stiffer flexure would help (user question, same
conversation)**: yes in principle — flexure resonant frequency scales as
`f_n ∝ sqrt(k/m)`, so stiffening raises where the rolloff starts. But
three caveats, now sharper given the resonance finding: (1) stiffness
trades against low-frequency gain/sensitivity (current gain has
headroom — 19px per 200 counts against a ~4000-count usable range — so
this is probably an acceptable trade); (2) stiffness alone doesn't fix
damping, and the step-response ringing already on record suggests the
system may already be underdamped — could just move the same ringing to
a higher frequency; (3) **given the two axes now look mechanically
different, stiffening only one wouldn't address the other** — worth
identifying which axis is softer before deciding where to act.

**Not yet done**: separating the two apparent modes properly (this
sweep only ever drove axis x — driving y separately, and ideally at
matched fine resolution, would show whether the y axis has its own
distinct resonance/rolloff shape); confirming any of this against a
camera-direct measurement (same outstanding caveat as the rolloff finding
above — this is all still over the VCP relay path); using the resonance
location (once better pinned down) to inform any actual hardware change
before assuming a stiffer flexure is the right fix.

**Follow-up same session**: user pushed further — the high-frequency
DAC-vs-pixel plots (5/10/15/20Hz slide) still showed x-pixel and y-pixel
as two separate traces, implicitly treating x as "the" signal and y as
"cross-coupling noise." That framing only makes sense if the actuator's
axis is close to the camera's x-axis, which the angle finding above says
it isn't (~-150 to -172°, not ~0/180°). Fixed by projecting the raw
(x, y) trace onto each frequency's own fitted motion direction
(`x·cos(angle) + y·sin(angle)`) instead — the actual 1D displacement
along the axis the actuator really moves on, replacing the old two-trace
slide 10 in `docs/session_results_2026-08-04.pptx`. **Notable**: even
correctly rotated, the 10-20Hz traces still look noisy rather than
cleanly sinusoidal — reassuring, not concerning: projecting onto the true
axis maximizes captured signal (recovers the fitted magnitude exactly),
it doesn't reduce measurement noise, so this confirms the earlier "signal
genuinely weak at 15-20Hz" read wasn't an artifact of looking at the
wrong axis split.

### RETRACTED: the "angle rotation" finding was a batch artifact — and a clean amplitude comparison finds the 10-20Hz "rolloff" was mostly a nonlinear threshold effect, not a bandwidth ceiling (2026-08-06)

**User question that unraveled this**: could the 2-3Hz angle rotation
just be a sudden camera/rig shift rather than real frequency-dependent
dynamics? Good instinct — checked immediately rather than defended the
original read.

**Checked baseline offsets first**: mean pixel y-position jumped from
~478-480px (0.1-2Hz batch, run ~18:58) to ~379-386px (3-20Hz batch, run
~19:39 onward — *after* the two calibration sweeps at ~19:12-19:16 that
swing the actuator across nearly its full DAC range). Mean x shifted
too, ~15px. DAC-y was confirmed unchanged (95) throughout via the
printed status line on every run — this isn't a commanded-position
difference.

**Decisive test**: reran 2Hz standalone, well after the calibration
sweeps. Result: offset and angle matched the "high-frequency batch"
pattern (~-151°), **not** the original 2Hz result (~-171.5°) — same
frequency, different answer, on the same hardware, same session. This
proves the shift tracks *elapsed time / an intervening event* (most
likely the two calibration sweeps' large excursions — real mechanical
hysteresis/settling, or a Pi-side ROI recenter, not yet distinguished),
not test frequency. **The "two mechanical axes have different dynamics"
claim in the section above is retracted.** The root cause: every sine
test up to that point was run in increasing-frequency order over time,
so frequency and elapsed-time/intervening-disturbance were perfectly
confounded — a real methodology gap, not just an isolated bad reading.

**Real fix — clean resweep, one uninterrupted session, 3 amplitudes**:
reran the full 0.1→20Hz sweep (16 frequencies) three times back to back
with no calibration sweep or other large excursion in between, at
±200/±400/±800 DAC counts (all `--amplitude`, `center=2000`,
`--update-rate 400` throughout). This is the comparison that actually
matters:

| freq | ret. @200 | ret. @400 | ret. @800 |
|---|---|---|---|
| 0.1 Hz | 100% | 100% | 100% |
| 2 Hz | 80% | 84% | 81% |
| 5 Hz | 60% | 68% | 68% |
| 10 Hz | 57% | 69% | 61% |
| 15 Hz | **14%** | **82%** | **66%** |
| 20 Hz | **22%** | **61%** | **69%** |

(retention = % of that sweep's own 0.1Hz magnitude; full table and
`amplitude_comparison.png` in `docs/session_results_2026-08-04.pptx`)

**Finding: the severe 15-20Hz collapse was largely a small-amplitude
nonlinear threshold effect (stiction/backlash/dead-band), not a hard
bandwidth ceiling.** If the system were linear, doubling amplitude
200→400 should double the response at every frequency equally. That
roughly holds 0.1-9Hz (ratios 1.6-2.3×, consistent with local linearity
— matches the calibration sweep's own finding of reasonable local
linearity away from the full-range extremes). It breaks badly at 15Hz
(10.8× — not 2×) and 20Hz (4.9×) — exactly what you'd expect if a small
command can't generate enough force fast enough to break through some
threshold at high frequency, while a larger command has enough "push" to
get past it. **Bonus confirmation**: angle stayed stable (~-146 to
-157°) across all 16 frequencies in *both* clean amplitude sweeps —
independent confirmation the earlier rotation finding was a batch
artifact, not real.

**Not fully resolved — even at ±800 there's still real rolloff** (20Hz
retains only ~69%, not ~100%), so this doesn't mean "no bandwidth
concern at all," just a much smaller one than the ±200 data implied.

**Practical implication, genuinely open**: which amplitude regime is the
right one to design around depends on how large the actual beacon-wobble
disturbance is in DAC-equivalent terms — a number this project hasn't
established yet. If the real disturbance is small, the controller may
still be fighting this stiction/threshold effect in practice even though
the large-signal bandwidth looks much healthier than first measured —
classic territory for needing a small dither signal to keep the actuator
unstuck, a standard trick for exactly this kind of nonlinearity.

**Not yet done**: pinning down what physically caused the batch-to-batch
offset shift (calibration-sweep hysteresis vs. Pi-side ROI recenter vs.
something else — not yet distinguishable from the laptop side);
characterizing the threshold itself more precisely (e.g. an amplitude
sweep at a fixed high frequency like 15Hz, from 100 to 800+ counts in
steps, to find where the transition actually happens); getting a real
number for the expected beacon-wobble amplitude so the right regime can
be identified; axis y at any of this.

### FTA amplifier static voltage/power calibration, both axes — very linear, and a real travel-limit finding (2026-08-06)

Manually-measured (multimeter at the amp output, not an automated
script-driven sweep — unlike every other `fta_*.py` script):
`fta_amp_voltage_calibration.py` records DAC 200-4000 (step 200, 20
points) → amplifier output voltage, both axes, same 2.85Ω load. Y-axis
had one clear outlier (nominally DAC=2200, read 0.85V, breaking an
otherwise consistent ~-0.147V/200-count trend) — excluded as bad/missing
rather than guessed at. X-axis came back clean; its 10th value (.084V)
lands almost exactly where the Y-axis trend predicted its own excluded
point would have been — a nice independent check that the exclusion was
the right call.

**Both axes are extremely linear statically**: X R²=0.99999 (slope
-0.000732 V/count), Y R²=0.9971 (slope -0.000784 V/count) — no visible
kink or dead-band anywhere in the range. Also reports counts/volt
(X: -1365, Y: -1276) and counts/amp (X: -3891, Y: -3636, via R). Plots
carry a secondary top axis in volts (computed from each axis's own fit)
alongside the primary DAC-counts axis. **Useful negative result**: since
the DAC/amp stage is this linear when swept slowly, the dynamic
nonlinear threshold found in the sine-tracking amplitude comparison
(above) more likely lives in actuator mechanics (stiction/backlash) or a
frequency-dependent electrical effect, not simple DC nonlinearity in the
drive electronics.

**Follow-up (`fta_travel_range_analysis.py`), combining this with the
calibration sweep — a real, important mechanical finding**: plotted
centroid position against DAC counts (using the wider 200-3800
calibration sweep) alongside power (from this amp calibration's fits),
to see where the beam actually stops moving vs. where the amp is just
spending power. **X axis**: `cx` tracks `dac_x` linearly across the
*entire* tested range (200-3800) — no flattening at either end.
**Y axis**: `cy` rises steeply from `dac_y`=200→~500 (~0.028 px/count),
then flattens almost completely for the rest of the range 500-3800
(~0.003 px/count mid-range, ~0.0015 at the high end — a 94-95%
sensitivity drop) while power keeps climbing to ~850mW at the far end.
**Read: the Y axis's real useful mechanical travel is only about DAC
200-500/800 — a small fraction of the 200-3800 range that was actually
swept.** Past that, DAC-y commands spend power for essentially zero
additional beam movement, consistent with hitting a real mechanical
limit (flexure hard stop or similar) early. See
`docs/session_results_2026-08-04.pptx`'s two newest slides for the
plots.

**Practical implications, not yet acted on**: (1) any future Y-axis
calibration/PID work should probably restrict itself to the ~200-800
DAC range rather than the wider range used so far — commanding beyond
that is likely wasted effort; (2) this asymmetry between X (full-range
linear) and Y (saturates early) is a plausible mechanical explanation
for why Y's sine-tracking gain was consistently smaller than X's
throughout this session's testing, independent of the frequency-domain
findings; (3) worth checking whether X has an analogous limit just
outside the 200-3800 tested window, or whether it genuinely has much
more usable travel than Y.

### RETRACTED (2026-08-12): the Y-axis travel-limit finding above was a `set_x`/`set_y` race condition in `fta_calibration_vcp.py`, not a real mechanical limit

Asked to re-run the grid sweep after switching the camera to full-frame
mode (to test whether a too-narrow ROI explained the Y flattening).
First full-frame attempt had to be killed mid-sweep — the beam was
visibly clipping out the top of frame — but the follow-up full-frame run
completed 169/169 with no clipping, and STILL showed the same near-
degenerate mapping (`fta_calibration_grid_mesh.py`, a new script that
draws the commanded DAC grid as a polygonal mesh directly in centroid
space: connect points sharing `dac_y` as "rows", points sharing `dac_x`
as "columns" — a linear, well-conditioned actuator draws a rectangle;
this drew a near-degenerate sliver, 2x2 gain-block determinant
~0.0003-0.0004 in both the old windowed-ROI sweep and the new full-frame
one). That ruled out the ROI hypothesis but not the underlying finding —
until manual DAC control was compared directly: manual `set_x`/`set_y`
commands clearly moved the beam on both axes, but the automated grid
sweep visibly moved only one axis at a time, scanning back and forth
along a single line.

**Root cause, found by reading the firmware's VCP receive ISR**
(`HAL_UART_RxCpltCallback` in `nucleo_firmware/camera_centroid_receiver/Core/Src/main.c`):
it buffers exactly one pending command line at a time and silently drops
incoming bytes of a new command if the previous line's `vcp_line_ready`
flag hasn't been drained by the main loop yet — no error, no signal,
just dropped. `fta_calibration_vcp.py`'s sweep loop wrote `set_x` then
immediately `set_y` with zero delay and never read either reply. `set_x`
(sent first) almost always landed; `set_y`'s bytes frequently arrived
while `set_x`'s `OK` reply was still pending in the buffer and got
dropped, silently sticking `dac_y` while `dac_x` kept updating — exactly
the near-degenerate/one-axis-only grid observed in **every** prior
calibration sweep this session, including the one behind slide 18.
`fta_manual_control.py` was never affected because its `send()` helper
already waits for each command's reply before returning — which is
exactly why manual control looked fine while the automated sweep didn't.

**Fix**: added `send_command()` to `fta_calibration_vcp.py`, which writes
a command and blocks (skipping telemetry/heartbeat lines) until it sees
that command's `OK`/`ERR` reply before returning, guaranteeing the
firmware's one-line buffer is empty before the next command is sent.
Applied to both the main sweep loop and the park-at-exit in `finally`.

**Re-run with the fix, full-frame, same 200-3800/step-300 grid**
(`results/fta_calibration_vcp_fullframe_fixed.npz`): 2x2 gain-block
determinant jumped to **0.01** (~25-30x larger than either corrupted
run) and the grid mesh (`results/fta_calibration_grid_mesh_fullframe_fixed.png`)
is now a clean, evenly-spaced parallelogram — rotated relative to
camera x/y (consistent with the independently-derived ~-150° motion-
axis angle from the sine-tracking rotated-view slides, which were never
affected by this bug since they only ever send `set_x` in a loop), but
genuinely crossed and non-degenerate, no collapse on either axis.
Re-running `fta_travel_range_analysis.py` on the fixed data
(`results/fta_travel_range_analysis_fixed.png`) shows `cy` now declining
smoothly across the **entire** DAC-y 200-3800 range, closely mirroring
`cx`'s behavior — no flattening past ~500. **The Y-axis travel-limit
finding, and by extension the "restrict Y calibration to 200-800"
recommendation above, are both retracted.** RMS residual on the fixed
sweep is still large for a pure affine fit (18.42px, vs. ~8-9px on the
corrupted runs) — expected, since the fixed data now reflects the
actuator's real (mildly nonlinear/curved) response instead of a mostly-
flat one-axis line; a higher-order fit would likely do much better, not
yet tried.

**Any DAC↔centroid calibration data collected before 2026-08-12 via
`fta_calibration_vcp.py` should be treated as unreliable** and, if
still needed, re-collected with the fixed script.

**Confirmation re-run at ~4x dwell** (`--settle-s 1.2 --capture-s 0.6`
vs. the 0.3/0.15 default, to rule out settling-time noise as the source
of the still-large RMS residual): `results/fta_calibration_vcp_fullframe_fixed_longdwell.npz`.
Determinant 0.0116 and RMS 19.4px, both consistent with the 4x-shorter-dwell
fixed run (0.0101, 18.4px) — the ~19px affine-fit residual is real
actuator curvature, not measurement noise from under-settling.

**Camera physically repositioned, grid re-swept again** (2026-08-12,
`results/fta_calibration_vcp_fullframe_adjusted.npz`, same fixed script,
long dwell): gain matrix is now close to diagonal — off-diagonal terms
(~0.001-0.012) are roughly 10x smaller than the diagonal terms (~0.11),
vs. the previous camera position where they were comparable magnitude
(a ~45°-ish rotation). Grid mesh (`results/fta_calibration_grid_mesh_adjusted.png`)
is now nearly axis-aligned with camera x/y — `dac_x` mostly drives `cx`,
`dac_y` mostly drives `cy`. Both axes track smoothly across the full
200-3800 range (`results/fta_travel_range_analysis_adjusted.png`); Y
shows a mild ~3x gain reduction in just the bottom segment (200-500),
nothing like the previous (retracted) 60-90x collapse. This is now the
best-aligned calibration in hand — worth using as the baseline for
picking Kp/Ki once that work resumes.

**Optics recollimated, grid re-swept again** (2026-08-12,
`results/fta_calibration_vcp_fullframe_recollimated.npz`): gain matrix
flipped to near-**anti**-diagonal — `dac_x` now barely moves `cx`
(-0.0069) but strongly moves `cy` (0.130); `dac_y` barely moves `cy`
(0.0014) but strongly moves `cx` (-0.107). Determinant 0.0139, same
order of magnitude as the previous (diagonal) calibration, and the grid
mesh (`results/fta_calibration_grid_mesh_recollimated.png`) is still a
clean, evenly-spaced, non-degenerate parallelogram — just rotated
~90° from the post-repositioning run above. **Important**: recollimation
changed which DAC axis drives which camera axis, and did so in the
opposite sense from the earlier repositioning (that one made the mapping
*more* diagonal; this made it anti-diagonal). Any control code must use
the fitted `M`/`M_inv` from the *current* calibration rather than
assuming `dac_x`→`cx`/`dac_y`→`cy` — that identity does not hold right
now. Slide-18-style single-axis plots (`cx` vs. `dac_x` alone) are
actively misleading for this calibration since the real per-axis signal
now lives almost entirely in the *other* pixel axis; use the grid-mesh
plot instead when checking alignment quality going forward.

### Optics locked down, final reference calibration taken, first control axis chosen and sine-checked (2026-08-12)

Final grid sweep (`results/fta_calibration_vcp_final.npz`, same fixed
script/full-frame/long-dwell protocol) after locking the optics down —
matrix confirms the same near-anti-diagonal mapping as the last
recollimation check, consistent/stable: `dac_y`'s effect on `cx` is
**+0.126 px/count, the single largest coefficient in the matrix** (vs.
-0.007 for `dac_x`→`cx`, -0.104 for `dac_x`→`cy`, +0.006 for
`dac_y`→`cy`). Determinant 0.0131, clean evenly-spaced grid mesh
(`results/fta_calibration_grid_mesh_final.png`), both axes' travel
consistent across the full range (no saturation).

**Chosen first control axis: `dac_y` → `cx`** (actuator's Y drives the
camera's X), matching this coefficient and the plan to start with 1D
PID. Sine-checked at amplitude 400 (the amplitude previously found to
avoid the small-signal threshold nonlinearity), 5/10/15/20Hz, gain-sign
fixed to +1.0 (`fta_sine_response_test_vcp.py --axis y`, since this
pathway's sign is positive — opposite the old axis-x default of -1.0 that
every prior sine test used, so `--gain-sign` must be passed explicitly
now). Driving `dac_y` moves `cy` (the "driven" axis in the script's own
terminology) almost not at all (+0.5-3.4px) while moving `cx` (the
"cross-coupled" axis) strongly — confirms the pathway is clean:

| freq | true displacement magnitude | direction (0°=pure camera-x) |
|---|---|---|
| 5Hz | 29.0px | 1.0° |
| 10Hz | 24.6px | 3.2° |
| 15Hz | 19.1px | 6.4° |
| 20Hz | 19.4px | 10.0° (only 100 telemetry samples this run, script's own low-sample-count warning fired — lower confidence than the other three) |

Only a mild rolloff across the whole 5-20Hz band (not the severe
small-amplitude collapse found earlier at amp=200) and the signal is
still strong at 20Hz — this pathway looks usable for the actual
disturbance-rejection target band. Direction drifting from ~1° to ~10°
with frequency is a small, secondary effect (likely relative timing
skew between how `cx`/`cy` get sampled/fitted, not a real alignment
problem) — worth a closer look eventually but not blocking.

Raw data: `results/fta_sine_response_vcp_y_check2Hz.npz`,
`results/fta_sine_response_vcp_y_final_{5,10,15,20}Hz.npz`.

**Re-run at full ROI telemetry rate — the full-frame numbers above were
degraded, superseded** (2026-08-12): the above sine check ran while the
Pi's camera was still in full-frame mode (left over from the final
calibration sweep, which needed the wide FOV). Full sensor detection is
known-slow (see "fps root-cause" section below: ~45fps at full sensor
vs. ~527fps binned/windowed) and the 20Hz run above had already thrown
the script's own low-sample-count warning — a real red flag. Switched
the Pi back to `camera_view_tool.py`'s own default fast mode
(`640x200`, one `h` keypress) and confirmed the improvement directly
from the Nucleo's own relay counter before re-testing: `pkts` delta
over 2s went from full-frame's effective ~45-50Hz to **~207Hz**,
`tel_age_ms` staying at 0-10ms.

Re-ran the same 5/10/15/20Hz / amplitude-400 sweep
(`results/fta_sine_response_vcp_y_roi_{5,10,15,20}Hz.npz`) — materially
different, more trustworthy result:

| freq | true displacement magnitude | direction (0°=pure camera-x) |
|---|---|---|
| 5Hz | 26.8px | 0.9° |
| 10Hz | 26.5px | 1.1° |
| 15Hz | 24.5px | 0.1° |
| 20Hz | 33.8px | 0.5° |

Direction now stays pinned to ~0-1° at every frequency (vs. drifting
1°→10° in the degraded full-frame run — that drift was a sampling
artifact, not a real alignment effect). Magnitude is roughly flat
5-15Hz then **rises** at 20Hz rather than declining — consistent with
the resonance/anti-resonance behavior already found on the old axis-x
pathway (see "fine 3-12Hz sweep" section below), not the full-frame
run's misleading monotonic-rolloff shape. No low-sample-count warning
this time. **Treat the full-frame numbers in the section above as
superseded by this table** — kept for the record, not as the reference
data going forward.

Plotted (`fta_sine_response_plot.py`): `results/fta_sine_response_y_roi_traces.png`
(commanded `dac_y` vs. measured `cx`, one column per frequency, same
visual style as the earlier slide-10 rotated-axis plots) and
`results/fta_sine_response_y_roi_summary.png` (magnitude/direction vs.
frequency, makes the flat-then-rises-at-20Hz shape and the pinned-near-0°
direction easy to see at a glance).

**Microns added** (2026-08-12): all pixel-based displacement/gain
numbers going forward can be converted to real physical units via
`MICRONS_PER_PIXEL = 3.0` (OV9281 pixel pitch, confirmed live via
`Picamera2(0).camera_properties["UnitCellSize"]`, same constant already
used in `fta_step_response_test.py`). This applies directly and
uniformly to every VCP-relayed x/y sample this project produces,
**regardless of which capture mode the Pi's camera is in** (full-frame
or a binned windowed mode like 640x200) — the Pi-side streamer
(`camera_view_tool.py`/`beam_position_streamer.py`) already multiplies
its detected centroid by `v_bin` before sending over I2C, so what
arrives over the relay is always in native/pre-bin-pixel-equivalent
coordinates. No separate binning correction is ever needed on the
laptop side. Also 1:1 to real fiber-tip displacement, since the optical
path has no external magnification. Added to
`fta_sine_response_test_vcp.py` (µm alongside px in the printed
gain/lag/offset/vector report), `fta_sine_response_plot.py` (µm
secondary axes), and `fta_calibration_grid_mesh.py` (µm secondary axes
+ µm in the row/column travel printout) — e.g. the final calibration's
full grid spans ~1088-1124µm of travel per dac_x sweep and
~1275-1363µm per dac_y sweep (`results/fta_calibration_grid_mesh_final.png`).

**Next step**: implement the actual PID loop on the Nucleo (`cmd_set_mode`'s
`closed_loop` branch is currently a deliberate stub, `ERR closed_loop not
yet implemented` — nothing PID-related exists in firmware yet: no
setpoint command, no gain storage, no control loop). Plan: single axis
first (`dac_y` against a `cx` pixel setpoint), add a setpoint command +
live-tunable Kp/Ki + the loop itself + a telemetry-staleness fail-safe,
bench-test with a step setpoint before a sine setpoint, then work up to
10-20Hz. See the assistant's response in-session (2026-08-12) for the
fuller architecture writeup if picking this up fresh.

### First closed-loop PID bench test — diverged twice, root-caused to real actuator hysteresis (not a sign/rate bug), then converged on the confirmed-clean branch (2026-08-13)

Picked up directly from the "Next step" above. Found the single-axis
P+I closed-loop implementation already written in `main.c`
(`MODE_CLOSED_LOOP`, `cmd_set_target_x`/`cmd_set_kp`/`cmd_set_ki`,
`run_closed_loop_step`) sitting **uncommitted** in the working tree from
a prior session that never got logged here — `main.c` last saved
2026-08-12 17:42, `Debug/camera_centroid_receiver.elf` built one minute
later. Rebuilt from source with the project's own bundled toolchain
(same bypass-CubeIDE approach as "Firmware phase 1") to confirm the
`.elf` genuinely matched current source (`make` reported nothing to
rebuild — it already did), then flashed it for real via
`STM32_Programmer_CLI` (board serial `066FFF515152827187153930`,
confirmed against this file's own on-record serial) so what's running on
the chip is verified, not assumed.

**Real bug #1, found before any PID testing could even start: a single
`ser.write()` burst of a whole VCP command line reliably loses/corrupts
bytes under the Pi's current high telemetry rate (~150-200Hz).**
`get_status` consistently arrived as `getsa` — not random noise, the
*exact same* corruption reproduced across many independent attempts,
before and after reflashing, and across two completely different serial
stacks (.NET `SerialPort` and `pyserial`). Isolated with the bare `!`
e-stop byte (ISR-level, bypasses the line parser entirely): it landed
cleanly every time (`estop=1` confirmed via heartbeat), proving raw
byte-level UART RX works fine — the bug is specific to the multi-byte
line path. Pacing the write at ~20ms/character instead of one burst
call made the corruption disappear completely (confirmed repeatedly).
Root cause not fully instrumented, but consistent with the whole
~11-byte/~1ms burst window occasionally landing on the main loop's
`__disable_irq()` telemetry-snapshot critical section (see the
`g_new_packet_ready` handling in `main()`) — human/manual typing over a
terminal is naturally paced well past this, which is almost certainly
why earlier sessions' "30/30 commands clean" validation never caught it.
**Not fixed in firmware — a host-side workaround only.** This affects
the *existing* `fta_manual_control.py` and `fta_calibration_vcp.py` too
(`send_command`/`send` in both do a single-burst `ser.write()`), not
just new tooling — worth porting the paced-write fix into those if this
keeps biting.

**First closed-loop attempt — diverged almost immediately.** Baseline
`cx≈251.6`, `set_target_x` +25px, `set_kp 1750` (Kp=1.75 counts/px),
`set_ki 0`, `amp_enable`, `set_mode closed_loop`. Within under a second
`dac_y` climbed 95→417+ and `cx` crashed the *wrong* direction (251.6→
~90) instead of toward the target. A host-side monitoring script (not
committed, scratchpad-only) watching for DAC-clamp/divergence caught it
and sent the bare `!` e-stop within ~3.2s — `dac_y` peaked at 426, well
short of the `[95,4000]` clamp, no real risk to hardware.

**Added a 25Hz control-update throttle to firmware, suspecting the
control step (previously run on every confident telemetry packet,
~150-200Hz) was outrunning the actuator's own settling time** — new
`CONTROL_INTERVAL_MS` (40U) in `main.c`, gating the
`run_closed_loop_step` call site in `main()`'s `while(1)` loop against
`g_last_control_tick`. Rebuilt, reflashed, retested with identical
Kp/target: **nearly identical divergence numbers.** This ruled out
update rate as the cause — the throttle is still a reasonable thing to
have (25Hz is comfortably above the 10-20Hz disturbance band this
project targets, comfortably below anything that could outrun this
actuator), just not what was actually wrong.

**Root cause found: real, substantial hysteresis in `dac_y`→`cx`, not a
sign or timing bug.** An open-loop sweep first at coarse resolution
(200-450, settled single reads) showed a clean positive slope and looked
like sign confirmation — but it never sampled below 200. A follow-up
fine sweep (25-count steps, 95→600, ~40 telemetry samples averaged per
point, both directions, all via the paced-write fix above, scratchpad
scripts not committed) told the real story:
- **Ascending** (95→600): a clear U-shape. `cx` falls from 67.7 at
  `dac_y=95` to a minimum of 60.4 around `dac_y≈195`, then rises
  smoothly the rest of the way to 88.0 at 600.
- **Descending** (600→95): clean and monotonic across the *entire*
  range, no reversal — 88.3 falling smoothly to 42.6.
- The gap between the two branches at the same commanded `dac_y` grows
  toward the floor: ~11.5px at `dac_y=200`, ~25px at `dac_y=100`. Both
  curves are individually smooth (well-averaged, not noise) — this is
  real mechanical hysteresis/backlash, worst near the DAC floor.

This directly explains attempt #1: starting at the floor (95) with a
positive error, the controller's first correction raised `dac_y` into
exactly the ascending branch's reversed-slope region (95-195), driving
the error the wrong way from the very first step — a plant nonlinearity,
not a firmware bug. It also fills a gap `fta_calibration_vcp.py`'s own
docstring already flagged as untested ("Not yet tested for hysteresis").
Raw sweep data lives only in this session's scratchpad script output,
not saved as a committed `results/*.npz` — worth a proper committed
script if this needs to be trusted/reused long-term.

**Second attempt — pre-positioned to `dac_y=300`** (inside the range the
coarse check had called "positive"). Result: bounded, no clamp/runaway
this time, but also **didn't converge** — error crept from 26.5px to
30.5px over 8s, and `cx` drifted the *wrong* direction in response to
real `dac_y` changes (300→353) despite this nominally being the
"confirmed good" zone. Demonstrates the transfer function has real local
wiggle even inside that zone at fine (50-count) resolution — a coarse
±150-count check isn't reliable enough to trust for gain selection here.

**Third attempt — SUCCESS (partial): pre-positioned to `dac_y=550`, then
picked a target BELOW the current `cx`** so the controller's error is
always negative and it only ever needs to *decrease* `dac_y` — staying
entirely on the confirmed-clean descending branch. Same Kp=1.75
counts/px, Ki=0. Result: monotonic, bounded convergence — error shrank
from -18.4px to -13.0px over 10s, `dac_y` held in [519,528], no
divergence, no clamp. Slow (expected: P-only leaves a real steady-state
residual without Ki), but the **first genuinely-converging closed-loop
run of this project's PID implementation.**

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`). Firmware source has two uncommitted changes as of
this entry: the original closed-loop P+I implementation (from the
unlogged prior session) and this session's `CONTROL_INTERVAL_MS` 25Hz
throttle — both rebuilt clean and reflashed to real hardware, confirmed
running via `get_status`, not just built.

**Not yet done**:
1. Ki still untried (every run above was P-only) — needed to eliminate
   the residual steady-state error seen in the successful run.
2. Kp could likely go higher now that a clean branch is confirmed, for
   faster convergence — not explored.
3. **Real hysteresis handling is still an open design question.** "Only
   ever approach the target from above" is a workaround that made this
   one test converge, not a real control strategy — an actual
   disturbance-rejecting controller must handle error in both
   directions, which means either a dead-band/dither scheme, a
   hysteresis-aware model, or confining real operation to a region where
   the effect is small enough to ignore. Needs a decision before this
   goes further, not more gain guessing.
4. The VCP burst-write corruption fix (pace at ~20ms/char) has not been
   ported into `fta_manual_control.py`/`fta_calibration_vcp.py` — both
   still do a single-burst write and will hit the same corruption under
   high telemetry load.
5. ~~Firmware changes described above are still uncommitted~~ — done,
   see below.

**Follow-up, same day: throttle removed, retested at full telemetry
speed — confirms the throttle was never the actual fix.** Per the user's
request, reverted `CONTROL_INTERVAL_MS`/its call-site gate entirely
(removed rather than left dead/disabled, since the divergence theory it
was built on turned out to be wrong) so `run_closed_loop_step` runs on
every confident packet again (~150-200Hz). Rebuilt, reflashed, reran the
exact same pre-position-550/target-below/Kp=1.75/Ki=0 test: **nearly
identical convergence** — error -18.1px → -12.9px over 10s, `dac_y` held
in [519,528], no divergence. Confirms conclusively that update rate was
never the cause; staying on the hysteresis-clean descending branch is
what makes this converge, independent of control rate. Hardware left
idle (`amp=0 estop=0 dac_x=95 dac_y=95`); firmware (PID implementation,
now without the disproven throttle) committed to the repo.

**Open question raised the same session: does the hysteresis mean this
can never track the real beacon disturbance?** Answered, not yet tested
empirically — worth flagging clearly for whoever picks this up: every
hysteresis measurement so far (fine sweep, all three closed-loop
attempts) only ever moved `dac_y` in ONE direction at a time over a wide
excursion (the sweep) or approached a target monotonically from a fixed
starting side (the successful closed-loop runs). **Real disturbance
rejection needs the controller to correct in BOTH directions repeatedly
at small amplitude** — the 10-20Hz beacon wobble this project targets,
not a single one-way step. Hysteretic actuators very commonly behave
much better on these small "minor loops" than the wide "major loop"
characterized here (a standard property of magnetic/mechanical
hysteresis, e.g. Preisach-type models) — but that hasn't been measured
on this rig. The honest answer is "probably not a dead end, but
unconfirmed": if minor-loop hysteresis at small amplitude turns out to
be comparably wide, standard mitigations exist (a small dither signal to
keep the actuator off the sticky region — already flagged as a
candidate in the amplitude-comparison sine-tracking work earlier in this
file; or operating with a deliberate one-sided bias/preload) but neither
has been tried. **Recommended next diagnostic**: repeat the fine sweep
but with small (~10-20 count) back-and-forth oscillations superimposed
at a few points across the range, instead of one big monotonic pass each
direction, to directly measure minor-loop width where real control
actually has to operate.

### Minor-loop hysteresis measured — tight and correctly-signed everywhere checked; a well-behaved operating point found near mid-range (dac_y≈2048); first real PI convergence, including a target reversal (2026-08-13, same day)

Ran the recommended diagnostic from above: at four base points (200,
300, 400, 500), traced a small closed loop (`base` → `base-20` →
`base` → `base+20` → `base`, ~40 telemetry samples averaged per
reading, same paced-write approach as the rest of this session) and
compared `cx` arriving at `base` via a small ascend vs. a small descend.
**Minor-loop gap was under 1px at every point** (500: -0.84px, 400:
-0.37px, 300: -0.38px, 200: -0.25px), with every leg moving in the
*correct, consistent* direction — a dramatic contrast with the ~10-25px
major-loop gap found earlier. This is the standard "minor loop much
tighter than major loop" signature of real mechanical hysteresis, and it
directly answers the open question above: **small bidirectional
corrections behave close to linearly away from the DAC floor, so the
major-loop hysteresis does not look like a fundamental blocker for real
disturbance rejection** — as long as the controller's operating point
stays clear of the floor's U-shaped region (roughly below `dac_y≈200`).

**Extended to a completely unexplored region: dac_y≈2048** (middle of
the full DAC range — every prior test this session, calibration or
control, stayed inside 95-600). Same minor-loop technique at base=2048,
delta=40: gap **+0.04px** — essentially zero — with consistent slopes on
both legs (down: +0.0786 px/count, up: +0.0981 px/count). This is the
cleanest, most linear region found anywhere in this session.

**Bidirectional closed-loop test at dac_y=2048, P-only (Kp=1.75
counts/px, Ki=0)** — the first test this session to require the
controller to correct in *both* directions rather than approach a target
from one consistent side. Pre-positioned to 2048, set a target 25px
below baseline: `dac_y` moved 2048→2009 and held there, error settling
at a constant -22.4px. Then flipped the target to 25px *above* baseline
while still in `closed_loop` (no mode toggle, no re-bias): `dac_y`
immediately reversed direction, 2009→2083, holding steady at +20.0px.
No oscillation, no overshoot, no divergence, no clamp — the reversal was
handled correctly and immediately. The steady, non-zero residual in both
phases isn't a hysteresis artifact: it's the textbook P-only
steady-state offset, and the numbers confirm it exactly
(`dac_y = base(2048) + Kp×error`, e.g. `2009 = 2048 + 1.75×(-22.3)`) —
expected with `Ki=0`, not evidence of anything wrong with the plant.

**Added a conservative `Ki` (0.5 counts/(px·s), `set_ki 500`) and
re-ran the identical bidirectional test, 14s per phase.** Real PI
behavior this time — error decayed **monotonically and without
oscillation** in both directions instead of sitting at a flat P-only
equilibrium: phase 1 (target below) went from -22.6px to -12.5px over
14s; after the reversal, phase 2 (target above) went from +31.3px to
+17.4px over 14s. `dac_y` moved smoothly the whole time (1908→2121
across both phases), no sign-flip/overshoot observed in either phase, no
divergence, no clamp. Neither phase fully reached the <3px convergence
threshold within the 14s window — the conservative Ki was deliberately
slow — but the decay is clean and monotonic in both directions, which is
the real result: **this is the first genuinely-working bidirectional PI
closed-loop behavior in this project**, not just a one-way approach like
every earlier successful run this session.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`). Firmware's live `kp_milli=1750`/`ki_milli=500`
persist in RAM (not flash) until the next boot or explicit change — a
fresh flash or power cycle resets them to 0.

**Not yet done**:
1. Full convergence to <3px wasn't confirmed within the tested window —
   would need either a longer run or a larger Ki to actually watch it
   settle, not just trend toward zero.
2. Kp itself (1.75 counts/px) is low relative to the measured local
   plant gain at 2048 (~0.09 px/count → a matching P-only Kp would be
   roughly 11 counts/px) — likely could be raised substantially now that
   a clean, linear region is confirmed there, for much faster response.
   Not tried yet.
3. Only the `dac_y`→`cx` pairing and only step/quasi-static setpoints
   have been tested. No dynamic-bandwidth validation (sine tracking,
   10-20Hz) has been done against this new PI implementation — everything
   above is slow step-response behavior, not a real disturbance-rejection
   demonstration yet.
4. Regions other than 95-600 and the immediate neighborhood of 2048
   remain uncharacterized — no claim is made about anywhere else in the
   full [95,4000] range.
5. Scratchpad-only scripts this session (minor-loop check, 2048 checks,
   both closed-loop tests) are not committed — worth turning into a real
   committed tool if this hysteresis-aware operating strategy is kept
   long-term.

### Ki escalation search at dac_y=2048 — no overshoot found up to 200x the original value; settled on Ki=30 for ongoing work (2026-08-13, same day)

Followed up on item 2 above (Kp/Ki not yet pushed) by escalating Ki
specifically, to find where overshoot actually starts — useful to know
the real margin rather than just picking a number that happened to work
once. Single-direction step test per trial (target 25px below baseline,
Kp fixed at 1.75 counts/px), re-entering `closed_loop` fresh each trial
(resets the integral + rebiases via bumpless transfer) so trials don't
contaminate each other.

**Result: no meaningful overshoot anywhere in the tested range, all the
way up to Ki=300 counts/(px·s) — 200x the first working value (1.5).**
Convergence just kept getting faster with no stability cost:

| Ki (counts/px/s) | time to converge (\|err\|<3px) |
|---|---|
| 1.5 | not fully converged in 12s |
| 3.0 | 8.4s |
| 5.0 | 4.9s |
| 8.0 | 3.7s |
| 12.0 | 2.3s |
| 20.0 | 1.6s |
| 40.0 | 1.1s |
| 80.0 – 300.0 | ~0.36s (flat from here) |

Every trial's error stayed monotonic, no real sign-flip past ~0.2px
(noise-level), no divergence, no DAC clamp.

**Why it never overshoots, even at absurd gains — a real, useful
mechanism worth understanding, not just an empirical curiosity.** The
firmware's anti-windup clamp in `run_closed_loop_step` bounds the
integral state to `max_integral = DAC_range / Ki`, which means the
*maximum possible* contribution from the integral term is always
`Ki × max_integral = DAC_range` — a constant, independent of Ki.
Cranking Ki doesn't let the integral term push harder, it just lets it
reach that same fixed ceiling faster. This is a genuinely good
anti-windup design already present in the firmware (see the "Anti-
windup" note in `run_closed_loop_step`'s own docstring) — it
structurally prevents integral-driven overshoot by construction, which
is exactly why this search never found a break point. **Practical
implication**: overshoot risk in this system, if there is one, will not
show up in a quasi-static step-and-hold search like this one — it would
have to come from Kp/plant-dynamics interaction under an actual
disturbance (the real 10-20Hz band), not from the integral term. Not
worth pushing Ki any higher for this reason.

**Chosen for ongoing work: Ki=30 counts/(px·s) (`set_ki 30000`)** —
comfortably inside the flat, fast, zero-overshoot region (converges in
~1-1.5s), with margin on both sides. Kp stays at 1.75 counts/px for now
(still untouched since the very first attempt, despite item 2 above
noting it's likely well below what the plant could support at 2048 —
still not explored).

**Not yet done**: everything from the "Not yet done" list above still
applies (no dynamic/sine validation of the closed loop yet, Kp untried
at higher values, other DAC regions uncharacterized, scripts
uncommitted). This session's escalation-search script
(`ki_overshoot_search.py`, scratchpad only) reuses the same
paced-write/get_status-polling pattern as the rest of this session's
tooling — not proper high-rate logging or plotting, just a terminal
readout. **Next planned step (per user, same session): a proper
closed-loop step-response test with real data logging and a plot**
(matching the rigor of the existing open-loop `fta_step_response_test_vcp.py`,
rather than this session's crude ~3Hz terminal-polling checks) at
Kp=1.75/Ki=30, before moving on to closed-loop sine tracking across the
actual 5-20Hz disturbance band — the step test first, per this
project's own established methodology (see "PI control law designed,"
2026-07-23: step-response dynamics before sine, same logic applied here
to the closed loop instead of the open-loop plant).

### `fta_closed_loop_step_response_vcp.py` built and committed — real closed-loop step-response data, plus two real analysis bugs found and fixed (2026-08-13, same day)

Built the closed-loop equivalent of `fta_step_response_test_vcp.py`:
pre-positions to `--base-dac-y` (default 2048, this session's cleanest
region), primes `target_x` to baseline (zero-error hold), engages
`closed_loop`, records via the same telemetry-relay reader-thread pattern
as the open-loop script, steps `target_x` by `--step-px` (default -25),
records `--post-s` more, then computes rise time / overshoot / settling
time via a (fixed, see below) `analyze_step` and saves both a raw
`results/*.npz` and a plot.

**Every setup command uses the paced-write fix from earlier in this
session** (`send_command`, ~20ms/char) rather than a single burst —
necessary given the demonstrated corruption risk under the Pi's current
telemetry rate. **The measured step itself does NOT use a bare burst
write either, and this mattered on the very first live run**: an initial
version sent the step as one `ser.write()` burst (to preserve precise
step-onset timing the way the open-loop script does) and added a
post-recording `get_status` check to verify `target_x` actually changed
as a safety net. First real run: the safety net fired for real — the
burst-written step command silently lost bytes, `target_x` never
changed, and the "run" was just 3.5s of flat noise. Not a hypothetical
risk once again, a real failure on the first try. Fixed by pacing the
step command's write too (still not reading a reply during it, to avoid
racing the reader thread's exclusive `ser.readline()` for the whole
recording window — `t_step` is stamped after the last character is sent,
not after a confirmed reply). Rerun immediately after: `target_x`
verified correct, clean data. **Practical lesson for any future VCP
scripting on this rig**: pace every write, including one-off "the thing
being measured" commands, not just interactive setup commands — a burst
write is not safe here even for a single carefully-timed send.

**Second real bug, found from the data itself, not anticipated:** the
very first paced-step run reported "rise time: could not be determined"
despite clearly-real, clean-looking data (delta -25.9px, 0% overshoot,
settling in ~1015ms). Traced to `analyze_step`'s `first_crossing` helper
— duplicated from `fta_step_response_test_vcp.py`'s own version, which
conditionally flips the crossing comparison (`frac >= target` vs.
`frac <= target`) based on whether `delta` is positive or negative. That
conditional is wrong: `frac = (v - baseline) / delta` is already
sign-normalized by the division, so it climbs 0→1 as the signal moves
from baseline to final **regardless of whether the raw value is rising
or falling** (hand-verified with a synthetic example, and against this
script's own real falling-step data). With the old conditional, any
falling step (every closed-loop test this session steps in the negative
direction) has `first_crossing(0.90)` triggering on the very first
post-step sample — any frac below 0.90, including near-zero opening
noise, satisfies `frac <= 0.90` — landing `t90` before `t10` and hitting
the `t90 >= t10` guard, silently returning `None` every time. **Same
root issue affects the overshoot calculation** (`-np.min(frac)` for
`delta<0` measures something unrelated to the real peak and could
silently under-report real overshoot) — fixed the same way, to a plain
`np.max(frac)` in both cases. Both fixes only changed `first_crossing`
and the overshoot line; baseline/final/settling-time logic was already
correct and untouched. **This same latent bug likely exists in
`fta_step_response_test_vcp.py` and `fta_step_response_test.py`**
(neither has been patched — out of scope for this entry, but worth
knowing: any of their historical "rise time unresolved" results for a
falling/negative step should be treated as possibly this bug, not
necessarily the "detection confidence gate drops fast frames" explanation
recorded elsewhere in this file for the open-loop tests. The two
explanations aren't mutually exclusive and haven't been distinguished).

**Real closed-loop step-response numbers, Kp=1.75/Ki=30, -25px step at
dac_y=2048** (`results/fta_closed_loop_step_response_vcp_20260813T214602Z.{npz,png}`):

| metric | value |
|---|---|
| rise time (10-90%) | 891ms |
| overshoot | 0.4% (noise-level) |
| settling time (within 2px / 6µm) | 1000ms |

Clean single S-curve in the plot, no ringing, no oscillation — matches
the qualitative behavior from the interactive Ki-escalation search
earlier this session. **Worth flagging**: this closed-loop settling time
(~1000ms) is noticeably *slower* than the open-loop plant's own
small-step settling times measured much earlier in this project
(45-470ms, "PI control law designed" section, 2026-07-23) — consistent
with the already-noted observation that Kp=1.75 is likely well below
what this plant could actually support at 2048 (~11 counts/px would
match the measured local gain there, vs. the 1.75 used throughout this
session) — the control loop itself, not the actuator, is the current
speed bottleneck. Kp has still never been increased from its original
value; that's the natural next lever now that a real quantified baseline
exists to compare against.

**State left**: hardware safely idle. Script committed at repo root
(`fta_closed_loop_step_response_vcp.py`), results committed
(`results/fta_closed_loop_step_response_vcp_20260813T214602Z.{npz,png}`).

**Not yet done**: Kp untried at higher values (see above); no repeats
for statistical confidence (n=1); positive-direction steps untested with
this script (only the -25px convention used all session); closed-loop
sine tracking across 5-20Hz — the actual next planned step per the prior
entry — not started.

### Real tuning pass with the new step-response script — 1000ms settling was the control loop under-using Ki, not a plant limit; a real correction to the earlier "no overshoot up to 300" claim (2026-08-13, same day)

Prompted by the user asking, reasonably, why 1000ms settling (the
`fta_closed_loop_step_response_vcp.py` baseline result above) wasn't
"absurdly slow" given the project's actual 10-20Hz disturbance target (a
50-100ms period). Root cause, worked out from numbers already on record
before touching hardware: Kp=1.75 counts/px was never really tuned — it
was a conservative guess from very early in this session, before the
2048 operating point or its local gain (~0.09 px/count, from the
minor-loop check) even existed. A Kp matched to that gain would be
`1/0.09 ≈ 11 counts/px`; at the actual Kp=1.75, the P-term only ever
supplies about 16% of the correction a 25px step needs (`1.75×25≈44`
counts vs. the `≈278` counts the plant actually needs) — the rest has to
come from the slow-accumulating Ki term, which is why the response was a
gradual, Ki-dominated S-curve rather than a fast P-driven jump.

**Escalated Kp and Ki together using the new script, comparing real
plotted results (not the earlier interactive polling):**

| Kp (counts/px) | Ki (counts/px/s) | rise | overshoot | settling | shape |
|---|---|---|---|---|---|
| 1.75 | 30 | 891ms | 0.4% | 1000ms | clean, slow (the original baseline) |
| 5.0 | 30 | 984ms | 1.1% | 1125ms | visible ringing, **not faster** |
| 1.75 | 100 | 234ms | 1.4% | 265ms | clean |
| 5.0 | 100 | 16ms | 10.2% | 375ms | real ringing |
| **1.75** | **200** | **141ms** | **1.1%** | **141ms** | **clean, single transition** |
| 1.75 | 400 | 32ms | 15.1% | 63ms | real, visible oscillation |

**Finding 1 — Ki was the actual bottleneck the whole session, not Kp.**
Raising Kp alone (5.0 vs. 1.75, same Ki=30) made the response worse, not
better: real oscillatory ringing appeared right at the step, but overall
settling barely changed (1125ms vs. 1000ms) — because Ki's own
accumulation rate, unchanged, still dominated the slow tail. Raising Ki
alone (same Kp=1.75) is what actually worked: 30→100→200 took settling
from 1000ms→265ms→141ms, each time staying clean with no visible
ringing. **Chosen going forward: Kp=1.75 (unchanged), Ki=200** — a ~7x
speedup from the original baseline, still a single clean transition, and
comfortably below where Ki itself starts causing real overshoot (see
Finding 2). Set as this script's new default.

**Finding 2 — a real, useful correction to the earlier interactive Ki
escalation search** ("Ki escalation search at dac_y=2048," earlier this
same day): that search's headline claim — no overshoot found up to
Ki=300, 200x the first working value — does not hold up against this
script's higher-resolution data. That search polled `get_status` at only
~3-4Hz (every ~250-300ms); this script logs the full ~135Hz telemetry
relay stream. At Ki=400, this script found clear, real oscillatory
ringing (15.1% overshoot, visible in the plot as several damped bounces)
— a transient happening on a ~30-60ms timescale that a ~250-300ms
polling interval cannot resolve at all (well below Nyquist for that
signal). **The earlier "no overshoot" conclusion was very likely a
measurement-resolution artifact, not a real absence of overshoot** — it's
plausible real (smaller) overshoot was already present at some of the
values between 80-300 that search called clean, just never visible at
that sample rate. The earlier entry's *mechanism* explanation (the
anti-windup clamp bounds the integral's total possible contribution to a
fixed ceiling) is still correct as far as it goes, but is now known to be
incomplete: it bounds the final settled contribution, not how fast the
integral state can swing on the way there, and that swing rate is
exactly what interacts with real plant lag to produce the overshoot seen
here. **Practical lesson**: don't trust an overshoot/stability
conclusion from a control-loop test sampled far slower than the
loop itself runs — this is exactly why `fta_closed_loop_step_response_vcp.py`
was built logging the real telemetry rate instead of polling
`get_status` in the first place, and it's already paid for itself.

**Not yet done**: the true Ki overshoot boundary is only bracketed
(clean at 200, real ringing at 400), not precisely located; Kp was only
tried at one elevated value (5.0) combined with two Ki values — a
proper 2D sweep (Kp × Ki) has not been done; only the -25px step
direction and dac_y=2048 operating point have been characterized under
this rigorous method; closed-loop sine tracking across 5-20Hz (the
actual disturbance-band deliverable) still hasn't been attempted with
any of these gain choices. Result files:
`results/fta_closed_loop_step_response_vcp_kp{5000,1750}_ki{30000,100000,200000,400000}.{npz,png}`
(a subset also uses the bare `_kp5000.npz`/`.png` naming from the first
Kp-only trial) — not all committed as of this entry, only the original
baseline run was.

Combined comparison plot (both this entry's 6 runs, ordered baseline →
chosen): `results/fta_closed_loop_step_response_tuning_panel.png`, built
by the new `fta_closed_loop_step_response_plot.py` (reuses `analyze_step`
from the test script itself rather than re-deriving metrics, zoomed to
the interesting ~1.5s window around each step, color-coded clean/ringing
by the same >5%-overshoot threshold used in this file's prose).

### Pushed further per the user's request ("get to a usable region") — found a real Kp instability boundary (~6-6.5 counts/px), best result 63-156ms settling depending on overshoot tolerance (2026-08-13, same day)

**Real, safety-relevant finding: Kp above ~6 counts/px is genuinely
closed-loop UNSTABLE at this operating point, not just "more overshoot."**
Tried Kp=8.0 with the previously-untested Ki=50 (chosen to pair with the
plant's measured local gain, ~0.09 px/count at 2048, whose matched
static Kp would be `1/0.09≈11`): the recorded trace showed large,
sustained, chaotic-looking oscillation (up to ~800px against a ~25px
step) **present even during the pre-step baseline hold**, i.e. this
combination is unstable just sitting at a fixed setpoint, not only in
response to a step. Isolated whether Ki was implicated: Kp=8.0 with
Ki=0 (pure P) was *also* unstable — small ripple at rest, but the step
itself triggered a clearly GROWING oscillation that never settled in the
3s recorded window. This is Kp alone, nothing to do with Ki or the
anti-windup mechanism discussed in the two entries above.

**Bisected with Ki=0 to isolate the boundary**: Kp=5.5 stable (damped
ringing, settles by 156ms, real steady-state offset expected since
Ki=0); Kp=6.0 marginal (settles, but only after ~5-6 visibly decaying
oscillation cycles, 484ms); Kp=6.5 and Kp=8.0 both genuinely unstable
(growing, never settling). **The real closed-loop instability boundary
for this operating point sits between Kp=6.0 and Kp=6.5** — far below
the naive "matched-to-plant-gain" estimate of ~11, almost certainly
because that estimate only accounts for static gain and ignores the
real phase lag already characterized elsewhere in this project (the
~41ms pipeline delay, the actuator's own dynamics/resonance near 11Hz) —
phase lag is exactly what erodes stability margin as loop gain rises,
so a naive static-gain match was never going to be safe. **Practical
lesson: don't estimate a safe Kp from static plant gain alone on this
rig — the real ceiling is roughly half the naive estimate, found only by
testing.** All of this stayed physically safe throughout: `apply_dac`'s
own `[95,4000]` clamp bounded every excursion regardless of how bad the
tuning was, and every test script run returned hardware to idle
afterward regardless of outcome.

**Also found: pushing toward the instability boundary is not a good way
to get speed.** Kp=6.0's marginal case took LONGER to settle (484ms)
than the comfortably-stable Kp=5.5 (156ms) — near a stability boundary,
damping drops and oscillation cycles multiply, which costs time even
though the "loop gain" is nominally higher. The better lever, confirmed
again here, stayed Ki: combining a moderate, comfortably-stable Kp (3.5,
scaled up from 1.75 but well clear of the ~6 boundary) with Ki=200 gave
a clean single-transition response (78ms rise, 1.1% overshoot, 156ms
settling) — comparable to, not better than, the Ki-alone result from the
entry above (Kp=1.75/Ki=200: 141ms rise and settling). Pushing that same
Kp=3.5 combo to Ki=300 got faster (94ms settling) but reintroduced real
ringing (28.6% overshoot) — and notably, this was WORSE (slower, more
overshoot) than simply cranking Ki alone at the original low Kp
(Kp=1.75/Ki=400: 63ms settling, 15.1% overshoot, see the entry above).
**Net conclusion: raising Kp did not help anywhere in this session's
testing** — every combination that added Kp was either no faster than
the equivalent Ki-alone result, or strictly worse (more overshoot for
similar or worse settling). Ki alone, kept away from its own (separately
bracketed, see the prior entry) overshoot onset around 200-400, remains
the best lever found on this rig.

**Where this leaves tuning, answering the user's "usable region"
question**: two real candidates, both far better than the un-tuned
1000ms starting point, trading settling speed against overshoot:

| candidate | rise | overshoot | settling | character |
|---|---|---|---|---|
| Kp=1.75, Ki=200 | 141ms | 1.1% | 141ms | clean, single transition, no visible ringing |
| Kp=1.75, Ki=400 | 32ms | 15.1% | 63ms | fast, but real (if brief) ringing each correction |

Neither has been validated against the actual 10-20Hz sine disturbance
this project targets — step-response settling time is a reasonable
proxy for closed-loop bandwidth but not a substitute for measuring it
directly, and repeated overshoot on every correction cycle (the Ki=400
case) could plausibly matter more under continuous oscillatory tracking
than it does on a single one-off step. **Recommendation, not yet
executed**: start closed-loop sine tracking with the clean Kp=1.75/Ki=200
choice first (lower risk of exciting resonance under sustained
correction), and only reach for Ki=400 if 141ms of step-settling turns
out to translate into inadequate 10-20Hz rejection.

Second combined comparison plot (stability search + combined-gain
candidates): `results/fta_closed_loop_step_response_stability_panel.png`,
built by `fta_closed_loop_step_response_plot2.py` — unstable runs shown
over the full recorded window (the sustained oscillation IS the finding)
and color-coded red, distinct from the clean/ringing blue/orange coding
used in the first panel.

**Not yet done**: the Kp instability boundary (6.0-6.5) is only
bracketed to within 0.5 counts/px, not precisely located; no Kp×Ki 2D
sweep away from the two 1D slices tried (Ki=0 for the Kp search, Kp
fixed at 1.75/3.5/5.0 for the Ki searches); closed-loop sine tracking
across 5-20Hz — the real deliverable — still not attempted with either
candidate gain set.

### Closed-loop sine test attempted — hit a real, hard VCP throughput ceiling (~3Hz) that blocks any real 10-20Hz characterization; root cause chain untangled but not fixed (2026-08-13, same day)

Built `fta_closed_loop_sine_response_test_vcp.py`: the closed-loop analog
of the open-loop `fta_sine_response_test_vcp.py`, driving `target_x`
(pixels) through a sinusoid instead of a raw DAC value, and fitting the
measured `cx` against the known-frequency reference. The rationale for
why this measures disturbance rejection at all (not just tracking): for
a unity-feedback loop, reference-tracking T(s) and disturbance-rejection
sensitivity S(s) are complementary, S+T=1 — good tracking of a moving
target_x at some frequency directly implies good rejection of a real
disturbance there. This is the actual project deliverable.

**First real run (1Hz, 200Hz requested update rate, fire-and-forget
bursts like the open-loop script uses): tracking gain came back exactly
0.000.** Checked `target_x` via `get_status` after the run — it was
still sitting at its original primed value, unchanged. **100% of 1600
attempted `set_target_x` updates were silently dropped** — not a tuning
problem, the setpoint never moved from its starting value for the entire
8s recording. This is a much more severe version of the burst-write
corruption already found earlier this session (see "First closed-loop
PID bench test," which needed ~20ms/char pacing to reliably land a
single one-shot command) — here EVERY attempt failed, most likely
because 200Hz fire-and-forget also collides with the firmware's
one-pending-line VCP buffer (a new command's bytes arriving before the
previous line's reply has been drained get silently dropped, a
mechanism already documented in this file's 2026-08-12 fix to
`fta_calibration_vcp.py`), not just the timing-critical-section race the
20ms/char fix was built for.

**Switched to per-character pacing (matching the 20ms/char approach used
for one-shot commands) and immediately hit a second, confounding bug,
independent of the firmware entirely: Windows' default ~15.6ms timer
tick.** Measured directly: `time.sleep(0.001)` (intended: 1ms) actually
took ~15.6ms on this machine — a 15x inflation. This means an earlier
same-day reliability probe (in "First closed-loop PID bench test") that
seemed to show 1ms/char pacing was just as reliable as 20ms/char was
**comparing two settings that were secretly running at the same real
delay the whole time** — that probe's conclusion needs the same kind of
asterisk as the Ki-escalation "no overshoot" claim corrected earlier
this session: not wrong about what it measured, wrong about what it
thought it was varying. Fixed with `winmm.timeBeginPeriod(1)` (a
standard Windows high-resolution-timer request, wrapped in `atexit` so
it's restored on every exit path) — confirmed directly, this dropped
`sleep(0.001)`'s real duration to ~1.5ms, a genuine ~10x improvement.

**With the timer fix in place and pacing now genuinely fast, reliability
collapsed again** — 1.5ms/char (now real) landed only 0.2% of attempts
in the actual sine-loop context, even though an ISOLATED single-threaded
probe at the same delay measured 100% reliable (60/60). The likely
difference: the sine script runs a concurrent background reader thread
draining telemetry throughout the update loop, and that thread appears
to introduce enough GIL/scheduler jitter to reintroduce the
critical-section race that isolated single-threaded pacing doesn't
suffer from. Escalated pacing empirically in the real multi-threaded
context: 4ms/char reached only 14.5% applied; **the only pacing that
reproduced this whole session's proven 100% reliability was the original
~20ms/char**, which caps real achievable `target_x` updates at **~3Hz**
(a ~17-character `set_target_x N` line takes ~340ms to send paced).

**Practical conclusion: ~3Hz is nowhere near enough to trace a valid
sine anywhere close to the project's actual target.** Confirmed
empirically, not just by the math (10x-oversampling would need ~10Hz for
a 1Hz test, itself 30x short of the real 10-20Hz band): a 1Hz test at
the honest 3Hz update ceiling produced a coarse triangle wave, not a
sine — visible directly in the plot
(`results/fta_closed_loop_sine_response_vcp_1Hz_20260813T222143Z.png`)
— and the real closed-loop response visibly saturates against that
triangle rather than tracking a clean sinusoid, an encouraging sign the
control loop itself is responding correctly, but not usable frequency-
response data. Fitted gain/lag numbers from this run (27.5%, -35.5ms)
should not be trusted as real closed-loop tracking-gain/phase-lag
measurements — they're an artifact of fitting a sine model against a
trajectory that was never actually sinusoidal.

**This is a real architectural blocker, not a parameter to keep
tuning.** The current design — the laptop streaming individual
`set_target_x` VCP commands to trace the disturbance waveform — cannot
reliably exceed ~3 commands/second under the Pi's current telemetry
load, regardless of pacing strategy tried. Real options, not yet
decided:
1. **Most likely the real fix**: give the firmware an on-board sine (or
   general trajectory) generator — a new command like
   `start_sine freq amplitude center` that has the Nucleo itself compute
   the moving setpoint every control tick, eliminating the need for
   ~150+ host commands/second entirely. This is genuine new firmware
   work, not yet designed or attempted.
2. Investigate whether the Pi's telemetry rate could be reduced for this
   specific test (lower packet rate might leave more main-loop headroom
   for servicing VCP commands) — not controllable from this laptop-only
   session, would need Pi access.
3. Accept the ~3Hz ceiling and only characterize frequencies low enough
   for it to give valid (if not project-relevant) data — doesn't reach
   the actual 10-20Hz deliverable, of limited value.

**State left**: hardware safely idle. Script committed with its
docstring corrected to describe the real, measured throughput ceiling
rather than the original (wrong) fire-and-forget assumption — default
`--update-rate` changed from 200 to 3 to reflect reality rather than
imply a false choice.

### Firmware queue rewrite + clock/baud raise — real ~13x VCP command throughput improvement, several regeneration regressions found and fixed along the way (2026-08-13, same day)

Prompted by the user asking why this project couldn't hit the ~1600Hz
command rate the old "FTA Controller" firmware achieved. Real answer,
worked out before touching hardware: three compounding differences, not
one. (1) The old firmware had a real 64-deep command queue; this
firmware's VCP RX held exactly one pending line and silently dropped
anything arriving before the main loop drained it — its own source
comment said why: "VCP commands are low-rate ... not a hot path," an
assumption that stopped being true once host scripts started streaming
setpoints. (2) USART2 ran at 115200 baud here vs. 460800 on the old
firmware. (3) The old firmware had no I2C telemetry traffic at all
(driven by an onboard photodiode, not the Pi) — this one prints a full
relay line over the *same wire* for every I2C packet, ~150-200Hz, which
bandwidth math showed was already eating ~60% of 115200 baud's raw
capacity on its own.

**Firmware rewrite #1 — real command queue + non-blocking TX.** Replaced
the single-pending-line VCP RX buffer with an 8-deep ring buffer of
complete lines (ISR fills, main loop drains one per pass — see
`vcp_rx_queue`/`vcp_rx_head`/`vcp_rx_tail`/`vcp_rx_count` in `main.c`).
Replaced every blocking `HAL_UART_Transmit(..., 100)` call (heartbeat,
the per-packet relay print, every command reply) with `enqueue_tx()` — a
small TX ring buffer (8 deep, `tx_queue`) drained by interrupt-driven
`HAL_UART_Transmit_IT` + a new `HAL_UART_TxCpltCallback`, so the main
loop never blocks on transmission again. Clean rebuild, zero warnings.

**Attempted raising baud to 460800 alone first — total silent failure,
not a partial improvement.** After flashing, zero bytes came out at
*any* baud rate, not even the heartbeat. Root cause: this project's
whole clock tree ran at just 4MHz (`SystemClock_Config`, `MSIRANGE_6`,
no PLL) — at 4MHz with standard 16x oversampling, the maximum
achievable UART baud is ~250,000, so 460800 was mathematically
unreachable and the firmware silently hung in `Error_Handler` before
ever reaching `main()`'s loop. Reverted immediately rather than push
further blind.

**User raised the clock via CubeMX** (switched `SystemClockMux` from
MSI to HSI16, 16MHz, no PLL needed — the simple option, recommended
specifically to minimize risk over a full PLL-based 80MHz config) and
regenerated code. This is exactly the kind of change flagged as needing
CubeMX's own tool rather than a hand-derived value: I2C1's `Timing`
register (`hi2c1.Init.Timing`) is a hand-tuned magic value correct only
for the clock it was computed against, and CubeMX recomputed it
correctly for 16MHz (`0x00100D14` → `0x00503D58`) as part of the same
regeneration.

**Regeneration broke several things that needed manual reapplication —
all found and fixed before flashing, not after:**
1. **The DAC HAL driver source files were deleted outright**
   (`stm32l4xx_hal_dac[_ex].c/.h`) — CubeMX cleaned up driver files for
   a peripheral it doesn't think is used, since DAC1 was never added to
   the `.ioc` (same "hand-added outside CubeMX" situation as the pins
   themselves). Restored via `git checkout --` (still safely committed).
2. **`HAL_DAC_MODULE_ENABLED` was re-commented-out** in
   `stm32l4xx_hal_conf.h` — the exact gotcha this file already warned
   about elsewhere, hit for real this time. Re-enabled by hand.
3. **I2C1's NVIC priority reset to CubeMX's default (0)**, undoing the
   2026-08-04 tuning that put USART2 at priority 0 and I2C1 at 1 (I2C
   can tolerate a delay via clock stretching; UART RX cannot). This
   block is CubeMX-generated, not USER-CODE-protected, so it will always
   reset on regeneration — reapplied by hand, comment updated to say so
   explicitly for next time.
4. **A genuine compile-breaking regression**: `stm32l4xx_it.c` still
   calls `HAL_UART_IRQHandler(&huart2)` from `USART2_IRQHandler`, but
   the `extern UART_HandleTypeDef huart2;` declaration that makes that
   legal was silently dropped from the regenerated "External variables"
   section. Re-added — this time *inside* a USER CODE block so it
   survives the next regeneration too.
5. **Self-inflicted, separate from the regeneration**: while checking
   the CLI rebuild for warnings, ran `rm -rf Debug/Core Debug/Drivers`
   to force a clean rebuild — this deleted STM32CubeIDE's auto-generated
   per-folder build rules (`subdir.mk` etc.), which are gitignored and
   only created by the IDE's own build system, not by plain `make`. Broke
   the CLI-only rebuild path entirely (link stage silently skipped
   compiling anything and failed on missing .o files). Recovered by
   asking the user to do one normal Build in STM32CubeIDE, which
   regenerated the missing files properly. **Lesson: don't `rm -rf`
   inside a CubeIDE-managed Debug/ directory to force a "clean" CLI
   rebuild — `make clean` (if the generated makefile supports it) or
   just trust incremental rebuilds instead.**

**Raised USART2 to 460800 baud** (now safely inside the 16MHz clock's
range, `16MHz/(16×460800)=2.17`, a valid divisor with margin) — not
`.ioc`-tracked (same situation as the other hand-added settings), so
flagged inline to reapply by hand if ever regenerated again.

**Real, measured result after all of this — a genuine ~13x throughput
improvement, but burst writes remain fundamentally broken regardless:**
- **True single-burst writes** (one `ser.write()` call for the whole
  command line) are **still 0% reliable** at the new clock+baud — proves
  this specific failure mode was never about raw bandwidth or CPU speed
  at all; something about how a burst gets chunked/timed at the
  USB↔UART bridge causes it to lose bytes regardless.
- **Paced writes are where the real win is.** A back-to-back reliability
  test (real multi-threaded context, a concurrent reader thread
  draining telemetry, matching how the actual test scripts operate) at
  460800 baud with the queue fix: 0.1-0.5ms/char pacing all landed
  ~99-100% clean, achieving **~38-39 commands/s** — throughput plateaus
  there regardless of pushing the delay lower (per-`write()`-call
  overhead becomes the bottleneck, not the sleep itself). That's a
  **~13x improvement over the ~3Hz ceiling** this project was stuck at
  immediately before this fix (see the entry above). Undelayed
  per-character writes (still individual `ser.write()` calls, just no
  explicit `time.sleep`) reached **~739/s at 85% reliability** — faster
  but not fully reliable, not recommended over the ~38-39Hz/~100% option
  for anything that needs to actually land.
- **Real end-to-end sanity check, not just raw command echo**: reran
  `fta_closed_loop_step_response_vcp.py` (Kp=1.75/Ki=200, -25px step) —
  93ms rise, 2.0% overshoot, 140ms settling, matching the pre-existing
  validated numbers closely (previously 141ms/1.1%/141ms) — confirms
  the whole closed-loop control pipeline (DAC output, I2C telemetry,
  PID math, `HAL_GetTick()`-based timing) survived the clock change
  intact. Telemetry relay rate during this run averaged ~233/s, up from
  the earlier ~135-150/s — plausibly more CPU headroom at 16MHz, not
  confirmed further.

**Practical takeaway for any future VCP scripting on this rig**: use
paced per-character writes (~0.2-1ms/char is now enough, previously
needed ~20ms/char) for anything that must reliably land, never a single
burst `ser.write()` call regardless of how fast the link is configured.
`fta_manual_control.py`'s `send()` (previously an unpaced burst) and
`fta_closed_loop_sine_response_test_vcp.py`'s target-update loop
(previously 20ms/char, based on the now-corrected confounded finding)
were both updated to the new faster pacing. All host-side `FTA_BAUD`/
`BAUD` constants raised back to 460800 to match.

**Not yet done**: the actual 10-20Hz closed-loop sine test (the real
deliverable, see the entry above) has not been re-attempted with the
new ~38-39Hz update ceiling — better than before, but still short of
the >=10x-oversampling guideline for the full 10-20Hz band (comfortably
covers <=~3.5Hz, only partial coverage up to ~10Hz). An on-board
firmware setpoint generator (`start_sine freq amplitude center`,
proposed in the entry above) would still be the more robust long-term
fix if full-fidelity 10-20Hz sine characterization is needed. Windows
timer-resolution fix (`winmm.timeBeginPeriod(1)`) is only applied in
the sine-test script so far, not the other paced-write scripts — likely
harmless to add elsewhere but not done. Full Kp re-exploration at the
new, much healthier command throughput hasn't been attempted (all
tuning-pass numbers above predate this fix).

### On-board sine setpoint generator built — sidesteps the VCP throughput ceiling entirely; real closed-loop 1-20Hz frequency response captured; a negative-lag artifact found, root-caused, and fixed twice (once cheaply, once properly) (2026-08-13, same day)

After the entry above left off with "the actual 10-20Hz closed-loop sine
test... has not been re-attempted," the user made a decisive call rather
than continuing to chase VCP throughput further: **"i feel like we took
a wrong turn a long time ago- lets just do your onboard sine generation
idea as an emergency solution to get some plots today, and come back to
this [the VCP root-cause investigation] later."** Deprioritized the
remaining VCP-throughput work (still not resumed as of this entry — the
~38-39Hz paced-write ceiling from the entry above is where that thread
stands) and built a firmware-side sine generator instead, eliminating
the need for host-streamed per-sample setpoints entirely.

**Firmware additions** (`main.c`): `start_sine FREQ_MILLIHZ AMPLITUDE_PX
CENTER_PX` / `stop_sine` VCP commands; `update_sine_target()` computes
`target_x(t) = center + amplitude*sin(2*pi*freq*(t-t0))` using the
firmware's own `HAL_GetTick()`, called once per confident telemetry
packet (same cadence `run_closed_loop_step` already runs at) right
before the control step. No host involvement needed once started — this
is what actually removes the throughput ceiling as a constraint, since
the setpoint no longer needs streaming at all.

**First negative-lag bug — user caught it immediately, correctly called
it impossible.** First test run (1Hz) reported `lag=-71.9ms`; 5Hz and
10Hz also came back negative. User: *"im looking at the plot, why does
the cx lead the commanded position, that seems impossible"* — correct: a
passive causal system cannot lead its own commanded reference. Root
cause: the test script's `t_sine_start = time.monotonic()` was captured
**after** the full paced round-trip for the `start_sine` command
completed (~20ms/char × ~23 chars ≈ 460ms of transmission, plus the
reply's own transit time) — but the firmware's real `g_sine_start_tick`
is latched the instant the command line finishes parsing, well before
it even starts transmitting the `OK` reply. This is a systematic
"host's assumed t=0 is late relative to the sine's real start" offset,
which reads exactly like negative lag once the recorded samples are fit
against a sine assumed to start at that too-late t=0 — not a real
physical effect, not a sign/phase-ambiguity bug in the fit itself (the
`fit_tracking()` math for extracting phase from an
`A*sin(wt)+B*cos(wt)+C` fit was independently re-derived and confirmed
correct for a real positive lag).

**Fix #1 (cheap, partial): `send_command_timed()`** — captures the
host-side timestamp right after the last character (`\n`) is
transmitted, not after the reply arrives, and uses that as the t=0
reference instead. Rerunning 1Hz with just this fix: `lag=+32.7ms` —
sane and positive. Better, but still an *estimate* subject to residual
USB-CDC/OS scheduling jitter on the write path.

**Fix #2 (proper, per the user's suggestion): report the live setpoint
in every telemetry line instead of reconstructing it from any host-side
clock at all.** User: *"why not transmit a field in the telemetry that
indicates the current setpoint, then the timing will match perfectly"*
— the better fix, adopted immediately. Added a `tgt=` field to the
per-packet VCP relay line (`seq=... status=... x=... y=... tgt=...
pkts=... errs=...`, sourced from `g_target_x_scaled` at the exact same
point in the main loop that just fed it to `run_closed_loop_step`, so
it's not a stale/asynchronous read). Host-side `fit_tracking()` rewritten
to fit **both** the measured `cx` trace and the firmware-reported `tgt`
trace against the same `sin(wt)/cos(wt)` basis (same `t` array, whatever
it is), then take the **difference** of their fitted phases as the lag.
This is deliberately immune to any t0 error — a constant offset in `t`
shifts both fitted phases equally, which cancels out of the difference —
and no longer needs to trust that the firmware's `sinf()`/integer
rounding produced exactly the requested amplitude/center either, since
both are read from the real `tgt` fit rather than assumed. `line[]`
buffer grown 80→100 bytes to fit the new field.

**Second bug, found immediately after switching to the tgt-based fit**:
15Hz came back `lag=-50.1ms` (`-270.4°`) — again impossible-looking, but
this time a genuine remaining math bug, not a physical or timing issue:
`atan2()` alone only guarantees each individual phase lands in
`(-pi,pi]`, not their *difference*, so a real ~90°+ lag could surface as
e.g. -270° instead of the equivalent, sensible +90°. Fixed by wrapping
`phase_x - phase_t` into `(-pi,pi]` (smallest-magnitude branch) before
converting to a lag — same standard treatment, different specific bug,
as the sign/phase-disambiguation work this project already did for the
open-loop `fit_sine()` (see "RESOLVED (2026-08-06)" above) — still the
same fundamental single-frequency wraparound ambiguity (can't
distinguish a lag from `lag ± n*period` off one test tone), just
resolved to its least-aliased branch rather than left unwrapped.

**Real, final, validated closed-loop frequency response, `dac_y`→`cx`,
Kp=1.75/Ki=200, amplitude=25px @ dac_y=2048** (the same clean operating
point characterized throughout the last several entries), full
telemetry-rate data (~210-216 samples/s throughout, no longer
command-throughput-limited at all):

| freq | gain (T) | lag |
|---|---|---|
| 1 Hz | 0.956 | 57.6ms (20.7°) |
| 5 Hz | 0.507 | 36.0ms (64.9°) |
| 10 Hz | 0.298 | 21.8ms (78.5°) |
| 15 Hz | 0.281 | 16.7ms (90.4°) |
| 20 Hz | 0.262 | 12.5ms (89.8°) |

Gain and lag both decrease smoothly and monotonically with frequency —
no sign flips, no impossible values, no discontinuities. **Flagged
honestly, not hidden**: 15Hz and 20Hz sit right at the ~90° mark, the
edge of what a single test frequency's phase fit can resolve
unambiguously (same limit noted for the open-loop 10Hz+ results
elsewhere in this file) — those two lag numbers are lower-confidence
than 1/5/10Hz, not wrong, but shouldn't be over-trusted to more than
about ±(half the period) without a corroborating method (e.g. a
two-tone or swept-sine test) if that precision ever matters. Gain
values were trustworthy throughout this whole debugging arc (magnitude
`hypot(A,B)` is phase-independent, never affected by either bug) — only
the lag/phase numbers needed fixing.

Raw data: `results/fta_closed_loop_onboard_sine_{1,5,10,15,20}Hz_*.npz`
(now includes `t`, `x`, and `tgt` arrays). Earlier, pre-fix runs at each
frequency (negative-lag and/or pre-phase-wrap) were deleted from
`results/` rather than left alongside the corrected ones — the plot
script keys files by frequency via a glob + dict, so stale duplicates
were a real risk of silently feeding wrong data into a future summary
plot, not just clutter. Combined summary figure built via
`fta_closed_loop_onboard_sine_plot.py` (also updated to plot the real
`tgt` trace instead of a reconstructed ideal reference):
`results/fta_closed_loop_onboard_sine_summary.png`.

**This is the project's first real closed-loop frequency-response
characterization actually spanning the full 10-20Hz disturbance target**
(every closed-loop attempt before today was blocked by the VCP
throughput ceiling documented in the entries above). Gain at 10-20Hz
(26-30%) is in a broadly similar range to what the open-loop plant
itself showed in this same band (see "Pushed sine tracking to
5/10/15/20Hz" and the fine-sweep/resonance entries above) — consistent
with the closed loop's rejection being limited more by the plant's own
10-20Hz rolloff than by anything control-loop-specific, though a direct
side-by-side comparison hasn't been done.

**Not yet done**: the deliberately-deferred VCP root-cause investigation
(DMA + idle-line UART RX, see the entry above) — still not resumed, per
the user's explicit "come back to this later"; Kp is still untried at
higher values against this new on-board-sine measurement (all of today's
closed-loop tuning work used step response, not sine, as the target
metric); axis y not tested with the on-board sine generator; no repeats
for statistical confidence (n=1 per frequency); the 15-20Hz phase-wrap
ambiguity noted above isn't resolved by anything short of a different
measurement method.

**Follow-up, same session: repeated at a much smaller, more
disturbance-realistic amplitude (10um peak-to-peak) — required a
firmware precision fix first.** User asked to rerun at 10um peak-to-peak
instead of the original 25px/150um-peak-to-peak amplitude. `start_sine`'s
`AMPLITUDE_PX` argument was whole-pixels-only (`strtol`, no decimal
support) — 10um peak-to-peak needs 1.667px amplitude
(`5um / MICRONS_PER_PIXEL(3.0)`), which would only round to 1 or 2 whole
px (6 or 12um peak-to-peak), not close enough. Changed the wire format:
`start_sine` now takes `AMPLITUDE_X10` (tenths of a pixel, i.e. the same
`POSITION_SCALE` units `g_target_x_scaled`/`tel_x_scaled` already use
elsewhere in this firmware) instead of whole pixels — `g_sine_amplitude_scaled`
is now set directly from the parsed integer rather than re-deriving it
via another `*POSITION_SCALE`. The `OK sine_started` reply's `amplitude=`
field now reports the real decoded value (e.g. `amplitude=1.7`) instead
of echoing the raw integer, so a caller can see exactly what was applied
after rounding. `fta_closed_loop_onboard_sine_test.py` updated to send
`round(amplitude_px * 10)`. Rebuilt, reflashed — **hit the same
total-silence-after-reflash glitch documented earlier this session, same
fix (reflash again, no code change) resolved it.**

Reran all 5 frequencies at `--amplitude-px 1.6667` (firmware applied
1.7px = 10.2um peak-to-peak, confirmed via the `OK sine_started` reply
each time):

| freq | gain (T) | lag |
|---|---|---|
| 1 Hz | 0.941 | 63.5ms (22.8°) |
| 5 Hz | 0.470 | 36.3ms (65.4°) |
| 10 Hz | 0.328 | 22.6ms (81.3°) |
| 15 Hz | 0.268 | 16.0ms (86.5°) |
| 20 Hz | 0.332 | 13.8ms (99.2°) |

Broadly similar shape to the 25px/150um sweep (gain rolls off from ~0.94
at 1Hz down to the 0.27-0.33 range by 10-20Hz; lag decreases smoothly
from 63.5ms to 13.8ms) — **not a dramatically different regime**, unlike
the sharp small-amplitude stiction/threshold collapse this project found
in the *open-loop* plant early on (see "RETRACTED: ... a clean amplitude
comparison finds the 10-20Hz 'rolloff' was mostly a nonlinear threshold
effect" above, ±200 vs ±800 DAC counts). One visible wrinkle: gain dips
to a minimum at 15Hz (0.27) then rises slightly at 20Hz (0.33) rather
than monotonically decreasing — consistent with, though not the same
frequencies as, the resonance/anti-resonance plateau-dip-recovery shape
already documented for the open-loop plant's fine 3-12Hz sweep. 20Hz's
phase (99.2°) is past the ±90° smallest-magnitude-branch boundary this
session's phase-wrap fix resolves to, so that one lag number in
particular should be read as lower-confidence, same caveat as 15-20Hz in
the 25px sweep above.

`fta_closed_loop_onboard_sine_plot.py` updated to select which
amplitude's sweep to summarize by each npz's own stored `amplitude_px`
(via `--amplitude-px`, defaulting to the largest available for backward
compatibility) rather than assuming one file per frequency — `results/`
now holds both the 25px and 1.667px sweeps side by side, and a filename-
only match would have silently picked whichever happened to glob last.
Two summary figures now exist:
`results/fta_closed_loop_onboard_sine_summary.png` (25px/150um) and
`results/fta_closed_loop_onboard_sine_summary_10um.png` (1.667px/10um).

**Follow-up, same session: added the live commanded actuator output
(`dac_y`) to the telemetry relay line, and re-ran the 10um sweep at half
duration to plot it.** User asked to see the actual DAC command
alongside cx/target, not just infer it from the control law offline.
Added a `dac_y=` field to the per-packet VCP relay line, sourced from
`g_last_dac_y` (the real value `apply_dac()` last wrote to the DAC,
plain counts, not `POSITION_SCALE`-scaled since it's a hardware setpoint
not a pixel measurement) — `line[]` grown 100→120 bytes. Rebuilt,
reflashed — **hit the same total-silence-after-reflash glitch documented
twice already this session, same fix (reflash again, no code change)
resolved it a third time**, reinforcing that this really is a one-off
flash/reset artifact on this hardware, not something to chase further.

`fta_closed_loop_onboard_sine_test.py`'s `TELEMETRY_RE`/`_reader_thread`
updated to parse and record `dac_y`; `save_plot()` refactored out of
`main()` into a standalone function and given a second stacked panel
(`dac_y` vs. time, sharing the x-axis) below the existing cx panel.
Also added a `--replot PATH` mode (loads an existing `results/*.npz` and
regenerates just its PNG, no hardware touched) — used it to apply a
legend-readability fix (opaque background, since the two-panel layout
left less room for the previous frameless legend to avoid overlapping
data) without re-running any test, and to backfill-safe older npz files
that predate `dac_y` (falls back to a NaN gap in that panel rather than
erroring).

**Re-ran the full 1/5/10/15/20Hz sweep at 1.667px/10um pk-pk, half the
original duration** (`--duration 4` at 1Hz, `--duration 1` at
5/10/15/20Hz, vs. 8s/2s before) — still ~200+ samples per run at the
~210Hz telemetry rate, plenty for a stable fit at these frequencies:

| freq | gain (T) | lag |
|---|---|---|
| 1 Hz | 0.933 | 64.2ms (23.1°) |
| 5 Hz | 0.458 | 36.7ms (66.1°) |
| 10 Hz | 0.286 | 22.7ms (81.7°) |
| 15 Hz | 0.279 | 16.9ms (91.1°) |
| 20 Hz | 0.382 | 12.9ms (93.1°) |

Matches the full-duration 10um sweep's numbers closely (within noise) —
halving duration didn't change the result, as expected given sample
count stayed well above what the linear-lstsq fit needs. Superseded the
prior (dac_y-less, full-duration) 10um sweep files in `results/` (deleted
rather than kept alongside, to avoid the summary-plot amplitude-matching
logic picking arbitrarily between two files at the same commanded
amplitude). `fta_closed_loop_onboard_sine_plot.py` also updated with a
matching `dac_y` row (3-row grid: cx traces, dac_y traces, gain/lag
summary) and the same NaN-fallback for pre-dac_y files — the 25px sweep's
summary figure still renders correctly, just with an empty `dac_y` row,
since those runs predate the field. Both summary PNGs regenerated.

### Off-the-shelf PID adopted per Phil's e-mail — `PIDController.hpp` (his class, verbatim) integrated via a thin C-callable shim, hardware-validated, replacing the hand-rolled P+I control law (2026-08-18)

Phil e-mailed a short survey of C++ PID options (WPILib's `PIDController`,
`PatrickBaus/PID-CPP`, and a self-contained custom class pasted directly
in the e-mail) and asked which to use. Recommended against the first two
(WPILib: wrong domain, FRC-ecosystem dependency tree; PID-CPP: still C++
for no real benefit over the pasted snippet) and suggested porting just
the pasted class's filtered-derivative-on-measurement technique into the
existing hand-rolled C loop, since this firmware had never had a D term.
**User asked to use the e-mailed code completely instead, no mish-mash of
hand-rolled and borrowed control logic.**

**Decision: keep the project a plain-C CubeIDE project, add the class as
new C++ files rather than converting the whole project to C++.** Reasons
specific to this codebase, not generic caution: this project has already
been hit once by a CubeMX regeneration silently reverting multiple hand-
added settings (DAC driver files deleted, `HAL_DAC_MODULE_ENABLED` re-
commented-out, the I2C1/USART2 NVIC priority swap reset, an `extern
huart2` dropped — see "Firmware queue rewrite" above) — a project-wide
language-mode flip was judged too large a blast radius against a
one-class need. `main.c` is also the single most interrupt-sensitive,
heavily-tuned file in this project; touching all of it for a change that
doesn't affect Phil's class's fidelity either way wasn't worth the risk.

**What was added, all new files (no existing file renamed):**
- `Core/Inc/PIDController.hpp` — Phil's class, byte-for-byte as pasted in
  his e-mail. Not modified at all, including keeping `double` throughout
  despite this MCU's FPU (`fpv4-sp-d16`) being single-precision-only
  (software-emulated double math) — a deliberate, flagged tradeoff to
  honor "use it completely," not an oversight; negligible cost against
  this loop's real timing budget.
- `Core/Inc/pid_wrapper.h` / `Core/Src/pid_wrapper.cpp` — a thin
  `extern "C"` shim owning one `PIDController` instance and forwarding
  `pid_wrapper_init/_set_gains/_calculate/_reset` calls, so `main.c`
  (still plain C) can drive it. No control-law logic of its own.
  Constructed via placement-new into a `alignas(PIDController)` byte
  buffer, not a function-local `static` — deliberately avoids the
  compiler's thread-safe "magic statics" guard-variable machinery
  (`__cxa_guard_acquire/release`, lives in libstdc++/libsupc++), which
  this firmware's link line doesn't otherwise pull in. Compiled with
  `-fno-exceptions -fno-rtti -fno-threadsafe-statics`; empirically links
  clean against the existing `-lc -lm`-only link line (confirmed by
  actually building it, not just reasoned about) — no libstdc++
  dependency needed at all, since the class touches nothing (virtual
  functions, heap, exceptions) that would require it.

**Two real, project-specific integration decisions, not part of "just
call the class":**
1. **Correction, not absolute output.** `pid_wrapper_calculate()` returns
   a value relative to `g_closed_loop_base_dac_y` (this firmware's
   existing bumpless-transfer bias), not an absolute DAC value — `main.c`
   adds its own base and does the final `[DAC_MIN_COUNT, DAC_MAX_COUNT]`
   clamp via `apply_dac()`, exactly as before. That bumpless-transfer/
   final-clamp design is this firmware's own architecture, not something
   `PIDController.hpp` needs to know about; `setOutputLimits()` is set to
   a generous symmetric ±(DAC_MAX-DAC_MIN) so the class's own back-
   calculation anti-windup stays meaningful without fighting the outer
   clamp.
2. **Fixed `ts_`, not measured `dt`.** The previous hand-rolled loop
   measured a real (slightly variable) `dt` via `HAL_GetTick()` every
   step, specifically because control steps fire on telemetry arrival,
   not a fixed timer. `PIDController::calculate()` takes no `dt` argument
   at all — `ts_` is baked in at construction. Using the class unmodified
   means accepting this as a known, deliberate approximation (`ts_s =
   1/210`, this firmware's typical closed-loop telemetry rate measured
   repeatedly across 2026-08-13/14) rather than working around it.
   `g_last_control_tick` (no longer needed) was removed entirely rather
   than left dead.

**Build system: hand-extended, not CubeIDE-regenerated.** This project's
command-line build (`make.exe all` in `Debug/`, used all session to
bypass the IDE) runs off CubeIDE's auto-generated `subdir.mk`/`sources.mk`
files, which only get new-file compile rules when CubeIDE's own indexer
sees a file added *through the IDE*. Rather than requiring that GUI step,
hand-extended the generated files directly: added `CPP_SRCS`/`CPP_DEPS`
to `sources.mk`, a `.cpp` pattern rule (`arm-none-eabi-g++`, `-std=gnu++17`)
to `Core/Src/subdir.mk`, a `CPP_DEPS` include to the top-level `makefile`,
switched the final link driver from `arm-none-eabi-gcc` to
`arm-none-eabi-g++` (standard practice once any C++ translation unit is
in the mix), and added the new `.o` to `objects.list`. **Flagged inline
in both `sources.mk` and `Core/Src/subdir.mk`** (matching this project's
existing convention for every other hand-added-outside-CubeMX setting):
if this project is ever regenerated or rebuilt fresh from inside CubeIDE
without the IDE's own project settings also knowing about the `.cpp`
file, these hand-added blocks will be silently lost and need reapplying.

**`main.c` wiring**: `run_closed_loop_step()` now just descales
target/measured to real px doubles, calls `pid_wrapper_calculate()`, adds
the base, and calls `apply_dac()` — the P+I+anti-windup math that used to
live inline is gone, replaced by the class. `cmd_set_kp`/`cmd_set_ki` now
call `pid_wrapper_set_gains()` (which reconstructs the class — it has no
gain setters by design — implicitly clearing integral/derivative history
the same way this file's old code explicitly zeroed the integral on a Ki
change). Added `cmd_set_kd`/`set_kd` (milli-units, matching `set_kp`/
`set_ki`'s existing convention) since the class now makes a real D term
available — defaults to `Kd=0`, i.e. behaviorally P+I until deliberately
tuned. `cmd_set_mode`'s closed_loop bumpless-transfer branch now calls
`pid_wrapper_reset()` instead of zeroing a local integral variable.
`cmd_get_status`'s STATUS line gained `kd_milli=` (`line[]` grown
250→280 bytes for headroom, matching this project's established pattern
of bumping buffer sizes when adding fields).

**Build verified clean** (zero warnings, `main.c` via `gcc -std=gnu11`,
`pid_wrapper.cpp` via `g++ -std=gnu++17`, link via `g++`) before ever
touching hardware. Flashed — **hit the same total-silence-after-reflash
glitch documented several times already this session; reflashed again
with no code change, resolved it, same as every prior occurrence.**
`get_status` confirmed `kd_milli=0` present and correctly initialized.

**Hardware-validated two ways:**
1. A quick manual `get_status`-polling diagnostic (target set 25px off
   baseline before engaging closed_loop, Kp=1.75/Ki=200/Kd=0, `dac_y`
   watched over ~3s): `dac_y` moved cleanly from base 2048 toward ~1810
   and `tel_x` converged smoothly to within ~0.1px of the exact target —
   confirms the new PID path drives real hardware correctly.
2. Reran `fta_closed_loop_step_response_vcp.py` (same Kp=1.75/Ki=200
   baseline this project has used throughout). **First attempt found a
   real, unrelated regression**: that script's `TELEMETRY_RE` predated
   the `tgt=`/`dac_y=` fields added to the telemetry relay line earlier
   this same session (see the two entries above) and could no longer
   match any line at all — "0 usable telemetry samples." Fixed by
   updating the regex to the current wire format (this script only
   consumes the `x`/`y` groups, so no other code needed to change). A
   second, separate hiccup on the very next run (`set_mode closed_loop`'s
   reply went unconfirmed by `send_command`, a known, recurring VCP
   flakiness this project has hit repeatedly all session under live
   telemetry load) resolved itself on a plain retry — the command had
   actually landed both times, only the confirmation read was lost.
   **Real result once both were sorted out**: delta -24.99px (essentially
   exact against the commanded -25px step), rise time 78ms, overshoot
   13.9%, settling 297ms (`results/fta_closed_loop_step_response_vcp_20260818T172003Z.{npz,png}`)
   — clean convergence, a few damped oscillations, no divergence, no
   clamp. **Somewhat more overshoot/slower settling than the old hand-
   rolled controller's best-known result at this exact Kp/Ki (1.1%/141ms,
   see "Real tuning pass" above)** — plausible explanations, not yet
   distinguished: the fixed-`ts_` approximation vs. the old measured-`dt`,
   or Phil's back-calculation anti-windup behaving differently in the
   transient than the old pre-clamped-integral approach. Not necessarily
   a problem (still comfortably damped, no divergence), but worth knowing
   before assuming the old Kp/Ki are still optimal for this new
   implementation.

**Not yet done**: Kp/Ki/Kd have not been re-tuned against this new
implementation at all (everything above reused the old hand-rolled
controller's best-known gains as a parity check, not a fresh search); Kd
has never been set away from 0, so the derivative-on-measurement/filter
path this whole integration was originally motivated by is unexercised;
the same `TELEMETRY_RE`-predates-`tgt=`/`dac_y=` bug almost certainly
still affects other scripts sharing the old fixed-format assumption
(`fta_calibration_vcp.py`, `fta_manual_control.py`,
`fta_sine_response_test_vcp.py`, the open-loop `fta_step_response_test_vcp.py`)
— only the one script actually run today was fixed; sine-tracking
against this new controller (the real 10-20Hz deliverable) hasn't been
re-run either. (Committed and pushed later the same session, see below.)

### D-term evaluated properly: a direct free-decay test confirms a real ~15.3Hz resonance, D does not help at any tested filter cutoff, and the PID rate/cross-axis questions are answered (2026-08-18, same day)

Follow-up questions from the user after the PID integration above, addressed
in order:

**Does the PID loop run faster than telemetry?** No — `run_closed_loop_step()`
only fires inside `if (g_new_packet_ready) { ... }`, i.e. once per I2C
packet, capped by telemetry exactly as before. What changed is that
`PIDController::calculate()` has no `dt` argument (`ts_` fixed at
construction, `~1/210s`), vs. the old controller's real per-call
`HAL_GetTick()` measurement — a known, already-documented tradeoff, not a
new finding.

**Is cross-axis coupling causing the "wild x jumping"?** Checked directly
against already-captured data rather than guessing: `cy` (the telemetry
`y` channel) stays flat (~0.3px std, ~1.6px range) through the Kd=0
step response — cross-axis coupling is NOT excited at Kd=0. The
`cy` std=14.4 / range=109px seen at Kd=0.05 tracks the *whole loop* going
unstable at that gain (matches `cx`'s own blow-up there), not an
axis-coupling-specific effect. **A second PID axis would not fix the
overshoot** — the wildness is `cx` itself (the controlled axis)
overshooting more than the old hand-rolled controller did at identical
nominal Kp/Ki (13.9%/297ms new vs. 1.1%/141ms old, see the integration
entry above) — a same-axis effect, most likely from a real anti-windup
*mechanism* difference (old code proactively clamped the integral state
every step; `PIDController.hpp` only reactively claws back integral when
the *combined* `p+i+d` output saturates its configured limits, which
were set generously wide — so for a small ±25px step the anti-windup
essentially never engages). **Not yet tested** — tightening
`pid_wrapper_init`'s output limits to force earlier engagement is a
concrete, cheap next experiment, flagged but not done this session.

**Ring-down resonance test — user's own idea, executed directly, real
finding.** With the amp OFF, `set_y` has no physical effect (the DAC
register changes but the amplifier stage isn't gating current to the
coil) — pre-load a step target while off, pulse the amp briefly on (a
real force step) then back off, and watch the mechanical system decay
under its own free dynamics, unconfounded by any control loop or
telemetry-rate-limited phase fitting. Built `fta_ringdown_test.py`.

**Real bug on the first attempt, caught and fixed before trusting the
result**: the pulse-sequence commands (`set_y`/`amp_enable`/`amp_disable`)
were sent via `send_command()` (which reads replies) while the background
reader thread was *already* consuming `ser.readline()` on the same
`Serial` object — exactly the two-threads-racing-for-`readline()` mistake
`fta_closed_loop_step_response_vcp.py`'s own docstring already warns
against, made fresh in the new script. All three commands exhausted their
full retry budget waiting for replies the reader thread kept stealing,
stretching an intended ~80ms pulse into ~24 real seconds. Fixed by
switching the pulse sequence to paced writes with no reply read (matching
that script's established pattern), same as every other "send something
mid-recording" case in this project.

**Result, second (fixed) attempt: a clean, textbook free decay.** Flat
baseline, forced ringing while the amp is driven, then — the instant the
amp cuts — a smooth, visibly-decaying oscillation matched closely by a
fitted damped sinusoid: **freq=15.35Hz, damping ratio ζ≈0.105**
(`results/fta_ringdown_20260818T174119Z.{npz,png}`). This is a genuine
mechanical resonance measurement, independent of the control loop or any
sine-fit ambiguity — confirms (and sharpens, from the earlier fine-sweep's
rougher ~11Hz estimate) that this rig has a lightly-damped resonance
sitting *inside* the project's own 10-20Hz disturbance-rejection target.
Q = 1/(2ζ) ≈ 4.8 — a real, noticeable peak, not an extreme one; not
obviously fatal to the control goal on its own, but a hard constraint any
controller (not just D) has to respect near 15Hz, not something tuning
alone dissolves. Standard mitigations if more aggressive rejection is
ever needed: a notch filter at ~15.3Hz, or physically stiffening/damping
the flexure (hardware, out of firmware's reach).

**This directly explains the D-term instability found earlier**: the
20Hz derivative filter cutoff barely attenuates a 15.3Hz resonance at
all — the filter was passing the plant's worst dynamics straight through
into the correction.

**Exposed the filter cutoff as live-tunable to test that properly.**
Added `pid_wrapper_set_fc()` (`pid_wrapper.cpp`/`.h`) — reconstructs the
`PIDController` with a new `fc` using the last-commanded gains (now
tracked in module-level `g_kp`/`g_ki`/`g_kd` statics, since the class
itself has no getters). Added `set_fc MILLIHZ` VCP command + `fc_millihz=`
in `get_status` (same milli-units integer convention as `set_kp`/`ki`/`kd`).
Rebuilt, reflashed (same post-reflash silence glitch as every other
reflash this session, resolved by reflashing again). Added `--fc-milli`
to `fta_closed_loop_step_response_vcp.py`.

**Systematic result: D does not help at ANY tested (Kd, fc) combination**
— `results/fta_closed_loop_dterm_comparison.png`:

| Kd | fc | overshoot | settling |
|---|---|---|---|
| 0 (baseline) | — | 13.9% | 297ms |
| 0.001 | 20Hz | 34.7% | 1047ms |
| 0.005 | 20Hz | 45.4% | 2469ms |
| 0.05 | 20Hz | unstable at rest | never |
| 0.005 | 3Hz | 30.2% | 2187ms |
| 0.001 | 3Hz | 23.5% | 343ms |
| 0.001 | 1Hz | 14.2% | 391ms |

Clear, consistent trend: lowering `fc` monotonically converges back
*toward* the `Kd=0` baseline (as expected — a lower cutoff attenuates the
derivative signal toward zero) but never actually beats it, at any `Kd`
or `fc` tried. **Conclusion: for this plant, a simple single-pole
low-pass-filtered derivative-on-measurement term (the technique
`PIDController.hpp` implements) does not help** — any cutoff low enough
to avoid exciting the 15.3Hz resonance also filters away whatever useful
rate information D could have contributed; any cutoff high enough to let
D respond usefully also passes the resonance straight through. **Kept
`Kd=0` as the working configuration** — P+I alone remains the best result
found on this rig. A real notch filter targeting 15.3Hz specifically
(not just a low-pass) is the more principled next step if D/higher
bandwidth is still wanted later, not attempted this session.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`) after every test. `fta_ringdown_test.py` committed-
style (repo root, not yet git-committed); `scratch_kd_compare_plot.py`
(repo root) is genuinely ad hoc/one-off, not written to project
convention. **Not yet done**: the anti-windup output-limits experiment
flagged above; a full sine-tracking sweep was deliberately NOT re-run
with any D configuration, since none beat the P+I baseline on the much
cheaper step-response test — no reason to spend the extra hardware time
confirming the same negative result at 5 frequencies. (Committed and
pushed later the same session, commit `12548ba`.)

### I2C1 bus speed investigated — real ~106kHz standard mode found; STM32 slave side raised to ~333kHz Fast Mode and tested; Pi (master) side still needs a matching change (2026-08-19)

Prompted by the user asking why the Pi->Nucleo telemetry rate (~200-235Hz,
observed repeatedly all session) is so far below this project's own
~1kHz raw-camera-capture ceiling (`MODE_640_100_ROI`, ~854-880fps). Two
things resolved, one thing still open (needs the Pi, not available from
this laptop-only session):

**Clarified the actual comparison, not apples-to-apples.** The ~1kHz
numbers came from `camera_throughput_test.py` -- pure sensor capture, no
beam detection, no I2C send. The real streaming path
(`camera_view_tool.py`) does capture + `find_beam_blob()` + an I2C send
every frame. This project already measured `find_beam_blob()` alone at
640x200 binned: ~2.32ms/call, a ~430fps ceiling from detection alone
("fps root-cause" section above) -- already well under 1kHz before I2C
even enters the picture. Real observed throughput (~200-235Hz) is still
roughly half of that detection-alone estimate, so detection cost doesn't
explain the whole gap either.

**Checked what's directly checkable from this session (no Pi access):
I2C1's actual bus speed.** Decoded the STM32 slave's own
`hi2c1.Init.Timing` register (`0x00503D58`, CubeMX-computed when the
system clock was raised to 16MHz -- see "Firmware queue rewrite" above)
using the STM32L4 I2C_TIMINGR formula (RM0394 26.4.9): PRESC=0,
SCLH=61, SCLL=88 -> SCL period ~9.4us -> **~106kHz, standard mode, not
Fast Mode** -- never previously checked or documented. At ~106kHz, one
8-byte telemetry packet costs ~700-800us of raw bus time alone, a
~1.3kHz ceiling by itself -- real, but not the dominant factor given the
~430fps detection ceiling is already lower than that.

**Important clarification given to the user, worth remembering**: I2C
slaves never drive SCL. The STM32's `Timing` register only configures
how *this MCU* samples/filters the bus to correctly decode whatever
clock the master actually drives -- it does not by itself change the
real bus speed. The Raspberry Pi (bus master, via `smbus2`/`/dev/i2c-1`)
is what actually sets SCL frequency, and needs its own change too.
Raising only the STM32 side is a real, useful, testable step (validates
the slave doesn't break at the new sampling config) but cannot alone
demonstrate a throughput improvement.

**STM32 side raised to Fast Mode and hardware-tested.** Hand-derived a
new `Timing` value (no CubeMX GUI access from this session) from the
register formula rather than trusting a memorized reference table: with
I2CCLK=16MHz, PRESC=1 (`t_PRESC=125ns`), SCLL=13 (`t_SCLL=1750ns`, above
the Fast Mode spec minimum tLOW=1300ns), SCLH=9 (`t_SCLH=1250ns`, above
spec minimum tHIGH=600ns) -> total SCL period ~3.0us -> **~333kHz**,
deliberately short of the 400kHz spec ceiling for margin since this
bus's real rise/fall times have never been scoped. SCLDEL=4/SDADEL=0 are
conservative values matching the general shape of ST's own Fast Mode
reference tables, not tuned to this specific board.
`hi2c1.Init.Timing = 0x1040090D`. **Not `.ioc`-tracked** (same situation
as every other hand-added-outside-CubeMX setting in this file -- NVIC
priorities, DAC1 init, USART2 baud) -- flagged inline in `main.c`,
reapply by hand if this project is ever regenerated from the `.ioc`.

Rebuilt, reflashed (same post-reflash silence glitch as every other
reflash this session, resolved by reflashing again). **Confirmed clean**:
`errs=0`, telemetry still flowing at the same ~220Hz baseline as before
(expected -- the Pi hasn't changed yet, so the real bus clock is still
whatever the Pi was already driving). This validates the new slave-side
timing config doesn't break compatibility with a slower master (a slave
configured for faster sampling has *more* margin against a slower real
clock, not less) -- a real, useful confirmation, even though it can't
demonstrate a speed win by itself.

**Pi-side change needed to actually get a faster bus -- not done this
session, no access to that machine from here.** Add (or edit, if a
lower value already exists) in `/boot/firmware/config.txt`:
```
dtparam=i2c_arm_baudrate=400000
```
(`dtparam=i2c_arm=on` is already confirmed present in that file per the
"Header I2C bus" section above -- add the baudrate line alongside it,
same file.) This is the standard, documented Raspberry Pi OS parameter
name for the general-purpose I2C bus baud rate; Raspberry Pi 5's RP1
southbridge kept the same `i2c_arm` device-tree alias/binding for
compatibility, so this should apply the same way it does on earlier Pi
models, but **has not been verified against this specific RP1-based
Pi 5 from this session** -- worth confirming directly rather than
assuming. Requires a reboot to take effect (device tree overlay
parameter, not a runtime-settable value).

**Verification steps for after the reboot**, in order:
1. `sudo i2cdetect -y 1` should still cleanly find the Nucleo at `0x42`
   -- confirms the bus still enumerates correctly at the new speed
   before trusting anything higher-level.
2. Restart whichever streaming script was running
   (`camera_view_tool.py`'s default mode, or `beam_position_streamer.py`)
   and watch the Nucleo's own heartbeat/telemetry-relay line for
   `errs=` -- if the physical wiring/pull-ups aren't good enough for
   400kHz-class signal integrity (a real risk Standard Mode is more
   tolerant of than Fast Mode), checksum errors would show up here
   first, not as a silent failure.
3. Compare the achieved packet rate against this entry's ~220Hz
   baseline (same simple raw-line-count-over-N-seconds check used
   throughout this file, e.g. reading the VCP for a few seconds and
   counting `seq=` lines, or watching the Nucleo's own `pkts=` delta in
   `get_status`/heartbeat over a fixed window).
4. If throughput improves but is still well under the ~430fps
   detection-alone ceiling, the remaining gap is most likely Pi-side
   compute (detection + Python/`smbus2` per-call overhead) -- worth a
   direct, isolated timing check of `NucleoLink.send_position()` and/or
   `find_beam_blob()` at whatever mode is actually in use, the same way
   this project isolated `find_beam_blob()`'s cost once before (see
   "fps root-cause" above) -- not done this session.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`), `get_status` confirms `errs=0` at the new STM32-side
timing config. Committed same session (`dbfd825`), pushed.

### Pi-side ROI + other changes raised real telemetry to ~440-475Hz; updated the PID's fixed ts_ to match; found the OLD Kp/Ki gains are now genuinely unstable at the new rate, found a new conservative stable point (2026-08-19, same day)

User made changes on the Pi (ROI + other changes, exact details not made
from this laptop-only session) that raised real end-to-end telemetry
throughput from ~200-235Hz to a measured **~465Hz** (confirmed directly:
1396 lines in 3s from this session, right in the reported 440-475Hz
range).

**Updated the PID's fixed `ts_` to match.** `pid_wrapper_init()`'s
`ts_s` argument (see the PID-integration entry above for why this is
fixed rather than measured per-call) was still `1/210` from before this
change -- now stale, since the class assumes each `calculate()` call
represents a fixed real time slice. Updated to `1/457.5` (midpoint of
the reported 440-475Hz range) in `main.c`, `pid_wrapper.cpp`'s defensive
default, and both files' comments. Rebuilt, reflashed (same
post-reflash silence glitch as every other reflash this session, same
fix).

**Real, important finding: Kp=1.75/Ki=200 (the gains validated clean at
~210-235Hz all session) are now genuinely UNSTABLE at ~465Hz** -- not a
corrupted-command artifact (checked: a `get_status` reply came back
visibly spliced with a telemetry line mid-transmission, real evidence
the VCP TX side is under more contention at the higher telemetry rate,
but the readable portion confirmed the gains/target landed correctly;
the control loop reads I2C data directly in memory, never via the VCP
text stream, so TX corruption can't have affected the actual closed-loop
math). The real step-response trace shows genuine, physically bounded
(DAC-clamped, safe) but clearly *growing* oscillation after the step --
not noise, not a plotting artifact.

**Root cause, isolated via a quick systematic search rather than
assumed**: tested pure P (`Ki=0`, `Kp=1.75` alone) first -- clean,
damped, stable (settles ~344ms to the expected small P-only steady-state
offset, `results/fta_closed_loop_step_response_465hz_kp1750_ki0.png`).
**Kp alone is fine at the new rate; the instability is specifically from
the integral term.** Likely mechanism: this plant has a fixed real
transport delay (~41ms pipeline lag, established earlier in this
project) that doesn't shrink just because the control loop now updates
~2.2x more often -- so at 465Hz, many more integral-accumulating
corrections now land *within* that same fixed delay window before the
plant's response to earlier corrections is even reflected back,
effectively raising loop gain relative to what the plant can physically
keep up with, even with `ts_` correctly rescaled so the integral math
itself is numerically consistent with real elapsed time. A standard,
expected consequence of raising sample rate without re-tuning integral
gain down to match, not a bug in the `ts_` fix.

**Ki search at fixed Kp=1.75** (`results/fta_closed_loop_465hz_ki_search.png`,
`scratch_465hz_ki_search.py`):

| Ki | character |
|---|---|
| 0 | stable, clean, but only reaches a small fraction of a 25px step (expected P-only limitation) |
| 10 | **stable** -- smooth approach, 4.1% overshoot, settles in 2844ms, no growth anywhere across a 6s window |
| 20 | marginal -- converges cleanly for ~1.7s, then a slow-building oscillation starts growing in the last ~1.3s of the recording |
| 50 | unstable -- clearly growing oscillation |
| 200 (old baseline) | unstable -- growing oscillation, largest amplitude tested |

**Chosen for now: Kp=1.75, Ki=10** -- confirmed stable across a full 6s
window, though slow (2.5s rise, 2.8s settling) compared to the old
141-297ms range this project achieved at the slower rate. This is a
conservative, safe starting point, not a final tuned answer -- there's
real room between Ki=10 (safely stable) and Ki=20 (marginal) that a
finer search could recover, and Kp itself hasn't been re-explored at
this new rate at all (matching the earlier finding that Kp, not just Ki,
has its own real instability boundary -- that boundary was only ever
characterized at the old, slower rate and may have shifted too).

**Not yet done**: fine Ki search between 10-20; Kp re-exploration at the
new rate; Kd (still 0, untouched this entry) revisiting now that the
resonance/rate relationship is better understood; a full sine-tracking
sweep at the new rate/gains (the actual 10-20Hz deliverable); the exact
Pi-side ROI/other changes that produced the rate increase were not
recorded from this laptop-only session -- worth a brief note from
whoever made them, for the record. **State left**: hardware safely idle.
Not yet committed to git as of this entry.

### Pi-side I2C1 baud change landed and verified; `camera_view_tool.py` streaming throughput root-caused and fixed — 238Hz → ~440-475Hz real, measured (2026-08-18)

Direct follow-up to the "Pi-side change needed" verification steps above.
Added `dtparam=i2c_arm_baudrate=400000` to `/boot/firmware/config.txt`
(alongside the already-present `dtparam=i2c_arm=on`) and rebooted.
Confirmed applied: the live devicetree `clock-frequency` property under
`/sys/class/i2c-dev/i2c-1/device/of_node/` reads `400000`, and
`i2cdetect -y 1` still cleanly finds the Nucleo at `0x42` (~0.018s, not a
timeout) — bus healthy at the new speed, step 1-2 of the prior entry's
verification plan both pass.

**Real camera driver work in this same session turned out to be a false
alarm, not related to any of this.** A separate, uncommitted mid-session
detour investigated `camera_preview_roi.py` showing "staticy flickering"
and failing to center on the beam right after this same reboot — root
cause was NOT the baud change (that bus is physically separate from the
camera CSI/I2C controllers) and NOT a driver regression: only camera 0
enumerated that boot (camera 1's already-documented intermittent
ribbon-seating fault, see "Hardware status" below, coincidentally on the
same boot), and `camera_preview_roi.py` itself has no beam-centering
logic at all — it always resets to `y_start=0`, and the beam had drifted
to sensor row ~426 (outside that fixed top-of-sensor window) since this
tool was last used back in July. Fixed by adding an optional `y_start`
CLI arg to `camera_preview_roi.py` (reuses `roi_set_selection.py`'s
`set_roi_y_start`). Also cleaned up 3 stray artifacts (a bare `h`
appended to the end of `camera_preview_roi.py`/`camera_view_tool.py`, a
`workedh` typo in `camera_preview.py`'s docstring) — accidental keystrokes
that landed in Thonny's editor instead of the OpenCV window, from running
these tools inside Thonny rather than a plain terminal.

**The real question — why does `camera_view_tool.py`'s live streaming to
the Nucleo only achieve ~238Hz when this project once measured this
camera at nearly 1kHz — was root-caused with real, isolated timing
measurements** (matching this project's own established methodology, not
estimated):

| stage | measured |
|---|---|
| `NucleoLink.send_position()` alone, new 400kHz bus | ~0.29-0.34ms/call (was ~1ms/call pre-baud-raise) |
| `find_beam_blob()` alone, `640x200` | ~1.79-2.02ms/call |
| `find_beam_blob()` alone, `640x100` | ~1.14ms/call |
| raw capture alone, `640x200` | ~1.875ms/frame (533fps, matches the old validated floor) |
| combined capture+detect+send loop, `640x200`, no recenter/display | **342Hz** |
| combined capture+detect+send loop, `640x100`, no recenter/display | **558Hz** |
| + auto-track recenter (blocking, ~8.6ms/call × ~20/s), `640x200` | 274Hz |
| + display/GTK overhead (~15Hz throttled draw) | ~238Hz (the originally-reported number) |

**I2C was never the bottleneck after the baud raise** — `send_position()`
dropped to ~0.3ms/call, a small fraction of the loop. The real cost was
`apply_y_start()` (two `v4l2-ctl` subprocess calls, ~8.6ms combined)
running synchronously inside the same loop that feeds the Nucleo, firing
~20×/second whenever auto-track is on (the default whenever streaming is
on) — plus `640x200`'s per-pixel detection cost being roughly double
`640x100`'s.

**Two real fixes landed in `camera_view_tool.py`, not yet committed as of
mid-session but committed by the end (see push note below):**
1. `DEFAULT_STREAM_ROI` switched from `(640, 200)` to `(640, 100)` — the
   fastest binned mode, ~558Hz pure ceiling vs ~342Hz. Real tradeoff:
   half the vertical drift margin (100 output / 200 real pre-bin rows),
   which is exactly why fix #2 needed to land first.
2. `apply_y_start()`'s per-frame auto-track calls replaced with a new
   `request_recenter()` — runs the actual subprocess work on a background
   thread instead of blocking the capture/detect/send loop. A `cycle_height()`-
   or startup-triggered `apply_y_start()` call stays synchronous (rare,
   one-time events, not hot-path).

**A real correctness bug was caught and fixed before landing, not
theoretical** — the user asked directly whether the streamed coordinates
were independent of recentering, which they weren't in the first cut.
`last_centroid_abs_y[i] = y_starts[i] + cy * v_bin` converts a frame's
local centroid to an absolute sensor row using `y_starts[i]` as the
offset; that's only correct if `y_starts[i]` can't change between "frame
captured" and "coordinate computed" for that frame. The first background-
thread version wrote `y_starts[i]` directly from the worker thread, at
whatever arbitrary moment the subprocess call happened to finish relative
to the main loop's own capture timing — if that landed between a frame
being captured under the *old* window and its coordinate being computed,
the tool would report a spurious jump of however far the recenter moved,
straight into the Nucleo's telemetry stream as if it were real beam
motion. Fixed: the background worker now writes into a separate
`_recenter_applied` dict; the main loop is the only thing that ever moves
a value from there into `y_starts`, and only right before that camera's
own next `capture_array()` call — restoring the same "no capture in
flight across a `y_starts` change" guarantee the old blocking design had
for free, while keeping the actual slow work off the hot path. See
`request_recenter()`'s docstring in the script for the full mechanism.

**Verified live after each change** (`DISPLAY=:0`, real GUI, real Nucleo,
`--signal=INT` so the `finally` cleanup runs): 342Hz → ~440Hz after the
two fixes → ~449-475Hz after the correctness fix (no regression; run-to-
run variance, if anything slightly faster). 0 I2C send failures across
every run, `tainted` stayed `4096`, no dmesg anomalies, camera/I2C
handles clean after each run. Not stress-tested against a *large* beam
correction specifically (the bench beam was already well-centered during
these runs, so recenter targets were close to the already-applied
position) — the fix is a structural guarantee, not something that needed
a big jump to validate, but flagging that this specific caveat wasn't
exercised.

**Committed and pushed** (see git log). `beam_position_streamer.py` was
not touched this session — it has its own, separate blocking
`set_roi_y_start` usage pattern and wasn't part of what the user was
running, so it may still have the same recenter-blocks-the-loop cost if
its own auto-recentering path is ever added/used; worth revisiting if it
becomes the streaming path of choice instead of `camera_view_tool.py`.

**State left**: Pi-side camera/I2C in this file's usual clean idle state.
User moving to the laptop next to work on the Nucleo's closed-loop PID
code — nothing about this session's changes touches firmware or the
Nucleo side at all, purely Pi-side camera/I2C throughput.

### Chasing "why is the loop unstable at the new ~465Hz rate" — rate itself ruled out by direct diagnostic; a real ring-down remeasurement effort found the ORIGINAL 15.3Hz resonance number was itself broken (host timestamp bucketing), and the true resonance is ~38.5Hz, not 15.3 or 22 (2026-08-19, same day)

Direct continuation of the entry above: after pulling the Pi-side
440-475Hz throughput fix, updated `pid_wrapper_init`'s `ts_s` to
`1/457.5` (from `1/210`) to match, rebuilt/reflashed. Reran the
established `Kp=1.75/Ki=200` step-response baseline — **genuinely
unstable, growing oscillation, not settling** (`results/fta_closed_loop_step_response_vcp_20260818T191117Z.png`).
Verified this wasn't a corrupted-command artifact first (a `get_status`
reply came back visibly spliced with a telemetry line under the higher
load, but the readable portion confirmed gains/target landed correctly,
and the control loop reads I2C data directly in memory, never via VCP,
so TX corruption can't have touched the actual control math).

**Ki search at the new rate** (`Kp=1.75` fixed): Ki=200/50 clearly
unstable, Ki=20 marginal (clean for ~1.7s then slowly builds
oscillation), **Ki=10 genuinely stable** (4.1% overshoot, 2844ms
settling, no growth across a 6s window) — see
`results/fta_closed_loop_465hz_ki_search.png`. Initial (WRONG, see below)
theory: a sample-rate effect, since the plant's fixed ~41ms pipeline
delay doesn't shrink just because the loop updates more often.

**User pushed back, correctly**: pure delay-based control theory says a
fixed dead-time's effect on stability margin depends on absolute delay,
not sampling density, as long as well below Nyquist (true at both 210Hz
and 465Hz relative to a ~15-40Hz plant). **Direct diagnostic run to
settle it**: added a TEMPORARY firmware throttle
(`DIAG_CONTROL_INTERVAL_MS`, gates how often `run_closed_loop_step`
actually fires back to ~200Hz while telemetry TX stays at the full
~465Hz) and reran the exact same `Kp=1.75/Ki=200` baseline. **Still
deeply unstable (1179.6% overshoot), same growing-oscillation
signature** (`results/fta_closed_loop_step_response_465hz_telemetry_200hz_control_kp1750_ki200.png`)
— this cleanly falsifies sample rate as the cause. Checked static local
plant gain next (small `dac_y` steps around 2048, open-loop): **0.094-0.095
px/count, essentially unchanged** from the ~0.09 px/count baseline this
control loop was originally tuned against — rules out a simple DC
calibration shift too.

**Re-ran the same free-decay ring-down test from the previous PID entry
(user's own idea) to check whether the actuator's resonance itself
moved.** First re-measurement: 22.09Hz, ζ=0.082 (vs. the previously-
documented 15.35Hz, ζ=0.105) — reported as "the resonance moved,
explaining the instability." **User pushed back again, correctly**:
insufficient resolution was the likely explanation, not a real physical
shift. Investigated directly rather than assuming either way:

- Re-examined the OLD ring-down capture's own raw `t[]` array: **69-86%
  of consecutive host-side timestamps were EXACTLY identical** — Windows
  batches this project's `_reader_thread`'s `readline()` calls into
  ~15-16ms bursts (thread-scheduling granularity), not the ~2-3ms real
  telemetry spacing. For a 15-65ms-period oscillation, that's only
  ~1-4 real timestamp buckets per cycle — nowhere near enough to trust a
  frequency fit, regardless of the underlying ~210-465Hz telemetry rate
  itself being fine.
- Tried the obvious fix (`winmm.timeBeginPeriod(1)`, already used
  elsewhere in this project for `time.sleep()` inflation) — **did not
  help** (still 85% zero-dt after applying it). That fixes Sleep()
  granularity, not general thread-scheduling preemption granularity —
  a real, useful negative result, not just an oversight.
- **Real fix: stopped trusting host arrival timestamps entirely.** Added
  a `tick=` field (raw `HAL_GetTick()`, ms) to the telemetry relay line
  (`line[]` grown 120→140 bytes) — the firmware's own free-running
  1ms SysTick counter, immune to host OS scheduling. Rebuilt, reflashed
  (same post-reflash silence glitch as every other reflash this session,
  same fix). Updated `fta_ringdown_test.py` to timestamp every sample
  from this field instead of `time.monotonic()`.

**With real per-sample timing, the picture changed completely** — and
took two more wrong turns before landing somewhere trustworthy:
1. A "biggest single-sample drop" heuristic for finding the amp-off
   moment picked an early point still inside the forced-drive transient
   (the amp is actually driven for ~500-800ms in practice, not the
   nominal `--pulse-ms=80ms` — paced command transmission dominates,
   same finding as the negative-lag t0 bug from the on-board-sine-
   generator entry above) — fed `curve_fit` a mixed forced+free window,
   converged on a slow envelope (1.5Hz, ζ=0.677) that was visibly wrong
   against the raw data once plotted.
2. Switched to anchoring on the unambiguous global-minimum trough (real
   free decay, nothing can drive the system past it once the amp is
   truly off) minus a 60ms margin. Better-anchored window, FFT-seeded
   initial guess — `curve_fit` converged to 8.56Hz/ζ=0.368 with a fit
   curve that visually tracked individual oscillation cycles closely,
   looked trustworthy at the time.
3. **User pushed back a third time, correctly**: three different
   `curve_fit` answers on three attempts, even after fixing real bugs
   each time, isn't something to trust blindly — asked for a model-free
   peak-to-peak spacing measurement instead, starting a little after the
   amp turns off.

**Model-free peak/trough spacing analysis — the trustworthy result.**
Skipped the messier initial 2-3 cycles (still visibly settling from the
forced-drive release, larger and more irregular in the raw data) and
measured spacing between consecutive peaks AND troughs independently in
the clean decaying tail (`t≈1.3-1.7s`): **16 peaks + 17 troughs, 31
independent spacing measurements, mean 26.0ms → 38.51Hz** — peaks and
troughs agree closely when computed separately (both ~40Hz via median).
Marked directly on a plot for verification, not just reported as a
number: `results/fta_ringdown_peak_spacing_analysis.png`
(`scratch_ringdown_peak_analysis.py`, ad hoc, not committed-quality).

**Net conclusion: the true resonance is ~38.5Hz, not the previously-
documented 15.3Hz (which was itself measured with the same broken
host-timestamp methodology and should now be considered unverified, not
a real "before" baseline) and not the 22Hz/8.56Hz intermediate numbers
from this entry's own earlier, flawed attempts.** Whether ~38.5Hz
represents a real change from before the Pi-side ROI/other changes, or
whether the ORIGINAL 15.3Hz was simply always wrong, cannot be
determined — the original data is unsalvageable (no `tick=` field
existed yet when it was captured). Given 38.5Hz is well outside the
project's 10-20Hz disturbance-rejection target band (unlike 15.3 or
22Hz, both of which sat inside or near it), **this may substantially
change the practical implications of the whole resonance-vs-D-term
thread** — a resonance safely above the target band is a much less
acute constraint than one sitting inside it. Not yet reconciled with
the Ki-instability finding above (Ki=200 stable before, unstable now,
at Kp=1.75) — that finding stands on its own (independently confirmed,
rate-independent per the throttle diagnostic) but its connection to
"the resonance" is now an open question again given how much the
resonance number itself has moved through this entry.

**Not yet done**: reconciling the ~38.5Hz resonance finding with the
Ki-instability finding; the messier initial 2-3 cycles' own frequency
content (looked visually different/faster than the clean 38.5Hz tail —
possibly a second mode, possibly still-settling noise, not analyzed);
a fresh Ki/Kp search informed by the corrected resonance number; the D-
term question (deferred pending the above). `fta_ringdown_test.py`'s
`curve_fit`-based analysis is now known-unreliable for initial-window
selection even with correct per-sample timing (converged wrong twice)
and should probably be replaced with the peak-spacing approach as the
primary method, not just an ad hoc side script, if this test gets used
again. **State left**: hardware safely idle
(`mode=open_loop amp=0 estop=0 dac_x=95 dac_y=95`). Nothing from this
entry committed to git yet.

### Fresh gain search at the corrected ~465Hz rate — Ki pushed to 19 cleanly, but Kp has almost no headroom above 1.75 (unstable by 2.5), contradicting the "38.5Hz gives us margin" hope (2026-08-19, same day)

Direct follow-up once the true resonance (~38.5Hz, see entry above) was
established. User asked two framing questions before tuning: does
anti-windup need fixing first (**no** -- it only engages once output
saturates, and small-signal step tests never get near the ±3905
correction limit, so it doesn't participate in the linear small-signal
stability question actually driving the current instability; real for
large-disturbance recovery, but not gating this search); should D be
retried during this pass (**after**, not during -- get a solid P+I
baseline first with a resonance-informed low filter cutoff, layer D on
top once that's settled, not concurrently).

**Real bug hit immediately**: `fta_closed_loop_step_response_vcp.py`'s
`TELEMETRY_RE` still didn't have the `tick=` field added in the entry
above -- exact same "add a wire-format field, forget the OTHER script
using the same regex" mistake this project has now made three separate
times this session (`tgt=`, `dac_y=`, now `tick=`). Two back-to-back
runs both failed with "0 usable telemetry samples" before this was
caught and fixed. **Worth fixing properly at some point**: every VCP
telemetry consumer script re-declares its own copy of `TELEMETRY_RE`
independently rather than sharing one definition -- a shared parsing
module would close off this whole class of bug permanently instead of
catching it fresh reactively each time a field gets added.

**Ki search (Kp=1.75 fixed), bisecting between the previous session's
Ki=10 (stable) and Ki=20 (marginal)**:

| Ki | overshoot | settling |
|---|---|---|
| 10 | -- | 2844ms |
| 15 | 1.6% | 2265ms |
| 18 | 1.6% | 1953ms |
| 19 | 1.2% | 1797ms |

Clean, monotonic improvement, no instability anywhere in this range —
**Ki=19 is the best confirmed-clean result**, though still far slower
than the old controller's 141-297ms best.

**Kp increase attempted next, expecting the 38.5Hz finding to have
opened up real headroom — it did not.** `Kp=3.5/Ki=15`: 693.3%
overshoot, and tellingly the reported PRE-STEP baseline itself was
already unsettled (284.4px instead of the expected clean ~253-255px) --
unstable just holding a fixed setpoint, not only during the step.
Backed off to `Kp=2.5/Ki=10`: **also unstable** (422.2% overshoot, same
unsettled-baseline signature, baseline=281.2px). **The real Kp
instability boundary at this rate sits somewhere between 1.75 and 2.5 --
a much narrower margin than the 38.5Hz resonance measurement seemed to
promise.** Telemetry rate also visibly dropped during both unstable Kp
runs (~320-359/s vs. the normal ~445-450/s) -- plausibly detection
confidence degrading at the oscillation's extremes, not investigated
further.

**Practical conclusion**: Ki remains the effective, safe lever at this
rate (matching the ORIGINAL pre-2026-08-19 finding that Ki, not Kp, was
the real speed lever -- re-confirmed, not overturned, by this session's
work). Kp should stay at 1.75 for now. **Current best working
configuration: Kp=1.75, Ki=19** (1.2% overshoot, 1797ms settling).
Neither this nor the Ki=10-18 intermediate points have been re-verified
with a longer post-window than 5-6s or cross-checked against a sine
sweep yet -- step response only so far.

**Not yet done**: narrower Kp bisection (1.75-2.5) to find its precise
boundary, if worth the hardware time given Ki is already the more
productive lever; the D-term retry (deferred per the sequencing decided
at the start of this entry, not yet attempted); reconciling why Kp has
so little margin despite the resonance sitting comfortably above the
target band (the closed-loop bandwidth Kp alone drives may be reaching
up toward 38.5Hz even though the *target* disturbance band is only
10-20Hz -- plausible, not confirmed); a proper shared-regex fix for the
`TELEMETRY_RE`-duplication bug class. **State left**: hardware safely
idle (`mode=open_loop amp=0 estop=0 dac_x=95 dac_y=95`). Nothing from
this entry committed to git yet.

### D-term retried with a resonance-informed (10Hz) cutoff on top of the working Ki=19 baseline — still fails at every tested Kd, now a decisive, well-tested conclusion (2026-08-19, same day)

User asked directly: since D adds phase lead and could in principle buy
back stability margin for higher Kp/Ki (a fair, correct point), and
given the D-term failure earlier was plausibly explained by a careless
20Hz cutoff chosen against a WRONG 15.3Hz resonance reading, retry D now
with a cutoff properly informed by the real 38.5Hz measurement.

**`Kp=1.75/Ki=19/Kd=0.005/fc=10Hz`**: 3434.0% overshoot, drifted the
WRONG direction entirely (delta=+8.68px against a commanded -25px step)
— badly unstable, same growing-oscillation signature as every earlier D
failure. **`Kp=1.75/Ki=19/Kd=0.001/fc=10Hz`** (smallest representable
nonzero Kd in this firmware's milli-unit convention): still badly
unstable, 1391.5% overshoot.

**This is now a decisive result, not an unlucky parameter choice.**
Across this session, D has been tested at cutoffs spanning 1-20Hz and
`Kd` spanning 0.001-0.05, combined with both `Ki=19` (this session's
best clean P+I result) and the original `Ki=200` — **every single
combination made things worse than P+I alone; none improved on it.**
The earlier hope that a properly-chosen cutoff (informed by the
corrected 38.5Hz resonance, comfortably above the 10-20Hz target band
this time) would let D work has not panned out.

**Most likely explanation, consistent with the class's design**:
`PIDController.hpp`'s derivative filter is a single-pole EMA low-pass —
a fundamentally blunt instrument that trades "reject the resonance"
directly against "pass useful signal" along a single knob (cutoff
frequency), with no way to do both at once when the resonance is close
enough to the frequencies where useful error-rate information also
lives. A cutoff low enough to meaningfully attenuate 38.5Hz also
attenuates most of what would make D useful in the 10-20Hz target band;
a cutoff high enough to preserve useful signal doesn't reject the
resonance at all. This is a structural limitation of the *filter
design*, not evidence the underlying idea (derivative action) can't
ever work here.

**Answering the user's real question ("can this get the old speed back")
honestly: not with this D implementation.** The old 141-297ms result was
achieved under conditions (rate, and very possibly a different true
resonance situation) that no longer hold, and every attempt to recover
it — through Ki alone, through Kp, and now through D — has hit a real
wall well short of that target. `Kp=1.75, Ki=19` (1797ms settling)
remains the best confirmed-clean result this session has found. Genuine
remaining options, neither attempted: (1) a real notch filter targeting
38.5Hz specifically (rejects just that frequency, doesn't blanket-
attenuate everything above the cutoff the way a low-pass does) — new
firmware DSP work, not a parameter change; (2) physical/hardware changes
to the actuator mount (stiffen/damp to push the resonance further out or
reduce its Q) — outside firmware's reach entirely. Kept `Kd=0` (back to
the working P+I baseline) as the final state.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`). Nothing from this entry committed to git yet.

### Notch filter built and tested — first result looked like parity with P+I, but the WHOLE post-rate-change search (including that result) turns out to have been run under a leftover ~200Hz diagnostic throttle; at the true ~465Hz rate Ki=19 is badly unstable, and the notch does not move the Ki/Kp boundary (2026-08-19, same day)

Implemented the notch filter flagged as the next real option in the entry
above: a proper 2nd-order biquad notch (RBJ "Audio EQ Cookbook" formulas,
`notch_configure`/`notch_apply`/`notch_reset_state` in `main.c`), applied
to the measurement before it reaches the PID, with new `set_notch
FREQ_MILLIHZ Q_MILLI` / `notch_off` VCP commands and `notch=`/
`notch_freq_millihz=`/`notch_q_milli=` added to `get_status`. Tested at
38.5Hz/Q=3 on top of the Kp=1.75/Ki=19 baseline: 1.2% overshoot, 1781ms
settling — indistinguishable from the no-notch Ki=19 result (1.2%/1797ms).

**That comparison turned out to be meaningless — a real, important
correction.** Reread the firmware and realized `DIAG_CONTROL_INTERVAL_MS`
(the ~200Hz control-rate throttle added earlier in the day purely to
falsify sample rate as the cause of the original Ki=200 instability) and
the matching `ts_s = 1/200` were never reverted. This means the ENTIRE
post-rate-change tuning campaign from earlier today — the Ki=15/18/19
bisection, the Kp=2.5/3.5 instability check, both D-term retests, and
the first notch test above — were all run with the control loop
artificially capped at ~200Hz, not the true ~440-475Hz the Pi has
actually been streaming at since the ROI change. Telemetry TX itself was
always full-rate; only `run_closed_loop_step`'s call frequency was
throttled. `notch_configure()`'s hardcoded `457.5f` sample-rate
assumption was therefore ALSO wrong the entire time, silently
mismatched against the true ~200Hz the loop was actually running at.

**Fixed properly, not patched around**: removed `DIAG_CONTROL_INTERVAL_MS`
and its call-site gate entirely (`run_closed_loop_step` fires on every
confident packet again, matching the pre-diagnostic design), removed the
now-unused `g_last_ctrl_step_tick`, and restored `pid_wrapper_init`'s
`ts_s` to `1/457.5`. Rebuilt (clean, `text=43212`, slightly smaller with
the throttle code gone), reflashed — clean on the first try, no
post-reflash silence glitch this time.

**Retested Kp=1.75/Ki=19/notch=38.5Hz at the TRUE full rate: badly
unstable — chaotic oscillation between ~150-650px even during the
pre-step baseline hold**, nothing like a step response
(`results/fta_closed_loop_step_response_fullrate_kp1750_ki19_notch385_q3.png`).
This is a real, decisive correction: Ki=19 was never actually validated
against the rate this project has been running at since the ROI change —
every number reported for it earlier today (1.2%/1797ms, the "new best
result") was only ever true under the artificial ~200Hz cap.

**Real bug found and fixed in the host test script while chasing this**:
after a command times out (~2s with no reply — normal, already-documented
VCP flakiness under load), ~2s of telemetry backlog accumulates unread.
The NEXT command's own reply-matching window was being spent draining
that stale backlog instead of watching for a fresh reply — confirmed
directly (isolated repro: `clear_estop` and `set_mode open_loop` both
timeout, then `get_status` fails all 5 of its own retries, ~11.6s total,
even though `get_status` called on its own with a clean buffered
succeeds in <1s every time). Fixed by adding `ser.reset_input_buffer()`
to the start of `send_command()` in `fta_closed_loop_step_response_vcp.py`,
right before each paced write — clears stale backlog before it can
cascade into starving the next command's own reply. Confirmed fixed via
isolated repro before trusting it for the real test runs below.

**Fresh Ki search at the TRUE ~465Hz rate, Kp=1.75, notch=38.5Hz/Q=3
active throughout:**

| Ki | character |
|---|---|
| 15 | **clean** — bounded convergence, no growth, ~2-3px steady noise band (`results/fta_closed_loop_step_response_fullrate_kp1750_ki15_notch385_q3.png`) |
| 19 | unstable — chaotic, unbounded oscillation even at rest |
| 20 | marginal — bounded but a visible low-frequency "beating" envelope, not growing but not clean either (`..._ki20_notch385_q3.png`) |
| 30 | worse — same beating pattern but now visibly GROWING in amplitude across the recorded window (`..._ki30_notch385_q3.png`) |

Also retried `Kp=2.5/Ki=15` (notch on) to check whether the notch buys
Kp headroom the way it was hoped to when first proposed: **still badly
unstable** (575% overshoot, chaotic, telemetry rate itself dropped to
~289/s from the usual ~435-450/s — matching the same "detection
struggles during violent oscillation" signature seen for unstable Kp
values earlier today).

**Conclusion: the notch filter does not move the Ki or Kp stability
boundary in any measurable way.** The clean/marginal/unstable
transition found here with the notch active (clean at 15, marginal at
20, worse at 30) lands in essentially the same place as the boundary
already on record from EARLIER today's true-full-rate search *without*
any notch (`Ki=10` clean, `Ki=20` marginal, `Ki=50`/`200` unstable, all
pre-throttle) — if anything Ki=15 clean/Ki=20 marginal is a slightly
tighter band than the no-notch Ki=10/Ki=20 result, not a looser one,
though the difference is small enough it could just be the low-Ki edge
being under-sampled in one search vs. the other rather than a real
notch-makes-it-worse effect. Either way, there is no evidence here that
filtering out the 38.5Hz resonance from the feedback path buys back any
of the speed lost since the ROI-change rate increase. **Best honest
working point at the true full rate, with or without the notch: Kp=1.75,
Ki=15** — clean, but far short of the pre-ROI-change 141ms/1.1% result,
and no faster than what was already achievable without the notch.

**Plotting changed to always show notch status, not just when active**
(direct response to wanting this visible at a glance, not buried in a
CLI flag): `fta_closed_loop_step_response_vcp.py` now queries
`get_status` for the REAL `notch=`/`notch_freq_millihz=`/`notch_q_milli=`
fields right before engaging closed_loop, rather than trusting whether
`--notch-freq-milli` was passed on the command line — the firmware's
notch state persists in RAM across runs, so a run that passes neither
`--notch-freq-milli` nor the new `--notch-off` flag could silently still
be running with a notch a PREVIOUS run left enabled. This ground-truth
value drives a high-contrast badge on the plot itself (top-left, amber
"NOTCH ON: 38.5Hz Q=3.0" when active, muted "notch: off" when not) and
is baked into the figure title, plus saved into the npz as
`notch_active`/`notch_freq_hz`/`notch_q` — a saved plot can no longer be
mistaken for the wrong condition.

**On the user's question "is the change with the ROI turning binning on
where it was off before?"**: no — both `640x200` and `640x100` are
binned modes (`MODE_640_200_ROI`/`MODE_640_100_ROI`, see the ROI-mode
sections elsewhere in this file); binning was never off. The real
throughput fix (see "Pi-side I2C1 baud change landed and verified"
above) was three separate things: (1) `DEFAULT_STREAM_ROI` switched from
the already-binned `640x200` to the smaller, faster, ALSO-already-binned
`640x100`; (2) `apply_y_start()`'s blocking auto-track subprocess calls
moved off the hot capture/detect/send loop onto a background thread
(`request_recenter()`); (3) the I2C bus itself raised from 100kHz to
400kHz (a separate, earlier fix). None of the three is a binned/unbinned
switch.

**On the still-open "should I send out for a stiffer flexure?" question**:
this result sharpens the case for yes, more than it did before the notch
test. The notch was the cheap, no-hardware option, specifically chosen
because it should in principle reject only the resonance without the
D-term low-pass's blanket-attenuation tradeoff — and it measurably did
not help. That doesn't prove a stiffer flexure WOULD help (the
underlying instability mechanism at higher Ki/Kp still isn't fully
explained — see the "not yet done" list below), but it does mean
the one concrete firmware-only lever left on the table has now been
tried and found wanting, narrowing the realistic remaining options to
(a) accept ~Ki=15's ~1.5-2s settling as the working point, or (b) a
physical change to the actuator mount. Not a firm recommendation to
order hardware yet — worth first understanding WHY the notch didn't
help (a notch only rejects a narrow band right at 38.5Hz; if the real
instability mechanism is closed-loop bandwidth/phase-margin erosion
building up well below 38.5Hz, rather than resonance energy specifically
at 38.5Hz feeding back, a notch would never have been expected to help
regardless of hardware, and stiffening the flexure — which mainly just
relocates the resonance — might not help either).

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`). Firmware throttle-removal, `ts_s` fix, and the
notch filter itself are all uncommitted. `fta_closed_loop_step_response_vcp.py`'s
`send_command` backlog fix and ground-truth notch-status plotting are
also uncommitted. **Not yet done**: WHY the instability boundary sits
where it does (~Ki=15-20 at Kp=1.75, ~Kp=2.5 regardless of Ki) is still
not explained by anything more specific than "phase margin," not
confirmed against the 38.5Hz resonance or any other specific mechanism;
Kp has not been sub-2.5 fine-bisected at the true rate (jumped straight
from the working 1.75 to 2.5); no sine-tracking validation at true rate
with any of today's gain points; the D-term has not been retried at the
true (un-throttled) rate at all — every D-term result on record is also
now suspect for the same throttle reason and should be treated as
unvalidated at the real rate, not just the notch/Ki results.

### Output-limit tightening tried (the flagged-but-never-tried anti-windup experiment) — decisively does NOT help, and makes a marginal case actively worse; confirms the instability is a real gain/phase-margin problem, not a windup problem (2026-08-19, same day)

Direct follow-up to the "same-axis effect, most likely from a real
anti-windup mechanism difference" theory in the "D-term evaluated
properly" entry above (Phil's `PIDController.hpp` only claws back
integral reactively once the *combined* p+i+d output saturates; the old
hand-rolled controller clamped proactively every step). That theory was
never tested — `pid_wrapper_init`'s output limits were left at the full
`±3905` DAC-count span (the whole clamped DAC range), so for a 25px step
needing only ~250-300 counts of correction, the class's back-calculation
anti-windup never engages at all, regardless of how badly Ki=200 behaves.

**Made it live-testable without touching `PIDController.hpp` itself** —
respects the standing "use Phil's class completely, no mixing in
hand-rolled logic" constraint, since `setOutputLimits()` is a real,
unmodified part of the class's own API. Added `pid_wrapper_set_out_limits()`
(`pid_wrapper.cpp`/`.h`, mutates the live instance via `setOutputLimits()`
without reconstructing — deliberately does NOT clear the integral, meant
for A/B comparison mid-run) and a new `set_out_limit N` VCP command
(symmetric ±N DAC counts, `g_out_limit_counts` global, defaults to the
same `±3905`), plus `out_limit=` added to `get_status`. Rebuilt (clean,
`text=43688`), reflashed cleanly. `fta_closed_loop_step_response_vcp.py`
got a matching `--out-limit` arg, ground-truth `out_limit` read back via
`get_status` (same pattern as the notch ground-truth fix), and a plot
annotation that appears only when the limit is tightened from the
default.

**Tested tightening to ±500 counts (well above the ~278-count steady-state
need for a 25px step, but tight enough that windup-driven overshoot
past that should get clawed back) at two gain points:**
- **Kp=1.75/Ki=200** (the original pre-ROI-change gains this whole
  thread is trying to recover): still badly unstable, chaotic
  oscillation even at rest before the step — no different in character
  from the untightened result
  (`results/fta_closed_loop_step_response_fullrate_kp1750_ki200_outlimit500.png`).
- **Kp=1.75/Ki=20** (previously the borderline "marginal, bounded
  beating" case *without* the tightened limit): with the limit tightened
  to ±500, this became **fully chaotic — WORSE than without the limit**,
  diverging even during the pre-step baseline hold
  (`results/fta_closed_loop_step_response_fullrate_kp1750_ki20_outlimit500.png`).

**Conclusion: tightening the output limit is not the fix, and can
actively make a marginal case worse.** This is a real, useful negative
result, not just "didn't help": it confirms the instability found at
these gains and this rate is a genuine linear stability-margin problem
(loop gain vs. the plant's real phase lag at the control bandwidth these
gains push toward), not a saturation/anti-windup problem — the system
goes unstable well before ever coming close to needing anti-windup
protection in the first place. The Ki=20 regression under a *tighter*
limit is consistent with a classic nonlinear failure mode (saturation +
integral action + delay can themselves create or worsen a limit cycle),
not a coincidence.

**Practical implication for the user's question about reintroducing the
old hand-rolled proactive integral clamp**: not expected to help either,
for the same reason — that technique is also fundamentally an
anti-windup mechanism (bounding the integral state itself rather than
reactively undoing a saturated step), and this result shows anti-windup
design of any kind is not what's gating stability here. Recommended
NOT pursuing that path (also avoids reopening the "use Phil's class
completely, don't mix in hand-rolled logic" decision for a change
unlikely to fix the actual problem). The genuinely still-open question
is why the linear stability margin collapsed between the pre-ROI-change
rate and the current ~465Hz rate — not yet tied to a specific mechanism
(the 38.5Hz resonance was checked via the notch filter and ruled out as
the explanation in the entry above).

**Output limit reset back to the default `±3905`** before leaving
hardware idle (confirmed via `get_status`) — the tightened value was
purely diagnostic, not adopted. **Current honest best working
configuration remains unchanged: Kp=1.75, Ki=15**, notch off (no
measurable benefit), output limit at its default (tightening it doesn't
help, so no reason to deviate from default). `set_out_limit`/
`--out-limit` are kept as a permanent live-tunable feature (cheap to
keep, useful if this needs revisiting) even though this particular test
came back negative.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`). Firmware (`pid_wrapper.cpp`/`.h`, `main.c`'s
`set_out_limit` command + `get_status` field) and
`fta_closed_loop_step_response_vcp.py`'s `--out-limit` support are both
uncommitted. **Not yet done**: sine-tracking validation at Kp=1.75/Ki=15
(the true-rate honest best point) — the natural next step now that a
real stable operating point exists, and the actual test of whether this
settles the "does the loop reject the real 10-20Hz disturbance" question
rather than just a step-response proxy; a Kp sub-2.5 fine bisection at
the true rate; the D-term retried at the true (un-throttled) rate
(everything on record for D predates the throttle fix and is unvalidated
at the real rate); the still-open "why did the stability margin collapse
at the higher rate" question.

### Sine-tracking validation at Ki=15 reveals a much bigger problem: T≈0.10-0.16 at 5-20Hz means ~85-90% of a real disturbance passes through unrejected — a second host-timestamp-bucketing bug found and fixed along the way; then a properly-built, live-tunable control-rate throttle unexpectedly recovers the OLD fast/clean Ki=200 behavior, though sine validation shows it's still not enough (2026-08-19, same day)

**Ran the first real sine-tracking test against today's honest best point
(Kp=1.75/Ki=15, no notch) using the on-board sine generator.** Found
`fta_closed_loop_onboard_sine_test.py`'s own `send_command`/
`send_command_timed` never got the backlog-reset fix from the entry
above (separate function, own copy) — `start_sine` failed outright until
ported over. Also found the abort path never called `stop_sine`, so a
command that actually landed firmware-side (confirmed via `get_status`
showing `sine=1`) but lost its confirmation reply left the sine
generator latched on after a false-alarm abort — fixed by verifying
`start_sine` against `get_status` ground truth (added `sine`/
`sine_freq_millihz` to `STATUS_FIELD_RE`) instead of trusting the reply
alone, matching this session's established notch/out_limit pattern.

**Real result, amplitude=25px (150um pk-pk):** gain 0.098-0.163 across
1-10Hz — **much lower than hoped**. User's reaction, correctly: "we
can't have such low gain, we need to use this to put the fiber at exact
positions, not generate waves." This reframed what the number means:
for a unity-feedback loop, T (tracking gain) and S (disturbance
sensitivity) satisfy S=1-T — a low T at a frequency IS poor rejection at
that frequency, not a separate/different concern. T≈0.10-0.16 means
S≈0.84-0.90: **if a real 5-10Hz beacon wobble hit the rig right now,
84-90% of it would still show up as position error, essentially
uncorrected.** This is the sine test finally proving, by direct
measurement, what the whole day's stability-vs-speed fight implied: the
"safe" Ki=15 is stable but has nowhere near enough closed-loop bandwidth
for the actual mission.

**Second host-timestamp-bucketing bug found, same class as the ring-down
test's original bug, in TWO more scripts.** User asked, correctly
suspicious of the low sample rate: "are we having the same time binning
issue... at 400hz i'd expect a 10hz sine wave to look smoother." Checked
directly: `fta_closed_loop_onboard_sine_test.py`'s `_reader_thread` used
`time.monotonic()`, never the `tick=` field it was already parsing —
77.7% of consecutive samples landed on the exact same host timestamp in
one recorded run. Fixed the same way as `fta_ringdown_test.py` before it:
timestamp from `tick=` (added a `t0` no longer needed, cleaned up the
now-unused parameter). Refit the numbers with corrected timestamps: gain
barely changed (0.129/0.105 vs 0.125/0.128 at 5/10Hz) — confirms the low
gain is real, not a timestamp artifact (the least-squares sine fit over
hundreds of points is fairly robust to this kind of jitter, unlike the
ring-down test's single-cycle peak-spacing measurement, which the same
bug corrupted badly).

**Checked `fta_closed_loop_step_response_vcp.py` too — same bug, worse
implications.** It also parses `tick=` but timestamps off
`time.monotonic()`. Unlike the sine script, this one's `t_step` (when the
step was commanded) is measured on the HOST clock with no firmware-
reported echo to anchor against, so switching straight to tick-based `t`
would leave `t_step` on a different, unsynchronized clock. Fixed
properly: record both a host arrival timestamp and `tick_ms` per sample,
then fit an affine mapping (`np.polyfit`, least-squares over thousands of
samples so per-sample OS jitter averages out) from host time to firmware
tick, and map `t_step` through that fit. Rerunning the Kp=1.75/Ki=15
baseline with the fix: rise time 1484ms→**2586ms**, overshoot 10.5%→
**1.5%**, settling now resolves at **2984ms** (previously never converged
in the recorded window) — a real, substantial correction, not noise.
Visually, the corrected plot shows a single clean damped approach over
~3s; the old bucketed version's compressed time axis had made it look
faster and noisier than it really is. **This means every precise rise/
settling-time number reported earlier today (Ki=15/19/20/30, the
Kp=2.5/3.5 checks, both out_limit tests) should be treated as
directionally correct but not trustworthy to the millisecond** — the
stable-vs-unstable classifications themselves are unaffected (those come
from raw amplitude, not timing), only the precise ms figures.

**Checked whether the "wobble in the undriven axis" the user noticed
during testing meant cross-axis coupling needs its own controlled axis.**
Compared `y` (undriven) statistics directly: clean Ki=15 run — std=0.47px,
range=2.3px (noise floor); every unstable run tested (Ki=19/200/20 with
various notch/out_limit settings) — std=8.5-12.5px, range=47-80px. The
wobble tracks X-axis instability, not independent cross-coupling — real
evidence against needing a second controlled axis right now, though worth
rechecking once X is genuinely stable through a real disturbance test
(not yet done at a fully validated operating point).

**Answered a Pi-side question**: switching back to `640x200` and still
seeing ~448Hz confirms the ROI mode itself (binning, coordinate scaling)
never changed — the throughput win came from the I2C baud raise
(100kHz→400kHz) and moving auto-track recentering off the hot capture
loop onto a background thread, both mode-independent; `640x100` was a
separate, additional lever, not a prerequisite.

**User's hypothesis, tested properly: does throttling the control rate
back to ~200Hz (this time with a genuinely clean, atomically-paired
rate+`ts_s` mechanism, not the earlier one-off `DIAG_CONTROL_INTERVAL_MS`)
recover the OLD Kp=1.75/Ki=200 behavior?** Built `set_ctrl_rate MILLIHZ`
(`pid_wrapper_set_ts()` + `g_control_interval_ms`, both firmware/
`pid_wrapper.cpp`/`.h`) — ONE command sets both the throttle gate and the
PID's `ts_s` together, specifically to prevent the two ever silently
drifting apart the way the removed `DIAG_CONTROL_INTERVAL_MS` diagnostic
did earlier the same day. `MILLIHZ=0` disables the throttle (full
telemetry-driven rate, default); a positive value throttles to that rate.
Added matching `--ctrl-rate-milli` to `fta_closed_loop_step_response_vcp.py`,
ground-truth `ctrl_rate_millihz`/`ctrl_interval_ms` via `get_status`
(same pattern as notch/out_limit), and a high-contrast red "THROTTLED"
plot badge (top-right, notch's badge owns top-left) shown only when
active, plus a title suffix — a throttled run can't be mistaken for a
full-rate one at a glance.

**Result: Kp=1.75/Ki=200 throttled to 200Hz is genuinely clean and fast
again** — 168ms/2.0%/193ms then, on an immediate repeat, 163ms/1.9%/201ms
(`results/fta_closed_loop_step_response_throttle200_kp1750_ki200*.png`)
— essentially matching the historical pre-ROI-change numbers
(141-297ms range), and confirmed reproducible, not a fluke. **This
directly contradicts the earlier same-day finding** ("Chasing 'why is the
loop unstable'" entry) that the same nominal config (Kp=1.75/Ki=200,
throttled ~200Hz control rate via `DIAG_CONTROL_INTERVAL_MS`, `ts_s`
matched) was catastrophically unstable (1179.6% overshoot). Compared the
old failing run's own saved npz directly: identical `base_dac_y`/
`step_px`/`kp_milli`/`ki_milli`, and the raw `x` trace genuinely swings
from 2.9 to 535.0px — a real amplitude blowup, not a timestamp artifact
(overshoot is computed from amplitude alone, unaffected by either
timestamp bug). **Could not identify a concrete mechanism explaining the
discrepancy** — the old throttle gate and the new `set_ctrl_rate` gate
are functionally equivalent on inspection, and `ts_s` appears to have
been correctly matched in both cases from what's still readable in
comments/history. Flagging honestly as an unresolved mystery rather than
claiming a mechanism: something about the old one-off diagnostic
implementation (or conditions at the time) differed from today's more
careful, live-tunable version, but what exactly isn't established.

**Sine-tracking validation at the recovered config (Kp=1.75/Ki=200,
throttled 200Hz), 2.5px/15um pk-pk, informed by the earlier T=S-1
lesson**:

| freq | gain (T) | implied \|S\| |
|---|---|---|
| 5 Hz | 0.342 | 0.658 |
| 10 Hz | 0.223 | 0.777 |
| 15 Hz | 0.205 | 0.795 |
| 20 Hz | 0.235 | 0.765 (lag past the ±90° wraparound boundary, lower confidence) |

Clean, stable, amplitude-limited traces throughout (`results/fta_closed_loop_onboard_sine_throttle200_ki200_*Hz_15umpp.png`)
— no instability at any tested frequency. **A real, meaningful
improvement over Ki=15** (T was 0.10-0.16 there) — roughly 2-3x better
gain across the band — but **still far short of good rejection**: S
staying at 0.66-0.80 across 5-20Hz means 66-80% of a real disturbance in
that band would still show up as position error. **This is the clearest
evidence yet that fast STEP-response settling and good DISTURBANCE
REJECTION at 10-20Hz are not the same property** — the historically
"best" pre-ROI-change configuration was always somewhat limited here; it
just settles a single big jump quickly, which is a different thing from
continuously cancelling an oscillating input. Consistent with the much
earlier open-loop finding (see "Pushed sine tracking to 5/10/15/20Hz" and
the fine-sweep/resonance sections further below) that the actuator/plant
itself has real rolloff in this band, not fully fixable by PID tuning
alone regardless of which specific instability this session has been
chasing.

**Practical recommendation**: adopt Kp=1.75/Ki=200 + `set_ctrl_rate
200000` as the new best working point — strictly better than Ki=15 on
every measured axis (settling ~15x faster, tracking gain ~2-3x higher at
5-20Hz), reproducibly stable. But be explicit that this does NOT fully
solve the "hold exact position against a real 10-20Hz disturbance"
mission on its own — real residual error will remain. Worth exploring:
whether an intermediate throttle rate (e.g. 300-400Hz, between the
unstable full ~465Hz and the conservative 200Hz) does better than either
end; and revisiting the D-term / Kp headroom now that a working, live-
tunable rate control exists to explore around.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`). Firmware (`pid_wrapper.cpp`/`.h`'s `pid_wrapper_set_ts`,
`main.c`'s `set_ctrl_rate` command + `get_status` fields, `TX_MSG_MAX_LEN`/
`line[]` grown to 400 for the extra fields) and both Python scripts'
timestamp fixes / `--ctrl-rate-milli` support are uncommitted. **Not yet
done**: understanding why the old throttle attempt failed when the new
one doesn't; an intermediate-rate sweep (300/400Hz) to see if there's a
better point than 200Hz; Kp headroom re-exploration now that Ki=200 is
recoverable; D-term retried at a genuinely working configuration; a
longer-duration/repeated sine validation for statistical confidence (n=1
per frequency so far).

### Real closed-loop delay measured directly: ~11.5ms, not the previously-assumed ~41ms — changes the phase-margin picture significantly (2026-08-19, same day)

User asked, after the "is PID even sufficient given S+T=1" discussion:
does the mission need unity gain (yes, essentially — S=1-T, so poor
rejection IS what a low T at a frequency means), and what's actually
capping achievable bandwidth. Reasoned that the empirical pattern all
day (Ki/Kp capped regardless of anti-windup/notch tuning) is the
signature of loop delay eating phase margin, and flagged that the only
delay number on record (~41ms, from the "RESOLVED (2026-08-06)" sine-lag
section) was measured over a fundamentally different, slower path — the
VCP-relay test methodology (camera → I2C → Nucleo print → USB VCP →
Python read) — not today's real control path, which reads I2C telemetry
directly in memory with no VCP round trip at all. Needed a real,
directly-measured number for the actual path before reasoning further.

**Built a measurement designed to avoid every host-timing trap hit this
session** (the on-board sine generator's original negative-lag bug, two
separate host-timestamp-bucketing bugs) by keeping BOTH ends of the
measurement on the firmware's own clock, zero host timestamps involved.
New firmware command `pulse_step DELTA` (`main.c`): applies a DAC step to
`dac_y` and latches `g_pulse_step_tick = HAL_GetTick()` in the same
atomic action, open-loop and amp-enabled only (same guard pattern as
`set_x`/`set_y`). Added `pulse_tick=` to `get_status` so the exact
applied tick survives even if the command's own confirmation reply gets
lost under load (routine, same as every other VCP reply this session) —
a retried `get_status` recovers it from firmware memory instead.

Built `fta_loop_delay_test.py`: one pulse per trial (matching this
project's "don't read VCP replies while the reader thread owns
`ser.readline()`" discipline — the pulse itself is fire-and-forget during
recording, `pulse_tick` is only read back via `get_status` AFTER the
reader thread stops), pre/post windows timestamped entirely via the
firmware's `tick=` telemetry field, baseline computed from the pre-pulse
window, onset detected as the first post-pulse sample exceeding
`max(1px, 3×pre-pulse noise std)`.

**Result: 11.5ms mean delay, very tight (std=1.7ms, range 10-15ms across
6/6 usable trials, alternating step direction)** — `results/fta_loop_delay_20260818T220222Z.png`.
Visually confirmed clean: flat pre-pulse baseline, sharp unambiguous
onset in every trial, no ambiguity in picking the threshold crossing.
Bonus: each trial's post-step trace visibly rings down at a period
consistent with the ~38.5Hz resonance already characterized via the
dedicated ring-down test — a nice independent corroboration of that
number from a completely different measurement.

**This is much smaller than the ~41ms figure this project had been using
in reasoning about phase margin, and changes the picture meaningfully.**
Recomputing: an 11.5ms pure delay alone doesn't consume 90° of phase
until ~22Hz (vs. ~6Hz using the old, wrong 41ms figure) — real headroom
remains from a pure-latency standpoint well into the project's actual
10-20Hz target band. **This means the earlier reasoning ("PID can't work
because of loop delay, needs a Smith predictor or hardware fix") was
likely overstated for THIS real control path** — the ~41ms number was
correct for the old VCP-relay test methodology, just not representative
of the real closed-loop path that was actually being reasoned about.

**Practical implication**: given delay itself has real headroom to
~20Hz+, the more likely dominant constraint on today's stability ceiling
is the plant's own resonance dynamics (~38.5Hz, ζ≈0.105, lightly damped —
directly visible in this test's own ringdown) rather than raw sensor/
transmission latency. This reopens two options with more confidence than
before: (1) the notch filter approach (tried today, found not to move
the Ki/Kp boundary) may be worth retrying now that the throttled-200Hz
recovery is understood and the delay budget is known to be generous —
the earlier notch test's null result might have been confounded by the
same throttle/rate confusion this session worked through afterward, not
a real dead end; (2) hardware stiffening/damping of the flexure, which
directly targets the resonance, looks more clearly relevant now that
resonance (not delay) is the more likely dominant constraint. A Smith
predictor is less obviously necessary given how modest the real delay
turned out to be — worth deprioritizing relative to the resonance-focused
options above.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`, confirmed via the script's own cleanup + final
`get_status`). Firmware (`pulse_step` command, `g_pulse_step_tick`,
`pulse_tick=` status field) and `fta_loop_delay_test.py` (new file) are
uncommitted. **Not yet done**: repeating at other operating points (only
dac_y≈2048 tested); distinguishing how much of the 11.5ms is
camera/detection/transmission vs. the actuator's own electrical/
mechanical onset (this measurement gives the total, not the breakdown);
retrying the notch filter now that the throttle confusion is resolved;
using this real delay number to properly evaluate a Smith-predictor-style
compensator if the resonance-focused options don't fully close the gap.

### Control-step timing regularity measured directly; a boxcar smoothing pre-filter implemented and tested — reveals the ORIGINAL Kp=1.75/Ki=200 instability has stopped reproducing at all for unexplained reasons, and once a genuinely-still-unstable config was found to test against, smoothing added NOTHING beyond what throttling alone already provides (2026-08-19, same day)

Direct follow-up to "why did throttling help" — still unanswered after the
delay measurement above. User's sharpened question: is it raw rate, or
specifically how REGULAR the control-update timing is; and would a
rolling/boxcar average (a real anti-aliasing pre-filter) do better than
the existing throttle's naive skip-based decimation (which just keeps
whichever single sample crosses the interval gate and discards the rest,
doing nothing to reduce noise)?

**Part 1 — measured real control-step timing regularity, not just rate.**
Used telemetry's own `dac_y=` field as a firing-detector (`apply_dac` is
only ever called from `run_closed_loop_step` in closed-loop mode, so
`dac_y` changes value only on a real control-step firing) to reconstruct
actual control-step intervals from firmware-tick-timestamped data —
`scratch_ctrl_jitter_check.py`. First attempt was contaminated by
DAC-value quantization during near-settled periods (many consecutive
samples share the same rounded integer `dac_y` even though the underlying
state is still moving, inflating apparent "gaps") — fixed by restricting
analysis to each condition's own active fast-changing transient window,
found empirically rather than assumed.

**Result: full rate CV (std/mean of firing intervals) ≈ 71% (median 4ms,
mean 5.7ms, real tail to 20+ms) vs. throttled-200Hz CV ≈ 44% (median
7ms, mean 8.3ms, shorter tail)** — `results/fta_ctrl_jitter_check_final.png`.
Genuinely more regular under throttling, not just slower — supports the
consistency hypothesis, though n=21 for the throttled case is a small
sample and 44% CV is still real jitter, not a clean metronome.

**Part 2 — implemented a real boxcar pre-filter to test noise/aliasing
directly, decoupled from rate.** New firmware state: `g_smooth_sum`/
`g_smooth_count` accumulate every confident telemetry sample regardless
of whether a control step fires that cycle; `g_smoothing_enabled`
(`set_smoothing 0|1` VCP command, `smoothing=` in `get_status`) toggles
whether `run_closed_loop_step` gets the accumulator's mean (reset after
each firing) or just the latest raw sample, same as today. Deliberately
independent of `set_ctrl_rate` — full-rate+smoothing, throttled+smoothing,
and throttled-without-smoothing (already had) can all be tested and
compared to separate "does averaging help" from "does throttling help."
Also reset on `set_mode closed_loop` engagement (bumpless-transfer-style,
same reasoning as `pid_wrapper_reset()`). Build clean, flashed clean.

**First test looked like a huge, clean win — but wasn't.** Kp=1.75/
Ki=200, full rate (no throttle at all), smoothing ON: 161ms rise, 3.5%
overshoot, 247ms settling — clean and stable, matching the throttled-only
result almost exactly, at the FULL update rate. Looked like decisive
confirmation of the noise-reduction theory.

**Caught before trusting it: re-ran the identical full-rate/NO-smoothing
baseline that was catastrophically unstable earlier the same day
(1180% overshoot) — it no longer reproduces at all.** Freshly re-run,
smoothing explicitly OFF, otherwise identical config: 211ms rise, 2.3%
overshoot, 233ms settling — clean, stable, visually confirmed
(`results/fta_closed_loop_step_response_fullrate_nosmoothing_recheck_kp1750_ki200.png`).
**This means the "smoothing fixed it" claim was confounded, not real** —
Kp=1.75/Ki=200 has apparently become stable at full rate for reasons
entirely unrelated to smoothing, the same unexplained-mystery pattern as
the throttle-recovery entry above. Mathematically this also makes sense
in hindsight: at full rate, `run_closed_loop_step` fires on essentially
every confident packet, so the boxcar accumulator almost always contains
exactly 1 sample when it fires — averaging 1 sample is a pure no-op,
so smoothing genuinely could not have been responsible for a behavior
change at full rate.

**Found a genuinely-still-unstable config to test against properly**:
Kp=2.50/Ki=15 at full rate reproduces real, growing-oscillation
instability right now (239.1% overshoot, `results/fta_closed_loop_step_response_fullrate_recheck_kp2500_ki15.png`)
— confirms the system hasn't become universally stable at every gain,
just at the specific Ki=200/Kp=1.75 point tested most today. Retested
this SAME config with smoothing ON: **still unstable** (194.7% overshoot,
visually near-identical growing oscillation,
`results/fta_closed_loop_step_response_fullrate_smoothing_kp2500_ki15.png`)
— smoothing does not rescue a genuinely unstable full-rate case, matching
the "near no-op at full rate" expectation above, not contradicting it.

**Then tested where smoothing could actually matter — on top of
throttling, where the accumulator genuinely holds multiple samples.**
Same Kp=2.50/Ki=15: throttled-200Hz/no-smoothing is stable but slow
(2223ms rise, 1.9% overshoot, 2482ms settling); throttled-200Hz WITH
smoothing is **essentially identical** (2236ms rise, 1.7% overshoot,
2513ms settling) — visually indistinguishable traces
(`results/fta_smoothing_vs_throttle_comparison.png`, 4-panel: both
full-rate traces unstable/near-identical, both throttled traces
stable/near-identical). **Decisive: boxcar smoothing adds no measurable
benefit beyond what throttling alone already provides**, at least for
this configuration and the ~5ms window size the current throttle
interval implies.

**Net conclusion, correcting the earlier (wrong) excitement**: the real
stabilizing lever found today is RATE REDUCTION (throttling) itself, not
noise/aliasing filtering — the boxcar hypothesis, while a reasonable and
worth-testing idea, does not hold up under a properly-controlled test.
This still leaves "why does reduced rate help" as the open, unresolved
question from the entry above (the measured 71%-vs-44% CV difference is
real but, combined with this result, looks more like a correlate of
throttling than an independent causal lever on its own). **A second,
equally important finding**: the original Kp=1.75/Ki=200 full-rate
instability that motivated this entire day's investigation no longer
reproduces at all, for reasons unconnected to any software change made
today (thermal drift, mechanical settling, or some other physical/
environmental factor, none of which can be diagnosed from this
laptop-only session). This casts a real shadow over confidence in
ANY of today's "fixes" — if the underlying system's behavior can shift
this much without any code change, today's validated-stable
configurations aren't guaranteed to stay validated.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`, confirmed via `get_status` after every test).
Firmware (`g_smooth_sum`/`g_smooth_count`/`g_smoothing_enabled`,
`set_smoothing` command, `smoothing=` status field) and both Python
scripts' `--smoothing`/`--ctrl-rate-milli` support are uncommitted.
`scratch_ctrl_jitter_check.py` is ad hoc, not committed-quality.
**Not yet done**: understanding what changed to make the original
Ki=200 full-rate case stable (the single most important open question
now, arguably more important than the smoothing/throttle question this
entry set out to answer); testing a LARGER boxcar window (independent
of the 5ms throttle interval) in case the current ~5ms window is simply
too short to show a benefit; repeating the currently-unstable
Kp=2.5/Ki=15 config over time to see whether IT also spontaneously
stabilizes, which would be strong evidence for a real physical/
environmental drift rather than a one-off fluke.

### Notch filter retested honestly (correct rate/ts_s pairing this time) — real, substantial sine-tracking improvement across 5-20Hz; Kp still hard-capped even with the notch; a real safety gap found and fixed in the sine test script (2026-08-19, same day)

Direct continuation toward the actual T=1 goal, per the plan: real closed-loop
delay is only ~11.5ms (measured earlier today), leaving more phase-margin
headroom than assumed, which points at the ~38.5Hz resonance (not raw
latency) as the more likely dominant constraint — so retry the notch, this
time with a fair comparison (today's earlier notch test was confounded by
the throttle/`ts_s` mismatch bug, since fixed).

**Sanity-checked first**: Kp=1.75/Ki=200/notch@38.5Hz/throttled-200Hz — 133ms
rise, 2.7% overshoot, 189ms settling, matching (slightly beating) the
already-known-good no-notch baseline. Notch doesn't hurt the known-good
point.

**Ki pushed with the notch active** (Kp=1.75 fixed, throttled 200Hz):

| Ki | rise | overshoot | settling | character |
|---|---|---|---|---|
| 200 (baseline) | 133ms | 2.7% | 189ms | clean |
| 400 | 35ms | 9.1% | 134ms | clean-ish, faster |
| 550 | 10ms | 33.2% | 275ms | real ringing, persistent low-level buzz |
| 800 | 6ms | 62.4% | 342ms | bounded but sustained ringing throughout the window — too aggressive |

**Chose Ki=400 as the working point** — meaningfully faster than the
Ki=200 baseline with acceptable (not excessive) overshoot, well short of
the real ringing that starts around Ki=550.

**Kp still hard-capped even with the notch active**: Kp=3.5/Ki=200/notch
→ 1255% overshoot, never settles — confirms (again) that Kp headroom is
not what the notch buys; Ki remains the only usable lever, same
conclusion as every earlier gain search this project has done.

**Real, substantial sine-tracking improvement — the actual test that
matters.** Kp=1.75/Ki=400/notch@38.5Hz/throttled-200Hz, 2.5px/15um pk-pk
(same protocol as every other sine check today):

| freq | gain (T), Ki=200 no notch | gain (T), Ki=400 + notch | implied \|S\| |
|---|---|---|---|
| 5 Hz | 0.342 | **0.713** | 0.287 |
| 10 Hz | 0.223 | **0.509** | 0.491 |
| 15 Hz | 0.205 | **0.293** | 0.707 |
| 20 Hz | 0.235 | **0.314** | 0.686 (lag past ±90°, lower confidence per the usual single-tone wraparound caveat) |

Roughly 1.5-2.3x better tracking gain across the whole band, biggest win
at 5-10Hz. Visually confirmed clean, stable, real sine-following (not
chaotic) at every tested frequency
(`results/fta_sine_notch_ki400_{5,10,15,20}Hz_15umpp.png`). **Genuine
progress toward the T=1 goal, not there yet** — S still 0.29-0.71 across
the band, best at 5Hz and degrading toward 15-20Hz, so a real 10-20Hz
beacon wobble would still be meaningfully (though now much less
severely) under-corrected.

**Real safety gap found and fixed, live, mid-session.** `fta_closed_loop_onboard_sine_test.py`'s
`main()` had NO exception handling around the hardware-interacting
section at all — when the `get_status` verification after `start_sine`
raised (the same VCP-reply-loss pattern hit repeatedly all session, not
rare), the script crashed without ever reaching its cleanup code, leaving
the sine generator running with the amp energized. Happened twice in a
row live this session, both requiring manual intervention
(`stop_sine`/`set_mode open_loop`/`set_y 95`/`amp_disable` sent by hand)
before it was caught and fixed. **Fixed properly**: extracted the
hardware-interacting body into `_run_sine_test()`, wrapped in `main()`'s
`try/finally` with a new `emergency_cleanup()` (each shutdown command in
its own `try/except` so one failing doesn't block the rest — must never
itself raise, since it runs from a `finally`). Also made the
`start_sine`-verification `get_status` call more resilient in its own
right: catches a `RuntimeError` from `get_status` and proceeds anyway
(with a clear `WARNING:` printed) rather than aborting, since a lost
reply here doesn't mean the command didn't land — matches the same
"ground truth over trusting a reply, but don't over-index on any single
verification attempt" lesson already applied elsewhere this session.
**A real bug was introduced and caught during this refactor**: the
extracted `_run_sine_test()` initially referenced `duration` without it
being passed in (an outer-scope variable computed in `main()`) — caught
by re-reading the diff before trusting it, fixed by threading `duration`
through as an explicit parameter. Verified live afterward: the fixed
script hit the exact same `get_status`-after-`start_sine` failure on the
very next run, printed the new warning, and completed normally instead
of crashing — the fix works, confirmed under the real failure condition,
not just in theory.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`, confirmed via `get_status` after every test).
Firmware config left at Kp=1.75/Ki=400/notch=38.5Hz,Q=3/throttled-200Hz.
`fta_closed_loop_onboard_sine_test.py`'s exception-safety fix is
uncommitted. **Not yet done**: pushing Ki further with a finer
bisection between 400 (clean-ish) and 550 (real ringing) to see if
there's a faster point that's still acceptably clean; checking whether
the notch's Q (currently 3.0, untried at other values) trades off
against this differently; a longer-duration/repeated sine validation
(n=1 per frequency); whether the persistent "Kp=1.75/Ki=200 spontaneously
stable" mystery from the entries above also affects THIS result's
durability over time, not yet checked.

### PIDController.hpp made dt-aware (real per-call elapsed time, not a fixed ts_) — a well-motivated, correctly-implemented fix that does NOT solve the Kp limitation, giving a decisive, convergent negative result across three independent hypotheses (2026-08-19, same day)

User's sharp diagnostic question, prompted by the jitter-consistency
measurement two entries above: `PIDController.hpp`'s `calculate()` takes
no `dt` argument at all -- `ts_` is fixed at construction, so every call
does `integral_ += error * ts_` regardless of how much real time actually
elapsed. Since this loop fires on irregular telemetry arrival (not a
fixed timer), and real inter-call jitter was just measured directly
(~71% CV full-rate, ~44% throttled), this is a real, structural
mismatch, not a hypothetical concern -- the previous hand-rolled
controller (pre-2026-08-18) measured genuine per-call `dt` via
`HAL_GetTick()`, a property knowingly given up when adopting Phil's class
verbatim.

**Decision: modify `PIDController.hpp` minimally rather than switch to a
different library.** Reasoned through with the user: most simple
embedded PID libraries assume a fixed-timer caller (the normal case) and
would have the identical problem; switching libraries would also discard
everything learned today about this specific algorithm's behavior
(the anti-windup mechanism, the filter design) and require re-deriving
tuning from scratch. A small, additive change -- same P/I/D terms, same
back-calculation anti-windup, same EMA-filtered derivative, only the
timing math changed -- was judged to honor the spirit of "use his
design" better than either leaving a known-wrong assumption in place or
replacing the algorithm outright. This is a real, explicit reversal of
the earlier "use his code completely unmodified" instruction, done with
the user's go-ahead in this specific case, not unilaterally.

**Implementation**: `calculate()` gained an optional `dt` parameter
(default -1.0, falling back to the constructor's `ts_` for the first
call after construction/reset); the derivative filter's smoothing
coefficient is now recomputed per call from the real `dt` too (`fc_` is
stored instead of a precomputed fixed `alpha_`). `pid_wrapper_calculate()`
gained a matching `dt_s` parameter, threaded from `main.c`'s
`run_closed_loop_step()`, which now measures real elapsed time from
`g_last_ctrl_step_tick` (the same variable the throttle gate already
used) before the caller updates it for the new firing. `g_last_ctrl_step_tick`
is now also reset to 0 on `set_mode closed_loop` engagement (0 is the
documented "never fired since engagement" sentinel, preventing a bogus
multi-second "dt" on the first step after re-engaging). Build clean, zero
warnings, flashed clean (no post-reflash silence glitch this time).

**Sanity check first**: Kp=1.75/Ki=200/notch/throttled-200Hz — 93ms rise,
2.7% overshoot, 133ms settling, matching (slightly beating) the pre-dt-
aware baseline. No regression.

**Real test 1 — does dt-awareness alone stabilize Ki=200 at full rate
(unthrottled)?** Yes -- 84ms/5.6%/194ms, clean. **But this doesn't prove
anything on its own**: Ki=200/full-rate was already known to have
mysteriously become stable on its own hours earlier (the still-
unexplained mystery from the entries above), so a clean result here is
consistent with either "dt-awareness fixed it" or "it was already fixed
by whatever the mystery factor is" -- can't distinguish between them
using this config.

**Real test 2 — the decisive one: retested the currently-still-unstable
Kp=2.5/Ki=15 at full rate, dt-aware, notch active.** Still badly unstable
-- 717.1% overshoot, never settles
(`results/fta_dtaware_fullrate_kp2500_ki15.png`). **dt-awareness does
NOT rescue a genuinely unstable config, same negative-result pattern as
the notch (which also failed on this exact axis: Kp=3.5/Ki=200/notch was
1255% overshoot) and smoothing (which also failed on this exact
Kp=2.5/Ki=15 config in the two entries above).** Three independently-
motivated, mechanistically-different fixes -- resonance notch,
timing-jitter correction, noise-reduction pre-filter -- have now all
failed to move the Kp stability boundary. Only throttling (real control-
rate reduction) has ever moved it.

**Real test 3 — does dt-awareness at least match throttled cleanliness
on the Ki axis?** Kp=1.75/Ki=400/notch/dt-aware, full rate (unthrottled):
27ms rise, 19.5% overshoot, 283ms settling -- stable but visibly more
marginal than the equivalent throttled result (9.1% overshoot, 134ms
settling, from the entry above). A persistent low-level buzz is visible
throughout the recorded window
(`results/fta_dtaware_fullrate_kp1750_ki400.png`), not present in the
throttled version. **Throttling still provides a real, independent
benefit even after dt-awareness removes the timing-precision confound.**

**Net conclusion, now well-supported by convergent evidence across three
different fixes**: the Kp/bandwidth limitation this whole day has been
chasing is very likely NOT primarily a timing-precision, jitter, resonance-
aliasing, or noise problem -- all three of those hypotheses were tested
directly and properly, and none moved the needle. The pattern instead
looks like a genuine rate-vs-phase-margin effect: running the control
loop faster (even with everything else done correctly) pushes the
closed-loop crossover frequency up, closer to where the real plant's
combined phase budget (delay + resonance + whatever else hasn't been
individually characterized) runs out. This reframes the remaining
options: further software-only fixes in this family (notch tuning,
timing correction, filtering) have now been reasonably exhausted without
solving the core constraint; the most likely-to-matter remaining levers
are either accepting throttled operation as a genuine design choice
(not a workaround for a bug, but the actual right operating point given
today's evidence), or a hardware change to the actuator/flexure that
raises the plant's own phase margin at any control rate.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`, confirmed via `get_status` after every test).
Firmware (`PIDController.hpp`'s dt-aware `calculate()`, `pid_wrapper.cpp`/
`.h`'s `dt_s` threading, `main.c`'s `run_closed_loop_step`/
`cmd_set_mode` changes) is uncommitted, left flashed and running.
Current best validated operating point remains Kp=1.75/Ki=400/notch=
38.5Hz,Q=3/**throttled-200Hz** (not full rate) -- dt-awareness is a real,
correct fix worth keeping (it removes a genuine source of numerical
error regardless of whether it solves the headline problem), but doesn't
change the practical recommendation. **Not yet done**: characterizing
the plant's phase budget more directly (e.g. a proper frequency-response
sweep of the open-loop plant, not just the closed-loop sine tests done
so far) to confirm the "rate pushes crossover into the phase-margin wall"
theory rather than leaving it as an inference; whether an intermediate
throttle rate (300-400Hz) with dt-awareness+notch both active does
better than the plain 200Hz/no-dt-awareness point already found.

### Second control axis added for completeness (dac_x <- cy, identical Kp/Ki to the primary axis) — basic non-divergence smoke test only, real validation deferred (2026-08-19, same day, ~30min available)

User's explicit framing: not expected to fix the bandwidth/Kp problem,
just makes the system "more complete" -- two independent PID controllers
with identical parameters, one per axis, given ~30 minutes available.

**Real correctness hazard identified before writing any code**: the
locked-optics calibration (2026-08-12 entry above) found `dac_y`'s effect
on `cx` is +0.126 px/count but `dac_x`'s effect on `cy` is **-0.104
px/count** -- opposite sign. Feeding the second axis literally identical-
sign gains into an identical control law would drive it the WRONG
direction (positive feedback, immediate divergence) -- "identical
parameters" has to mean identical Kp/Ki *magnitude*, with the correction
sign-flipped to match the real (opposite-sign) plant.

**Implementation** (`pid_wrapper.cpp`/`.h`, `main.c`): `pid_wrapper`
generalized to own a second `PIDController` instance (`g_pid2`),
reconstructed in lockstep with the first by every existing gain/fc/ts/
limit setter -- no new tuning commands needed, "identical" falls out for
free. New `pid_wrapper_calculate2()`. `main.c` gained
`run_closed_loop_step_axis2()` (deliberately simpler than the primary
axis's `run_closed_loop_step` -- no notch, no boxcar smoothing, no sine
generator, just the dt-aware PID + the sign-flipped correction), called
right after the primary axis on the same telemetry packet with the same
measured `dt`. Target for the second axis is auto-captured (bumpless,
"hold cy where it already is") on `set_mode closed_loop` engagement --
no separate `set_target_y` command, kept deliberately minimal.
`cmd_set_axis`'s closed-loop guard extended from "block `set_y`" to
"block `set_x` and `set_y`", since `dac_x` is now actively controlled
too. Build clean, zero warnings, flashed clean.

**Verified**: basic non-divergence smoke test only, at the known-safe
Kp=1.75/Ki=200/throttled-200Hz/no-notch config -- `dac_x` moved from its
idle value (95) to a bounded 295 and held there (not pinned at either
DAC clamp), while the primary axis converged normally (cx -> target as
expected). A backwards sign would very likely have driven `dac_x` to a
rail almost immediately at this Ki -- it didn't, which is real, if not
exhaustive, evidence the sign correction is right. **Not validated**:
no step-response or sine-tracking test of the second axis specifically
(time ran out) -- the smoke test only observed it holding steady near
its auto-captured target under near-zero cy error, not actively
correcting a real, deliberate cy disturbance. Real validation (a genuine
cy step/disturbance, watching `dac_x` correct it) is the natural next
step before trusting this axis for anything beyond "doesn't immediately
blow up."

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`, confirmed via `get_status`). All changes
(`pid_wrapper.cpp`/`.h`, `main.c`) uncommitted. **Not yet done**: a real
disturbance-rejection test of axis 2 (deliberately displace cy, confirm
dac_x corrects it in reasonable time, not just holds steady); whether
axis 2 needs its own notch/smoothing given it's a different physical
pathway (untested whether it has its own resonance behavior); `get_status`
doesn't yet report `target_y` for visibility (skipped for time -- `dac_x`/
`tel_y` are visible and were sufficient for this smoke test).

### Was axis 2 actually needed? A direct A/B test, same Y-step, controller on vs. off — real, if modest, value found (2026-08-19, same day)

Direct follow-up: added `g_axis2_enabled`/`set_axis2 0|1` (`main.c`) so
the second axis can be disabled (dac_x held at its bumpless-transfer
base instead of correcting) without touching the primary axis at all --
lets the exact same Y-axis step test be run twice, axis2 on vs. off,
isolating its effect. `fta_closed_loop_step_response_vcp.py` gained
`--axis2`, ground-truth `axis2` status (same get_status pattern as
notch/smoothing), and the plot was restructured to a 2-panel figure (cx
on top as before, cy below) plus a std/range annotation and an ON/OFF
badge -- so a single saved PNG now shows both axes' behavior together,
not just the driven one. Build clean, flashed clean.

**Same Kp=1.75/Ki=200/throttled-200Hz Y-step, axis2 ON vs. OFF**
(`results/fta_axis2_on_kp1750_ki200.png` /
`..._off_kp1750_ki200.png`, combined comparison in
`results/fta_axis2_needed_comparison.png`): primary axis (cx) behaved
identically either way (~82-90ms rise, ~2% overshoot, ~144-165ms
settling -- axis2 has no effect on the driven axis, as expected). cy:
**both conditions show the same-sized instantaneous dip right at the
step** (real, physical cross-coupling excited by the Y-axis moving --
too fast for any controller to catch, present regardless of axis2) --
**but they diverge afterward**: with axis2 OFF, cy never recovers and
instead drifts slowly upward for the rest of the ~4s window, ending
noticeably above where it started (std=0.43px, range=2.00px, still
trending at the end); with axis2 ON, cy recovers back to within noise of
its original value in ~300-400ms and stays there tightly for the rest of
the recording (std=0.20px, range=1.60px).

**Answer: yes, axis 2 is doing something real, though modest in this
test.** It can't (and isn't expected to) suppress the instantaneous
coupling transient itself -- that's faster than any feedback loop can
react to. What it does do is prevent the slow post-transient drift/creep
that shows up when nothing is correcting cy, keeping it anchored near
setpoint instead of wandering. Roughly 2x tighter std, and critically a
qualitatively different long-term behavior (recovers vs. drifts away),
not just a small numeric improvement. Whether this matters in practice
depends on how large real disturbances are and how long the system runs
between corrections -- this test only used a single ~25px Y-step as the
disturbance source, not a sustained/repeated one.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`, `axis2` left enabled -- the sensible default,
confirmed via `get_status`). All changes uncommitted. **Not yet done**:
a longer or sustained-disturbance test (this was one step, not a real
operating scenario); testing axis2 on/off during a SINE test (continuous
disturbance) rather than just a step, which is closer to the project's
actual 10-20Hz beacon-wobble target than a one-off step comparison.

### Axis-2 A/B test repeated with a continuous sine disturbance instead of a one-off step — same conclusion, more decisive (2026-08-19, same day)

Direct follow-up per the user's request: the step-based A/B test above
was one disturbance event, not representative of a real operating
scenario. Added `--axis2`/ground-truth `axis2` status/a third (`cy`)
plot panel to `fta_closed_loop_onboard_sine_test.py` (mirroring the
step-response script's earlier additions) -- the sine generator now
continuously drives `dac_y` (hence `cx`) while `cy` is watched for the
full recording, axis2 on vs. off.

**Same Kp=1.75/Ki=200/throttled-200Hz, 10Hz/2.5px sine, axis2 ON vs.
OFF** (`results/fta_axis2_sine_on_10Hz.png` / `..._off_10Hz.png`,
combined in `results/fta_axis2_sine_comparison.png`, each trace zeroed
to its own start so the shapes are directly comparable despite the two
runs' bumpless-transfer baselines landing ~8px apart): primary axis (cx)
tracking gain matched closely either way (0.314 ON vs 0.299 OFF) --
confirms axis2 has no effect on the driven axis, as expected. `cy`:
**axis2 OFF drifted 0.43px over the 3s window (real, one-directional,
not just noise -- std=0.16px, range=0.70px); axis2 ON stayed within
0.03px of its start the whole time (std=0.09px, range=0.50px)** -- both
the drift magnitude AND the noise floor are roughly half with axis2 on.

**Conclusion, now on firmer footing than the single-step test**: axis 2
provides a real, measurable benefit under a continuous disturbance, not
just a one-off transient -- it doesn't eliminate `cy` movement entirely
(both conditions show similar-amplitude fast fluctuation, consistent
with real per-cycle coupling from the 10Hz drive that's too fast to
fully correct), but it reliably prevents the slow systematic drift that
appears when nothing is correcting `cy`. This matches and reinforces the
step-test finding rather than contradicting it -- same "prevents drift,
doesn't suppress the instantaneous coupling" pattern shows up under a
sustained disturbance too, which is the more representative test for
this project's actual mission.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`, `axis2` left enabled, confirmed via `get_status`).
All changes uncommitted. **Not yet done**: repeating at other
frequencies (only 10Hz tested); a longer-duration run (3s used here) to
see whether the OFF-condition drift continues growing or saturates;
whether the drift is a real physical effect (mechanical creep/hysteresis
excited by the sustained Y-axis motion) or something else, not
investigated further.

### Notch vs. no-notch re-tested properly — each pushed to its OWN max stable Ki, not compared against a fixed low-Ki baseline; a real, mixed result (2026-08-19, same day)

Direct follow-up to the "was Ki=400 a new ceiling the notch unlocked, or
just a Ki push that would've worked anyway?" question raised earlier —
never actually answered on record. Built `scratch_notch_comparison.py`
(repo root, reuses `fta_closed_loop_step_response_vcp.py`'s helpers) to
bisect each config's real ceiling properly.

**Real bug found and fixed before trusting any data**: the first two
bisection trials (Ki=300, no notch) came back with the actuator barely
moving at all post-step (std ~0.09px the whole window) despite a
commanded -25px target, and `set_mode closed_loop` got a `None` reply
both times. `fta_closed_loop_step_response_vcp.py` (and
`fta_closed_loop_onboard_sine_test.py`) trust that reply without ever
reading back the raw `mode=` field to confirm the switch actually took —
a real, previously-undiscovered gap, not just the already-known lost-
reply flakiness (which usually means the command landed even if the
reply didn't). Fixed by adding `ensure_mode()` to the scratch harness:
resend + read back raw `mode=` via `get_status`, retrying up to 4x before
treating a trial as failed. Re-ran Ki=300 with the fix: clean, real data
(5.2% overshoot) — confirms the earlier "882% overshoot" reading was
pure garbage from a closed-loop engagement that silently never happened,
not evidence of instability.

**No-notch Ki bisection** (Kp=1.75, throttled 200Hz): 300/350/400/500
all clean (5-21% overshoot), 550 ringing (29.5%), 600 ringing (42.7%) —
**real ceiling ~500**, well above the Ki=200 baseline used everywhere
else this session.

**With-notch (38.5Hz, Q=3) Ki bisection, same harness**: 400 clean
(19.9%), 450 AND 500 both ringing (31.3%/29.5%) — **real ceiling ~400**,
*lower* than no-notch's. This directly answers the open question: **the
notch never raised the Ki ceiling** — the earlier "notch let me push Ki
up" framing was a confound from comparing against a fixed low-Ki
baseline, not a real causal effect. Matches (independently reproduces,
with a properly mode-verified harness this time) the original
2026-08-19 "notch doesn't move the Ki/Kp boundary" finding.

**Sine-tracking comparison at each config's own real max (Ki=500
no-notch vs. Ki=400+notch), 2.5px/15um pk-pk, 5/10/15/20Hz:**

| freq | T, no-notch/Ki=500 | T, notch/Ki=400 |
|---|---|---|
| 5 Hz | 0.903 | 0.918 |
| 10 Hz | 0.778 | 0.924 |
| 15 Hz | 0.740 | 0.446 |
| 20 Hz | 0.881 | 0.560 |

**Notch wins at 5-10Hz, loses badly at 15-20Hz** — worse than no notch
at all, not just less-improved. Most likely cause: the notch's stopband
skirt (Q=3 @ 38.5Hz) still attenuates real 15-20Hz signal, with no extra
Ki headroom to offset it since the ceiling never actually moved. **This
reverses the conclusion of the slide built earlier today** ("Notch
filter (38.5Hz): real 10-20Hz tracking-gain improvement"), which only
ever compared notch+higher-Ki against a fixed LOW-Ki no-notch baseline —
slide and its closing-slide summary bullet corrected to match (see
`docs/session_results_2026-08-18_pid_tuning.pptx`, now 18 slides,
combined T-vs-frequency + 15Hz-raw-trace comparison figure at
`results/scratch_notch_maxspeed_comparison.png`).

**Second real bug found and fixed, in `fta_closed_loop_onboard_sine_test.py`
itself, while re-running the sine comparison**: its `get_status()`
required ALL STATUS fields to match, including `sine_freq_millihz` — the
LAST field on the STATUS line, and (confirmed live, reproduced 3x
identically) exactly the field a dropped trailing byte under load
truncates (`"sine_freq_millihz=" landing directly butted against the
next telemetry line, e.g. "...millihz=seq= 10 status=1..."`), so the
whole connectivity check failed every retry, deterministically, not
randomly. **First fix attempt overcorrected**: defaulting a missing
`sine_freq_millihz` to 0 unblocked the connectivity check but broke the
*separate* post-`start_sine` confirmation check, which specifically
needs to tell "genuinely 0" apart from "corrupted, keep retrying" — the
fabricated 0 made every sine test report `start_sine not confirmed`
(sine_freq_millihz=0, expected 5000) even when it had genuinely landed
at the right frequency, 3 times in a row. **Real fix**: `get_status()`
now takes an explicit `required` field set — the plain connectivity
check (which doesn't care about sine fields) passes a reduced core set,
the sine-confirmation check keeps the original strict requirement (all
fields, real retries on corruption, no silent default). Both scripts'
working trees carry these fixes, uncommitted.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`). Raw sine data: `results/scratch_sine_nonotch_ki500_{5,10,15,20}Hz.npz`,
`results/scratch_sine_notch_ki400_{5,10,15,20}Hz.npz`. Not yet committed:
`scratch_notch_comparison.py`, `scratch_build_notch_comparison_plot.py`,
both sine-test-script fixes.

### Open-loop plant Bode plot — real, cross-validated resonance measurement; strong evidence the 38.5Hz mechanical resonance, not delay or controller tuning, is the actual ceiling on 10-20Hz rejection (2026-08-19, same day)

Prompted by the user asking for a direction given being "pretty stuck"
after exhausting the controller-tuning space (Kp's hard ceiling never
moves; Ki's ceiling doesn't move with rate fixes, jitter fixes/dt-aware
PID, boxcar smoothing, or the notch; D actively hurts everywhere tried).
Recommended getting a real open-loop plant frequency response instead of
another closed-loop trial, since the real loop delay was already known
to be small (~11.5ms, `fta_loop_delay_test.py`, 2026-08-18) — the
missing piece was never measuring the *plant itself* (actuator + optics
+ flexure) independent of any controller, only ever characterizing
closed-loop behavior or a single ring-down point.

**Built a firmware-level open-loop sine generator** (`main.c`):
`start_open_sine FREQ_MILLIHZ AMPLITUDE_COUNTS CENTER_COUNTS` /
`stop_open_sine`, direct analog of the existing closed-loop
`start_sine`/`update_sine_target` but writing straight to `apply_dac()`
(bypassing the PID entirely) — only armed in `MODE_OPEN_LOOP` (rejects
otherwise, and `set_mode closed_loop` force-clears it defensively, same
belt-and-suspenders pattern as every other mode-gated feature in this
file). Deliberately reuses the EXISTING `dac_y=`/`tick=` telemetry relay
fields as ground truth for both the commanded waveform and its real
timebase — no new telemetry field needed, since `apply_dac()` already
updates `g_last_dac_y` regardless of who calls it. `get_status` gained
`open_sine=`/`open_sine_freq_millihz=` (added BEFORE the existing
`sine=`/`sine_freq_millihz=` fields, deliberately, given the fresh
first-hand experience above with trailing-field truncation under load).
`TX_MSG_MAX_LEN`/`line[]` grown 400->460 for the two new fields (a real
`-Wformat-truncation=` warning from GCC's own worst-case bound, not just
convention-following headroom this time — fixed by growing the buffer,
not silencing the warning).

Rebuilt via the project's established bypass-CubeIDE toolchain
(`arm-none-eabi-gcc`/`make` from the STM32CubeIDE-bundled 13.3.rel1
tools, NOT `rm -rf`'d first — an incremental `make all` on the
already-CubeIDE-populated `Debug/`), 0 errors/0 warnings on the second
pass. Flashed via `STM32_Programmer_CLI` (SN `066FFF515152827187153930`,
matches this file's on-record board) — download verified, **no
post-reflash silence this time** (unlike nearly every other reflash this
session), `get_status` responded immediately with the new fields present
and zeroed.

**Built `fta_open_loop_bode_test.py`** (new, repo root): same
paced-write/ground-truth-verification/`emergency_cleanup`-in-`finally`
discipline as every other VCP script this session. Fits BOTH the
measured `cx` and the reported `dac_y` against the same `sin(wt)/cos(wt)`
basis (identical trick to `fit_tracking()` in the closed-loop onboard
sine script) — gain comes out in real physical units (px measured per
DAC count commanded, not a % of commanded amplitude), and the lag/phase
diff is immune to any host t0 error. Supports `--sweep "f1,f2,..."` to
run a whole frequency list in one connection/one amp-enable window.

**Swept 16 points, 1-50Hz, amplitude=300 DAC counts, base dac_y=2048**
(the established clean small-signal operating point) in one run, ~2-8s
per point (`duration = max(2.0, 8.0/freq)`), hardware confirmed idle
before and after:

| freq (Hz) | gain (px/count) | phase (deg, unwrapped) |
|---|---|---|
| 1 | 0.0919 | 7.1 |
| 2 | 0.0900 | 10.4 |
| 3 | 0.0891 | 14.0 |
| 5 | 0.0891 | 21.6 |
| 7 | 0.0893 | 28.7 |
| 10 | 0.0909 | 40.4 |
| 13 | 0.0951 | 52.0 |
| 15 | 0.0992 | 58.3 |
| 18 | 0.1062 | 70.1 |
| 20 | 0.1159 | 79.1 |
| 25 | 0.1353 | 106.8 |
| 30 | 0.1845 | 130.0 |
| 35 | 0.2838 | 159.0 |
| 38.5 | 0.6777 | 218.2 |
| 42 | 0.4234 | 291.9 |
| 50 | 0.1143 | 349.6 |

(Phase is the raw per-point fit unwrapped via `np.unwrap` across the
sorted-by-frequency sequence — the same fundamental single-frequency
wraparound ambiguity this project has hit before, resolved here by the
data's own smooth monotonic progression rather than left aliased; see
`results/fta_open_loop_bode_summary.png`.)

**A real, clean, textbook result — the first genuine open-loop frequency
response this project has ever measured across a wide band:**
- Gain is flat, ~0.089-0.095 px/count, from 1-13Hz — benign low-frequency
  plant behavior, consistent with the DC-ish gain (~0.09-0.13 px/count)
  already established via the minor-loop hysteresis check and the final
  locked-optics calibration.
- Phase grows smoothly through the 10-20Hz target band: **40° at 10Hz,
  79° at 20Hz** — real margin remains in the plant alone at these
  frequencies, well short of 180°.
- Gain climbs sharply and phase rotates fast starting ~20-25Hz, peaking
  in a sharp resonance at **38.5Hz, gain 0.68 px/count (~7.6x the DC
  gain)** — matching the ring-down test's 38.5Hz finding EXACTLY, from a
  completely independent method (forced swept-sine vs. free decay). Two
  unrelated measurement techniques landing on the same number is strong
  corroboration this resonance is real, not an artifact of either
  method.
- 5Hz's measured lag (12.0ms) closely matches the directly-measured pure
  loop delay (~11.5ms, `fta_loop_delay_test.py`) — a third independent
  cross-check landing in the right ballpark.

**Fit quality visually confirmed, not just trusted from the printed
numbers**: `save_point_plot()` (rewritten mid-session after the first
version's full-duration-only view became an unreadable wall of
overlapping cycles above ~15Hz — the 38.5Hz point crams 84+ cycles into
one panel) now shows a 2x2 grid per frequency (full-duration context +
a zoomed first-~6-cycle view, for both the measured `cx` and commanded
`dac_y`, raw scatter plus the actual fitted sine overlaid) — saved to
`results/fta_open_loop_bode_<freq>Hz_<timestamp>.png`, one per swept
frequency, regenerated from the saved `.npz` raw data (fit coefficients
now persisted too) without touching hardware again. Spot-checked 1Hz,
20Hz, and 38.5Hz (the resonance peak, where fit trustworthiness matters
most) — all show the fitted sine tracking the raw telemetry closely, in
both the compressed full-duration view and the individually-resolvable
zoomed view.

**Why this matters for the "tune more vs. fundamental change" question**:
closing the loop with more Ki pushes the gain-crossover frequency
higher. Even though 10-20Hz itself has real margin, any crossover
pushed toward ~20-25Hz+ runs straight into the resonance's rapid phase
rotation (~140 degrees in under two octaves) and eats the margin —
directly explaining why Ki has hit a hard, narrow ceiling all session
regardless of rate fixes, jitter fixes, or the notch. It also explains
the notch's mixed result above: gain/phase are already climbing well
before 38.5Hz, so a notch centered exactly there doesn't cover the
rising skirt that actually intrudes into where the crossover needs to
sit. This is a real, physical, narrow-band mechanical resonance, not
delay and not a filter-design choice — the strongest evidence yet for
the "fundamental (likely hardware) change" side of the question, e.g.
stiffening/damping the flexure to push the resonance higher or reduce
its Q, rather than continued controller tuning.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`, confirmed via `get_status` after the sweep and after
the plot-regeneration pass, which touched no hardware at all). Firmware
(`start_open_sine`/`stop_open_sine`, `TX_MSG_MAX_LEN`/`line[]` grown to
460) and `fta_open_loop_bode_test.py` are both new/uncommitted. **Not yet
done**: repeating at a different base `dac_y` operating point or a
different excitation amplitude (only 2048/300-counts tested, and 300
counts is well into the region where the resonance peak alone produces
~200px+ swings — worth checking whether the resonance's gain/Q are
amplitude-dependent, given this project's earlier finding of real
small-amplitude nonlinearity in the *closed-loop* sine tests); the y-axis
(dac_x<->cy) pathway not characterized this way at all; using this real
measured phase/gain data to actually design a compensator (e.g. proper
loop-shaping) rather than continued trial-and-error gain search, if
tuning is still pursued alongside/before any hardware change.


### Found and fixed: the notch filter's sample-rate bug was still live in firmware — every throttled notch test (including both "retested" entries above) measured the wrong filter (2026-08-27)

Found while chasing why the fresh lead-compensator scratch tests
(`scratch_lead_*_ki200`, `results/`) were unstable even layered on top of
the supposedly-working notch+Ki=200 baseline. Went back to the actual
firmware source rather than trusting the comments describing it, and
`cmd_set_notch` (main.c) was still calling `notch_configure(freq, q,
457.5f)` — a hardcoded literal, not `g_ctrl_rate_millihz`. Every one of
the five scratch runs (and, on inspection, the sanity-check line in the
"Notch filter retested honestly" entry above, which explicitly says
"throttled-200Hz") ran with `ctrl_rate=200Hz`, not the assumed 457.5Hz.

**What that actually does, computed from the real RBJ biquad math**: a
digital notch designed for 38.5Hz assuming a 457.5Hz sample rate, but
then clocked at a different real rate `fs`, lands at `38.5 * (fs/457.5)`
Hz instead. At throttled 200Hz that's **16.82Hz** — not 38.5Hz. Frequency
response of that exact (mis-clocked) filter: **-0.10dB at the real
38.5Hz resonance** (essentially no effect on the thing it was built to
suppress) and **-37.98dB at 16.8Hz**, a deep, narrow hole carved dead
center in the 10-20Hz band this project actually needs clean. -4.6dB
bleeds into 15Hz too.

**Consequence**: every "notch@38.5Hz" result taken while throttled —
both the "Notch filter retested honestly" entry above (Ki pushed to 400,
real substantial improvement claimed) and the "Notch vs. no-notch
re-tested properly ... mixed result" entry — was measuring this wrong,
~16.8Hz filter, not the intended 38.5Hz one. Neither conclusion can be
trusted as evidence about the actual 38.5Hz notch until reproduced with
the fix below. (Full-rate/unthrottled notch tests, if any were run, are
unaffected — 457.5Hz was correct there by coincidence.)

**Fix applied** (`nucleo_firmware/camera_centroid_receiver/Core/Src/main.c`,
`cmd_set_notch`, ~line 2449): now calls
`notch_configure(freq, q, (float)g_ctrl_rate_millihz / 1000.0f)` —
the real current rate, exactly the same pattern `cmd_set_lead` /
`lead_configure` already used correctly (lead was never affected by this
bug). Also corrected the two doc comments (above `cmd_set_notch`, and in
the lead-compensator block) that described the hardcoded-457.5Hz
behavior as current/intentional — they now document the bug and the fix
instead.

**State left**: firmware change is written but **uncommitted, not yet
built, not yet flashed** — no `arm-none-eabi-gcc`/STM32CubeIDE toolchain
available in the environment this fix was made from, so it hasn't been
compile-verified, only reviewed by eye (change mirrors the already-proven
`cmd_set_lead` pattern exactly; brace count unchanged).

**Next steps, in order**:
1. Open in STM32CubeIDE, build, flash to the Nucleo.
2. Rerun the notch-alone Ki sweep (same protocol as "Ki pushed with the
   notch active" above: Kp=1.75, throttled 200Hz, Ki stepped 200/400/550/
   800) — see whether the ceiling/character changes now that the notch
   actually targets 38.5Hz instead of 16.8Hz.
3. Rerun `scratch_lead_notch_sanity_ki200` / `scratch_lead_notch_fix_ki200`
   (lead+notch combo, Kp=1.75/Ki=200/notch@38.5Hz/lead fz=6/fp=20,
   throttled 200Hz) — see whether the lead-compensator instability found
   in every one of today's scratch runs persists once the notch stops
   eating a hole in the 10-20Hz band.
4. Treat every throttled-mode notch number logged before 2026-08-27 as
   unverified until reproduced with this fix in place — don't cite the
   old "Ki=400 chosen as working point" or the "mixed result, never
   raised the Ki ceiling" conclusion as settled without a rerun.

### Notch fix built, flashed, and rerun — the REAL 38.5Hz notch has a LOWER Ki ceiling than either no notch or the buggy 16.8Hz one; does not explain the lead-compensator divergence (2026-08-27, same day)

Executed the "Next steps" list from the entry above. Built clean (0
errors/0 warnings, `arm-none-eabi-gcc`/`make` via the project's usual
bypass-CubeIDE toolchain), flashed via `STM32_Programmer_CLI`
(SN `066FFF515152827187153930`, matches this file's on-record board),
`get_status` confirmed alive and idle immediately after — no post-reflash
silence this time.

**Reran the notch-alone Ki sweep** (`scratch_notch_comparison.py`,
Kp=1.75, throttled 200Hz, -25px step, same explicit-mode-verification
harness used throughout this thread) at Ki=200/300/350/400/550/800, plus
a bisection at 300/350 to pin the ceiling more precisely than the
original 200/400/550/800 checkpoints:

| Ki | verdict | overshoot |
|---|---|---|
| 200 | CLEAN | 9.9% |
| 300 | CLEAN | 21.2% |
| 350 | CLEAN | 13.9% |
| 400 | RINGING | 31.1% |
| 550 | RINGING | 80.5% |
| 800 | DIVERGED | — |

**Real ceiling with the CORRECT 38.5Hz notch: clean through ~350, rings
by 400.** Plotted against the two other conditions already on record
(`fta_notch_ki_sweep_plot.py`, new committed script,
`results/fta_notch_ki_ceiling_comparison.png`):

| condition | ceiling (last clean Ki) |
|---|---|
| no notch (unaffected by the bug) | ~500-550 |
| buggy notch (~16.8Hz, mis-clocked) | ~400 |
| **real notch (38.5Hz, this fix)** | **~350** |

**Surprising result, stated plainly: the correctly-configured notch has
the WORST Ki ceiling of the three, not the best.** Worse than no notch
at all, and worse than the accidentally-mis-notched 16.8Hz version. Not
yet root-caused further — plausible candidates, none confirmed: the
notch's own phase contribution (a band-stop filter adds real phase
distortion near its center frequency, not just gain attenuation) could
be eating into the loop's phase margin right at the frequencies that
matter for the Ki boundary; or Q=3 is simply too narrow/aggressive for
this plant now that it's centered correctly. Not investigated this
session — flagged as a real open question, not a settled explanation.

**Answers the question this whole rerun was for: no, this does NOT
explain the lead-compensator divergence found earlier.** Every lead+notch
scratch trial that day ran at Ki=200 — clean under both the buggy notch
AND the confirmed-real one (9.9% overshoot here). Since the notch was
already stable alone at the exact Ki used in every lead+notch test, the
chaos seen when the lead compensator was added cannot be attributed to
"the notch wasn't real." That investigation remains genuinely open,
unrelated to this bug.

**Visual confirmation**: `results/fta_notch_sweep_panel.png` (6-panel
small multiples of the real-notch sweep, color-coded clean/ringing/
diverged) and `results/fta_notch_ki_ceiling_comparison.png` (overshoot
vs. Ki, all three conditions overlaid, symlog y-axis). Worth noting from
the panel: even the "CLEAN" traces (Ki=200/300/350) show a visible
continuous high-frequency buzz riding on the whole signal, both before
and after the step — more pronounced than earlier "clean" traces
elsewhere in this project. Didn't affect the CLEAN/RINGING/DIVERGED
classification (based on peak excursion and settling, not noise floor),
but worth keeping in mind if revisiting this data.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`, confirmed after every trial). Firmware fix is
committed (`5b23da3`) and now also build/flash-verified on real hardware
for the first time. `fta_notch_ki_sweep_plot.py` is a new, real,
committed tool (not scratch) — reusable if this sweep needs repeating
(e.g. at a different Q, or once the ceiling-regression mechanism above
is investigated). Raw sweep data:
`results/scratch_notch_cmp_notchfix_ki{200,300,350,400,550,800}_*.npz`.

**Not yet done**: root-causing why the correctly-centered notch has a
worse Ki ceiling than no notch at all (the real open question this entry
surfaces); the lead-compensator investigation itself, still unresolved
and now confirmed unrelated to this bug; Bode/ring-down characterization
of the OTHER axis (dac_x -> cy) — flagged as worth doing given the
locked-optics calibration already found the two axes have different gain
magnitude AND sign (dac_y->cx: +0.126 px/count, dac_x->cy: -0.104
px/count), so there's no reason to assume they share a resonance either,
and there's now an active flexure redesign (FEA-vs-bench slides,
`docs/session_results_2026-08-18_pid_tuning.pptx`) that would benefit
from knowing if the two axes need different treatment.

### Notch/lead sample rate made fully self-calibrating (measured, not assumed); real ~35% control-step throughput ceiling found and root-caused to double-precision PID math + axis2; fixed via float conversion + -O2 build; a real measurement-methodology bug caught along the way (2026-08-27, same day)

Direct continuation of the entry above. The "real notch has a WORSE Ki
ceiling than no notch" finding prompted a closer look at whether
`g_ctrl_rate_millihz` (the value the fixed notch now uses) is itself
trustworthy — it's the NOMINAL/requested throttle target, not the real
achieved control-step rate.

**Confirmed it isn't, by direct hardware measurement.** Built a host-side
rate-measurement technique (`dac_y=` value-change detection against
firmware `tick=` timestamps, restricted to the active step-response
transient window — same fix `scratch_ctrl_jitter_check.py` needed earlier
this session for the identical reason: near-settled periods let many
real firings round to the same integer `dac_y`, inflating apparent
gaps). Result: at ~285Hz Pi telemetry, throttled-200Hz's real achieved
rate was **~107-143Hz**, not 200Hz (-46% to -73% off nominal). At ~462Hz
telemetry, the gap shrank to ~12% (176Hz real vs 200Hz nominal) — the
mismatch is real and telemetry-rate-dependent, confirmed to swing
within a single session (telemetry itself observed at 285→462→483→
488→516Hz at different points today, all on the same rig).

**Real fix: firmware self-calibration, not another manual measurement.**
Added `g_measured_ctrl_interval_ms` (main.c) — an EMA of the REAL
inter-firing interval, updated every control step from `dt_s` (already
computed for the dt-aware PID, no new measurement needed) and initialized
to the same `1/457.5s` fallback used elsewhere. `notch_configure`/
`lead_configure` were split into `notch_compute_coeffs`/`lead_compute_coeffs`
(coefficients only) + the existing `notch_reset_state`/`lead_reset_state`
(x1/x2/y1/y2 history only) — necessary because the ORIGINAL combined
functions reset the filter's recursive history as a side effect, and
recomputing coefficients every control step (needed for continuous
self-calibration) would have zeroed that history every ~2-5ms, breaking
the filters entirely (caught before it was ever wired in, not a live
bug). `run_closed_loop_step` now recomputes both filters' coefficients
from the live EMA every firing when enabled. `get_status` gained
`meas_ctrl_rate_millihz=` for direct visibility — confirmed live: firmware's
own self-reported rate (177.84Hz) matched an independent host-side
measurement (176.0Hz) taken under similar conditions, real agreement.

**User pushback, correctly targeted: "if the Pi sends 490Hz, why does the
Nucleo only process 177Hz?"** — this surfaced a genuinely separate,
bigger problem. Distinguished the two possible causes directly:
non-confident-detection filtering (ruled out — 100% of received relay
lines were confident) vs. the firmware's single-slot "latest packet"
receive buffer coalescing multiple I2C arrivals into one when the main
loop can't cycle back around fast enough (not yet confirmed at that
point). A clean, SIMULTANEOUS measurement (raw `pkts=` growth vs.
`meas_ctrl_rate`, same window, unthrottled) found: **555 raw I2C
packets/s arriving, zero checksum errors, only ~353-364Hz (~64-66%)
actually driving a control step** — a real, structural ~35% loss, not a
throttle artifact (ctrl_rate=0 for this test).

**Isolated the cause with one cheap, no-rebuild test: `set_axis2 0`.**
Same simultaneous measurement, axis2 disabled: 432.2 raw pkts/s, 407.6-
413.3Hz achieved — **~94-96% throughput**, up from ~64-66%. Disabling the
second axis (which runs the identical PID math a second time per packet)
nearly eliminated the gap on its own — strong, direct, quantified
evidence the bottleneck was axis2's *duplicate* cost, not I2C, not
telemetry, not some other structural limit.

**Root cause: `PIDController.hpp` uses `double` throughout, and this
MCU's FPU (fpv4-sp-d16) is single-precision hardware only** — every
double op was software-emulated, a known, deliberately-accepted tradeoff
from when Phil's class was first adopted (flagged at the time as
"negligible... against this loop's real timing budget," an assessment
that didn't anticipate axis2 running the same expensive math twice per
packet). **Fixed with the user's explicit go-ahead — a second, deliberate
deviation from "use Phil's class completely unmodified"** (the first was
the earlier dt-awareness change): converted `PIDController.hpp`,
`pid_wrapper.h`/`.cpp`, and every `main.c` call site from `double` to
`float` throughout. Precision risk judged low for this application
(pixel-scale errors, anti-windup clamped, bumpless-reset every
engagement) — same P/I/D terms, same back-calculation anti-windup, same
EMA-filtered derivative, only the storage/arithmetic type changed.
Built clean (0 warnings), `.text` shrank 49200→46048 bytes (~3.2KB),
consistent with real double-emulation code being removed, not just a
source-level change with no effect.

**Rebuilt project also found and fixed at -O0 (no optimization) this
entire session** — a debug-friendly setting CubeIDE generates by
default, never revisited. Edited both `Debug/Core/Src/subdir.mk` and
`Debug/Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk` (`-O0`→`-O2`, the only
two files with the flag) directly — **not persisted in `.cproject`**, so
this reverts to `-O0` if the project is ever regenerated from inside
CubeIDE's GUI, same class of gotcha as every other hand-added setting in
this file; reapply by hand if that happens. Deleted only the `.o`/`.d`
build outputs (not `subdir.mk`/`makefile`/`objects.mk`) to force a real
recompile — the earlier `rm -rf Debug/Core Debug/Drivers` mistake from
2026-08-13 (deletes CubeIDE's generated rule files, not just outputs)
deliberately NOT repeated. `.text` shrank further, 46048→31468 bytes
(~32% smaller) — real optimization, not a no-op.

**Net throughput result, axis2 ON, unthrottled, same measurement
technique throughout** (raw telemetry rate varied between each test,
Pi-side, outside firmware control — the fraction/ratio is the fair
comparison, not the absolute Hz):

| build | raw pkts/s | achieved rate | fraction |
|---|---|---|---|
| double (original) | 554.8 | 353-364Hz | ~64-66% |
| float, -O0 | 341.2 | 317.4-323.7Hz | ~93-95% |
| float, -O2 | ~460 (see correction below) | 413-482Hz | ~90-105%* |

*the -O2 row's fraction briefly looked impossible (raw pkts/s computed as
516.5, HIGHER than the achieved rate would allow given the Pi's own
~480Hz report) — see the measurement-bug correction immediately below;
the corrected raw rate (459.9/s) restores a sane, sub-100% fraction.

**A second real measurement bug, caught by direct user pushback ("516
can't be right, the Pi says 480").** The "clean" simultaneous-measurement
technique used firmware `uptime=` (whole SECONDS only) as the elapsed-
time denominator for a ~4s window — a `"4s"` reading can correspond to
anywhere from 4.0 to 4.99s of true elapsed time (integer truncation), and
dividing a real packet count by an artificially-small denominator
inflates the computed rate by up to ~25%. Confirmed directly: precise
Python wall-clock timing (`time.monotonic()`, taken exactly when each
confirmed status read lands, not a nominal sleep duration) gave 4.484s
real elapsed for the same window firmware `uptime` reported as exactly
4s — corrected raw rate **459.9 pkts/s**, consistent with the Pi's own
~480Hz report, not the impossible 515-516/s reported twice before this
was caught. **Lesson, worth remembering for any future rate measurement
on this rig**: never use the firmware's whole-second `uptime` field as a
timing denominator for a short window — always use host-side
`time.monotonic()` captured at the exact moment of each confirmed read.

**Also observed, real and not yet explained**: within one clean 4.5s
measurement window (float+-O2 build), `meas_ctrl_rate_millihz` swung
from 481.8Hz down to 413.3Hz — a genuine, fairly large real-time
fluctuation, not measurement noise (this was after the uptime-denominator
bug was already fixed). Could be real telemetry-rate variation on the
Pi side within that window, or something else — not investigated
further this session.

**A real, unresolved side effect of the float conversion**: rerunning the
long-established Kp=1.75/Ki=200 baseline (throttled 200Hz, no notch/lead,
axis2 on — historically ALWAYS clean, 4-10% overshoot in every earlier
test this session) came back **RINGING, 33.8% overshoot**. Directly
measured the real control-step rate under this exact config immediately
after: 173.1Hz — essentially unchanged from the pre-float measurement
(177.8Hz) under the same throttled+axis2-on conditions, so the new
ringing is NOT explained by a rate change (ruled out by direct
measurement, not assumed). Two live hypotheses, genuinely undecided:
(1) a real numerical/precision difference from the float conversion
itself (plausible but a large jump — 4-10%→33.8% — for values in this
range, where float's ~7 decimal digits should be far more than enough);
(2) something about the physical rig changed independent of any
firmware work today — this project has already documented at least one
UNEXPLAINED session-to-session behavior shift before (the original
Kp=1.75/Ki=200 full-rate instability that "stopped reproducing, for
reasons unconnected to any software change," 2026-08-19), and the most
recent commit before this session added flexure FEA-vs-bench redesign
slides, meaning the physical rig may be under active hardware
investigation in parallel. Recommended next step, not yet done: try a
comfortably lower Ki (e.g. 100) to see if it's still clean — would help
distinguish "needs modest re-tuning given today's changes" from "the
plant itself changed."

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`, confirmed via `get_status` after every test, `errs=0`
throughout). Firmware changes (self-calibrating notch/lead, float PID,
-O2 build) are all built, flashed, and hardware-verified working — but
**uncommitted** as of this entry. `ctrl_rate_millihz` left at the
established default (200000, throttled) after every test that changed
it. `scratch_verify_notch_rate.py` (repo root, not committed — ad hoc
diagnostic, same convention as this session's other scratch_* scripts)
has the transient-window-restricted rate measurement technique, in case
it's needed again — though prefer `get_status`'s own `meas_ctrl_rate_millihz`
field going forward over any host-side re-measurement, now that the
firmware self-reports it directly.

**Not yet done**: the Ki=200 ringing regression (immediate next step:
lower-Ki check, described above); root-causing the real-time
`meas_ctrl_rate` fluctuation observed in one window; re-running the
notch Ki sweep from the entry above now that both the float PID and -O2
are in place (that sweep predates both — its exact numbers should be
treated as representative of the double/-O0 build, not necessarily
still accurate); the lead-compensator investigation, still open and
unrelated to any of today's fixes; Bode/ring-down on the other axis,
flagged in the entry above, still not started.

### Revert-and-retest: double precision ruled out as the cause of the Ki=500 divergence/settling regression; two flawed measurement methods found and fixed while chasing an honest double-vs-float comparison; a new firmware ground-truth counter built; ring-down re-run on both axes and the resonance has moved from 38.5Hz to ~47-50Hz (2026-08-27)

Direct follow-up to the entry above: with float+`-O2` landed, a real Ki=500
closed-loop divergence/settling regression turned up (still diverges,
where it was historically clean), and it wasn't clear whether float
precision itself was a contributing factor or whether it was purely
mechanical (a hysteresis issue found and mostly fixed earlier the same
day via cable straightening + manual actuator jogging). User's framing:
**"revert and retest, let's just be sure."**

**Revert done properly, isolating ONLY the precision variable.**
`PIDController.hpp`/`pid_wrapper.h`/`.cpp`/`main.c`'s call sites reverted
double→float, deliberately keeping `-O2` (the build-optimization change)
constant — reverting that too would have re-conflated two variables
instead of isolating one. Rebuilt (`.text` 31468→34612 bytes, consistent
with double-emulation code reappearing), reflashed (hit the by-now-
routine post-reflash silence glitch once, resolved by reflashing again,
as it always has this session).

**Result: identical behavior under double, both tests.** Kp=1.75/Ki=500,
axis2 on: **diverged**, `max_dev=387.9px` — essentially the same
magnitude as the float build's 390.5px. Kp=1.75/Ki=200: **clean but
slow**, 17.0% overshoot / 2988ms settle — matching the float build's
post-mechanical-fix pattern closely. **Float precision is ruled out as
the cause of the Ki=500 regression** — same divergence, same character,
under double. The regression is mechanical (or at least not explained by
precision), consistent with the hysteresis finding from earlier the same
day. Restored float+`-O2` afterward (`git checkout` on the three files
that matched HEAD exactly; `main.c` needed the double→float edits
reapplied by hand since it also carries unrelated same-day changes that
must NOT be reverted — the self-calibrating notch/lead EMA and the lead
compensator).

**Follow-up thread: fixing a slide claiming this precision switch also
improved timing "consistency" (jitter/CV) — this uncovered real, deeper
measurement problems, not just noise.** The existing slide's chart
(`scratch_ctrl_jitter_check.py`, built earlier this session) compared
**full-rate vs. throttled-200Hz**, both already under the float+`-O2`
build — never actually a before/after precision comparison at all, and
its "CV roughly halved" claim quoted older, differently-sourced numbers
presented as if they were a controlled result. User caught this
directly ("slide 19 shouldn't include throttled, it should compare
before and after the switch to single precision — and what is meant by
'CV' on that slide?"). CV = coefficient of variation (std/mean) of the
firing-interval distribution, a measure of timing regularity.

**Attempt 1 (flawed): a fresh double-vs-float jitter histogram via
`dac_y`-value-change detection.** Same method `scratch_ctrl_jitter_check.py`
already used (detect when `dac_y`, the DAC-register output, changes
value, treat that as a "firing"). Result was wildly noisy even across
repeated trials on the IDENTICAL build — float CV ranged 88-127% across
3 back-to-back trials at Kp=1.75/Ki=15/full-rate, nothing like a stable
number. **User identified the root cause precisely, unprompted**: "only
measuring dac_y changes instead of firings of the PID loop is a critical
mistake." Correct — during near-settled periods the loop keeps firing
every packet, but the correction often rounds to the same integer DAC
count, so real firings go structurally invisible to this method. Not
sampling noise — a wrong measurement, and it retroactively invalidated
the ORIGINAL slide's "CV halved" claim too (same method), not just
today's fresh attempt.

**Attempt 2 (also flawed, self-caught): a throughput-fraction comparison
using `meas_ctrl_rate_millihz` (the firmware's EMA of recent firing
rate) divided by a host-measured windowed-average raw packet rate.**
Produced a **102% reading** — impossible if "control steps fired" is
truly bounded by "packets arrived." Root cause: `meas_ctrl_rate_millihz`
is a near-instantaneous EMA snapshot (~20-sample window) at the moment
of the LAST reply, while the raw-rate denominator was a true average
over the full ~4.5s measurement window — two different statistics over
different effective time spans, not directly comparable as a ratio.
Caught before the slide was rebuilt around it a second time; user's
reaction was blunt and fair ("we've made a bit of a mess... I'm not
confident in any of this now") — correctly so, two flawed methods in a
row.

**Real fix: a new firmware ground-truth counter, `g_ctrl_step_seq`
(`main.c`), per the user's own suggested design** ("number each sample
end to end and plainly see skipped numbers"). A plain `uint32_t`,
incremented unconditionally exactly once per real `run_closed_loop_step()`
firing (no proxy, no averaging), relayed per-packet as a new `cseq=`
telemetry field alongside the existing `tick=`. Firing count over any
window is now a plain integer subtraction (`last_cseq - first_cseq`);
real per-firing intervals come directly from consecutive increments'
`tick=` deltas, with no gating on `dac_y` or any derived rate. `line[]`
grown 140→165 bytes for the new field. Build clean, `.text` 31468→31500
(float+`-O2`+counter). **`scratch_cseq_measurement.py`** built as the
new host-side driver (repo root, not committed, matches this session's
scratch-script convention).

**First run with the new counter immediately surfaced a second real,
previously-invisible bug**: 4-8% of telemetry LINES were being lost in
transit per trial (83-163 of ~2000 lines), confirmed via the Pi's own
relayed `seq=` byte (u8, wraps) showing real gaps. This had been
silently undercounting the denominator in every earlier measurement this
session that counted captured lines rather than trusting embedded
sequence data — exactly what produced the impossible >100% reading.
Fixed the analysis (not the line loss itself, which is a separate,
unexplored TX-queue/host-read-timing question) by using the unwrapped
`pi_seq` delta as the raw-packet-count denominator instead of counting
captured lines — robust to line loss, since whatever DID arrive still
carries the true cumulative count. Also added a tick-delta sanity filter
(reject any inter-sample gap >500ms) after finding one genuinely
corrupted/torn line (two lines' bytes concatenated across a lost read)
had produced a 10,200-*second* fake "interval" that blew up the naive
CV to 8+ million percent — syntactically valid per the regex, semantically
garbage.

**Clean result, 3 trials each, `-O2` held constant on both sides:**

| | double, `-O2` | float, `-O2` (current) |
|---|---|---|
| throughput fraction per trial | 94.7%, 95.3%, 95.3% | 100%, 77.8%, 100% |
| pooled firing-interval CV | 26.6% (n=6330) | 37.4% (n=5814) |
| pooled mean interval | 2.30ms | 2.20ms |

No impossible values, no massive outliers, tight and reproducible across
all 3 double trials specifically. **This does not support the original
slide's "float is faster and more consistent" claim** — if anything,
double reads tighter here. Net honest conclusion: `-O2` is doing the
real work (both precisions land in the 78-100% range, both far above the
`-O0` baseline's documented ~64-66%); whether float adds anything
further on top of `-O2` is genuinely unresolved by this data (n=3 per
condition, one float trial notably lower than the other two). **Slide
19 pulled from the deck entirely** (`python-pptx` slide removal via
`_sldIdLst`/`drop_rel`, zip integrity verified afterward — no orphaned
parts, matching this project's established safe-removal recipe) rather
than left in a still-shaky state. `results/fta_cseq_jitter_final.png`
holds the real histogram; `results/fta_precision_before_after.png`,
`fta_o2_vs_precision_credit.png`, `fta_precision_jitter_histogram.png`
are earlier, superseded/flawed attempts, deliberately NOT committed.

**Ring-down test generalized to both axes and re-run — a further real,
reproducible finding.** `fta_ringdown_test.py` gained `--axis x|y`
(x=pulse `dac_y`/watch `cx`, the original/primary pathway; y=pulse
`dac_x`/watch `cy`, never ring-down-tested before) and folded in the
model-free peak/trough-spacing frequency estimate as the PRIMARY result
(previously only an ad hoc side script, `scratch_ringdown_peak_analysis.py`,
built 2026-08-19 after `curve_fit` converged on the wrong answer twice on
the same data) — `curve_fit`'s fit is now reported as secondary/for
damping-ratio only. Needed a `TELEMETRY_RE` fix first (same "add a wire
field, forget the other consuming script" class of bug this project has
hit repeatedly — `cseq=` broke this script's regex the same way `tgt=`/
`dac_y=`/`tick=` broke others before it).

**x-axis (dac_y→cx, primary): 47.3Hz, then 47.0Hz on an immediate
repeat** — tight, reproducible (`curve_fit` gave nonsense both times,
zeta≈1.0, confirming it's still not trustworthy here, consistent with
this project's established finding). **y-axis (dac_x→cy, second axis,
first-ever ring-down): 50.1Hz, one trial.** Both axes land close to each
other, notably NOT close to the previously-documented 38.5Hz. Since both
today's measurements and the original 38.5Hz used the same trustworthy
tick-timestamped peak-spacing method (not the older, since-discredited
host-timestamp-bucketed approach), this reads as a real, reproducible
shift in the physical resonance since that measurement was taken, not a
methodology artifact — worth stating plainly if this goes into a
bench-vs-FEA comparison, not silently swapped in as if it were always
the number.

**Real cosmetic bug found and fixed while reviewing the y-axis plot**:
the "amp on" marker (`rise_idx = argmax(x)`) assumed the driven excursion
is always a rise — true for the positive-gain x-axis, wrong for the
y-axis, whose locked-optics-calibration gain is NEGATIVE (dac_x→cy:
-0.104 px/count, 2026-08-12) — so the real driven excursion there is a
dip, and `argmax` was landing on an unrelated later feature. Fixed to
pick whichever of max/min is further from the pre-pulse baseline,
regardless of sign. Purely a plot annotation — never affected the actual
fit-window anchor (`argmin`-based, already sign-agnostic) or the
frequency measurement itself.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`) after every test, confirmed via `get_status`.
Firmware is float+`-O2`+the new `g_ctrl_step_seq`/`cseq=` counter — the
committed double↔float question is resolved (float is fine, keep it;
`-O2` is the real throughput fix); the `cseq` counter addition and
`fta_ringdown_test.py`'s axis/peak-spacing generalization are new,
real, keep-worthy changes pending commit. `docs/session_results_2026-08-18_pid_tuning.pptx`
has slide 19 removed (zip-integrity-verified).

**Not yet done**: the physical cause of the 38.5Hz→~47-50Hz resonance
shift (rig change? cable/hysteresis fix side effect? never investigated
further); the y-axis resonance has only one trial, unlike x's two
matching runs — worth a repeat before trusting it equally; the
double-vs-float `-O2` throughput-fraction question is still only 3
trials per side, with one float outlier (77.8%) unexplained; whether the
real ~4-8% VCP line loss found via the `cseq`/`pi_seq` cross-check is
itself worth root-causing (TX queue depth? host read timing?) is
untouched; the Ki=500 divergence's actual mechanical root cause (beyond
"not float") remains unconfirmed.

### Confident-packet / link completeness measured directly (both ~100%); a real, 100%-reproducible STATUS-reply truncation bug found and fixed; fresh open-loop Bode sweep confirms the same resonance shape (2026-09-01)

Picked up the "is every incoming I2C message getting a completed PID
loop" question. Two new ISR-level counters added to `main.c`:
`g_confident_packet_count` (increments in `process_beam_packet()` when
the packet's status bit0 is set) and reuse of the existing
`g_ctrl_step_seq` (already exposed per-packet as `cseq=`, added
2026-08-27 — see "Revert-and-retest" above) now also surfaced via
`get_status` as `confident_pkts=`/`cseq=`. Built `scratch_confident_vs_fired.py`:
diffs two `get_status` snapshots bracketing a closed-loop window. Result,
3 trials, axis2 on, unthrottled, Kp=1.75/Ki=15: 99.96%/99.95%/100.00%
per-trial, pooled **6423/6425 (99.97%) confident packets completed a PID
loop** — the single-slot `g_latest_beam` mailbox's theoretical drop risk
(two I2C completions landing before the main loop drains
`g_new_packet_ready`) is real but rare, ~1 in 3200, not a practical
limiter.

Separately measured **link completeness** (does every Pi send reach the
Nucleo's I2C receiver at all): unwrapped the Pi's own u8 `seq` byte
across the relay stream and compared its reconstructed delta against
`(pkts + errs)` delta over the same window — both firmware-side counters,
immune to the already-known VCP-relay host-side line loss. 3 trials,
open_loop/amp off: 3610/3610, 3672/3672, 3591/3591 — **0 lost across
~10,873 real sends.** Both results say the I2C link + packet-to-PID
pipeline are essentially lossless; nothing here is limiting control
performance.

**Built a real jitter histogram using `cseq` as ground truth** (real
per-firing intervals from consecutive `tick=` deltas, not the old
`dac_y`-value-change proxy this file has already documented as unreliable
near settled periods). Kp=1.75/Ki=15, axis2 on, unthrottled, 3 trials
pooled: dominant cluster n=6250, mean=2.17ms, std=0.86ms, **CV=39.6%** —
a real, trustworthy consistency number. One 174ms outlier (1/6251,
0.016%) reported separately rather than folded into the CV — since
`cseq` only increments on a confident detection, this most likely
reflects a real, brief detection dropout, not a comms/firmware stall.
Both a link-completeness/PID-completion slide and a jitter-histogram
slide were added to `docs/session_results_2026-08-18_pid_tuning.pptx`
(now 25 slides at that point) — see the git history for the merge with
another device's concurrent FEA-vs-bench/ring-down slide work
(`dcd1cf6`/`672163c`), resolved via `git show origin/master:<path> ><path>`
rather than `git checkout --theirs` (which failed once due to a
lingering `soffice.bin` holding the file open).

**Real, 100%-reproducible firmware bug found and fixed while re-running
the existing open-loop Bode sweep (`fta_open_loop_bode_test.py`, built
2026-08-19, see "Open-loop plant Bode plot" above).** Every
`start_open_sine` confirmation read came back garbled (`freq=100,
expected 1000` — consistently losing 2-3 trailing zero digits). Two
theories were chased and disproven before finding the real cause: (1)
digit corruption in transit — ruled out by slowing per-character pacing
20ms→50ms, which changed nothing; (2) a bad host-side rate measurement
that had reported "164Hz" against the Pi's real ~284Hz (traced to
timestamping *after* a blind fixed-size `ser.read(3000)` call instead of
at the moment a confirmed reply parses — fixed by reusing the
already-proven timestamp-at-parse helpers from other scripts; corrected
measurement: 286.6Hz, matching the Pi). Neither fix resolved the garbled
replies. **Real root cause, found via direct raw-byte inspection**: the
STATUS reply was being torn mid-transmission with no line break
(`...open_sine_freq_millihz=10seq=102 status=1 x=...`) — a direct 20-sample
test showed **0/20 complete replies, every single one exactly 614 bytes**,
a perfectly consistent length that was the key clue this was a fixed
buffer-size truncation, not a random timing race. Traced to
`TX_MSG_MAX_LEN`/`cmd_get_status`'s local `line[]` still at 520 bytes
(last bumped 460→520 earlier the same day) while the real STATUS line had
grown to ~521-522 bytes once `confident_pkts=`/`cseq=` were added above —
`enqueue_tx()`'s silent truncation clamp (`if (len >= TX_MSG_MAX_LEN) len
= TX_MSG_MAX_LEN - 1`) was clipping the exact same offset every call, and
`ser.readline()` kept reading past the missing `\n` into the next
telemetry line, producing the corrupted-looking concatenation. Fixed:
`TX_MSG_MAX_LEN` and `line[650]` both grown 520→650. Rebuilt (`.bss` grew
by exactly 1040 bytes = 8×130, confirming the fix's arithmetic), reflashed,
confirmed 20/20 clean replies afterward (521-522 bytes each). **This bug
had nothing to do with the actuator, timing, or transmission pacing** —
every earlier theory this incident chased was a red herring; the fix is
purely "grow the buffer to match the current STATUS line length," and is
the same recurring bug class this file has hit before (add a wire-format
field, forget to bump the buffer that holds it).

**Real hardware consequence mid-incident**: with the buffer bug
un-diagnosed, the sweep script's flawed confirmation logic issued new
`start_open_sine` calls for the next frequency without ever confirming
the previous one had stopped, cascading into overlapping/rapidly-changing
sine drives (`dac_y=1771` observed live while supposedly idle). Not a
hazard-level event — this is a small, low-force flexure/voice-coil
actuator, not something that can hurt anyone — but real, unnecessary
component stress and bad data, secured immediately (`stop_open_sine`,
mode/dac/amp reset, `clear_estop`, confirmed idle) once caught.

**Fresh 18-point sweep, 1-55Hz, amplitude reduced 300→150 counts for
margin** (this incident is not itself evidence 300 was unsafe — it's just
extra headroom given what happened), confirms the same qualitative shape
already on record from the original 2026-08-19 16-point sweep: flat gain
1-13Hz (~0.088-0.093 px/count), real phase margin through 10-20Hz
(40° lag at 10Hz, matching the earlier sweep almost exactly), a sharp
resonant peak with the sampled grid maximum landing at **40Hz** (gain
0.997, ~11x the low-frequency baseline). Given this sweep's grid only
samples 35/40/44Hz around the peak, this 40Hz figure should be read as
"grid-limited," not a precise re-measurement — it's consistent with,
not a correction to, both the 38.5Hz ring-down number and the later
~47-50Hz re-measurement already on record above; the true peak could sit
anywhere in the unsampled 35-47Hz window depending on which session's
number is currently accurate. `results/fta_open_loop_bode_20260901_*`
(per-frequency plots + summary) regenerated after the fix; single-point
10Hz validation (gain=0.0892 px/count, lag=10.7ms/38.6°) matches the
historical baseline (0.0909, 40.4°) closely.

**State left**: hardware safely idle. `TX_MSG_MAX_LEN`/`line[650]` fix,
`scratch_confident_vs_fired.py`/`scratch_link_completeness.py` (scratch,
not committed), and the two new pptx slides were committed/pushed this
session (see git log around this date). **Not yet done**: the
`TELEMETRY_RE`-duplication bug class (every VCP telemetry consumer
script re-declares its own copy, independently — flagged again, not yet
fixed with a shared module) hit again this session in
`fta_open_loop_bode_test.py`/`fta_closed_loop_step_response_vcp.py`
(missing `cseq=`), same as several times before.

### Fresh lead compensator, designed from real Bode data, fails violently — a loop-gain simulation and a real-hardware higher-Q notch sweep both point at the plant resonance itself, not an implementation bug, as the reason every compensator trick (D-term, notch, now lead) keeps failing (2026-09-01, same day)

With a trustworthy fresh Bode sweep in hand (previous entry), designed a
lead compensator from first principles: standard sizing formula
(`α=(1-sinφ)/(1+sinφ)`, `fz=fc√α`, `fp=fc/√α`) targeting ~55-60° added
phase margin at a 20Hz crossover, using the 40Hz worst-case phase
estimate from the fresh sweep (not the ring-down number, per an explicit
decision to proceed conservatively rather than re-measure the
38.5Hz-vs-47-50Hz discrepancy) — landed on **fz≈15.4Hz, fp≈26Hz**
(`set_lead 15400 26000`).

**Tested on hardware (Kp=1.75/Ki=400, unthrottled, no notch) — violently
unstable, worse than the no-lead baseline, not better.** No-lead baseline
(same Kp/Ki): 105.8% overshoot, never settled. With lead: **598.1%
overshoot**, never settled, and the recorded trace shows oscillation
building even BEFORE the commanded step — the signature of a genuinely
unstable (not just poorly-damped) closed loop, independent of any input.
This closely reproduces an earlier, previously-unresolved "sustained
instability" finding from a prior session that a bumpless-transfer fix
had reduced but not solved.

**Checked the implementation for a bug before concluding it's fundamental
— found none.** Hand-derived the bilinear-transform discretization of
`C(s)=(1+s/ωz)/(1+s/ωp)` and compared directly against
`lead_compute_coeffs()`/`lead_apply()` in `main.c`: the `Kz=1/(π·fz·T)`,
`Kp_=1/(π·fp·T)` constants and the `b0,b1,a1` formulas match the standard
substitution exactly, and the difference equation
(`y[n]=b0·x[n]+b1·x[n-1]-a1·y[n-1]`) is the correct realization. Sanity
checks on the actual fz=15.4/fp=26 coefficients at the real ~280Hz rate:
DC gain ≈1.0 (correct for a lead compensator) and Nyquist-band gain
≈1.69 (matches the designed fp/fz ratio exactly). **The math is right —
this ruled out a coefficient bug as the explanation.**

**Root cause found instead via a loop-gain simulation against the real
measured Bode data** (continuous-domain: PI's own frequency response
`Kp-jKi/w`, the lead's `(1+jw/wz)/(1+jw/wp)`, and the plant's measured
gain/phase interpolated from the fresh sweep, all multiplied together —
valid regardless of where in the loop the lead filter is actually applied,
since LTI blocks in series commute for loop-gain/stability purposes; this
firmware applies lead to the *measurement* before computing error, not to
the PI output, which doesn't change this analysis). Both configurations
show loop gain **crossing back above 1 a second time near the resonance,
with phase already well past -180°**:

| | 2nd crossover | phase there | naive "margin" |
|---|---|---|---|
| no lead (baseline) | 36.4Hz | -217° | -37° |
| with lead (fz=15.4/fp=26) | 34.3Hz | -177° | +2.6° |

The lead's extra ~1.2-1.5x gain boost in the 15-35Hz band (inherent to
any lead compensator — phase lead cannot be added without adding gain)
lands right where the plant's resonance gain is already climbing steeply
(20Hz=0.11, 25Hz=0.136, 30Hz=0.182, 35Hz=0.282 px/count, well before the
40Hz peak itself) — pushing the second crossover to a lower frequency
with essentially zero margin instead of the comfortable ~55-60° the
single-worst-case-point design assumed. **This is a real, physical
mechanism, not a coefficient bug**: the phase-margin design method used a
single worst-case phase estimate rather than checking the full measured
gain curve, and the plant's resonance skirt is wide/steep enough that any
added gain in that band is dangerous regardless of how it's implemented.

**Caveat on the "margin" numbers above**: the naive "180+phase at a gain=1
crossing" formula only strictly applies to a loop with one crossing. This
loop's gain dips below 1 then rises again (three total crossings), and
when the same formula was applied to a config already known clean on real
hardware (Kp=1.75/Ki=15, throttled 200Hz) it predicted -103° margin —
contradicting known ground truth. So the ABSOLUTE margin numbers above
aren't trustworthy; the COMPARATIVE conclusion (lead's second crossing is
lower-frequency and worse than no-lead's, both landing in the
resonance-driven danger zone) is the load-bearing part of this finding,
not the exact degree values. A real Nyquist encirclement count would be
needed for trustworthy absolute margins; not built (time-boxed).

**Followed up by testing whether a much tighter notch could recover
margin, on real hardware rather than trusting more simulation** (the
simulation's passband-cost check — Q=3 through Q=30 all showed <0.4% gain
loss in the 10-20Hz band relative to no notch — suggested tightening
should be free or beneficial). `scratch_notch_q_sweep.py` built (reuses
`scratch_notch_comparison.py`'s harness), swept **Q ∈ {8,12,20} × Ki ∈
{200,400,600,800}** at Kp=1.75, notch centered 40Hz, throttled 200Hz, on
top of the already-fixed real (not mis-clocked) notch from the
"Notch fix built" entry above. **Result: every single one of the 12
combinations was worse than Q=3, and worse monotonically as Q
increased** — Q=8/Ki=200 (a value safe under both no-notch and Q=3):
RINGING (91.4% overshoot); Q=12/Ki=200: GROWING/UNSTABLE; Q=20/Ki=200:
DIVERGED outright. Every higher Ki at every Q: DIVERGED. **This directly
contradicts the simulation's magnitude-only prediction** — real hardware
says tighter is worse, not better or neutral.

**Best explanation**: the passband simulation only checked *magnitude*
attenuation in 10-20Hz, never *phase*. A notch's phase distortion swings
across a much wider band than its visible magnitude dip, and at a ~200Hz
sample rate against a 40Hz center (only ~5x oversampled), that phase
penalty is real, extends into the control-relevant band, and gets worse
as Q tightens — the opposite of what the magnitude-only view suggested.
Not confirmed by a follow-up phase-domain simulation (time-boxed), but
consistent with every other result this investigation produced.

**Net conclusion — four independent approaches have now failed against
this resonance**: the D-term (multiple cutoffs, multiple sessions), the
original Q=3 notch, this session's higher-Q notch sweep (Q=8/12/20), and
now a freshly-designed lead compensator with verified-correct math. This
is no longer "try another filter" territory — it's strong, repeated,
convergent evidence that the plant's resonance is a genuine mechanical
bandwidth ceiling that firmware/controller tricks cannot route around at
this Kp/Ki operating regime. Recommend treating the flexure hardware
redesign (already in progress per the FEA-vs-bench slides) as the primary
path forward, not further controller tuning.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0
dac_x=95 dac_y=95`) after every trial, confirmed via `get_status`.
`scratch_notch_q_sweep.py` committed at repo root. **Not yet done**: a
proper Nyquist/discrete-time stability analysis (to get trustworthy
absolute margins, not just the comparative conclusion above); a
phase-domain check of why tighter notches make things worse in practice;
whether a hardware fix (stiffer/damped flexure) genuinely raises the
resonance frequency enough to matter, vs. just moving the same problem —
not testable until the new flexures arrive.

### dt-aware notch/lead coefficients tried and reverted (made things worse); a real bug found and fixed in PIDController.hpp's D-term (computed on error, not measurement, despite its own docstring); D-term retested — no longer catastrophic, but a real unresolved set_fc anomaly limits how far the new numbers can be trusted (2026-09-01, same day)

Follow-up to the user's sharp question: "are we sure it's handling dt correctly, given the jitter in the samples?" Reread `run_closed_loop_step()` and found a real, concrete inconsistency: the PID's own `dt_s` (real per-call interval, dt-aware since 2026-08-19) was NOT being used for the notch/lead coefficient computation — those still used `g_measured_ctrl_interval_ms`, an EMA-smoothed *average* rate, despite this loop's documented, substantial per-call jitter (44-71% CV, see "Control-step timing regularity" above). A fixed-coefficient discrete filter is a uniform-sampling construct; feeding it coefficients for the average rate while applying the recursion at whatever the actual (often quite different) interval was seemed like a plausible, previously-unidentified source of filter mismatch.

**Fix attempted**: notch/lead coefficients recomputed from the real per-call `dt_s` (clamped to [0.5ms, 50ms], falling back to the EMA outside that range) instead of the smoothed average. Built clean, flashed, verified alive.

**Retested on hardware — measurably WORSE, not better, disproving the hypothesis.** Reran the notch Q-sweep (`scratch_notch_q_sweep.py`, same Q∈{8,12,20}×Ki∈{200,400,600,800} matrix as the entry above): every single combination that had been merely RINGING/GROWING before now DIVERGED or GROWING, including Q=8/Ki=200 (previously 91.4% overshoot, now DIVERGED at ~785px max deviation) and Q=20 at every Ki (previously already the worst case). Max deviation clustered tightly around 775-800px across nearly every trial regardless of gain — consistent with hitting a hard limit rather than scaling with gain the way a real instability normally does. The lead-compensator retest (`fz=15.4/fp=26`, same config as the entry above) was only marginally different (522.5% vs. 598.1% overshoot) — still violently unstable, no meaningful change.

**Best explanation**: recomputing coefficients from the raw, jittery per-call interval turns a fixed (if average-mismatched) LTI filter into a genuinely time-varying one — its pole/zero locations now fluctuate call to call in step with real timing jitter. A time-varying filter doesn't inherit the stability guarantees of any single frozen LTI snapshot; parameter fluctuation itself can inject energy into an already-marginal loop. Being *consistently* wrong (fixed average-rate coefficients) is evidently safer than being *correctly but unpredictably* wrong every sample. **This closes the dt-handling question for the notch/lead filters**: the fix was reverted (`git checkout --` on `main.c`, confirmed byte-identical to the last committed build via `.text`=31560 matching exactly), rebuilt, reflashed, and confirmed alive/idle. Nothing about this thread is committed as new state — the repo is back to exactly what was already on `origin/master`.

**Separately, a real, independently-verified bug found by reading `PIDController.hpp` directly** (prompted by the user asking "are we sure this one behaves completely correctly?" rather than accepting an inherited assumption). The derivative term's own comment claims "Derivative Term on Measurement (Prevents derivative kick)," but the code computed it as `(error - prev_error_) / dt` — derivative of *error*, not of *measurement*. Algebraically, `(error-prev_error)/dt = d(setpoint)/dt - d(measurement)/dt`, which only equals the pure measurement derivative when setpoint is unchanged between calls. The instant setpoint moves — a step in `set_target_x`, or every single tick while the on-board sine generator runs — this formula silently reintroduces exactly the setpoint-derivative spike ("derivative kick") the technique exists to eliminate. **Every D-term test this project has ever run used a step or the sine generator**, i.e. every single one was contaminated by this confound, layered on top of whatever real resonance-amplification effect D also has.

**Fixed** (`PIDController.hpp`, third explicit user-approved deviation from Phil's original, same pattern as the dt-awareness and float-conversion fixes before it): added a `prev_measurement_` state member, differenced directly (`raw_d_meas = -(measurement - prev_measurement_) / dt`) instead of derived from `prev_error_`. Identical output to the old formula whenever setpoint is held constant (the only case they were ever mathematically equivalent); genuinely kick-free otherwise. Nothing else in the class touched. Built clean (`.bss` grew by exactly 8 bytes = 2×4-byte float, matching the two PID instances `g_pid`/`g_pid2` gaining one new member each — confirms the change compiled/linked as expected), flashed, confirmed alive/idle.

**D-term retested with the fixed formula, Kp=1.75/Ki=200/throttled 200Hz (this session's confirmed-clean baseline), notch/lead off:**

| Kd | fc | overshoot | settling |
|---|---|---|---|
| 0 (fc left at firmware default 20Hz, no `--fc-milli` sent) | 20Hz | 4.7% | 185ms |
| 0 | 10Hz | 10.7% | 613ms |
| 0 | 10Hz (repeat) | 10.3% | 596ms |
| 0.001 | 10Hz | 13.1% | 663ms |
| 0.005 | 10Hz | 6.7% | 471ms |
| 0.02 | 10Hz | 92.1% | never settled |

**The catastrophic failure mode is gone.** With the old buggy formula, Kd=0.001/fc=10Hz gave 1391.5% overshoot (see "D-term retried with a resonance-informed cutoff" above) — total instability at the smallest testable gain. With the fix, the same nominal config gives 13.1% overshoot: a normal, gradual degradation, not a cliff. This is consistent with a real share of every earlier "D-term always fails" conclusion in this file being the derivative-kick artifact, not purely the resonance — though D still doesn't beat pure P+I anywhere tested, and Kd=0.02 shows there's still a real (softer) ceiling.

**Real, unresolved anomaly flagged rather than smoothed over**: sending `set_fc 10000` with `Kd=0` should be a mathematical no-op (`d_term = kd_ * anything = 0` regardless of `fc`'s value), but it reproducibly (twice, tightly repeatable: 10.7%/613ms and 10.3%/596ms) changed the baseline from the no-`--fc-milli`-sent run's 4.7%/185ms. Not explained — plausibly a side effect of `pid_wrapper_set_fc()`'s reconstruction interacting with the subsequent `set_ctrl_rate` call's own reconstruction in a way not yet traced through, or real session-to-session drift that happened to correlate with introducing the flag. **The exact Kd/fc numbers above should be treated as directionally correct (D no longer catastrophic, still doesn't help) but not fully trustworthy in absolute terms** until this is root-caused.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0 dac_x=95(bumpless-transfer leftover on dac_x, harmless with amp off) dac_y=95`), confirmed via `get_status` after every trial. `PIDController.hpp`'s derivative-on-measurement fix is the one real, kept change from this entry — committed separately from the (reverted, not kept) dt-aware notch/lead attempt. Raw D-term retest data: `results/fta_dfix_*.npz`/`.png` (not yet committed as of this entry). **Not yet done**: root-causing the `set_fc` anomaly; a proper Kd/fc 2D sweep now that the catastrophic-failure confound is removed (only 6 points tried, all at fc=10Hz except the two Kd=0 references); whether D ever becomes net-beneficial at some (Kd, fc) combination not yet tried, now that the kick confound is gone — genuinely open again, not closed the way the pre-fix testing implied.

### First-ever open-loop Bode sweep on the second axis (dac_x -> cy) — real, clean 18-point data, and its resonance peak (~47-50Hz) closely corroborates the earlier ring-down remeasurement on the same axis (2026-09-01, same day)

Direct continuation of "what to try now" — axis 2 (dac_x -> cy) has never had a proper frequency-response characterization, only a single-step smoke test and one ring-down trial (50.1Hz, see "Ring-down test generalized to both axes" above). The existing open-loop Bode tooling (`start_open_sine`/`fta_open_loop_bode_test.py`, built 2026-08-19) was hardcoded to drive `dac_y` only, so this needed real firmware extension, not just a new host-side flag.

**Firmware changes** (`main.c`): `start_open_sine` now takes an optional 4th argument, `AXIS` (0=x, 1=y, matching `fta_axis_t`), defaulting to 1 (y) so every existing 3-argument caller is unaffected — `g_open_sine_axis` global, consumed by `update_open_sine_dac()`'s `apply_dac()` call. `get_status` gained `open_sine_axis=` for ground-truth visibility (same pattern as every other live-tunable feature this project has added). **A real gap found while wiring this up**: the per-packet telemetry relay line only ever reported `dac_y=`, never `dac_x=` — fine for the already-characterized `dac_y->cx` pathway, but it meant a `dac_x`-driven sweep would have no per-sample ground-truth commanded value to fit against (the whole "fit both traces, diff cancels" trick this test method relies on needs a REPORTED command on the same timebase, not an assumed one). Added `dac_x=%ld` (from `g_last_dac_x`) to the telemetry line, `line[]` grown 165->190 for headroom. Build clean, `.text` grew ~31596->31816 across the two build passes (axis-selectable sine + the telemetry field), flashed, confirmed alive/idle both times via `get_status`.

**`fta_open_loop_bode_test.py` extended with `--axis x|y`** (default y, unchanged behavior): `TELEMETRY_RE` updated for the new `dac_x=` field; `_reader_thread` picks measured=x/commanded=dac_y for axis=y or measured=y/commanded=dac_x for axis=x; `run_one_frequency` sends `set_x`/`set_y` and the new 4th `start_open_sine` argument accordingly, and `verify_open_sine_state` now also confirms `open_sine_axis` matches before trusting a start was real; `emergency_cleanup` resets both `set_x 95` and `set_y 95` now (belt-and-suspenders, matches the existing pattern elsewhere in this project); plot titles/labels and the default `--out-prefix` (auto-suffixed `_xaxis` unless the user overrides it) are axis-aware so an x-axis sweep can't be visually or file-name-confused with a y-axis one, or silently overwrite one.

**Validated with a single point first (10Hz, axis=x)** before committing to a full sweep: gain=0.0792 px/count, a real, distinct value from axis y's ~0.089-0.093 at the same frequency — confirms the new measurement path is actually exercising the second, physically different pathway, not accidentally re-measuring the first. The raw wrapped lag read as a "negative" -39.6ms, which looked alarming in isolation but is the same known single-tone wraparound ambiguity this project has flagged repeatedly (a passive system can't truly lead its command; a wrapped negative lag near a period boundary just needs a full sweep to unwrap correctly) -- not investigated as a bug, since it resolved cleanly once the full sweep was run.

**Full 18-point sweep, 1-55Hz, amplitude=150 counts, base_dac=2048** (same grid and amplitude as the fresh axis-y sweep earlier this session), all 18 points succeeded:

| freq (Hz) | gain (px/count) | lag (deg, unwrapped) |
|---|---|---|
| 1 | 0.0814 | -173.5 |
| 2 | 0.0799 | -170.0 |
| 3 | 0.0732 | -163.8 |
| 5 | 0.0773 | -159.1 |
| 7 | 0.0761 | -152.9 |
| 10 | 0.0754 | -145.2 |
| 13 | 0.0826 | -130.0 |
| 15 | 0.0847 | -122.8 |
| 18 | 0.0856 | -110.2 |
| 20 | 0.0906 | -103.6 |
| 25 | 0.1016 | -84.3 |
| 30 | 0.1209 | -64.1 |
| 35 | 0.1460 | -39.3 |
| 40 | 0.1967 | -17.0 |
| 44 | 0.3156 | 3.2 |
| **47** | **0.6342** | 41.3 |
| **50** | **0.6384** | 132.6 |
| 55 | 0.2052 | -173.7 (near-resonance aliasing, see below) |

**Real, cross-validated resonance found: peak gain 0.63-0.64 px/count at 47-50Hz** (grid-limited to that 3Hz window, true peak could sit anywhere inside it) — an ~8x amplification over the ~0.07-0.08 px/count low-frequency baseline. **This closely matches the ring-down remeasurement already on record for this exact pathway (50.1Hz, single trial, "Revert-and-retest" entry above)** — two independently-different methods (forced swept-sine here, free decay there) landing on essentially the same number is the same kind of strong cross-validation that established the primary axis's resonance figure. **Notably gentler than the primary axis's resonance**: axis 1 peaked at ~11x its own DC gain (0.997 vs ~0.09 px/count); axis 2 peaks at ~8x (0.64 vs ~0.08) -- a real, lower-Q, less severe peak, not just a smaller number by coincidence of units. Both axes' resonances land in the same rough 40-50Hz neighborhood -- worth flagging for the FEA/flexure-redesign work already in progress (see the FEA-vs-bench slides): this isn't a problem unique to one axis, and it's worth checking whether the two axes share a common physical mode (e.g. a flexure cross-coupling resonance) or are genuinely independent before finalizing a stiffening design that might only address one.

**One thing that looks odd but isn't a new bug, flagged so it isn't mistaken for one later**: unlike axis y's fresh sweep (which started near 6° lag at 1Hz), axis x's unwrapped phase starts near -170° even at 1Hz. Expected, not investigated further: axis x's static gain is already established as NEGATIVE (dac_x's effect on cy is -0.104 px/count, locked-optics calibration, 2026-08-12) -- a negative gain is mathematically indistinguishable from a ~180° phase offset in a single-tone fit with no external sign reference, the exact phenomenon this project already root-caused and fixed once before (see "RESOLVED (2026-08-06): the 'large phase lag' below was a sign-unaware analysis bug"). `fit_bode_point()` doesn't take an explicit sign reference the way that earlier fix's `fit_sine()` eventually did -- this doesn't need fixing for the gain/resonance conclusions above (magnitude is sign-independent) but would need addressing if this axis's absolute phase/lag numbers are ever load-bearing for a future compensator design.

Combined summary figure: `results/fta_open_loop_bode_xaxis_summary.png`. Per-frequency 2x2 diagnostic plots (full-duration + zoomed, both measured cy and commanded dac_x, fit overlaid): `results/fta_open_loop_bode_xaxis_<freq>Hz_<timestamp>.png`/`.npz`.

**State left**: hardware safely idle (`mode=open_loop amp=0 estop=0 dac_x=95 dac_y=95`), confirmed via `get_status` after the sweep. Firmware (`start_open_sine` AXIS arg, `dac_x=` telemetry field, `open_sine_axis=` status field) and `fta_open_loop_bode_test.py`'s `--axis` support are both new, tested, and ready to commit. **Not yet done**: a ring-down repeat on axis x (still only one trial, per the still-open item from "Revert-and-retest" above); resolving the true peak location within the unsampled 47-50Hz gap (a finer-grid sub-sweep, e.g. 46/47/48/49/50Hz, would resolve this cheaply); whether axis x's resonance interacts with axis y's the way a shared mechanical mode would predict (would need a cross-axis excitation test -- drive one axis, watch the other's resonance response -- not attempted); using this real axis-x data for any compensator/notch design the way axis y's data has already been used, if axis 2 tuning is ever pursued more seriously than its current "holds steady, prevents drift" role.

### Sine-tracking test built, 3 frequencies run — clean shape tracking, phase-lag magnitude unresolved (2026-08-04)

Frequency-domain complement to the step-response tests, motivated by the
project's actual goal (top of this file): a step tells you settling time
for a one-off jump, not whether the actuator can continuously reject a
10-20Hz disturbance. Built `fta_sine_response_test_vcp.py` — same
laptop-only VCP architecture as the step test (paced `set_x`/`set_y`
commands, position sourced from the Nucleo's telemetry-relay print via a
background reader thread) — commands
`center + amplitude*sin(2*pi*freq*t)` and fits the measured trace to
`A*sin(wt) + B*cos(wt) + C` (linear least-squares, since the test
frequency is known exactly) to extract amplitude and phase lag.

**Fixed a latent timing bug while building this** (present in
`fta_step_response_test_vcp.py` too, but harmless there): the reader
thread didn't start until after the pre-test 0.5s settle sleep, so
telemetry arriving during that sleep would burst through with
near-identical timestamps once the thread finally started reading — fine
for the step test (only ever took a *mean* over its pre-step baseline,
insensitive to relative timing within that window) but would corrupt a
sine-phase fit, which depends on precise timing throughout. Added
`ser.reset_input_buffer()` right before starting the reader thread.
**Verified this wasn't the actual cause of the finding below** — retested
0.5Hz before/after the fix, result didn't meaningfully change (169° → also
~169° reported phase).

**Ran 0.5Hz, 1Hz, 2Hz on axis x (±200 DAC counts around 2000).** Plotted
(`docs/session_results_2026-08-04.pptx`, sine-tracking slides) — visually,
the measured pixel trace is a clean, low-noise sinusoid locked to the
exact commanded frequency at every rate tested, for the full duration of
each run. **Qualitatively this is a real, positive result**: the actuator
tracks a continuous sine faithfully in shape, not distorted, clipped, or
backlash-limited. (The 1Hz run has an obvious startup transient in its
first ~0.3s, excluded from the fit by dropping the first full commanded
cycle.)

**But the fitted phase lag is large and hard to trust as-is.** Even at
0.5Hz — where the step-response settling times (125-469ms) would predict
a small lag if the actuator behaved like a simple low-order system — the
fit shows ~940ms of lag, uncomfortably close to half that 2000ms period
(a single-frequency phase fit can't distinguish a lag from
`lag - n*period`, a fundamental ambiguity, and physically a passive
actuator can't lead its own command, so a fitted "phase lead" is really an
aliased large lag, not evidence of anything faster than expected).
Fitted lag values: 0.5Hz → -940ms, 1Hz → -447ms, 2Hz → -204ms — roughly
constant *phase* (~190-210°, computed as `period + lag_ms` mod period)
rather than constant *time delay*, which doesn't cleanly point at either a
simple fixed transport delay or a simple actuator time-constant
explanation; not resolved.

**Most likely explanation, not yet confirmed**: this test's measurement
path — I2C → Nucleo prints a line → USB VCP → Python `readline()` in a
background thread — is nothing like how a real closed-loop controller
would read position. A future PID running on the Nucleo reads
`g_latest_beam` directly at ISR level, with none of this print/USB/Python
overhead. The fitted lag numbers here almost certainly conflate real
actuator+optics dynamics with this test method's own added latency, which
a Nucleo-side PID would never actually pay — meaning these specific
numbers should NOT be used for gain tuning without first separating the
two. **Two ways to disambiguate, neither done yet**: test at an even lower
frequency (e.g. 0.1Hz, where true actuator lag should become negligible
relative to the period — a persistent large offset there would implicate
fixed pipeline latency, not frequency-dependent actuator dynamics); or run
a camera-direct sine test on the Pi (analogous to why
`fta_step_response_test.py` was kept as the camera-direct ground truth)
that removes the VCP relay from the measurement path entirely.

**Not yet done**: resolving the above (see "RESOLVED (2026-08-06)" just
above — sign-unaware analysis, not a real actuator lag problem); pushing
frequency toward the actual 10-20Hz disturbance band (still not attempted
— now that the analysis is trustworthy, this is the real next step);
testing axis y and the cross-axis coupling further (already logged
per-run in the results but not separately analyzed).

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
