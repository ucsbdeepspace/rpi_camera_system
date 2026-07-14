# MODE_640_200_ROI results summary (2026-07-08)

Out-of-tree `ov9282` driver patch (`kernel_patch/ov9282/`) adds two experimental
sensor modes on top of the stock driver's three. This summarizes the two that
matter: the stock baseline and the one that actually delivered a speed win.

## Pure capture throughput (dual-camera concurrent)

| mode | rated/native max fps | best achieved fps | frame duration | floor |
|---|---|---|---|---|
| Stock 640x400 (binned, no crop) | 309.79 | 281.8-282.4 | 3400us | hangs at 3228us |
| MODE_1280_400_ROI (crop only, no binning) | 296.47 | 283.6 | 3373us | not pushed further -- no real gain over stock |
| **MODE_640_200_ROI (crop + binning combined)** | **588.93** | **526.8** | **1800us** | **hangs at 1750us** |

`MODE_1280_400_ROI` crops vertically but keeps full 1280px width unbinned, so
the extra per-row readout time eats the savings from reading fewer rows --
essentially no gain over stock. `MODE_640_200_ROI` applies the stock mode's
horizontal binning *and* a shrunk vertical window, which is what actually
produces a speed win: **~1.9x the stock throughput.**

Full sweep data: `camera_throughput_sweep_640x200.csv` (3400us down to
1750us, 13 points, stops at first hang).

## Closed-loop LED round-trip (both cameras confirming a toggle)

| mode | confirmed toggle rate | mean latency | max latency |
|---|---|---|---|
| Stock 640x400 @ 3400us | ~90-122 Hz | ~7 ms | 10-15 ms |
| **MODE_640_200_ROI @ 1800us** | **~207 Hz** (206.64 / 207.87 Hz, 2 runs, 0 timeouts) | **4.1-4.4 ms** | **7.6-8.3 ms** |

Inter-camera skew at the new setting: mean ~0 ms, max \|skew\| 2.7-4.1 ms
across the two runs.

Same ~1.9-2x win as pure throughput carries through to the closed loop.
Mean latency is now comfortably under Phil's 10ms loop-latency target for
the first time -- previous best (~7ms mean, 10-15ms tail) is now roughly
where this setting's *max* sits.

## Caveats / not yet done

- Closed-loop test only run twice at the new setting (both clean, 0
  timeouts) -- more repeats would build confidence.
- True floor is somewhere between 1750-1800us, not narrowed further.
- Frame content at 640x200 was visually sanity-checked (real image, no
  tearing/garbage) but not rigorously validated against a known test
  pattern -- the y_output_size halving in the new mode's register set is
  an educated guess, not a datasheet-confirmed value.
- Open question for Phil: discrete-step-and-confirm vs continuous
  latest-frame tracking determines whether the ~207Hz closed-loop number
  or a higher continuous-capture number is the actually relevant ceiling
  to design around.

See `CLAUDE.md` for full narrative detail, safety notes on how the 1750us
hang was recovered without a reboot, and script usage.
