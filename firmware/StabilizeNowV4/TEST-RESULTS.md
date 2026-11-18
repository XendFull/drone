# V4 batch observations, 2026-10-20

Source: LAST-BATCH-LOG.txt, supplied by the user after reporting the drone
battery ran out. No new motor test was requested or performed to analyze it.
No controller values were changed after this batch.

## Results

All three runs stopped at the intended 15-second limit. No USB cancellation
appears in this batch. This verifies that the revised guard allowed this
particular batch to finish; it does not establish physical USB reliability.

| Run | Roll mean / motion RMS | Pitch mean / motion RMS | Roll / pitch gyro RMS | Final roll / pitch |
| --- | --- | --- | --- | --- |
| 1 | -0.95 / 0.68 deg | -2.42 / 0.65 deg | 11.51 / 3.24 deg/s | -1.86 / -1.92 deg |
| 2 | -1.61 / 0.48 deg | -1.90 / 0.15 deg | 7.98 / 2.72 deg/s | -0.47 / -1.36 deg |
| 3 | -0.74 / 0.62 deg | +0.66 / 1.45 deg | 2.06 / 1.91 deg/s | -1.07 / +0.03 deg |

These are fitted summaries of the settled windows, not independent physical
angle measurements. The 20 Hz recorder cannot resolve fast vibration.

Runs 1 and 2 show relatively small estimated angle variation with residual
nose-down lean. Their pitch estimates were moving toward level in run 1
(+0.18 deg/s) and approximately steady in run 2 (-0.02 deg/s). Integrals were
not near their cap and no sampled output limiting was reported. This does
not prove thrust, headroom or instantaneous absence of limiting.

In run 3, final rates are nearly zero and the first post-stop pose hardly
changes. Combined with the user's depleted-battery report, reduced or lost
thrust is a plausible explanation. Voltage, current and motor RPM were not
recorded, so neither the precise depletion time nor the thrust state can be
established. Exclude run 3 from choosing gains. Treat the earlier runs with
caution too: the battery's loaded voltage is unknown throughout the batch.

Post-stop pitch change in runs 1/2 agrees reasonably between fused and
gyro-only estimates (+1.68/+1.70 deg and +1.18/+1.20 deg after 2 seconds).
This supports actual pitch settling after thrust stopped, but the two
estimates share a gyro and are not independent proof of physical movement.
The stand remains a mechanical influence on the test.

## Startup fault

The log starts with MPU6050 identification failures. After the power-on
restart it reports READY with 600 calibrated samples, then completes all
three runs without a logged MPU fault. The startup failure's cause is not
identified; do not attribute it to the flight battery or mark it repaired
from this log alone. Existing fault handling keeps motors disarmed on an
identification failure. No sensor fault or arm gate was bypassed.

## Decision

Retain roll P=0.50, pitch P=0.70, roll/pitch I=0.12 and D=0.002; angle gain=3.
There is insufficient evidence for another gain increase, decrease, trim or
learned motor offset. Stand bias and changing thrust cannot be isolated
using these summaries. Do not carry this stand's learned integral between
runs or calibrate away a powered lean.

The requested same-file sketch remains in place. Current batch behavior is
15-second runs, up to 3 by explicit command, at least 5 seconds OFF between
runs, immediate STOP, 500 ms continuous USB-loss cancellation and a current
connected status required before an automatic restart. This does not add
altitude/position hold or demonstrate stable free flight. No further motor
tests are needed to complete this log analysis.
