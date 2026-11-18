# StabilizeNow V4: automated tuning comparisons and bounded batches

## Automated comparison (new, explicit command only)

The same sketch now supports `autotune roll 1400` and `autotune pitch 1400`.
The throttle argument is optional (default 1400); only 1250..1400 is accepted.
This is an experimental A/B/A comparison of one bounded candidate, not a
general PID solver. It never launches automatically at boot or from a phone
ARM. It does not tune yaw, motor curves, gyro filters or position hold.

Each command schedules three 15-second runs with the existing 5-second OFF
gaps: baseline, candidate, baseline again. Roll compares P 0.50 against 0.40;
pitch compares I 0.12 against 0.16. The other axis and coefficients stay at
their default values. After every stop/cancellation, defaults are restored.
No recommended gains are automatically applied or saved to flash.

**This deliberately commands motion. Use only a securely retained, freely
tilting test fixture with adequate clearance, never free flight or a loose
frame grip.** A rigid clamp or props-off motor run cannot provide a valid
tuning response. Remove props and disconnect ESC power for uploading and
initial STOP checks. Do not run a loaded comparison on a depleted battery.
Software cannot certify that a fixture or power system is suitable.

The selected axis requests 0 degrees during the first 3 seconds, then +3
for 2 seconds, 0 for 2, -3 for 2, 0 for 2, +3 for 2, and 0 for the final 2.
Metrics use fresh IMU/controller samples rather than the 20 Hz report rows.
Known commands and signed gyro-integrated movement check that the frame
responded in the expected direction. The tighter comparison guard stops
the batch above 12 degrees on either axis or a 120 deg/s rate magnitude.
Existing STOP, USB-loss, stale-sensor, fault and output limits still apply.

A run is rejected for insufficient data, bad timing, inadequate signed
response, excessive gyro RMS or significant limiting. Any rejected run
cancels subsequent runs. Both references must also agree within the stated
heuristics (25% in response span and score); otherwise no recommendation is
made. Score is tracking-error RMS plus 0.05 times gyro-rate RMS. The candidate
must improve score at least 15% versus BOTH references before being reported
as a BENCH CANDIDATE. These are conservative experimental heuristics, not a
statistical confidence interval or proof of aircraft stability.

Wait for all three runs before interpreting the result. USB prints each
comparison summary; `autotune report` reprints it while disarmed. This summary
is separate from `report` / the phone's standard last-run report. All three
small comparison results remain in RAM until a new comparison or reboot.
The ordinary `test` commands still use fixed defaults and zero tilt targets.

The references help detect changing conditions, but **there is no battery
voltage or RPM measurement**. Partially weakened thrust or fixture resonance
can still distort a comparison. A quiet, nonresponsive run cannot win, but
these checks cannot prove a healthy battery or select flight-ready gains.
No upload, motor run or hardware validation of this feature was performed.

## Previously supplied fixed-gain tests

Latest supplied batch: all three 15-second runs completed without USB
cancellation. The user reported the flight battery ran out. Gains are held
unchanged; the last run is unsuitable for selecting gains, and voltage was
not logged in any run. See TEST-RESULTS.md and LAST-BATCH-LOG.txt. No further
motor test was requested to analyze this batch. Flight stability is unverified.

Latest in-place revision: automatic USB bench runs last 15 seconds, and an
explicit count can request up to 3 runs. Default gains remain at the values
from the user's V4 run; only an explicit automated comparison temporarily
changes one coefficient.
The run had roll mean 0.02 deg / motionRMS 1.00 deg / gyroRMS 9.18 deg/s;
pitch mean -3.74 deg, with fitted drift +0.42 deg/s toward level. A longer
bounded observation can show whether pitch keeps settling; it cannot prove
the integral, stand resistance or any single cause explains the lean.
The same `test 1400` command is used. The filename and folder are unchanged.

## Multiple tests with one command

Send `test 1400 3` at 115200 baud with New Line for three 15-second runs at
the same throttle. Counts 1 through 3 are accepted; `test 1400` still means
one run. Batch mode is a restrained bench feature, not a flight mode.

Each run stops, records the 2-second post-stop comparison, and prints its
numbered report. Motors remain OFF for at least 5 seconds before another
run can start. Keep clear: the next run starts automatically. A fresh
health/level/still check and a connected USB monitor are required for each
restart. An unfinished report can delay the restart up to 3 additional
seconds; then the batch cancels rather than waiting indefinitely.

`stop`, `1000`, phone STOP, USB loss, calibration, unknown commands, excessive
tilt or a fault cancels all remaining runs. Reconnecting USB never resumes
a canceled batch. Phone ARM is ignored while a USB batch is active. The
phone's live command lease is unchanged.

USB guard fix: the previous batch version canceled on one false `Serial`
status. The installed C3 core's USB helper can transiently report false.
This revision samples status once per loop and requires 500 ms of continuous
false status before stopping/canceling. A true reading resets that timer.
Every automatic restart additionally requires a current true USB reading;
the grace period cannot authorize restarting while the current reading is
false. STOP remains immediate. USB status is not an operator heartbeat and
cannot prove a person is watching the serial monitor. The phone lease and
15-second/3-run bounds remain unchanged.

All reports print into the USB monitor history. `report` and the phone report
link show the latest run only; copy the whole USB history to share a batch.

V4 changes one controller value from V3: roll rate P is 0.50 instead of 0.70.
This reduces the roll proportional contribution by about 29%; roll I=0.12,
D=0.002, pitch P=0.70/I=0.12/D=0.002, yaw P=0.50 and outer angle gain=3
remain unchanged. The gyro fusion, filters, throttle limits, protocol and
stop protections are unchanged. Reduced P also reduces proportional rate
damping; this is a comparison trial, not a guaranteed cure or flight tune.

The latest V3 run after loosening the stand showed roll motionRMS 2.35 deg
and gyroRMS 58.73 deg/s, versus 0.70 deg and 1.82 deg/s in the earlier run.
Stand mechanics changed between runs. Neither controller instability nor
stand/prop vibration is uniquely identified by these 20 Hz summaries.
Keep the physical setup consistent when comparing V4 with that V3 run.
Secure frame-gripping clips; a loose frame attachment invalidates a test.

## Open/build/upload

Open StabilizeNowV4.ino with all headers in this folder. Arduino board:
ESP32C3 Dev Module, USB CDC On Boot Enabled, Arduino-ESP32 core 3.3.10.
No additional libraries. Remove propellers and disconnect ESC LiPo during
upload; the ESP32 can remain USB-powered. No upload is performed by preparing
these files. Firmware starts disarmed and sends 1000 us OFF.

## Wiring

Viewed from above, front of frame at top:

| Position | GPIO | Direction |
| --- | --- | --- |
| Front left | 7 | CCW |
| Front right | 5 | CW |
| Rear right | 3 | CCW |
| Rear left | 2 | CW |

MPU6050 SDA10, SCL1, VCC3.3V, common ground, AD0 ground.
Sensor +X RIGHT, +Y FRONT, +Z UP. Keep existing 10k signal pulldowns.
GPIO4 is unused. RC-input brushed ESCs use 50 Hz pulses, not direct motor PWM.
Cordless operation requires a suitable regulated ESP supply; do not connect
raw LiPo to 3V3. Wi-Fi does not supply power.

## Controls and comparison

Hold frame level and still at startup; wait for READY after 600 calibration
samples and the minimum 5-second startup hold. Movement restarts calibration.
Verify sensor signs with propellers removed: right side down increases roll;
nose up increases pitch. Verify motor position mapping and STOP before loaded
operation. Phone Wi-Fi QuadLevel, password CHANGE_ME_123; open http://192.168.4.1.
Explicitly enable outputs and ARM at zero throttle. Steering release requests
level, not cancellation of existing sideways velocity.

Phone sends a 100 ms heartbeat, with a 500 ms command lease. STOP, link loss,
backgrounding, excessive tilt or sensor/control fault stops and disarms.
A new explicit ARM is required. No fixed test timer applies in phone mode.
Throttle 1000..1700 us; motor cap 1800 us. Tilt stop is 35 degrees. Sensor or
control gaps above 25 ms latch a fault. A thrust cut is not a landing.

USB: 115200 baud, New Line. With props removed, `test 1400` starts a
15-second bounded test, automatically stops and prints a report. No rapid
keep typing is needed. `stop` or `1000` stops, `status` prints once disarmed,
`report` reprints the saved result, and `cal` stops and recalibrates. Unknown
USB commands stop. Old bench/arm/bare throttle commands are not supported.
Do not use the automatic bench test as a takeoff mode.

After STOP, leave frame untouched for 2 seconds. Gyro-only comparison
snapshots at approximately 0.5, 1 and 2 seconds append automatically over USB.
Refresh the phone's saved report after 2 seconds. A new ARM or cal cancels a
pending comparison. Missing fresh samples are marked unavailable.

Gyro-only is initialized from fused attitude at ARM and never controls
motors. Both estimates share the same gyro and can suffer vibration errors.
The comparison is diagnostic, not an independent attitude reference.

## Interpreting results

Compare settled roll motionRMS and gyroRMS under the same conditions, plus
visible motion and report angle ranges. Lower software command gain does
not prove lower physical oscillation. Do not compensate a stand's mechanical
resistance with arbitrary gain or trim changes.

Crossing Hz is a rough estimate: zero can mean irregular/insufficient
crossings, not no wobble. 20 Hz recording cannot resolve fast vibration.
A sampled limiter fraction of zero does not prove no instantaneous limiting.
Motor values are requested pulses, not measured thrust or RPM. Gyro witness
and post-stop rows help investigate attitude shifts, not tune automatically.

Flight stability remains unverified. This firmware has no altitude/position
hold, voltage sensing or automatic landing. Software stops require a running
CPU; hardware PWM may persist during a CPU hang. Pulldowns and ESC signal-loss
behavior are not an independent CPU-hang watchdog.

See VALIDATION.txt for completed checks, BUILD.txt for compile output and
V3-LOOSE-STAND.txt for the supplied comparison data. Earlier versions remain
unchanged. V4 has not been uploaded or physically tested by the tools.
