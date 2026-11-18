# Stabilization bench prototype

This is the first self-level control implementation for this quad. It is not tuned or flight-qualified. It cannot hold horizontal position or altitude, and level flight can still drift. There is no pilot receiver or Wi-Fi control in this version. Keep all props removed for its tests. Disconnect the ESC LiPo for upload, power the ESP32 by USB, and connect motor power only after OFF pulses are running. Keep every ESC's 10k PPM-to-B- resistor and the common ground.

## Wiring and orientation

Viewed from above, top is front:

| Position | GPIO | Rotor direction |
|---|---|---|
| Front-left | 7 | Anticlockwise |
| Front-right | 5 | Clockwise |
| Rear-right | 3 | Anticlockwise |
| Rear-left | 2 | Clockwise |

Each direction needs its matching prop when flight commissioning eventually reaches that stage. This sketch's array order is FL, FR, RR, RL, not the earlier m1..m4 order. The MPU uses 3V3, common GND, SDA10, SCL1 and AD0=GND. **Sensor +Y must point toward the drone front.** Secure the sensor and put the actual frame level and still during calibration. Startup determines which sensor face is up, applies a proper rotation into front/right/down axes, and captures a level trim. It never flips only one gyro axis. The original report of X left/Y front and the prior positive-Z readings need this orientation check; the READY log reports which X direction it inferred. If the mounting is not approximately parallel to the frame (within about 26 degrees), calibration refuses it.

## First run: preview, no motor motion

Select ESP32C3 Dev Module, USB CDC On Boot Enabled, Espressif core 3.x (project installation 3.3.10). Upload the complete `StabilizeBench` folder and open Serial Monitor at 115200 baud with New Line. Keep the ESC LiPo disconnected for this first check.

Quiet v2 starts with automatic status streaming OFF. `status` prints one snapshot, including the MPU fault or calibration restart cause. `arm` reports its exact blocking reason. `logs on` enables repeated angle/output telemetry for tilt checks; `logs off` stops it. Mode/stop commands preserve the original latched fault reason. Quiet mode does not relax calibration checks or motor protections.

Quiet v3 fixes a pitch mixer sign error present in v1/v2 and adds `check`. Reupload v3 before any further motor testing. The corrected FRD mixer increases front thrust for a positive nose-up torque request, so nose-up tilt produces a compensating rear-thrust increase. Roll/pitch corrections are checked against the independent physical relation moment = position cross force, in addition to bounds tests. Reference: [PX4 rotor force/moment calculation](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRotors.cpp).

For the simplest response check, keep ESC LiPo disconnected, wait for READY with frame level, then gently tilt and hold the frame and send `check`. It prints one explanation: left side raised should show RIGHT pair higher; front raised should show REAR pair higher. `check` stops/disarms first, switches to preview/quiet, and computes hypothetical 1400 us motor commands using the same controller/mixer as the motor mode. Actual outputs remain 1000 us. It requires no arm/throttle commands and has no short timing window. It checks software demands, not actual motor speed or thrust. Hold the tilt still so the rate-damping term has settled.

Keep the frame level and stationary for about three seconds until READY. Motion restarts calibration. Check the displayed angles before arming: lower the right side and R must increase; raise the nose and P must increase. Return level. Corrected rates should be near zero while stationary. If either angle moves the wrong way, do not proceed to motor output; recheck the physical mounting and mapping.

Enter each command separately:

```text
preview
arm
1400
```

The actual motor commands stay at 1000 us in preview. `calculated` shows proposed pulses. `sent` shows the requested pulses last written to LEDC; it does not prove the electrical output. With small tilts, check the correction direction:

| Frame motion | Expected calculated response |
|---|---|
| Right side down | Right motors increase, left motors decrease |
| Nose down | Front motors increase, rear motors decrease |
| Clockwise yaw while viewed from above | CCW rotor pair decreases, CW rotor pair increases |

The exact values also depend on angular velocity and settling. They are not all supposed to stay at 1400: stabilization changes individual motor commands around the requested base. The base slews at 200 us/s. Send `keep` within five seconds to renew the command lease. Every test ends 15 seconds after arm regardless of keep. `status` does not renew the lease. These stops are intentional, not evidence of battery sag. `stop` immediately requests 1000 us on every output and disarms.

## Automatic test recording (Quiet v4)

Every successful arm starts a new recording in RAM, sampled at 20 Hz. After any stop, timeout or fault, the motors are stopped first and a short `TEST REPORT` prints automatically. It includes the recorded roll/pitch ranges, snapshots at their extremes, and the final snapshot from BEFORE the outputs were cleared. No command needs to be sent during the wobble. Gains, throttle limits, the five-second lease and the absolute 15-second limit are unchanged.

After stopping, `report` reprints the short report whenever convenient. `trace` prints the complete saved sequence, one line per sample, to show the order of swings. Both commands only replay while disarmed and never renew the command lease. Only the latest run is stored; the next successful arm or a reset replaces it. Printing waits for serial queue space and does not block or print a full trace while armed.

Report times are milliseconds since arm. `base` is actual/requested base; pulse order is FL7, FR5, RR3, RL2. `calc` is the controller demand and `sent` is the last request written to LEDC, not measured RPM or thrust. `g` is measured acceleration magnitude. A fault's final sample may contain the last valid sensor data; it is not proof of fresh data after a sensor failure. Recording is nominally every 50 ms, plus an unconditional final snapshot, so a brief event between samples can be missed. This instrumentation does not qualify the prototype for flight.

## Restart diagnostic (Quiet v5)

Startup and `status` now print `Reset reason: NAME (numeric code); uptime=... ms`, using the installed ESP32 core's `esp_reset_reason()` API. This identifies the reset cause reported for the CURRENT boot; the initial reset caused by uploading is not evidence of a motor-power fault. If a new startup banner appears during or after a test, copy the reset-reason line from that new startup. Brownout, crash/watchdog, and USB resets need different investigations. A USB reset alone does not prove a battery problem, and a power LED can remain lit across a processor reset. Old Serial Monitor text is not a new boot. All control settings and safeguards remain unchanged.

## Persistent tilt correction (Quiet v6 candidate)

V1-v5 used zero integral gain. Those controllers can require a persistent angle error to oppose a steady torque imbalance. V6 adds a small roll/pitch rate integral gain of 0.12, with each integral bounded to +/-25 us. It is a candidate for bench tuning, not a validated cure for the reported tilt. Cable forces, prop thrust differences, frame balance, fixture friction and vibration-corrupted attitude can produce similar observations. An MPU6050 estimates attitude and angular rate; this firmware cannot detect or arrest horizontal drift while level, or hold altitude.

The integrals reset at stop/disarm and below a 1250 us base. They freeze when mixer authority is exhausted, and an axis's own +/-80 us correction limit prevents integration further into that limit. P/D gains and the outer angle gain are unchanged. Yaw I remains zero. The `I=` fields in saved reports show the learned corrections. `balance off` while disarmed selects the previous zero-I baseline; `balance on` restores the candidate. Each resets the integrals. Default after reboot is ON. Do not change this setting during a run.

The same five-second command lease and absolute 15-second maximum remain. This is a bench sketch, and those automatic motor stops make it unsuitable for free-flight piloting. USB keyboard commands do not provide a continuous airborne pilot link. The change has math/model checks, not a physical flight validation. The simple disturbance model used in checks verifies algorithm behavior only; it does not establish gains for this quad or reproduce its ESC/motor response.

## Bare-motor test after the preview directions pass

Remove all props. Power the ESP32 first; let startup OFF pulses run, then power the ESCs. All motors must remain stopped. With the frame level, enter:

```text
bench
arm
1400
```

Now actual motor pulses vary around 1400. Briefly tilt the prop-free frame to verify the motor response follows the preview. Send `stop`. A separate test can use `bench`, `arm`, `1700`, then `stop`, with the same time limits. All physical commands are capped at 1800 us. These are spin/response tests, not a flight procedure. Bench mode does not carry over across reset. `preview`, `bench` and `cal` all stop/disarm first; `cal` also returns to preview.

## Control and limits

The MPU supplies fresh data at 200 Hz, with gyro +/-500 degrees/s, accel +/-4g and a roughly 42 Hz gyro low-pass filter. Calibration estimates gyro bias from 600 stationary samples; it does not perform six-position accelerometer calibration. A quaternion complementary filter estimates tilt. Level trim defines the startup level frame; calibrating on a sloping surface will teach the wrong level. It cannot correct a misleading accelerometer bias under all orientations.

An outer angle P controller requests roll/pitch rate, inner rate controllers produce roll/pitch corrections, and yaw rate damping requests zero yaw rate. P, I and D values are **untuned starting values**, not a stable-flight claim; roll/pitch I is bounded as described above and yaw I is zero. Mixer scaling preserves correction ratios within 1100..1800 us and avoids silently clipping individual axes. Angle/rate/PID math has startup self-tests before any PWM attachment. Real ESC pulses remain 50 Hz; a 200 Hz sensor/control computation does not make the ESC respond at 200 Hz. This latency and the motors' observed starting/hysteresis behavior must be evaluated before flight tuning.

Stops include operator stop, invalid command, five-second command timeout, 15-second maximum run and tilt above 35 degrees. Sensor read failures, >25 ms sample/control gaps, sensor saturation, invalid attitude and LEDC failures latch a fault until reset. OFF is the previously tested 1000 us; if an OFF write fails, the code also attempts detach and drives that pin low. A CPU hang can leave hardware PWM running: software timeouts are not an independent flight failsafe. Reset/signal-loss tests and reliable loaded power remain necessary hardware checks.

Actual flight still requires tuned gains, verified power and ESC response, calibrated motor thrust/start behavior, suitable pilot input and a dependable stop path. No battery voltage measurement or low-voltage landing is implemented. USB bench control is not an airborne control link.

References: [MPU register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf), [Arduino LEDC API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html), [PX4 controller tuning principles](https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter).
