# StabilizeNow v3: gyro comparison and automatic post-stop readings

Runs directly on the existing ESP32-C3 SuperMini. No second ESP32 or receiver is needed for this version: your phone connects to the drone's Wi-Fi. This is custom experimental firmware, separate from the ESP-FC evaluation.

It calibrates automatically, estimates attitude, varies all four motors to correct roll/pitch, damps yaw rate and records each run. **It does not automatically take off, maintain height or hold horizontal position.** Stable real flight has not been demonstrated with this build. Compilation and PC checks cannot tune an unidentified aircraft.

## Open and build

Open `StabilizeNowV3.ino` in Arduino IDE with all accompanying headers in this same folder. Select **ESP32C3 Dev Module**, **USB CDC On Boot: Enabled**, Arduino-ESP32 core **3.3.10**. No extra Arduino libraries are required; Wi-Fi and the HTTP server come with the installed core.

Before uploading: remove all propellers and physically disconnect the ESC LiPo. The sketch starts disarmed and sends the previously verified 1000 us OFF command. An upload is not included in the prepared files.

## Wiring (unchanged)

| Position, viewed from above | GPIO | Motor direction |
| --- | --- | --- |
| Front left | 7 | CCW |
| Front right | 5 | CW |
| Rear right | 3 | CCW |
| Rear left | 2 | CW |

MPU6050: **SDA10, SCL1, VCC3.3V, common GND, AD0 GND**. Sensor +Y points FRONT, +X RIGHT, +Z UP. Retain the existing ESC signal-to-ground 10k resistors and common ground. These are RC-pulse-input ESCs: outputs remain **50 Hz, 1000 us OFF**, not direct brushed-motor duty PWM. GPIO4 is unused.

Phone control removes the USB data connection requirement; it does not supply power. Cordless use still requires a suitable regulated ESP32 supply. Do not connect a raw LiPo to the 3V3 pin.

## First check with props removed

1. Power up with the actual frame level and still. The controller collects 600 samples (about 3 seconds) and enforces a minimum 5-second startup hold. Moving it restarts calibration.
2. On your phone, connect to **QuadLevel**, password **CHANGE_ME_123**. Stay connected even if the phone reports no internet. Open **http://192.168.4.1**.
3. Wait for **READY**. Level readings should be near zero. With props removed, right side down must increase roll; nose up must increase pitch. Return level.
4. Enable physical motor outputs, then tap **ARM at zero throttle**. Raise throttle slowly. The displayed motor order is FL7 FR5 RR3 RL2. Confirm restoring responses: right side down requests more right-pair thrust; nose up requests more rear-pair thrust. Actual motors must match the displayed positions.
5. Tap **STOP MOTORS**. Arm at zero throttle again for a brief bare-motor run and turn off the phone's Wi-Fi: output commands must go OFF within the 500 ms command lease. Reconnecting must never restart the motors; a new ARM is required.

These new link/timing checks must pass on the real board before considering flight testing. The 50 Hz ESC update speed, motor-start thresholds and loaded power remain hardware constraints. This build alone cannot establish sufficient lift or stable gains. First lifting tests need live pilot throttle and STOP; the automatic USB bench test below is not a flight mode. A link-loss stop or excessive-tilt stop cuts thrust rather than performing a landing.

## Phone control

The phone sends commands every 100 ms while armed. Throttle range is 1000..1700 us, with each motor capped at 1800 us as in the previous bench controller. Collective throttle rises at 200 us/s and falls at 600 us/s; zero throttle and STOP request OFF immediately. Steering is limited to +/-8 degrees, yaw to +/-30 deg/s. Releasing a steering button requests level / zero yaw rate. Release does not stop horizontal motion already acquired.

There is **no fixed 5- or 15-second cutoff in live phone mode**: the live command lease replaces the old timed typing requirement. Losing commands for 500 ms, closing/backgrounding the page, pressing STOP, a sensor/control fault or tilt above 35 degrees stops/disarms. An explicit ARM at zero throttle is required again. The phone must stay on the controller page. The HTTP server runs at lower priority than the control loop; runtime load must still be measured on this C3. Any fresh-sample gap above 25 ms latches a fault.

Firmware uses a fresh-sample 200 Hz controller and 50 Hz ESC outputs. The roll/pitch rate P=0.7, D=0.002 and outer angle gain=3 retain the previous controller values. **Roll and pitch I are both 0.12.** V2 rolls pitch I back from v1's 0.24 after the reported props-on stand wobble. The integral remains bounded to +/-25 us with anti-windup. This is a controlled return to the prior setting, not proof that I caused the wobble. A gentler integral can remove a steady bias more slowly. All other gains, throttle limits, networking, stop protections and test duration are unchanged. No gain change can guarantee zero tilt or drift on the actual drone.

## Quiet USB bench option

If testing without the phone, keep all props removed, wait for READY and send a single line at 115200 baud:

```text
test 1400
```

This explicitly enables outputs, ramps toward 1400 with self-level correction and stops automatically **10 seconds after the command**. It requires no rapid `keep` typing. `test 1700` uses the same limits. `stop` always stops. `status` prints once while disarmed; `cal` stops and repeats calibration; `report` reprints the saved report. Old `bench`, `arm`, bare numbers and `keep` are not commands in this sketch; an unknown USB command stops a run.

## Saved report

After stopping, the report prints automatically over USB and is available using the phone's report link. It contains a rolling window of the final approximately 15 seconds at 20 Hz, angle ranges, final pre-stop motor commands, integrals, limiter state and largest fresh-sample gap. No instantaneous copying is needed. Motor values are software pulse requests, not measured RPM or thrust. A sensor-fault stop can contain the last valid sensor data.

V2 adds a **STEADY WINDOW** motion summary after the original report. It considers only the final contiguous window with the same throttle target >=1250 us, base within 5 us of target and zero steering/yaw commands. It excludes its first 0.5 seconds for settling and requires at least 1 second / 20 samples afterward. It reports average lean, linear drift, motion RMS after removing that drift, gyro-rate RMS, sampled acceleration range, limiter fraction and time near the integral cap. Statistics describe this selected window, not the whole run.

`crossingHz` is a rough rate of repeated crossings about the fitted drift line with at least 0.5 degrees hysteresis. It requires at least three upward crossings and reasonably consistent periods. **Zero means insufficient or irregular crossings; it does not prove no wobble.** This is not a vibration spectrum or automatic PID tuner, and 20 Hz logging cannot resolve fast vibration. Pilot steering, ramping and short windows are excluded rather than given a misleading estimate. Stand motion and vibration can affect these results; they are not free-flight validation.

The MPU-only controller cannot reliably observe sideways position or altitude. This version has no voltage sensing or automatic landing. Software timeouts require the CPU to be running; hardware PWM can persist during a CPU hang. The existing pulldown/signal-loss protection is not an independent processor-hang watchdog.

## V3 comparison: leave the frame untouched after stopping

V3 keeps every V2 controller gain, output limit and stop protection unchanged. The latest V2 run had powered average roll 0.46 degrees and pitch 0.34 degrees, but pitch changed from 0.39 degrees just before STOP to 4.07 degrees afterward. The frame could have settled when thrust stopped, or vibration could have affected the attitude estimate. Those readings alone cannot distinguish the causes.

At ARM, V3 starts a second attitude estimate from the controller's current orientation and integrates only the calibrated gyro rates. This **gyro witness never controls the motors**. The report includes its pre-stop roll/pitch alongside the existing fused estimate.

After a run stops, leave the frame untouched for **2 seconds**. Fresh readings are collected automatically about 0.5, 1 and 2 seconds after STOP. USB appends them automatically; on the phone, refresh the saved report after 2 seconds. There is no timed typing requirement. Starting another run or sending `cal` cancels a pending comparison.

Each `OFF` line shows fused (`F`) and gyro-only (`G`) angles, their changes since STOP (`dF`, `dG`) and acceleration magnitude. A missing fresh healthy sample is explicitly marked unavailable. `valid=1` means gyro comparisons are available; it does not certify physical accuracy. Both estimates share the same sensor: gyro-only readings can drift or suffer vibration errors and are not an independent measurement of the frame. These diagnostics help investigate the change; they do not automatically tune the gains or prove flight stability.

See `VALIDATION.txt` for the checks actually completed and `V2-RUN.txt` for the supplied readings that prompted this comparison. Earlier sketch folders remain unchanged. This revision has not been uploaded or physically tested.
