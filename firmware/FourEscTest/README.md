# Four ESC bench test

ESP32-C3 SuperMini with Espressif Arduino-ESP32 Core 3.x. Select ESP32C3 Dev Module and USB CDC On Boot Enabled. Monitor: 115200 baud, New Line. The sketch rejects compilation if native USB Serial is disabled.

| Motor command | Signal GPIO |
|---|---|
| m1 | 2 |
| m2 | 3 |
| m3 | 5 |
| m4 | 7 |

GPIO2 can output PWM after boot. Espressif recommends pulling GPIO2 up at startup due to glitches; the ESC's required 10k PPM-to-B- pulldown differs from that recommendation. Keep the ESC pulldown installed and verify startup/reset behavior without props before relying on this pin mapping. If startup is unreliable, use a different signal GPIO.

Each motor has its own ESC. Each ESC needs a local 10k resistor from PPM to B-. ESP32 GND, battery negative and all ESC B- pads share ground. Motor current returns through adequate battery-to-ESC conductors, not through the ESP32 ground lead. ESC B+ goes to battery positive; USB powers ESP32. Never connect ESC B+ to ESP32 3V3 or 5V.

Remove ALL props. Disconnect ESC LiPo for wiring/upload. Power ESP32 first and let OFF pulses run before switching ESC power on. Do not repeatedly cycle ESC power while a high command is active. 1000 us is presumed OFF based on the earlier ESC tests, but must be checked on the new ESC4 too. If it spins at 1000, disconnect LiPo and investigate its actual mapping before arming.

Upload FourEscTest. Wait five seconds after boot. Send `status`: all four should show attached=1, requested=1000 and fault=0. Then power the ESCs and verify all remain stopped. Test ESC4 first, entering each command separately:

```text
m4
arm
1200
stop
```

If needed increase gradually to 1500 and then 1800, briefly, with no prop. Verify starts/stops repeatedly. After normal operation, test signal loss with ESC4's pulldown installed, still without props:

```text
m4
arm
1500
loss
```

ESC4 must stop rather than hold speed or accelerate. If it fails, disconnect LiPo. `resume` restores all outputs to OFF and restarts the five-second hold. Test MCU reset and powered-down MCU behavior without props before treating ESC4's shutdown as verified.

`m1`, `m2`, `m3`, `m4` select a motor and stop/disarm all outputs; send `arm` again afterward. `all` explicitly selects all four. A number from 1000 to 2000 sets the selected output(s). `stop` requests OFF on every attached output and disarms; detached outputs remain in loss mode until `resume`. `keep` renews the 15-second command timeout, while `status` does not. A timeout reports Command timeout and requests OFF; this message alone does not prove battery sag. No automatic ramp is provided.

Only after ESC4 passes individual tests, an optional brief simultaneous test WITHOUT props is `resume` -> wait five seconds -> `all` -> `arm` -> `1200` -> `stop`. Loaded four-motor current capability has not been established. The switch rating, contradictory earlier supply readings, and intermittent startup with several props remain unresolved. Do not treat a successful bare-motor run as proof of loaded operation.

LEDC generates 50 Hz, 14-bit hardware pulses on four distinct channels. Requested duty is rounded as pulse_us * 16384 / 20000. A queued nonblocking logger, bounded input parsing, explicit arming, five-second startup hold and 15-second command timeout are retained from ThreeEscTest. No immediate duty/frequency readback is treated as electrical proof. Physical output must still be checked. A CPU hang can keep the last hardware command running; software timeout is not an independent flight failsafe. 50 Hz actuator latency remains a separate issue to evaluate before PID/flight.
