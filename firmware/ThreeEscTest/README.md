# Three ESC bench test

Serial troubleshooting: the IDE was observed with USB CDC On Boot Disabled while COM8 received brownout system messages but no sketch replies. This option has been changed to Enabled in Arduino IDE. The sketch now rejects compilation if USB CDC On Boot is disabled, preventing silent routing of `Serial` away from native USB. Re-upload is required for this change to take effect. The updated sketch compiled successfully with Core 3.3.10. Brownout reports are a separate power symptom and remain unresolved; keep ESC LiPo disconnected while checking `status` after upload.

ESP32-C3 SuperMini, Arduino-ESP32 Core 3.x. ESC1 PPM -> GPIO4, ESC2 PPM -> GPIO5, ESC3 PPM -> GPIO6. Each motor gets its own ESC. All ESC B- pads, LiPo negative, and ESP32 GND share ground. Each ESC needs its own 10k resistor from PPM to local B-. An optional 1k goes in series with each GPIO signal. ESC B+ connects to LiPo positive only; ESP32 uses USB power. Keep motor current out of the ESP32 ground wire.

Remove ALL propellers. Disconnect LiPo while wiring and uploading. Select ESP32C3 Dev Module, USB CDC On Boot Enabled, Serial Monitor 115200 baud and New Line. Power the ESP32 first, wait five seconds at 1000 us, and check `status`. Connect ESC power only after OFF pulses are running. Verify that every connected motor stays stopped before sending `arm`; a different ESC variant might not treat 1000 us as OFF. Disconnect LiPo immediately if a motor starts at the presumed OFF command.

Test each ESC individually first. Enter one command per line:

```text
m1
arm
1200
stop
m2
arm
1200
stop
m3
arm
1200
stop
```

If 1200 is below a motor's startup threshold, increase gradually using 1300, 1500, then 1800 while observing it. A command timeout disarms after 15 seconds without a valid throttle command or `keep`. `status` does not renew the timeout. After timeout, send `arm` again before throttle. Selecting another motor stops and disarms all attached outputs. Each numeric value applies only to the selected motor(s); 1000..2000 is supported, and other valid nonnegative integers are clamped. Malformed and overflow input is rejected.

After individual tests and pulldown/loss checks pass, simultaneous testing is explicit:

```text
all
arm
1200
stop
```

Simultaneous three-motor current capability of the battery and wiring remains unverified. Stop on sag, resets, cycling, or rapid heating. Bare-motor testing does not establish loaded capacity or flight readiness.

`loss` releases only the selected output(s), including while running. Nonselected attached outputs receive OFF. Test this ONLY without props and with ESC-side pulldowns installed. Motors must stop rather than hold speed or accelerate. `resume` restores all outputs at 1000 us, disarms, and restarts the five-second hold; then select/arm again. `stop` sends OFF to all attached outputs but leaves intentionally detached outputs in loss mode. Disconnect LiPo before disconnecting or resetting the ESP32 during routine power-down.

The sketch uses independent LEDC channels for 50 Hz, 14-bit hardware pulses. No immediate duty/frequency readback checks are used as proof of electrical output: earlier Core 3.3.10 diagnostic checks produced misleading failures. `status` reports requested values, attached state, reset reason and latched failures. Physical waveforms and motor behavior must still be verified. Documentation: [Espressif LEDC API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html).

A CPU hang can leave hardware producing the last throttle. The software timeout and resistor alone are not a guaranteed flight failsafe. Each ESC needs verified shutdown on actual wire loss, MCU reset and power loss. The earlier contradictory ESC-input/battery voltage readings and loaded startup behavior remain unresolved; this firmware is a bench diagnostic, not flight firmware.
