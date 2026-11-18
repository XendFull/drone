// ESP32-C3 SuperMini: ESC1 PPM=GPIO4, ESC2=GPIO5, ESC3=GPIO6.
// BENCH ONLY: remove ALL props; one ESC-side 10k PPM-to-B- per ESC.
// USB powers ESP32. LiPo powers ESCs directly. All grounds common.
#include <Arduino.h>
#include <esp_arduino_version.h>
#include <esp_system.h>
#include <stdarg.h>
#include <errno.h>
#include <stdlib.h>
#include <ctype.h>

#if !defined(CONFIG_IDF_TARGET_ESP32C3)
#error "Select ESP32C3 Dev Module."
#endif
#if ESP_ARDUINO_VERSION_MAJOR != 3
#error "Use Espressif Arduino-ESP32 Core 3.x (tested with 3.3.10)."
#endif
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT
#error "Set Tools > USB CDC On Boot > Enabled, then upload again."
#endif

constexpr uint8_t PINS[3] = {4, 5, 6};
constexpr uint32_t HOLD_MS = 5000, TIMEOUT_MS = 15000;
constexpr uint16_t OFF_US = 1000; // Verify OFF independently for EACH ESC.
bool attached[3] = {false, false, false};
uint16_t pulses[3] = {OFF_US, OFF_US, OFF_US};
bool armed = false, faulted = false;
const char *faultReason = "none";
uint8_t selected = 0; // 0..2 = one motor, 3 = explicitly selected ALL
uint32_t holdStart = 0, lastCommand = 0;
esp_reset_reason_t bootReason;
char rx[32];
size_t rxLength = 0;
bool invalidLine = false;
char tx[1536];
size_t txHead = 0, txTail = 0;
uint32_t droppedLogs = 0;

// No waiting for a serial host. Logs can be dropped instead of blocking safety.
void say(const char *format, ...) {
  char msg[220];
  va_list args;
  va_start(args, format);
  vsnprintf(msg, sizeof(msg), format, args);
  va_end(args);
  const size_t length = strlen(msg);
  const size_t used = (txHead + sizeof(tx) - txTail) % sizeof(tx);
  if (length > sizeof(tx) - 1 - used) { ++droppedLogs; return; }
  for (size_t i = 0; i < length; ++i) {
    tx[txHead] = msg[i]; txHead = (txHead + 1) % sizeof(tx);
  }
}

void drainLog() {
  const int available = Serial.availableForWrite();
  size_t budget = available > 0 ? min((size_t)available, (size_t)64) : 0;
  while (budget-- && txTail != txHead) {
    if (Serial.write((uint8_t)tx[txTail]) != 1) break;
    txTail = (txTail + 1) % sizeof(tx);
  }
}

uint32_t dutyFor(uint16_t us) {
  return ((uint32_t)us * 16384UL + 10000UL) / 20000UL;
}

bool targets(uint8_t i) { return selected == 3 || selected == i; }
bool allAttached() { return attached[0] && attached[1] && attached[2]; }

const char *resetName() {
  switch (bootReason) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_USB: return "USB reset";
    default: return "other";
  }
}

void lowPin(uint8_t i) {
  digitalWrite(PINS[i], LOW); pinMode(PINS[i], OUTPUT);
  digitalWrite(PINS[i], LOW);
}

// Latched fault: best-effort OFF on working outputs; LOW on failed ones.
// Neither a GPIO LOW nor the software timeout guarantees ESC shutdown.
void fail(const char *reason) {
  armed = false; faulted = true; faultReason = reason;
  for (uint8_t i = 0; i < 3; ++i) {
    if (attached[i] && ledcWrite(PINS[i], dutyFor(OFF_US))) {
      pulses[i] = OFF_US;
    } else {
      if (attached[i]) ledcDetach(PINS[i]);
      attached[i] = false; lowPin(i);
    }
  }
  say("FAULT: %s. Disconnect ESC LiPo. Reboot only after power removed.\n", reason);
}

bool writePulse(uint8_t i, uint16_t us) {
  if (!attached[i] || faulted) return false;
  if (!ledcWrite(PINS[i], dutyFor(us))) { fail("LEDC write failed"); return false; }
  pulses[i] = us;
  return true;
}

bool attachOff(uint8_t i) {
  if (attached[i]) return writePulse(i, OFF_US);
  lowPin(i);
  // Unique channels give independent duty cycles at the same 50 Hz.
  if (!ledcAttachChannel(PINS[i], 50, 14, i)) { fail("LEDC attach failed"); return false; }
  attached[i] = true;
  return writePulse(i, OFF_US);
}

bool stopAll() {
  armed = false;
  bool ok = !faulted;
  for (uint8_t i = 0; i < 3 && !faulted; ++i) {
    if (attached[i] && !writePulse(i, OFF_US)) ok = false;
  }
  return ok && !faulted;
}

void status() {
  say("Core %s reset=%s(%d) uptime=%lu ms\n", ESP_ARDUINO_VERSION_STR,
      resetName(), (int)bootReason, (unsigned long)millis());
  say("selected=%s armed=%d fault=%d reason=%s droppedLogs=%lu\n",
      selected == 3 ? "ALL" : selected == 0 ? "m1" : selected == 1 ? "m2" : "m3",
      armed, faulted, faultReason, (unsigned long)droppedLogs);
  for (uint8_t i = 0; i < 3; ++i) {
    say("m%u GPIO%u attached=%d requested=%u us duty=%lu (inactive if detached)\n",
        i + 1, PINS[i], attached[i], pulses[i], (unsigned long)dutyFor(pulses[i]));
  }
}

void command(char *s) {
  while (*s == ' ' || *s == '\t') ++s;
  size_t n = strlen(s);
  while (n && (s[n-1] == ' ' || s[n-1] == '\t')) s[--n] = 0;
  if (!n) return;
  if (!strcmp(s, "status")) { status(); return; }
  if (!strcmp(s, "help")) {
    say("m1/m2/m3/all, arm, 1000..2000, keep, stop, status, loss, resume. NO PROPS.\n"); return;
  }
  if (!strcmp(s, "stop")) {
    stopAll(); say("Disarmed. OFF requested on all attached outputs.\n"); return;
  }
  if (faulted) { say("Fault latched: %s. Disconnect LiPo.\n", faultReason); return; }
  if (!strcmp(s, "resume")) {
    if (!stopAll()) return;
    for (uint8_t i = 0; i < 3; ++i) if (!attachOff(i)) return;
    holdStart = millis(); say("All outputs at 1000 us; wait 5 seconds before arm.\n"); return;
  }
  if (!strcmp(s, "loss")) {
    armed = false;
    for (uint8_t i = 0; i < 3; ++i) {
      if (!targets(i)) { if (attached[i] && !writePulse(i, OFF_US)) return; continue; }
      if (attached[i]) {
        if (!ledcDetach(PINS[i])) { fail("LEDC detach failed"); return; }
        attached[i] = false;
      }
      pinMode(PINS[i], INPUT); // External 10k must remain at ESC.
    }
    say("Selected output(s) high impedance. NO PROPS. Observe motor stop; resume restores OFF.\n"); return;
  }
  if (!allAttached()) { say("Loss mode: send resume, then wait 5 seconds.\n"); return; }
  if (!strcmp(s, "m1") || !strcmp(s, "m2") || !strcmp(s, "m3") || !strcmp(s, "all")) {
    if (!stopAll()) return;
    selected = !strcmp(s, "all") ? 3 : (uint8_t)(s[1] - '1');
    say("Selected %s; all outputs stopped/disarmed. Send arm next.\n", s); return;
  }
  if (!strcmp(s, "arm")) {
    if ((uint32_t)(millis() - holdStart) < HOLD_MS) { say("Wait for 5-second OFF hold.\n"); return; }
    if (!stopAll()) return;
    armed = true; lastCommand = millis();
    say("ARMED %s. Send throttle; stop affects all. Timeout 15 s.\n",
        selected == 3 ? "ALL THREE" : selected == 0 ? "m1" : selected == 1 ? "m2" : "m3"); return;
  }
  if (!armed) { say("Disarmed. Select m1/m2/m3 or all, then arm.\n"); return; }
  if (!strcmp(s, "keep")) { lastCommand = millis(); return; }
  for (size_t i = 0; s[i]; ++i) {
    if (!isdigit((unsigned char)s[i])) { say("Unknown command. Send help.\n"); return; }
  }
  errno = 0;
  const unsigned long value = strtoul(s, nullptr, 10);
  if (errno == ERANGE) { say("Integer overflow rejected.\n"); return; }
  const uint16_t us = value < 1000 ? 1000 : value > 2000 ? 2000 : (uint16_t)value;
  if (us != value) say("Clamped to %u us.\n", us);
  for (uint8_t i = 0; i < 3; ++i) if (targets(i) && !writePulse(i, us)) return;
  lastCommand = millis();
  say("Requested %u us, duty=%lu/16384 on selected output(s).\n", us, (unsigned long)dutyFor(us));
}

void serviceTimeout() {
  if (armed && (uint32_t)(millis() - lastCommand) >= TIMEOUT_MS) {
    stopAll(); say("Command timeout: all attached outputs OFF, disarmed.\n");
  }
}

void setup() {
  for (uint8_t i = 0; i < 3; ++i) lowPin(i);
  bootReason = esp_reset_reason();
  for (uint8_t i = 0; i < 3; ++i) if (!attachOff(i)) break;
  holdStart = millis();
  Serial.begin(115200); // Do not wait for USB before starting hardware pulses.
  say("THREE ESC BENCH TEST. ALL PROPS OFF. Wait 5 s; default m1; help for commands.\n");
  status();
}

void loop() {
  serviceTimeout();
  for (uint8_t i = 0; i < 32 && Serial.available(); ++i) {
    const int c = Serial.read();
    if (c < 0) break;
    if (c == '\r' || c == '\n') {
      if (invalidLine) say("Malformed/overlong line rejected.\n");
      else { rx[rxLength] = 0; command(rx); }
      rxLength = 0; invalidLine = false;
    } else if (c == '\t' || (c >= 32 && c <= 126)) {
      if (!invalidLine && rxLength < sizeof(rx) - 1) rx[rxLength++] = (char)c;
      else invalidLine = true;
    } else invalidLine = true;
  }
  serviceTimeout();
  drainLog();
  delay(1); // Scheduler yield. Hardware pulses continue independently.
}
