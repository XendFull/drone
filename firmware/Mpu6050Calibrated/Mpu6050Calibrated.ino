// PRE-FLIGHT SENSOR CHECK ONLY. No motor commands and no arming command.
// Disconnect ESC LiPo. ESP32-C3 on USB; MPU VCC=3V3, SDA=10, SCL=1, AD0=GND.
// Keep the MPU absolutely still for calibration. Then tilt it by hand.
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#if !defined(CONFIG_IDF_TARGET_ESP32C3)
#error "Select ESP32C3 Dev Module."
#endif
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT
#error "Enable Tools > USB CDC On Boot."
#endif

constexpr uint8_t ESC_PINS[] = {2, 3, 5, 7};
constexpr unsigned CALIBRATION_SAMPLES = 600;
constexpr float PI_F = 3.14159265359f;
uint8_t address = 0;
bool calibrated = false, fault = false;
unsigned samples = 0, resets = 0;
float mean[6] = {}, m2[6] = {}, bias[3] = {};
float q[4] = {1, 0, 0, 0}; // Sensor -> reference frame; reference Z is up.
float rates[3] = {}, accelNorm = 0;
uint32_t lastPoll = 0, lastSample = 0, lastLog = 0, lastCalibration = 0;
uint32_t skippedLogs = 0, readErrors = 0;
char tx[1024];
size_t txHead=0, txTail=0;

// Telemetry is disposable: never wait for an absent/slow USB reader.
void logLine(const char *line) {
  const size_t length = strlen(line);
  const size_t used=(txHead+sizeof(tx)-txTail)%sizeof(tx);
  if (length>sizeof(tx)-1-used) { ++skippedLogs; return; }
  for (size_t i=0; i<length; ++i) { tx[txHead]=line[i]; txHead=(txHead+1)%sizeof(tx); }
}

void drainLog() {
  const int available=Serial.availableForWrite();
  size_t budget=available>0 ? min((size_t)available, (size_t)64) : 0;
  while (budget-- && txTail!=txHead) {
    if (Serial.write((uint8_t)tx[txTail])!=1) break;
    txTail=(txTail+1)%sizeof(tx);
  }
}

void fail(const char *reason) {
  fault = true;
  logLine(reason);
}

bool readAt(uint8_t device, uint8_t reg, uint8_t *data, size_t length) {
  Wire.beginTransmission(device);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(device, length, true) != length) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (size_t i = 0; i < length; ++i) data[i] = Wire.read();
  return true;
}

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg); Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool setReg(uint8_t reg, uint8_t value) {
  uint8_t actual;
  return writeReg(reg, value) && readAt(address, reg, &actual, 1) && actual == value;
}

int16_t word(const uint8_t *p) {
  const uint16_t u = ((uint16_t)p[0] << 8) | p[1];
  return u < 32768 ? (int16_t)u : (int16_t)((int32_t)u - 65536);
}

bool configure() {
  for (uint8_t candidate = 0x68; candidate <= 0x69; ++candidate) {
    uint8_t id;
    if (readAt(candidate, 0x75, &id, 1) && id == 0x68) { address = candidate; break; }
  }
  if (!address || !writeReg(0x6B, 0x80)) return false;
  delay(100);
  if (!setReg(0x6B, 0x01)) return false;
  delay(100);
  return setReg(0x6C, 0) && setReg(0x1A, 3) && setReg(0x19, 4) &&
         setReg(0x1B, 0x08) && setReg(0x1C, 0x08) && setReg(0x38, 0x01);
  // 200 Hz data, DLPF ~42 Hz gyro, +/-500 deg/s (65.5 LSB), +/-4g (8192 LSB).
}

void restartCalibration() {
  samples = 0;
  memset(mean, 0, sizeof(mean));
  memset(m2, 0, sizeof(m2));
  ++resets;
}

void initializeAttitude() {
  const float roll = atan2f(mean[1], mean[2]);
  const float pitch = atan2f(-mean[0], sqrtf(mean[1]*mean[1] + mean[2]*mean[2]));
  const float cr = cosf(roll/2), sr = sinf(roll/2);
  const float cp = cosf(pitch/2), sp = sinf(pitch/2);
  q[0] = cr*cp; q[1] = sr*cp; q[2] = cr*sp; q[3] = -sr*sp;
}

void calibrate(const float *a, const float *g) {
  bool moving = accelNorm < 0.75f || accelNorm > 1.25f;
  for (unsigned i = 0; i < 3; ++i) {
    if (fabsf(g[i]) > 25) moving = true;
    if (samples > 30 && (fabsf(a[i] - mean[i]) > 0.08f || fabsf(g[i] - mean[i+3]) > 3)) moving = true;
  }
  if (moving) { restartCalibration(); return; }
  ++samples;
  for (unsigned i = 0; i < 6; ++i) {
    const float v = i < 3 ? a[i] : g[i-3];
    const float delta = v - mean[i];
    mean[i] += delta / samples;
    m2[i] += delta * (v - mean[i]);
  }
  if (samples < CALIBRATION_SAMPLES) return;
  for (unsigned i = 0; i < 3; ++i) {
    if (sqrtf(m2[i]/(samples-1)) > 0.025f || sqrtf(m2[i+3]/(samples-1)) > 0.8f || fabsf(mean[i+3]) > 15) {
      restartCalibration(); return;
    }
    bias[i] = mean[i+3];
  }
  initializeAttitude();
  calibrated = true;
  logLine("CALIBRATED. Tilt by hand; R/P are SENSOR axes. No motor output.\n");
  // Do not subtract the mean acceleration: that would remove gravity.
}

void updateAttitude(const float *a, const float *g, float dt) {
  float w[3];
  for (unsigned i = 0; i < 3; ++i) {
    rates[i] = g[i] - bias[i];
    w[i] = rates[i] * PI_F / 180;
  }
  if (accelNorm > 0.8f && accelNorm < 1.2f) {
    const float ax=a[0]/accelNorm, ay=a[1]/accelNorm, az=a[2]/accelNorm;
    const float vx=2*(q[1]*q[3]-q[0]*q[2]);
    const float vy=2*(q[2]*q[3]+q[0]*q[1]);
    const float vz=1-2*(q[1]*q[1]+q[2]*q[2]);
    // Gravity correction; yaw is unobservable with a six-axis IMU.
    w[0] += 2*(ay*vz-az*vy);
    w[1] += 2*(az*vx-ax*vz);
    w[2] += 2*(ax*vy-ay*vx);
  }
  const float w0=q[0], x=q[1], y=q[2], z=q[3];
  q[0] += (-x*w[0]-y*w[1]-z*w[2])*dt/2;
  q[1] += (w0*w[0]+y*w[2]-z*w[1])*dt/2;
  q[2] += (w0*w[1]-x*w[2]+z*w[0])*dt/2;
  q[3] += (w0*w[2]+x*w[1]-y*w[0])*dt/2;
  const float norm=sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
  if (!isfinite(norm) || norm < 0.5f) { fail("FAULT: invalid attitude. Reset to recalibrate.\n"); return; }
  for (float &v : q) v /= norm;
}

void setup() {
  for (uint8_t pin : ESC_PINS) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
  Serial.begin(115200);
  const uint32_t start=millis();
  while (!Serial && (uint32_t)(millis()-start)<2500) delay(1);
  logLine("CALIBRATED MPU CHECK ONLY. LiPo disconnected. Keep still for 3 seconds.\n");
  if (!Wire.begin(10, 1, 100000)) { fail("FAULT: I2C initialization.\n"); return; }
  Wire.setTimeOut(10);
  if (!configure()) { fail("FAULT: MPU missing or configuration failed. Check wiring; reset to retry.\n"); return; }
  lastPoll=lastSample=micros();
}

void loop() {
  drainLog();
  if (fault) {
    if ((uint32_t)(millis()-lastLog)>1000) {
      lastLog=millis(); logLine("FAULT latched. Check wiring and reset. No motor commands.\n");
    }
    delay(1); return;
  }
  const uint32_t now=micros();
  if ((uint32_t)(now-lastPoll)<1000) { delay(1); return; }
  lastPoll=now;
  uint8_t status=0, data[14];
  if (!readAt(address, 0x3A, &status, 1)) {
    ++readErrors; fail("FAULT: I2C read failed.\n"); return;
  }
  if (!(status & 1)) {
    if ((uint32_t)(now-lastSample)>25000) fail("FAULT: no fresh MPU data for 25 ms.\n");
    return;
  }
  if (!readAt(address, 0x3B, data, sizeof(data))) {
    ++readErrors; fail("FAULT: MPU data read failed.\n"); return;
  }
  const uint32_t timestamp=micros();
  const uint32_t elapsed=timestamp-lastSample;
  lastSample=timestamp;
  if (elapsed>25000) { fail("FAULT: sample gap exceeded 25 ms.\n"); return; }
  float a[3], g[3];
  for (unsigned i=0; i<3; ++i) {
    const int16_t ar=word(data+2*i), gr=word(data+8+2*i);
    if (abs((int)ar)>32000 || abs((int)gr)>32000) { fail("FAULT: sensor range saturated.\n"); return; }
    a[i]=ar/8192.0f; g[i]=gr/65.5f;
  }
  accelNorm=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
  if (!calibrated) {
    // Calibration must use a continuous stationary interval.
    if (samples && timestamp-lastCalibration>10000) restartCalibration();
    lastCalibration=timestamp;
    calibrate(a, g);
  } else updateAttitude(a, g, elapsed*0.000001f);
  if ((uint32_t)(millis()-lastLog)<250) return;
  lastLog=millis();
  char line[220];
  if (!calibrated) {
    snprintf(line, sizeof(line), "KEEP STILL calibration=%u/%u restarts=%u |a|=%.3fg\n", samples, CALIBRATION_SAMPLES, resets, accelNorm);
  } else {
    const float roll=atan2f(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]))*180/PI_F;
    const float s=constrain(2*(q[0]*q[2]-q[3]*q[1]), -1.0f, 1.0f);
    const float pitch=asinf(s)*180/PI_F;
    snprintf(line, sizeof(line), "R/P=%.1f %.1f rate=%.2f %.2f %.2f bias=%.2f %.2f %.2f |a|=%.3f errors=%lu dropped=%lu\n",
      roll,pitch,rates[0],rates[1],rates[2],bias[0],bias[1],bias[2],accelNorm,(unsigned long)readErrors,(unsigned long)skippedLogs);
  }
  logLine(line);
}
