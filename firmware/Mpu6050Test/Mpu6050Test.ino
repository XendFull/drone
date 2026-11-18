// Sensor-only test. ESC LiPo MUST be disconnected; ESP32 uses USB.
// MPU6050: VCC=3V3, GND=GND, SDA=GPIO10, SCL=GPIO1, AD0=GND.
// No external libraries required. Arduino-ESP32 Core 3.x.
#include <Arduino.h>
#include <Wire.h>
#include <esp_system.h>

#if !defined(CONFIG_IDF_TARGET_ESP32C3)
#error "Select ESP32C3 Dev Module."
#endif
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT
#error "Set Tools > USB CDC On Boot > Enabled."
#endif

uint8_t sensorAddress = 0;
bool ready = false;
bool busReady = false;
uint32_t lastPrint = 0;
uint32_t readErrors = 0;
uint32_t samples = 0;

bool readRegisters(uint8_t address, uint8_t reg, uint8_t *data, size_t count) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  const size_t received = Wire.requestFrom(address, count, true);
  if (received != count) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (size_t i = 0; i < count; ++i) {
    const int value = Wire.read();
    if (value < 0) return false;
    data[i] = (uint8_t)value;
  }
  return true;
}

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(sensorAddress);
  Wire.write(reg); Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool writeChecked(uint8_t reg, uint8_t value) {
  uint8_t readback = 0;
  return writeRegister(reg, value) &&
         readRegisters(sensorAddress, reg, &readback, 1) && readback == value;
}

int16_t signedWord(const uint8_t *p) {
  const uint16_t value = ((uint16_t)p[0] << 8) | p[1];
  return value <= 32767 ? (int16_t)value : (int16_t)((int32_t)value - 65536);
}

bool findAndConfigure() {
  sensorAddress = 0;
  for (uint8_t address = 0x68; address <= 0x69; ++address) {
    Wire.beginTransmission(address);
    const uint8_t error = Wire.endTransmission(true);
    if (error != 0) {
      Serial.printf("No ACK at 0x%02X (I2C error %u)\n", address, error);
      continue;
    }
    uint8_t identity = 0;
    if (!readRegisters(address, 0x75, &identity, 1)) {
      Serial.printf("Device at 0x%02X, but WHO_AM_I read failed\n", address);
      continue;
    }
    Serial.printf("Device at 0x%02X; WHO_AM_I=0x%02X (MPU6050 expects 0x68)\n", address, identity);
    if (identity == 0x68) { sensorAddress = address; break; }
    Serial.println("Unexpected identity: check actual sensor model; not configuring it.");
  }
  if (!sensorAddress) {
    Serial.println("MPU6050 not identified. Check 3V3/GND, SDA=10, SCL=1 and AD0. Retrying.\n");
    return false;
  }
  if (!writeRegister(0x6B, 0x80)) return false; // Device reset
  delay(100);
  if (!writeChecked(0x6B, 0x01)) return false; // Awake; gyro X PLL clock
  delay(100);
  if (!writeChecked(0x6C, 0x00) || // All axes enabled
      !writeChecked(0x1A, 0x03) || // DLPF ~44 Hz accel / ~42 Hz gyro
      !writeChecked(0x19, 0x09) || // Internal output 100 Hz
      !writeChecked(0x1B, 0x00) || // Gyro +/-250 degrees/s, 131 LSB/(degree/s)
      !writeChecked(0x1C, 0x00)) return false; // Accel +/-2g, 16384 LSB/g
  delay(100);
  Serial.printf("MPU6050 READY at 0x%02X. Tilt by hand; keep ESC LiPo DISCONNECTED.\n", sensorAddress);
  Serial.println("Printing fresh accel[g], gyro[deg/s], temperature[C] five times per second.");
  return true;
}

void setup() {
  // This sketch does NOT generate ESC commands. Motor battery stays unplugged.
  constexpr uint8_t escPins[] = {2, 3, 5, 7};
  for (uint8_t pin : escPins) {
    digitalWrite(pin, LOW); pinMode(pin, OUTPUT); digitalWrite(pin, LOW);
  }
  Serial.begin(115200);
  const uint32_t start = millis();
  while (!Serial && (uint32_t)(millis() - start) < 3000) delay(10);
  Serial.printf("MPU6050 SENSOR TEST: SDA=GPIO10 SCL=GPIO1, reset reason=%d\n", (int)esp_reset_reason());
  busReady = Wire.begin(10, 1, 100000);
  Wire.setTimeOut(30);
  if (!busReady) Serial.println("ERROR: I2C initialization failed. Reset after checking wiring.");
  if (busReady) ready = findAndConfigure();
  lastPrint = millis();
}

void loop() {
  const uint32_t now = millis();
  if (!busReady) {
    if ((uint32_t)(now - lastPrint) >= 1000) {
      lastPrint = now; Serial.println("ERROR: I2C not initialized. SDA=10 SCL=1. Reset to retry.");
    }
    delay(1); return;
  }
  if (!ready) {
    if ((uint32_t)(now - lastPrint) >= 2000) {
      lastPrint = now;
      ready = findAndConfigure();
      if (!ready && sensorAddress) Serial.println("ERROR: sensor configuration/readback failed; retrying.");
    }
    delay(1); return;
  }
  if ((uint32_t)(now - lastPrint) < 200) { delay(1); return; }
  lastPrint = now;
  uint8_t interruptStatus = 0;
  uint8_t data[14];
  if (!readRegisters(sensorAddress, 0x3A, &interruptStatus, 1) ||
      !readRegisters(sensorAddress, 0x3B, data, sizeof(data))) {
    ++readErrors; ready = false;
    Serial.printf("ERROR: sensor read failed (total=%lu); rechecking sensor.\n", (unsigned long)readErrors);
    return;
  }
  if (!(interruptStatus & 0x01)) {
    Serial.println("WARNING: no fresh sensor sample; not counting this as a valid reading."); return;
  }
  const float ax = signedWord(data) / 16384.0f;
  const float ay = signedWord(data + 2) / 16384.0f;
  const float az = signedWord(data + 4) / 16384.0f;
  const float temperature = signedWord(data + 6) / 340.0f + 36.53f;
  const float gx = signedWord(data + 8) / 131.0f;
  const float gy = signedWord(data + 10) / 131.0f;
  const float gz = signedWord(data + 12) / 131.0f;
  Serial.printf("sample=%lu accel[g]=%.3f %.3f %.3f gyro[deg/s]=%.2f %.2f %.2f temp[C]=%.2f errors=%lu\n",
                (unsigned long)++samples, ax, ay, az, gx, gy, gz, temperature, (unsigned long)readErrors);
}
