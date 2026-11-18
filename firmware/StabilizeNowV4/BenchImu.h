#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "FlightMath.h"

class BenchImu {
 public:
  bool ready=false;
  const char *fault=nullptr;
  const char *calibrationWait="collecting still samples";
  const char *lastRestart="none";
  uint16_t count=0;
  uint32_t restarts=0, errors=0;
  int face=1;
  float bias[3]={}, dt=0.005f;
  flight::Vec accel={}, rate={};
  flight::Quaternion attitude;
  flight::Quaternion gyroOnly;
  bool gyroOnlyValid=false;

  void resetGyroWitness() {
    gyroOnly=attitude;
    gyroOnlyValid=ready && !fault;
  }
  // NVS writes are performed only while motors are OFF. Resync the sample
  // clock afterward; this never bypasses an armed-loop gap or a latched fault.
  void resyncDisarmedClock() { lastSample=lastPoll=micros(); }

  bool begin() {
    if (!Wire.begin(6,7,100000)) return fail("I2C init failed");
    Wire.setTimeOut(10);
    for (uint8_t a=0x68; a<=0x69; ++a) {
      uint8_t id;
      if (read(a,0x75,&id,1) && id==0x68) { address=a; break; }
    }
    if (!address || !write(0x6B,0x80)) return fail("MPU6050 not identified");
    delay(100);
    if (!set(0x6B,1)) return fail("MPU wake failed");
    delay(100);
    if (!set(0x6C,0) || !set(0x1A,3) || !set(0x19,4) || !set(0x1B,8) || !set(0x1C,8) || !set(0x38,1))
      return fail("MPU register verification failed");
    lastSample=lastPoll=micros();
    return true;
  }

  void recalibrate() {
    ready=false;
    resetStats("operator recalibration");
    attitude={};
    gyroOnly={}; gyroOnlyValid=false;
  }

  bool tick() {
    if (fault) return false;
    const uint32_t now=micros();
    if ((uint32_t)(now-lastPoll)<1000) return false;
    lastPoll=now;
    uint8_t status, data[14];
    if (!read(address,0x3A,&status,1)) { ++errors; return fail("MPU status read failed"); }
    if (!(status&1)) {
      if ((uint32_t)(now-lastSample)>25000) fail("MPU data stale >25 ms");
      return false;
    }
    if (!read(address,0x3B,data,sizeof(data))) { ++errors; return fail("MPU sample read failed"); }
    const uint32_t stamp=micros(), gap=stamp-lastSample;
    lastSample=stamp;
    if (gap>25000) return fail("Sensor/control gap >25 ms");
    dt=gap*0.000001f;
    if (dt<0.001f) return false;
    float a[3], g[3];
    for (unsigned i=0; i<3; ++i) {
      const int16_t ar=word(data+2*i), gr=word(data+8+2*i);
      if (abs((int)ar)>32000 || abs((int)gr)>32000) return fail("MPU range saturated");
      a[i]=ar/8192.0f; g[i]=gr/65.5f;
    }
    if (!ready) { calibrate(a,g,gap); return false; }
    accel=mount.rotate(map(a));
    float corrected[3];
    for (unsigned i=0; i<3; ++i) corrected[i]=g[i]-bias[i];
    rate=mount.rotate(map(corrected));
    if (!attitude.update(accel,rate,dt)) return fail("Attitude invalid");
    // Diagnostic only: the controller continues to use 'attitude'.
    if (gyroOnlyValid && !gyroOnly.update({0,0,0},rate,dt)) gyroOnlyValid=false;
    return true;
  }

 private:
  uint8_t address=0;
  uint32_t lastSample=0, lastPoll=0;
  float mean[6]={}, m2[6]={};
  flight::Quaternion mount;

  bool fail(const char *reason) { fault=reason; ready=false; return false; }
  static int16_t word(const uint8_t *p) {
    const uint16_t u=((uint16_t)p[0]<<8)|p[1];
    return u<32768 ? (int16_t)u : (int16_t)((int32_t)u-65536);
  }
  bool read(uint8_t a,uint8_t reg,uint8_t *p,size_t n) {
    Wire.beginTransmission(a); Wire.write(reg);
    if (Wire.endTransmission(false)!=0) return false;
    if (Wire.requestFrom(a,n,true)!=n) { while (Wire.available()) Wire.read(); return false; }
    for (size_t i=0;i<n;++i) p[i]=Wire.read();
    return true;
  }
  bool write(uint8_t reg,uint8_t v) {
    Wire.beginTransmission(address); Wire.write(reg); Wire.write(v);
    return Wire.endTransmission(true)==0;
  }
  bool set(uint8_t reg,uint8_t v) {
    uint8_t r;
    return write(reg,v) && read(address,reg,&r,1) && r==v;
  }
  flight::Vec map(const float *v) const {
    // User mount: sensor +X toward front, +Y left with +Z face up.
    // Detect face while frame is level.
    // This remains a proper rotation on either sensor face, not a reflection.
    return flight::sensorToBody({v[0],v[1],v[2]},face);
  }
  void resetStats(const char *why) {
    count=0; memset(mean,0,sizeof(mean)); memset(m2,0,sizeof(m2)); ++restarts;
    lastRestart=calibrationWait=why;
  }
  void calibrate(const float *a,const float *g,uint32_t gap) {
    const float n=flight::norm({a[0],a[1],a[2]});
    if (n<0.75f || n>1.25f) { resetStats("acceleration outside 0.75..1.25g"); return; }
    if (gap>10000) { resetStats("sample spacing exceeded 10 ms"); return; }
    for (unsigned i=0;i<3;++i) {
      if (fabsf(g[i])>25) { resetStats("gyro motion exceeded 25 deg/s"); return; }
      if (count>30 && fabsf(a[i]-mean[i])>0.08f) { resetStats("accelerometer changed while calibrating"); return; }
      if (count>30 && fabsf(g[i]-mean[i+3])>3) { resetStats("gyro changed while calibrating"); return; }
    }
    calibrationWait="collecting still samples";
    ++count;
    for (unsigned i=0;i<6;++i) {
      const float v=i<3 ? a[i] : g[i-3], delta=v-mean[i];
      mean[i]+=delta/count; m2[i]+=delta*(v-mean[i]);
    }
    if (count<600) return;
    for (unsigned i=0;i<3;++i) {
      if (sqrtf(m2[i]/(count-1))>0.025f) { resetStats("accelerometer variation too high"); return; }
      if (sqrtf(m2[i+3]/(count-1))>0.8f) { resetStats("gyro variation too high"); return; }
      if (fabsf(mean[i+3])>15) { resetStats("gyro offset exceeds 15 deg/s"); return; }
      bias[i]=mean[i+3];
    }
    if (fabsf(mean[2])<0.9f*n) { fail("Sensor too tilted: frame level, sensor X forward; recalibrate after reset"); return; }
    face=mean[2]>0 ? 1 : -1;
    const flight::Vec a0=map(mean);
    mount=flight::Quaternion::fromGravity({-a0.x,-a0.y,-a0.z});
    attitude={};
    ready=true;
    resetGyroWitness();
  }
};
