#pragma once
#include <stdint.h>
#include <stddef.h>

struct FlightRow {
  uint32_t ms=0;
  float roll=0,pitch=0,rates[3]={},integral[3]={},g=0;
  float gyroAngles[2]={};
  float integralCap[2]={25,25};
  uint16_t base=1000,target=1000,sent[4]={1000,1000,1000,1000};
  bool limited=false;
  float calculatedPitch=0;
  float balanceTrim[2]={}; // Scheduled R/P pulse trim, before mixer/axis limits.
  float pitchTargetRate=0,pitchControlRate=0,pitchPterm=0,pitchDterm=0;
  float pitchGains[3]={},angleGain=0; // Actual runtime gains, before STOP restores trials.
  uint16_t controlAgeMs=0;
  bool controlValid=false;
  int8_t requested[3]={}; // roll/pitch degrees, yaw degrees/s.
};
class FlightRecorder {
 public:
  static constexpr size_t CAPACITY=302;
  FlightRow rows[CAPACITY];
  size_t count=0;
  void begin(uint32_t now) { start=last=now; next=count=0; }
  void capture(uint32_t now,FlightRow row,bool final=false) {
    if (!final && count && (uint32_t)(now-last)<50) return;
    row.ms=now-start;
    if (count && at(count-1).ms==row.ms) rows[(next+CAPACITY-1)%CAPACITY]=row;
    else {
      rows[next]=row; next=(next+1)%CAPACITY;
      if (count<CAPACITY) ++count;
    }
    last=now;
  }
  const FlightRow &at(size_t i) const { return rows[(next+CAPACITY-count+i)%CAPACITY]; }
 private:
  uint32_t start=0,last=0;
  size_t next=0;
};
