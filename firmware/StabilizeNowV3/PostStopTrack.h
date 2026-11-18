#pragma once
#include <stdint.h>
#include <stddef.h>

struct StopPose {
  float roll=0,pitch=0,gyroRoll=0,gyroPitch=0,g=0;
  bool gyroValid=false;
};
struct StopSample {
  uint32_t ms=0;
  StopPose pose;
  bool valid=false;
};
class PostStopTrack {
 public:
  bool active=false;
  StopSample before,samples[3];
  size_t count=0;
  void begin(uint32_t now,const StopPose &pose,bool healthy) {
    start=now; active=true; count=0;
    before={0,pose,healthy};
    for (auto &s:samples) s={};
  }
  bool cancel() { const bool wasActive=active; active=false; return wasActive; }
  // Called from the control thread after polling the IMU. Missed sampling
  // windows are marked unavailable rather than reusing an old sensor value.
  bool tick(uint32_t now,bool fresh,bool healthy,const StopPose &pose) {
    if (!active) return false;
    static constexpr uint32_t due[3]={500,1000,2000};
    const uint32_t elapsed=now-start;
    while (count<3 && elapsed>=due[count]) {
      if (elapsed<=due[count]+100 && fresh && healthy) {
        samples[count++]={elapsed,pose,true};
      } else if (elapsed>due[count]+100) {
        samples[count++]={due[count],{},false};
      } else return false;
    }
    if (count==3) { active=false; return true; }
    return false;
  }
 private:
  uint32_t start=0;
};
