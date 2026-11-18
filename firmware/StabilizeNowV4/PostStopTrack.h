#pragma once
#include <stdint.h>
#include <stddef.h>

struct StopPose {
  float roll=0,pitch=0,gyroRoll=0,gyroPitch=0,g=0;
  bool gyroValid=false;
  float pitchRate=0,yawRate=0;
};
struct StopSample {
  uint32_t ms=0;
  StopPose pose;
  bool valid=false;
};
class PostStopTrack {
 public:
  static constexpr size_t COUNT=7;
  bool active=false;
  StopSample before,samples[COUNT];
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
    static constexpr uint32_t due[COUNT]={20,50,100,250,500,1000,2000};
    const uint32_t elapsed=now-start;
    while (count<COUNT && elapsed>=due[count]) {
      const uint32_t tolerance=count<4?10:100;
      if (elapsed<=due[count]+tolerance && fresh && healthy) {
        samples[count++]={elapsed,pose,true};
      } else if (elapsed>due[count]+tolerance) {
        samples[count++]={due[count],{},false};
      } else return false;
    }
    if (count==COUNT) { active=false; return true; }
    return false;
  }
 private:
  uint32_t start=0;
};
