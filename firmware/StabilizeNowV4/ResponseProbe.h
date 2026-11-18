#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// One explicitly requested fixture experiment. This does not identify thrust,
// battery voltage, fixture friction, or flight-ready PID settings.
class ResponseProbe {
 public:
  enum Axis { ROLL, PITCH };
  static constexpr float BIAS_US=20;
  static constexpr uint32_t DURATION_MS=7000;
  bool active=false,done=false;
  Axis axis=PITCH;
  const char *reason="No response check recorded";
  float rotation[2]={},correction[2]={};
  static bool parse(const char *line,Axis &axis,int &pulse) {
    char name[8]={}; int us=1400,n=-1;
    if (strncmp(line,"response ",9)) return false;
    if (sscanf(line,"response %7s %4d%n",name,&us,&n)!=2 || n!=(int)strlen(line)) return false;
    if (us<1250 || us>1400) return false;
    if (!strcmp(name,"pitch")) axis=PITCH;
    else if (!strcmp(name,"roll")) axis=ROLL;
    else return false;
    pulse=us; return true;
  }
  void begin(Axis selected) {
    *this=ResponseProbe{}; axis=selected; active=true;
    reason="Seven-second fixture response check running";
  }
  static int segment(uint32_t ms) {
    return ms>=2500 && ms<3250?0:(ms>=4250 && ms<5000?1:-1);
  }
  float bias(uint32_t ms) const {
    if (!active) return 0;
    const int s=segment(ms); return s<0?0:(s==0?BIAS_US:-BIAS_US);
  }
  void sample(uint32_t ms,float rate,float axisCorrection,float dt,bool clipped) {
    if (!active || ms<2500 || ms>=DURATION_MS) return;
    if (!isfinite(rate) || !isfinite(axisCorrection) || !isfinite(dt) || dt<=0 || dt>0.015f ||
        (haveStamp && ms<=lastStamp)) { bad=true; return; }
    lastStamp=ms; haveStamp=true; seconds+=dt;
    if (clipped) clippedSeconds+=dt;
    const int s=segment(ms);
    if (s>=0) { rotation[s]+=rate*dt; correction[s]+=axisCorrection*dt; window[s]+=dt; ++rows[s]; }
  }
  void finish(bool completed) {
    if (!active) return;
    active=false; done=true;
    for (unsigned s=0;s<2;++s) if (window[s]>0) correction[s]/=window[s];
    if (!completed) reason="CANCELED: no response conclusion; no gains changed";
    else if (bad || seconds<4.1f || seconds>4.9f || rows[0]<90 || rows[1]<90 ||
        window[0]<0.60f || window[1]<0.60f || window[0]>0.90f || window[1]>0.90f)
      reason="INCONCLUSIVE: insufficient fresh sensor data; no gains changed";
    else if (clippedSeconds>0.01f*seconds)
      reason="INCONCLUSIVE: correction/output limited; no gains changed";
    else if (rotation[0]<-0.5f && rotation[1]>0.5f)
      reason="OPPOSITE response measured: check motor positions and sensor axes before tuning";
    else if (rotation[0]>0.5f && rotation[1]<-0.5f)
      reason="BOTH DIRECTIONS measured: response present; PID tuning still unverified";
    else reason="NO CLEAR BIDIRECTIONAL RESPONSE: check fixture, motor mapping, thrust and power; cause not identified";
  }
  int format(char *out,size_t size) const {
    return snprintf(out,size,
        "RESPONSE CHECK %s: %s\n"
        "7 s total; two 0.75 s bounded +/-20 us axis biases; level feedback retained, integral frozen.\n"
        "Integrated gyro rotation +bias/-bias=%.2f/%.2f deg; screening needs >0.50/<-0.50.\n"
        "Mean requested axis correction +bias/-bias=%.2f/%.2f us (pair difference is twice this).\n"
        "Positive pitch: FRONT GPIO0/19 higher. Positive roll: LEFT GPIO0/3 higher.\n"
        "Requested pulses, not measured motor speed or thrust. No PID gains applied or saved.\n"
        "Hand movement or fixture forces can affect this result; it does not certify flight stability.\n",
        axis==PITCH?"PITCH":"ROLL",reason,rotation[0],rotation[1],correction[0],correction[1]);
  }
 private:
  float seconds=0,clippedSeconds=0,window[2]={};
  unsigned rows[2]={};
  uint32_t lastStamp=0;
  bool bad=false,haveStamp=false;
};
