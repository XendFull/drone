#pragma once
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "FlightRecorder.h"

// Read-only pitch diagnostics. Keep adjacent 50 ms samples around angle/rate
// extrema, plus an overview and the final rows; never format while armed.
inline size_t formatPitchTrace(char *out,size_t size,const FlightRecorder &record) {
  size_t used=0;if(size)out[0]=0;
  const auto append=[&](const char *fmt,...) {
    if(!size || used>=size-1)return;
    va_list args;va_start(args,fmt);const int n=vsnprintf(out+used,size-used,fmt,args);va_end(args);
    if(n>0)used+=(size_t)n<size-used?(size_t)n:size-used-1;
  };
  append("PITCH TIMELINE: selected 20 Hz samples; adjacent rows around sampled extrema retained.\n");
  if(!record.count){append("No recorded samples.\n");return used;}
  const auto &last=record.at(record.count-1);
  append("Runtime pitch P/I/D=%.3f/%.3f/%.4f; level strength=%.2f. Captured before STOP/restoration.\n",
      last.pitchGains[0],last.pitchGains[1],last.pitchGains[2],last.angleGain);
  append("t(ms) pitch q usedQ targetQ Pterm I Dterm bias calc sent age(ms)\n");
  size_t low=0,high=0,fast=0;
  for(size_t i=1;i<record.count;++i){
    if(record.at(i).pitch<record.at(low).pitch)low=i;
    if(record.at(i).pitch>record.at(high).pitch)high=i;
    if(fabsf(record.at(i).rates[1])>fabsf(record.at(fast).rates[1]))fast=i;
  }
  const size_t stride=(record.count+9)/10;
  const auto near=[](size_t a,size_t b){return a>b?a-b<=3:b-a<=3;};
  for(size_t i=0;i<record.count;++i){
    if(i%stride && !near(i,low) && !near(i,high) && !near(i,fast) && i+3<record.count)continue;
    const auto &r=record.at(i);
    const float sentPitch=((float)r.sent[0]+r.sent[1]-r.sent[2]-r.sent[3])*0.25f;
    append("%lu %.1f %.1f ",(unsigned long)r.ms,r.pitch,r.rates[1]);
    if(r.controlValid)
      append("%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %u\n",r.pitchControlRate,r.pitchTargetRate,
          r.pitchPterm,r.integral[1],r.pitchDterm,r.balanceTrim[1],r.calculatedPitch,sentPitch,r.controlAgeMs);
    else append("NA NA NA %.1f NA %.1f NA %.1f NA\n",r.integral[1],r.balanceTrim[1],sentPitch);
  }
  append("pitch=deg; q/usedQ/targetQ=deg/s; terms/bias/calc/sent=us per motor.\n"
      "q is capture-time rate; usedQ/terms are from the last calculation, whose age is shown.\n"
      "calc includes axis/mixer limits; sent is the last requested pulse difference, not measured thrust.\n"
      "Selected samples cannot establish actuator delay or resolve fast vibration. END PITCH TIMELINE.\n");
  return used;
}

// Read-only formatting after every motor has been requested OFF.
inline size_t formatYawTrace(char *out,size_t size,const FlightRecorder &record) {
  size_t used=0;if(size)out[0]=0;
  const auto append=[&](const char *fmt,...) {
    if(!size || used>=size-1)return;
    va_list args;va_start(args,fmt);const int n=vsnprintf(out+used,size-used,fmt,args);va_end(args);
    if(n>0)used+=(size_t)n<size-used?(size_t)n:size-used-1;
  };
  append("YAW TIMELINE: sampled commands, not measured motor torque. Positive yaw = clockwise.\n"
      "t(ms) base targetYaw measuredYaw sentYaw IYaw\n");
  if(!record.count) {append("No recorded samples.\n");return used;}
  const size_t stride=(record.count+14)/15;
  size_t peak=0;
  for(size_t i=1;i<record.count;++i)
    if(fabsf(record.at(i).rates[2])>fabsf(record.at(peak).rates[2]))peak=i;
  for(size_t i=0;i<record.count;++i) {
    if(i%stride && i!=peak && i+1!=record.count)continue;
    const auto &r=record.at(i);
    const float sentYaw=((float)r.sent[0]-r.sent[1]+r.sent[2]-r.sent[3])*0.25f;
    append("%lu %u %d %.1f %.1f %.1f\n",(unsigned long)r.ms,r.base,(int)r.requested[2],
        r.rates[2],sentYaw,r.integral[2]);
  }
  append("Yaw rates/targets are deg/s; sentYaw/IYaw are us. Includes peak sampled yaw and final row.\n"
      "20 Hz snapshots cannot resolve fast vibration or establish actuator response by themselves.\n");
  return used;
}
