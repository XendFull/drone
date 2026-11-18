#pragma once
#include <stdio.h>
#include <stdarg.h>
#include "FlightRecorder.h"
#include "PostStopTrack.h"

// Formatting is called only after motor OFF and the post-stop observations.
inline size_t formatHoldTrace(char *out,size_t size,const FlightRecorder &record,const PostStopTrack &off) {
  size_t used=0;if(size)out[0]=0;
  const auto append=[&](const char *fmt,...) {
    if(!size || used>=size-1)return;
    va_list args;va_start(args,fmt);const int n=vsnprintf(out+used,size-used,fmt,args);va_end(args);
    if(n>0)used+=(size_t)n<size-used?(size_t)n:size-used-1;
  };
  append("HOLD TIME TRACE: last up to 10 sampled rows before STOP.\n"
      "t=ms since arm; R/P=deg; q/z=body pitch/yaw deg/s; I/calc/sent=us pitch correction.\n"
      "calcAge=age of last calculation; sent=pulse request held for ESC, not measured thrust.\n"
      "t R P q z I calc sent calcAge\n");
  const size_t first=record.count>10?record.count-10:0;
  for(size_t i=first;i<record.count;++i) {
    const auto &r=record.at(i);
    const float sent=((float)r.sent[0]+r.sent[1]-r.sent[2]-r.sent[3])*0.25f;
    append("%lu %.1f %.1f %.1f %.1f %.1f ",(unsigned long)r.ms,r.roll,r.pitch,r.rates[1],r.rates[2],r.integral[1]);
    if(r.controlValid)append("%.1f %.1f %u\n",r.calculatedPitch,sent,r.controlAgeMs);
    else append("NA %.1f NA\n",sent);
  }
  append("AFTER OFF request: +ms pitch gyroPitch q z g; gyroPitch is diagnostic, not independent truth.\n");
  for(const auto &s:off.samples) {
    if(!s.valid) {append("+%lu UNAVAILABLE (no fresh healthy sample in window)\n",(unsigned long)s.ms);continue;}
    const auto &p=s.pose;
    append("+%lu %.2f ",(unsigned long)s.ms,p.pitch);
    if(p.gyroValid)append("%.2f ",p.gyroPitch);else append("NA ");
    append("%.2f %.2f %.2f\n",p.pitchRate,p.yawRate,p.g);
  }
  append("END HOLD TRACE. Commands/motion only: this cannot isolate thrust, fixture forces or ESC delay by itself.\n");
  return used;
}
