#pragma once
#include <math.h>
#include <stdlib.h>
#include "FlightRecorder.h"

struct MotionAxis {
  float mean=0,slope=0,residualRms=0,rateRms=0,hz=0,periodCv=0;
  unsigned rises=0,capped=0;
};
struct WobbleSummary {
  bool valid=false;
  size_t first=0,count=0;
  float seconds=0,gMin=0,gMax=0;
  unsigned limited=0;
  MotionAxis axis[2];
};

inline bool steadyRow(const FlightRow &r,uint16_t target) {
  return target>=1250 && r.target==target && abs((int)r.base-(int)target)<=5 &&
      r.requested[0]==0 && r.requested[1]==0 && r.requested[2]==0;
}

// Describes observed motion, not its cause. Fits/removes linear drift before
// counting hysteretic crossings. 20 Hz records cannot resolve fast vibration.
inline WobbleSummary summarizeWobble(const FlightRecorder &rec) {
  WobbleSummary out;
  if (!rec.count) return out;
  const size_t end=rec.count-1;
  const uint16_t target=rec.at(end).target;
  if (!steadyRow(rec.at(end),target)) return out;
  size_t first=end;
  while (first && steadyRow(rec.at(first-1),target) &&
      (uint32_t)(rec.at(first).ms-rec.at(first-1).ms)<=100) --first;
  const uint32_t settledStart=rec.at(first).ms;
  while (first<end && (uint32_t)(rec.at(first).ms-settledStart)<500) ++first;
  out.first=first; out.count=rec.count-first;
  out.seconds=(uint32_t)(rec.at(end).ms-rec.at(first).ms)*0.001f;
  if (out.count<20 || out.seconds<1) return out;
  out.valid=true; out.gMin=out.gMax=rec.at(first).g;
  float meanT=0;
  for (size_t i=first;i<rec.count;++i) {
    const auto &r=rec.at(i);
    meanT+=(uint32_t)(r.ms-rec.at(first).ms)*0.001f;
    out.axis[0].mean+=r.roll; out.axis[1].mean+=r.pitch;
    out.gMin=fminf(out.gMin,r.g); out.gMax=fmaxf(out.gMax,r.g);
    out.limited+=r.limited ? 1:0;
    for (unsigned j=0;j<2;++j) {
      out.axis[j].rateRms+=r.rates[j]*r.rates[j];
      out.axis[j].capped+=fabsf(r.integral[j])>=24.5f ? 1:0;
    }
  }
  meanT/=out.count;
  for (auto &a:out.axis) { a.mean/=out.count; a.rateRms=sqrtf(a.rateRms/out.count); }
  float tt=0;
  for (size_t i=first;i<rec.count;++i) {
    const auto &r=rec.at(i);
    const float t=(uint32_t)(r.ms-rec.at(first).ms)*0.001f-meanT;
    tt+=t*t;
    out.axis[0].slope+=t*(r.roll-out.axis[0].mean);
    out.axis[1].slope+=t*(r.pitch-out.axis[1].mean);
  }
  for (auto &a:out.axis) a.slope=tt>0 ? a.slope/tt:0;
  for (unsigned j=0;j<2;++j) {
    auto &a=out.axis[j];
    for (size_t i=first;i<rec.count;++i) {
      const auto &r=rec.at(i);
      const float t=(uint32_t)(r.ms-rec.at(first).ms)*0.001f-meanT;
      const float error=(j ? r.pitch:r.roll)-a.mean-a.slope*t;
      a.residualRms+=error*error;
    }
    a.residualRms=sqrtf(a.residualRms/out.count);
    const float threshold=fmaxf(0.5f,a.residualRms*0.25f);
    int side=0;
    float lastRise=0,periodSum=0,periodSquared=0;
    for (size_t i=first;i<rec.count;++i) {
      const auto &r=rec.at(i);
      const float time=(uint32_t)(r.ms-rec.at(first).ms)*0.001f;
      const float error=(j ? r.pitch:r.roll)-a.mean-a.slope*(time-meanT);
      if (error>=threshold) {
        if (side<0) {
          if (a.rises) {
            const float period=time-lastRise;
            periodSum+=period; periodSquared+=period*period;
          }
          ++a.rises; lastRise=time;
        }
        side=1;
      } else if (error<=-threshold) side=-1;
    }
    if (a.rises>=3 && periodSum>0) {
      const float periods=a.rises-1,mean=periodSum/periods;
      a.periodCv=sqrtf(fmaxf(0,periodSquared/periods-mean*mean))/mean;
      // Require at least two reasonably consistent periods. This is not a
      // spectrum estimate, and must not automatically change flight gains.
      if (a.periodCv<=0.3f) a.hz=1/mean;
    }
  }
  return out;
}
