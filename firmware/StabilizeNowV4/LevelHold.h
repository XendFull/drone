#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// One bounded, temporary lower pitch-P comparison against the starting settings.
// A short near-level observation is not PID acceptance or flight verification.
class LevelHold {
 public:
  static constexpr uint32_t DURATION_MS=7000;
  bool active=false,done=false;
  const char *reason="No level-hold check recorded";
  float baselinePitchP=0,trialPitchP=0;
  float mean[2]={},rms[2]={},rateRms[2]={},slope[2]={},lastIntegral[2]={};
  static bool parse(const char *line,int &pulse) {
    int us=0,n=-1;
    if (strncmp(line,"hold ",5) || sscanf(line,"hold %4d%n",&us,&n)!=1 ||
        n!=(int)strlen(line) || us<1250 || us>1400) return false;
    pulse=us; return true;
  }
  void begin(float pitchP) {
    *this=LevelHold{};
    if (!isfinite(pitchP) || pitchP<0.25f || pitchP>1.20f) {
      done=true; reason="BLOCKED: starting pitch P outside supported bounds"; return;
    }
    baselinePitchP=pitchP; trialPitchP=fmaxf(0.25f,0.50f*pitchP);
    active=true; reason="Seven-second temporary lower pitch-P trial running";
  }
  void sample(uint32_t ms,float roll,float pitch,float rollRate,float pitchRate,
      float rollIntegral,float pitchIntegral,float dt,bool clipped,bool integralLimited) {
    if (!active || ms<2500 || ms>=DURATION_MS) return;
    if (!isfinite(roll) || !isfinite(pitch) || !isfinite(rollRate) || !isfinite(pitchRate) ||
        !isfinite(rollIntegral) || !isfinite(pitchIntegral) || !isfinite(dt) || dt<=0 || dt>0.015f ||
        (haveStamp && ms<=lastStamp)) { bad=true; return; }
    haveStamp=true; lastStamp=ms; seconds+=dt; ++count;
    if (clipped) clipSeconds+=dt;
    if (integralLimited) capSeconds+=dt;
    lastIntegral[0]=rollIntegral; lastIntegral[1]=pitchIntegral;
    if (ms<6000) return;
    tailSeconds+=dt; ++tailCount;
    const float t=(ms-6000)*0.001f,angle[]={roll,pitch},rate[]={rollRate,pitchRate};
    sumT+=t*dt; sumTT+=t*t*dt;
    for (unsigned a=0;a<2;++a) {
      sum[a]+=angle[a]*dt; sum2[a]+=angle[a]*angle[a]*dt;
      sumRate2[a]+=rate[a]*rate[a]*dt; sumTA[a]+=t*angle[a]*dt;
    }
  }
  void finish(bool completed) {
    if (!active) return;
    active=false; done=true;
    const float denominator=tailSeconds*sumTT-sumT*sumT;
    if (tailSeconds>0) for (unsigned a=0;a<2;++a) {
      mean[a]=sum[a]/tailSeconds; rms[a]=sqrtf(sum2[a]/tailSeconds);
      rateRms[a]=sqrtf(sumRate2[a]/tailSeconds);
      if (denominator>0.000001f) slope[a]=(tailSeconds*sumTA[a]-sumT*sum[a])/denominator;
    }
    windowAvailable=!bad && tailCount>=120 && tailSeconds>=0.85f && tailSeconds<=1.15f &&
        denominator>0.000001f;
    for (unsigned a=0;a<2;++a) windowAvailable=windowAvailable && isfinite(mean[a]) &&
        isfinite(rms[a]) && isfinite(rateRms[a]) && isfinite(slope[a]);
    if (!completed) reason="CANCELED: no level-hold conclusion";
    else if (bad || count<700 || seconds<4.1f || seconds>4.9f || tailCount<120 ||
        tailSeconds<0.85f || tailSeconds>1.15f || denominator<=0.000001f)
      reason="INCONCLUSIVE: insufficient fresh sensor data";
    else if (clipSeconds>0.01f*seconds || capSeconds>0.01f*seconds)
      reason="CORRECTION LIMITED: no near-level conclusion; retain current gains";
    else {
      bool level=true;
      for (unsigned a=0;a<2;++a)
        level=level && isfinite(mean[a]) && isfinite(rms[a]) && isfinite(rateRms[a]) && isfinite(slope[a]) &&
            fabsf(mean[a])<=2 && rms[a]<=2.5f && rateRms[a]<=12 && fabsf(slope[a])<=1.5f;
      reason=level?"NEAR-LEVEL WINDOW measured with temporary P; verification still required":
          "LEVEL HOLD NOT ESTABLISHED: residual tilt, motion or drift remains";
    }
  }
  int format(char *out,size_t size) const {
    // An early stop may never reach the final 6000..7000 ms window. Default
    // zero-initialized statistics are not measurements of a level airframe.
    if (!done || !windowAvailable) {
      if (!count) return snprintf(out,size,
          "LEVEL HOLD: %s\n"
          "TEMPORARY pitch P %.3f -> %.3f; roll gains and pitch I/D unchanged.\n"
          "Last 1 s statistics UNAVAILABLE: no usable evaluation window collected.\n"
          "No settled-window integral measurement available. No new gains accepted or saved.\n"
          "Temporary P restored after STOP. This does not verify free flight or position hold.\n",
          reason,baselinePitchP,trialPitchP);
      return snprintf(out,size,
          "LEVEL HOLD: %s\n"
          "TEMPORARY pitch P %.3f -> %.3f; roll gains and pitch I/D unchanged.\n"
          "Last 1 s statistics UNAVAILABLE: no complete usable 6000..7000 ms evaluation window collected.\n"
          "Last sampled time=%lu ms; last measured integral R/P=%.2f/%.2f us.\n"
          "Zero-angle control used normal bounded integral correction; no forced pulses.\n"
          "No new gains accepted or saved. Temporary P restored after STOP.\n"
          "This does not verify free flight or position hold.\n",
          reason,baselinePitchP,trialPitchP,(unsigned long)lastStamp,lastIntegral[0],lastIntegral[1]);
    }
    return snprintf(out,size,
        "LEVEL HOLD: %s\n"
        "TEMPORARY pitch P %.3f -> %.3f; roll gains and pitch I/D unchanged.\n"
        "7 s stabilization; roll/pitch targets ZERO, no forced pulses; integral ENABLED within existing bounds.\n"
        "Last 1 s R mean/RMS/rate/drift=%.2f/%.2f/%.2f/%.2f; P=%.2f/%.2f/%.2f/%.2f.\n"
        "Screen limits: abs mean 2 deg, RMS 2.5 deg, rate RMS 12 deg/s, abs drift 1.5 deg/s.\n"
        "Last measured integral R/P=%.2f/%.2f us; no new gains accepted or saved.\n"
        "Temporary P restored after STOP.\n"
        "Fixture forces or blocked axes affect the result. This does not verify free flight or position hold.\n",
        reason,baselinePitchP,trialPitchP,mean[0],rms[0],rateRms[0],slope[0],mean[1],rms[1],rateRms[1],slope[1],lastIntegral[0],lastIntegral[1]);
  }
 private:
  unsigned count=0,tailCount=0;
  float seconds=0,tailSeconds=0,clipSeconds=0,capSeconds=0,sumT=0,sumTT=0;
  float sum[2]={},sum2[2]={},sumRate2[2]={},sumTA[2]={};
  uint32_t lastStamp=0;
  bool haveStamp=false,bad=false;
  bool windowAvailable=false;
};
