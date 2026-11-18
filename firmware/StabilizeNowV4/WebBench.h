#pragma once
#include <stdio.h>
#include <string.h>
#include "TuneSettings.h"
#include "FlightMath.h"

// One atomic record includes the existing R/P settings and online yaw/level tuning.
struct OnlineSettings {
  uint32_t signature=0x514F4E32;
  TuneSettings pid;
  AutoCompare::Gains yaw{0.5f,flight::kYawBalanceKi,0};
  float yawCap=flight::kYawIntegralLimit,angleGain=3;
  float rollTrim=0,pitchTrim=0; // us per motor at base 1400; not an attitude offset.
  uint32_t checksum=0;
  void seal(){pid.seal();checksum=TuneSettings::hashBytes(this,offsetof(OnlineSettings,checksum));}
  bool valid() const {
    return signature==0x514F4E32 && pid.valid() &&
      isfinite(yaw.p)&&yaw.p>=0.1f&&yaw.p<=2 && isfinite(yaw.i)&&yaw.i>=0&&yaw.i<=0.5f &&
      isfinite(yaw.d)&&yaw.d>=0&&yaw.d<=0.006f &&
      isfinite(yawCap)&&yawCap>=0&&yawCap<=30 && isfinite(angleGain)&&angleGain>=1&&angleGain<=5 &&
      isfinite(rollTrim)&&fabsf(rollTrim)<=40 && isfinite(pitchTrim)&&fabsf(pitchTrim)<=40 &&
      checksum==TuneSettings::hashBytes(this,offsetof(OnlineSettings,checksum));
  }
  struct Legacy {
    uint32_t signature;TuneSettings pid;AutoCompare::Gains yaw;
    float yawCap,angleGain;uint32_t checksum;
  };
  bool importLegacy(const Legacy &old){
    if(old.signature!=0x514F4E31||old.checksum!=TuneSettings::hashBytes(&old,offsetof(Legacy,checksum)))return false;
    OnlineSettings next;next.pid=old.pid;next.yaw=old.yaw;next.yawCap=old.yawCap;next.angleGain=old.angleGain;
    next.seal();if(!old.pid.valid()||!next.valid())return false;*this=next;return true;
  }
};
static_assert(sizeof(OnlineSettings)==76,"Online settings layout changed");
static_assert(sizeof(OnlineSettings::Legacy)==68,"Earlier online settings layout changed");

struct OnlinePacket {
  enum Kind {APPLY=1,SAVE,CALIBRATE,DEFAULTS,LEVEL_TEST,EQUAL_TEST,ROLL_RESPONSE,PITCH_RESPONSE,HOLD_TEST};
  uint32_t nonce=0,session=0,sequence=0,received=0;
  int kind=0,pulse=0,seconds=0;
  OnlineSettings values;
  bool validStart() const {
    if(kind==LEVEL_TEST)return pulse>=1100&&pulse<=1400&&seconds>=3&&seconds<=7;
    if(kind==EQUAL_TEST)return pulse==1250&&seconds==3;
    return kind>=ROLL_RESPONSE&&kind<=HOLD_TEST&&pulse>=1250&&pulse<=1400&&seconds==7;
  }
  static bool parse(const char *body,OnlinePacket &out){
    OnlinePacket next;unsigned long n=0,s=0,q=0;int used=-1;
    if(sscanf(body,"%lu %lu %lu %d %d %d%n",&n,&s,&q,&next.kind,&next.pulse,&next.seconds,&used)!=6 || used<0)return false;
    next.nonce=n;next.session=s;next.sequence=q;
    if(!next.nonce||!next.session||!next.sequence)return false;
    if(next.kind==APPLY){
      auto &v=next.values;int tail=-1;
      if(sscanf(body+used," %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f%n",
        &v.pid.roll.p,&v.pid.roll.i,&v.pid.roll.d,&v.pid.pitch.p,&v.pid.pitch.i,&v.pid.pitch.d,
        &v.yaw.p,&v.yaw.i,&v.yaw.d,&v.pid.rollCap,&v.pid.pitchCap,&v.yawCap,&v.angleGain,&v.rollTrim,&v.pitchTrim,&tail)!=15 ||
        tail<0 || body[used+tail])return false;
      v.seal();if(!v.valid()||next.pulse||next.seconds)return false;
    } else {
      if(body[used])return false;
      if(next.kind>=SAVE && next.kind<=DEFAULTS){if(next.pulse||next.seconds)return false;}
      else if(!next.validStart())return false;
    }
    out=next;return true;
  }
};
