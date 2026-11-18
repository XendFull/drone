#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "AutoCompare.h"

struct TuneSettings {
  uint32_t signature=0x51545032;
  AutoCompare::Gains roll{0.50f,0.12f,0.002f},pitch{0.70f,0.12f,0.002f};
  float rollCap=25,pitchCap=25;
  uint32_t checksum=0;
  static uint32_t hashBytes(const void *data,size_t size) {
    const auto *p=static_cast<const uint8_t *>(data); uint32_t h=2166136261u;
    for (size_t j=0;j<size;++j) h=(h^p[j])*16777619u;
    return h;
  }
  void seal() { checksum=hashBytes(this,offsetof(TuneSettings,checksum)); }
  bool valid() const {
    return signature==0x51545032 && checksum==hashBytes(this,offsetof(TuneSettings,checksum)) &&
        AutoCompare::validGains(roll) && AutoCompare::validGains(pitch) &&
        isfinite(rollCap) && rollCap>=25 && rollCap<=60 &&
        isfinite(pitchCap) && pitchCap>=25 && pitchCap<=60;
  }
  bool acceptVerified(const AutoCompare &t) {
    if (!t.verified()) return false;
    TuneSettings next=*this;
    next.roll=t.selectedGains(AutoCompare::ROLL); next.pitch=t.selectedGains(AutoCompare::PITCH);
    next.seal(); if (!next.valid()) return false;
    *this=next; return true;
  }
  struct Legacy {
    uint32_t signature;
    float rollP,pitchI,rollCap,pitchCap;
    uint32_t checksum;
  };
  bool importLegacy(const Legacy &old) {
    if (old.signature!=0x51544131 || old.checksum!=hashBytes(&old,offsetof(Legacy,checksum)) ||
        !isfinite(old.rollP) || old.rollP<0.30f || old.rollP>0.60f ||
        !isfinite(old.pitchI) || old.pitchI<0.04f || old.pitchI>0.20f) return false;
    TuneSettings next; next.roll.p=old.rollP; next.pitch.i=old.pitchI;
    next.rollCap=old.rollCap; next.pitchCap=old.pitchCap; next.seal();
    if (!next.valid()) return false;
    *this=next; return true;
  }
};
static_assert(sizeof(TuneSettings)==40,"Persistent PID record layout changed");
static_assert(sizeof(TuneSettings::Legacy)==24,"Legacy record layout changed");
