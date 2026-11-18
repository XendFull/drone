#pragma once
#include <stdint.h>

struct ControlPacket {
  uint32_t nonce=0, session=0, sequence=0, received=0;
  int action=0, throttle=1000, roll=0, pitch=0, yaw=0;
};

class ControlSession {
 public:
  enum Result { IGNORED, STOP, ARM, UPDATE, BLOCKED };
  static constexpr uint32_t LEASE_MS=500, QUEUE_MAX_MS=200;
  uint32_t nonce=1, owner=0, sequence=0, last=0;
  bool active=false;
  int throttle=1000, roll=0, pitch=0, yaw=0;

  void stop() {
    active=false; owner=sequence=0; throttle=1000; roll=pitch=yaw=0;
    nonce=nonce*1664525u+1013904223u;
    if (!nonce) nonce=1;
  }
  Result accept(const ControlPacket &p,uint32_t now,bool canArm) {
    if (p.action==0) return STOP; // Any connected operator can request stop.
    if ((uint32_t)(now-p.received)>QUEUE_MAX_MS || p.nonce!=nonce || !p.session)
      return IGNORED;
    if (p.throttle<1000 || p.throttle>1700 || p.roll<-8 || p.roll>8 ||
        p.pitch<-8 || p.pitch>8 || p.yaw<-30 || p.yaw>30) return IGNORED;
    if (p.action==1) {
      if (active) return IGNORED;
      if (!canArm || p.throttle!=1000 || p.roll || p.pitch || p.yaw) return BLOCKED;
      active=true; owner=p.session; sequence=p.sequence; last=now;
      throttle=1000; roll=pitch=yaw=0;
      return ARM;
    }
    if (p.action!=2 || !active || owner!=p.session || (int32_t)(p.sequence-sequence)<=0)
      return IGNORED;
    sequence=p.sequence; last=now;
    throttle=p.throttle; roll=p.roll; pitch=p.pitch; yaw=p.yaw;
    return UPDATE;
  }
  bool expired(uint32_t now) const { return active && (uint32_t)(now-last)>=LEASE_MS; }
};
