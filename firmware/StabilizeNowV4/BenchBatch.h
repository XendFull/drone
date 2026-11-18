#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Cancellation tolerates a single transient USB status sample and uses
// continuous loss; a separate current sample still gates every restart.
class UsbPresence {
 public:
  static constexpr uint32_t LOSS_MS=500;
  void reset() { missing=false; }
  bool observe(uint32_t now,bool connected) {
    if (connected) { missing=false; return true; }
    if (!missing) { missing=true; missingSince=now; }
    return (uint32_t)(now-missingSince)<LOSS_MS;
  }
 private:
  bool missing=false;
  uint32_t missingSince=0;
};

class BenchBatch {
 public:
  static constexpr unsigned MAX_RUNS=3;
  static constexpr uint32_t OFF_GAP_MS=5000, MAX_GAP_MS=8000;
  static constexpr uint32_t STILL_MS=500;
  enum Next { WAIT, START, CANCELED };
  bool active=false,waiting=false;
  unsigned total=0,index=0;
  int pulse=1000;
  static bool parse(const char *line,int &us,unsigned &runs) {
    int p=0,c=1,n=-1;
    if (strncmp(line,"test",4) || line[4]!=' ') return false;
    if (sscanf(line,"test %4d %1d%n",&p,&c,&n)!=2 || n!=(int)strlen(line)) {
      n=-1; c=1;
      if (sscanf(line,"test %4d%n",&p,&n)!=1 || n!=(int)strlen(line)) return false;
    }
    if (p<1100 || p>1700 || c<1 || c>(int)MAX_RUNS) return false;
    us=p; runs=(unsigned)c; return true;
  }
  void begin(int us,unsigned runs) {
    pulse=us; total=runs; index=1; waiting=false;
    active=us>=1100 && us<=1700 && runs>=1 && runs<=MAX_RUNS;
  }
  // Longer schedules are available only through explicit automated search.
  void beginSearch(int us,unsigned runs) {
    pulse=us; total=runs; index=1; waiting=false;
    active=us>=1250 && us<=1400 && runs>=1 && runs<=16;
  }
  void cancel() { active=waiting=false; }
  void complete(uint32_t now) {
    if (!active) return;
    if (index>=total) { cancel(); return; }
    waiting=true; stoppedAt=now; healthySeen=false;
  }
  Next poll(uint32_t now,bool connected,bool healthy,bool reportFinished) {
    if (!active || !waiting) return WAIT;
    if (!connected) { cancel(); return CANCELED; }
    const uint32_t elapsed=now-stoppedAt;
    if (elapsed>MAX_GAP_MS) { cancel(); return CANCELED; }
    if (!healthy) { healthySeen=false; return WAIT; }
    if (!healthySeen) { healthySeen=true; healthySince=now; }
    if (elapsed<OFF_GAP_MS || (uint32_t)(now-healthySince)<STILL_MS || !reportFinished) return WAIT;
    ++index; waiting=false; return START;
  }
 private:
  uint32_t stoppedAt=0;
  uint32_t healthySince=0;
  bool healthySeen=false;
};
