#pragma once
#include <stddef.h>
#include <stdint.h>

// Fixed RAM storage: no serial printing or allocation during a test.
struct BenchRecord {
  uint32_t ms;
  float roll, pitch, rates[3], accelG, integral[3];
  uint16_t base, target, calculated[4], sent[4];
  bool limited;
};
class BenchRecorder {
 public:
  static constexpr uint32_t PERIOD_MS=50;
  static constexpr size_t CAPACITY=302; // 15 s at 20 Hz, plus final snapshot.
  BenchRecord rows[CAPACITY];
  size_t count=0;
  void begin(uint32_t now) { start=last=now; count=0; }
  void capture(uint32_t now, BenchRecord row, bool final=false) {
    if (!final && count && (uint32_t)(now-last)<PERIOD_MS) return;
    row.ms=(uint32_t)(now-start);
    if (count && rows[count-1].ms==row.ms) rows[count-1]=row;
    else if (count<CAPACITY) rows[count++]=row;
    else if (final) rows[count-1]=row;
    last=now;
  }
 private:
  uint32_t start=0, last=0;
};
