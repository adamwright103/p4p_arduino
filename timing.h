// timing.h
// Fixed-rate, non-blocking timer built on micros(). Safe across the 70-minute
// micros() overflow. Use one per periodic task instead of delay().
#pragma once
#include <Arduino.h>

class RateTimer {
 public:
  explicit RateTimer(uint32_t period_us)
      : period_us_(period_us), next_us_(micros()) {}

  // True once per period. If the loop falls behind, fires once per call
  // until caught up; if it falls far behind, resynchronises instead of
  // firing in a burst.
  bool due() {
    uint32_t now = micros();
    if ((int32_t)(now - next_us_) < 0) return false;
    next_us_ += period_us_;
    if ((int32_t)(now - next_us_) > (int32_t)period_us_) next_us_ = now;
    return true;
  }

  void restart() { next_us_ = micros(); }
  uint32_t periodUs() const { return period_us_; }

 private:
  uint32_t period_us_;
  uint32_t next_us_;
};
