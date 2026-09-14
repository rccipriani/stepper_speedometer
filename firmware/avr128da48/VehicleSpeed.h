#pragma once
#include <Arduino.h>
#include <util/atomic.h>
#include "BoardConfig.h"
// Backend boundary: an EVSYS/TCB replacement must preserve pulse totals,
// first-edge invalidity, stop aging and atomic snapshot semantics.
class VehicleSpeed {
  volatile uint32_t vssPulses = 0, vssLastUs = 0, vssPeriodUs = 0;
  volatile bool vssSeen = false;
public:
  static constexpr uint32_t PulsesPerMile = 4000, StopTimeoutUs = 3000000UL;
  void begin(void (*handler)()) {
    pinMode(Board::speedPulsePin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Board::speedPulsePin), handler, RISING);
  }
  void onPulse() {
    uint32_t now = micros();
    uint32_t elapsed = now - vssLastUs;
    vssPeriodUs = vssSeen && elapsed <= StopTimeoutUs ? elapsed : 0;
    vssLastUs = now; vssSeen = true; ++vssPulses;
  }
  uint32_t totalPulses() const {
    uint32_t total;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { total = vssPulses; }
    return total;
  }
  float getMph(uint16_t ratioMilli = 1000) {
    uint32_t last, period, now;
    bool seen;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      last = vssLastUs;
      period = vssPeriodUs;
      seen = vssSeen;
      now = micros();
      // Clear stale state so micros() rollover cannot revive an old speed.
      if (seen && now - last > StopTimeoutUs) {
        vssSeen = false;
        vssPeriodUs = 0;
        seen = false;
      }
    }
    // As pulses slow/stop, age the period until the next edge or zero timeout.
    bool validPeriod = period != 0;
    uint32_t age = now - last;
    if (age > period) period = age;
    return seen && validPeriod ? (3600000000.0f / PulsesPerMile) / period *
      (ratioMilli / 1000.0f) : 0.0f;
  }
};
