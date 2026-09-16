#pragma once
#include <avr/wdt.h>
#include <avr/cpufunc.h>

// Enable after validating board power/reset behavior. Feed only after a full
// scheduler pass, never inside peripheral wait loops.
#ifndef SPEEDOMETER_WATCHDOG_ENABLED
#define SPEEDOMETER_WATCHDOG_ENABLED 0
#endif
namespace Watchdog {
inline void begin() {
#if SPEEDOMETER_WATCHDOG_ENABLED
  while (WDT.STATUS & WDT_SYNCBUSY_bm) {}
  _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_8KCLK_gc);
#endif
}
inline void feed() {
#if SPEEDOMETER_WATCHDOG_ENABLED
  wdt_reset();
#endif
}
}
