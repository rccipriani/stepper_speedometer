#pragma once
#include <Wire.h>
#include "BoardConfig.h"

#ifndef TWI_TIMEOUT_ENABLE
#error "Use DxCore Wire with TWI_TIMEOUT_ENABLE for bounded FRAM transactions"
#endif

// Same 16-bit-address I2C FRAM protocol as the prior Adafruit device.
// Latch any bus error. No automatic retry or reinitialization can overwrite
// an unreadable odometer. Recovery requires inspection and reset.
class FramDevice {
  bool ok = false;
public:
  bool begin() {
    Wire.beginTransmission(Board::framAddress);
    ok = Wire.endTransmission() == 0;
    return ok;
  }
  bool healthy() const { return ok; }
  uint8_t read(uint16_t address) {
    if (!ok) return 0xFF;
    Wire.beginTransmission(Board::framAddress);
    Wire.write(uint8_t(address >> 8)); Wire.write(uint8_t(address));
    if (Wire.endTransmission(false) != 0 ||
        Wire.requestFrom(Board::framAddress, uint8_t(1)) != 1) {
      ok = false; return 0xFF;
    }
    int value = Wire.read();
    if (value < 0) { ok = false; return 0xFF; }
    return uint8_t(value);
  }
  void write(uint16_t address, uint8_t value) {
    if (!ok) return;
    Wire.beginTransmission(Board::framAddress);
    Wire.write(uint8_t(address >> 8)); Wire.write(uint8_t(address));
    Wire.write(value);
    if (Wire.endTransmission() != 0) ok = false;
  }
};
