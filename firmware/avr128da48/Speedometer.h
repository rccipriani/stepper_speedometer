#pragma once
#include <SwitecX25.h>
#include "BoardConfig.h"

// Preserve the v1.12 mapping and the upstream Switec sequence/acceleration.
// The 510-step homing travel matches Switec.zero() with steps=511.
class Speedometer {
  SwitecX25 motor{511, Board::motor1, Board::motor2, Board::motor3, Board::motor4};
  enum Phase : uint8_t { Running, Homing, SweepUp, HoldUp, SweepDown, HoldDown };
  Phase phase = Running;
  uint32_t since = 0;
  uint16_t target = 0;
public:
  void home() {
    motor.currentStep = 510;
    motor.targetStep = 0; motor.vel = 0; motor.dir = 0; motor.stopped = true;
    since = micros(); phase = Homing;
  }
  bool ready() const { return phase == Running; }
  uint16_t targetPosition() const { return target; }
  void setSpeed(float mph, int16_t offset = 0) {
    float steps = mph * 5.1f + offset;
    target = steps <= 0 ? 0 : steps >= 510 ? 510 : uint16_t(steps + 0.5f);
    if (ready()) motor.setPosition(target);
  }
  void update() {
    if (phase == Homing) {
      uint32_t now = micros();
      if (uint32_t(now - since) < 800) return;
      since = now; // Never burst missed steps after a slow peripheral operation.
      motor.stepDown();
      if (motor.currentStep == 0) {
        motor.setPosition(510); phase = SweepUp;
      }
      return;
    }
    motor.update();
    uint32_t now = millis();
    if (phase == SweepUp && motor.stopped) { phase = HoldUp; since = now; }
    else if (phase == HoldUp && uint32_t(now - since) >= 1000) {
      motor.setPosition(0); phase = SweepDown;
    } else if (phase == SweepDown && motor.stopped) { phase = HoldDown; since = now; }
    else if (phase == HoldDown && uint32_t(now - since) >= 1000) {
      phase = Running; motor.setPosition(target);
    }
  }
};
