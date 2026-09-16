void applyLayoutAdjustment(int8_t delta, uint32_t now) {
  switch (calibrationTarget) {
    case CAL_X: layout.x = constrain(layout.x + delta, 0, (int16_t)OLED_WIDTH - 1); break;
    case CAL_Y: layout.y = constrain(layout.y + delta, 0, (int16_t)OLED_HEIGHT - 1); break;
    case CAL_SIZE: layout.font = constrain((int16_t)layout.font + delta, 0, DigitFontCount - 1); break;
    case CAL_DIGITS: layout.digits = constrain((int16_t)layout.digits + delta, 1, 40); break;
    default: return;
  }
  layoutDirty = true;
  lastLayoutChange = now;
  layoutHint = false; // rotation reveals the result immediately
}

// Extracted from v1.12; included once by Application.cpp.
// Returns 1 on short release, 2 once while held for LongPressMs.
uint8_t pollButtonReading(Button &button, bool reading, uint32_t now) {
  if (reading != button.lastReading) {
    button.lastReading = reading;
    button.changedAt = now;
  }
  if (now - button.changedAt >= DebounceMs && reading != button.stable) {
    button.stable = reading;
    if (!reading) {
      button.pressedAt = now;
      button.longHandled = false;
    } else if (!button.longHandled) {
      return now - button.pressedAt >= LongPressMs ? 2 : 1;
    }
  }
  if (!button.stable && !button.longHandled && now - button.pressedAt >= LongPressMs) {
    button.longHandled = true;
    return 2;
  }
  return 0;
}

uint8_t pollButton(Button &button, uint8_t pin, uint32_t now) {
  return pollButtonReading(button, digitalRead(pin), now);
}

void handleControls(uint32_t now) {
  bool calibration = digitalRead(calSwitchPin) == LOW;
  serviceCalibrationInputs();
  if (calibration != harnessWasPresent) {
    if (!calibration) saveLayout();
    calibrationTarget = CAL_RATIO;
    layoutInfo = layoutHint = false;
    harnessWasPresent = calibration;
  }
  int8_t delta;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    delta = encoderPending;
    encoderPending = 0;
    if (!calibration) encoderQuarterSteps = 0;
  }
  if (calibration && delta) {
    // Account for all preceding pulses using the preceding ratio.
    collectDistance();
    if (calibrationTarget == CAL_OFFSET) {
      state.offset = constrain(state.offset + delta, -60, 60);
      storageDirty = true;
    } else if (calibrationTarget == CAL_RATIO) {
      state.ratioMilli = constrain((int16_t)state.ratioMilli + delta, 500, 2000);
      storageDirty = true;
    } else {
      applyLayoutAdjustment(delta, now);
    }
  }
  uint8_t encoderEvent = calibrationInputsValid ?
    pollButtonReading(encoderButton, encoderButtonReading, now) : 0;
  if (calibration && encoderEvent == 1) {
    saveLayout();
    calibrationTarget = (CalibrationTarget)((calibrationTarget + 1) % CAL_COUNT);
    layoutInfo = false;
    layoutHint = calibrationTarget >= CAL_X;
    layoutHintStarted = now;
  }
  if (calibration && encoderEvent == 2 && calibrationTarget >= CAL_X) {
    layoutInfo = !layoutInfo;
    layoutHint = false;
  }
  uint8_t modeEvent = pollButton(modeButton, modeButtonPin, now);
  if (!calibration && modeEvent == 1) displayMode = (displayMode + 1) % DisplayModeCount;
  if (!calibration && modeEvent == 2 && displayMode == 1) {
    collectDistance();
    state.tripTenths = 0;
    state.tripFraction = 0;
    storageDirty = true;
    saveState();
  }
  if (layoutDirty && now - lastLayoutChange >= LayoutSaveMs) saveLayout();
  if (layoutHint && now - layoutHintStarted >= LayoutHintMs) layoutHint = false;
}

