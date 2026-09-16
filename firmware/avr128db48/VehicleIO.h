// Extracted from v1.12; included once by Application.cpp.
// Complete cycles reject contact bounce; impossible two-bit jumps discard
// partial motion. Table is in flash to preserve Nano SRAM.
int8_t decodeTransition(uint8_t previous, uint8_t current) {
  static const int8_t table[16] PROGMEM = {
    0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0
  };
  return (int8_t)pgm_read_byte(&table[(previous << 2) | current]);
}

void onEncoderChange(uint8_t current) {
  if (digitalRead(calSwitchPin) == HIGH || ((current ^ encoderPrevious) == 3)) {
    encoderQuarterSteps = 0;
  } else {
    encoderQuarterSteps += decodeTransition(encoderPrevious, current);
    if (encoderQuarterSteps >= 4) {
      if (encoderPending < 100) ++encoderPending;
      encoderQuarterSteps = 0;
    } else if (encoderQuarterSteps <= -4) {
      if (encoderPending > -100) --encoderPending;
      encoderQuarterSteps = 0;
    }
  }
  encoderPrevious = current;
}

uint8_t readEncoderState() {
  return (digitalRead(encoderA) ? 2 : 0) | (digitalRead(encoderB) ? 1 : 0);
}
void serviceCalibrationInputs() {
  if (digitalRead(calSwitchPin) == HIGH) {
    calibrationInputsValid = false;
    encoderPending = encoderQuarterSteps = 0;
    return;
  }
  encoderButtonReading = digitalRead(encoderKey);
  uint8_t current = readEncoderState();
  if (!calibrationInputsValid) {
    encoderPrevious = current;
    encoderPending = encoderQuarterSteps = 0;
    uint32_t now = millis();
    encoderButton = {encoderButtonReading, encoderButtonReading, true, now, now};
    calibrationInputsValid = true;
  } else onEncoderChange(current);
}


void beginVehicleIO() {
  const uint8_t inputs[] = {speedPulsePin, maxSecondChannelPin, calSwitchPin,
       modeButtonPin, rpmPulsePin, encoderA, encoderB, encoderKey};
  for (uint8_t pin : inputs)
    pinMode(pin, INPUT_PULLUP);
  analogReadResolution(10);
  analogReference(VDD);
}

void serviceFastInputs() {
  speedometer.update();
  serviceCalibrationInputs(); // main context only, between I2C transactions
}

