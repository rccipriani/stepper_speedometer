// Extracted from v1.12; included once by Application.cpp.
uint16_t readAnalogAverage(uint8_t pin) {
  analogRead(pin); // discard first conversion after ADC channel switching
  uint16_t total = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    total += analogRead(pin);
    serviceFastInputs();
  }
  return (total + 2) / 4;
}

uint8_t contrastFromAdc(uint16_t adc) {
  int32_t contrast = DimmerContrastMin +
    ((int32_t)adc - DimmerAdcDark) * (DimmerContrastMax - DimmerContrastMin) /
    (DimmerAdcBright - DimmerAdcDark);
  return constrain(contrast, DimmerContrastMin, DimmerContrastMax);
}

void updateAnalogInputs(uint32_t now) {
  if (now - lastAnalogUpdate < AnalogUpdateMs) return;
  lastAnalogUpdate = now;
#if AFR_INPUT_ENABLED
  float volts = readAnalogAverage(afrInputPin) * (AnalogReferenceVolts / 1023.0f);
  afrValid = volts >= AfrVoltageLow && volts <= AfrVoltageHigh;
  if (afrValid) afr = AfrAtLowVoltage + (volts - AfrVoltageLow) *
    (AfrAtHighVoltage - AfrAtLowVoltage) / (AfrVoltageHigh - AfrVoltageLow);
  // Controller warm-up/fault voltages need its specific documented treatment;
  // an in-range analog voltage alone cannot establish sensor readiness.
#endif
#if DIMMER_INPUT_ENABLED
  uint16_t raw = readAnalogAverage(dimmerInputPin);
  dimmerFilteredAdc = dimmerInitialized ? (dimmerFilteredAdc * 3U + raw + 2U) / 4U : raw;
  dimmerInitialized = true;
  uint8_t contrast = contrastFromAdc(dimmerFilteredAdc);
  if (contrast != lastDimmerContrast) {
    display.setContrast(contrast);
    lastDimmerContrast = contrast;
  }
#endif
}

