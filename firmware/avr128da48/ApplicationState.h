#pragma once
// AVR128DA48 port of Robert Cipriani's v1.12. See README.md.
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "BoardConfig.h"
#include "FramDevice.h"
#include "Speedometer.h"
#include "Watchdog.h"
#include <SwitecX25.h>
#include <U8g2lib.h>
#include <util/atomic.h>
#include <avr/interrupt.h>
#include <stddef.h>

#ifndef U8G2_16BIT
#error "Enable U8G2_16BIT in the U8g2 library for the 256-pixel-wide OLED"
#endif

#if !defined(__AVR_AVR128DA48__)
#error "Select DxCore AVR128DA48"
#endif

#define DEBUG_SERIAL 0
const char VERSION[] = "DA 1.0";
using namespace Board;
bool calibrationInputsValid = false;
bool encoderButtonReading = HIGH;
constexpr float RpmPulsesPerRevolution = 4.0f;
// Source switches: leave disabled while these harness leads are unconnected.
#ifndef AFR_INPUT_ENABLED
#define AFR_INPUT_ENABLED 0
#endif
#ifndef DIMMER_INPUT_ENABLED
#define DIMMER_INPUT_ENABLED 0
#endif

const uint8_t DisplayModeCount = 6;
const uint16_t AnalogUpdateMs = 100;
// DEFAULT ADC reference is the AVR DA VDD (explicit 10-bit ADC mode); set this to its measured voltage.
constexpr float AnalogReferenceVolts = 5.0f;
constexpr float AfrVoltageLow = 0.0f, AfrVoltageHigh = 5.0f;
// REQUIRED: replace both zero placeholders with the wideband controller's
// specified AFR values at AfrVoltageLow/High before enabling AFR input.
constexpr float AfrAtLowVoltage = 0.0f, AfrAtHighVoltage = 0.0f;
#if AFR_INPUT_ENABLED
static_assert(AnalogReferenceVolts > 0 && AfrVoltageLow >= 0 &&
  AfrVoltageHigh > AfrVoltageLow && AfrVoltageHigh <= AnalogReferenceVolts,
  "Configure AFR voltage range and measured ADC reference");
static_assert(AfrAtLowVoltage > 0 && AfrAtHighVoltage > 0 &&
  AfrAtLowVoltage != AfrAtHighVoltage, "Configure the controller AFR transfer curve first");
#endif
// The dimmer expects a DC level from the external conditioning/filter circuit. These
// calibration endpoints may be reversed for an inverted dashboard signal.
constexpr int16_t DimmerAdcDark = 0, DimmerAdcBright = 1023;
constexpr uint8_t DimmerContrastMin = 16, DimmerContrastMax = 255;
static_assert(DimmerAdcDark >= 0 && DimmerAdcDark <= 1023 &&
  DimmerAdcBright >= 0 && DimmerAdcBright <= 1023 && DimmerAdcDark != DimmerAdcBright,
  "Dimmer endpoints must be distinct ADC counts in 0..1023");
static_assert(DimmerContrastMin <= DimmerContrastMax, "Invalid OLED contrast range");
float afr = 0;
bool afrValid = false;
uint32_t lastAnalogUpdate = 0;
uint16_t dimmerFilteredAdc = 0;
bool dimmerInitialized = false;
uint8_t lastDimmerContrast = DimmerContrastMax;

const uint16_t MaxMotorSteps = 510;
const float StepsPerMPH = 5.1f;
const uint32_t PulsesPerMile = 4000;
// Distance units are 1/(PulsesPerMile * 1000) mile. Each pulse adds ratioMilli.
const uint32_t UnitsPerTenth = PulsesPerMile * 100UL;
const uint32_t StopTimeoutUs = 3000000UL; // supports ~0.3 MPH at ratio 1
const uint16_t CheckpointMs = 100, SpeedUpdateMs = 100, DisplayUpdateMs = 200;
const uint16_t DebounceMs = 35, LongPressMs = 1000;
const int VIEW_X = 5, VIEW_Y = 38, VIEW_W = 130, VIEW_H = 24;
const uint16_t OLED_WIDTH = 256, OLED_HEIGHT = 64;
const uint16_t LayoutSaveMs = 1000, LayoutHintMs = 1200;

// Fonts live in flash; numeric subsets keep the Nano build small. Index 2 is
// v1.07's original font. Names contain point sizes, not measured pixel heights.
const uint8_t * const DigitFonts[] PROGMEM = {
  u8g2_font_luBS08_tn, u8g2_font_logisoso16_tn, u8g2_font_luBS12_tn,
  u8g2_font_logisoso18_tn, u8g2_font_logisoso20_tn, u8g2_font_logisoso22_tn,
  u8g2_font_logisoso24_tn, u8g2_font_logisoso26_tn, u8g2_font_logisoso28_tn,
  u8g2_font_logisoso32_tn, u8g2_font_logisoso38_tn, u8g2_font_logisoso42_tn
};
const uint8_t DigitFontCount = sizeof(DigitFonts) / sizeof(DigitFonts[0]);
const uint8_t DefaultDigitFont = 2;

// One-page buffer saves RAM. All screens must use firstPage/nextPage.
U8G2_SH1122_256X64_1_4W_HW_SPI display(U8G2_R0, oledCs, oledDc, oledReset);
// Switec counts positions 0..steps-1; allow the requested endpoint 510.
Speedometer speedometer;
FramDevice fram;
bool storageFailed = false;
bool mileageKnown = false;

#include "StorageLayout.h"
StoredState state;
uint8_t activeSlot = 1;
bool storageDirty = false;
uint32_t lastCheckpoint = 0, lastSpeedUpdate = 0, lastDisplayUpdate = 0;
float mph = 0;
uint16_t motorStep = 0;
uint8_t displayMode = 0;
enum CalibrationTarget : uint8_t { CAL_RATIO, CAL_OFFSET, CAL_X, CAL_Y, CAL_SIZE, CAL_DIGITS, CAL_COUNT };
CalibrationTarget calibrationTarget = CAL_RATIO;

DisplayLayout layout;
uint8_t activeLayoutSlot = 1;
bool layoutDirty = false, layoutInfo = false, layoutHint = false;
bool harnessWasPresent = false;
uint32_t lastLayoutChange = 0, layoutHintStarted = 0;


uint32_t consumedPulses = 0;
uint8_t encoderPrevious = 0;
int8_t encoderQuarterSteps = 0, encoderPending = 0;

struct Button {
  bool lastReading;
  bool stable;
  bool longHandled;
  uint32_t changedAt;
  uint32_t pressedAt;
};
Button encoderButton = {HIGH, HIGH, false, 0, 0};
Button modeButton = {HIGH, HIGH, false, 0, 0};

