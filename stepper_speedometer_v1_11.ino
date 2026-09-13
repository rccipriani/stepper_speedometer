/*
 * Stepper Speedometer v1.11 -- Robert Cipriani -- 2026-09-13
 * File: stepper_speedometer_v1_11.ino
 * Target: classic Arduino Nano / ATmega328P, 16 MHz, 2 KB SRAM.
 *
 * v1.11: A6 allocated to a wideband controller's conditioned analog AFR output;
 * A7 allocated to a conditioned, FILTERED analog dashboard brightness input.
 * Adds AFR mode. Both analog features default disabled until wired/configured.
 * AFR shows -- when disabled or out of range. No guessed AFR transfer curve.
 * A6/A7 are analog-only: no digital input, pull-up or PWM edge capture.
 *
 * v1.10: reserve D17/A3 for future conditioned RPM input. Source-configurable
 * pulses/revolution defaults to 4.0 for conventional V8 GM HEI. RPM capture
 * remains unimplemented; the RPM screen continues to show --.
 *
 * v1.09: enforce saved capacity and OLED bounds before numeric rendering.
 * ODO requires room for all eight cells of 999999.9, even at low mileage.
 * An invalid fit shows FIT (or ! in a narrow space), never partial mileage.
 * Calibration readout adds FIT8 if the layout cannot show the full odometer.
 * Preview remains unobstructed so undersized layouts can still be adjusted.
 * FRAM formats are unchanged; mileage continues to count and save normally.
 *
 * v1.08: harness adds display X, baseline Y, numeric font and digit count.
 * Repeating 0123456789 draws exactly the selected count without line wrapping.
 * The physical speedometer window is the mask; no guessed software window is
 * imposed on the test digits. Selected layout also applies to normal numbers.
 * Display settings use a separate CRC journal at 0x100/0x120. Existing mileage
 * records remain byte-compatible with v1.07. Defaults preserve its X/Y/font.
 *
 * v1.07: D2 interrupt pulse counting/period measurement replaces FreqMeasure;
 * distance is independent of needle limits and display refresh. Integer distance
 * fractions and settings are journaled in two CRC-protected FRAM records.
 * EC11 quadrature uses pin-change interrupts; OLED uses U8g2 page rendering.
 * Debug output is off; startup uses the existing small font to save flash.
 *
 * Wiring (unchanged from the v1.06 harness):
 * D2: conditioned VSS, RISING edge, 4000 pulses/mile before calibration.
 * D3: encoder S1/A, D12: S2/B, A1: KEY (active low).
 * A0: calibration harness detect, grounded when plugged in.
 * A2: mode/trip button to ground.
 * D17/A3: reserved conditioned RPM input (0-5V logic), idle pull-up enabled.
 * HEI TACH/coil negative requires a suitable inline coil-negative conditioner
 * BEFORE connecting to D17/A3; leave that harness lead insulated until then.
 * A3 is no longer available for a future dimmer/AFR input.
 * A6: protected 0-5V analog AFR signal from a wideband controller (not sensor).
 * A7: protected 0-5V DC brightness level. Vehicle PWM must be level-protected
 * AND low-pass filtered before A7; never connect dashboard PWM directly.
 * D4-D7: X25/X27 motor. OLED SPI: D11 MOSI, D13 SCK, D10 CS, D9 DC,
 * D8 RESET. FRAM: A4 SDA, A5 SCL, address 0x50. Encoder power: 5V/GND.
 * VSS must come through the Mini MAX A2 or equivalent signal conditioner;
 * never connect vehicle voltage or raw VR signals directly to the Nano.
 *
 * Turn encoder to adjust ratio (0.500..2.000) or offset (-60..60 steps).
 * Short encoder press cycles RATIO -> OFFSET -> X -> Y -> SIZE -> DIGITS -> RATIO.
 * X/Y move one pixel per pulse. SIZE selects a readable numeric font.
 * DIGITS sets 1..40 test digits, recording the operator-observed capacity.
 * In display modes, long encoder press toggles settings readout / test digits;
 * it never advances the target. New display modes show a brief mode hint.
 * Layout autosaves after 1 second without adjustment, on target change or
 * harness removal. Main mode/trip button is ignored while harness is present.
 * Without harness, short mode press cycles ODO/TRIP/MPH/RATIO/RPM/AFR.
 * Long mode press in TRIP resets trip AND its fractional remainder.
 * One adjustment per complete quadrature cycle (normally one EC11 pulse).
 * Swap S1/S2 if the desired rotation direction is reversed.
 *
 * FRAM layout: legacy v1.06 bytes 0x00..0x31 are read only for migration.
 * New records: 0x80..0xA0 and 0xC0..0xE0, schema 1, 33 bytes each.
 * First boot imports valid legacy digits, ratio and offset; old bytes are
 * left intact. Downgrading to v1.06 will show those OLD mileage/settings.
 * Commit every 100 ms while dirty (plus write/service time). A power cut can lose the uncommitted
 * interval, but cannot turn a partially written record into a valid update.
 * No software can save pulses after power has failed; hold-up power is needed
 * if losing even the last checkpoint interval is unacceptable.
 * If both records are damaged after migration, halt rather than restore stale
 * legacy mileage. FRAM failure also halts; inspect hardware before recovery.
 *
 * Credits retained from earlier versions:
 * Luke Hurst: https://retromini.weebly.com/blog/arduino-speedometer
 * Kevin Gale / Walter Clark: https://github.com/Walterclark1/Stepper_Speedometer
 * Guy Carpenter: https://github.com/clearwater/SwitecX25
 * PJRC (earlier FreqMeasure implementation): https://www.pjrc.com/
 * Trewjohn2001: http://www.mgexp.com/phorum/read.php?40,2761694
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_FRAM_I2C.h>
#include <SwitecX25.h>
#include <U8g2lib.h>
#include <util/atomic.h>
#include <avr/interrupt.h>
#include <stddef.h>

#ifndef U8G2_16BIT
#error "Enable U8G2_16BIT in the U8g2 library for the 256-pixel-wide OLED"
#endif

#if !defined(__AVR_ATmega328P__)
#error "v1.11 pin-change interrupt mapping requires an ATmega328P Nano/Uno"
#endif

#define DEBUG_SERIAL 0
const char VERSION[] = "v1.11";
const uint8_t speedPulsePin = 2, encoderAPin = 3, encoderBPin = 12;
const uint8_t encoderButtonPin = A1, calSwitchPin = A0, modeButtonPin = A2;

// Future RPM expansion: A3 is digital D17 / PC3 / PCINT11 on this Nano.
// Future pulse capture must use its pin-change interrupt, not attachInterrupt.
// Conditioner output must be 0-5V logic referenced to Nano GND. The HEI TACH
// terminal is coil negative and must not be wired directly to this input.
const uint8_t rpmPulsePin = A3;
// Conventional four-stroke, single-coil V8 HEI: 8 firing events per two crank
// revolutions = 4 pulses/revolution. Change this constant for another engine
// or a conditioner that changes pulse count (V6: 3.0, four-cylinder: 2.0).
// Future conversion: RPM = frequencyHz * 60.0f / RpmPulsesPerRevolution.
constexpr float RpmPulsesPerRevolution = 4.0f;
static_assert(RpmPulsesPerRevolution > 0.0f, "RPM pulses/revolution must be positive");

// Source switches: leave disabled while these harness leads are unconnected.
#ifndef AFR_INPUT_ENABLED
#define AFR_INPUT_ENABLED 0
#endif
#ifndef DIMMER_INPUT_ENABLED
#define DIMMER_INPUT_ENABLED 0
#endif
const uint8_t afrInputPin = A6, dimmerInputPin = A7;
const uint8_t DisplayModeCount = 6;
const uint16_t AnalogUpdateMs = 100;
// DEFAULT ADC reference is the Nano's AVcc; set this to its measured voltage.
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
// A7 expects a DC level from the external conditioning/filter circuit. These
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
U8G2_SH1122_256X64_1_4W_HW_SPI display(U8G2_R0, 10, 9, 8);
// Switec counts positions 0..steps-1; allow the requested endpoint 510.
SwitecX25 Motor(MaxMotorSteps + 1, 4, 5, 6, 7);
Adafruit_FRAM_I2C fram;

struct __attribute__((packed)) StoredState {
  uint32_t magic;
  uint32_t sequence;
  uint32_t odoTenths;
  uint32_t tripTenths;
  uint32_t odoFraction;
  uint32_t tripFraction;
  uint16_t ratioMilli;
  int16_t offset;
  uint16_t schema;
  uint16_t crc;
};
// 32-byte payload + separate commit byte at offset 32.
static_assert(sizeof(StoredState) == 32, "FRAM layout changed");
const uint32_t RecordMagic = 0x53503137UL;
const uint16_t SlotAddress[2] = {0x80, 0xC0};
const uint8_t CommitMarker = 0xA5;
// Migration sentinel written only AFTER a valid first journal commit.
const uint16_t MigrationAddress = 0x70;
StoredState state;
uint8_t activeSlot = 1;
bool storageDirty = false;
uint32_t lastCheckpoint = 0, lastSpeedUpdate = 0, lastDisplayUpdate = 0;
float mph = 0;
uint16_t motorStep = 0;
uint8_t displayMode = 0;
enum CalibrationTarget : uint8_t { CAL_RATIO, CAL_OFFSET, CAL_X, CAL_Y, CAL_SIZE, CAL_DIGITS, CAL_COUNT };
CalibrationTarget calibrationTarget = CAL_RATIO;

struct __attribute__((packed)) DisplayLayout {
  uint32_t magic;
  uint32_t sequence;
  int16_t x;
  int16_t y; // text BASELINE, not top edge
  uint8_t font;
  uint8_t digits; // observed window capacity; never truncates stored mileage
  uint8_t schema;
  uint16_t crc;
};
static_assert(sizeof(DisplayLayout) == 17, "Display FRAM layout changed");
const uint16_t LayoutSlotAddress[2] = {0x100, 0x120};
const uint32_t LayoutMagic = 0x44503138UL;
DisplayLayout layout;
uint8_t activeLayoutSlot = 1;
bool layoutDirty = false, layoutInfo = false, layoutHint = false;
bool harnessWasPresent = false;
uint32_t lastLayoutChange = 0, layoutHintStarted = 0;

volatile uint32_t vssPulses = 0, vssLastUs = 0, vssPeriodUs = 0;
volatile bool vssSeen = false;
uint32_t consumedPulses = 0;
volatile uint8_t encoderPrevious = 0;
volatile int8_t encoderQuarterSteps = 0, encoderPending = 0;

struct Button {
  bool lastReading;
  bool stable;
  bool longHandled;
  uint32_t changedAt;
  uint32_t pressedAt;
};
Button encoderButton = {HIGH, HIGH, false, 0, 0};
Button modeButton = {HIGH, HIGH, false, 0, 0};

void onVssPulse() {
  uint32_t now = micros();
  uint32_t elapsed = now - vssLastUs;
  vssPeriodUs = vssSeen && elapsed <= StopTimeoutUs ? elapsed : 0;
  vssLastUs = now;
  vssSeen = true;
  ++vssPulses;
}

// A in bit 1, B in bit 0. Direct port reads keep both PCINT handlers short.
uint8_t readEncoderPins() {
  return ((PIND & _BV(PD3)) ? 2 : 0) | ((PINB & _BV(PB4)) ? 1 : 0);
}

// Complete cycles reject contact bounce; impossible two-bit jumps discard
// partial motion. Table is in flash to preserve Nano SRAM.
int8_t decodeTransition(uint8_t previous, uint8_t current) {
  static const int8_t table[16] PROGMEM = {
    0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0
  };
  return (int8_t)pgm_read_byte(&table[(previous << 2) | current]);
}

void onEncoderChange() {
  uint8_t current = readEncoderPins();
  if ((PINC & _BV(PC0)) || ((current ^ encoderPrevious) == 3)) {
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
ISR(PCINT0_vect) { onEncoderChange(); }
ISR(PCINT2_vect) { onEncoderChange(); }

uint16_t bytesCrc(const uint8_t *bytes, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)bytes[i] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

uint16_t recordCrc(const StoredState &record) {
  return bytesCrc((const uint8_t *)&record, offsetof(StoredState, crc));
}

bool validRecord(const StoredState &record) {
  return record.magic == RecordMagic && record.schema == 1 &&
    record.crc == recordCrc(record) && record.odoTenths < 10000000UL &&
    record.tripTenths < 10000UL && record.odoFraction < UnitsPerTenth &&
    record.tripFraction < UnitsPerTenth && record.ratioMilli >= 500 &&
    record.ratioMilli <= 2000 && record.offset >= -60 && record.offset <= 60;
}

bool readRecord(uint8_t slot, StoredState &record) {
  uint16_t address = SlotAddress[slot];
  if (fram.read(address + sizeof(record)) != CommitMarker) return false;
  uint8_t *bytes = (uint8_t *)&record;
  for (uint8_t i = 0; i < sizeof(record); ++i) bytes[i] = fram.read(address + i);
  return validRecord(record);
}

void storageFault() {
  display.firstPage();
  do {
    display.setFont(u8g2_font_5x7_tr);
    display.setCursor(VIEW_X, VIEW_Y);
    display.print(F("FRAM ERROR"));
  } while (display.nextPage());
  // Do not continue accumulating mileage which cannot be saved reliably.
  while (true) { Motor.update(); }
}

void saveState() {
  uint8_t nextSlot = activeSlot ^ 1;
  uint16_t address = SlotAddress[nextSlot];
  ++state.sequence;
  state.crc = recordCrc(state);
  fram.write(address + sizeof(state), 0); // invalidate destination FIRST
  if (fram.read(address + sizeof(state)) != 0) storageFault();
  const uint8_t *bytes = (const uint8_t *)&state;
  for (uint8_t i = 0; i < sizeof(state); ++i) {
    fram.write(address + i, bytes[i]);
    Motor.update();
  }
  // Verify payload before publishing the commit marker.
  for (uint8_t i = 0; i < sizeof(state); ++i) {
    if (fram.read(address + i) != bytes[i]) storageFault();
    Motor.update();
  }
  fram.write(address + sizeof(state), CommitMarker);
  if (fram.read(address + sizeof(state)) != CommitMarker) storageFault();
  activeSlot = nextSlot;
  storageDirty = false;
}

uint32_t readLegacyDigits(uint16_t address, uint8_t digits) {
  uint32_t value = 0, place = 1;
  for (uint8_t i = 0; i < digits; ++i) {
    uint8_t digit = fram.read(address + i);
    value += (digit <= 9 ? digit : 0) * place;
    place *= 10;
  }
  return value;
}

void loadState() {
  StoredState other;
  bool firstValid = readRecord(0, state);
  bool secondValid = readRecord(1, other);
  if (secondValid && (!firstValid || (int32_t)(other.sequence - state.sequence) > 0)) {
    state = other;
    activeSlot = 1;
  } else if (firstValid) {
    activeSlot = 0;
  } else {
    if (fram.read(MigrationAddress) == CommitMarker) storageFault();
    memset(&state, 0, sizeof(state));
    state.magic = RecordMagic;
    state.schema = 1;
    state.odoTenths = readLegacyDigits(0x04, 7);
    state.tripTenths = readLegacyDigits(0x00, 4);
    union { float value; uint8_t bytes[4]; } legacyRatio;
    for (uint8_t i = 0; i < 4; ++i) legacyRatio.bytes[i] = fram.read(0x20 + i);
    state.ratioMilli = isfinite(legacyRatio.value) && legacyRatio.value >= 0.5f &&
      legacyRatio.value <= 2.0f ? (uint16_t)(legacyRatio.value * 1000.0f + 0.5f) : 1000;
    int16_t offset = (uint16_t)fram.read(0x30) | ((uint16_t)fram.read(0x31) << 8);
    state.offset = offset >= -60 && offset <= 60 ? offset : 0;
    saveState();
  }
  fram.write(MigrationAddress, CommitMarker);
  if (fram.read(MigrationAddress) != CommitMarker) storageFault();
}

uint16_t layoutCrc(const DisplayLayout &record) {
  return bytesCrc((const uint8_t *)&record, offsetof(DisplayLayout, crc));
}

bool validLayout(const DisplayLayout &record) {
  return record.magic == LayoutMagic && record.schema == 1 &&
    record.crc == layoutCrc(record) && record.x >= 0 && record.x < (int16_t)OLED_WIDTH &&
    record.y >= 0 && record.y < (int16_t)OLED_HEIGHT && record.font < DigitFontCount &&
    record.digits >= 1 && record.digits <= 40;
}

bool readLayout(uint8_t slot, DisplayLayout &record) {
  uint16_t address = LayoutSlotAddress[slot];
  if (fram.read(address + sizeof(record)) != CommitMarker) return false;
  uint8_t *bytes = (uint8_t *)&record;
  for (uint8_t i = 0; i < sizeof(record); ++i) bytes[i] = fram.read(address + i);
  return validLayout(record);
}

void loadLayout() {
  DisplayLayout other;
  bool firstValid = readLayout(0, layout), secondValid = readLayout(1, other);
  if (secondValid && (!firstValid || (int32_t)(other.sequence - layout.sequence) > 0)) {
    layout = other;
    activeLayoutSlot = 1;
  } else if (firstValid) {
    activeLayoutSlot = 0;
  } else {
    // No display record in v1.07. Corrupt/unknown layout falls back to these
    // defaults without touching mileage, fractions, ratio or needle offset.
    memset(&layout, 0, sizeof(layout));
    layout.magic = LayoutMagic;
    layout.schema = 1;
    layout.x = VIEW_X;
    layout.y = VIEW_Y;
    layout.font = DefaultDigitFont;
    layout.digits = 10;
    activeLayoutSlot = 1;
  }
  layoutDirty = false;
}

void saveLayout() {
  if (!layoutDirty) return;
  uint8_t nextSlot = activeLayoutSlot ^ 1;
  uint16_t address = LayoutSlotAddress[nextSlot];
  ++layout.sequence;
  layout.crc = layoutCrc(layout);
  fram.write(address + sizeof(layout), 0);
  if (fram.read(address + sizeof(layout)) != 0) storageFault();
  const uint8_t *bytes = (const uint8_t *)&layout;
  for (uint8_t i = 0; i < sizeof(layout); ++i) {
    fram.write(address + i, bytes[i]);
    Motor.update();
  }
  for (uint8_t i = 0; i < sizeof(layout); ++i) {
    if (fram.read(address + i) != bytes[i]) storageFault();
    Motor.update();
  }
  fram.write(address + sizeof(layout), CommitMarker);
  if (fram.read(address + sizeof(layout)) != CommitMarker) storageFault();
  activeLayoutSlot = nextSlot;
  layoutDirty = false;
}

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

// Bounded chunks avoid 32-bit multiplication overflow even after a long pause.
void addDistance(uint32_t pulses) {
  while (pulses) {
    uint16_t chunk = pulses > 10000UL ? 10000 : pulses;
    uint32_t units = (uint32_t)chunk * state.ratioMilli;
    state.odoFraction += units;
    state.tripFraction += units;
    state.odoTenths = (state.odoTenths + state.odoFraction / UnitsPerTenth) % 10000000UL;
    state.tripTenths = (state.tripTenths + state.tripFraction / UnitsPerTenth) % 10000UL;
    state.odoFraction %= UnitsPerTenth;
    state.tripFraction %= UnitsPerTenth;
    pulses -= chunk;
    storageDirty = true;
  }
}

void collectDistance() {
  uint32_t total;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { total = vssPulses; }
  addDistance(total - consumedPulses);
  consumedPulses = total;
}

void updateSpeedometer() {
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
  mph = seen && validPeriod ? (3600000000.0f / PulsesPerMile) / period *
    (state.ratioMilli / 1000.0f) : 0.0f;
  // Apply calibration before clamping, so a negative offset does not reduce
  // the attainable endpoint when the calculated speed exceeds the dial range.
  float steps = mph * StepsPerMPH + state.offset;
  motorStep = steps <= 0 ? 0 : steps >= MaxMotorSteps ? MaxMotorSteps : (uint16_t)(steps + 0.5f);
  Motor.setPosition(motorStep);
}

// Returns 1 on short release, 2 once while held for LongPressMs.
uint8_t pollButton(Button &button, uint8_t pin, uint32_t now) {
  bool reading = digitalRead(pin);
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

void handleControls(uint32_t now) {
  bool calibration = digitalRead(calSwitchPin) == LOW;
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
  uint8_t encoderEvent = pollButton(encoderButton, encoderButtonPin, now);
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

void selectDigitFont() {
  const uint8_t *font = (const uint8_t *)pgm_read_ptr(&DigitFonts[layout.font]);
  display.setFont(font);
  display.setFontPosBaseline();
  display.setFontRefHeightAll();
}

uint8_t digitCellWidth() {
  uint8_t widest = 1;
  char digit[2] = {'0', 0};
  for (; digit[0] <= '9'; ++digit[0]) {
    uint16_t width = display.getStrWidth(digit);
    if (width > widest) widest = width;
  }
  return widest + 1; // consistent spacing for test digits AND normal numbers
}

// Caller selects the numeric font first. Saved N describes the operator's
// observed capacity; additionally reject any geometry clipped by the OLED.
bool numericFits(uint8_t cells) {
  uint16_t right = layout.x + (uint16_t)cells * digitCellWidth();
  return cells <= layout.digits && layout.x >= 0 && right <= OLED_WIDTH &&
    layout.y >= display.getAscent() &&
    layout.y - display.getDescent() < (int16_t)OLED_HEIGHT;
}

void drawFitWarning() {
  // Small text fits within the observed horizontal span where possible.
  // A wholly off-screen layout cannot guarantee a visible warning, but still
  // must never render a misleading fragment of the numeric value.
  uint16_t available = (uint16_t)layout.digits * digitCellWidth();
  uint16_t remaining = OLED_WIDTH - layout.x;
  if (available > remaining) available = remaining;
  display.setFont(u8g2_font_5x7_tr);
  display.setCursor(layout.x, constrain(layout.y, 7, 63));
  if (available >= display.getStrWidth("FIT")) display.print(F("FIT"));
  else display.print(F("!"));
}

void drawNumericText(const char *text) {
  selectDigitFont();
  if (!numericFits(strlen(text))) {
    drawFitWarning();
    return;
  }
  uint8_t cell = digitCellWidth();
  uint16_t x = layout.x;
  for (; *text && x < OLED_WIDTH; ++text, x += cell)
    display.drawGlyph(x, layout.y, *text);
}

void printTenths(uint32_t tenths) {
  // A decimal point occupies one cell, so 999999.9 needs EIGHT visible cells.
  char text[12];
  ultoa(tenths / 10, text, 10);
  uint8_t length = strlen(text);
  text[length++] = '.';
  text[length++] = '0' + tenths % 10;
  text[length] = 0;
  drawNumericText(text);
}

void drawOdometer() {
  selectDigitFont();
  // Validate the full lifetime range before showing even a short value.
  if (!numericFits(8)) {
    drawFitWarning();
    return;
  }
  printTenths(state.odoTenths);
}

void drawFloatValue(float value, uint8_t decimals) {
  char text[20]; // even the shortest supported pulse period fits this buffer
  dtostrf(value, 1, decimals, text);
  drawNumericText(text);
}

void drawLayoutCalibration() {
  selectDigitFont();
  uint8_t cell = digitCellWidth();
  int16_t height = display.getAscent() - display.getDescent();
  bool odometerFits = numericFits(8);
  if (layoutInfo || layoutHint) {
    // Readout REPLACES the digits, never overlays the fitting pattern. Keep
    // its baseline near the chosen position; small Y values still show a row.
    display.setFont(u8g2_font_5x7_tr);
    display.setCursor(layout.x, max((int16_t)7, layout.y - 9));
    switch (calibrationTarget) {
      case CAL_X: display.print(F("X ")); display.print(layout.x); break;
      case CAL_Y: display.print(F("Y ")); display.print(layout.y); break;
      case CAL_SIZE: display.print(F("SIZE ")); display.print(height); display.print(F("px")); break;
      case CAL_DIGITS: display.print(F("DIGITS ")); display.print(layout.digits); break;
      default: break;
    }
    if (!odometerFits) display.print(F(" FIT8"));
    display.setCursor(layout.x, max((int16_t)16, layout.y));
    display.print(F("X")); display.print(layout.x);
    display.print(F(" Y")); display.print(layout.y);
    display.print(F(" H")); display.print(height);
    display.print(F(" N")); display.print(layout.digits);
    return;
  }
  // Pattern and normal readings have identical glyph, baseline and spacing.
  // Keep arithmetic 16-bit: 40 large digits extend beyond the OLED and must
  // clip at its edge rather than wrap around to the left or onto another row.
  for (uint8_t i = 0; i < layout.digits; ++i) {
    uint16_t x = layout.x + (uint16_t)i * cell;
    if (x >= OLED_WIDTH) break;
    display.drawGlyph(x, layout.y, '0' + i % 10);
  }
}

void drawScreen(bool calibration) {
  display.setFont(u8g2_font_5x7_tr);
  if (calibration) {
    if (calibrationTarget >= CAL_X) {
      drawLayoutCalibration();
      return;
    }
    display.setCursor(layout.x, max((int16_t)7, layout.y - 18));
    display.print(F("CAL MPH "));
    display.print(mph, 1);
    display.setCursor(layout.x, max((int16_t)16, layout.y - 8));
    if (calibrationTarget == CAL_OFFSET) {
      display.print(F("OFFSET "));
      display.print(state.offset);
      display.print(F(" st"));
    } else {
      display.print(F("RATIO "));
      display.print(state.ratioMilli / 1000.0f, 3);
    }
    display.setCursor(layout.x + VIEW_W - 30, layout.y);
    display.print(VERSION);
    return;
  }
  selectDigitFont();
  int16_t labelY = layout.y - display.getAscent() - 2;
  display.setFont(u8g2_font_5x7_tr);
  display.setCursor(layout.x, max((int16_t)7, labelY));
  // If the numeric line reaches the OLED top, omit the label instead of
  // drawing it over the digits. The physical window may hide labels anyway.
  if (labelY >= 7) {
    switch (displayMode) {
      case 0: display.print(F("ODO")); break;
      case 1: display.print(F("TRIP")); break;
      case 2: display.print(F("MPH")); break;
      case 3: display.print(F("RATIO")); break;
      case 4: display.print(F("RPM (not connected)")); break;
      case 5: display.print(F("AFR")); break;
    }
  }
  switch (displayMode) {
    case 0: drawOdometer(); break;
    case 1: printTenths(state.tripTenths); break;
    case 2: drawFloatValue(mph, 1); break;
    case 3: drawFloatValue(state.ratioMilli / 1000.0f, 3); break;
    case 4: drawNumericText("--"); break;
    case 5:
      if (afrValid) drawFloatValue(afr, 1);
      else drawNumericText("--");
      break;
  }
}

void updateDisplay() {
  bool calibration = digitalRead(calSwitchPin) == LOW;
  display.firstPage();
  do {
    drawScreen(calibration);
    Motor.update();
  } while (display.nextPage());
}

// Service mileage during the startup sweep/delays as well as normal operation.
void serviceMotion() {
  Motor.update();
  collectDistance();
  uint32_t now = millis();
  if (storageDirty && now - lastCheckpoint >= CheckpointMs) {
    saveState();
    lastCheckpoint = millis();
  }
}

void servicedWait(uint16_t duration) {
  uint32_t started = millis();
  while (millis() - started < duration) serviceMotion();
}

uint16_t readAnalogAverage(uint8_t pin) {
  analogRead(pin); // discard first conversion after ADC channel switching
  uint16_t total = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    total += analogRead(pin);
    Motor.update();
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

void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);
#endif
  pinMode(speedPulsePin, INPUT_PULLUP);
  pinMode(encoderAPin, INPUT_PULLUP);
  pinMode(encoderBPin, INPUT_PULLUP);
  pinMode(encoderButtonPin, INPUT_PULLUP);
  pinMode(calSwitchPin, INPUT_PULLUP);
  pinMode(modeButtonPin, INPUT_PULLUP);
  pinMode(rpmPulsePin, INPUT_PULLUP); // reserved only; no RPM ISR enabled yet
  // A6/A7 have no digital buffers/pull-ups: do not call pinMode on them.
  display.begin();
#if DIMMER_INPUT_ENABLED
  display.setContrast(DimmerContrastMax);
#endif
  if (!fram.begin(0x50)) storageFault();
  loadState();
  loadLayout();
  attachInterrupt(digitalPinToInterrupt(speedPulsePin), onVssPulse, RISING);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    encoderPrevious = readEncoderPins();
    PCMSK0 |= _BV(PCINT4); // D12 / PB4
    PCMSK2 |= _BV(PCINT19); // D3 / PD3
    PCIFR = _BV(PCIF0) | _BV(PCIF2);
    PCICR |= _BV(PCIE0) | _BV(PCIE2);
  }
  display.firstPage();
  do {
    display.setFont(u8g2_font_5x7_tr);
    display.setCursor(VIEW_X, VIEW_Y);
    display.print(F("Savoy "));
    display.print(VERSION);
  } while (display.nextPage());
  // zero() blocks while homing, but VSS interrupts remain active; account for
  // those pulses immediately afterward. Start the car stationary for homing.
  Motor.zero();
  collectDistance();
  Motor.setPosition(MaxMotorSteps);
  while (!Motor.stopped) serviceMotion();
  servicedWait(1000);
  Motor.setPosition(0);
  while (!Motor.stopped) serviceMotion();
  servicedWait(1000);
  updateSpeedometer();
  updateDisplay();
}

void loop() {
  serviceMotion();
  uint32_t now = millis();
  handleControls(now);
  updateAnalogInputs(now);
  if (now - lastSpeedUpdate >= SpeedUpdateMs) {
    updateSpeedometer();
    lastSpeedUpdate = now;
  }
  if (now - lastDisplayUpdate >= DisplayUpdateMs) {
    updateDisplay();
    lastDisplayUpdate = millis();
#if DEBUG_SERIAL
    Serial.print(F("MPH=")); Serial.print(mph, 1);
    Serial.print(F(" ODO tenths=")); Serial.println(state.odoTenths);
#endif
  }
}
