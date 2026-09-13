/*
 * Stepper Speedometer v1.07 -- Robert Cipriani -- 2026-09-12
 * File: stepper_speedometer_v1_07.ino
 * Target: classic Arduino Nano / ATmega328P, 16 MHz, 2 KB SRAM.
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
 * A2: mode/trip button to ground. A3 reserved for conditioned dimmer/AFR.
 * D4-D7: X25/X27 motor. OLED SPI: D11 MOSI, D13 SCK, D10 CS, D9 DC,
 * D8 RESET. FRAM: A4 SDA, A5 SCL, address 0x50. Encoder power: 5V/GND.
 * VSS must come through the Mini MAX A2 or equivalent signal conditioner;
 * never connect vehicle voltage or raw VR signals directly to the Nano.
 *
 * Turn encoder to adjust ratio (0.500..2.000) or offset (-60..60 steps).
 * Short encoder press toggles target. Long press does not toggle/reset.
 * Short mode press cycles ODO/TRIP/MPH/RATIO/RPM (RPM is unimplemented).
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
#error "v1.07 pin-change interrupt mapping requires an ATmega328P Nano/Uno"
#endif

#define DEBUG_SERIAL 0
const char VERSION[] = "v1.07";
const uint8_t speedPulsePin = 2, encoderAPin = 3, encoderBPin = 12;
const uint8_t encoderButtonPin = A1, calSwitchPin = A0, modeButtonPin = A2;
const uint16_t MaxMotorSteps = 510;
const float StepsPerMPH = 5.1f;
const uint32_t PulsesPerMile = 4000;
// Distance units are 1/(PulsesPerMile * 1000) mile. Each pulse adds ratioMilli.
const uint32_t UnitsPerTenth = PulsesPerMile * 100UL;
const uint32_t StopTimeoutUs = 3000000UL; // supports ~0.3 MPH at ratio 1
const uint16_t CheckpointMs = 100, SpeedUpdateMs = 100, DisplayUpdateMs = 200;
const uint16_t DebounceMs = 35, LongPressMs = 1000;
const int VIEW_X = 5, VIEW_Y = 38, VIEW_W = 130, VIEW_H = 24;

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
bool adjustOffset = false;

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

uint16_t recordCrc(const StoredState &record) {
  const uint8_t *bytes = (const uint8_t *)&record;
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < offsetof(StoredState, crc); ++i) {
    crc ^= (uint16_t)bytes[i] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
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
  int8_t delta;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    delta = encoderPending;
    encoderPending = 0;
    if (!calibration) encoderQuarterSteps = 0;
  }
  if (calibration && delta) {
    // Account for all preceding pulses using the preceding ratio.
    collectDistance();
    if (adjustOffset) state.offset = constrain(state.offset + delta, -60, 60);
    else state.ratioMilli = constrain((int16_t)state.ratioMilli + delta, 500, 2000);
    storageDirty = true;
  }
  uint8_t encoderEvent = pollButton(encoderButton, encoderButtonPin, now);
  if (calibration && encoderEvent == 1) adjustOffset = !adjustOffset;
  uint8_t modeEvent = pollButton(modeButton, modeButtonPin, now);
  if (modeEvent == 1) displayMode = (displayMode + 1) % 5;
  if (modeEvent == 2 && displayMode == 1) {
    collectDistance();
    state.tripTenths = 0;
    state.tripFraction = 0;
    storageDirty = true;
    saveState();
  }
}

void printTenths(uint32_t tenths) {
  // Avoid loss of decimal digits from float conversion at high mileages.
  display.print(tenths / 10);
  display.print('.');
  display.print(tenths % 10);
}

void drawScreen(bool calibration) {
  display.setFont(u8g2_font_5x7_tr);
  if (calibration) {
    display.setCursor(VIEW_X, VIEW_Y - 18);
    display.print(F("CAL MPH "));
    display.print(mph, 1);
    display.setCursor(VIEW_X, VIEW_Y - 8);
    if (adjustOffset) {
      display.print(F("OFFSET "));
      display.print(state.offset);
      display.print(F(" st"));
    } else {
      display.print(F("RATIO "));
      display.print(state.ratioMilli / 1000.0f, 3);
    }
    display.setCursor(VIEW_X + VIEW_W - 30, VIEW_Y);
    display.print(VERSION);
    return;
  }
  display.setCursor(VIEW_X, VIEW_Y - 18);
  switch (displayMode) {
    case 0: display.print(F("ODO")); break;
    case 1: display.print(F("TRIP")); break;
    case 2: display.print(F("MPH")); break;
    case 3: display.print(F("RATIO")); break;
    case 4: display.print(F("RPM (not connected)")); break;
  }
  display.setFont(u8g2_font_luBS12_tn);
  display.setCursor(VIEW_X, VIEW_Y);
  switch (displayMode) {
    case 0: printTenths(state.odoTenths); break;
    case 1: printTenths(state.tripTenths); break;
    case 2: display.print(mph, 1); break;
    case 3: display.print(state.ratioMilli / 1000.0f, 3); break;
    case 4: display.print(F("--")); break;
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
  display.begin();
  if (!fram.begin(0x50)) storageFault();
  loadState();
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
