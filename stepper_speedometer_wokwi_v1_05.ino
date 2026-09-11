/******************************************************************************
 * Stepper Speedometer - Wokwi Simulation
 * File: stepper_speedometer_wokwi_v1_05.ino
 * Author: Robert Cipriani
 * Date: 2026-06-09
 * Version: v1.05-wokwi
 *
 * Wokwi version derived from physical firmware v1.05.
 * Uses SSD1306 I2C OLED, RAM-backed fake FRAM, D2 pulse counting,
 * and a servo on D4 as a visual stand-in for the X25/X27 gauge motor.
 ******************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

const char VERSION[] = "v1.05-wokwi";
const char SKETCH_FILE[] = "stepper_speedometer_wokwi_v1_05.ino";
const char SKETCH_DATE[] = "2026-06-09";

// -----------------------------------------------------------------------------
// Wokwi OLED Display Settings
// -----------------------------------------------------------------------------
// Wokwi SSD1306 I2C wiring for Arduino Nano:
//   VCC -> 5V
//   GND -> GND
//   SDA -> A4
//   SCL -> A5
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// -----------------------------------------------------------------------------
// Fake Wokwi speedometer needle using a servo
// -----------------------------------------------------------------------------
// This is only for simulation. The physical sketch uses SwitecX25.
// Wokwi servo wiring:
//   Signal -> D4
//   V+     -> 5V
//   GND    -> GND
const uint8_t fakeNeedleServoPin = 4;
Servo fakeNeedleServo;

const int FakeNeedleMinAngle = 0;
const int FakeNeedleMaxAngle = 170;

// -----------------------------------------------------------------------------
// Fake FRAM for Wokwi simulation
// -----------------------------------------------------------------------------
// Volatile RAM-backed stand-in. Resets when the simulation restarts.
// Kept small because Adafruit_SSD1306 needs a 1024-byte display buffer on Nano.
uint8_t fakeFram[64];

uint8_t framRead(uint16_t addr)
{
  if (addr >= sizeof(fakeFram)) return 0;
  return fakeFram[addr];
}

void framWrite(uint16_t addr, uint8_t value)
{
  if (addr >= sizeof(fakeFram)) return;
  fakeFram[addr] = value;
}

// -----------------------------------------------------------------------------
// Speedometer constants
// -----------------------------------------------------------------------------
const int UpdateInterval = 100;      // 100 ms speed update rate
const int UpdateInterval2 = 1000;    // 1 second odometer/display update rate
const double StepsPerDegree = 3.0;   // Motor step is 1/3 degree of rotation
const unsigned int MaxMotorRotation = 170;
const unsigned int MaxMotorSteps = MaxMotorRotation * StepsPerDegree;
const double PulsesPerMile = 4000.0;
const double SecondsPerHour = 3600.0;
const int FeetPerMile = 5280;
const double SpeedoDegreesPerMPH = 170.0 / 100.0; // 100 MPH at 170 degrees

unsigned long PreviousMillis = 0;
unsigned long PreviousMillis2 = 0;
unsigned long distsubtotalFeetTenths = 0;

unsigned int motorStep = 0;  // calculated speedometer step before needle offset
unsigned int needleStep = 0; // final step after needle offset
float mph = 0.0;
float rpm = 0.0; // future placeholder

// Wokwi speed pulse input. Use Clock Generator OUT -> D2, GND -> GND.
// Physical v1.05 expects a conditioned VSS input, currently documented as the
// WTMtronics Mini MAX A2 MAX9926-based VR conditioner. Wokwi replaces that with
// a clean simulated pulse source.
const uint8_t speedPulsePin = 2;
volatile unsigned long pulseCount = 0;
unsigned long lastPulseCount = 0;

// -----------------------------------------------------------------------------
// Calibration ratio storage
// -----------------------------------------------------------------------------
union RatioUnion
{
  float f;
  byte b[4];
};

RatioUnion ratio;

const float DefaultRatio = 1.000;
const float MinRatio = 0.500;
const float MaxRatio = 2.000;
const float RatioStep = 0.001;

// Needle offset shifts the simulated needle without changing MPH/odometer math.
// Matches physical v1.05 behavior.
int16_t needleOffsetSteps = 0;
const int16_t DefaultNeedleOffsetSteps = 0;
const int16_t MinNeedleOffsetSteps = -60; // -20 degrees at 3 steps/degree
const int16_t MaxNeedleOffsetSteps = 60;  // +20 degrees at 3 steps/degree
const int16_t NeedleOffsetStep = 1;       // one motor step = 1/3 degree

// Fake FRAM address map, matching physical v1.05.
const uint16_t ADDR_TRIP_TENTHS = 0x00;
const uint16_t ADDR_TRIP_ONES = 0x01;
const uint16_t ADDR_TRIP_TENS = 0x02;
const uint16_t ADDR_TRIP_HUNDREDS = 0x03;

const uint16_t ADDR_ODO_TENTHS = 0x04;
const uint16_t ADDR_ODO_ONES = 0x05;
const uint16_t ADDR_ODO_TENS = 0x06;
const uint16_t ADDR_ODO_HUNDREDS = 0x07;
const uint16_t ADDR_ODO_THOUSANDS = 0x08;
const uint16_t ADDR_ODO_TEN_THOUSANDS = 0x09;
const uint16_t ADDR_ODO_HUNDRED_THOUSANDS = 0x0A;

const uint16_t ADDR_RATIO = 0x20;
const uint16_t ADDR_NEEDLE_OFFSET = 0x30;

// Odometer digit storage
uint8_t tripdisttenthsm = 0;
uint8_t tripdistm = 0;
uint8_t tripdistenm = 0;
uint8_t tripdisthundredm = 0;

uint8_t disttenthsm = 0;
uint8_t distm = 0;
uint8_t distenm = 0;
uint8_t disthundredm = 0;
uint8_t disthousandm = 0;
uint8_t disttenthousandm = 0;
uint8_t disthundredthousandm = 0;

// -----------------------------------------------------------------------------
// Calibration harness / controls
// -----------------------------------------------------------------------------
// Harness detect: A0/D14 to GND when calibration harness is plugged in.
const uint8_t calSwitchPin = A0;

// Encoder wiring for Wokwi rotary encoder:
//   CLK/A -> D3
//   DT/B  -> D12
//   SW    -> A1
//   +     -> 5V
//   GND   -> GND
const uint8_t encoderAPin = 3;
const uint8_t encoderBPin = 12;
const uint8_t encoderButtonPin = A1;

// Mode/trip button:
//   A2 -> pushbutton -> GND
const uint8_t modeButtonPin = A2;

int lastEncoderA = HIGH;

const unsigned long EncoderSaveIntervalMs = 1000;
unsigned long lastRatioChangeMillis = 0;
unsigned long lastOffsetChangeMillis = 0;
bool ratioDirty = false;
bool offsetDirty = false;

const unsigned long DebounceMs = 35;
const unsigned long LongPressMs = 1000;

bool encButtonLastReading = HIGH;
bool encButtonStableState = HIGH;
unsigned long encButtonLastChange = 0;
unsigned long encButtonPressedAt = 0;

bool modeButtonLastReading = HIGH;
bool modeButtonStableState = HIGH;
unsigned long modeButtonLastChange = 0;
unsigned long modeButtonPressedAt = 0;
bool modeLongPressHandled = false;

// Display modes. uint8_t avoids Arduino/Wokwi auto-prototype enum issues.
const uint8_t MODE_ODO = 0;
const uint8_t MODE_TRIP = 1;
const uint8_t MODE_MPH = 2;
const uint8_t MODE_RATIO = 3;
const uint8_t MODE_RPM = 4;
const uint8_t MODE_COUNT = 5;
uint8_t displayMode = MODE_ODO;

// Function declarations
void speedPulseISR();
void loadStoredValues();
void saveOdometerToFram();
void saveTripToFram();
void resetTrip();
void loadRatioFromFram();
void saveRatioToFram();
float clampRatio(float value);
void loadNeedleOffsetFromFram();
void saveNeedleOffsetToFram();
int16_t clampNeedleOffset(int16_t value);
unsigned int applyNeedleOffset(unsigned int baseStep);
void handleEncoderCalibration();
void handleEncoderButton();
void handleModeButton();
void updateSpeedFromPulses(unsigned long elapsedMs);
void updateDistance();
void updateodometer();
void incrementOdometerTenth();
void incrementTripTenth();
float getOdometerMiles();
float getTripMiles();
unsigned int mphToStep(float speedMph);
void displayStartupLogo();
void updateDisplay();
void updateFakeNeedleServo();
void servoStartupSweep();
const __FlashStringHelper* displayModeName(uint8_t mode);
void printStatusToSerial();

void setup(void)
{
  Serial.begin(9600);
  delay(50);

  pinMode(speedPulsePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(speedPulsePin), speedPulseISR, RISING);

  pinMode(calSwitchPin, INPUT_PULLUP);
  pinMode(encoderAPin, INPUT_PULLUP);
  pinMode(encoderBPin, INPUT_PULLUP);
  pinMode(encoderButtonPin, INPUT_PULLUP);
  pinMode(modeButtonPin, INPUT_PULLUP);

  lastEncoderA = digitalRead(encoderAPin);

  // Initialize OLED before attaching Servo to keep Nano RAM behavior predictable.
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 init failed at 0x3C"));
    Serial.println(F("Check A4=SDA, A5=SCL, or reduce RAM usage."));
    while (1) delay(10);
  }

  fakeNeedleServo.attach(fakeNeedleServoPin);
  fakeNeedleServo.write(FakeNeedleMinAngle);

  displayStartupLogo();
  servoStartupSweep();

  loadStoredValues();
  loadRatioFromFram();
  loadNeedleOffsetFromFram();

  Serial.println(F("Savoy Speedometer Wokwi Simulation"));
  Serial.print(F("Firmware: "));
  Serial.println(VERSION);
  Serial.println(F("Vehicle speed input: Clock Generator OUT -> D2"));
  Serial.println(F("Calibration harness detect: A0 -> switch -> GND"));
  Serial.println(F("Encoder: CLK D3, DT D12, SW A1"));
  Serial.println(F("Mode/trip button: A2 -> button -> GND"));
  Serial.println(F("Fake speedometer servo: signal D4, V+ 5V, GND GND"));
  Serial.println();

  PreviousMillis = millis();
  PreviousMillis2 = millis();
  updateDisplay();
}

void loop()
{
  unsigned long currentMillis = millis();

  handleEncoderCalibration();
  handleEncoderButton();
  handleModeButton();

  if (currentMillis - PreviousMillis >= UpdateInterval) {
    unsigned long elapsed = currentMillis - PreviousMillis;
    PreviousMillis = currentMillis;
    updateSpeedFromPulses(elapsed);
  }

  if (currentMillis - PreviousMillis2 >= UpdateInterval2) {
    PreviousMillis2 = currentMillis;

    updateDistance();
    saveOdometerToFram();
    saveTripToFram();

    if (ratioDirty && (currentMillis - lastRatioChangeMillis >= EncoderSaveIntervalMs)) {
      saveRatioToFram();
      ratioDirty = false;
    }

    if (offsetDirty && (currentMillis - lastOffsetChangeMillis >= EncoderSaveIntervalMs)) {
      saveNeedleOffsetToFram();
      offsetDirty = false;
    }

    updateDisplay();
    printStatusToSerial();
  }
}

void speedPulseISR()
{
  pulseCount++;
}

void loadStoredValues()
{
  tripdisttenthsm = framRead(ADDR_TRIP_TENTHS);
  tripdistm = framRead(ADDR_TRIP_ONES);
  tripdistenm = framRead(ADDR_TRIP_TENS);
  tripdisthundredm = framRead(ADDR_TRIP_HUNDREDS);

  disttenthsm = framRead(ADDR_ODO_TENTHS);
  distm = framRead(ADDR_ODO_ONES);
  distenm = framRead(ADDR_ODO_TENS);
  disthundredm = framRead(ADDR_ODO_HUNDREDS);
  disthousandm = framRead(ADDR_ODO_THOUSANDS);
  disttenthousandm = framRead(ADDR_ODO_TEN_THOUSANDS);
  disthundredthousandm = framRead(ADDR_ODO_HUNDRED_THOUSANDS);

  if (tripdisttenthsm > 9) tripdisttenthsm = 0;
  if (tripdistm > 9) tripdistm = 0;
  if (tripdistenm > 9) tripdistenm = 0;
  if (tripdisthundredm > 9) tripdisthundredm = 0;

  if (disttenthsm > 9) disttenthsm = 0;
  if (distm > 9) distm = 0;
  if (distenm > 9) distenm = 0;
  if (disthundredm > 9) disthundredm = 0;
  if (disthousandm > 9) disthousandm = 0;
  if (disttenthousandm > 9) disttenthousandm = 0;
  if (disthundredthousandm > 9) disthundredthousandm = 0;
}

void saveOdometerToFram()
{
  framWrite(ADDR_ODO_TENTHS, disttenthsm);
  framWrite(ADDR_ODO_ONES, distm);
  framWrite(ADDR_ODO_TENS, distenm);
  framWrite(ADDR_ODO_HUNDREDS, disthundredm);
  framWrite(ADDR_ODO_THOUSANDS, disthousandm);
  framWrite(ADDR_ODO_TEN_THOUSANDS, disttenthousandm);
  framWrite(ADDR_ODO_HUNDRED_THOUSANDS, disthundredthousandm);
}

void saveTripToFram()
{
  framWrite(ADDR_TRIP_TENTHS, tripdisttenthsm);
  framWrite(ADDR_TRIP_ONES, tripdistm);
  framWrite(ADDR_TRIP_TENS, tripdistenm);
  framWrite(ADDR_TRIP_HUNDREDS, tripdisthundredm);
}

void resetTrip()
{
  tripdisttenthsm = 0;
  tripdistm = 0;
  tripdistenm = 0;
  tripdisthundredm = 0;
  saveTripToFram();
  Serial.println(F("Trip reset to 0.0"));
  updateDisplay();
}

void loadRatioFromFram()
{
  ratio.b[0] = framRead(ADDR_RATIO + 0);
  ratio.b[1] = framRead(ADDR_RATIO + 1);
  ratio.b[2] = framRead(ADDR_RATIO + 2);
  ratio.b[3] = framRead(ADDR_RATIO + 3);

  if (isnan(ratio.f) || ratio.f < MinRatio || ratio.f > MaxRatio) {
    ratio.f = DefaultRatio;
    saveRatioToFram();
  }
}

void saveRatioToFram()
{
  ratio.f = clampRatio(ratio.f);
  framWrite(ADDR_RATIO + 0, ratio.b[0]);
  framWrite(ADDR_RATIO + 1, ratio.b[1]);
  framWrite(ADDR_RATIO + 2, ratio.b[2]);
  framWrite(ADDR_RATIO + 3, ratio.b[3]);

  Serial.print(F("Ratio saved: "));
  Serial.println(ratio.f, 3);
}

float clampRatio(float value)
{
  if (value < MinRatio) return MinRatio;
  if (value > MaxRatio) return MaxRatio;
  return value;
}

void loadNeedleOffsetFromFram()
{
  uint8_t lo = framRead(ADDR_NEEDLE_OFFSET + 0);
  uint8_t hi = framRead(ADDR_NEEDLE_OFFSET + 1);
  needleOffsetSteps = (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));

  if (needleOffsetSteps < MinNeedleOffsetSteps || needleOffsetSteps > MaxNeedleOffsetSteps) {
    needleOffsetSteps = DefaultNeedleOffsetSteps;
    saveNeedleOffsetToFram();
  }
}

void saveNeedleOffsetToFram()
{
  needleOffsetSteps = clampNeedleOffset(needleOffsetSteps);
  uint16_t raw = (uint16_t)needleOffsetSteps;
  framWrite(ADDR_NEEDLE_OFFSET + 0, raw & 0xFF);
  framWrite(ADDR_NEEDLE_OFFSET + 1, (raw >> 8) & 0xFF);

  Serial.print(F("Needle offset saved: "));
  Serial.print(needleOffsetSteps);
  Serial.println(F(" steps"));
}

int16_t clampNeedleOffset(int16_t value)
{
  if (value < MinNeedleOffsetSteps) return MinNeedleOffsetSteps;
  if (value > MaxNeedleOffsetSteps) return MaxNeedleOffsetSteps;
  return value;
}

unsigned int applyNeedleOffset(unsigned int baseStep)
{
  long adjusted = (long)baseStep + (long)needleOffsetSteps;
  if (adjusted < 0) adjusted = 0;
  if (adjusted > MaxMotorSteps) adjusted = MaxMotorSteps;
  return (unsigned int)adjusted;
}

void handleEncoderCalibration()
{
  bool calMode = (digitalRead(calSwitchPin) == LOW);
  if (!calMode) {
    lastEncoderA = digitalRead(encoderAPin);
    return;
  }

  int encoderA = digitalRead(encoderAPin);
  if (encoderA != lastEncoderA) {
    if (encoderA == HIGH) {
      int encoderB = digitalRead(encoderBPin);
      bool offsetAdjustMode = (digitalRead(encoderButtonPin) == LOW);

      if (offsetAdjustMode) {
        if (encoderB == LOW) {
          needleOffsetSteps += NeedleOffsetStep;
        } else {
          needleOffsetSteps -= NeedleOffsetStep;
        }

        needleOffsetSteps = clampNeedleOffset(needleOffsetSteps);
        lastOffsetChangeMillis = millis();
        offsetDirty = true;
        needleStep = applyNeedleOffset(motorStep);
        updateFakeNeedleServo();

        Serial.print(F("Needle offset adjusted: "));
        Serial.print(needleOffsetSteps);
        Serial.println(F(" steps"));
      } else {
        if (encoderB == LOW) {
          ratio.f += RatioStep;
        } else {
          ratio.f -= RatioStep;
        }

        ratio.f = clampRatio(ratio.f);
        lastRatioChangeMillis = millis();
        ratioDirty = true;

        Serial.print(F("Ratio adjusted: "));
        Serial.println(ratio.f, 3);
      }

      updateDisplay();
    }
    lastEncoderA = encoderA;
  }
}

void handleEncoderButton()
{
  bool reading = digitalRead(encoderButtonPin);
  unsigned long now = millis();

  if (reading != encButtonLastReading) {
    encButtonLastChange = now;
    encButtonLastReading = reading;
  }

  if ((now - encButtonLastChange) > DebounceMs && reading != encButtonStableState) {
    encButtonStableState = reading;

    if (encButtonStableState == LOW) {
      encButtonPressedAt = now;
      updateDisplay();
    } else {
      if (ratioDirty) {
        saveRatioToFram();
        ratioDirty = false;
      }

      if (offsetDirty) {
        saveNeedleOffsetToFram();
        offsetDirty = false;
      }

      updateDisplay();
    }
  }
}

void handleModeButton()
{
  bool reading = digitalRead(modeButtonPin);
  unsigned long now = millis();

  if (reading != modeButtonLastReading) {
    modeButtonLastChange = now;
    modeButtonLastReading = reading;
  }

  if ((now - modeButtonLastChange) > DebounceMs && reading != modeButtonStableState) {
    modeButtonStableState = reading;

    if (modeButtonStableState == LOW) {
      modeButtonPressedAt = now;
      modeLongPressHandled = false;
    } else {
      if (!modeLongPressHandled) {
        displayMode = (displayMode + 1) % MODE_COUNT;
        Serial.print(F("Mode: "));
        Serial.println(displayModeName(displayMode));
        updateDisplay();
      }
    }
  }

  if (modeButtonStableState == LOW && !modeLongPressHandled && (now - modeButtonPressedAt >= LongPressMs)) {
    if (displayMode == MODE_TRIP) {
      resetTrip();
    }
    modeLongPressHandled = true;
  }
}

void updateSpeedFromPulses(unsigned long elapsedMs)
{
  noInterrupts();
  unsigned long currentPulseCount = pulseCount;
  interrupts();

  unsigned long pulses = currentPulseCount - lastPulseCount;
  lastPulseCount = currentPulseCount;

  float elapsedSeconds = elapsedMs / 1000.0;
  float frequencyHz = 0.0;
  if (elapsedSeconds > 0.0) {
    frequencyHz = pulses / elapsedSeconds;
  }

  mph = frequencyHz * SecondsPerHour / PulsesPerMile;
  mph *= ratio.f;

  if (mph < 0.5) mph = 0.0;
  if (mph > 120.0) mph = 120.0;

  motorStep = mphToStep(mph);
  needleStep = applyNeedleOffset(motorStep);
  updateFakeNeedleServo();
}

void updateDistance()
{
  unsigned long feetTenthsThisSecond = (unsigned long)((mph * FeetPerMile * 10.0) / SecondsPerHour);
  distsubtotalFeetTenths += feetTenthsThisSecond;
  updateodometer();
}

void updateodometer()
{
  while (distsubtotalFeetTenths >= 5280UL) {
    incrementOdometerTenth();
    incrementTripTenth();
    distsubtotalFeetTenths -= 5280UL;
  }
}

void incrementOdometerTenth()
{
  ++disttenthsm;

  if (disttenthsm > 9) {
    ++distm;
    disttenthsm = 0;
  }
  if (distm > 9) {
    ++distenm;
    distm = 0;
  }
  if (distenm > 9) {
    ++disthundredm;
    distenm = 0;
  }
  if (disthundredm > 9) {
    ++disthousandm;
    disthundredm = 0;
  }
  if (disthousandm > 9) {
    ++disttenthousandm;
    disthousandm = 0;
  }
  if (disttenthousandm > 9) {
    ++disthundredthousandm;
    disttenthousandm = 0;
  }
  if (disthundredthousandm > 9) {
    disthundredthousandm = 0;
  }
}

void incrementTripTenth()
{
  ++tripdisttenthsm;

  if (tripdisttenthsm > 9) {
    ++tripdistm;
    tripdisttenthsm = 0;
  }
  if (tripdistm > 9) {
    ++tripdistenm;
    tripdistm = 0;
  }
  if (tripdistenm > 9) {
    ++tripdisthundredm;
    tripdistenm = 0;
  }
  if (tripdisthundredm > 9) {
    tripdisthundredm = 0;
  }
}

float getOdometerMiles()
{
  unsigned long wholeMiles = 0;
  wholeMiles += (unsigned long)disthundredthousandm * 100000UL;
  wholeMiles += (unsigned long)disttenthousandm * 10000UL;
  wholeMiles += (unsigned long)disthousandm * 1000UL;
  wholeMiles += (unsigned long)disthundredm * 100UL;
  wholeMiles += (unsigned long)distenm * 10UL;
  wholeMiles += (unsigned long)distm;
  return wholeMiles + (disttenthsm / 10.0);
}

float getTripMiles()
{
  unsigned long wholeMiles = 0;
  wholeMiles += (unsigned long)tripdisthundredm * 100UL;
  wholeMiles += (unsigned long)tripdistenm * 10UL;
  wholeMiles += (unsigned long)tripdistm;
  return wholeMiles + (tripdisttenthsm / 10.0);
}

unsigned int mphToStep(float speedMph)
{
  float steps = speedMph * SpeedoDegreesPerMPH * StepsPerDegree;
  if (steps < 0) steps = 0;
  if (steps > MaxMotorSteps) steps = MaxMotorSteps;
  return (unsigned int)(steps + 0.5);
}

void displayStartupLogo()
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(30, 18);
  display.println(F("Savoy"));
  display.setTextSize(1);
  display.setCursor(38, 44);
  display.println(VERSION);
  display.display();
  delay(1200);
  display.clearDisplay();
  display.display();
}

void updateDisplay()
{
  bool calMode = (digitalRead(calSwitchPin) == LOW);
  bool offsetView = (digitalRead(encoderButtonPin) == LOW);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("MPH "));
  display.print(mph, 1);

  display.setCursor(78, 0);
  if (calMode) {
    display.print(F("CAL"));
  } else {
    display.print(displayModeName(displayMode));
  }

  display.setTextSize(2);
  display.setCursor(0, 16);

  if (calMode) {
    display.print(mph, 1);
    display.setTextSize(1);
    display.print(F(" mph"));

    display.setTextSize(1);
    display.setCursor(0, 42);
    if (offsetView) {
      display.print(F("OFFSET "));
      display.print(needleOffsetSteps);
      display.print(F(" stp"));
    } else {
      display.print(F("RATIO "));
      display.print(ratio.f, 3);
    }

    display.setCursor(0, 54);
    display.print(F("Step:"));
    display.print(motorStep);
    display.print(F(" N:"));
    display.print(needleStep);
  } else if (displayMode == MODE_ODO) {
    display.print(getOdometerMiles(), 1);
    display.setTextSize(1);
    display.print(F(" mi"));
  } else if (displayMode == MODE_TRIP) {
    display.print(getTripMiles(), 1);
    display.setTextSize(1);
    display.print(F(" trip"));
  } else if (displayMode == MODE_MPH) {
    display.print(mph, 1);
    display.setTextSize(1);
    display.print(F(" mph"));
  } else if (displayMode == MODE_RATIO) {
    display.print(ratio.f, 3);
    display.setTextSize(1);
    display.print(F(" ratio"));
  } else if (displayMode == MODE_RPM) {
    display.print(rpm, 0);
    display.setTextSize(1);
    display.print(F(" rpm"));
  }

  if (!calMode) {
    display.setTextSize(1);
    display.setCursor(0, 42);
    display.print(F("ODO:"));
    display.print(getOdometerMiles(), 1);
    display.print(F(" T:"));
    display.print(getTripMiles(), 1);

    display.setCursor(0, 54);
    display.print(F("R:"));
    display.print(ratio.f, 3);
    display.print(F(" Off:"));
    display.print(needleOffsetSteps);
  }

  display.display();
}

void updateFakeNeedleServo()
{
  int angle = (int)(((float)needleStep / (float)MaxMotorSteps) * (FakeNeedleMaxAngle - FakeNeedleMinAngle) + FakeNeedleMinAngle + 0.5);

  if (angle < FakeNeedleMinAngle) angle = FakeNeedleMinAngle;
  if (angle > FakeNeedleMaxAngle) angle = FakeNeedleMaxAngle;

  fakeNeedleServo.write(angle);
}

void servoStartupSweep()
{
  for (int angle = FakeNeedleMinAngle; angle <= FakeNeedleMaxAngle; angle += 4) {
    fakeNeedleServo.write(angle);
    delay(10);
  }

  delay(250);

  for (int angle = FakeNeedleMaxAngle; angle >= FakeNeedleMinAngle; angle -= 4) {
    fakeNeedleServo.write(angle);
    delay(10);
  }

  fakeNeedleServo.write(FakeNeedleMinAngle);
}

const __FlashStringHelper* displayModeName(uint8_t mode)
{
  switch (mode) {
    case MODE_ODO:
      return F("ODO");
    case MODE_TRIP:
      return F("TRIP");
    case MODE_MPH:
      return F("MPH");
    case MODE_RATIO:
      return F("RATIO");
    case MODE_RPM:
      return F("RPM");
    default:
      return F("?");
  }
}

void printStatusToSerial()
{
  Serial.print(F("MPH="));
  Serial.print(mph, 1);
  Serial.print(F("  Step="));
  Serial.print(motorStep);
  Serial.print(F("  Needle="));
  Serial.print(needleStep);
  Serial.print(F("  Offset="));
  Serial.print(needleOffsetSteps);
  Serial.print(F("  ODO="));
  Serial.print(getOdometerMiles(), 1);
  Serial.print(F("  TRIP="));
  Serial.print(getTripMiles(), 1);
  Serial.print(F("  Ratio="));
  Serial.print(ratio.f, 3);
  Serial.print(F("  Mode="));
  Serial.print(displayModeName(displayMode));
  Serial.print(F("  Cal="));
  Serial.println(digitalRead(calSwitchPin) == LOW ? F("YES") : F("NO"));
}
