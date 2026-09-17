#pragma once
#include <Arduino.h>

// EV35L43A harness, using DxCore 48pin-standard MCU port constants.
// SPI0/TWI0 default routing. PB0/PB1 are CDC; PB2/PB3 are button/LED.
// X27 moves off PB0..PB3 to PC0/PC1/PC6/PC7, preserving driver order.
// Keep R204 fitted: PORTC VDDIO2 = VDD (5 V), with MVIO disabled.
// PF6/PF7 remain reset/UPDI; PA0/PA1 and PF0/PF1 remain crystal pins.
namespace Board {
constexpr uint8_t speedPulsePin = PIN_PC2, maxSecondChannelPin = PIN_PC3;
constexpr uint8_t motor1 = PIN_PC0, motor2 = PIN_PC1;
constexpr uint8_t motor3 = PIN_PC6, motor4 = PIN_PC7;
constexpr uint8_t oledMosi = PIN_PA4, oledMiso = PIN_PA5;
constexpr uint8_t oledSck = PIN_PA6, oledCs = PIN_PA7;
constexpr uint8_t oledDc = PIN_PE0, oledReset = PIN_PB4;
constexpr uint8_t framSda = PIN_PA2, framScl = PIN_PA3, framAddress = 0x50;
constexpr uint8_t encoderA = PIN_PC4, encoderB = PIN_PC5, encoderKey = PIN_PB5;
constexpr uint8_t calSwitchPin = PIN_PF4, modeButtonPin = PIN_PF5;
constexpr uint8_t rpmPulsePin = PIN_PF2;
constexpr uint8_t afrInputPin = PIN_PD0, dimmerInputPin = PIN_PF3;

// Include future reservations: all three external OPAMP groups must stay free.
constexpr bool pinAssigned(uint8_t pin) {
  return pin == speedPulsePin || pin == maxSecondChannelPin ||
    pin == motor1 || pin == motor2 || pin == motor3 || pin == motor4 ||
    pin == oledMosi || pin == oledMiso || pin == oledSck || pin == oledCs ||
    pin == oledDc || pin == oledReset || pin == framSda || pin == framScl ||
    pin == encoderA || pin == encoderB || pin == encoderKey ||
    pin == calSwitchPin || pin == modeButtonPin || pin == rpmPulsePin ||
    pin == afrInputPin || pin == dimmerInputPin || pin == PIN_PB0 || pin == PIN_PB1;
}
static_assert(!pinAssigned(PIN_PD1) && !pinAssigned(PIN_PD2) && !pinAssigned(PIN_PD3),
              "Reserve OPAMP0 INP/OUT/INN");
static_assert(!pinAssigned(PIN_PD4) && !pinAssigned(PIN_PD5) && !pinAssigned(PIN_PD7),
              "Reserve OPAMP1 INP/OUT/INN");
static_assert(!pinAssigned(PIN_PE1) && !pinAssigned(PIN_PE2) && !pinAssigned(PIN_PE3),
              "Reserve OPAMP2 INP/OUT/INN (PE1 is INP, PE2 is OUT)");
}
