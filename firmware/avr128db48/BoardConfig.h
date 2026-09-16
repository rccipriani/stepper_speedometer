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
constexpr uint8_t oledDc = PIN_PE0, oledReset = PIN_PE1;
constexpr uint8_t framSda = PIN_PA2, framScl = PIN_PA3, framAddress = 0x50;
constexpr uint8_t encoderA = PIN_PC4, encoderB = PIN_PC5, encoderKey = PIN_PE2;
constexpr uint8_t calSwitchPin = PIN_PE3, modeButtonPin = PIN_PD1;
constexpr uint8_t rpmPulsePin = PIN_PD2;
constexpr uint8_t afrInputPin = PIN_PD3, dimmerInputPin = PIN_PD4;
}
