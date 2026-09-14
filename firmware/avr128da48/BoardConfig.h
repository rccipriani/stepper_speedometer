#pragma once
#include <Arduino.h>

// Proposed harness, expressed as MCU port names (not Nano pin numbers).
// SPI0 default and TWI0 default. Keep PC0/PC1 for the board's CDC UART,
// PC6/PC7 for its LED/button, and PF6/PF7 for reset/UPDI.
namespace Board {
constexpr uint8_t speedPulsePin = PIN_PC2, maxSecondChannelPin = PIN_PC3;
constexpr uint8_t motor1 = PIN_PB0, motor2 = PIN_PB1;
constexpr uint8_t motor3 = PIN_PB2, motor4 = PIN_PB3;
constexpr uint8_t oledMosi = PIN_PA4, oledMiso = PIN_PA5;
constexpr uint8_t oledSck = PIN_PA6, oledCs = PIN_PA7;
constexpr uint8_t oledDc = PIN_PE0, oledReset = PIN_PE1;
constexpr uint8_t framSda = PIN_PA2, framScl = PIN_PA3, framAddress = 0x50;
constexpr uint8_t encoderA = PIN_PC4, encoderB = PIN_PC5, encoderKey = PIN_PE2;
constexpr uint8_t calSwitchPin = PIN_PE3, modeButtonPin = PIN_PD1;
constexpr uint8_t rpmPulsePin = PIN_PD2;
constexpr uint8_t afrInputPin = PIN_PD3, dimmerInputPin = PIN_PD4;
}
