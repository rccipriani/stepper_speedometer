#include "ApplicationState.h"

void saveLayout();
void collectDistance();
void serviceFastInputs();
void saveState();
#include "VehicleSpeed.h"
VehicleSpeed vehicleSpeed;
void onVssPulse() { vehicleSpeed.onPulse(); }

#include "VehicleIO.h"

#include "Storage.h"

#include "Distance.h"

#include "Controls.h"

#include "Display.h"

// Service mileage during the startup sweep/delays as well as normal operation.
void serviceMotion() {
  serviceFastInputs();
  collectDistance();
  uint32_t now = millis();
  if (storageDirty && now - lastCheckpoint >= CheckpointMs) {
    saveState();
    lastCheckpoint = millis();
  }
}

#include "AnalogInputs.h"

void setup() {
  Watchdog::begin();
#if DEBUG_SERIAL
  Serial3.begin(115200); // EV35L43A CDC UART: PB0 TX / PB1 RX
#endif
  beginVehicleIO();
  vehicleSpeed.begin(onVssPulse);
  speedometer.home();
  beginStorage();
  beginDisplay();
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
  }
  Watchdog::feed();
}
