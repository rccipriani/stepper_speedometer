// Extracted from v1.12; included once by Application.cpp.
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

// Speed measurement/needle operation do not depend on mileage persistence.
// A known reading alone is insufficient after a latched FRAM failure.
bool mileagePersistenceOperational() {
  return mileageKnown && !storageFailed && fram.healthy();
}

void collectDistance() {
  uint32_t total = vehicleSpeed.totalPulses();
  if (mileagePersistenceOperational()) addDistance(total - consumedPulses);
  // Discard unpersistable pulses, including across total-counter rollover.
  // Never leave a backlog that could later be counted as saved mileage.
  consumedPulses = total;
}

void updateSpeedometer() {
  mph = vehicleSpeed.getMph(state.ratioMilli);
  speedometer.setSpeed(mph, state.offset);
  motorStep = speedometer.targetPosition();
}

