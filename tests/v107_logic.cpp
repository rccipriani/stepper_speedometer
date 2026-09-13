// Execute production logic with fake time, GPIO, motor and FRAM.
#include "firmware.ino"
#include <cassert>
#include <iostream>

void fresh() {
  fram.memory.fill(0);
  fram.writesBeforeCut = -1;
  memset(&state, 0, sizeof(state));
  activeSlot = 1;
  storageDirty = false;
  vssPulses = consumedPulses = vssLastUs = vssPeriodUs = 0;
  vssSeen = false;
  clockUs = clockMs = 0;
  pinLevels.fill(HIGH);
  PINC = 1;
  encoderPending = encoderQuarterSteps = encoderPrevious = 0;
  encoderButton = {HIGH,HIGH,false,0,0};
  modeButton = {HIGH,HIGH,false,0,0};
  loadState();
}

void setEncoder(uint8_t pins) {
  PIND = (pins & 2) ? _BV(PD3) : 0;
  PINB = (pins & 1) ? _BV(PB4) : 0;
  onEncoderChange();
}

int main() {
  fresh();
  assert(state.ratioMilli == 1000 && state.odoTenths == 0);
  addDistance(399);
  assert(state.odoTenths == 0 && state.odoFraction == 399000);
  addDistance(1);
  assert(state.odoTenths == 1 && state.odoFraction == 0);
  // Million pulses at each calibration extreme and exact fixed-point fractions.
  for (uint16_t ratio : {500,1001,2000}) {
    fresh(); state.ratioMilli = ratio;
    addDistance(1000001);
    uint64_t expected = uint64_t(1000001) * ratio;
    assert(state.odoTenths == expected / UnitsPerTenth);
    assert(state.odoFraction == expected % UnitsPerTenth);
  }
  fresh(); state.odoTenths=9999999; state.tripTenths=9999;
  addDistance(400);
  assert(state.odoTenths == 0 && state.tripTenths == 0);
  // Counter wrap must not lose or invent pulses.
  fresh(); consumedPulses=0xFFFFFFFE; vssPulses=2;
  collectDistance(); assert(state.odoFraction == 4000);
  std::cout << "PASS distance: fractions, calibration, rollover, counter wrap\n";

  fresh(); clockUs=100; onVssPulse();
  clockUs=200; updateSpeedometer(); assert(mph == 0); // first edge is not a period
  clockUs=150100; onVssPulse(); updateSpeedometer();
  assert(std::abs(mph-6.0f)<0.001f);
  clockUs+=300000; onVssPulse(); updateSpeedometer();
  assert(std::abs(mph-3.0f)<0.001f); // creeping is no longer discarded
  clockUs+=7500; onVssPulse(); updateSpeedometer();
  assert(std::abs(mph-120.0f)<0.001f && Motor.target == 510);
  state.offset=-60; updateSpeedometer(); assert(Motor.target==510);
  clockUs+=StopTimeoutUs+1; updateSpeedometer(); assert(mph == 0 && !vssSeen);
  clockUs=vssLastUs+100; updateSpeedometer(); assert(mph == 0); // no wrap revival
  fresh(); clockUs=0xFFFFFF00; onVssPulse();
  clockUs+=15000; onVssPulse(); updateSpeedometer();
  assert(std::abs(mph-60.0f)<0.001f);
  std::cout << "PASS speed: first pulse, low/high speed, timeout, micros wrap\n";

  fresh(); PINC=0;
  for (int cycle=0;cycle<20;++cycle) {
    for (uint8_t pins : {2,3,1,0}) setEncoder(pins);
  }
  assert(encoderPending == 20);
  for (int cycle=0;cycle<20;++cycle) {
    for (uint8_t pins : {1,3,2,0}) setEncoder(pins);
  }
  assert(encoderPending == 0);
  for (uint8_t pins : {2,0,2,3,2,3,1,3,1,0}) setEncoder(pins);
  assert(encoderPending == 1); // bouncing contacts, one completed cycle
  encoderPending=0;
  for (uint8_t pins : {2,1,0}) setEncoder(pins); // invalid two-bit transition
  assert(encoderPending == 0);
  PINC=1;
  for (uint8_t pins : {2,3,1,0}) setEncoder(pins);
  assert(encoderPending == 0);
  std::cout << "PASS encoder: both directions, bounce, invalid transition, harness\n";

  fresh();
  Button button={HIGH,HIGH,false,0,0};
  pinLevels[A1]=LOW; assert(pollButton(button,A1,0)==0);
  assert(pollButton(button,A1,35)==0);
  pinLevels[A1]=HIGH; assert(pollButton(button,A1,100)==0);
  assert(pollButton(button,A1,135)==1);
  pinLevels[A1]=LOW; pollButton(button,A1,200); pollButton(button,A1,235);
  assert(pollButton(button,A1,1235)==2);
  assert(pollButton(button,A1,1300)==0);
  pinLevels[A1]=HIGH; pollButton(button,A1,1400);
  assert(pollButton(button,A1,1435)==0);
  // Reset through the actual control handler, preserving the main fraction.
  displayMode=1; addDistance(123);
  pinLevels[A2]=LOW; handleControls(0); handleControls(35); handleControls(1035);
  assert(state.tripTenths==0 && state.tripFraction==0);
  assert(state.odoFraction==123000);
  std::cout << "PASS buttons: short/long press, no release toggle, trip remainder\n";

  fresh();
  addDistance(123); saveState();
  auto baseline=fram.memory;
  auto previous=state;
  uint8_t previousSlot=activeSlot;
  // Cut power before every individual FRAM write in a new checkpoint.
  // Until the last commit byte lands, boot MUST select the complete old record.
  for (int cutoff=0;cutoff<=34;++cutoff) {
    fram.memory=baseline; state=previous; activeSlot=previousSlot;
    state.odoTenths=1234567; state.tripTenths=3456; state.odoFraction=1;
    fram.writesBeforeCut=cutoff;
    try { saveState(); } catch (PowerCut&) {}
    fram.writesBeforeCut=-1;
    loadState();
    assert(state.odoTenths == (cutoff==34 ? 1234567U : previous.odoTenths));
    assert(state.tripTenths == (cutoff==34 ? 3456U : previous.tripTenths));
    assert(state.odoFraction == (cutoff==34 ? 1U : previous.odoFraction));
  }
  // Every single-bit error anywhere in a record must be rejected by CRC/range.
  StoredState valid=state;
  for (size_t byte=0;byte<sizeof(valid);++byte) {
    for (uint8_t bit=0;bit<8;++bit) {
      StoredState bad=valid;
      reinterpret_cast<uint8_t*>(&bad)[byte] ^= 1U<<bit;
      assert(!validRecord(bad));
    }
  }
  // Journal sequence wrap selects the newly committed record.
  fresh(); state.sequence=0xFFFFFFFE; saveState();
  state.odoTenths=42; saveState(); loadState();
  assert(state.sequence==0 && state.odoTenths==42);
  // Losing both migrated records must not silently restore old mileage.
  fram.memory[SlotAddress[0]+sizeof(state)]=0;
  fram.memory[SlotAddress[1]+sizeof(state)]=0;
  bool halted=false;
  try { loadState(); } catch (FakeFault&) { halted=true; }
  assert(halted);
  std::cout << "PASS FRAM: all 35 cut points, all 256 bit flips, sequence wrap, fail closed\n";

  // Import the exact little-endian v1.06 legacy layout, then keep it untouched.
  fresh(); fram.memory.fill(0); activeSlot=1;
  fram.memory[0]=4; fram.memory[1]=3; fram.memory[2]=2; fram.memory[3]=1;
  for (int i=0;i<7;++i) fram.memory[4+i]=i+1;
  float legacy=1.234f; memcpy(&fram.memory[0x20],&legacy,4);
  int16_t legacyOffset=-12; memcpy(&fram.memory[0x30],&legacyOffset,2);
  auto legacyBytes=fram.memory;
  loadState();
  assert(state.odoTenths==7654321 && state.tripTenths==1234);
  assert(state.ratioMilli==1234 && state.offset==-12);
  for (int i=0;i<=0x31;++i) assert(fram.memory[i]==legacyBytes[i]);
  loadState(); assert(state.odoTenths==7654321 && state.ratioMilli==1234);
  std::cout << "PASS legacy migration: digits, ratio, signed offset, non-destructive restart\n";
}
