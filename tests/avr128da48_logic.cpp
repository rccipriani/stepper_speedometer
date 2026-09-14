#include "Application.cpp"
#include "SwitecImplementation.h"
#include <cassert>
#include <iostream>

void fresh() {
  Wire = FakeWire{}; fram = FramDevice{}; assert(fram.begin());
  storageFailed=false; mileageKnown=false; storageDirty=false;
  activeSlot=activeLayoutSlot=1;
  vehicleSpeed=VehicleSpeed{}; consumedPulses=0;
  clockUs=clockMs=0; pinLevels.fill(HIGH);
  memset(&state,0,sizeof(state));
  loadState(); loadLayout();
  calibrationInputsValid=false; encoderPending=encoderQuarterSteps=0;
  encoderButton={HIGH,HIGH,false,0,0}; modeButton=encoderButton;
  harnessWasPresent=false; calibrationTarget=CAL_RATIO;
  layoutInfo=layoutHint=false;
}
void encoder(uint8_t bits) {
  pinLevels[encoderA]=(bits&2)!=0; pinLevels[encoderB]=(bits&1)!=0;
  serviceCalibrationInputs();
}
int main() {
  const uint8_t assigned[] = {speedPulsePin,maxSecondChannelPin,motor1,motor2,motor3,motor4,
    oledMosi,oledMiso,oledSck,oledCs,oledDc,oledReset,framSda,framScl,
    encoderA,encoderB,encoderKey,calSwitchPin,modeButtonPin,rpmPulsePin,afrInputPin,dimmerInputPin};
  for (unsigned i=0;i<sizeof(assigned);++i) {
    assert(assigned[i]!=PIN_PC0 && assigned[i]!=PIN_PC1 && assigned[i]!=PIN_PC6 && assigned[i]!=PIN_PC7);
    assert(assigned[i]!=PIN_PF6);
    for (unsigned j=0;j<i;++j) assert(assigned[i]!=assigned[j]);
  }
  fresh(); assert(mileageKnown && !storageFailed && sizeof(state)==32 && sizeof(layout)==17);
  for (uint16_t ratio : {500,1001,2000}) {
    fresh(); state.ratioMilli=ratio; addDistance(1000001);
    uint64_t units=uint64_t(1000001)*ratio;
    assert(state.odoTenths==units/UnitsPerTenth && state.odoFraction==units%UnitsPerTenth);
  }
  fresh(); state.odoTenths=9999999; state.tripTenths=9999; addDistance(400);
  assert(state.odoTenths==0 && state.tripTenths==0);
  consumedPulses=0xFFFFFFFE; collectDistance(); assert(state.odoFraction==2000);
  fresh(); clockUs=100; onVssPulse(); assert(vehicleSpeed.getMph()==0);
  clockUs=150100; onVssPulse(); assert(std::abs(vehicleSpeed.getMph()-6)<0.001);
  assert(std::abs(vehicleSpeed.getMph(2000)-12)<0.001);
  clockUs=450100; assert(std::abs(vehicleSpeed.getMph()-3)<0.001);
  clockUs=3150101; assert(vehicleSpeed.getMph()==0);
  clockUs=150101; assert(vehicleSpeed.getMph()==0); // no stale revival
  vehicleSpeed=VehicleSpeed{}; clockUs=0xFFFFFF00; onVssPulse();
  clockUs+=150000; onVssPulse(); assert(std::abs(vehicleSpeed.getMph()-6)<0.001);
  speedometer.setSpeed(200,-60); assert(speedometer.targetPosition()==510);
  speedometer.setSpeed(0,-60); assert(speedometer.targetPosition()==0);
  speedometer.setSpeed(10,2); assert(speedometer.targetPosition()==53);
  std::cout<<"PASS distance, pulse aging/wrap, calibration and gauge mapping\n";

  fresh(); state.odoTenths=123456; state.tripTenths=456; storageDirty=true; saveState();
  auto durable=Wire.memory;
  for (int cut=0;cut<=34;++cut) {
    fresh(); Wire.memory=durable; loadState();
    state.odoTenths=123457; storageDirty=true; Wire.writesBeforeCut=cut;
    try { saveState(); } catch (int) {}
    Wire.writesBeforeCut=-1; loadState();
    assert(state.odoTenths==123456 || state.odoTenths==123457);
  }
  fresh(); Wire.memory=durable; loadState();
  auto oldSlot=activeSlot; state.odoTenths++; storageDirty=true;
  Wire.writesBeforeFailure=5; saveState();
  assert(storageFailed && storageDirty && activeSlot==oldSlot);
  unsigned tx=Wire.transactions; saveState(); assert(Wire.transactions==tx);
  clockUs=100; onVssPulse(); clockUs=150100; onVssPulse(); updateSpeedometer();
  assert(mph>0); // failed storage cannot stop needle logic
  fresh(); Wire.shortRead=true; loadState(); assert(storageFailed && !fram.healthy());
  fresh(); Wire.memory.fill(0); Wire.memory[MigrationAddress]=CommitMarker;
  auto corrupt=Wire.memory; mileageKnown=false; loadState();
  assert(storageFailed && !mileageKnown && Wire.memory==corrupt);
  std::cout<<"PASS journal power cuts, write errors, short reads, corrupt migration protection\n";

  fresh(); layout.x=17; layoutDirty=true; saveLayout();
  auto savedLayout=Wire.memory;
  for (int cut=0;cut<=19;++cut) {
    fresh(); Wire.memory=savedLayout; loadLayout();
    layout.x=18; layoutDirty=true; Wire.writesBeforeCut=cut;
    try { saveLayout(); } catch (int) {}
    Wire.writesBeforeCut=-1; loadLayout();
    assert(layout.x==17 || layout.x==18);
  }
  fresh(); Wire.memory.fill(0); Wire.memory[4]=3; Wire.memory[5]=2;
  union { float value; uint8_t bytes[4]; } ratio{1.25f};
  for (int i=0;i<4;++i) Wire.memory[0x20+i]=ratio.bytes[i];
  Wire.memory[0x30]=0xF6; Wire.memory[0x31]=0xFF;
  auto legacy=Wire.memory; loadState();
  assert(state.odoTenths==23 && state.ratioMilli==1250 && state.offset==-10);
  for (int i=0;i<0x32;++i) assert(Wire.memory[i]==legacy[i]);
  std::cout<<"PASS layout power cuts and unchanged legacy migration\n";

  fresh(); pinLevels[calSwitchPin]=LOW; encoder(0);
  unsigned before=Wire.transactions;
  encoder(2); encoder(3); encoder(1); encoder(0);
  assert(encoderPending==1 && Wire.transactions==before);
  handleControls(0); assert(state.ratioMilli==1001);
  encoder(3); assert(encoderQuarterSteps==0); // impossible jump
  pinLevels[calSwitchPin]=HIGH; serviceCalibrationInputs();
  assert(!calibrationInputsValid && encoderPending==0);
  layout.digits=7; display.labels.clear(); drawOdometer();
  assert(display.labels.find("FIT")!=std::string::npos);
  std::cout<<"PASS native encoder, calibration and odometer fit guard\n";

  fresh(); speedometer.home(); speedometer.setSpeed(10);
  for (int i=0;i<509;++i) { clockUs+=800; speedometer.update(); }
  assert(!speedometer.ready());
  // Run actual Switec acceleration and the two startup holds, with no delays.
  for (int i=0;i<20000 && !speedometer.ready();++i) {
    clockUs+=800; clockMs=clockUs/1000; speedometer.update();
  }
  assert(speedometer.ready() && speedometer.targetPosition()==51);
  fresh(); mileageKnown=false; Wire.present=false; setup();
  assert(storageFailed && !mileageKnown && state.ratioMilli==1000);
  clockMs=100; loop(); // missing FRAM returns to scheduler
  std::cout<<"PASS cooperative homing/sweep and missing-FRAM startup\n";
}
