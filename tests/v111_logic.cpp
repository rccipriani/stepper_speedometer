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
  loadLayout();
  harnessWasPresent=false;
  calibrationTarget=CAL_RATIO;
  layoutInfo=layoutHint=false;
  display.glyphs.clear(); display.labels.clear();
  afr=0; afrValid=false; lastAnalogUpdate=0;
  dimmerInitialized=false; dimmerFilteredAdc=0; lastDimmerContrast=DimmerContrastMax;
  analogLevels.fill(0); analogReadCalls=0; analogQueue.clear(); analogQueueIndex=0;
  display.contrasts.clear();
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

  fresh();
  assert(layout.x==5 && layout.y==38 && layout.font==DefaultDigitFont && layout.digits==10);
  auto mileageBefore=state;
  // Each adjustment is isolated from mileage/ratio/offset and clamps safely.
  calibrationTarget=CAL_X; applyLayoutAdjustment(100,100); assert(layout.x==105);
  applyLayoutAdjustment(100,200); applyLayoutAdjustment(100,300); assert(layout.x==255);
  for (int i=0;i<3;++i) applyLayoutAdjustment(-100,400); assert(layout.x==0);
  calibrationTarget=CAL_Y; applyLayoutAdjustment(100,500); assert(layout.y==63);
  applyLayoutAdjustment(-100,600); assert(layout.y==0);
  calibrationTarget=CAL_SIZE; applyLayoutAdjustment(100,700); assert(layout.font==DigitFontCount-1);
  applyLayoutAdjustment(-100,800); assert(layout.font==0);
  calibrationTarget=CAL_DIGITS; applyLayoutAdjustment(100,900); assert(layout.digits==40);
  applyLayoutAdjustment(-100,1000); assert(layout.digits==1);
  assert(memcmp(&mileageBefore,&state,sizeof(state))==0);
  std::cout << "PASS layout controls: defaults, all four limits, isolated mileage\n";

  // Pattern repeats exactly, same baseline/font/spacing as actual numbers.
  layout.x=5; layout.y=38; layout.font=0; layout.digits=12;
  layoutInfo=layoutHint=false; display.glyphs.clear(); display.labels.clear();
  drawLayoutCalibration();
  assert(display.glyphs.size()==12 && display.labels.empty());
  for (size_t i=0;i<12;++i) {
    const auto &glyph=display.glyphs[i];
    assert(glyph.digit=='0'+i%10 && glyph.x==5+i*8 && glyph.y==38);
    assert(glyph.font==DigitFonts[0]);
  }
  layout.digits=8; display.glyphs.clear(); drawLayoutCalibration();
  auto preview=display.glyphs;
  display.glyphs.clear(); printTenths(9999999);
  assert(display.glyphs.size()==8 && display.glyphs[6].digit=='.');
  for (size_t i=0;i<8;++i) {
    assert(display.glyphs[i].x==preview[i].x && display.glyphs[i].y==preview[i].y);
    assert(display.glyphs[i].font==preview[i].font);
  }
  layout.x=250; layout.digits=40; display.glyphs.clear(); drawLayoutCalibration();
  assert(display.glyphs.size()==1 && display.glyphs[0].x==250); // no 8-bit wrap
  layoutInfo=true; display.glyphs.clear(); display.labels.clear(); drawLayoutCalibration();
  assert(display.glyphs.empty() && display.labels.find("DIGITS")!=std::string::npos);
  std::cout << "PASS rendering: repeat/count, normal-reading agreement, OLED clipping, separate readout\n";

  // Harness KEY cycles all six targets; a long press toggles only the readout.
  fresh(); pinLevels[A0]=LOW; PINC=0; handleControls(0);
  uint32_t t=100;
  for (int target=1;target<=6;++target) {
    pinLevels[A1]=LOW; handleControls(t); handleControls(t+35);
    pinLevels[A1]=HIGH; handleControls(t+100); handleControls(t+135);
    assert(calibrationTarget==target%6);
    t+=200;
  }
  calibrationTarget=CAL_DIGITS;
  pinLevels[A1]=LOW; handleControls(t); handleControls(t+35); handleControls(t+1035);
  assert(layoutInfo && calibrationTarget==CAL_DIGITS);
  pinLevels[A1]=HIGH; handleControls(t+1100); handleControls(t+1135);
  assert(layoutInfo && calibrationTarget==CAL_DIGITS);
  // Mode/trip button cannot reset trip while the harness is active.
  displayMode=1; state.tripTenths=456;
  pinLevels[A2]=LOW; handleControls(t+1200); handleControls(t+1235); handleControls(t+2235);
  assert(state.tripTenths==456 && displayMode==1);
  // Autosave uses rollover-safe elapsed time; unplug also flushes pending work.
  calibrationTarget=CAL_X; applyLayoutAdjustment(1,0xFFFFFF00);
  handleControls(0xFFFFFF64); assert(layoutDirty);
  handleControls(0x00000300); assert(!layoutDirty);
  calibrationTarget=CAL_DIGITS; applyLayoutAdjustment(-3,2000); assert(layout.digits==7);
  pinLevels[A0]=HIGH; handleControls(2001); assert(!layoutDirty && calibrationTarget==CAL_RATIO);
  loadLayout(); assert(layout.digits==7 && layout.x==6);
  std::cout << "PASS harness: six targets, long press, trip protection, autosave wrap, unplug save\n";

  fresh(); layoutDirty=true; saveLayout();
  baseline=fram.memory; auto originalLayout=layout; uint8_t originalLayoutSlot=activeLayoutSlot;
  // Independent layout journal: every interrupted write retains old settings.
  for (int cutoff=0;cutoff<=int(sizeof(DisplayLayout))+2;++cutoff) {
    fram.memory=baseline; layout=originalLayout; activeLayoutSlot=originalLayoutSlot;
    layout.x=80; layout.y=50; layout.font=5; layout.digits=8; layoutDirty=true;
    fram.writesBeforeCut=cutoff;
    try { saveLayout(); } catch (PowerCut&) {}
    fram.writesBeforeCut=-1; loadLayout();
    bool completed=cutoff==int(sizeof(DisplayLayout))+2;
    assert(layout.x==(completed?80:5) && layout.y==(completed?50:38));
    assert(layout.font==(completed?5:DefaultDigitFont) && layout.digits==(completed?8:10));
    for (size_t address=0;address<0x100;++address) assert(fram.memory[address]==baseline[address]);
  }
  auto goodLayout=layout;
  for (size_t byte=0;byte<sizeof(DisplayLayout);++byte) {
    for (uint8_t bit=0;bit<8;++bit) {
      auto bad=goodLayout;
      reinterpret_cast<uint8_t*>(&bad)[byte]^=1U<<bit;
      assert(!validLayout(bad));
    }
  }
  auto bad=goodLayout; bad.font=DigitFontCount; bad.crc=layoutCrc(bad); assert(!validLayout(bad));
  bad=goodLayout; bad.digits=0; bad.crc=layoutCrc(bad); assert(!validLayout(bad));
  bad=goodLayout; bad.x=256; bad.crc=layoutCrc(bad); assert(!validLayout(bad));
  bad=goodLayout; bad.y=64; bad.crc=layoutCrc(bad); assert(!validLayout(bad));
  layout.sequence=0xFFFFFFFE; layoutDirty=true; saveLayout();
  layout.x=42; layoutDirty=true; saveLayout(); loadLayout(); assert(layout.x==42 && layout.sequence==0);
  fram.memory[LayoutSlotAddress[0]+sizeof(layout)]=0;
  fram.memory[LayoutSlotAddress[1]+sizeof(layout)]=0;
  loadLayout(); assert(layout.x==5 && layout.y==38 && layout.font==DefaultDigitFont && layout.digits==10);
  loadState(); assert(state.odoTenths==0 && state.ratioMilli==1000);
  std::cout << "PASS layout FRAM: all 20 cut points, 136 bit flips, validation, wrap, safe defaults\n";

  // ODO must reserve its full lifetime range, even when current mileage is low.
  fresh(); layout.font=0; layout.x=5; layout.y=38; layout.digits=7;
  state.odoTenths=12; displayMode=0;
  auto savedMileage=state;
  display.glyphs.clear(); display.labels.clear(); drawScreen(false);
  assert(display.glyphs.empty() && display.labels.find("FIT")!=std::string::npos);
  assert(memcmp(&state,&savedMileage,sizeof(state))==0);
  layout.digits=8; state.odoTenths=9999999;
  display.glyphs.clear(); display.labels.clear(); drawScreen(false);
  assert(display.glyphs.size()==8 && display.glyphs[6].digit=='.');
  assert(display.labels.find("FIT")==std::string::npos);
  // Exact right edge is allowed; a one-pixel overflow blocks the ENTIRE value.
  layout.x=192; display.glyphs.clear(); drawOdometer(); assert(display.glyphs.size()==8);
  layout.x=193; display.glyphs.clear(); display.labels.clear(); drawOdometer();
  assert(display.glyphs.empty() && display.labels=="FIT");
  // Vertical clipping is rejected too; a larger font cannot bypass the check.
  layout.x=5; layout.y=10; display.glyphs.clear(); display.labels.clear(); drawOdometer();
  assert(display.glyphs.empty() && display.labels=="FIT");
  layout.y=11; display.glyphs.clear(); drawOdometer(); assert(display.glyphs.size()==8);
  layout.font=DigitFontCount-1; layout.y=38; display.glyphs.clear(); drawOdometer();
  assert(display.glyphs.empty());
  // Narrow capacities get an unambiguous symbol, never a truncated number.
  layout.font=0; layout.y=38; layout.digits=1;
  display.glyphs.clear(); display.labels.clear(); drawOdometer();
  assert(display.glyphs.empty() && display.labels=="!");
  // Shared guard covers trip, MPH and ratio as well as the odometer.
  layout.digits=4; display.glyphs.clear(); display.labels.clear(); drawNumericText("12.34");
  assert(display.glyphs.empty() && display.labels=="FIT");
  display.glyphs.clear(); drawNumericText("12.3"); assert(display.glyphs.size()==4);
  // Fitting preview stays usable at invalid settings; readout explains FIT8.
  calibrationTarget=CAL_DIGITS; layoutHint=false; layoutInfo=false; layout.digits=7;
  display.glyphs.clear(); display.labels.clear(); drawLayoutCalibration();
  assert(display.glyphs.size()==7 && display.labels.empty());
  layoutInfo=true; display.glyphs.clear(); display.labels.clear(); drawLayoutCalibration();
  assert(display.glyphs.empty() && display.labels.find("FIT8")!=std::string::npos);
  layout.digits=8; display.labels.clear(); drawLayoutCalibration();
  assert(display.labels.find("FIT8")==std::string::npos);
  std::cout << "PASS enforced fit: full odometer range, exact edge, vertical/font limits, no partial values, FIT8\n";

  fresh(); displayMode=0;
  uint32_t pressTime=100;
  for (int expected=1;expected<=6;++expected) {
    pinLevels[A2]=LOW; handleControls(pressTime); handleControls(pressTime+35);
    pinLevels[A2]=HIGH; handleControls(pressTime+100); handleControls(pressTime+135);
    assert(displayMode==expected%6);
    pressTime+=200;
  }
  displayMode=5; display.labels.clear(); display.glyphs.clear(); drawScreen(false);
  assert(display.labels.find("AFR")!=std::string::npos);
  assert(display.glyphs.size()==2 && display.glyphs[0].digit=='-' && display.glyphs[1].digit=='-');
  assert(afrInputPin==A6 && dimmerInputPin==A7 && rpmPulsePin==A3);
#if AFR_INPUT_ENABLED && DIMMER_INPUT_ENABLED
  analogLevels[A6]=512; analogLevels[A7]=DimmerAdcDark;
  updateAnalogInputs(99); assert(analogReadCalls==0);
  updateAnalogInputs(100);
  assert(analogReadCalls==10 && afrValid && std::abs(afr-15.0f)<0.02f);
  assert(display.contrasts.back()==DimmerContrastMin);
  display.glyphs.clear(); drawScreen(false);
  std::string afrText;
  for (const auto &glyph:display.glyphs) afrText+=glyph.digit;
  assert(afrText=="15.0");
  analogLevels[A6]=0; updateAnalogInputs(200); assert(!afrValid);
  display.glyphs.clear(); drawScreen(false); assert(display.glyphs.size()==2);
  analogLevels[A6]=1023; updateAnalogInputs(300); assert(!afrValid);
  analogLevels[A6]=512; updateAnalogInputs(400); assert(afrValid);
  assert(contrastFromAdc(DimmerAdcDark)==DimmerContrastMin);
  assert(contrastFromAdc(DimmerAdcBright)==DimmerContrastMax);
  uint8_t half=contrastFromAdc(512); assert(half>=134 && half<=136);
  analogLevels[A7]=DimmerAdcBright; updateAnalogInputs(500);
  assert(lastDimmerContrast>DimmerContrastMin && lastDimmerContrast<DimmerContrastMax);
  for (uint32_t ms=600;ms<=6000;ms+=100) updateAnalogInputs(ms);
  assert(lastDimmerContrast>=DimmerContrastMax-1);
  auto writes=display.contrasts.size(); updateAnalogInputs(6100);
  assert(display.contrasts.size()==writes);
  lastAnalogUpdate=0xFFFFFFE0; updateAnalogInputs(0x00000080);
  assert(lastAnalogUpdate==0x80);
  std::cout << "PASS enabled analog fixture: AFR curve/range, dimmer polarity/filter, timing/wrap\n";
#else
  updateAnalogInputs(100); updateAnalogInputs(200);
  assert(analogReadCalls==0 && !afrValid && display.contrasts.empty());
  std::cout << "PASS default analog inputs disabled: no floating readings or brightness changes\n";
#endif
  analogQueue={999,100,200,300,400}; analogQueueIndex=0;
  assert(readAnalogAverage(A6)==250 && analogQueueIndex==5);
  std::cout << "PASS AFR mode cycle, pin reservations, ADC dummy conversion/averaging\n";
}
