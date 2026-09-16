// Extracted from v1.12; included once by Application.cpp.
uint16_t bytesCrc(const uint8_t *bytes, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)bytes[i] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

uint16_t recordCrc(const StoredState &record) {
  return bytesCrc((const uint8_t *)&record, offsetof(StoredState, crc));
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

void storageFault() { storageFailed = true; }

void saveState() {
  if (storageFailed || !fram.healthy()) { storageFault(); return; }
  uint8_t nextSlot = activeSlot ^ 1;
  uint16_t address = SlotAddress[nextSlot];
  ++state.sequence;
  state.crc = recordCrc(state);
  fram.write(address + sizeof(state), 0); // invalidate destination FIRST
  if (fram.read(address + sizeof(state)) != 0 || !fram.healthy()) { storageFault(); return; }
  const uint8_t *bytes = (const uint8_t *)&state;
  for (uint8_t i = 0; i < sizeof(state); ++i) {
    fram.write(address + i, bytes[i]);
    serviceFastInputs();
  }
  // Verify payload before publishing the commit marker.
  for (uint8_t i = 0; i < sizeof(state); ++i) {
    if (fram.read(address + i) != bytes[i] || !fram.healthy()) { storageFault(); return; }
    serviceFastInputs();
  }
  fram.write(address + sizeof(state), CommitMarker);
  if (fram.read(address + sizeof(state)) != CommitMarker || !fram.healthy()) { storageFault(); return; }
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
  if (!fram.healthy()) { storageFault(); return; }
  if (secondValid && (!firstValid || (int32_t)(other.sequence - state.sequence) > 0)) {
    state = other;
    activeSlot = 1;
  } else if (firstValid) {
    activeSlot = 0;
  } else {
    if (fram.read(MigrationAddress) == CommitMarker) { storageFault(); return; }
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
    if (!fram.healthy()) { storageFault(); return; }
    saveState();
    if (storageFailed) return;
  }
  mileageKnown = true;
  fram.write(MigrationAddress, CommitMarker);
  if (fram.read(MigrationAddress) != CommitMarker || !fram.healthy()) { storageFault(); return; }
}

uint16_t layoutCrc(const DisplayLayout &record) {
  return bytesCrc((const uint8_t *)&record, offsetof(DisplayLayout, crc));
}

bool validLayout(const DisplayLayout &record) {
  return record.magic == LayoutMagic && record.schema == 1 &&
    record.crc == layoutCrc(record) && record.x >= 0 && record.x < (int16_t)OLED_WIDTH &&
    record.y >= 0 && record.y < (int16_t)OLED_HEIGHT && record.font < DigitFontCount &&
    record.digits >= 1 && record.digits <= 40;
}

bool readLayout(uint8_t slot, DisplayLayout &record) {
  uint16_t address = LayoutSlotAddress[slot];
  if (fram.read(address + sizeof(record)) != CommitMarker) return false;
  uint8_t *bytes = (uint8_t *)&record;
  for (uint8_t i = 0; i < sizeof(record); ++i) bytes[i] = fram.read(address + i);
  return validLayout(record);
}

void loadLayout() {
  DisplayLayout other;
  bool firstValid = readLayout(0, layout), secondValid = readLayout(1, other);
  if (secondValid && (!firstValid || (int32_t)(other.sequence - layout.sequence) > 0)) {
    layout = other;
    activeLayoutSlot = 1;
  } else if (firstValid) {
    activeLayoutSlot = 0;
  } else {
    // No display record in v1.07. Corrupt/unknown layout falls back to these
    // defaults without touching mileage, fractions, ratio or needle offset.
    memset(&layout, 0, sizeof(layout));
    layout.magic = LayoutMagic;
    layout.schema = 1;
    layout.x = VIEW_X;
    layout.y = VIEW_Y;
    layout.font = DefaultDigitFont;
    layout.digits = 10;
    activeLayoutSlot = 1;
  }
  layoutDirty = false;
}

void saveLayout() {
  if (storageFailed || !fram.healthy()) { storageFault(); return; }
  if (!layoutDirty) return;
  uint8_t nextSlot = activeLayoutSlot ^ 1;
  uint16_t address = LayoutSlotAddress[nextSlot];
  ++layout.sequence;
  layout.crc = layoutCrc(layout);
  fram.write(address + sizeof(layout), 0);
  if (fram.read(address + sizeof(layout)) != 0 || !fram.healthy()) { storageFault(); return; }
  const uint8_t *bytes = (const uint8_t *)&layout;
  for (uint8_t i = 0; i < sizeof(layout); ++i) {
    fram.write(address + i, bytes[i]);
    serviceFastInputs();
  }
  for (uint8_t i = 0; i < sizeof(layout); ++i) {
    if (fram.read(address + i) != bytes[i] || !fram.healthy()) { storageFault(); return; }
    serviceFastInputs();
  }
  fram.write(address + sizeof(layout), CommitMarker);
  if (fram.read(address + sizeof(layout)) != CommitMarker || !fram.healthy()) { storageFault(); return; }
  activeLayoutSlot = nextSlot;
  layoutDirty = false;
}


void beginStorage() {
  Wire.pins(framSda, framScl);
  Wire.begin(); // DxCore Wire enables bounded transaction timeouts by default.
  if (!fram.begin()) storageFault();
  if (!storageFailed) loadState();
  if (!mileageKnown) {
    memset(&state, 0, sizeof(state));
    state.magic = RecordMagic; state.schema = 1; state.ratioMilli = 1000;
  }
  loadLayout();
  if (!fram.healthy()) storageFault();
}
