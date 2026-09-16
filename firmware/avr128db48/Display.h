// Extracted from v1.12; included once by Application.cpp.
void selectDigitFont() {
  const uint8_t *font = (const uint8_t *)pgm_read_ptr(&DigitFonts[layout.font]);
  display.setFont(font);
  display.setFontPosBaseline();
  display.setFontRefHeightAll();
}

uint8_t digitCellWidth() {
  uint8_t widest = 1;
  char digit[2] = {'0', 0};
  for (; digit[0] <= '9'; ++digit[0]) {
    uint16_t width = display.getStrWidth(digit);
    if (width > widest) widest = width;
  }
  return widest + 1; // consistent spacing for test digits AND normal numbers
}

// Caller selects the numeric font first. Saved N describes the operator's
// observed capacity; additionally reject any geometry clipped by the OLED.
bool numericFits(uint8_t cells) {
  uint16_t right = layout.x + (uint16_t)cells * digitCellWidth();
  return cells <= layout.digits && layout.x >= 0 && right <= OLED_WIDTH &&
    layout.y >= display.getAscent() &&
    layout.y - display.getDescent() < (int16_t)OLED_HEIGHT;
}

void drawFitWarning() {
  // Small text fits within the observed horizontal span where possible.
  // A wholly off-screen layout cannot guarantee a visible warning, but still
  // must never render a misleading fragment of the numeric value.
  uint16_t available = (uint16_t)layout.digits * digitCellWidth();
  uint16_t remaining = OLED_WIDTH - layout.x;
  if (available > remaining) available = remaining;
  display.setFont(u8g2_font_5x7_tr);
  display.setCursor(layout.x, constrain(layout.y, 7, 63));
  if (available >= display.getStrWidth("FIT")) display.print(F("FIT"));
  else display.print(F("!"));
}

void drawNumericText(const char *text) {
  selectDigitFont();
  if (!numericFits(strlen(text))) {
    drawFitWarning();
    return;
  }
  uint8_t cell = digitCellWidth();
  uint16_t x = layout.x;
  for (; *text && x < OLED_WIDTH; ++text, x += cell)
    display.drawGlyph(x, layout.y, *text);
}

void printTenths(uint32_t tenths) {
  // A decimal point occupies one cell, so 999999.9 needs EIGHT visible cells.
  char text[12];
  ultoa(tenths / 10, text, 10);
  uint8_t length = strlen(text);
  text[length++] = '.';
  text[length++] = '0' + tenths % 10;
  text[length] = 0;
  drawNumericText(text);
}

void drawOdometer() {
  selectDigitFont();
  // Validate the full lifetime range before showing even a short value.
  if (!numericFits(8)) {
    drawFitWarning();
    return;
  }
  printTenths(state.odoTenths);
}

void drawFloatValue(float value, uint8_t decimals) {
  char text[20]; // even the shortest supported pulse period fits this buffer
  dtostrf(value, 1, decimals, text);
  drawNumericText(text);
}

void drawLayoutCalibration() {
  selectDigitFont();
  uint8_t cell = digitCellWidth();
  int16_t height = display.getAscent() - display.getDescent();
  bool odometerFits = numericFits(8);
  if (layoutInfo || layoutHint) {
    // Readout REPLACES the digits, never overlays the fitting pattern. Keep
    // its baseline near the chosen position; small Y values still show a row.
    display.setFont(u8g2_font_5x7_tr);
    display.setCursor(layout.x, max((int16_t)7, layout.y - 9));
    switch (calibrationTarget) {
      case CAL_X: display.print(F("X ")); display.print(layout.x); break;
      case CAL_Y: display.print(F("Y ")); display.print(layout.y); break;
      case CAL_SIZE: display.print(F("SIZE ")); display.print(height); display.print(F("px")); break;
      case CAL_DIGITS: display.print(F("DIGITS ")); display.print(layout.digits); break;
      default: break;
    }
    if (!odometerFits) display.print(F(" FIT8"));
    display.setCursor(layout.x, max((int16_t)16, layout.y));
    display.print(F("X")); display.print(layout.x);
    display.print(F(" Y")); display.print(layout.y);
    display.print(F(" H")); display.print(height);
    display.print(F(" N")); display.print(layout.digits);
    return;
  }
  // Pattern and normal readings have identical glyph, baseline and spacing.
  // Keep arithmetic 16-bit: 40 large digits extend beyond the OLED and must
  // clip at its edge rather than wrap around to the left or onto another row.
  for (uint8_t i = 0; i < layout.digits; ++i) {
    uint16_t x = layout.x + (uint16_t)i * cell;
    if (x >= OLED_WIDTH) break;
    display.drawGlyph(x, layout.y, '0' + i % 10);
  }
}

void drawScreen(bool calibration) {
  if (storageFailed || !fram.healthy()) {
    display.setFont(u8g2_font_5x7_tr);
    display.setCursor(VIEW_X, VIEW_Y);
    display.print(F("FRAM ERROR"));
    return;
  }
  display.setFont(u8g2_font_5x7_tr);
  // Keep the original startup greeting visible throughout cooperative homing
  // and the sweep/holds. Persistence faults above always take precedence.
  if (!speedometer.ready()) {
    display.setCursor(VIEW_X, VIEW_Y);
    display.print(F("Savoy "));
    display.print(VERSION);
    return;
  }
  if (calibration) {
    if (!calibrationInputsValid) {
      display.setCursor(layout.x, max((int16_t)7, layout.y));
      display.print(F("CAL INPUT"));
      return;
    }
    if (calibrationTarget >= CAL_X) {
      drawLayoutCalibration();
      return;
    }
    display.setCursor(layout.x, max((int16_t)7, layout.y - 18));
    display.print(F("CAL MPH "));
    display.print(mph, 1);
    display.setCursor(layout.x, max((int16_t)16, layout.y - 8));
    if (calibrationTarget == CAL_OFFSET) {
      display.print(F("OFFSET "));
      display.print(state.offset);
      display.print(F(" st"));
    } else {
      display.print(F("RATIO "));
      display.print(state.ratioMilli / 1000.0f, 3);
    }
    display.setCursor(layout.x + VIEW_W - 30, layout.y);
    display.print(VERSION);
    return;
  }
  selectDigitFont();
  int16_t labelY = layout.y - display.getAscent() - 2;
  display.setFont(u8g2_font_5x7_tr);
  display.setCursor(layout.x, max((int16_t)7, labelY));
  // If the numeric line reaches the OLED top, omit the label instead of
  // drawing it over the digits. The physical window may hide labels anyway.
  if (labelY >= 7) {
    switch (displayMode) {
      case 0: display.print(F("ODO")); break;
      case 1: display.print(F("TRIP")); break;
      case 2: display.print(F("MPH")); break;
      case 3: display.print(F("RATIO")); break;
      case 4: display.print(F("RPM (not connected)")); break;
      case 5: display.print(F("AFR")); break;
    }
  }
  switch (displayMode) {
    case 0: drawOdometer(); break;
    case 1: printTenths(state.tripTenths); break;
    case 2: drawFloatValue(mph, 1); break;
    case 3: drawFloatValue(state.ratioMilli / 1000.0f, 3); break;
    case 4: drawNumericText("--"); break;
    case 5:
      if (afrValid) drawFloatValue(afr, 1);
      else drawNumericText("--");
      break;
  }
}

void updateDisplay() {
  bool calibration = digitalRead(calSwitchPin) == LOW;
  display.firstPage();
  do {
    drawScreen(calibration);
    serviceFastInputs();
  } while (display.nextPage());
}


void beginDisplay() {
  SPI.pins(oledMosi, oledMiso, oledSck, oledCs);
  display.begin();
  display.setContrast(DimmerContrastMax);
  updateDisplay(); // Show the startup greeting immediately, before the first loop.
}
