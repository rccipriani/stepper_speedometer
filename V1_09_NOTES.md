# v1.09: enforce numeric display fit

v1.09 retains v1.08's harness controls and FRAM formats, and adds an enforced
fit check before normal numeric rendering. Existing mileage and display settings
load without migration. Earlier sketch versions are unchanged.

## Behavior

- **ODO always requires eight visible character cells** for the full
  `999999.9` range, even when today's mileage takes fewer characters.
- The fit check enforces the saved, operator-observed digit capacity and the
  actual OLED's right, top and bottom bounds using the selected font metrics.
- If the full value cannot fit, **no digits of that value are drawn**. The
  screen shows `FIT`, or `!` where there is too little horizontal room for the
  word. Fix the layout with the calibration harness. An extremely misplaced
  layout can also hide the warning behind the physical window; reposition X/Y.
- Trip, MPH and ratio use the same guard for their current formatted length.
- In the display calibration settings readout, **FIT8** means the layout cannot
  accommodate the full odometer. Increase the confirmed count to at least eight,
  select a smaller font or reposition it as appropriate.
- The repeating-digit fitting preview remains unrestricted, so it is still
  possible to inspect and correct undersized or clipped layouts. Long-press
  the encoder to toggle between the preview and the settings readout.
- Mileage continues accumulating and saving when a fit warning is displayed.
  The display guard does not change the stored odometer, trip, ratio or offset.

The capacity is still a human measurement of the smaller physical opening;
the firmware cannot detect its edges. Recheck the actual visible count after
changing X, Y or font size. A falsely high saved count cannot establish that
digits are physically visible. The decimal point occupies one character cell.

For the unchanged calibration procedure and dependency list, see
[V1_08_NOTES.md](V1_08_NOTES.md). Its statement that the count is advisory applies
to v1.08 only; v1.09 enforces it as described above.

## Verification

- Nano build with Arduino AVR Boards 1.8.8, U8g2 2.36.19, Adafruit FRAM I2C
  2.0.3, BusIO 1.17.4 and the existing SwitecX25 snapshot passed.
- Flash: **29,224 / 30,720 bytes (95%)**. Static RAM: **1,264 / 2,048 bytes**,
  leaving **784 bytes** for runtime/stack. No sketch warnings; upstream Switec
  signed/unsigned comparison warnings remain.
- All previous logic tests pass. New tests cover an undersized layout at low
  mileage, the full eight-character maximum, an exact right-edge fit and
  one-pixel overflow, vertical/font limits, narrow warnings, other numeric
  modes, the unobstructed preview and the FIT8 readout.

Run tests with a C++17 compiler:

```powershell
python tests/run_v109_tests.py --compiler C:\path\to\zig.exe
```

Compile `stepper_speedometer_v1_09.ino` alone in a matching sketch directory;
do not compile the repository's several `.ino` versions together. No hardware
upload was performed. On the bench, set DIGITS to seven and verify ODO shows
a warning; then fit and confirm eight complete cells and check `999999.9`
using a test setup without overwriting the actual saved mileage.
