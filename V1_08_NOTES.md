# v1.08 display calibration

The OLED is larger than the speedometer's physical window. This version lets
the operator position and size the actual digits behind that opening, then
record how many complete digits fit. No software rectangle assumes where the
physical window begins or ends.

## Using the harness

Ground A0 through the existing calibration harness. Encoder wiring is unchanged:
S1=D3, S2=D12, KEY=A1, 5V and GND. Short encoder presses cycle these targets:

| Target | Turning the encoder changes |
|---|---|
| RATIO | Speed/distance ratio, 0.500–2.000 |
| OFFSET | Needle offset, −60 to +60 motor steps |
| X | Left edge of the numeric line, 0–255 OLED pixels |
| Y | Numeric **baseline**, 0–63 OLED pixels; increasing moves down |
| SIZE | One of 12 readable numeric fonts |
| DIGITS | Exactly 1–40 test digits, recording the observed window capacity |

After DIGITS, the next short press returns to RATIO. Reconnecting the harness
starts at RATIO. The normal mode/trip button is ignored with the harness
connected, preventing an accidental trip reset during display setup.

In X, Y, SIZE and DIGITS, a mode/settings hint appears for 1.2 seconds, then
the numeric test pattern appears. Turning immediately dismisses the brief hint.
A **long encoder press** toggles a persistent settings readout and the test
pattern. Release after the long press does not advance to another target.
The readout shows X, Y, font height H in pixels and digit count N. It replaces
the pattern rather than covering it. If the current position hides the readout
behind the window, continue adjusting X/Y and use the documented mode order.

The pattern is `0123456789012345…`, ending at exactly N selected digits.
It does not wrap onto another line; pixels outside the OLED are clipped. All
digits use the selected font, baseline and fixed character-cell spacing used
by the normal numeric screens. The count is not calculated from the OLED's
256-pixel width: it is the operator's measurement of the smaller opening.

## Fitting the window

1. Choose X/Y so the first test digit is fully visible at the left of the opening.
2. Cycle to SIZE and select the largest readable font whose digits fit vertically.
   Adjust Y again as needed: Y is the baseline, not the top of the text.
3. Cycle to DIGITS. Increase N until the final digit clips, then reduce N until
   every selected digit is visible at full height and width.
4. Refine X/Y and size if needed, then repeat the count measurement. A setting
   change does not automatically remeasure the physical window.
5. Long-press to inspect the saved X/Y/H/N values. Short-press to another target,
   stop adjusting for at least one second, or unplug the harness to save.
6. Check ODO, TRIP, MPH and RATIO with the harness unplugged.

The largest odometer value is `999999.9`: **seven digits plus a decimal point,
requiring eight visible character cells**. The decimal point uses one cell,
just like a digit. Aim for at least eight fitted test digits if the full
odometer range must remain readable. If that does not fit, reduce font size.
N is saved as the observed capacity; it does not truncate stored mileage,
hide leading digits, or automatically shrink overflowing normal readings.
The operator must choose a layout wide enough for the intended values.

Font choices are LuBS08, Logisoso16, the original LuBS12, then Logisoso18,
20, 22, 24, 26, 28, 32, 38 and 42. Font names are not exact pixel-height labels;
the readout uses U8g2's ascent-minus-descent metric. The test digits themselves
determine whether their full shapes are visible through the physical window.
Defaults retain v1.07's X=5, baseline Y=38 and LuBS12 font, with N=10.
Spacing is now fixed consistently between preview and normal numeric readings.
Small mode labels sit above the digits when the OLED has enough vertical room;
the physical opening may hide them. Display fitting never overlays a label on
the test digits.

## Persistence and compatibility

The v1.07 mileage/settings journal remains unchanged at 0x80/0xC0. Existing
odometer, trip fractions, ratio and offset load without migration changes.
Display layout has its own CRC-protected alternating records at **0x100 and
0x120**, each 17 bytes plus one commit byte. These include X, baseline Y,
font index and observed digit capacity N. They use the same invalidate/write/
verify/commit approach as mileage storage. A failed write never validates a
partial new layout; an earlier complete record remains recoverable.

If neither layout record is valid, the display returns to defaults, leaving
mileage and its calibration intact. A missing/invalid mileage journal retains
v1.07's fail-closed behavior. The layout is saved after one second without
adjustment, when changing target, and when unplugging the harness.

v1.07 can still read mileage written by v1.08, but ignores the new display
settings. v1.06 retains the older downgrade limitation described in
[V1_07_NOTES.md](V1_07_NOTES.md): it reads stale legacy mileage bytes.

## Build and validation

Use the same Nano toolchain and libraries as v1.07; FreqMeasure is not needed.
The verified build uses **28,864 / 30,720 bytes flash (93%)** and
**1,260 / 2,048 bytes static RAM**, leaving **788 bytes** for stack/runtime use.
All host regression checks passed. The sketch emits no warnings; a clean
library build may show the existing Switec signed/unsigned comparison warnings.
Compile this `.ino` in its own directory so Arduino does not combine all the
repository's versions:

```powershell
$sketchDir = Join-Path $env:TEMP 'stepper_speedometer_v1_08'
New-Item -ItemType Directory -Force -Path $sketchDir | Out-Null
Copy-Item .\stepper_speedometer_v1_08.ino $sketchDir
arduino-cli compile --fqbn arduino:avr:nano --warnings all $sketchDir
```

Supply `--libraries` if SwitecX25 is outside your installed Arduino libraries.
U8g2 must have `U8G2_16BIT` enabled, as required for the 256-pixel OLED.

Run the host regression tests with a C++17 compiler:

```powershell
python tests/run_v108_tests.py --compiler C:\path\to\zig.exe
```

Tests execute the actual sketch against fake time, GPIO, display and FRAM.
They retain v1.07's mileage/power-cut/encoder tests and add layout limits,
all six harness targets, long-press readout, trip-reset protection, precise
repeating digit counts, consistent preview/normal spacing, OLED clipping,
autosave and unplug behavior, all 20 display-record power-cut boundaries,
all 136 single-bit record corruptions, sequence wrap and safe layout defaults.
Font metrics in host tests are representative; actual bitmap appearance and
the physical window fit must be checked on the gauge. No hardware upload was
performed.
