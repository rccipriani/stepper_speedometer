# v1.07 build and bench notes

`stepper_speedometer_v1_07.ino` targets the classic 16 MHz ATmega328P Nano.
v1.06 and the Wokwi sketch are unchanged. No hardware upload was performed.

## Changes

- Conditioned VSS stays on **D2**, using an external interrupt instead of
  FreqMeasure. OLED reset stays on **D8**. The existing encoder wiring stays
  S1=D3, S2=D12, KEY=A1, 5V and GND; the harness also grounds A0.
- Actual pulse counts determine distance. Integer fractions retain partial
  tenths independently for the main odometer and trip. Ratio changes affect
  future distance; needle offset never affects distance or digital MPH.
- Speed uses pulse periods, requires two edges after startup/timeout, and
  decays toward zero when pulses slow or stop. The zero timeout is 3 seconds;
  very slow movement below approximately 0.3 MPH at ratio 1 may show zero,
  but all received pulses still contribute to mileage.
- Encoder pin-change interrupts decode complete quadrature cycles, cancelling
  ordinary contact bounce and rejecting impossible two-bit transitions. The
  pins/interrupt vectors are specific to the ATmega328P. The EC11's actual
  clicks per pulse should be checked on the bench.
- U8g2 renders every display page, with a 256-byte one-page pixel buffer.
  Display refresh is scheduled every 200 ms; motor updates run between pages
  and FRAM transactions. The startup logo uses a small font to save flash.
- Trip reset clears its fractional remainder without disturbing the odometer.
  Odometer formatting uses integer digits rather than a large floating value.
  RPM displays `--` until a real RPM input is implemented.

## Storage and migration

Two alternating FRAM records at **0x80 and 0xC0** contain mileage, fractions,
ratio, offset, sequence, schema and CRC. Each uses 32 payload bytes plus a
commit byte. The destination is invalidated, written, read back and verified,
then committed. The preceding record remains available if power interrupts
the write. Dirty data is checkpointed every 100 ms plus write/service time.
Power loss can still discard data since the last completed checkpoint;
uninterrupted hardware power or a hold-up supply is needed to eliminate that
window. Homing also briefly blocks checkpointing, although pulses are counted.

On first boot, v1.07 imports v1.06's digits at 0x00–0x0A, float ratio at 0x20,
and signed offset at 0x30. Invalid legacy digits become zero, and invalid
settings use defaults, matching the earlier sketch's recovery policy. Legacy
bytes remain unchanged. A sentinel at 0x70 prevents silently importing stale
legacy mileage if both new records become invalid after migration.

**Downgrading to v1.06 restores the old legacy values, not mileage accumulated
in v1.07.** After migration, `FRAM ERROR` requires investigation; do not erase
the new records or sentinel to bypass it unless deliberately resetting or
recovering saved mileage. Start the car stationary while the needle homes.

## Reproducing the Nano build

Validated dependencies:

- Arduino AVR Boards 1.8.8
- U8g2 2.36.19, with `U8G2_16BIT` enabled in the library
- Adafruit FRAM I2C 2.0.3 and Adafruit BusIO 1.17.4
- SwitecX25 from https://github.com/clearwater/SwitecX25 (master snapshot)

FreqMeasure is no longer required. Install SwitecX25 through Arduino's Add
ZIP Library feature or supply its parent libraries directory to the CLI's
`--libraries` argument. Debug serial output defaults to disabled.

Final verified Nano build: **20,170 / 30,720 bytes flash (65%)** and
**1,223 / 2,048 bytes static RAM**, leaving **825 bytes** for stack/runtime use.
All host regression checks passed. Existing signed/unsigned comparison warnings
in SwitecX25/SwitecX12 are upstream library warnings; the sketch emitted none.

**Compile only v1.07.** Arduino combines every `.ino` in a sketch directory,
so compiling the repository root would combine several versions and fail.
For example, in PowerShell from this repository:

```powershell
$sketchDir = Join-Path $env:TEMP 'stepper_speedometer_v1_07'
New-Item -ItemType Directory -Force -Path $sketchDir | Out-Null
Copy-Item .\stepper_speedometer_v1_07.ino $sketchDir
arduino-cli compile --fqbn arduino:avr:nano --warnings all $sketchDir
```

The CLI may require its full installed path until the terminal's PATH refreshes:
`C:\Program Files\Arduino CLI\arduino-cli.exe`.

## Automated checks

`tests/run_v107_tests.py` compiles the **actual sketch** against host fakes for
GPIO, time, display, motor and FRAM; it does not upload to a board. Pass a
C++17 compiler executable (g++, clang++, or Zig):

```powershell
python tests/run_v107_tests.py --compiler C:\path\to\zig.exe
```

Tests cover distance fractions and calibration, distance/counter rollover,
first-pulse speed, low/high speeds, needle endpoint with a negative offset,
timeout and timer wrap, encoder directions/bounce/invalid transitions,
short/long presses, trip reset, all 35 checkpoint power-cut boundaries,
all 256 single-bit record corruptions, sequence wrap, fail-closed recovery,
and legacy migration. Host tests do not simulate AVR interrupt timing,
electrical noise, I2C timing or physical display/motor behavior.

## Bench checklist

1. Start stationary. Confirm the startup text, full needle sweep and return,
   then check that all screens render through the actual odometer window.
2. With ratio 1.000 and offset 0, feed a clean, conditioned D2 pulse signal:
   3.333 Hz should show about 3 MPH; 66.667 Hz about 60 MPH; 133.333 Hz about
   120 MPH while the needle remains at its maximum. Confirm no fabricated
   speed after the first isolated edge, and zero within about 3.1 seconds
   after the final edge. Do not feed raw vehicle voltage into D2.
3. Feed exactly 4,000 rising edges: main and trip should advance 1.0 mile at
   ratio 1.000, 0.5 mile at ratio 0.500, or 2.0 miles at ratio 2.000. Test
   partial distances across a restart after allowing a checkpoint to finish.
4. Turn the EC11 slowly and quickly both ways. Confirm one adjustment per
   complete encoder pulse without jumps. Check short KEY presses toggle
   ratio/offset, long presses do not toggle, and unplugging the harness exits
   calibration. Swap S1/S2 if clockwise adjustment is reversed.
5. Check short mode presses, long TRIP reset and the next tenth after reset.
   Verify a reset clears the trip fraction but preserves main distance.
6. After recording the known mileage, check power cycling on bench hardware.
   Confirm settings and fractional mileage return from the last checkpoint.
   Watch the needle for stutter during display refresh/FRAM writes; host tests
   cannot validate motor smoothness or vehicle noise immunity.
