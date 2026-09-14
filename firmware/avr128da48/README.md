AVR128DA48 port of Robert Cipriani's stepper speedometer v1.12
================================================================

Open `avr128da48.ino` in this directory. The historical Nano sketches remain
unchanged; do not compile the repository root as one Arduino sketch.

This is the first Arduino/DxCore port, compiled and host-tested but not yet
tested on the physical instrument. No hardware was flashed or fuses changed.

Porting decisions
-----------------

The MCU-specific parts of v1.12 were its ATmega328P guard, Nano pin numbers,
MCP23017 calibration input transport, Wire timeout API, ADC assumptions, and
blocking Switec homing/startup. Its atomic pulse snapshots work on AVR DA too.
Older historical versions also contain pin-change-interrupt/register code;
they are not the source of this port.

The implementation keeps the application state and mature functions together,
with implementation headers included once by `Application.cpp`. This avoids
Arduino-generated prototypes and preserves the existing logic while establishing
separate hardware boundaries:

| Module | Responsibility / entry points |
| --- | --- |
| `Application.cpp`, `ApplicationState.h` | Timestamp scheduler and shared application settings/state |
| `VehicleSpeed.h` | `vehicleSpeed.getMph(ratioMilli)`, atomic pulse totals; GPIO interrupt backend |
| `Distance.h` | Original fixed-point mileage accumulation and speed-to-gauge orchestration |
| `Speedometer.h` | `speedometer.setSpeed(mph, offset)`, `home()`, `update()`; Switec driver |
| `Display.h` | `beginDisplay()`, `updateDisplay()`; existing SH1122 rendering and hardware SPI |
| `Storage.h` | `beginStorage()`, `saveState()`, `saveLayout()`; CRC journals and migration |
| `StorageLayout.h` | Packed records, schema identifiers and all FRAM addresses |
| `FramDevice.h` | Checked, bounded Wire transactions; no heap-backed device wrapper |
| `VehicleIO.h`, `Controls.h`, `AnalogInputs.h` | Native calibration inputs, controls, optional conditioned analog inputs |
| `BoardConfig.h`, `Watchdog.h` | Physical pin assignments and isolated AVR DA watchdog support |

Preserved behavior
------------------

* 4000 pulses/mile, ratio 500–2000 in thousandths, 3-second stop timeout,
  period aging, independent pulse-count distance, fractional remainders and rollovers.
* Gauge mapping `mph * 5.1 + offset`, offset −60…60, rounded/clamped to 0…510
  **after** offset. Switec's six-state sequence and acceleration are unchanged.
* State schema 1: 32-byte payload plus commit byte at 0x80/0xC0. Layout schema 1:
  17-byte payload plus commit byte at 0x100/0x120. Sentinel at 0x70. Legacy
  migration reads 0x00–0x31 without modifying those bytes. CRC and commit order
  are retained. No engine-hours fields or new storage schema were introduced.
* Existing display modes, fonts, fit checks, calibration targets, trip reset,
  debounce, and layout autosave. AFR/dimmer remain disabled by default, with
  the original transfer-curve placeholders. ADC is explicitly 10-bit, VDD referenced.
* Speed updates and dirty mileage checkpoints at 100 ms; OLED at 200 ms (5 Hz),
  deliberately preserving the previous refresh rate. Page buffering is retained.

Intentional behavior changes
----------------------------

* Native pull-up GPIO replaces the MCP23017 entirely. Calibration is polled
  every fast-service pass and between FRAM bytes/OLED pages. No I2C is used for
  calibration and no I2C work occurs in an ISR. Very rapid encoder edges can
  still be missed during peripheral work; verify maximum hand rotation speed.
* Homing is scheduled at a minimum 800 us per step, then the existing full-scale
  sweep and two 1-second holds run as a state machine. Controls, distance and
  display continue during startup; the old startup splash is removed. No
  `delay()`/blocking Switec calls run. Missed deadlines extend motion rather
  than issuing a burst of catch-up steps. The original 510-step homing travel
  is preserved: validate that it reaches the zero stop over this instrument's
  possible needle travel before accepting it. Homing does not sense the stop.
* A FRAM transport/verification failure latches `FRAM ERROR` and disables further
  writes until reset. The needle and RAM distance calculations continue. If
  startup mileage is unreadable, defaults (ratio 1, offset 0) drive the needle,
  mileage stays marked unknown, and default mileage is never committed. If both
  journals are invalid after migration, stale legacy data is not restored.
  Inspect storage before resetting; unsaved distance cannot survive loss of power.
* SH1122 SPI has no presence acknowledgement. A disconnected display does not
  gate motion; transfers remain bounded, but firmware cannot diagnose a blank
  OLED through this interface. The fault message is visible only on a working OLED.

Proposed wiring
---------------

Use MCU port labels, not classic Nano pin numbers. All assignments are in
`BoardConfig.h`. These avoid the Curiosity Nano's CDC UART (PC0/PC1), LED/button
(PC6/PC7), reset and UPDI. See the [Microchip board schematic and user guide](https://www.microchip.com/content/dam/mchp/documents/MCU08/ProductDocuments/UserGuides/AVR128DA48-Curiosity-Nano-UG-DS50002971B.pdf).

| Signal | AVR128DA48 port pin |
| --- | --- |
| X27 wires, original driver order 1/2/3/4 | PB0 / PB1 / PB2 / PB3 |
| OLED MOSI / SCK / CS | PA4 / PA6 / PA7 |
| OLED D/C / RESET | PE0 / PE1 |
| FRAM SDA / SCL, address 0x50 | PA2 / PA3 |
| MAX9924 conditioned VSS, rising edge | PC2 |
| Encoder S1 / S2 / KEY (to ground) | PC4 / PC5 / PE2 |
| Calibration harness detect / mode button (to ground) | PE3 / PD1 |
| Reserved conditioned MAX channel 2 / RPM | PC3 / PD2 |
| Optional AFR / filtered analog dimmer | PD3 / PD4 |

PA5 is reserved as SPI MISO even though the OLED does not use it. Spare pins
include PA0/PA1, PB4/PB5, PD0/PD5/PD6/PD7, PF0–PF5. PC0/PC1 can provide
`Serial1` through the on-board CDC interface. Future CAN requires an external
CAN controller as well as its transceiver; a transceiver alone is insufficient.

For external regulated 5 V, follow Microchip's [external-supply instructions](https://onlinedocs.microchip.com/oxy/GUID-71FEFD75-E46F-4322-9BEB-8C68686D4A50-en-US-2/GUID-BAA668EF-E4C0-44A9-866D-9A3B949DF2EA.html):
feed VTG, and ground VOFF to disable the on-board regulator when USB is connected.
VOFF is not a power input. Check the particular OLED/FRAM modules' logic-voltage
requirements and pull-ups, and the motor's coil current against MCU per-pin/port
limits. The code requests GPIO drive; it does not establish electrical suitability.
Only conditioned, voltage-compatible vehicle signals belong on MCU inputs.

Build and verification
----------------------

Verified with DxCore 1.6.2, U8g2 2.36.19 and upstream
[clearwater/SwitecX25](https://github.com/clearwater/SwitecX25). Install SwitecX25
from its ZIP (it is not in the Arduino library index). Retain its bundled license.
DxCore's [installation and core documentation](https://github.com/SpenceKonde/DxCore)
uses the additional board-manager URL `https://drazzy.com/package_drazzy.com_index.json`.

```sh
arduino-cli core update-index --additional-urls https://drazzy.com/package_drazzy.com_index.json
arduino-cli core install DxCore:megaavr@1.6.2 --additional-urls https://drazzy.com/package_drazzy.com_index.json
arduino-cli lib install "U8g2@2.36.19"
# Install the SwitecX25 ZIP, then:
arduino-cli compile -b DxCore:megaavr:avrda:chip=avr128da48,clock=24internal firmware/avr128da48
```

IDE: AVR DA-series **no bootloader**, chip AVR128DA48, internal 24 MHz, default
48-pin mapping, attachInterrupt enabled on all pins, default Wire mode and
millis timer. Select Microchip Curiosity Nano (nEDBG) when programming. No custom
timer configuration is needed. U8g2 2.36.19 already enables `U8G2_16BIT`; the
sketch fails explicitly if a different library build disables it. Do not define
it only in the sketch while compiling the library with a different setting.

Optional watchdog build: add
`--build-property compiler.cpp.extra_flags=-DSPEEDOMETER_WATCHDOG_ENABLED=1`.
It uses the AVR DA 8K-clock watchdog period and feeds once per complete loop.
Default is off for bench bring-up; this assumes the watchdog fuse is also off.
Choose/test brownout and watchdog fuses for the finished hardware separately.

```sh
python tests/run_avr128da48_tests.py --compiler /path/to/zig --switec-source /path/to/SwitecX25
python tests/run_v112_tests.py --compiler /path/to/zig
```

The DA tests compile the actual application and real Switec implementation with
fake time/GPIO/SPI/Wire. They check unchanged schema definitions and distance
code against v1.12, speed aging/wrap, mapping, both journals' interrupted commits,
legacy migration, transport faults, direct encoder inputs, fit protection, and
cooperative startup. Host tests do not measure electrical behavior or ISR latency.

Verified 2026-09-14: both the DA and original v1.12 host suites pass. Default
target build: 26,994 bytes flash, 1,239 bytes static RAM; watchdog build:
26,992 bytes flash, 1,239 bytes static RAM. Remaining compiler warnings are
signed/unsigned comparisons in the upstream Switec library.

Before vehicle use, bench-check 6.6667 Hz = 6 MPH at ratio 1, 400 pulses = 0.1 mile,
stop timeout, direction/endpoints/zero stop, fast encoder turns during saves,
FRAM disconnect/SDA held low, display disconnect, power cuts, and watchdog reset.
Use a scope to measure worst-case step jitter and pulse capture under OLED/FRAM
load. There is no claim of hardware validation from a successful compile.

Next hardware optimizations
---------------------------

First measure cooperative stepping jitter. If needed, move stepping into an
isolated timer service, keeping floating-point mapping and all bus work outside
the ISR. Replace only the `VehicleSpeed` backend with EVSYS + TCB capture after
allocating timers around DxCore's millis source. Preserve independent pulse
totals (odometer), first-edge validity, atomic snapshots and the 3-second timeout;
a fast-clock 16-bit capture alone cannot cover the current low-speed range.
Plan overflow extension/cascading or suitable clocking before choosing TCB mode.
Partial display updates can follow measured SPI load; page rendering is currently
bounded and serviced between pages. Tach, analog gauges, GPS and CAN can use
separate input modules without changing mileage storage or calibration math.

Original project credits remain in the historical v1.12 source: Luke Hurst,
Kevin Gale / Walter Clark, Guy Carpenter, PJRC and Trewjohn2001.
