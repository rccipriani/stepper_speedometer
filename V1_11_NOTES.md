# v1.11: AFR and dashboard dimmer inputs

| Nano pin | Allocation | Signal at the Nano |
|---|---|---|
| A3 / D17 | Future RPM | Conditioned 0–5V logic pulses |
| A6 | Wideband AFR | Protected controller analog output, within 0–5V |
| A7 | Dashboard dimmer | Protected, low-pass-filtered 0–5V DC brightness level |

The normal mode cycle is now **ODO → TRIP → MPH → RATIO → RPM → AFR → ODO**.
AFR uses the calibrated numeric layout and its fit guard. Existing mileage,
calibration harness controls and FRAM record formats are unchanged. RPM remains
a future placeholder with the V8 HEI source setting added in v1.10.

## Defaults while hardware is undecided

Both new inputs are disabled in the delivered source:

```cpp
#define AFR_INPUT_ENABLED 0
#define DIMMER_INPUT_ENABLED 0
```

AFR displays `--`, and OLED brightness retains its existing behavior. The ADC
does not sample the unconnected leads. A6 and A7 are analog-only on the classic
Nano; the sketch does not call `pinMode()` or enable internal pull-ups on them.
See the [official Nano pinout](https://docs.arduino.cc/resources/pinouts/A000005-full-pinout.pdf).

## AFR configuration

The wideband controller has not been chosen. A6 connects to its protected analog
output, not directly to an oxygen sensor. Follow the controller's signal-ground
instructions when building the interface.

Before setting `AFR_INPUT_ENABLED` to 1, configure:

- `AnalogReferenceVolts`: the measured Nano ADC supply/reference voltage.
- `AfrVoltageLow` and `AfrVoltageHigh`: the two voltage calibration endpoints.
- `AfrAtLowVoltage` and `AfrAtHighVoltage`: the corresponding controller-specified
  AFR values, using the controller's fuel scale/output configuration.

The AFR values deliberately default to zero placeholders. Enabling AFR without
replacing them produces a compile-time error instead of silently assuming a
controller transfer curve. The code performs linear interpolation, displaying
one decimal place. Voltages outside the configured interval display `--`.
Controller warm-up/fault signaling must be checked when choosing the controller:
an in-range voltage alone does not establish sensor readiness. More specific
fault handling may be needed once those specifications are known.

## Existing 12V PWM dimmer

The external signal path must be:

**12V dashboard PWM → automotive input protection/level conversion → low-pass
filter → A7 (0–5V DC referenced to Nano ground).**

A7 does not capture digital PWM edges. The external filter converts duty cycle
to an analog brightness level; a voltage divider alone does not perform that
filtering. Component values depend on PWM frequency, polarity, output topology
and the vehicle's voltage/transient range. Those details remain to be determined;
do not connect the 12V PWM lead directly to A7. Leave prepared leads insulated
until their conditioning circuits are installed.

When the interface is ready, set `DIMMER_INPUT_ENABLED` to 1 and adjust:

- `DimmerAdcDark` / `DimmerAdcBright`: ADC readings at the desired dimmest and
  brightest settings (defaults 0 and 1023). These may be reversed for an inverted
  signal. Account for the dimmer's lights-off/daytime behavior when choosing the
  interface and mapping.
- `DimmerContrastMin` / `DimmerContrastMax`: OLED contrast endpoints, initially
  16 and 255. Bench-check readable night/day levels on the actual display.

The enabled dimmer starts at maximum configured contrast during startup, then
follows its sampled input in normal operation, including harness calibration.
Samples are taken every 100 ms; the first ADC conversion after channel selection
is discarded, four readings are averaged, and dimmer readings are further
smoothed before updating contrast. This software smoothing supplements the
required external PWM filtering. U8g2's
[setContrast API](https://github.com/olikraus/u8g2/wiki/u8g2reference#setcontrast)
controls brightness; it does not change digit position or font size.

## Verification

The delivered Nano build passed at **29,300 / 30,720 bytes flash (95%)**,
with **780 bytes RAM free**. A temporary test build with both inputs enabled and
an explicitly test-only AFR curve passed at **29,820 bytes flash (97%)**, with
**772 bytes RAM free**. Those test endpoints were not copied into the delivered
firmware. Both builds used the same toolchain/libraries as v1.09/v1.10.

Host tests run the actual sketch against fake hardware. All existing firmware
regressions pass, plus mode cycling, disabled-input behavior, ADC averaging,
AFR conversion/range checks, brightness endpoints, reversed dimmer polarity,
smoothing and timer wrap. The enabled test fixtures are synthetic configurations,
not a recommendation for any particular controller.

```powershell
python tests/run_v111_tests.py --compiler C:\path\to\zig.exe
python tests/run_v111_tests.py --compiler C:\path\to\zig.exe --analog-test-config enabled
python tests/run_v111_tests.py --compiler C:\path\to\zig.exe --analog-test-config inverted
```

Compile v1.11 alone in a matching sketch directory. No firmware was uploaded.
Hardware testing and final conditioner/controller-specific configuration remain
for the future expansion.
