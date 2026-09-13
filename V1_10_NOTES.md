# v1.10: reserve the future RPM connection

**D17 is the Nano pin labeled A3.** It is now reserved for a future conditioned
RPM pulse signal, replacing the earlier dimmer/AFR reservation. v1.09 remains
unchanged.

| Harness connection | Destination |
|---|---|
| Future conditioner logic output | Nano A3 / D17 |
| Conditioner logic ground | Nano GND |

Prepare and label the wire `RPM IN — CONDITIONED 0–5V`, and insulate its unused
end. Add the coil-negative-rated conditioner inline before connecting the HEI
TACH terminal. That terminal must never connect straight through to the Nano.
Conditioner power wiring depends on the unit selected later.

The source reserves the input and its conversion setting:

```cpp
const uint8_t rpmPulsePin = A3; // digital D17
constexpr float RpmPulsesPerRevolution = 4.0f;
```

Four pulses per crankshaft revolution assumes a conventional single-coil,
four-stroke **V8 GM HEI** with a conditioner that preserves one pulse per firing
event. A V6 would use 3.0 and a four-cylinder 2.0. The future formula is
`RPM = frequencyHz * 60 / RpmPulsesPerRevolution`. This constant is configured
in the source, independently of the speedometer calibration ratio.

The pin is initialized with `INPUT_PULLUP` so the disconnected input has a
defined idle state. No RPM interrupt, counting or display calculation is
enabled in this release. The RPM screen still shows `--`. A future
implementation will use A3's pin-change interrupt (PCINT11); ordinary
`attachInterrupt()` does not support A3 on the classic Nano.

Existing mileage and display FRAM formats are unchanged. Compile v1.10 alone
in its own matching sketch folder using the same dependencies as v1.09.
