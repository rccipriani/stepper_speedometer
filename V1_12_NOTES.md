# v1.12: local MCP23017 calibration interface

Use `stepper_speedometer_v1_12.ino`. This version requires rerouting the
calibration encoder to an MCP23017 mounted beside the Nano inside the speedometer.
The calibration box contains the encoder and harness-detect ground connection;
the I2C bus stays inside the speedometer.

## Wiring

| Connection | Destination |
|---|---|
| Encoder S1 | MCP23017 GPA0 |
| Encoder S2 | MCP23017 GPA1 |
| Encoder KEY | MCP23017 GPA2 |
| Harness presence contact | Nano A0, grounded by attached harness |
| MCP23017 INTA | Nano D12 |
| MCP23017 SDA / SCL | Nano A4 / A5, shared with FRAM |
| MCP23017 address A0, A1, A2 | Ground: address `0x20` |
| MCP23017 RESET | Nano RESET, with pull-up to regulated 5V |
| MCP23017 VDD / VSS | Regulated 5V / common ground |
| Encoder module power | Regulated 5V / common ground |
| Future MAX A2 second-channel conditioned output | Nano D3 / INT1 |

The expander address pins are distinct from the Nano analog pins. FRAM remains
at `0x50`. Provide local supply decoupling (100 nF at the expander) and I2C pull-ups
to regulated 5V, accounting for resistors already fitted to the breakout boards.
Use a common reset so the expander starts with its default BANK=0 register map.
INTB and unused GPIO need no external connection; firmware enables GPIO pull-ups.
INTA is configured open-drain, active-low; D12 uses the Nano pull-up.

Nano A1 is now spare. D3 is reserved as an input with a pull-up; channel-two pulse
capture and its eventual function are not implemented yet. A3/D17 remains the
future conditioned RPM input, A6 AFR, and A7 filtered analog dimming. AFR and
dimming remain disabled until their external conditioning and source settings
are chosen. Never connect raw vehicle signals directly to these inputs.

## Behavior

All six calibration targets and FRAM formats are retained. The encoder is read
through small Wire register helpers; no additional Arduino library is required.
D12 is polled in the main execution context, with no I2C work inside an ISR.
Captured and current input states are serviced between OLED pages and FRAM
transactions, as well as during normal motion servicing.

With a detected harness, an initialization/read failure suppresses calibration
actions and displays `CAL I2C`. Initialization retries once per second. Attaching
the harness or recovering a connection primes the inputs so a held KEY release
does not trigger an action. An absent expander does not prevent ordinary
speed/mileage operation; an electrical fault that holds the shared I2C bus low
can also affect FRAM. Wire transactions have a 2.5 ms timeout.

The MCP23017 captures one interrupt state, not a queue of every encoder edge.
Bench-test fast turns and contact bounce with the actual harness while the OLED
refreshes and FRAM saves. Software tests cannot establish the maximum reliable
rotation speed or electrical noise tolerance. Register/interrupt reference:
[Microchip MCP23017 datasheet](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/20001952C.pdf).

## Validation

- Arduino AVR Nano build: 29,898 / 30,720 bytes flash, 1,274 / 2,048 bytes static RAM.
- Both analog features enabled with a temporary example AFR mapping: 30,406 bytes
  flash, 1,284 bytes static RAM. Only 314 flash bytes remain in that configuration;
  further RPM/channel-two features will require a flash-budget review.
- Production-sketch host tests passed for default, enabled analog, and inverted
  dimmer configurations. Coverage includes expander configuration, idle polling,
  capture/current decoding, short reads, missing device, retry and held-key
  recovery, plus existing mileage, speed, debounce, FRAM corruption/power-cut,
  calibration, fit-guard and analog regression tests.
- Existing third-party Switec signedness warnings remain. No hardware upload or
  physical timing test was performed.

Run `python tests/run_v112_tests.py --compiler <C++ compiler path>`; add
`--analog-test-config enabled` or `--analog-test-config inverted` for temporary
analog fixtures. Compile the selected `.ino` in its own matching sketch folder,
not all historical versions together.
