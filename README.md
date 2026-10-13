# Bitchshifter

Frequency shifter / ring mod voiced as a shortwave sideband receiver, for the Synthux
Simple Touch. Design: `docs/superpowers/specs/2026-10-12-bitchshifter-design.md`.

## Control map (measured with hwtest/ and padtest/ on 2026-10-12)

| Control | Input | Increases when |
|---|---|---|
| Fader 1 TUNE (left fader) | A6 | pushed up |
| Fader 2 RECEPTION (right fader) | A7 | pushed up |
| Knob 1 RING | A1 | turned clockwise |
| Knob 2 CARRIER | A2 | turned clockwise |
| Knob 3 FILTER | A3 | turned clockwise |
| Knob 4 WIDTH | A4 | turned clockwise |
| Knob 5 FEEDBACK | A0 | turned clockwise |
| Knob 6 MIX | A5 | turned clockwise |

Knobs are numbered top row left to right, then the next row left to right.

**Switches** (libDaisy `Switch3`, both read mirror-image: physically **left** reads
`POS_RIGHT` (2), physically **right** reads `POS_LEFT` (1), centre reads `POS_CENTER` (0)):
- Switch 1, pad mode (left switch): `Init(D9, D8)`
- Switch 2, modifier (right switch): `Init(D7, D6)`

**Pads** (MPR121 at 0x5A on I²C1, default libDaisy config; touches read 80–110 above
baseline against a threshold of 12, idle noise a few points):

| Channel | Pad | Channel | Pad |
|---|---|---|---|
| 0 | Top left (around the left switch) | 6 | Centre right (big pad) |
| 1 | Top centre (narrow wedge between the switches; hard to reach) | 7 | Right edge strip |
| 2 | Top right (right of the right switch) | 8 | Bottom left |
| 3 | Left edge strip | 9 | Bottom right |
| 4 | Centre left (big pad) | 10 | "TOU" of the TOUCH logo |
| 5 | Centre (narrow strip) | 11 | "CH" of the TOUCH logo |

Layout: 3 top, 5 middle, 2 bottom, plus the two logo pads (Synthux uses them as modifiers).
The blue triangles at the top corners aren't pads.

## Diagnostics

- `hwtest/`: prints the 8 analog inputs (0–100), both switches and the pads over USB serial.
- `padtest/`: prints each pad channel's raw touch strength (baseline − filtered) 10×/s.

Flash either with `make program-dfu` from its folder (RESET, then BOOT while the LED
pulses), and read the output with `cat /dev/cu.usbmodem*`.

## Part 1: playable shifter

| Control | Does |
|---|---|
| Fader 1 TUNE | Shift −2500 … 0 (centre) … +2500 Hz, fine near the centre |
| Knob 5 FEEDBACK | Feedback 0–95% (barberpole) |
| Knob 6 MIX | Dry/wet |

Build and flash: `make`, then RESET, BOOT while the LED pulses, `make program-dfu`.
Over USB serial (`cat /dev/cu.usbmodem*`), once a second: CPU load (about 10%), the
control values and the input/output peak levels.
Mac tests: `sim/run_tests.sh`. Listen on the Mac: `sim/render.sh in.wav out.wav <tune_hz> [feedback] [mix]`.
`leveltest/`: input DC/peak/audio levels over USB, with a test tone or passthrough on switch 1.

**Cabling note:** the outputs are stereo on one TRS jack. Into balanced interface inputs,
use a TRS → 2× TS (insert/Y) cable. A stereo cable into a balanced input reads
left − right, which cancels the (identical) left and right of a mono signal.
