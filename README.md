# bit-chifter

A frequency shifter and ring modulator voiced as a shortwave sideband receiver, for the
Synthux Simple Touch with an Electrosmith Daisy Seed. Inspired
by the Doepfer A-126 and other frequency shifters, and by the sound of tuning across a
shortwave band.

Fader 1 is a tuning dial: move it and your sound slides out of tune like mistuned single
sideband, sweeping through bell-like clangs and snapping back into harmony at multiples of its
pitch. Fader 2 loses the signal in the ether: drift, fading, static, crackle and a pumping AGC.
On top of that, 30 pad effects on three pages of 10 touch pads add echoes that come back shifted further each
time, endless spirals, a voice scrambler, hidden stations whistling as you tune past them,
selcall bursts and Morse chimes, bitcrush, stutter and dropouts; switch 2 freezes or loops
whatever's coming in so you can play the effects on it. Stereo in, stereo out.

## Hardware

- A Synthux Simple Touch with a Daisy Seed (tested with a Seed 1.2, PCM3060 codec).
- Stereo audio in and out on the board's two TRS jacks. A mono source into one input feeds
  both sides (an input counts as unused after 3 s of silence).
- **Cabling:** each jack carries left and right. Into a mixer or interface with balanced
  inputs, use a TRS → 2× TS (insert / Y) cable: a plain stereo cable into one balanced input
  reads left − right, which cancels a mono signal.

## Controls

| Control | Does |
|---|---|
| **Fader 1 TUNE** (left) | The sideband dial: 0 at the centre, a fine ±400 Hz over the middle 80%, accelerating to ±2 kHz over the last 10% at each end. It glides like a heavy dial, and gentle detents catch the shift where it turns harmonic again (at multiples of the source's pitch). |
| **Fader 2 RECEPTION** (right) | Clear (bottom) → lost in the noise (top): the two sides drift apart, the signal fades and swells, static and crackle rise, an AGC pumps, the input is driven, and the dial wanders on its own. It also sets **how strong the pad effects are** (clearly there at the bottom, extreme at the top). |
| **Knob 1 RING** | Ring-mod amount |
| **Knob 2 CARRIER** | Ring-mod carrier, 0.1 Hz (tremolo) to 5 kHz (bells) |
| **Knob 3 FILTER** | Radio filter centre, 150 Hz – 5 kHz |
| **Knob 4 WIDTH** | Radio filter: narrow and whistly → wide → fully open (from about 97% of the turn) |
| **Knob 5 FEEDBACK** | 0–95%: each pass through the loop is shifted again (barberpole spirals with TUNE off centre) |
| **Knob 6 MIX** | Dry/wet over the first 85% of the turn; the last 15% stays fully wet and adds a light stereo plate reverb |
| **Switch 1** (left) | The pad page: left = **Shift & time**, centre = **Radio**, right = **Digital & damage** |
| **Switch 2** (right) | Capture: left = **Normal**, centre = **Freeze** (the last 250 ms held as a drone), right = **Loop** (the last 2 s repeated, or less if you left Normal more recently; nothing is captured until 250 ms have been recorded). Everything after it, the dry signal included, hears the captured sound. |
| **TOU** (logo pad) | A quick tap (under 0.3 s) steps the stereo mode, starting from Linked at power-up, confirmed by blips: 1 Linked, 2 Mirror (the sides shift in opposite directions), 3 Rotate (they sweep around each other over 8 s) |
| **CH** (logo pad) | Unused |

Knobs are numbered top row left to right, then the next row left to right.

### The pads

Each pad's effect is **on while you hold it**; hold several to stack them, and they keep
going when you change page until you let go. The pads on the board:

```
   [top left]        [top centre]        [top right]
[left  ] [  big centre-left  ] [centre] [  big centre-right  ] [right ]
[ edge ]                       [ strip]                        [ edge ]
        [bottom left]                        [bottom right]
```

| Pad | Page 1: Shift & time | Page 2: Radio | Page 3: Digital & damage |
|---|---|---|---|
| Big centre-left | Echoes (shifted again each repeat; they ring out after you let go) | Scrambler (the voice turned upside down) | Bitcrush |
| Big centre-right | Spiral up (endless rising shift) | Passing carriers (hidden stations whistle as you tune) | Stutter |
| Left edge | Reverse echoes | Hopping scrambler | Sample-rate crush |
| Right edge | Spiral down | Selcall (5-tone bursts, mistuned by TUNE) | Dropouts |
| Centre strip | Ping-pong echoes | Jammer | Codec warble |
| Top left | Chirp (notes swoop up as they start) | Static burst | Ring 100% |
| Top right | Warble | Squelch (opens with a burst of noise) | Carrier ×2 |
| Top centre (narrow) | Feedback burst | Morse chimes | Hold drift |
| Bottom left | Dial spin down | Signal fade-out | Kill dry |
| Bottom right | Dial spin up | Filter slam | Chaos (2–3 random effects, new each touch) |

Most page 1 and 2 effects work on the wet signal, so you hear them best with MIX near fully
wet (Selcall and Signal fade-out work at any MIX). Page 3's damage (bitcrush, sample-rate
crush, stutter, dropouts, codec warble) works on the whole sound at any MIX; Ring 100%,
Carrier ×2 and Hold drift act on the wet signal, and Kill dry makes it fully wet.

## Sounds to try

- **Barberpole:** TUNE a little off centre (±30–100 Hz), FEEDBACK 80–90%, MIX fully wet. Then
  nudge TUNE into a swoop zone. Add the reverb at the end of MIX.
- **Lost station:** RECEPTION at 70% or more, WIDTH narrow (~30%), FILTER around 1 kHz; leave
  the faders alone. Hold Selcall and Morse chimes (page 2) and move TUNE slowly.
- **Shifted echoes:** page 1, TUNE off centre, FEEDBACK about 60%; hold Echoes briefly and let
  them ring out, each repeat climbing further.
- **Frozen spiral:** play something, flip switch 2 to Freeze, hold Spiral up and sweep TUNE.
- **Broken stream:** switch 2 to Loop, page 3, RECEPTION high, hold Codec warble, Dropouts and
  Stutter.
- **Metal bells:** RING 100%, CARRIER 300–1500 Hz, a small TUNE shift; Mirror stereo (tap TOU once from power-up: two blips).

## Building and flashing

Needs the Daisy toolchain (`arm-none-eabi-gcc` and `dfu-util`: see Electrosmith's
[C++ development environment guide](https://docs.daisy.audio/tutorials/cpp-dev-env/)) and
[libDaisy](https://github.com/electro-smith/libDaisy) cloned next to this repository (one folder
up) and built (`cd ../libDaisy && make`).

The firmware runs from the Seed's SRAM (`APP_TYPE = BOOT_SRAM`), so the Seed needs the
**Daisy bootloader**. To install it once: hold BOOT, tap RESET, release BOOT (the Seed's own
DFU mode), then, from this folder:

```sh
make program-boot
```

Then build and flash bit-chifter through the bootloader: tap RESET, press BOOT while the LED
pulses (the bootloader's update window), then

```sh
make
make program-dfu
```

`dfu-util` often ends with "Error during download get_status" after a successful flash; that's
harmless (the Seed has already restarted into the new firmware).

## The USB report

With the Seed connected over USB, `cat /dev/cu.usbmodem*` (macOS; `/dev/ttyACM*` on Linux) shows a two-line report once a
second: the CPU load (average and the busiest block: keep that below 100%), the control
values, the stereo mode, the input and output peak levels, a blip counter, the input's tracked
pitch (`--` when it isn't clear), the page, the active effects (`fx`, one bit per effect in
`engine/effects.h`) and the capture mode (`fb` is the FEEDBACK knob's position, 0–100%; "(pads not found)"
means the touch controller didn't answer at start-up). It's sent without waiting, so nothing stalls when no
one is reading. Measured on a Seed 1.2: about 31% CPU on average, 58% at most with many pads
held.

## Developing

- `engine/`: the sound, with no hardware code (it also runs on the Mac). `simple_touch.cpp` is
  the Simple Touch front end; `ui/pad_pages.h` the pad logic.
- **Tests on the Mac** (needs a C++17 compiler, e.g. Xcode's Command Line Tools):
  `sim/run_tests.sh` (prints `All tests passed`).
- **Listen on the Mac:** `sim/render.sh in.wav out.wav [option=value ...]` renders a 16-bit WAV
  (mono or stereo) through the engine. Options: `tune=Hz fb= mix= ring= carrier=Hz filter=Hz q= open= reception=
  detent= inertia= reverb=0-1 amount= effects=name,name every=s sweep=Hz freeze_at=s
  loop_at=s stereo=linked|mirror|rotate` (effect names are in `engine/effects.h`).
- **Hardware diagnostics** (`tools/`; each built with `make` and flashed with `make program-dfu`
  from its folder, through the bootloader as above, then read over USB; flash bit-chifter again
  afterwards): `hwtest/` prints the analog inputs, both switches and the pads; `padtest/`
  each pad's raw touch strength; `leveltest/` the input levels, with a test tone or
  passthrough on switch 1.
- **Design history:** `docs/` (see `docs/README.md`).

### Hardware notes (measured)

| Control | Seed input |
|---|---|
| Fader 1, fader 2 | A6, A7 |
| Knobs 1–6 | A1, A2, A3, A4, A0, A5 |

Both 3-way switches (libDaisy `Switch3`, switch 1 `Init(D9, D8)`, switch 2 `Init(D7, D6)`) read
mirror-image: physically left reads `POS_RIGHT`, right reads `POS_LEFT`. The pads are an MPR121
on I²C1 at 0x5A (default libDaisy config): channels 0–2 top left, top centre, top right; 3–7
left edge, centre-left, centre strip, centre-right, right edge; 8–9 bottom left and right;
10 "TOU" and 11 "CH" of the TOUCH logo.

## Future

A Daisy Patch front end (knobs, CV, screen and MIDI in) reusing the same engine. MIDI in would
also bring a **musical pitch shifter** (transposing by intervals, keeping the harmonics),
which the Simple Touch can't play without a MIDI input.

## License

bit-chifter is released under the [MIT License](LICENSE). It builds against
[libDaisy](https://github.com/electro-smith/libDaisy) (MIT).
