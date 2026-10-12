# Bitchshifter — design

A frequency shifter and ring modulator for the Synthux Simple Touch, voiced as a
**shortwave sideband receiver**: voices sliding into "Donald Duck" pitch as the dial
is mistuned, heterodyne whistles, a narrow boxy band, drifting, fading and static.
Weird first; a clean, tuned shift is a bonus. It usually feeds other effects (e.g.
Weave), so it has no reverb or delay of its own (reverb may be added last).

Inspired by the Doepfer A-126-2 (analog frequency shifter, up/down mix, squelch) and
FreakShift (Reason RE: drive → ring mod → frequency shifter, feedback, width, LFO,
envelope follower). Local project only (`~/local_files/daisy/bitchshifter`).

## Hardware

- **Board:** Synthux Simple Touch with a **Daisy Seed 1.2 (Rev7, PCM3060)** for now; a
  Seed 3 later (needs a new socket). Same MCU and memory; no code changes between them.
  Daisy bootloader v6.4 installed; programs are `APP_TYPE = BOOT_SRAM`.
- **Controls** (from Synthux's TouchFX source):
  - 8 analog inputs on A0–A7 (Synthux names S30–S37): **6 knobs + 2 faders**. Which two are
    the faders is not documented; build step 1 measures it.
  - Two 3-way switches: D7/D6 and D9/D8 (libDaisy `Switch3`).
  - 12 touch pads: MPR121 on I²C1 at 0x5A (libDaisy `Mpr121I2C`, default config).
  - Stereo audio in/out on the Seed's codec. No screen, no CV.

## Controls

### Faders
| Fader | Control |
|---|---|
| **1 TUNE** | Base shift: −2500 Hz … **0 at centre** … +2500 Hz. Response is cubic around the centre (fine mistuning near 0, wild at the ends). Stations and keyboard offsets add to it. |
| **2 RECEPTION** | Macro, clear → lost in the noise: static and crackle level, fading depth, drift amount, AGC strength and drive all rise together. At the bottom the receiver effects are off. |

### Knobs
| Knob | Control |
|---|---|
| **1 RING** | Ring-mod amount, 0–100% (crossfade from unmodulated to fully ring-modulated) |
| **2 CARRIER** | Ring-mod carrier, 0.1 Hz – 5 kHz (exponential). In Keyboard mode: transpose ±12 semitones. |
| **3 FILTER** | Radio filter centre, 150 Hz – 5 kHz (exponential) |
| **4 WIDTH** | Radio filter width: very narrow and resonant ("whistly") → wide open |
| **5 FEEDBACK** | Shifter feedback, 0 – 95% (barberpole sweeps) |
| **6 MIX** | Dry/wet, 0–100% |

### Switch 1: pad mode
| Position | Pads |
|---|---|
| **Stations** | 12 stored tunings (shift offset, filter centre and width, carrier, ring amount). Touch = glide ("dial swoop") from the current tuning to the station. Hold a pad ≥ 2 s = store the current settings there (saved to QSPI). |
| **Keyboard** | Chromatic octave, pad 1–12 = C…B, playing the ring-mod carrier (base set by knob 2 transpose; middle octave C3 = 130.8 Hz). The shift also follows the note when knob 1 RING is at 0 (musical, pitch-shift-like use). **Hold:** touching a pad for ≥ 1 s latches it; the note keeps sounding until another pad is touched. |
| **Performance** | Momentary (or latched, see switch 2) effects — initial set, refined at step 6: 1 feedback burst (95%), 2 static burst, 3 filter slam (narrowest, max resonance), 4 signal fades out, 5 drift to maximum, 6 ring mod 100%, 7 squelch (wet muted), 8 dial spins up, 9 dial spins down, 10 hold drift (freeze the wander), 11 carrier ×2, 12 kill dry. |

### Switch 2: modifier (depends on switch 1)
| Mode | Left | Middle | Right |
|---|---|---|---|
| Keyboard | Octave down | Normal | Octave up |
| Stations | Snap (≈ 50 ms glide) | Swoop (≈ 600 ms) | Slow drift (≈ 3 s) |
| Performance | Momentary | Momentary | Latch (tap on / tap off) |

## Signal chain (per sample, 48 kHz, block 48)

```
IN L+R (summed to mono) → Drive → Ring mod → Frequency shifter ⟲ feedback → Radio filter
  → Reception (fading · static · AGC) → Mix with dry → soft limiter → OUT L / OUT R
```

The input is summed to mono so mono sources work on either jack. **Stereo comes from
the receiver:** from the shifter on, L and R are processed by separate instances whose
shift drift and fading wander independently (shared controls, different random paths).

1. **Drive:** `tanh`-style soft clip, gain 0 → +24 dB (from RECEPTION, 0 dB at its bottom).
2. **Ring mod:** `out = in × (1 − a) + in × carrier × a`, sine carrier from a phase
   accumulator; `a` = knob 1.
3. **Frequency shifter:**
   - Hilbert phase splitter: two parallel chains of first-order allpass sections
     (published 90° coefficient set, e.g. Olli Niemitalo's 4+4 sections, ≥ 40 dB image
     rejection from ~40 Hz to ~18 kHz).
   - Quadrature oscillator at the shift frequency (sin/cos from one phase accumulator);
     negative shift runs it backwards, so TUNE sweeps through 0.
   - Single-sideband output `I·cos − Q·sin`.
   - **Feedback:** output → 2 ms delay → `tanh` soft clip → × knob 5 → added to the
     shifter input. The delay keeps the loop stable and gives the barberpole its smear.
   - Shift = TUNE + station/keyboard offset + drift, glided.
4. **Radio filter:** two state-variable band-pass filters in series (DaisySP `Svf`),
   same centre (knob 3) and Q from knob 4 (≈ 0.7 wide → ≈ 12 narrow), with gain
   compensation so narrowing doesn't drop the level much.
5. **Reception** (amounts scale with fader 2):
   - **Drift:** smoothed random walk added to the shift, up to ±40 Hz, rate ~0.1–0.5 Hz.
   - **Fading:** slow random LFO (0.05–0.5 Hz) moving a notch/comb (multipath) and a
     level swell of up to −18 dB.
   - **Static:** band-limited noise (shaped by the radio filter) plus random crackles
     (sparse impulses through a short decay).
   - **AGC:** envelope follower (attack ~5 ms, release ~300 ms) driving gain toward a
     target level, max boost +18 dB × RECEPTION. Quiet passages let the static swell; it
     also keeps the level steady for downstream effects.
6. **Mix** with the dry (mono-summed) input, knob 6.
7. **Soft limiter** on the outputs.

## Code structure

```
bitchshifter/
  engine/            sound only, no hardware code; one file per stage
    drive.h  ring_mod.h  hilbert.h  shifter.h  radio_filter.h  reception.h  engine.h/.cpp
  simple_touch.cpp   Simple Touch front end: knobs, faders, switches, pads, pad modes,
                     stations storage → engine parameters; audio callback
  sim/               Mac harness: runs the engine on a WAV or test tones, writes a WAV
  hwtest/            step 1 hardware test firmware
  Makefile           libDaisy/DaisySP one level up, APP_TYPE = BOOT_SRAM
```

- `Engine` takes a block of audio and a `Params` struct (tune, reception, ring, carrier,
  filter, width, feedback, mix, plus pad/station overrides) and returns processed L/R.
  Each stage is a small class with `Init(sample_rate)` and `Process(...)`.
- A later `patch.cpp` front end (knobs + CV + screen) reuses the engine unchanged.
- Stations are saved to QSPI at **0x7C0000** (Plooper 0x7D0000, Edges 0x7E0000,
  Grainwaves 0x7F0000), with a version number so format changes don't load garbage.

## Safety

- Soft clip in the feedback loop, feedback capped at 95%, NaN/Inf check per block that
  resets the shifter, filters and feedback state (as Plooper does for its reverb).
- SDRAM, if used, is zeroed before `Init()`.
- Output soft limiter; AGC boost capped.

## Testing

- **Mac harness** (like Plooper's `sim/`, with the macOS SDK flag from SESSIONS.md):
  - Shift accuracy: 1 kHz sine with +100 Hz shift → peak at 1100 Hz, image at 900 Hz at
    least 40 dB lower; −100 Hz → 900 Hz; 0 Hz → unchanged.
  - Stability: maximum feedback, drive and AGC for 60 s → no NaN, output bounded.
  - Listening renders of a voice or loop at a few settings, for the user.
- **Hardware:** CPU load via libDaisy `CpuLoadMeter`, reported over USB serial (no screen).

## Build order

1. **Hardware test firmware:** reports over USB serial which analog input is which
   knob/fader, plus pad and switch states. Fixes the control map.
2. **Shifter on the Mac:** Hilbert + quadrature oscillator + feedback, accuracy tests,
   a render to listen to.
3. **Shifter on the Simple Touch:** TUNE fader and feedback knob only.
4. **Ring mod and radio filter.**
5. **Reception:** drift, fading, static, AGC.
6. **Pads and switches:** stations (with QSPI storage), keyboard, performance.
7. **Later, optional:** reverb; the Daisy Patch front end.

## Out of scope (for now)

MIDI, CV, a screen UI, presets beyond the 12 stations, external carrier input, reverb.
