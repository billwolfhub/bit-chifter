# bit-chifter — strange pages design

Adds 30 pad effects on three pages, and a capture switch (Normal / Freeze / Loop), to make
bit-chifter much stranger and more active than a filter. Replaces the earlier plan for the
pads and switches (Stations / Keyboard / Performance). Stations and Keyboard are deferred:
they may come back if the sounds become distinct enough to be worth storing.

Main spec: `2026-10-12-bitchshifter-design.md` (faders, knobs, signal chain, safety).

## Controls

- **Switch 1 (left switch): page.** Physically left = page 1 **Shift & time**, middle =
  page 2 **Radio**, right = page 3 **Digital & damage**. (The switch reads mirror-image:
  see README.)
- **Switch 2 (right switch): capture.** Physically left = **Normal** (live input),
  middle = **Freeze**, right = **Loop** (see Capture).
- **Pads (10 per page):**
  - **Tap** (released within 0.3 s) **latches**: an effect that was off stays on; one
    that was latched turns off.
  - **Hold** (0.3 s or longer) is **momentary**: on while held, off on release (even if it
    was latched before).
  - The effect starts the moment the pad is touched, in both cases.
  - Effects from all pages stack. A latched effect stays on when the page changes; a held
    one stays on until its pad is released.
- **TOU logo pad:** a tap steps the stereo mode, as now; **holding it 1 s clears every
  latched effect** (confirmed by one low blip).
- **CH logo pad:** still reserved.
- **RECEPTION (fader 2) is also the effect amount:** every active effect uses
  `amount = 0.35 + 0.65 × reception`, so an effect is clearly audible with the fader at
  the bottom and extreme at the top. The fader keeps its reception effects too.

### Pad layout

Pad channels (from the README pad map): 0 top left, 1 top centre (narrow, hard to reach),
2 top right, 3 left edge, 4 centre left (big), 5 centre strip, 6 centre right (big),
7 right edge, 8 bottom left, 9 bottom right. The headline effects go on the big pads; the
least-needed one goes on the top centre.

| Channel | Page 1: Shift & time | Page 2: Radio | Page 3: Digital & damage |
|---|---|---|---|
| 4 (big, left) | Echoes | Scrambler | Bitcrush |
| 6 (big, right) | Spiral up | Passing carriers | Stutter |
| 3 (left edge) | Reverse echoes | Hopping scrambler | Sample-rate crush |
| 7 (right edge) | Spiral down | Morse chimes | Dropouts |
| 5 (centre strip) | Ping-pong echoes | Jammer | Codec warble |
| 0 (top left) | Chirp | Static burst | Ring 100% |
| 2 (top right) | Warble | Squelch | Carrier ×2 |
| 8 (bottom left) | Dial spin down | Signal fade-out | Kill dry |
| 9 (bottom right) | Dial spin up | Filter slam | Chaos |
| 1 (top centre) | Feedback burst | Drift to maximum | Hold drift |

## The effects

`a` is the amount (0.35–1, from RECEPTION). Every effect passes the sound through exactly
when off, and fades in and out over 10–30 ms so switching never clicks.

### Page 1: Shift & time

- **Echoes:** the shifter's feedback delay grows from 2 ms to an echo: 150 ms (a = 0.35)
  to 900 ms (a = 1). Each repeat goes through the shifter again, so it comes back shifted
  further (with TUNE off centre: spiralling, chiming repeats). Feedback is the FEEDBACK
  knob, but at least 50% while an echo is on. Changes of echo time glide over ~0.5 s like
  tape, warping the pitch. A gentle low-pass (~6 kHz) and the existing soft clip sit in the
  loop.
- **Reverse echoes:** like Echoes, but each repeat plays backwards (the delay is read in
  reverse, in crossfaded windows one echo long).
- **Ping-pong echoes:** like Echoes, but the repeats alternate between the sides, the left
  loop shifting up and the right shifting down.
- The three echo pads combine: any of them turns the echo on; Reverse and Ping-pong add
  their behaviour.
- **Spiral up / Spiral down:** an endless rising or falling shift, like a Shepard tone.
  Each side runs a second shifter; the two shifters' extra shifts ramp from 0 to 300 Hz,
  half a cycle apart, and crossfade so each ramp's restart is silent. Rate 0.05 (a = 0.35)
  to 0.5 cycles/s (a = 1). It adds to the dial. If both pads are on, the last one touched
  wins.
- **Chirp:** the shift follows the input's loudness (envelope: 2 ms attack, 150 ms
  release), up to +300 Hz (a = 0.35) to +1500 Hz (a = 1) at full level, so notes swoop
  up as they start.
- **Warble:** an LFO (a sine with some randomness) on the shift: depth 20–400 Hz and rate
  0.5–8 Hz, both rising with a.
- **Dial spin down / up:** while on, the dial keeps turning down or up at 150 Hz/s
  (a = 0.35) to 3000 Hz/s (a = 1), within ±2 kHz in total. When released, it returns with
  a ~1 s glide.
- **Feedback burst:** feedback to 95% (gliding over 30 ms).

### Page 2: Radio

- **Scrambler:** spectral inversion, like an old radio voice scrambler: the spectrum is
  turned upside down around an inversion frequency (the analytic signal is conjugated and
  shifted up by it, then low-passed at it), from 3.4 kHz (a = 0.35) down to 1.2 kHz
  (a = 1). It sits after Drive, before the ring mod.
- **Hopping scrambler:** like Scrambler, but the inversion frequency jumps to a random
  value between 1 and 4 kHz every 1.5 s (a = 0.35) to 0.1 s (a = 1), with 5 ms crossfades.
  Either scrambler pad turns the scrambler on; Hopping adds the hops.
- **Passing carriers:** 2 (a = 0.35) to 6 (a = 1) hidden "stations": steady sine carriers
  at random fixed positions within ±2.5 kHz of the dial. You hear each at its offset from
  the dial, so moving TUNE sweeps them through high, zero and back (heterodyne whistles).
  Level −30 to −12 dBFS with a; they drift slowly. Mixed in after the shifter, before the
  radio filter.
- **Morse chimes:** 1–3 carriers keyed in random Morse-like dits and dahs (12–25 words per
  minute), each key-down a tone with a soft attack and a bell-like decay, so they chime.
  Their pitch follows the dial like the passing carriers.
- **Jammer:** a sawtooth-swept tone (sweeping 300 Hz – 3 kHz at 0.5–4 Hz) plus noise
  bursts, over the signal; level rises with a.
- **Static burst:** static to its maximum level.
- **Squelch:** the wet signal mutes; on release it opens with a short squelch-tail noise
  burst.
- **Signal fade-out:** the signal fades to −40 dB over 1.5 s while on, and back after.
- **Drift to maximum:** the drift goes to its maximum, whatever RECEPTION is.
- **Filter slam:** the radio filter jumps to its narrowest and most resonant.

### Page 3: Digital & damage

These sit after the radio filter, before the AGC (so the AGC pumps on the damage).

- **Bitcrush:** 10 bits (a = 0.35) down to 3 bits (a = 1), no dither.
- **Sample-rate crush:** sample-and-hold at 12 kHz (a = 0.35) down to 1.5 kHz (a = 1),
  with no anti-aliasing.
- **Dropouts:** random gaps of 20–200 ms with hard (1 ms) edges, 0.3 (a = 0.35) to 4
  (a = 1) per second.
- **Stutter:** catches 30–120 ms fragments and repeats each 2–8 times; how often rises
  with a.
- **Codec warble:** the watery sound of a low-bitrate stream: an 8-band filter bank whose
  bands are each gated and re-levelled at random 15–40 times a second (depth rising with
  a), with the top cut at 4 kHz.
- **Ring 100%:** ring mod fully on.
- **Carrier ×2:** the ring-mod carrier an octave up.
- **Kill dry:** the dry signal muted (wet only).
- **Hold drift:** the reception wander and drift freeze where they are.
- **Chaos:** a tap turns on a new random set of 2–3 effects from the other 29 pads (a
  separate "chaos set", which replaces the previous one; the user's own latches are left
  alone). A hold turns the chaos set off on release. Clear-all (hold TOU) clears it too.

## Capture (switch 2)

At the input, before Drive, so the whole chain (dry included) processes the captured sound.

- **Normal:** live input.
- **Freeze:** on entering, the last 250 ms are held as an endless sound: two overlapping
  read heads with smooth (Hann) windows loop the slice, so it drones without clicking or
  fluttering.
- **Loop:** on entering, the last 2 s are looped, with a 20 ms crossfade at the loop
  point.
- Returning to Normal crossfades back to the live input over 30 ms. Moving between Freeze
  and Loop captures afresh.

## Architecture

- **Engine (`engine/`, hardware-free):** `Params` gains:
  - `uint32_t effects`: one bit per effect (an `Effect` enum, numbered as in the table,
    page by page);
  - `float effect_amount`: 0–1 (the front end passes `0.35 + 0.65 × reception`);
  - `Capture capture`: Normal / Freeze / Loop.
  Each effect is a small class with `Init` and `Process`, tested on the Mac. The engine
  reads the bits; nothing in it knows about pads.
- **Pad logic (`ui/pad_pages.h`, hardware-free):** turns touches, the page switch, TOU and
  the clock into the effects bits (tap/hold, latches, clear-all, chaos), so it can be
  tested on the Mac. `simple_touch.cpp` only reads the hardware and calls it.
- **Signal chain (changes in bold):**
  1. **Capture**
  2. Drive → **Scrambler** → Ring mod
  3. Shifter: shift = dial + tuning hand + **chirp + warble + spin + spiral**, then
     detents, stereo gains and drift; **the feedback delay grows into the echoes**;
     **a second shifter per side for the spiral**
  4. Fading + static + **passing carriers + Morse chimes + jammer**
  5. Radio filter → **Digital damage** → AGC
  6. Dry/wet mix, swap, soft clip, NaN guard (as now)
- **Memory:** the echo buffers (~1 s per side) and the capture buffer (2 s stereo) live in
  the Seed's 64 MB SDRAM (`DSY_SDRAM_BSS`), zeroed at start-up (uninitialised SDRAM holds
  NaNs).
- **CPU:** now ~22%. Budget: under ~60% with every effect on, measured on the hardware
  (USB report).

## Safety

- The echo loop keeps the soft clip and the 95% feedback cap; the low-pass in the loop
  stops high-frequency runaway.
- The existing output soft clip and NaN guard stay last.
- Stability test: every effect on, maximum feedback, 60 s → no NaN, output bounded.

## Testing

- Each effect: exact pass-through when off; on/off fades without steps (largest
  sample-to-sample jump bounded); its main behaviour (e.g. echo time, crush bits, carrier
  pitch follows the dial, inversion maps 1 kHz to the expected frequency).
- Capture: Freeze and Loop repeat the captured audio; returning to Normal is click-free.
- Pad logic: tap latches and unlatches, hold is momentary, page changes keep latches,
  TOU hold clears, chaos picks 2–3 other effects and replaces its previous set.
- Renders for the user at each step; hardware: flash, play, CPU with everything on.

## Build order (one plan each; renders, then flashing, after each)

1. **Pages:** the page switch, pad logic (tap/hold, latches, TOU clear-all), the effects
   bits in the engine, and the cheap effects: feedback burst, static burst, signal
   fade-out, drift to maximum, filter slam, squelch, ring 100%, carrier ×2, kill dry, hold
   drift, dial spin up/down. Other pads do nothing until their step.
2. **Time:** Echoes, Reverse and Ping-pong echoes; Freeze and Loop (shared buffer code).
3. **Motion:** Chirp, Warble, Spiral up/down.
4. **Radio:** Scrambler, Hopping scrambler, Passing carriers, Morse chimes, Jammer.
5. **Digital:** Bitcrush, Sample-rate crush, Dropouts, Stutter, Codec warble, Chaos.

## Out of scope (for now)

Stations and Keyboard (deferred); the CH pad's modes; per-effect amount controls; reverb;
the Daisy Patch front end.
