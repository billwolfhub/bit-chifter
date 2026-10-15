# bit-chifter Strange Pages Step 4 (Radio) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build page 2's radio effects: Scrambler, Hopping scrambler, Passing carriers, Selcall, Morse chimes and Jammer.

**Architecture:** Each effect is a small header-only class in `engine/`, tested on its own on the Mac: `Scrambler` (spectral inversion with a Hilbert pair, hopping inversion frequency), `Carriers` (hidden stations heard as heterodyne whistles, moving with TUNE like the voice, faded out near zero beat), `MorseChimes` (keyed carriers with a bell-like envelope), `Selcall` (5-tone ZVEI/CCIR bursts) and `Jammer` (swept buzz plus noise bursts). The engine runs each only while its pad's level (the existing ~30 ms per-effect fade, `level_[]`) is above 0, so with the pads off the sound is bit-for-bit unchanged. Selcall goes into the input (before Drive), so TUNE mistunes it like a real receiver; the scrambler sits after Drive, before the ring mod; carriers, Morse chimes and the jammer are added with the static, before the radio filter.

**Tech Stack:** C++17 (also gnu++14 for the firmware), libDaisy v8.1, Apple clang for `sim/`.

**Spec:** `docs/superpowers/specs/2026-10-13-bit-chifter-strange-pages-design.md` (Page 2: Radio; Signal chain).

**Conventions:**
- Project root `~/local_files/daisy/bit-chifter`, git, master (public on GitHub; don't push, the user asks for that). No `Co-Authored-By` or other attribution lines in commits.
- Tests: `sim/run_tests.sh` (prints `All tests passed`). Helpers in `sim/test_engine.cpp`: `CHECK(cond, fmt, ...)`, `kSampleRate` (48000.f), `kPi`, `Magnitude(x, freq)`, `Db(ratio)`, `Window(x, from_s, to_s)`, `Peak(x)`, `Energy(x)`, `Rms(x)`, `Sine(hz, amplitude, n)`, `RunFor(engine, params, seconds, signal, n, l, r)` (runs 48-sample blocks of `signal(n)` into both inputs, appending both outputs), `kBuiltEffects` (the effects already built; used by `TestEffectsUnbuiltDoNothing`). New test functions go above `main()` and are called from it.
- Engine code: single precision only (`sinf`, `1.f`), nothing allocating in `Process()`; the firmware builds as gnu++14 (no odr-used `static constexpr` members: use enums or functions).
- If a test fails after implementing, don't change tolerances or the given algorithm to make it pass: report the printed numbers.
- Hardware steps: the user listens to renders before the render commit, and flashes and confirms before the firmware commit.

## File structure

| File | Responsibility |
|---|---|
| `engine/effects.h` | `EffectRange(amount)`: 0.35–1 → 0–1 |
| `engine/scrambler.h` | `Scrambler` |
| `engine/rotor.h` | `Rotor` (a quadrature oscillator, retuned per block; added in review to replace per-sample trig) |
| `engine/carriers.h` | `ZeroBeatTaper()`, `Carriers` |
| `engine/morse_chimes.h` | `MorseChimes` |
| `engine/selcall.h` | `Selcall` |
| `engine/jammer.h` | `Jammer` |
| `engine/engine.h`, `engine/engine.cpp` | the six effects wired in; `SelcallToneHz()` test hook |
| `sim/test_engine.cpp` | tests |
| `simple_touch.cpp`, `README.md` | comment and controls table (the pads already send the effect bits) |

---

### Task 1: The scrambler

**Files:**
- Modify: `engine/effects.h`
- Create: `engine/rotor.h` (see Task 3; the review moved the scrambler onto it)
- Create: `engine/scrambler.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "scrambler.h"` with the engine includes, and above `main()`:
```cpp
// Runs a scrambler for `seconds` (block 48: Update per block, Process per sample) on a sine
static std::vector<float> RunScrambler(bitch::Scrambler &s, float amount, bool hopping, float seconds, float hz) {
    std::vector<float> out;
    for (int b = 0; b < (int)(seconds * kSampleRate / 48); b++) {
        s.Update(amount, hopping, 48.f / kSampleRate);
        for (int k = 0; k < 48; k++) out.push_back(s.Process(Sine(hz, 0.25f, b * 48 + k)));
    }
    return out;
}

static void TestScrambler() {
    CHECK(bitch::EffectRange(0.35f) == 0.f && bitch::EffectRange(1.f) == 1.f &&
              fabsf(bitch::EffectRange(0.675f) - 0.5f) < 1e-6f,
          "EffectRange");
    bitch::Scrambler s;
    s.Init(kSampleRate, 5u);
    // Amount 0.35: inverted around 3.4 kHz, so 1 kHz comes out at 2.4 kHz
    auto w = Window(RunScrambler(s, 0.35f, false, 0.5f, 1000.f), 0.25f, 0.5f);
    CHECK(fabsf(s.InversionHz() - 3400.f) < 1.f, "scrambler: inverts at %.0f Hz (want 3400)", s.InversionHz());
    CHECK(Magnitude(w, 2400.f) > 10 * Magnitude(w, 1000.f) && Magnitude(w, 2400.f) > 10 * Magnitude(w, 4400.f),
          "scrambler: 2400 Hz %.4f, 1000 Hz %.4f, 4400 Hz %.4f", Magnitude(w, 2400.f), Magnitude(w, 1000.f),
          Magnitude(w, 4400.f));
    // Amount 1: inverted around 1.2 kHz, so 400 Hz comes out at 800 Hz
    s.Reset();
    w = Window(RunScrambler(s, 1.f, false, 0.5f, 400.f), 0.25f, 0.5f);
    CHECK(Magnitude(w, 800.f) > 10 * Magnitude(w, 400.f), "scrambler at 1.2 kHz: 800 Hz %.4f, 400 Hz %.4f",
          Magnitude(w, 800.f), Magnitude(w, 400.f));
    // Above the inversion frequency: removed, not folded back
    s.Reset();
    w = Window(RunScrambler(s, 1.f, false, 0.5f, 3000.f), 0.25f, 0.5f);
    CHECK(Rms(w) < 0.02f, "scrambler: 3 kHz above a 1.2 kHz inversion still %.4f", Rms(w));
    // Band-limited like a real scrambler: just above the inversion frequency doesn't fold back
    // into the band, and DC doesn't leak out as a tone at f
    s.Reset();
    float in_band = Rms(Window(RunScrambler(s, 1.f, false, 0.5f, 400.f), 0.25f, 0.5f));
    for (float hz : {1300.f, 1400.f}) {
        s.Reset();
        float above = Rms(Window(RunScrambler(s, 1.f, false, 0.5f, hz), 0.25f, 0.5f));
        CHECK(Db(above / in_band) < -30.0, "scrambler at 1.2 kHz: %.0f Hz folds back at %.1f dB", hz,
              Db(above / in_band));
    }
    s.Reset();
    std::vector<float> dc;
    for (int b = 0; b < (int)(0.5f * kSampleRate / 48); b++) {
        s.Update(1.f, false, 48.f / kSampleRate);
        for (int k = 0; k < 48; k++) dc.push_back(s.Process(0.25f));
    }
    CHECK(Rms(Window(dc, 0.25f, 0.5f)) < 1e-3f, "scrambler: DC in leaks out at %g", Rms(Window(dc, 0.25f, 0.5f)));
    // Hopping at amount 1: a new inversion frequency every 0.1 s, between 1 and 4 kHz
    s.Reset();
    std::vector<float> seen;
    for (int b = 0; b < (int)(2.f * kSampleRate / 48); b++) {
        s.Update(1.f, true, 48.f / kSampleRate);
        for (int k = 0; k < 48; k++) s.Process(0.f);
        if (b % 50 == 49) seen.push_back(s.InversionHz());
    }
    int changes = 0;
    bool in_range = true;
    for (size_t k = 1; k < seen.size(); k++) {
        changes += fabsf(seen[k] - seen[k - 1]) > 10.f;
        in_range = in_range && seen[k] >= 999.f && seen[k] <= 4001.f;
    }
    CHECK(changes >= 10 && in_range, "hopping scrambler: %d changes in 2 s (want >= 10), in range %d", changes,
          (int)in_range);
}
```
Call `TestScrambler();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: compile error, `'scrambler.h' file not found`.

- [ ] **Step 3: Write the scrambler**

In `engine/effects.h`, after `Has()`:
```cpp
// An effect's amount (0.35-1, from RECEPTION) as 0-1, for its range
inline float EffectRange(float amount) {
    float u = (amount - 0.35f) / 0.65f;
    return u < 0.f ? 0.f : (u > 1.f ? 1.f : u);
}
```

`engine/scrambler.h`:
```cpp
#pragma once

#include <cmath>
#include <cstdint>

#include "effects.h"
#include "hilbert.h"
#include "random.h"
#include "rotor.h"

namespace bitch {

// Voice scrambler, like old radio scramblers: the spectrum is turned upside down around an
// inversion frequency f (0..f becomes f..0). Band-limited in and out like a real scrambler:
// the input is high-passed at 250 Hz (2nd-order Butterworth; this also keeps DC from coming
// out as a tone at f) and low-passed at 0.75 f (8th-order Butterworth), so what's above f is
// removed rather than folded back (just above f: at least 30 dB down); then split into I and
// Q, and mixed with an oscillator at f taking the opposite sideband from the shifter's:
// i*cos - q*sin; and the output is high-passed at 250 Hz again (what's left of the fold-back
// lands low, so this takes it further down).
// Plain: f from 3.4 kHz (amount 0.35) down to 1.2 kHz (amount 1). Hopping: f jumps to a
// random 1-4 kHz every 1.5 s (amount 0.35) down to every 0.1 s (amount 1), gliding over
// ~5 ms (per block). Update() once per block, Process() per sample.
class Scrambler {
  public:
    void Init(float sample_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        random_.Seed(seed);
        float g = tanf(3.14159265359f * 250.f / sample_rate);
        high_pass_.Set(g, 1.41421356f);
        out_high_pass_.Set(g, 1.41421356f);
        Reset();
    }

    void Reset() {
        hilbert_.Reset();
        high_pass_.ic1 = high_pass_.ic2 = out_high_pass_.ic1 = out_high_pass_.ic2 = 0.f;
        for (Stage &s : low_pass_) s.ic1 = s.ic2 = 0.f;
        rotor_.Reset();
        inversion_hz_ = target_hz_ = 3400.f;
        hop_left_s_ = 0.f;
        Retune();
    }

    void Update(float amount, bool hopping, float block_seconds) {
        float u = EffectRange(amount);
        if (!hopping) {
            target_hz_ = 3400.f - 2200.f * u;
        } else {
            hop_left_s_ -= block_seconds;
            if (hop_left_s_ <= 0.f) {
                target_hz_ = 1000.f + 3000.f * random_.Uniform();
                hop_left_s_ = 1.5f * powf(0.1f / 1.5f, u);
            }
        }
        if (block_seconds != block_seconds_) { // ~5 ms glide, per block
            block_seconds_ = block_seconds;
            glide_ = 1.f - expf(-block_seconds / 0.005f);
        }
        inversion_hz_ += glide_ * (target_hz_ - inversion_hz_);
        Retune();
    }

    float Process(float x) {
        float y = high_pass_.HighPass(x);
        for (Stage &s : low_pass_) y = s.LowPass(y);
        float i, q, c, s;
        hilbert_.Process(y, i, q);
        rotor_.Next(c, s);
        return out_high_pass_.HighPass(i * c - q * s);
    }

    float InversionHz() const { return target_hz_; }

  private:
    // Andrew Simper's state-variable filter (g = tan(pi fc / fs), k = 1 / Q)
    struct Stage {
        float a1 = 0.f, a2 = 0.f, a3 = 0.f, k = 0.f, ic1 = 0.f, ic2 = 0.f;
        void Set(float g, float damping) {
            k = damping;
            a1 = 1.f / (1.f + g * (g + k));
            a2 = g * a1;
            a3 = g * a2;
        }
        float LowPass(float x) {
            float v1, v2;
            Tick(x, v1, v2);
            return v2;
        }
        float HighPass(float x) {
            float v1, v2;
            Tick(x, v1, v2);
            return x - k * v1 - v2;
        }
        void Tick(float x, float &v1, float &v2) {
            float v3 = x - ic2;
            v1 = a1 * ic1 + a2 * v3;
            v2 = ic2 + a2 * ic1 + a3 * v3;
            ic1 = 2.f * v1 - ic1;
            ic2 = 2.f * v2 - ic2;
        }
    };

    // The low-pass and the oscillator follow the inversion frequency (per block)
    void Retune() {
        float g = tanf(3.14159265359f * fminf(0.75f * inversion_hz_, 0.45f * sample_rate_) / sample_rate_);
        static const float kK[4] = {1.f / 0.5098f, 1.f / 0.6013f, 1.f / 0.9000f, 1.f / 2.5629f}; // Butterworth 8th order
        for (int n = 0; n < 4; n++) low_pass_[n].Set(g, kK[n]);
        rotor_.SetHz(inversion_hz_, sample_rate_);
    }

    Hilbert hilbert_;
    Stage high_pass_, low_pass_[4], out_high_pass_;
    Rotor rotor_;
    Random random_;
    float sample_rate_ = 48000.f, glide_ = 0.f, block_seconds_ = 0.f;
    float inversion_hz_ = 3400.f, target_hz_ = 3400.f, hop_left_s_ = 0.f;
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`. If the first scrambler check finds the energy at 4400 Hz instead of 2400 Hz, the sideband sign is the other way round for this Hilbert: report it (the fix is `i * c + q * s` together with checking `TestShiftUpAndDown`'s convention), don't loosen the check.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/effects.h engine/scrambler.h sim/test_engine.cpp
git commit -m "Engine: voice scrambler (spectral inversion, hopping)"
```

---

### Task 2: The scrambler in the engine

**Files:**
- Modify: `engine/engine.h`
- Modify: `engine/engine.cpp`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing test**

In `sim/test_engine.cpp`, add `bitch::Bit(bitch::kScrambler) | bitch::Bit(bitch::kHoppingScrambler) |` to `kBuiltEffects`, and above `main()`:
```cpp
static void TestEffectScrambler() {
    for (int on = 0; on < 2; on++) {
        bitch::Engine e;
        e.Init(kSampleRate);
        bitch::Params p;
        p.effects = on ? bitch::Bit(bitch::kScrambler) : 0u;
        std::vector<float> l, r;
        int n = 0;
        RunFor(e, p, 1.f, [](int k) { return Sine(1000.f, 0.25f, k); }, n, l, r);
        auto w = Window(l, 0.5f, 1.f);
        if (on)
            CHECK(Magnitude(w, 2400.f) > 10 * Magnitude(w, 1000.f), "scrambler pad: 2400 Hz %.4f vs 1000 Hz %.4f",
                  Magnitude(w, 2400.f), Magnitude(w, 1000.f));
        else
            CHECK(Magnitude(w, 1000.f) > 10 * Magnitude(w, 2400.f), "no scrambler: not the plain tone");
    }
    // The Hopping pad on its own turns the scrambler on too (the hops are tested in TestScrambler)
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.effects = bitch::Bit(bitch::kHoppingScrambler);
    p.effect_amount = 1.f;
    std::vector<float> l, r;
    int n = 0;
    RunFor(e, p, 2.f, [](int k) { return Sine(500.f, 0.25f, k); }, n, l, r);
    auto hop = Window(l, 1.f, 2.f);
    CHECK(Rms(hop) > 0.05f && Magnitude(hop, 500.f) < 0.2 * Rms(hop), "hopping scrambler pad: rms %.4f, 500 Hz %.4f",
          Rms(hop), Magnitude(hop, 500.f));
}
```
Call `TestEffectScrambler();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: FAIL lines for `scrambler pad`, and `unbuilt effects changed the sound` should still pass.

- [ ] **Step 3: Wire it in**

`engine/engine.h`: add `#include "scrambler.h"`; private members (after the spin members):
```cpp
    // Radio pads
    Scrambler scrambler_[2];
    bool scrambler_running_ = false; // reset when it starts, so no stale state fades in
```

`engine/engine.cpp`:
- In `Init`, after the per-side Reception `Init` loop: `for (Scrambler &s : scrambler_) s.Init(sample_rate, 83u); // the same seed: both sides hop together`.
- In `Reset()`: `for (Scrambler &s : scrambler_) s.Reset(); scrambler_running_ = false;`.
- In `Process()`, after the spin code (before the plate block), add:
```cpp
    // Radio pads: the scrambler (either pad), run only while it's audible
    float amount_radio = Clamp(params.effect_amount, 0.35f, 1.f);
    float scramble = fmaxf(level_[kScrambler], level_[kHoppingScrambler]);
    if (scramble > 0.f) {
        if (!scrambler_running_) {
            for (Scrambler &s : scrambler_) s.Reset();
            scrambler_running_ = true;
        }
        for (Scrambler &s : scrambler_) s.Update(amount_radio, Has(fx, kHoppingScrambler), (float)size / sample_rate_);
    } else {
        scrambler_running_ = false;
    }
```
- In the per-sample loop, after `float driven = drive_.Process(dry);`, add:
```cpp
            if (scramble > 0.f) driven += scramble * (scrambler_[c].Process(driven) - driven);
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`, including `TestEffectsUnbuiltDoNothing` and every earlier test.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/engine.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: scrambler pads (after Drive, before the ring mod)"
```

---

### Task 3: Passing carriers

**Files:**
- Create: `engine/rotor.h`
- Create: `engine/carriers.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "carriers.h"`, and above `main()`:
```cpp
static std::vector<float> RunCarriers(bitch::Carriers &c, float amount, float dial_hz, float seconds) {
    std::vector<float> out;
    for (int b = 0; b < (int)(seconds * kSampleRate / 48); b++) {
        c.Update(amount, 1.f, dial_hz);
        for (int k = 0; k < 48; k++) out.push_back(c.Process());
    }
    return out;
}

static void TestCarriers() {
    // Rotor: a steady sine at the set frequency
    bitch::Rotor rotor;
    rotor.SetHz(440.f, kSampleRate);
    std::vector<float> x;
    for (int n = 0; n < 24000; n++) x.push_back(rotor.Next());
    CHECK(Magnitude(x, 440.f) > 0.45 && fabsf(Peak(x) - 1.f) < 0.01f, "rotor: 440 Hz %.3f, peak %.3f",
          Magnitude(x, 440.f), Peak(x));
    bitch::Carriers c;
    c.Init(kSampleRate, kSampleRate / 48.f, 3u);
    CHECK(c.Count(0.35f) == 2 && c.Count(1.f) == 6, "carriers: %d and %d stations", c.Count(0.35f), c.Count(1.f));
    for (int k = 0; k < 6; k++)
        CHECK(fabsf(c.Position(k)) <= 2500.f && fabsf(c.Position(k)) >= 150.f, "station %d at %.0f Hz", k,
              c.Position(k));
    // Heard at position + dial (20 ms windows: the stations drift by up to 20 Hz)
    float s0 = c.Position(0);
    auto w = Window(RunCarriers(c, 0.35f, 0.f, 1.f), 0.5f, 0.52f);
    CHECK(Magnitude(w, fabsf(s0)) > 0.005, "carriers, dial 0: station 0 (%.0f Hz) only %.5f", s0,
          Magnitude(w, fabsf(s0)));
    float dial = s0 > 0.f ? 300.f - s0 : -300.f - s0; // heard at position + dial: 300 Hz
    w = Window(RunCarriers(c, 0.35f, dial, 1.f), 0.5f, 0.52f);
    CHECK(Magnitude(w, 300.f) > 5 * Magnitude(w, fabsf(s0)), "carriers, dial %.0f: 300 Hz %.5f vs %.0f Hz %.5f",
          dial, Magnitude(w, 300.f), fabsf(s0), Magnitude(w, fabsf(s0)));
    // Louder at amount 1 (more stations, -12 dBFS each instead of -30)
    float quiet = Rms(Window(RunCarriers(c, 0.35f, 0.f, 1.f), 0.5f, 1.f));
    float loud = Rms(Window(RunCarriers(c, 1.f, 0.f, 1.f), 0.5f, 1.f));
    CHECK(loud > 8 * quiet, "carriers: amount 1 %.4f vs 0.35 %.4f", loud, quiet);
}
```
Call `TestCarriers();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: compile error, `'carriers.h' file not found`.

- [ ] **Step 3: Write the carriers**

`engine/rotor.h`:
```cpp
#pragma once

#include <cmath>

namespace bitch {

// Quadrature sine oscillator by rotation: cheap per sample (4 multiplies), retuned per block
// (SetHz, two trig calls) and renormalised there, so its level stays exact. Frequencies may be
// negative (the rotation runs backwards; the sine is the same pitch). Retuning keeps the phase.
class Rotor {
  public:
    void SetHz(float hz, float sample_rate) {
        float w = 6.28318530718f * hz / sample_rate;
        cos_ = cosf(w);
        sin_ = sinf(w);
        Normalize();
    }
    // Pulls the level back to exactly 1 (SetHz does this; call it per block when not retuning)
    void Normalize() {
        float norm = 1.5f - 0.5f * (c_ * c_ + s_ * s_);
        c_ *= norm;
        s_ *= norm;
    }
    float Next() {
        float c = c_ * cos_ - s_ * sin_;
        s_ = s_ * cos_ + c_ * sin_;
        c_ = c;
        return s_;
    }
    // Both outputs: cos and sin of the same advancing angle
    void Next(float &c, float &s) {
        Next();
        c = c_;
        s = s_;
    }
    void Reset() {
        c_ = 1.f;
        s_ = 0.f;
    }

  private:
    float cos_ = 1.f, sin_ = 0.f, c_ = 1.f, s_ = 0.f;
};

} // namespace bitch
```

`engine/carriers.h`:
```cpp
#pragma once

#include <cmath>
#include <cstdint>

#include "effects.h"
#include "random.h"
#include "rotor.h"

namespace bitch {

// Smooth fade to 0 as a station's heard offset nears 0 Hz (full from 200 Hz): tuned onto a
// station, the whistle falls to zero beat and fades out, rather than putting out DC and subsonics
inline float ZeroBeatTaper(float hz) {
    float t = fminf(fabsf(hz) / 200.f, 1.f);
    return t * t * (3.f - 2.f * t);
}

// Passing carriers: hidden "stations", steady carriers at fixed random places within
// +-2.5 kHz (at least 150 Hz from 0). A receiver tuned to `dial_hz` hears each one as a
// whistle at position + dial (the dial moves them the same way as the voice), so moving TUNE
// sweeps them through high, zero beat (faded out within 200 Hz of it) and back. 2 stations
// (amount 0.35) to 6 (amount 1), each at -30 dBFS (amount 0.35) to -12 dBFS (amount 1), each
// drifting slowly (+-20 Hz). Update() once per block (with the dial and the pad's fade level;
// the gains are ramped across the next block), Process() per sample.
class Carriers {
  public:
    enum : int { kMax = 6 };

    void Init(float sample_rate, float block_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        block_ = (int)lrintf(sample_rate / block_rate);
        if (block_ < 1) block_ = 1;
        Random random(seed);
        for (int k = 0; k < kMax; k++) {
            float side = random.Uniform() < 0.5f ? -1.f : 1.f;
            position_[k] = side * (150.f + 2350.f * random.Uniform());
            drift_[k].Init(block_rate, seed * 31u + (uint32_t)k);
        }
        Reset();
    }

    // Silent, with the gains at 0 (the stations stay where they are)
    void Reset() {
        for (int k = 0; k < kMax; k++) {
            rotor_[k].Reset();
            gain_[k] = step_[k] = target_[k] = 0.f;
            left_[k] = 0;
        }
    }

    static int Count(float amount) { return 2 + (int)lrintf(4.f * EffectRange(amount)); }
    float Position(int k) const { return position_[k]; }

    void Update(float amount, float level, float dial_hz) {
        float u = EffectRange(amount);
        float each = powf(10.f, 0.05f * (-30.f + 18.f * u)) * level;
        int count = Count(amount);
        for (int k = 0; k < kMax; k++) {
            float hz = position_[k] + 20.f * drift_[k].Next(0.1f) + dial_hz;
            rotor_[k].SetHz(hz, sample_rate_);
            target_[k] = k < count ? each * ZeroBeatTaper(hz) : 0.f;
            step_[k] = (target_[k] - gain_[k]) / (float)block_; // ramped over the next block
            left_[k] = block_;
        }
    }

    float Process() {
        float out = 0.f;
        for (int k = 0; k < kMax; k++) {
            if (left_[k] > 0) {
                gain_[k] = --left_[k] > 0 ? gain_[k] + step_[k] : target_[k];
            }
            if (gain_[k] != 0.f) out += gain_[k] * rotor_[k].Next();
        }
        return out;
    }

  private:
    float sample_rate_ = 48000.f;
    int block_ = 48;
    float position_[kMax] = {};
    Wander drift_[kMax];
    Rotor rotor_[kMax];
    float gain_[kMax] = {}, step_[kMax] = {}, target_[kMax] = {};
    int left_[kMax] = {};
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/rotor.h engine/carriers.h sim/test_engine.cpp
git commit -m "Engine: passing carriers (heterodyne whistles from hidden stations)"
```

---

### Task 4: Morse chimes

**Files:**
- Create: `engine/morse_chimes.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "morse_chimes.h"`, and above `main()`:
```cpp
static void TestMorseChimes() {
    bitch::MorseChimes m;
    m.Init(kSampleRate, kSampleRate / 48.f, 9u);
    CHECK(m.Keyers(0.35f) == 1 && m.Keyers(1.f) == 3, "morse: %d and %d keyers", m.Keyers(0.35f), m.Keyers(1.f));
    std::vector<float> x;
    for (int b = 0; b < (int)(4.f * kSampleRate / 48); b++) {
        m.Update(0.35f, 1.f, 0.f);
        for (int k = 0; k < 48; k++) x.push_back(m.Process());
    }
    // Keyed: tones and gaps (10 ms windows)
    int on = 0, off = 0, edges = 0;
    bool was_on = false;
    for (float t = 0.f; t + 0.01f <= 4.f; t += 0.01f) {
        bool now_on = Rms(Window(x, t, t + 0.01f)) > 0.01f;
        on += now_on;
        off += Rms(Window(x, t, t + 0.01f)) < 0.002f;
        edges += now_on != was_on;
        was_on = now_on;
    }
    CHECK(on > 40 && off > 40 && edges >= 16, "morse: %d windows on, %d off, %d edges in 4 s", on, off, edges);
    // At the keyer's place (dial 0)
    float hz = fabsf(m.Position(0));
    CHECK(Magnitude(x, hz) > 5 * Magnitude(x, hz * 1.5f), "morse: %.0f Hz %.5f vs %.0f Hz %.5f", hz,
          Magnitude(x, hz), hz * 1.5f, Magnitude(x, hz * 1.5f));
    CHECK(Peak(x) < 0.2f, "morse: peak %.3f", Peak(x));
}
```
Call `TestMorseChimes();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: compile error, `'morse_chimes.h' file not found`.

- [ ] **Step 3: Write the Morse chimes**

`engine/morse_chimes.h`:
```cpp
#pragma once

#include <cmath>
#include <cstdint>

#include "carriers.h"
#include "effects.h"
#include "random.h"

namespace bitch {

// Morse chimes: 1 keyer (amount 0.35) to 3 (amount 1), each a carrier at a fixed random place
// within +-1.5 kHz of the dial (at least 200 Hz from 0; heard at position + dial, like the
// passing carriers, and faded out within 200 Hz of zero beat), keyed in random Morse-like letters at 12 (amount 0.35) to 25
// (amount 1) words per minute: dits (1 unit) and dahs (3 units), 1 unit apart; letters of
// 1-4 elements, 3 units apart; words of 2-5 letters, 7 units apart. Each key-down is a
// chime: a 3 ms attack, then a bell-like decay (120 ms), released in 20 ms at the key-up.
// About -18 dBFS per keyer. Update() once per block (the gains are ramped across the next
// block), Process() per sample.
class MorseChimes {
  public:
    enum : int { kMax = 3 };

    void Init(float sample_rate, float block_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        block_ = (int)lrintf(sample_rate / block_rate);
        if (block_ < 1) block_ = 1;
        random_.Seed(seed);
        attack_ = 1.f / (0.003f * sample_rate);
        decay_ = expf(-1.f / (0.12f * sample_rate));
        release_ = expf(-1.f / (0.02f * sample_rate));
        for (int k = 0; k < kMax; k++) {
            Keyer &key = keyer_[k];
            float side = random_.Uniform() < 0.5f ? -1.f : 1.f;
            key.position = side * (200.f + 1300.f * random_.Uniform());
            key.left = (int)(random_.Uniform() * 0.3f * sample_rate); // start apart
            key.elements = key.letters = 0;
        }
        Reset();
    }

    // Silent: the gains and envelopes at 0 (the keyers' places and timing stay)
    void Reset() {
        for (Keyer &key : keyer_) {
            key.rotor.Reset();
            key.envelope = key.gain = key.step = key.target = 0.f;
            key.ramp = 0;
            key.down = key.rising = false;
        }
    }

    static int Keyers(float amount) { return 1 + (int)lrintf(2.f * EffectRange(amount)); }
    float Position(int k) const { return keyer_[k].position; }

    void Update(float amount, float level, float dial_hz) {
        float u = EffectRange(amount);
        unit_ = (int)(1.2f / (12.f + 13.f * u) * sample_rate_);
        count_ = Keyers(amount);
        for (Keyer &key : keyer_) {
            float hz = key.position + dial_hz;
            key.rotor.SetHz(hz, sample_rate_);
            key.target = 0.125f * level * ZeroBeatTaper(hz);
            key.step = (key.target - key.gain) / (float)block_; // ramped over the next block
            key.ramp = block_;
        }
    }

    float Process() {
        float out = 0.f;
        for (int k = 0; k < kMax; k++) {
            Keyer &key = keyer_[k];
            if (--key.left <= 0) Advance(key, k < count_);
            if (key.ramp > 0) key.gain = --key.ramp > 0 ? key.gain + key.step : key.target;
            if (key.down) {
                if (key.rising) {
                    key.envelope += attack_;
                    if (key.envelope >= 1.f) {
                        key.envelope = 1.f;
                        key.rising = false;
                    }
                } else {
                    key.envelope *= decay_;
                }
            } else {
                key.envelope *= release_;
                if (key.envelope < 1e-5f) key.envelope = 0.f;
            }
            if (key.envelope > 0.f) out += key.gain * key.envelope * key.rotor.Next();
        }
        return out;
    }

  private:
    struct Keyer {
        float position = 0.f, envelope = 0.f;
        float gain = 0.f, step = 0.f, target = 0.f; // level x zero-beat taper, ramped per block
        Rotor rotor;
        bool down = false, rising = false;
        int left = 0, elements = 0, letters = 0; // samples left in this state; elements / letters still to send
        int ramp = 0;                            // samples left in the gain's ramp
    };

    // The keyer's next state: an element, or a gap between elements, letters or words
    void Advance(Keyer &key, bool active) {
        if (key.down) { // the element ends: a gap
            key.down = false;
            if (--key.elements > 0) {
                key.left = unit_;
            } else if (--key.letters > 0) {
                key.left = 3 * unit_;
            } else {
                key.left = 7 * unit_;
            }
            return;
        }
        if (!active) { // this keyer is silent at this amount: check again later
            key.left = 7 * unit_;
            return;
        }
        if (key.letters <= 0) key.letters = 2 + (int)(random_.Uniform() * 4.f);
        if (key.elements <= 0) key.elements = 1 + (int)(random_.Uniform() * 4.f);
        key.down = key.rising = true;
        key.left = (random_.Uniform() < 0.5f ? 1 : 3) * unit_;
    }

    float sample_rate_ = 48000.f, attack_ = 0.f, decay_ = 0.f, release_ = 0.f;
    int unit_ = 4800, count_ = 1, block_ = 48;
    Random random_;
    Keyer keyer_[kMax];
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/morse_chimes.h sim/test_engine.cpp
git commit -m "Engine: Morse chimes (keyed carriers with a bell-like envelope)"
```

---

### Task 5: Selcall

**Files:**
- Create: `engine/selcall.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "selcall.h"`, and above `main()`:
```cpp
static std::vector<float> RunSelcall(bitch::Selcall &s, bool on, float amount, float seconds,
                                     std::vector<float> *tones = nullptr) {
    std::vector<float> out;
    for (int b = 0; b < (int)(seconds * kSampleRate / 48); b++) {
        s.Update(on, amount);
        for (int k = 0; k < 48; k++) out.push_back(s.Process());
        if (tones) tones->push_back(s.ToneHz());
    }
    return out;
}

// Burst starts: a sound after at least 40 ms of silence
static int CountBursts(const std::vector<float> &x) {
    int bursts = 0, quiet = 100000;
    for (float v : x) {
        if (fabsf(v) > 1e-3f) {
            if (quiet > (int)(0.04f * kSampleRate)) bursts++;
            quiet = 0;
        } else {
            quiet++;
        }
    }
    return bursts;
}

static void TestSelcall() {
    const float kZvei[11] = {2400, 1060, 1160, 1270, 1400, 1530, 1670, 1830, 2000, 2200, 2600};
    const float kCcir[11] = {1981, 1124, 1197, 1275, 1358, 1446, 1540, 1640, 1747, 1860, 2110};
    bitch::Selcall s;
    s.Init(kSampleRate, 4u);
    // The first burst fires at the touch, with tones from one of the two sets
    std::vector<float> tones;
    auto x = RunSelcall(s, true, 1.f, 0.05f, &tones);
    CHECK(Rms(Window(x, 0.01f, 0.05f)) > 0.05f, "selcall: no burst at the touch (%g)", Rms(Window(x, 0.01f, 0.05f)));
    bool known = false;
    for (int k = 0; k < 11; k++) known = known || tones.back() == kZvei[k] || tones.back() == kCcir[k];
    CHECK(known, "selcall: tone %.0f Hz isn't ZVEI or CCIR", tones.back());
    CHECK(Magnitude(Window(x, 0.01f, 0.05f), tones.back()) > 0.05, "selcall: not at its tone");
    // Five tones per burst, and bursts every 0.4 s at amount 1 (later ones wait for the last)
    s.Init(kSampleRate, 4u);
    tones.clear();
    x = RunSelcall(s, true, 1.f, 2.f, &tones);
    CHECK(CountBursts(x) >= 3, "selcall at amount 1: %d bursts in 2 s", CountBursts(x));
    s.Init(kSampleRate, 4u);
    x = RunSelcall(s, true, 0.35f, 3.f);
    CHECK(CountBursts(x) == 2, "selcall at amount 0.35: %d bursts in 3 s (want 2)", CountBursts(x));
    CHECK(Peak(x) < 0.25f, "selcall: peak %.3f", Peak(x));
    // Released: no new bursts; Stop() drops a burst that was cut off
    s.Init(kSampleRate, 4u);
    RunSelcall(s, true, 1.f, 0.01f);
    x = RunSelcall(s, false, 1.f, 2.f);
    CHECK(CountBursts(x) <= 1 && Rms(Window(x, 1.f, 2.f)) == 0.f, "selcall released: still bursting");
    s.Init(kSampleRate, 4u);
    RunSelcall(s, true, 1.f, 0.01f);
    s.Stop();
    x = RunSelcall(s, false, 1.f, 0.2f);
    CHECK(Peak(x) == 0.f, "selcall: Stop() didn't drop the burst (%g)", Peak(x));
    // A digit repeating the one before is sent as the repeat tone, but not after a repeat tone
    const int kDigits[2][5] = {{3, 3, 3, 3, 5}, {1, 2, 2, 2, 2}}, kWant[2][5] = {{3, 10, 3, 10, 5}, {1, 2, 10, 2, 10}};
    for (int d = 0; d < 2; d++) {
        int symbols[5];
        bitch::Selcall::Symbols(kDigits[d], symbols);
        CHECK(std::equal(symbols, symbols + 5, kWant[d]), "selcall: digits %d%d%d%d%d sent as %d %d %d %d %d",
              kDigits[d][0], kDigits[d][1], kDigits[d][2], kDigits[d][3], kDigits[d][4], symbols[0], symbols[1],
              symbols[2], symbols[3], symbols[4]);
    }
    // The tones within a burst follow on without a gap (edges only at the burst's start and end)
    s.Init(kSampleRate, 4u);
    tones.clear();
    x = RunSelcall(s, true, 1.f, 0.3f, &tones);
    int change = 0;
    for (int b = 1; b < (int)tones.size() && !change; b++)
        if (tones[b] != tones[b - 1]) change = b;
    std::vector<float> join(x.begin() + change * 48 - 24, x.begin() + change * 48 + 72);
    CHECK(change > 0 && Peak(join) > 0.15f, "selcall: the join between tones dips to %.3f", Peak(join));
}
```
Call `TestSelcall();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: compile error, `'selcall.h' file not found`.

- [ ] **Step 3: Write the selcall**

`engine/selcall.h`:
```cpp
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "effects.h"
#include "random.h"
#include "rotor.h"

namespace bitch {

// Selcall: 5-tone selective-calling bursts, as heard on shortwave and utility bands. Each
// burst is 5 random digits in the ZVEI tone set (70 ms per tone) or the CCIR set (100 ms),
// chosen at random; a digit repeating the one before is sent as the set's repeat tone (unless
// the one before was itself the repeat tone: 3,3,3 is sent 3,R,3). The tones follow on
// without gaps, phase-continuous, at about -14 dBFS, with 4 ms edges at the burst's start and
// end. While on: the first burst at the touch, then one every 2 s (amount 0.35) down to every
// 0.4 s (amount 1), counted from each burst's start (a burst still playing finishes first,
// then 50 ms of quiet). Off: the current burst finishes, no new ones. Update() once per
// block, Process() per sample (mono).
class Selcall {
  public:
    void Init(float sample_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        random_.Seed(seed);
        edge_ = (int)(0.004f * sample_rate);
        tone_ = -1;
        was_on_ = false;
        next_ = 0;
        tone_hz_ = 0.f;
        rotor_.Reset();
    }

    void Update(bool on, float amount) {
        int interval = (int)(2.f * powf(0.2f, EffectRange(amount)) * sample_rate_);
        if (on && !was_on_) next_ = 0; // fire at the touch
        was_on_ = on;
        on_ = on;
        interval_ = interval;
        rotor_.Normalize();
    }

    float Process() {
        if (on_ && tone_ < 0) {
            if (--next_ <= 0) Start();
        } else if (on_) {
            next_--;
        }
        if (tone_ < 0) return 0.f;
        int burst = 5 * length_, at = tone_ * length_ + position_; // samples into the burst
        float edge = fminf((float)std::min(at, burst - at) / (float)edge_, 1.f);
        float out = 0.2f * edge * rotor_.Next();
        if (++position_ >= length_) {
            position_ = 0;
            if (++tone_ >= 5) {
                tone_ = -1;
                if (next_ < (int)(0.05f * sample_rate_)) next_ = (int)(0.05f * sample_rate_);
            } else {
                SetTone();
            }
        }
        return out;
    }

    float ToneHz() const { return tone_hz_; } // the current (or last) tone

    // Drops a burst that was cut off (the engine calls this once the pad's fade has ended, so
    // a half-played burst doesn't resume at the next touch)
    void Stop() { tone_ = -1; }

    // The symbols sent for 5 digits: the digit, or 10 (the repeat tone) for a digit repeating
    // the one before, unless that one was sent as the repeat tone
    static void Symbols(const int digits[5], int symbols[5]) {
        for (int k = 0; k < 5; k++)
            symbols[k] = k > 0 && digits[k] == digits[k - 1] && symbols[k - 1] != 10 ? 10 : digits[k];
    }

  private:
    void Start() {
        ccir_ = random_.Uniform() < 0.5f;
        int digits[5];
        for (int k = 0; k < 5; k++) digits[k] = (int)(random_.Uniform() * 10.f) % 10;
        Symbols(digits, symbols_);
        length_ = (int)((ccir_ ? 0.1f : 0.07f) * sample_rate_);
        tone_ = 0;
        position_ = 0;
        rotor_.Reset();
        SetTone();
        next_ = interval_;
    }

    void SetTone() {
        static const float kZvei[11] = {2400.f, 1060.f, 1160.f, 1270.f, 1400.f, 1530.f,
                                        1670.f, 1830.f, 2000.f, 2200.f, 2600.f};
        static const float kCcir[11] = {1981.f, 1124.f, 1197.f, 1275.f, 1358.f, 1446.f,
                                        1540.f, 1640.f, 1747.f, 1860.f, 2110.f};
        tone_hz_ = ccir_ ? kCcir[symbols_[tone_]] : kZvei[symbols_[tone_]];
        rotor_.SetHz(tone_hz_, sample_rate_); // keeps the phase: the tones join without a click
    }

    float sample_rate_ = 48000.f, tone_hz_ = 0.f;
    Random random_;
    Rotor rotor_;
    int symbols_[5] = {};
    bool ccir_ = false, on_ = false, was_on_ = false;
    int tone_ = -1, position_ = 0, length_ = 1, edge_ = 1, next_ = 0, interval_ = 1;
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/selcall.h sim/test_engine.cpp
git commit -m "Engine: selcall bursts (ZVEI and CCIR 5-tone)"
```

---

### Task 6: The jammer

**Files:**
- Create: `engine/jammer.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "jammer.h"`, and above `main()`:
```cpp
static void TestJammer() {
    bitch::Jammer j;
    j.Init(kSampleRate, 6u);
    std::vector<float> x, sweep;
    for (int b = 0; b < (int)(1.f * kSampleRate / 48); b++) {
        j.Update(1.f, 1.f, 48.f / kSampleRate);
        for (int k = 0; k < 48; k++) x.push_back(j.Process());
        sweep.push_back(j.ToneHz());
    }
    // A sawtooth sweep 300 Hz -> 3 kHz, 4 times a second at amount 1
    float lo = 1e9f, hi = 0.f;
    int resets = 0;
    for (size_t k = 1; k < sweep.size(); k++) {
        lo = std::min(lo, sweep[k]);
        hi = std::max(hi, sweep[k]);
        resets += sweep[k] < 0.5f * sweep[k - 1];
    }
    CHECK(lo >= 299.f && lo < 350.f && hi > 2700.f && hi <= 3001.f, "jammer sweep %.0f-%.0f Hz", lo, hi);
    CHECK(resets >= 3 && resets <= 5, "jammer: %d sweeps in 1 s at amount 1 (want ~4)", resets);
    float loud = Rms(x);
    CHECK(loud > 0.05f && Peak(x) <= 1.f, "jammer: rms %.3f, peak %.3f", loud, Peak(x));
    // Quieter and slower at amount 0.35
    bitch::Jammer q;
    q.Init(kSampleRate, 6u);
    std::vector<float> y;
    for (int b = 0; b < (int)(1.f * kSampleRate / 48); b++) {
        q.Update(0.35f, 1.f, 48.f / kSampleRate);
        for (int k = 0; k < 48; k++) y.push_back(q.Process());
    }
    CHECK(Rms(y) < 0.5f * loud, "jammer at 0.35: rms %.3f vs %.3f", Rms(y), loud);
}
```
Call `TestJammer();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: compile error, `'jammer.h' file not found`.

- [ ] **Step 3: Write the jammer**

`engine/jammer.h`:
```cpp
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "effects.h"
#include "random.h"
#include "softclip.h"

namespace bitch {

// A cheap sine of a phase 0-1 (a parabola, refined: within 0.1% of sinf)
inline float FastSine(float phase) {
    float t = phase < 0.5f ? phase : phase - 1.f; // -0.5..0.5
    float y = 8.f * t * (1.f - 2.f * fabsf(t));
    return y + 0.225f * (y * fabsf(y) - y);
}

// Jammer: a buzzy tone (an overdriven sine) swept from 300 Hz up to 3 kHz in a sawtooth
// (exponential sweep), 0.5 (amount 0.35) to 4 (amount 1) sweeps a second, plus bursts of
// noise (30-150 ms, 1 to 6 a second, with 1.5 ms edges). Level -24 dBFS (amount 0.35) to
// -10 dBFS (amount 1). Update() once per block (with the pad's fade level; the gain is
// ramped across the next block), Process() per sample (mono).
class Jammer {
  public:
    void Init(float sample_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        random_.Seed(seed);
        edge_ = (int)(0.0015f * sample_rate);
        sweep_ = phase_ = 0.f;
        tone_hz_ = 300.f;
        Reset();
    }

    // Silent: the gain at 0, no burst
    void Reset() {
        gain_ = step_ = target_ = 0.f;
        ramp_ = 0;
        burst_left_ = 0;
    }

    void Update(float amount, float level, float block_seconds) {
        float u = EffectRange(amount);
        sweep_step_ = (0.5f + 3.5f * u) / sample_rate_;
        ratio_ = powf(10.f, sweep_step_); // the tone's rise per sample (a decade per sweep)
        target_ = powf(10.f, 0.05f * (-24.f + 14.f * u)) * level;
        ramp_ = std::max((int)lrintf(block_seconds * sample_rate_), 1);
        step_ = (target_ - gain_) / (float)ramp_;
        if (burst_left_ <= 0 && random_.Uniform() < (1.f + 5.f * u) * block_seconds) {
            burst_length_ = (int)((0.03f + 0.12f * random_.Uniform()) * sample_rate_);
            burst_left_ = burst_length_;
        }
    }

    float Process() {
        if (ramp_ > 0) gain_ = --ramp_ > 0 ? gain_ + step_ : target_;
        sweep_ += sweep_step_;
        if (sweep_ >= 1.f) { // back to 300 Hz
            sweep_ -= 1.f;
            tone_hz_ = 300.f * powf(10.f, sweep_);
        } else {
            tone_hz_ = fminf(tone_hz_ * ratio_, 3000.f);
        }
        phase_ += tone_hz_ / sample_rate_;
        phase_ -= floorf(phase_);
        float out = SoftClip(3.f * FastSine(phase_));
        if (burst_left_ > 0) {
            int from_start = burst_length_ - burst_left_;
            float edge = fminf((float)std::min(from_start, burst_left_) / (float)edge_, 1.f);
            burst_left_--;
            out += edge * random_.Bipolar();
        }
        return gain_ * out;
    }

    float ToneHz() const { return tone_hz_; }

  private:
    float sample_rate_ = 48000.f, sweep_ = 0.f, sweep_step_ = 0.f, ratio_ = 1.f, phase_ = 0.f, tone_hz_ = 300.f;
    float gain_ = 0.f, step_ = 0.f, target_ = 0.f;
    int ramp_ = 0, burst_left_ = 0, burst_length_ = 1, edge_ = 1;
    Random random_;
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/jammer.h sim/test_engine.cpp
git commit -m "Engine: jammer (swept buzz and noise bursts)"
```

---

### Task 7: Carriers, Morse chimes, Selcall and the jammer in the engine

**Files:**
- Modify: `engine/engine.h`
- Modify: `engine/engine.cpp`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp`, add `bitch::Bit(bitch::kCarriers) | bitch::Bit(bitch::kSelcall) | bitch::Bit(bitch::kMorseChimes) | bitch::Bit(bitch::kJammer) |` to `kBuiltEffects`, and above `main()`:
```cpp
// Two one-pole low-passes at `hz`: the RMS of what's below it
static float LowPassRms(const std::vector<float> &x, float hz) {
    float k = 1.f - expf(-2.f * kPi * hz / kSampleRate), a = 0.f, b = 0.f;
    std::vector<float> y;
    for (float v : x) {
        a += k * (v - a);
        b += k * (a - b);
        y.push_back(b);
    }
    return Rms(y);
}

static double Mean(const std::vector<float> &x) {
    double sum = 0.0;
    for (float v : x) sum += v;
    return x.empty() ? 0.0 : sum / x.size();
}

// Tuned onto a station (zero beat) it fades out, instead of putting out DC and subsonic signal
static void TestRadioZeroBeat() {
    bitch::Carriers c;
    c.Init(kSampleRate, kSampleRate / 48.f, 3u);
    auto x = Window(RunCarriers(c, 0.35f, -c.Position(0), 2.f), 0.5f, 2.f); // heard at position + dial: 0 Hz
    CHECK(fabs(Mean(x)) < 1e-3 && LowPassRms(x, 20.f) < 1e-3f, "carriers at zero beat: mean %g, below 20 Hz %g",
          Mean(x), LowPassRms(x, 20.f));
    bitch::MorseChimes m;
    m.Init(kSampleRate, kSampleRate / 48.f, 9u);
    float dial = 5.f - m.Position(0); // 5 Hz: subsonic
    std::vector<float> y;
    for (int b = 0; b < (int)(4.f * kSampleRate / 48); b++) {
        m.Update(0.35f, 1.f, dial);
        for (int k = 0; k < 48; k++) y.push_back(m.Process());
    }
    CHECK(fabs(Mean(y)) < 1e-3 && LowPassRms(y, 20.f) < 1e-3f, "morse at 5 Hz from zero beat: mean %g, below 20 Hz %g",
          Mean(y), LowPassRms(y, 20.f));
}

// A radio pad toggled on and off (every 50 ms, over a 200 Hz sine) doesn't click: as in
// NoClickWorstRatio, the largest step in the 35 ms after each toggle against the steady 15 ms
// before and after it. Returns the worst ratio.
static float RadioNoClickWorstRatio(bitch::Effect pad) {
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.tune_hz = 50.f;
    p.effect_amount = 1.f;
    std::vector<float> l, r;
    int n = 0;
    const int kToggles = 40;
    const size_t segment = (size_t)(0.05f * kSampleRate), fade = (size_t)(0.035f * kSampleRate);
    for (int i = 0; i < kToggles; i++) {
        p.effects ^= bitch::Bit(pad);
        RunFor(e, p, 0.05f, [](int k) { return Sine(200.f, 0.3f, k); }, n, l, r);
    }
    auto max_step = [&](size_t from, size_t to) {
        float step = 0.f;
        for (size_t k = std::max<size_t>(from, 1); k < std::min(to, l.size()); k++)
            step = std::max(step, fabsf(l[k] - l[k - 1]));
        return step;
    };
    float worst = 0.f;
    for (int i = 1; i < kToggles; i++) {
        size_t at = i * segment;
        float during = max_step(at, at + fade);
        float steady = std::max(max_step(at - (segment - fade), at), max_step(at + fade, at + segment));
        if (steady > 0.f) worst = std::max(worst, during / steady);
    }
    return worst;
}

static void TestEffectsRadioNoClick() {
    const bitch::Effect kPads[5] = {bitch::kScrambler, bitch::kCarriers, bitch::kMorseChimes, bitch::kSelcall,
                                    bitch::kJammer};
    for (bitch::Effect pad : kPads) {
        float worst = RadioNoClickWorstRatio(pad);
        printf("  %s on/off: worst step %.2fx the steady ones\n", bitch::EffectName(pad), worst);
        CHECK(worst < 2.5f, "%s on/off: a step %.2fx the steady ones around it", bitch::EffectName(pad), worst);
    }
}

// Each radio pad makes sound from silence while held, and none without it
static void TestEffectsRadioSounds() {
    const bitch::Effect kPads[4] = {bitch::kCarriers, bitch::kSelcall, bitch::kMorseChimes, bitch::kJammer};
    for (bitch::Effect pad : kPads) {
        bitch::Engine e;
        e.Init(kSampleRate);
        bitch::Params p;
        p.effect_amount = 1.f;
        p.effects = bitch::Bit(pad);
        std::vector<float> l, r;
        int n = 0;
        RunFor(e, p, 2.f, [](int) { return 0.f; }, n, l, r);
        CHECK(Rms(Window(l, 0.f, 2.f)) > 0.01f, "%s: only %g from silence", bitch::EffectName(pad),
              Rms(Window(l, 0.f, 2.f)));
        p.effects = 0u;
        RunFor(e, p, 1.f, [](int) { return 0.f; }, n, l, r);
        CHECK(Peak(Window(l, 2.5f, 3.f)) < 1e-4f, "%s: still sounding after release (%g)", bitch::EffectName(pad),
              Peak(Window(l, 2.5f, 3.f)));
    }
}

// Selcall goes in before the shifter, so TUNE mistunes it like a receiver
static void TestEffectSelcallMistuned() {
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.tune_hz = 200.f;
    p.effects = bitch::Bit(bitch::kSelcall);
    std::vector<float> l, r;
    int n = 0;
    RunFor(e, p, 0.06f, [](int) { return 0.f; }, n, l, r);
    float tone = e.SelcallToneHz();
    auto w = Window(l, 0.02f, 0.06f);
    CHECK(Magnitude(w, tone + 200.f) > 5 * Magnitude(w, tone), "selcall + tune 200: %.0f Hz %.4f vs %.0f Hz %.4f",
          tone + 200.f, Magnitude(w, tone + 200.f), tone, Magnitude(w, tone));
}

// The carriers move with TUNE: the whistles change pitch as the dial moves
static void TestEffectCarriersFollowDial() {
    bitch::Engine a, b;
    a.Init(kSampleRate);
    b.Init(kSampleRate);
    bitch::Params p;
    p.effects = bitch::Bit(bitch::kCarriers);
    std::vector<float> al, ar, bl, br;
    int n = 0;
    RunFor(a, p, 1.f, [](int) { return 0.f; }, n, al, ar);
    p.tune_hz = 300.f;
    n = 0;
    RunFor(b, p, 1.f, [](int) { return 0.f; }, n, bl, br);
    auto wa = Window(al, 0.5f, 0.52f), wb = Window(bl, 0.5f, 0.52f); // 20 ms: the stations drift
    // Station positions are the engine's (seed 3, as in TestCarriers): the same stations, heard 300 Hz higher
    // (TUNE moves them the same way as the voice)
    bitch::Carriers c;
    c.Init(kSampleRate, kSampleRate / 48.f, 3u);
    float s0 = c.Position(0);
    CHECK(Magnitude(wa, fabsf(s0)) > 5 * Magnitude(wb, fabsf(s0)) &&
              Magnitude(wb, fabsf(s0 + 300.f)) > 5 * Magnitude(wa, fabsf(s0 + 300.f)),
          "carriers don't follow the dial (station %.0f Hz)", s0);
}
```
Call `TestEffectsRadioSounds();`, `TestEffectSelcallMistuned();`, `TestEffectCarriersFollowDial();`, `TestRadioZeroBeat();` and `TestEffectsRadioNoClick();` from `main()`. Also fix `NoClickWorstRatio`'s toggle pick, which read past the end of its 7-entry array: `kToggled[(seed >> 16) % (sizeof(kToggled) / sizeof(kToggled[0]))]`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: compile error, `no member named 'SelcallToneHz' in 'bitch::Engine'`.

- [ ] **Step 3: Wire them in**

`engine/engine.h`:
- Add includes after `scrambler.h`: `#include "rotor.h"`, `#include "carriers.h"`, `#include "morse_chimes.h"`, `#include "selcall.h"`, `#include "jammer.h"`.
- Add a public test hook next to `ShifterPhase`: `float SelcallToneHz() const { return selcall_.ToneHz(); }`.
- Add private members after the scrambler members:
```cpp
    Carriers carriers_[2];   // the same stations both sides, heard against each side's dial
    MorseChimes morse_[2];
    Selcall selcall_;        // mono, into both inputs
    Jammer jammer_;          // mono, into both sides' receivers
    // The scrambler's and Selcall's levels at the end of the last block: ramped across each block
    // from there (Carriers, Morse chimes and the jammer ramp their own gains)
    float scramble_level_ = 0.f, selcall_level_ = 0.f;
```

`engine/engine.cpp`:
- In `Init`, after the scrambler `Init`:
```cpp
    for (int c = 0; c < 2; c++) {
        carriers_[c].Init(sample_rate, block_rate, 3u); // same seed: the same stations both sides
        morse_[c].Init(sample_rate, block_rate, 9u);
    }
    selcall_.Init(sample_rate, 71u);
    jammer_.Init(sample_rate, 6u);
```
- In `Reset()`, after the scrambler's reset:
```cpp
    for (Carriers &c : carriers_) c.Reset();
    for (MorseChimes &m : morse_) m.Reset();
    selcall_.Stop();
    jammer_.Reset();
    scramble_level_ = selcall_level_ = 0.f;
```
- In `Process()`'s first-block setup, after `squelch_level_ = level_[kSquelch];`:
```cpp
        scramble_level_ = fmaxf(level_[kScrambler], level_[kHoppingScrambler]);
        selcall_level_ = level_[kSelcall];
```
- Replace the scrambler block (from Task 2) with this, which also runs the other radio pads:
```cpp
    // Radio pads: the scrambler (either pad), run only while it's audible (its level ramped
    // across the block). The hops go on through the Hopping pad's release, unless the plain pad
    // is held
    float amount_radio = Clamp(params.effect_amount, 0.35f, 1.f);
    const float scramble_from = scramble_level_, scramble_to = fmaxf(level_[kScrambler], level_[kHoppingScrambler]);
    scramble_level_ = scramble_to;
    const bool scrambling = scramble_from > 0.f || scramble_to > 0.f;
    const bool hopping = level_[kHoppingScrambler] > 0.f && !Has(fx, kScrambler);
    if (scrambling) {
        if (!scrambler_running_) {
            for (Scrambler &s : scrambler_) s.Reset();
            scrambler_running_ = true;
        }
        for (Scrambler &s : scrambler_) s.Update(amount_radio, hopping, (float)size / sample_rate_);
    } else {
        scrambler_running_ = false;
    }
    // The other radio pads, run only while audible: the stations are heard against each side's
    // dial (the shift as it is now, with the side's stereo gain and drift). Selcall's level is
    // ramped across the block here; the others ramp their own gains
    const float level_carriers = level_[kCarriers], level_morse = level_[kMorseChimes];
    const float selcall_from = selcall_level_, selcall_to = level_[kSelcall], level_jammer = level_[kJammer];
    selcall_level_ = selcall_to;
    const bool calling = selcall_from > 0.f || selcall_to > 0.f;
    for (int c = 0; c < 2; c++) {
        float side_dial = shift_hz_ * side_gain_[c] + drift_hz_[c];
        if (level_carriers > 0.f) carriers_[c].Update(amount_radio, level_carriers, side_dial);
        if (level_morse > 0.f) morse_[c].Update(amount_radio, level_morse, side_dial);
    }
    if (calling || Has(fx, kSelcall)) {
        selcall_.Update(Has(fx, kSelcall), amount_radio);
    } else {
        selcall_.Stop();
    }
    if (level_jammer > 0.f) jammer_.Update(amount_radio, level_jammer, (float)size / sample_rate_);
```
- In the per-sample loop, before `float y[2];`, add:
```cpp
        float call = calling ? (selcall_from + t * (selcall_to - selcall_from)) * selcall_.Process() : 0.f;
        float jam = level_jammer > 0.f ? jammer_.Process() : 0.f;
        const float scramble = scramble_from + t * (scramble_to - scramble_from);
```
- In the per-side loop, change `if (scramble > 0.f) driven += ...` to `if (scrambling) driven += scramble * (scrambler_[c].Process(driven) - driven);`, change `float dry = signal_gain * (x[c] + take_other_[c] * x[1 - c]);` to
```cpp
            float dry = signal_gain * (x[c] + take_other_[c] * x[1 - c]) + call;
```
  and after `float received = fading_[c].Process(shifted) + static_[c].Process();` add:
```cpp
            if (level_carriers > 0.f) received += carriers_[c].Process();
            if (level_morse > 0.f) received += morse_[c].Process();
            received += jam;
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`, including `TestEffectsUnbuiltDoNothing` (now only the time, motion and digital pads are unbuilt) and every earlier test (`call` and `jam` are exactly 0 with the pads off, and `x + 0.f` is exact; the ramps are constant 0 too). The review fixes (zero beat, scrambler band-limit, stations moving with TUNE, Rotors instead of per-sample trig, ramped levels, Selcall's repeat tone and joins) were committed separately as "Radio: review fixes (zero beat, scrambler band-limit, stations move with TUNE, cheaper oscillators)"; the code in Tasks 1 and 3-7 is the final version.

- [ ] **Step 5: Check the firmware builds, and commit**

Run: `make -C ~/local_files/daisy/bit-chifter` (touch `engine/engine.cpp` first) — expected: no errors or warnings; report the DTCMRAM/SRAM lines.
```bash
cd ~/local_files/daisy/bit-chifter
git add engine/engine.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: carriers, Morse chimes, selcall and jammer pads"
```

(Review checkpoint: the engine tasks are done; review before Task 8.)

---

### Task 8: Renders and listening

**Files:** none (renders only)

- [ ] **Step 1: Render**

```bash
cd ~/local_files/daisy/bit-chifter/sim/out
../render.sh voice.wav r4_scrambler.wav effects=scrambler every=2
../render.sh voice.wav r4_hopping.wav effects=hopping_scrambler amount=0.7 every=3
../render.sh voice.wav r4_carriers_sweep.wav tune=-600 sweep=600 effects=carriers amount=0.8
../render.sh voice.wav r4_selcall_sweep.wav tune=-250 sweep=250 filter=1500 q=1.5 open=0 effects=selcall amount=0.8
../render.sh drone.wav r4_selcall_detents.wav tune=-150 sweep=150 detent=0.7 inertia=0.06 effects=selcall amount=1
../render.sh voice.wav r4_morse_sweep.wav tune=-300 sweep=300 effects=morse_chimes amount=1 reverb=0.6
../render.sh voice.wav r4_jammer.wav effects=jammer amount=0.6 every=2
../render.sh voice.wav r4_radio_mix.wav tune=60 reception=0.5 filter=1200 q=2 open=0 effects=carriers,morse_chimes,selcall amount=0.7
```
Expected: eight `Wrote …` lines, no compiler warnings.

- [ ] **Step 2: User listens**

Send the eight renders, saying what each is: the scrambler toggling every 2 s; the hopping scrambler; the voice swept −600 → +600 Hz past hidden stations (whistles through zero); selcall bursts mistuned as the dial sweeps, through a narrow radio filter; selcall on the drone with the detents; Morse chimes sweeping with reverb; the jammer toggling; a busy band (carriers, chimes and selcall together, with reception). Nothing to commit (the render tool is unchanged) unless the user asks for changes.

---

### Task 9: Firmware

**Files:**
- Modify: `simple_touch.cpp`
- Modify: `README.md`

- [ ] **Step 1: Update the comment and README**

The pads already send the effect bits, so no code changes: in `simple_touch.cpp`'s top comment, add "scrambler, hopping scrambler, passing carriers, selcall, Morse chimes, jammer" to the "Built so far" list. In `README.md`'s Pads row, add: "page 2 big left scrambler, big right passing carriers, left edge hopping scrambler, right edge selcall, centre strip jammer, top centre Morse chimes".

- [ ] **Step 2: Build**

Run: `make -C ~/local_files/daisy/bit-chifter`
Expected: memory table, no errors or warnings.

- [ ] **Step 3: User flashes and plays it**

User runs `make program-dfu` (RESET, then BOOT while the LED pulses). Switch 1 centre (page 2). Try, with TUNE moving: hold the big left pad (scrambler: speech turned upside down) and the left edge (hopping); the big right pad (passing carriers: whistles that move as TUNE moves); the right edge (selcall bursts, mistuned by TUNE, snapping in tune with the detents); the narrow top centre (Morse chimes); the centre strip (jammer). RECEPTION makes them busier/stronger. Check the CPU in the USB report with several held at once.

- [ ] **Step 4: Commit (after the user confirms)**

```bash
cd ~/local_files/daisy/bit-chifter
git add simple_touch.cpp README.md
git commit -m "Simple Touch firmware: the radio pads (page 2)"
```
