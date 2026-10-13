# Bitchshifter Part 2 (Ring Mod, Radio Filter, Stereo) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the ring modulator (knob 1 RING, knob 2 CARRIER) and the radio filter (knob 3 FILTER, knob 4 WIDTH), make the engine true stereo with mono detection, and add the stereo modes (Linked / Mirror / Rotate) on the TOU logo pad with confirmation blips.

**Architecture:** Two new engine stages, each a small header-only class with Mac tests: `RingMod` (sine carrier, amount crossfade) before the shifter, and `RadioFilter` (two band-pass state-variable filters in series, with partial gain make-up and a fully-open bypass) after it. Knob-to-value mappings move into `engine/mapping.h` so the firmware, a future Patch front end and the tests share them. The engine becomes true stereo: each side has its own ring mod → shifter (± feedback) → radio filter → dry/wet → soft clip, with shared controls; if one input is silent the other feeds both sides. Stereo modes set each side's shift (and, for Rotate, swap the sides over an 8 s cycle). A small `Blips` generator gives audible confirmation, since the board has no screen.

**Tech Stack:** C++17 (the firmware compiles the engine as gnu++14, so static constexpr members need out-of-class definitions in engine.cpp), libDaisy v8.1, Apple clang for `sim/`.

**Spec:** `docs/superpowers/specs/2026-10-12-bitchshifter-design.md` (Controls → Knobs 1–4; Signal chain: the stereo paragraph and items 2 and 4).

**Conventions:**
- Project root `~/local_files/daisy/bitchshifter`, local git, master. No `Co-Authored-By` lines.
- Tests: `sim/run_tests.sh` (prints `All tests passed`). It compiles `sim/test_engine.cpp` + `engine/engine.cpp`; helpers already there: `CHECK(cond, fmt...)`, `kSampleRate` (48000.f), `kPi`, `Magnitude(vector, freq)` (Goertzel amplitude), `Db(ratio)`, `RunEngine(params, amplitude, &right)` (1 kHz sine into both inputs, returns the last 0.5 s of OUT L).
- Engine code: single precision only (`f` literals, `sinf`/`tanf`/`powf`...), nothing allocating in `Process()`.
- Hardware steps: the user flashes (RESET, BOOT while the LED pulses, `make program-dfu`) and confirms before the firmware commit.

## File structure

| File | Responsibility |
|---|---|
| `engine/ring_mod.h` | `RingMod`: sine carrier × signal, amount crossfade |
| `engine/radio_filter.h` | `Bandpass` (one Simper SVF stage) and `RadioFilter` (two stages, make-up gain, open/bypass) |
| `engine/mapping.h` | Knob/fader (0–1) → parameter mappings: `TuneHz`, `CarrierHz`, `FilterHz`, `FilterWidth` |
| `engine/blips.h` | `Blips`: N short confirmation beeps |
| `engine/engine.h`, `engine/engine.cpp` | New `Params` fields, stereo chain, mono detection, stereo modes, smoothing |
| `sim/test_engine.cpp` | New tests for each stage, the mappings and the chain |
| `sim/render.cpp` | `key=value` options for all parameters |
| `simple_touch.cpp` | Knobs 1–4 via `mapping.h`; TOU pad cycles the stereo mode, with blips |

---

### Task 1: Ring modulator

**Files:**
- Create: `engine/ring_mod.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "ring_mod.h"` with the other engine includes, and above `main()`:
```cpp
// Ring-modulates a 1 kHz sine (amplitude 0.25); returns the last 0.5 s
static std::vector<float> RingSine(float carrier_hz, float amount) {
    bitch::RingMod r;
    r.Init(kSampleRate);
    std::vector<float> out;
    int n = (int)kSampleRate;
    for (int t = 0; t < n; t++) {
        float x = 0.25f * sinf(2 * kPi * 1000.f * t / kSampleRate);
        float y = r.Process(x, carrier_hz, amount);
        if (t >= n / 2) out.push_back(y);
    }
    return out;
}

static void TestRingMod() {
    auto full = RingSine(100.f, 1.f);
    CHECK(fabs(Magnitude(full, 900.f) - 0.125) < 0.01, "ring 1: 900 Hz %.3f", Magnitude(full, 900.f));
    CHECK(fabs(Magnitude(full, 1100.f) - 0.125) < 0.01, "ring 1: 1100 Hz %.3f", Magnitude(full, 1100.f));
    CHECK(Magnitude(full, 1000.f) < 0.005, "ring 1: original left %.4f", Magnitude(full, 1000.f));
    auto none = RingSine(100.f, 0.f);
    CHECK(fabs(Magnitude(none, 1000.f) - 0.25) < 0.005, "ring 0: 1000 Hz %.3f", Magnitude(none, 1000.f));
    CHECK(Magnitude(none, 1100.f) < 0.005, "ring 0: sideband %.4f", Magnitude(none, 1100.f));
    auto half = RingSine(100.f, 0.5f);
    CHECK(fabs(Magnitude(half, 1000.f) - 0.125) < 0.01, "ring 0.5: 1000 Hz %.3f", Magnitude(half, 1000.f));
    CHECK(fabs(Magnitude(half, 1100.f) - 0.0625) < 0.01, "ring 0.5: 1100 Hz %.3f", Magnitude(half, 1100.f));
}
```
Call `TestRingMod();` from `main()` after the existing tests.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile error, `'ring_mod.h' file not found`.

- [ ] **Step 3: Write the ring modulator**

`engine/ring_mod.h`:
```cpp
#pragma once

#include <cmath>

namespace bitch {

// Ring modulator: multiplies the signal by a sine carrier, giving the sum and
// difference sidebands (metallic, bell-like, or tremolo at low carrier rates).
// amount 0 leaves the signal unchanged, 1 is fully ring-modulated.
class RingMod {
  public:
    void Init(float sample_rate) {
        inv_sample_rate_ = 1.f / sample_rate;
        Reset();
    }

    void Reset() { phase_ = 0.f; }

    float Process(float x, float carrier_hz, float amount) {
        phase_ += carrier_hz * inv_sample_rate_;
        phase_ -= floorf(phase_);
        float carrier = sinf(6.28318530718f * phase_);
        return x * (1.f - amount + amount * carrier);
    }

  private:
    float inv_sample_rate_ = 1.f / 48000.f;
    float phase_ = 0.f;
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed`. If a check fails, report the printed numbers; don't change tolerances.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/ring_mod.h sim/test_engine.cpp
git commit -m "Engine: ring modulator"
```

---

### Task 2: Radio filter

**Files:**
- Create: `engine/radio_filter.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "radio_filter.h"`, and above `main()`:
```cpp
// Filters a sine of the given frequency (amplitude 0.25); returns the last 0.5 s
static std::vector<float> FilterSine(float freq, float centre_hz, float q, float open) {
    bitch::RadioFilter f;
    f.Init(kSampleRate);
    f.Set(centre_hz, q, open);
    std::vector<float> out;
    int n = (int)kSampleRate;
    for (int t = 0; t < n; t++) {
        float y = f.Process(0.25f * sinf(2 * kPi * freq * t / kSampleRate));
        if (t >= n / 2) out.push_back(y);
    }
    return out;
}

// RMS of white noise (amplitude 0.25) through the filter
static double FilterNoiseRms(float centre_hz, float q) {
    bitch::RadioFilter f;
    f.Init(kSampleRate);
    f.Set(centre_hz, q, 0.f);
    uint32_t seed = 7;
    double sum = 0;
    int n = (int)kSampleRate;
    for (int t = 0; t < n; t++) {
        seed = seed * 1664525u + 1013904223u;
        float y = f.Process(((seed >> 8) / 8388608.f - 1.f) * 0.25f);
        if (t >= n / 2) sum += y * y;
    }
    return sqrt(sum / (n / 2));
}

static void TestRadioFilter() {
    // At the centre the level is the input times the make-up gain, (q / 0.7)^0.35
    double makeup4 = pow(4.0 / 0.7, 0.35);
    auto centre = FilterSine(1000.f, 1000.f, 4.f, 0.f);
    CHECK(fabs(Magnitude(centre, 1000.f) / (0.25 * makeup4) - 1.0) < 0.1, "filter: centre gain %.3f",
          Magnitude(centre, 1000.f) / 0.25);
    // Two octaves away, at least 20 dB below the centre
    auto high = FilterSine(4000.f, 1000.f, 4.f, 0.f);
    auto low = FilterSine(250.f, 1000.f, 4.f, 0.f);
    CHECK(Db(Magnitude(centre, 1000.f) / Magnitude(high, 4000.f)) > 20, "filter: 4 kHz only %.1f dB down",
          Db(Magnitude(centre, 1000.f) / Magnitude(high, 4000.f)));
    CHECK(Db(Magnitude(centre, 1000.f) / Magnitude(low, 250.f)) > 20, "filter: 250 Hz only %.1f dB down",
          Db(Magnitude(centre, 1000.f) / Magnitude(low, 250.f)));
    // Fully open = bypassed
    auto open = FilterSine(4000.f, 1000.f, 4.f, 1.f);
    CHECK(fabs(Magnitude(open, 4000.f) - 0.25) < 0.0025, "filter open: 4 kHz %.4f", Magnitude(open, 4000.f));
    // Narrowing doesn't drop the level of a broadband signal by more than 6 dB
    double narrow = FilterNoiseRms(1000.f, 12.f), wide = FilterNoiseRms(1000.f, 0.7f);
    CHECK(fabs(Db(narrow / wide)) < 6, "filter: Q 12 vs 0.7 noise level %.1f dB", Db(narrow / wide));
}

// Sweeping the centre every block at maximum resonance stays finite and bounded
static void TestRadioFilterSweepStable() {
    bitch::RadioFilter f;
    f.Init(kSampleRate);
    uint32_t seed = 3;
    float peak = 0.f;
    bool finite = true;
    for (int b = 0; b < 2000; b++) {
        float centre = 150.f * powf(5000.f / 150.f, (b % 200) / 199.f);
        f.Set(centre, 12.f, 0.f);
        for (int k = 0; k < 48; k++) {
            seed = seed * 1664525u + 1013904223u;
            float y = f.Process(((seed >> 8) / 8388608.f - 1.f) * 0.5f);
            finite = finite && std::isfinite(y);
            peak = std::max(peak, fabsf(y));
        }
    }
    CHECK(finite, "filter sweep: non-finite output");
    CHECK(peak < 8.f, "filter sweep: peak %.2f", peak);
}
```
Call `TestRadioFilter();` and `TestRadioFilterSweepStable();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile error, `'radio_filter.h' file not found`.

- [ ] **Step 3: Write the filter**

`engine/radio_filter.h`:
```cpp
#pragma once

#include <cmath>

namespace bitch {

// One band-pass stage: a trapezoidal state-variable filter (Andrew Simper's form),
// stable under fast parameter changes. The output is scaled to unity gain at the centre.
class Bandpass {
  public:
    void Reset() { ic1_ = ic2_ = 0.f; }

    // g = tan(pi * centre / sample_rate), k = 1 / Q
    void SetCoefficients(float g, float k) {
        k_ = k;
        a1_ = 1.f / (1.f + g * (g + k));
        a2_ = g * a1_;
        a3_ = g * a2_;
    }

    float Process(float x) {
        float v3 = x - ic2_;
        float v1 = a1_ * ic1_ + a2_ * v3;
        float v2 = ic2_ + a2_ * ic1_ + a3_ * v3;
        ic1_ = 2.f * v1 - ic1_;
        ic2_ = 2.f * v2 - ic2_;
        return k_ * v1;
    }

  private:
    float ic1_ = 0.f, ic2_ = 0.f;
    float a1_ = 0.f, a2_ = 0.f, a3_ = 0.f, k_ = 1.f;
};

// The receiver's narrow, boxy band: two band-pass stages in series, with partial
// make-up gain so narrowing it doesn't make it much quieter, and an "open" control
// that crossfades to the unfiltered signal (1 = fully bypassed).
class RadioFilter {
  public:
    void Init(float sample_rate) {
        sample_rate_ = sample_rate;
        Reset();
        Set(1000.f, 0.7f, 1.f);
    }

    void Reset() {
        stage_[0].Reset();
        stage_[1].Reset();
    }

    // centre_hz: 150-5000 Hz; q: 0.7 (wide) to 12 (narrow, whistly); open: 0-1
    void Set(float centre_hz, float q, float open) {
        float centre = fminf(fmaxf(centre_hz, 20.f), 0.45f * sample_rate_);
        float qq = fminf(fmaxf(q, 0.5f), 20.f);
        float g = tanf(3.14159265f * centre / sample_rate_);
        for (Bandpass &s : stage_) s.SetCoefficients(g, 1.f / qq);
        makeup_ = powf(qq / 0.7f, 0.35f);
        open_ = fminf(fmaxf(open, 0.f), 1.f);
    }

    float Process(float x) {
        float y = stage_[1].Process(stage_[0].Process(x)) * makeup_;
        return y + open_ * (x - y);
    }

  private:
    Bandpass stage_[2];
    float sample_rate_ = 48000.f;
    float makeup_ = 1.f;
    float open_ = 1.f;
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed`. If the noise-level or attenuation checks fail, report the printed numbers (the make-up exponent 0.35 is a design choice to revisit together, not a tolerance to tweak).

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/radio_filter.h sim/test_engine.cpp
git commit -m "Engine: radio filter (two SVF band-pass stages, make-up gain, open bypass)"
```

---

### Task 3: Control mappings

**Files:**
- Create: `engine/mapping.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "mapping.h"`, and above `main()`:
```cpp
static void TestMappings() {
    CHECK(bitch::TuneHz(0.5f) == 0.f, "tune centre %.2f", bitch::TuneHz(0.5f));
    CHECK(bitch::TuneHz(0.51f) == 0.f, "tune dead zone %.2f", bitch::TuneHz(0.51f));
    CHECK(fabsf(bitch::TuneHz(1.f) - 2500.f) < 0.5f, "tune top %.1f", bitch::TuneHz(1.f));
    CHECK(fabsf(bitch::TuneHz(0.f) + 2500.f) < 0.5f, "tune bottom %.1f", bitch::TuneHz(0.f));
    CHECK(bitch::TuneHz(0.6f) > 0.f && bitch::TuneHz(0.6f) < 30.f, "tune fine near centre %.2f", bitch::TuneHz(0.6f));
    CHECK(fabsf(bitch::CarrierHz(0.f) - 0.1f) < 0.001f, "carrier bottom %.3f", bitch::CarrierHz(0.f));
    CHECK(fabsf(bitch::CarrierHz(1.f) - 5000.f) < 1.f, "carrier top %.1f", bitch::CarrierHz(1.f));
    CHECK(fabsf(bitch::FilterHz(0.f) - 150.f) < 0.1f, "filter bottom %.1f", bitch::FilterHz(0.f));
    CHECK(fabsf(bitch::FilterHz(1.f) - 5000.f) < 1.f, "filter top %.1f", bitch::FilterHz(1.f));
    float q, open;
    bitch::FilterWidth(0.f, q, open);
    CHECK(fabsf(q - 12.f) < 0.01f && open == 0.f, "width 0: q %.2f open %.2f", q, open);
    bitch::FilterWidth(0.85f, q, open);
    CHECK(fabsf(q - 0.7f) < 0.01f && open == 0.f, "width 0.85: q %.2f open %.2f", q, open);
    bitch::FilterWidth(1.f, q, open);
    CHECK(fabsf(q - 0.7f) < 0.01f && open == 1.f, "width 1: q %.2f open %.2f", q, open);
}
```
Call `TestMappings();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile error, `'mapping.h' file not found`.

- [ ] **Step 3: Write the mappings**

`engine/mapping.h`:
```cpp
#pragma once

#include <cmath>

namespace bitch {

// Knob and fader positions (0-1) to engine parameters. Shared by the Simple Touch
// front end, a future Patch front end and the tests.

// Fader -> shift: -2500..+2500 Hz, 0 in a small dead zone at the centre, cubic so
// it's fine near 0 and wild at the ends
inline float TuneHz(float fader) {
    const float kMaxShiftHz = 2500.f, kDeadZone = 0.02f;
    float x = 2.f * fader - 1.f;
    if (fabsf(x) < kDeadZone) return 0.f;
    x = (x - copysignf(kDeadZone, x)) / (1.f - kDeadZone);
    return kMaxShiftHz * x * x * x;
}

// Knob -> ring-mod carrier, 0.1 Hz (tremolo) to 5 kHz (bells), exponential
inline float CarrierHz(float knob) { return 0.1f * powf(50000.f, knob); }

// Knob -> radio filter centre, 150 Hz to 5 kHz, exponential
inline float FilterHz(float knob) { return 150.f * powf(5000.f / 150.f, knob); }

// Knob -> radio filter width: narrow and resonant (Q 12) to wide (Q 0.7) over the
// first 85% of the turn, then crossfading to fully open (bypassed) at the end
inline void FilterWidth(float knob, float &q, float &open) {
    float narrowing = fminf(knob / 0.85f, 1.f);
    q = 12.f * powf(0.7f / 12.f, narrowing);
    open = fminf(fmaxf((knob - 0.85f) / 0.15f, 0.f), 1.f);
}

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/mapping.h sim/test_engine.cpp
git commit -m "Engine: shared knob and fader mappings"
```

---

### Task 4: Stereo engine chain with mono detection

**Files:**
- Modify: `engine/engine.h`
- Modify: `engine/engine.cpp`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp`, add this helper above `main()`:
```cpp
// Runs the engine for `seconds` on independent sines into each input (amplitude 0 =
// silent input); fills the full OUT L / OUT R
static void RunEngineStereo(const bitch::Params &p, float seconds, float freq_l, float amp_l, float freq_r,
                            float amp_r, std::vector<float> &out_l, std::vector<float> &out_r) {
    bitch::Engine e;
    e.Init(kSampleRate);
    const size_t block = 48;
    std::vector<float> l(block), r(block), ol(block), orr(block);
    out_l.clear();
    out_r.clear();
    int blocks = (int)(seconds * kSampleRate) / block;
    for (int b = 0; b < blocks; b++) {
        for (size_t k = 0; k < block; k++) {
            float t = (b * block + k) / kSampleRate;
            l[k] = amp_l * sinf(2 * kPi * freq_l * t);
            r[k] = amp_r * sinf(2 * kPi * freq_r * t);
        }
        e.Process(l.data(), r.data(), ol.data(), orr.data(), block, p);
        out_l.insert(out_l.end(), ol.begin(), ol.end());
        out_r.insert(out_r.end(), orr.begin(), orr.end());
    }
}

// Samples [from_s, to_s) of a rendered output
static std::vector<float> Window(const std::vector<float> &x, float from_s, float to_s) {
    return std::vector<float>(x.begin() + (size_t)(from_s * kSampleRate), x.begin() + (size_t)(to_s * kSampleRate));
}
```
**Delete** the existing `TestEngineMonoSum()` function and its call in `main()` (it expected the old summed-to-mono half level), and add:
```cpp
// A mono source in one input comes out of both sides at full level
static void TestEngineMonoInput() {
    bitch::Params p;
    p.tune_hz = 100.f;
    std::vector<float> l, r;
    RunEngineStereo(p, 1.f, 1000.f, 0.25f, 0.f, 0.f, l, r); // IN R silent
    auto wl = Window(l, 0.5f, 1.f), wr = Window(r, 0.5f, 1.f);
    CHECK(fabs(Magnitude(wl, 1100.f) - 0.245) < 0.02, "mono L-only: OUT L 1100 Hz %.3f", Magnitude(wl, 1100.f));
    CHECK(fabs(Magnitude(wr, 1100.f) - 0.245) < 0.02, "mono L-only: OUT R 1100 Hz %.3f", Magnitude(wr, 1100.f));
    RunEngineStereo(p, 1.f, 0.f, 0.f, 1000.f, 0.25f, l, r); // IN L silent
    wl = Window(l, 0.5f, 1.f);
    CHECK(fabs(Magnitude(wl, 1100.f) - 0.245) < 0.02, "mono R-only: OUT L 1100 Hz %.3f", Magnitude(wl, 1100.f));
}

// A real stereo source keeps its sides apart
static void TestEngineStereoKeepsSides() {
    bitch::Params p;
    p.tune_hz = 100.f;
    std::vector<float> l, r;
    RunEngineStereo(p, 1.f, 1000.f, 0.25f, 1500.f, 0.25f, l, r);
    auto wl = Window(l, 0.5f, 1.f), wr = Window(r, 0.5f, 1.f);
    CHECK(Magnitude(wl, 1100.f) > 0.2, "stereo: OUT L 1100 Hz %.3f", Magnitude(wl, 1100.f));
    CHECK(Magnitude(wl, 1600.f) < 0.01, "stereo: R leaking into L %.3f", Magnitude(wl, 1600.f));
    CHECK(Magnitude(wr, 1600.f) > 0.2, "stereo: OUT R 1600 Hz %.3f", Magnitude(wr, 1600.f));
    CHECK(Magnitude(wr, 1100.f) < 0.01, "stereo: L leaking into R %.3f", Magnitude(wr, 1100.f));
}

// Ring then shift: 1 kHz ring-modulated by 100 Hz gives 900 + 1100 Hz, then +50 Hz
// shift gives 950 + 1150 Hz (proves the ring mod comes before the shifter)
static void TestEngineRingBeforeShift() {
    bitch::Params p;
    p.tune_hz = 50.f;
    p.ring = 1.f;
    p.carrier_hz = 100.f;
    auto out = RunEngine(p, 0.25f);
    CHECK(Magnitude(out, 950.f) > 0.1, "ring+shift: 950 Hz %.3f", Magnitude(out, 950.f));
    CHECK(Magnitude(out, 1150.f) > 0.1, "ring+shift: 1150 Hz %.3f", Magnitude(out, 1150.f));
    CHECK(Magnitude(out, 1000.f) < 0.01, "ring+shift: 1000 Hz left %.3f", Magnitude(out, 1000.f));
    CHECK(Magnitude(out, 900.f) < 0.01, "ring+shift: unshifted 900 Hz %.3f", Magnitude(out, 900.f));
}

// The radio filter is in the chain: a 1 kHz input through a 4 kHz-centred narrow
// filter is much quieter than through a 1 kHz-centred one
static void TestEngineFilter() {
    bitch::Params p;
    p.filter_q = 4.f;
    p.filter_open = 0.f;
    p.filter_hz = 1000.f;
    auto on = RunEngine(p, 0.25f);
    p.filter_hz = 4000.f;
    auto off = RunEngine(p, 0.25f);
    CHECK(Db(Magnitude(on, 1000.f) / Magnitude(off, 1000.f)) > 20, "engine filter: only %.1f dB",
          Db(Magnitude(on, 1000.f) / Magnitude(off, 1000.f)));
}

// The defaults (ring 0, filter open) leave part 1's shift unchanged
static void TestEngineDefaultsTransparent() {
    bitch::Params p;
    p.tune_hz = 100.f;
    auto out = RunEngine(p, 0.25f);
    CHECK(Magnitude(out, 1100.f) > 0.2, "defaults: 1100 Hz %.3f", Magnitude(out, 1100.f));
}
```
Strengthen `TestEngineNaNTune()`: after its loop, add
```cpp
    float last_peak = 0.f;
    for (int k = 0; k < 48; k++) last_peak = std::max(last_peak, fabsf(out_l[k]));
    CHECK(last_peak > 0.01f, "NaN tune: output silent (peak %.4f), clamp not working", last_peak);
```
Call `TestEngineMonoInput`, `TestEngineStereoKeepsSides`, `TestEngineRingBeforeShift`, `TestEngineFilter` and `TestEngineDefaultsTransparent` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile errors, `no member named 'ring'` (and `carrier_hz`, `filter_hz`, `filter_q`, `filter_open`) in `bitch::Params`.

- [ ] **Step 3: Extend the engine**

`engine/engine.h`:
```cpp
#pragma once

#include <cstddef>

#include "radio_filter.h"
#include "ring_mod.h"
#include "shifter.h"

namespace bitch {

// Everything the front end controls (physical values; see mapping.h for knobs).
struct Params {
    float tune_hz = 0.f;      // shift, -2500 to +2500 Hz
    float feedback = 0.f;     // 0 to 0.95
    float mix = 1.f;          // dry/wet, 0 to 1
    float ring = 0.f;         // ring-mod amount, 0 to 1
    float carrier_hz = 100.f; // ring-mod carrier, 0.1 to 5000 Hz
    float filter_hz = 1000.f; // radio filter centre, 150 to 5000 Hz
    float filter_q = 0.7f;    // 0.7 (wide) to 12 (narrow)
    float filter_open = 1.f;  // 1 = filter bypassed
};

// The sound engine: no hardware code, so it runs on the Daisy and on the Mac.
// True stereo: each side has its own ring mod -> shifter (with feedback) -> radio
// filter -> dry/wet -> soft clip, with shared controls. If one input is silent, the
// other feeds both sides at full level.
class Engine {
  public:
    void Init(float sample_rate);
    void Process(const float *in_l, const float *in_r, float *out_l, float *out_r, size_t size,
                 const Params &params);

  private:
    void Reset();

    RingMod ring_[2];
    Shifter shifter_[2];
    RadioFilter filter_[2];
    float shift_hz_ = 0.f;     // glided towards params.tune_hz
    float carrier_hz_ = 100.f; // glided towards params.carrier_hz
    float glide_ = 0.f;        // per-sample one-pole coefficient, ~20 ms
    float log2_filter_hz_ = 0.f, filter_q_ = 0.7f, filter_open_ = 1.f; // smoothed per block
    float block_glide_ = 0.f;  // per-block one-pole coefficient, ~30 ms
    float sample_rate_ = 48000.f;
    // Mono detection: input envelopes, and how much of the other side each chain takes
    float envelope_[2] = {0.f, 0.f};
    float take_other_[2] = {0.f, 0.f};
    float envelope_release_ = 0.f; // per sample, ~200 ms
    float take_glide_ = 0.f;       // per sample, ~50 ms
};

} // namespace bitch
```

`engine/engine.cpp`:
```cpp
#include "engine.h"

#include <cmath>

#include "softclip.h"

namespace bitch {

// Out-of-class definitions: the firmware builds as C++14, where odr-used static constexpr
// members need one (redundant but harmless in C++17)
constexpr float Hilbert::kA[4];
constexpr float Hilbert::kB[4];

static const float kSilent = 0.001f; // -60 dBFS: below this an input counts as unused

static float Clamp(float x, float lo, float hi) { return fminf(fmaxf(x, lo), hi); }

void Engine::Init(float sample_rate) {
    sample_rate_ = sample_rate;
    for (RingMod &r : ring_) r.Init(sample_rate);
    for (Shifter &s : shifter_) s.Init(sample_rate);
    for (RadioFilter &f : filter_) f.Init(sample_rate);
    glide_ = 1.f - expf(-1.f / (0.02f * sample_rate));
    envelope_release_ = expf(-1.f / (0.2f * sample_rate));
    take_glide_ = 1.f - expf(-1.f / (0.05f * sample_rate));
    Reset();
}

void Engine::Reset() {
    for (RingMod &r : ring_) r.Reset();
    for (Shifter &s : shifter_) s.Reset();
    for (RadioFilter &f : filter_) f.Reset();
    shift_hz_ = 0.f;
    carrier_hz_ = 100.f;
    log2_filter_hz_ = log2f(1000.f);
    filter_q_ = 0.7f;
    filter_open_ = 1.f;
    block_glide_ = 0.f; // set on the first block (depends on the block size)
    envelope_[0] = envelope_[1] = 0.f;
    take_other_[0] = take_other_[1] = 0.f;
}

void Engine::Process(const float *in_l, const float *in_r, float *out_l, float *out_r, size_t size,
                     const Params &params) {
    if (size == 0) return;
    float tune = Clamp(params.tune_hz, -2500.f, 2500.f);
    float feedback = Clamp(params.feedback, 0.f, 0.95f);
    float mix = Clamp(params.mix, 0.f, 1.f);
    float ring = Clamp(params.ring, 0.f, 1.f);
    float carrier = Clamp(params.carrier_hz, 0.1f, 5000.f);

    // Filter settings change once per block, smoothed (the centre in octaves)
    if (block_glide_ == 0.f) {
        block_glide_ = 1.f - expf(-(float)size / (0.03f * sample_rate_));
        log2_filter_hz_ = log2f(Clamp(params.filter_hz, 150.f, 5000.f));
        filter_q_ = Clamp(params.filter_q, 0.7f, 12.f);
        filter_open_ = Clamp(params.filter_open, 0.f, 1.f);
    }
    log2_filter_hz_ += block_glide_ * (log2f(Clamp(params.filter_hz, 150.f, 5000.f)) - log2_filter_hz_);
    filter_q_ += block_glide_ * (Clamp(params.filter_q, 0.7f, 12.f) - filter_q_);
    filter_open_ += block_glide_ * (Clamp(params.filter_open, 0.f, 1.f) - filter_open_);
    float filter_hz = exp2f(log2_filter_hz_);
    for (RadioFilter &f : filter_) f.Set(filter_hz, filter_q_, filter_open_);

    // Mono detection: a side whose input is silent takes the other side's input instead
    bool active_l = envelope_[0] > kSilent, active_r = envelope_[1] > kSilent;
    float take_target[2] = {active_r && !active_l ? 1.f : 0.f, active_l && !active_r ? 1.f : 0.f};

    for (size_t i = 0; i < size; i++) {
        float x[2] = {in_l[i], in_r[i]};
        for (int c = 0; c < 2; c++) envelope_[c] = fmaxf(fabsf(x[c]), envelope_[c] * envelope_release_);
        shift_hz_ += glide_ * (tune - shift_hz_);
        carrier_hz_ += glide_ * (carrier - carrier_hz_);
        float y[2];
        for (int c = 0; c < 2; c++) {
            take_other_[c] += take_glide_ * (take_target[c] - take_other_[c]);
            float dry = x[c] + take_other_[c] * x[1 - c];
            float ringed = ring_[c].Process(dry, carrier_hz_, ring);
            float wet = filter_[c].Process(shifter_[c].Process(ringed, shift_hz_, feedback));
            y[c] = SoftClip(dry + mix * (wet - dry));
        }
        out_l[i] = y[0];
        out_r[i] = y[1];
    }
    // A NaN/Inf would stay in the feedback loop and silence everything: reset instead
    if (!std::isfinite(out_l[size - 1]) || !std::isfinite(out_r[size - 1])) {
        Reset();
        for (size_t i = 0; i < size; i++) out_l[i] = out_r[i] = 0.f;
    }
}

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed` (all earlier tests too: identical inputs on both sides still give identical outputs, and the defaults keep part 1's shift). If a check fails, report the numbers; don't change tolerances.

- [ ] **Step 5: Check the firmware still builds**

Run: `make -C ~/local_files/daisy/bitchshifter`
Expected: memory table, no errors or warnings.

- [ ] **Step 6: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/engine.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: true stereo with mono detection; ring mod before the shifter, radio filter after"
```

---

### Task 5: Stereo modes (Linked, Mirror, Rotate)

**Files:**
- Modify: `engine/engine.h`
- Modify: `engine/engine.cpp`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp`, above `main()`:
```cpp
// Mirror: left shifted up, right shifted down
static void TestEngineMirror() {
    bitch::Params p;
    p.tune_hz = 100.f;
    p.stereo_mode = bitch::StereoMode::kMirror;
    std::vector<float> l, r;
    RunEngineStereo(p, 1.f, 1000.f, 0.25f, 1000.f, 0.25f, l, r);
    auto wl = Window(l, 0.5f, 1.f), wr = Window(r, 0.5f, 1.f);
    CHECK(Magnitude(wl, 1100.f) > 0.2, "mirror: OUT L 1100 Hz %.3f", Magnitude(wl, 1100.f));
    CHECK(Magnitude(wr, 900.f) > 0.2, "mirror: OUT R 900 Hz %.3f", Magnitude(wr, 900.f));
    CHECK(Magnitude(wr, 1100.f) < 0.01, "mirror: OUT R 1100 Hz %.3f", Magnitude(wr, 1100.f));
}

// Rotate, 8 s cycle. With no shift, only the swapping shows: a quarter of the way
// round (2 s) the sides are swapped, three quarters round (6 s) they're back
static void TestEngineRotateSwapsSides() {
    bitch::Params p;
    p.tune_hz = 0.f;
    p.stereo_mode = bitch::StereoMode::kRotate;
    std::vector<float> l, r;
    RunEngineStereo(p, 7.f, 1000.f, 0.25f, 1500.f, 0.25f, l, r);
    auto at2 = Window(l, 1.9f, 2.1f), at6 = Window(l, 5.9f, 6.1f);
    CHECK(Magnitude(at2, 1500.f) > 0.15, "rotate 2 s: OUT L has R's 1500 Hz %.3f", Magnitude(at2, 1500.f));
    CHECK(Magnitude(at2, 1000.f) < 0.05, "rotate 2 s: OUT L still L's 1000 Hz %.3f", Magnitude(at2, 1000.f));
    CHECK(Magnitude(at6, 1000.f) > 0.15, "rotate 6 s: OUT L back to 1000 Hz %.3f", Magnitude(at6, 1000.f));
    CHECK(Magnitude(at6, 1500.f) < 0.05, "rotate 6 s: OUT L still has 1500 Hz %.3f", Magnitude(at6, 1500.f));
}

// Rotate at the start of the cycle: the sides' shifts are opposite (+100 / -100 Hz),
// and the pair sits in the middle, so both outputs carry both
static void TestEngineRotateShifts() {
    bitch::Params p;
    p.tune_hz = 100.f;
    p.stereo_mode = bitch::StereoMode::kRotate;
    std::vector<float> l, r;
    RunEngineStereo(p, 0.3f, 1000.f, 0.25f, 1000.f, 0.25f, l, r);
    auto w = Window(l, 0.05f, 0.25f);
    CHECK(Magnitude(w, 1100.f) > 0.06, "rotate start: 1100 Hz %.3f", Magnitude(w, 1100.f));
    CHECK(Magnitude(w, 900.f) > 0.06, "rotate start: 900 Hz %.3f", Magnitude(w, 900.f));
}
```
Call the three from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile error, `no member named 'StereoMode' in namespace 'bitch'`.

- [ ] **Step 3: Add the modes**

In `engine/engine.h`, above `struct Params`:
```cpp
// How the two sides' shifts relate (cycled with the TOU logo pad)
enum class StereoMode { kLinked, kMirror, kRotate };
```
add to `Params`:
```cpp
    StereoMode stereo_mode = StereoMode::kLinked;
```
and to `Engine`'s private members:
```cpp
    float rotate_phase_ = 0.f; // 0-1 around the Rotate cycle
    float rotate_step_ = 0.f;  // per sample, one cycle per 8 s
```
In `engine/engine.cpp`:
- In `Init`, after `take_glide_`: `rotate_step_ = 1.f / (8.f * sample_rate);`
- In `Reset`: `rotate_phase_ = 0.f;`
- In `Process`, replace the per-side loop body so each side gets its own shift and Rotate swaps the processed sides:
```cpp
        float shift[2] = {shift_hz_, shift_hz_};
        float swap = 0.f; // 0 = sides as they are, 1 = fully swapped
        if (params.stereo_mode == StereoMode::kMirror) {
            shift[1] = -shift_hz_;
        } else if (params.stereo_mode == StereoMode::kRotate) {
            rotate_phase_ += rotate_step_;
            if (rotate_phase_ >= 1.f) rotate_phase_ -= 1.f;
            float angle = 6.28318530718f * rotate_phase_;
            float sweep = cosf(angle); // left's shift goes +shift .. -shift, right's the other way
            shift[0] = shift_hz_ * sweep;
            shift[1] = -shift_hz_ * sweep;
            swap = 0.5f + 0.5f * sinf(angle); // the pair circles from left to right
        }
        float y[2];
        for (int c = 0; c < 2; c++) {
            take_other_[c] += take_glide_ * (take_target[c] - take_other_[c]);
            float dry = x[c] + take_other_[c] * x[1 - c];
            float ringed = ring_[c].Process(dry, carrier_hz_, ring);
            float wet = filter_[c].Process(shifter_[c].Process(ringed, shift[c], feedback));
            y[c] = dry + mix * (wet - dry);
        }
        out_l[i] = SoftClip(y[0] + swap * (y[1] - y[0]));
        out_r[i] = SoftClip(y[1] + swap * (y[0] - y[1]));
```
(The `shift_hz_` and `carrier_hz_` glide lines stay above this, and the `envelope_` update stays first.) Also extend the class comment in engine.h with: "Stereo modes: Linked (same shift both sides), Mirror (right gets the opposite shift), Rotate (over 8 s the sides' shifts sweep through each other and the pair circles left to right)."

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/engine.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: stereo modes (Linked, Mirror, Rotate)"
```

---

### Task 6: Confirmation blips

**Files:**
- Create: `engine/blips.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing test**

In `sim/test_engine.cpp` add `#include "blips.h"`, and above `main()`:
```cpp
// Start(n) plays n separate beeps, then silence
static void TestBlips() {
    for (int n = 1; n <= 3; n++) {
        bitch::Blips b;
        b.Init(kSampleRate);
        b.Start(n);
        // A burst = sound after at least 200 silent samples (the start counts as silence)
        int bursts = 0, silent_run = 1000;
        for (int t = 0; t < (int)kSampleRate; t++) {
            float v = fabsf(b.Process());
            if (v > 0.01f) {
                if (silent_run >= 200) bursts++;
                silent_run = 0;
            } else {
                silent_run++;
            }
        }
        CHECK(bursts == n, "blips: Start(%d) gave %d bursts", n, bursts);
        CHECK(fabsf(b.Process()) < 1e-6f, "blips: still sounding after 1 s");
    }
}
```
Call `TestBlips();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile error, `'blips.h' file not found`.

- [ ] **Step 3: Write the blips**

`engine/blips.h`:
```cpp
#pragma once

#include <cmath>

namespace bitch {

// Short confirmation beeps (the board has no screen): Start(n) plays n beeps of a
// 1.5 kHz tone, 60 ms each with 90 ms gaps, at about -14 dBFS. Process() returns the
// next sample to add to both outputs.
class Blips {
  public:
    void Init(float sample_rate) {
        on_samples_ = (int)(0.06f * sample_rate);
        period_samples_ = (int)(0.15f * sample_rate);
        step_ = 1500.f / sample_rate;
        remaining_ = 0;
    }

    void Start(int count) {
        remaining_ = count;
        position_ = 0;
        phase_ = 0.f;
    }

    float Process() {
        if (remaining_ <= 0) return 0.f;
        float out = 0.f;
        if (position_ < on_samples_) {
            phase_ += step_;
            if (phase_ >= 1.f) phase_ -= 1.f;
            // 5 ms fade in and out so the beeps don't click
            float edge = fminf((float)position_, (float)(on_samples_ - position_)) / (0.005f * on_samples_ / 0.06f);
            out = 0.2f * fminf(edge, 1.f) * sinf(6.28318530718f * phase_);
        }
        if (++position_ >= period_samples_) {
            position_ = 0;
            remaining_--;
        }
        return out;
    }

  private:
    int on_samples_ = 0, period_samples_ = 0, position_ = 0, remaining_ = 0;
    float step_ = 0.f, phase_ = 0.f;
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/blips.h sim/test_engine.cpp
git commit -m "Engine: confirmation blips"
```

---

### Task 7: Render options, and listening

**Files:**
- Modify: `sim/render.cpp`
- Modify: `sim/render.sh`

- [ ] **Step 1: Replace the argument handling with key=value options**

`sim/render.cpp`:
```cpp
// Renders a 16-bit WAV through the Bitchshifter engine.
// Usage: render in.wav out.wav [tune=Hz] [fb=0-0.95] [mix=0-1] [ring=0-1] [carrier=Hz]
//                              [filter=Hz] [q=0.7-12] [open=0-1] [stereo=linked|mirror|rotate]
// (defaults are the engine's: no shift, no ring mod, filter open, linked)
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "engine.h"
#include "wav.h"

static bool SetOption(bitch::Params &p, const char *arg) {
    const char *eq = strchr(arg, '=');
    if (!eq) return false;
    size_t n = (size_t)(eq - arg);
    const char *value = eq + 1;
    if (n == 6 && strncmp(arg, "stereo", 6) == 0) {
        if (!strcmp(value, "linked")) p.stereo_mode = bitch::StereoMode::kLinked;
        else if (!strcmp(value, "mirror")) p.stereo_mode = bitch::StereoMode::kMirror;
        else if (!strcmp(value, "rotate")) p.stereo_mode = bitch::StereoMode::kRotate;
        else return false;
        return true;
    }
    struct {
        const char *name;
        float *field;
    } fields[] = {{"tune", &p.tune_hz},     {"fb", &p.feedback},        {"mix", &p.mix},
                  {"ring", &p.ring},        {"carrier", &p.carrier_hz}, {"filter", &p.filter_hz},
                  {"q", &p.filter_q},       {"open", &p.filter_open}};
    for (auto &f : fields) {
        if (strlen(f.name) == n && strncmp(arg, f.name, n) == 0) {
            *f.field = (float)atof(value);
            return true;
        }
    }
    return false;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr,
                "Usage: %s in.wav out.wav [tune=] [fb=] [mix=] [ring=] [carrier=] [filter=] [q=] [open=] "
                "[stereo=linked|mirror|rotate]\n",
                argv[0]);
        return 1;
    }
    bitch::Params p;
    for (int i = 3; i < argc; i++) {
        if (!SetOption(p, argv[i])) {
            fprintf(stderr, "Unknown option %s\n", argv[i]);
            return 1;
        }
    }
    Wav in;
    if (!ReadWav(argv[1], in)) {
        fprintf(stderr, "Couldn't read %s (needs 16-bit PCM WAV, mono or stereo)\n", argv[1]);
        return 1;
    }
    bitch::Engine engine;
    engine.Init((float)in.sample_rate);
    Wav out;
    out.sample_rate = in.sample_rate;
    out.left.resize(in.left.size());
    out.right.resize(in.left.size());
    const size_t block = 48;
    for (size_t i = 0; i + block <= in.left.size(); i += block)
        engine.Process(&in.left[i], &in.right[i], &out.left[i], &out.right[i], block, p);
    if (!WriteWav(argv[2], out)) {
        fprintf(stderr, "Couldn't write %s\n", argv[2]);
        return 1;
    }
    static const char *modes[3] = {"linked", "mirror", "rotate"};
    printf("Wrote %s (tune %g fb %g mix %g ring %g carrier %g filter %g q %g open %g stereo %s)\n", argv[2],
           p.tune_hz, p.feedback, p.mix, p.ring, p.carrier_hz, p.filter_hz, p.filter_q, p.filter_open,
           modes[(int)p.stereo_mode]);
    return 0;
}
```
Note: `ReadWav` copies a mono file into both `left` and `right`, so mono files behave like a source on both inputs.

In `sim/render.sh`, change the usage comment to: `# Builds the render tool and runs it: render.sh in.wav out.wav [tune=Hz] [fb=] [mix=] [ring=] [carrier=Hz] [filter=Hz] [q=] [open=] [stereo=linked|mirror|rotate]`.

- [ ] **Step 2: Render the receiver sounds**

```bash
cd ~/local_files/daisy/bitchshifter/sim/out
../render.sh voice.wav p2_radio_band.wav filter=1200 q=5 open=0
../render.sh voice.wav p2_ssb_mistuned.wav tune=180 filter=1100 q=4 open=0
../render.sh voice.wav p2_ring_bells.wav ring=1 carrier=700
../render.sh voice.wav p2_ring_tremolo.wav ring=0.8 carrier=6
../render.sh voice.wav p2_whistly.wav tune=-90 fb=0.6 filter=1800 q=12 open=0
../render.sh stereo_source.wav p2_stereo_mirror.wav tune=60 filter=1400 q=3 open=0 stereo=mirror
../render.sh stereo_source.wav p2_stereo_rotate.wav tune=150 ring=0.4 carrier=40 filter=1000 q=4 open=0 stereo=rotate
```
Expected: seven `Wrote …` lines, no compiler warnings. (`stereo_source.wav` is the stereo test file already in `sim/out/`; if it's missing, skip the last two and report it.)

- [ ] **Step 3: User listens, then commit**

Send the seven files to the user. After they're happy:
```bash
cd ~/local_files/daisy/bitchshifter
git add sim/render.cpp sim/render.sh
git commit -m "Render tool: key=value options for every engine parameter"
```

---

### Task 8: Simple Touch: knobs 1–4, TOU pad stereo mode

**Files:**
- Modify: `simple_touch.cpp`

- [ ] **Step 1: Wire the controls**

In `simple_touch.cpp`:
- Add `#include "mapping.h"` and `#include "blips.h"` after `#include "engine.h"`.
- Replace the control map constants with:
```cpp
// Control map, from hwtest/ and padtest/ (see README.md). None of the analog
// controls read reversed.
const int kInputTune = 6;     // fader 1
const int kInputRing = 1;     // knob 1
const int kInputCarrier = 2;  // knob 2
const int kInputFilter = 3;   // knob 3
const int kInputWidth = 4;    // knob 4
const int kInputFeedback = 0; // knob 5
const int kInputMix = 5;      // knob 6
const int kPadStereoMode = 10; // "TOU" of the TOUCH logo
```
- Delete `kMaxShiftHz`, `kTuneDeadZone`, the `kFlip…` constants and the local `TuneHz()` (now `bitch::TuneHz`), and simplify `Read`:
```cpp
float Read(int input) { return controls[input].Value(); }
```
- Add globals next to `engine`:
```cpp
Mpr121I2C pads;
bitch::Blips blips;
volatile int stereo_mode = 0; // 0 Linked, 1 Mirror, 2 Rotate (cycled by the TOU pad)
```
- In `AudioCallback`, replace the `p.` lines with:
```cpp
    p.tune_hz = bitch::TuneHz(Read(kInputTune));
    p.feedback = 0.95f * Read(kInputFeedback);
    p.mix = Read(kInputMix);
    p.ring = Read(kInputRing);
    p.carrier_hz = bitch::CarrierHz(Read(kInputCarrier));
    p.filter_hz = bitch::FilterHz(Read(kInputFilter));
    bitch::FilterWidth(Read(kInputWidth), p.filter_q, p.filter_open);
    p.stereo_mode = (bitch::StereoMode)stereo_mode;
```
and right after `engine.Process(...)`, add the blips to both outputs:
```cpp
    for (size_t i = 0; i < size; i++) {
        float b = blips.Process();
        out[0][i] += b;
        out[1][i] += b;
    }
```
(The existing peak-measurement loop stays after this.)
- In `main()`, before `hw.StartAudio(...)`:
```cpp
    Mpr121I2C::Config pad_config;
    bool pads_ok = pads.Init(pad_config) == Mpr121I2C::OK;
    blips.Init(hw.AudioSampleRate());
```
- Replace the main loop with one that polls the pads every 10 ms and reports once a second:
```cpp
    static const char *mode_names[3] = {"linked", "mirror", "rotate"};
    bool was_touched = false;
    uint32_t last_report = System::GetNow();
    while (1) {
        System::Delay(10);
        // TOU pad: each new touch steps the stereo mode; 1, 2 or 3 blips say which
        bool touched = pads_ok && (pads.Touched() & (1 << kPadStereoMode));
        if (touched && !was_touched) {
            stereo_mode = (stereo_mode + 1) % 3;
            blips.Start(stereo_mode + 1);
        }
        was_touched = touched;
        if (System::GetNow() - last_report < 1000) continue;
        last_report = System::GetNow();
        auto db = [](float v) { return v > 0.0000112f ? (int)lrintf(20.f * log10f(v)) : -99; };
        float q, open;
        bitch::FilterWidth(Read(kInputWidth), q, open);
        hw.PrintLine("CPU avg %d%% max %d%% | tune %d Hz fb %d%% mix %d%% | ring %d%% carrier %d Hz | "
                     "filter %d Hz q %d.%d open %d%% | %s%s | in L %d R %d dB | out L %d R %d dB",
                     (int)(cpu.GetAvgCpuLoad() * 100.f), (int)(cpu.GetMaxCpuLoad() * 100.f),
                     (int)bitch::TuneHz(Read(kInputTune)), (int)(Read(kInputFeedback) * 100.f),
                     (int)(Read(kInputMix) * 100.f), (int)(Read(kInputRing) * 100.f),
                     (int)bitch::CarrierHz(Read(kInputCarrier)), (int)bitch::FilterHz(Read(kInputFilter)),
                     (int)q, (int)(q * 10.f) % 10, (int)(open * 100.f), mode_names[stereo_mode],
                     pads_ok ? "" : " (pads not found)", db(in_peak[0]), db(in_peak[1]), db(out_peak[0]),
                     db(out_peak[1]));
        in_peak[0] = in_peak[1] = out_peak[0] = out_peak[1] = 0.f;
        cpu.Reset();
    }
```
- Update the file's top comment to list the seven controls and the TOU pad.

Note on threads: `blips.Start()` runs in the main loop while `blips.Process()` runs in the audio interrupt. A blip sequence starting mid-block is harmless (the worst case is one sample of an old sequence), so no locking is needed.

- [ ] **Step 2: Build**

Run: `make -C ~/local_files/daisy/bitchshifter`
Expected: memory table, no errors or warnings.

- [ ] **Step 3: User flashes and plays it**

User runs `make program-dfu` in `~/local_files/daisy/bitchshifter`. With a source into the inputs:
- WIDTH (knob 4) fully clockwise: filter open, like part 1. Turn it back: the band narrows, boxy then whistly. FILTER (knob 3) sweeps it.
- RING (knob 1) up, CARRIER (knob 2) low: tremolo; higher: metallic, bell-like.
- A mono source in one input comes out of both sides at full level.
- Tap TOU: 2 blips = Mirror (TUNE spreads the sides apart in opposite directions), 3 = Rotate (8 s cycle), 1 = back to Linked.
- Read the USB line (`cat /dev/cu.usbmodem*`): controls track, the stereo mode shows, CPU still modest (expect roughly 15–20%).

- [ ] **Step 4: Document and commit (after the user confirms)**

In `README.md`, rename the "Part 1: playable shifter" heading to "Controls so far" and replace its table with:
```markdown
| Control | Does |
|---|---|
| Fader 1 TUNE | Shift −2500 … 0 (centre) … +2500 Hz, fine near the centre |
| Knob 1 RING | Ring-mod amount |
| Knob 2 CARRIER | Ring-mod carrier, 0.1 Hz (tremolo) – 5 kHz (bells) |
| Knob 3 FILTER | Radio filter centre, 150 Hz – 5 kHz |
| Knob 4 WIDTH | Radio filter: narrow and whistly → wide → fully open (last 15% of the turn) |
| Knob 5 FEEDBACK | Feedback 0–95% (barberpole) |
| Knob 6 MIX | Dry/wet |
| "TOU" logo pad | Stereo mode: 1 blip Linked, 2 Mirror, 3 Rotate (8 s) |

Stereo in, stereo out; a mono source in one input feeds both sides at full level.
```
and change the render line to `sim/render.sh in.wav out.wav [tune=Hz] [fb=] [mix=] [ring=] [carrier=Hz] [filter=Hz] [q=] [open=] [stereo=linked|mirror|rotate]`.
```bash
cd ~/local_files/daisy/bitchshifter
git add simple_touch.cpp README.md
git commit -m "Simple Touch firmware: ring mod and radio filter on knobs 1-4, stereo mode on the TOU pad"
```

---

## Later plans

- **Part 3:** Reception (fader 2): drift (each side on its own random path), fading, static, AGC, drive.
- **Part 4:** pads and switches (stations, keyboard, performance), designed around the measured layout (3 + 5 + 2 pads plus the two TOUCH logo pads; top centre is hard to reach; TOU is now the stereo mode, CH is reserved for the carrier input mode).
- **Part 5 (optional):** the carrier input mode on CH (IN L audio, IN R external ring-mod / single-sideband carrier); reverb; Daisy Patch front end.
