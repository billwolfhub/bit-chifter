# Bitchshifter Part 3b (SSB Dial) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make fader 1 TUNE an SSB tuning dial: a fine ±400 Hz linear zone with swoop zones at the ends (to ±2 kHz), inertia so moves become smooth sweeps, a tuning hand that wanders the dial with RECEPTION, and harmonic detents that pull the shift onto multiples of the source's pitch.

**Architecture:** Two new header-only engine pieces with Mac tests: `SoftQuantize()` (the detent curve, `detents.h`) and `PitchTracker` (YIN on the input decimated to 8 kHz, its work spread across blocks, `pitch_tracker.h`). `TuneHz()` in `mapping.h` gets the new dial curve. The engine computes the shift once per block as: TUNE → two-stage inertia → + tuning hand (a shared `Wander`, ±150 Hz × reception) → `SoftQuantize` with the tracked pitch, strength × confidence → the existing per-sample glide, stereo side gains and per-side drift. **In the engine, detents and inertia default to off** (`Params::detent = 0`, `Params::dial_inertia_s = 0`), so every existing test keeps its meaning (they use pure 1 kHz sines the tracker would lock onto); the firmware turns them on (0.7 and 0.06 s).

**Tech Stack:** C++17 (also gnu++14 for the firmware), libDaisy v8.1, Apple clang for `sim/`.

**Spec:** `docs/superpowers/specs/2026-10-12-bitchshifter-design.md` (Faders → 1 TUNE and 2 RECEPTION; Signal chain item 3, the shift and harmonic detents).

**Conventions:**
- Project root `~/local_files/daisy/bitchshifter`, local git, master. No `Co-Authored-By` lines.
- Tests: `sim/run_tests.sh` (prints `All tests passed`). Helpers in `sim/test_engine.cpp`: `CHECK`, `kSampleRate` (48000.f), `kPi`, `Magnitude`, `Db`, `RunEngine`, `RunEngineStereo`, `Window`; `Engine::ShifterPhase(int side)` exists (0–1 oscillator phase).
- Engine code: single precision only, nothing allocating in `Process()`; the firmware compiles it as gnu++14, so new `static constexpr` members get out-of-class definitions in `engine/engine.cpp`.
- If a test fails after implementing, don't change tolerances or the given algorithm to make it pass: report the printed numbers.
- Hardware steps: the user flashes and confirms before the firmware commit.

## File structure

| File | Responsibility |
|---|---|
| `engine/mapping.h` | `TuneHz()`: the new dial curve |
| `engine/detents.h` | `SoftQuantize(shift, step, amount)` |
| `engine/pitch_tracker.h` | `PitchTracker`: `Push()` per sample, `Work()` per block, `Pitch()`, `Confidence()` |
| `engine/engine.h`, `engine/engine.cpp` | `Params::detent`, `Params::dial_inertia_s`; dial inertia, tuning hand, tracker, detents; `Pitch()` / `PitchConfidence()` accessors |
| `sim/test_engine.cpp` | tests |
| `sim/render.cpp`, `sim/render.sh` | `sweep=`, `detent=`, `inertia=` options |
| `simple_touch.cpp`, `README.md` | the firmware switches detents and inertia on; pitch in the USB report |

---

### Task 1: The dial curve

**Files:**
- Modify: `engine/mapping.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Update the mapping tests (they will fail)**

In `TestMappings()` in `sim/test_engine.cpp`, replace these three existing checks:
```cpp
    CHECK(fabsf(bitch::TuneHz(1.f) - 2500.f) < 0.5f, "tune top %.1f", bitch::TuneHz(1.f));
    CHECK(fabsf(bitch::TuneHz(0.f) + 2500.f) < 0.5f, "tune bottom %.1f", bitch::TuneHz(0.f));
    CHECK(bitch::TuneHz(0.6f) > 0.f && bitch::TuneHz(0.6f) < 30.f, "tune fine near centre %.2f", bitch::TuneHz(0.6f));
```
with:
```cpp
    CHECK(fabsf(bitch::TuneHz(1.f) - 2000.f) < 0.5f, "tune top %.1f", bitch::TuneHz(1.f));
    CHECK(fabsf(bitch::TuneHz(0.f) + 2000.f) < 0.5f, "tune bottom %.1f", bitch::TuneHz(0.f));
    CHECK(bitch::TuneHz(0.6f) > 80.f && bitch::TuneHz(0.6f) < 100.f, "tune fine zone %.2f", bitch::TuneHz(0.6f));
    CHECK(fabsf(bitch::TuneHz(0.9f) - 398.f) < 3.f, "tune end of the fine zone %.1f", bitch::TuneHz(0.9f));
    {
        // Increasing all the way, with no flat spot or jump at the start of the swoop zone
        float prev = bitch::TuneHz(0.f), max_jump = 0.f;
        bool increasing = true;
        for (int i = 1; i <= 1000; i++) {
            float v = bitch::TuneHz(i / 1000.f);
            increasing = increasing && v >= prev;
            max_jump = std::max(max_jump, v - prev);
            prev = v;
        }
        CHECK(increasing, "tune: not increasing");
        CHECK(max_jump < 40.f, "tune: jumps %.1f Hz in a 0.1%% fader step", max_jump);
    }
```

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: FAIL lines for `tune top`, `tune bottom`, `tune fine zone` and `tune end of the fine zone`.

- [ ] **Step 3: Write the new curve**

In `engine/mapping.h`, replace the existing `TuneHz` function (and its comment) with:
```cpp
// Fader -> the SSB dial: 0 in a small dead zone at the centre; the middle 80% of the
// travel is a fine, linear +-400 Hz (where mistuned sideband sweeps through its harmonic
// "snaps"); the last 10% at each end accelerates to +-2000 Hz for big swoops. The slope
// is continuous at the knee, so there's no flat spot.
inline float TuneHz(float fader) {
    const float kDeadZone = 0.02f, kFine = 0.8f, kFineHz = 400.f, kMaxHz = 2000.f;
    const float slope = kFineHz / kFine;                                                    // Hz per unit, fine zone
    const float curve = (kMaxHz - kFineHz - slope * (1.f - kFine)) / ((1.f - kFine) * (1.f - kFine));
    float x = 2.f * fader - 1.f;
    if (fabsf(x) < kDeadZone) return 0.f;
    float m = fminf((fabsf(x) - kDeadZone) / (1.f - kDeadZone), 1.f);
    float hz;
    if (m <= kFine) {
        hz = slope * m;
    } else {
        float d = m - kFine;
        hz = kFineHz + slope * d + curve * d * d;
    }
    return copysignf(hz, x);
}
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/mapping.h sim/test_engine.cpp
git commit -m "Engine: TUNE as an SSB dial curve (fine +-400 Hz, swoop zones to +-2 kHz)"
```

---

### Task 2: The detent curve

**Files:**
- Create: `engine/detents.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "detents.h"` with the other engine includes, and above `main()`:
```cpp
static void TestSoftQuantize() {
    // Off: unchanged
    CHECK(bitch::SoftQuantize(37.f, 120.f, 0.f) == 37.f, "detents off: changed");
    CHECK(bitch::SoftQuantize(37.f, 0.f, 0.7f) == 37.f, "detents, no pitch: changed");
    // Multiples of the pitch stay put
    for (int k = -3; k <= 3; k++) {
        float s = 120.f * k;
        CHECK(fabsf(bitch::SoftQuantize(s, 120.f, 0.7f) - s) < 1e-3f, "detents: %g Hz moved to %g", s,
              bitch::SoftQuantize(s, 120.f, 0.7f));
    }
    // Near a multiple, pulled towards it (but not past it)
    float a = bitch::SoftQuantize(10.f, 120.f, 0.7f), b = bitch::SoftQuantize(110.f, 120.f, 0.7f);
    CHECK(a > 0.f && a < 10.f, "detents: 10 Hz -> %g (want 0..10)", a);
    CHECK(b > 110.f && b < 120.f, "detents: 110 Hz -> %g (want 110..120)", b);
    // Continuous and increasing, so a sweep passes straight through
    float prev = bitch::SoftQuantize(-500.f, 120.f, 0.7f);
    bool increasing = true;
    for (float s = -499.9f; s <= 500.f; s += 0.1f) {
        float v = bitch::SoftQuantize(s, 120.f, 0.7f);
        increasing = increasing && v > prev;
        prev = v;
    }
    CHECK(increasing, "detents: not increasing through a sweep");
}
```
Call `TestSoftQuantize();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile error, `'detents.h' file not found`.

- [ ] **Step 3: Write the curve**

`engine/detents.h`:
```cpp
#pragma once

#include <cmath>

namespace bitch {

// Harmonic detents: a mistuned sideband signal is harmonic again whenever the shift is a
// multiple of the source's pitch (all components land on a new harmonic series). This
// soft-quantises the shift towards those multiples: continuous and always increasing
// (for amount < 1), so a sweep passes straight through, but flattened around each
// multiple, so the sound lingers there. amount 0 (or no pitch) leaves the shift as it is.
inline float SoftQuantize(float shift, float step, float amount) {
    if (!(step > 1.f) || !(amount > 0.f)) return shift;
    float a = fminf(amount, 0.95f);
    const float kTwoPi = 6.28318530718f;
    return shift - a * step * (1.f / kTwoPi) * sinf(kTwoPi * shift / step);
}

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/detents.h sim/test_engine.cpp
git commit -m "Engine: harmonic detent curve"
```

---

### Task 3: The pitch tracker

**Files:**
- Create: `engine/pitch_tracker.h`
- Modify: `engine/engine.cpp` (out-of-class constexpr definitions)
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "pitch_tracker.h"`, and above `main()`:
```cpp
// Runs the tracker for `seconds` on a signal (block 48: Push per sample, Work per block)
template <class Signal>
static void Track(bitch::PitchTracker &t, float seconds, Signal signal) {
    t.Init(kSampleRate);
    int blocks = (int)(seconds * kSampleRate / 48);
    for (int b = 0; b < blocks; b++) {
        for (int k = 0; k < 48; k++) t.Push(signal(b * 48 + k));
        t.Work();
    }
}

static void TestPitchTracker() {
    bitch::PitchTracker t;
    Track(t, 0.5f, [](int n) { return 0.25f * sinf(2 * kPi * 100.f * n / kSampleRate); });
    CHECK(fabsf(t.Pitch() - 100.f) < 1.f && t.Confidence() > 0.8f, "pitch: 100 Hz sine -> %.2f Hz (conf %.2f)",
          t.Pitch(), t.Confidence());
    Track(t, 0.5f, [](int n) { // sawtooth: harmonic-rich, like most real sources
        float x = 150.f * n / kSampleRate;
        return 0.25f * (2.f * (x - floorf(x)) - 1.f);
    });
    CHECK(fabsf(t.Pitch() - 150.f) < 2.f && t.Confidence() > 0.7f, "pitch: 150 Hz saw -> %.2f Hz (conf %.2f)",
          t.Pitch(), t.Confidence());
    Track(t, 0.5f, [](int n) { return 0.25f * sinf(2 * kPi * 440.f * n / kSampleRate); });
    CHECK(fabsf(t.Pitch() - 440.f) < 8.f && t.Confidence() > 0.8f, "pitch: 440 Hz sine -> %.2f Hz (conf %.2f)",
          t.Pitch(), t.Confidence());
    uint32_t seed = 3;
    Track(t, 0.5f, [&seed](int) {
        seed = seed * 1664525u + 1013904223u;
        return ((seed >> 8) / 8388608.f - 1.f) * 0.25f;
    });
    CHECK(t.Confidence() < 0.3f, "pitch: noise confidence %.2f", t.Confidence());
    Track(t, 0.5f, [](int) { return 0.f; });
    CHECK(t.Confidence() == 0.f, "pitch: silence confidence %.2f", t.Confidence());
}
```
Call `TestPitchTracker();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile error, `'pitch_tracker.h' file not found`.

- [ ] **Step 3: Write the tracker**

`engine/pitch_tracker.h`:
```cpp
#pragma once

#include <cmath>

namespace bitch {

// Pitch tracker for the harmonic detents: YIN (de Cheveigne & Kawahara, 2002) on the
// input decimated to 8 kHz, 60-800 Hz. A new analysis starts every 32 ms; its difference
// function is spread over the following audio blocks (a few lags per Work() call), so
// there are no CPU spikes. Push() every input sample, Work() once per block.
// Confidence() is 0-1 (0 = silence or no clear pitch).
class PitchTracker {
  public:
    static constexpr int kDecimate = 6;   // 48 kHz -> 8 kHz
    static constexpr int kWindow = 256;   // 32 ms at 8 kHz
    static constexpr int kMinLag = 10;    // 800 Hz
    static constexpr int kMaxLag = 133;   // ~60 Hz
    static constexpr int kFrame = kWindow + kMaxLag + 1;
    static constexpr int kLagsPerWork = 6; // all 134 lags in ~23 blocks, well within 32 ms
    static constexpr float kThreshold = 0.15f;

    void Init(float sample_rate) {
        rate_ = sample_rate / (float)kDecimate;
        lowpass_ = 1.f - expf(-6.28318530718f * 1000.f / sample_rate);
        Reset();
    }

    void Reset() {
        state_ = sum_ = 0.f;
        count_ = write_ = filled_ = since_analysis_ = 0;
        lag_ = -1;
        pitch_ = confidence_ = 0.f;
        for (float &v : history_) v = 0.f;
    }

    void Push(float x) {
        state_ += lowpass_ * (x - state_); // 1 kHz low-pass against aliasing, then average 6
        sum_ += state_;
        if (++count_ < kDecimate) return;
        history_[write_] = sum_ * (1.f / (float)kDecimate);
        write_ = (write_ + 1) % kFrame;
        sum_ = 0.f;
        count_ = 0;
        if (filled_ < kFrame) filled_++;
        since_analysis_++;
    }

    void Work() {
        if (lag_ < 0) {
            if (filled_ < kFrame || since_analysis_ < kWindow) return;
            since_analysis_ = 0;
            float energy = 0.f;
            for (int i = 0; i < kFrame; i++) frame_[i] = history_[(write_ + i) % kFrame]; // oldest first
            for (int i = 0; i < kWindow; i++) energy += frame_[i] * frame_[i];
            if (energy < (float)kWindow * 1e-7f) { // silence (below about -70 dBFS)
                confidence_ = 0.f;
                return;
            }
            lag_ = 1;
            running_sum_ = 0.f;
            cmndf_[0] = 1.f;
        }
        for (int k = 0; k < kLagsPerWork && lag_ <= kMaxLag; k++, lag_++) {
            float d = 0.f;
            for (int i = 0; i < kWindow; i++) {
                float e = frame_[i] - frame_[i + lag_];
                d += e * e;
            }
            running_sum_ += d;
            cmndf_[lag_] = running_sum_ > 0.f ? d * (float)lag_ / running_sum_ : 1.f;
        }
        if (lag_ > kMaxLag) {
            Finish();
            lag_ = -1;
        }
    }

    float Pitch() const { return pitch_; }
    float Confidence() const { return confidence_; }

  private:
    // The first lag below the threshold (followed down to its local minimum) is the period
    void Finish() {
        int best = -1;
        for (int t = kMinLag; t <= kMaxLag; t++) {
            if (cmndf_[t] < kThreshold) {
                while (t + 1 <= kMaxLag && cmndf_[t + 1] < cmndf_[t]) t++;
                best = t;
                break;
            }
        }
        if (best < 0) {
            confidence_ = 0.f;
            return;
        }
        float period = (float)best;
        if (best > kMinLag && best < kMaxLag) { // parabolic interpolation between lags
            float a = cmndf_[best - 1], b = cmndf_[best], c = cmndf_[best + 1];
            float den = a - 2.f * b + c;
            if (fabsf(den) > 1e-9f) period += 0.5f * (a - c) / den;
        }
        pitch_ = rate_ / period;
        confidence_ = fminf(fmaxf(1.f - cmndf_[best], 0.f), 1.f);
    }

    float rate_ = 8000.f, lowpass_ = 0.f, state_ = 0.f, sum_ = 0.f;
    int count_ = 0, write_ = 0, filled_ = 0, since_analysis_ = 0, lag_ = -1;
    float history_[kFrame] = {};
    float frame_[kFrame] = {};
    float cmndf_[kMaxLag + 1] = {};
    float running_sum_ = 0.f;
    float pitch_ = 0.f, confidence_ = 0.f;
};

} // namespace bitch
```
In `engine/engine.cpp`, add `#include "pitch_tracker.h"` at the top (engine.h will include it after Task 4) and, with the other out-of-class definitions:
```cpp
constexpr int PitchTracker::kDecimate;
constexpr int PitchTracker::kWindow;
constexpr int PitchTracker::kMinLag;
constexpr int PitchTracker::kMaxLag;
constexpr int PitchTracker::kFrame;
constexpr int PitchTracker::kLagsPerWork;
constexpr float PitchTracker::kThreshold;
```

- [ ] **Step 4: Run the tests, and check the firmware build**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed`. If a pitch is off by an octave or the confidence misses, report the numbers (don't tune the threshold to the test).
Run: `make -C ~/local_files/daisy/bitchshifter` — expected: builds, no warnings.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/pitch_tracker.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: YIN pitch tracker, spread across blocks"
```

---

### Task 4: The dial in the engine

**Files:**
- Modify: `engine/engine.h`
- Modify: `engine/engine.cpp`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp`, above `main()`:
```cpp
// Runs an engine block by block on a signal into both inputs, returning the left
// shifter's frequency (from its phase advance) at the end of each block
template <class Signal>
static std::vector<float> ShiftTrace(bitch::Engine &e, const bitch::Params &p, float seconds, Signal signal) {
    std::vector<float> trace;
    float in[48], out_l[48], out_r[48];
    float prev = e.ShifterPhase(0);
    int blocks = (int)(seconds * kSampleRate / 48);
    for (int b = 0; b < blocks; b++) {
        for (int k = 0; k < 48; k++) in[k] = signal(b * 48 + k);
        e.Process(in, in, out_l, out_r, 48, p);
        float phase = e.ShifterPhase(0), d = phase - prev;
        d -= floorf(d + 0.5f);
        trace.push_back(d * kSampleRate / 48.f);
        prev = phase;
    }
    return trace;
}

static float Sawtooth120(int n) {
    float x = 120.f * n / kSampleRate;
    return 0.25f * (2.f * (x - floorf(x)) - 1.f);
}

// With inertia, a jump of the dial becomes a sweep
static void TestEngineDialInertia() {
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.tune_hz = 0.f;
    ShiftTrace(e, p, 0.2f, [](int) { return 0.f; });
    p.tune_hz = 200.f;
    p.dial_inertia_s = 0.06f;
    auto trace = ShiftTrace(e, p, 1.f, [](int) { return 0.f; });
    CHECK(trace[50] < 120.f, "inertia: already at %.1f Hz after 50 ms", trace[50]);
    CHECK(fabsf(trace.back() - 200.f) < 1.f, "inertia: settled at %.1f Hz", trace.back());
    bitch::Engine quick;
    quick.Init(kSampleRate);
    p.tune_hz = 0.f;
    p.dial_inertia_s = 0.f;
    ShiftTrace(quick, p, 0.2f, [](int) { return 0.f; });
    p.tune_hz = 200.f;
    auto direct = ShiftTrace(quick, p, 0.1f, [](int) { return 0.f; });
    CHECK(direct[50] > 180.f, "no inertia: only %.1f Hz after 50 ms", direct[50]);
}

// Detents pull the shift towards a multiple of the source's pitch (120 Hz sawtooth)
static void TestEngineDetents() {
    bitch::Params p;
    p.tune_hz = 100.f;
    p.detent = 0.7f;
    bitch::Engine e;
    e.Init(kSampleRate);
    auto pulled = ShiftTrace(e, p, 1.5f, Sawtooth120);
    CHECK(pulled.back() > 104.f && pulled.back() < 119.f, "detents: 100 Hz shift on a 120 Hz source -> %.1f Hz",
          pulled.back());
    CHECK(fabsf(e.Pitch() - 120.f) < 3.f && e.PitchConfidence() > 0.7f, "detents: tracked %.1f Hz (conf %.2f)",
          e.Pitch(), e.PitchConfidence());
    p.detent = 0.f;
    bitch::Engine plain;
    plain.Init(kSampleRate);
    auto free_shift = ShiftTrace(plain, p, 1.5f, Sawtooth120);
    CHECK(fabsf(free_shift.back() - 100.f) < 0.5f, "no detents: %.1f Hz", free_shift.back());
}

// With reception up, the tuning hand wanders the dial around the TUNE setting
static void TestEngineTuningHand() {
    bitch::Params p;
    p.reception = 1.f;
    bitch::Engine e;
    e.Init(kSampleRate);
    auto trace = ShiftTrace(e, p, 10.f, [](int) { return 0.f; });
    float lo = 1e9f, hi = -1e9f;
    for (size_t i = trace.size() / 5; i < trace.size(); i++) {
        lo = std::min(lo, trace[i]);
        hi = std::max(hi, trace[i]);
    }
    CHECK(hi - lo > 80.f, "tuning hand: only wandered %.1f Hz", hi - lo);
}
```
Call the three from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile errors, `no member named 'dial_inertia_s'` / `'detent'` in `bitch::Params`, `no member named 'Pitch'` in `bitch::Engine`.

- [ ] **Step 3: Wire the dial into the engine**

`engine/engine.h`:
- Add includes after the existing ones: `#include "detents.h"` and `#include "pitch_tracker.h"`.
- Add to `Params` (after `reception`):
```cpp
    float detent = 0.f;          // harmonic detent strength 0-0.95 (0 = off; the firmware uses 0.7)
    float dial_inertia_s = 0.f;  // TUNE inertia, per stage of two (0 = off; the firmware uses 0.06)
```
- Add public accessors to `Engine` (next to `ShifterPhase`):
```cpp
    float Pitch() const { return pitch_.Pitch(); }                // the source's tracked fundamental, Hz
    float PitchConfidence() const { return pitch_.Confidence(); } // 0-1
```
- Add private members (after the Reception members):
```cpp
    // The SSB dial: inertia on TUNE, the tuning hand, and the harmonic detents
    float dial_[2] = {0.f, 0.f}; // two-stage inertia
    Wander hand_;                // wanders the dial with reception (shared by both sides)
    PitchTracker pitch_;
    float detent_amount_ = 0.f;  // strength x confidence, smoothed per block
    float target_shift_ = 0.f;   // the dial's shift for this block; shift_hz_ glides to it
```
- Extend the class comment: "The shift (TUNE) goes through optional inertia, the tuning hand (reception x ±150 Hz) and the harmonic detents (pulled onto multiples of the tracked pitch) once per block."

`engine/engine.cpp`:
- In `Init`, after the per-side Reception `Init` loop: `hand_.Init(block_rate, 47u);` and `pitch_.Init(sample_rate);`.
- In `Reset()`: `dial_[0] = dial_[1] = 0.f; detent_amount_ = 0.f; target_shift_ = 0.f; pitch_.Reset();`.
- In `Process()`:
  - In the first-block snap (`if (block_glide_ == 0.f)`), add `dial_[0] = dial_[1] = tune;`.
  - After the per-block Reception updates (after the loop that sets `drift_hz[c]`), add:
```cpp
    // The SSB dial, once per block: TUNE through the inertia, plus the tuning hand, then
    // pulled towards multiples of the source's pitch (strength x confidence, smoothed)
    float dial = tune;
    if (params.dial_inertia_s > 0.f) {
        float k = 1.f - expf(-(float)size / (params.dial_inertia_s * sample_rate_));
        dial_[0] += k * (tune - dial_[0]);
        dial_[1] += k * (dial_[0] - dial_[1]);
        dial = dial_[1];
    } else {
        dial_[0] = dial_[1] = tune;
    }
    float hand = r > 0.f ? r * 150.f * hand_.Next(0.15f + 0.15f * r) : 0.f;
    float detent_target = Clamp(params.detent, 0.f, 0.95f) * pitch_.Confidence();
    detent_amount_ += block_glide_ * (detent_target - detent_amount_);
    if (detent_amount_ < 1e-4f && detent_target == 0.f) detent_amount_ = 0.f;
    target_shift_ = SoftQuantize(dial + hand, pitch_.Pitch(), detent_amount_);
```
  - In the per-sample loop, change `shift_hz_ += glide_ * (tune - shift_hz_);` to `shift_hz_ += glide_ * (target_shift_ - shift_hz_);`, and at the top of the loop body (after `float x[2] = {in_l[i], in_r[i]};`) add `pitch_.Push(0.5f * (x[0] + x[1]));`.
  - After the per-sample loop (before the mono-detection hold loop), add `pitch_.Work();`.

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed`, including every earlier test (detents and inertia default to off; the hand is 0 at reception 0).

- [ ] **Step 5: Check the firmware still builds**

Run: `make -C ~/local_files/daisy/bitchshifter`
Expected: memory table, no errors or warnings.

- [ ] **Step 6: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/engine.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: SSB dial (inertia, tuning hand, harmonic detents)"
```

---

### Task 5: Render options, a drone, and listening

**Files:**
- Modify: `sim/render.cpp`
- Modify: `sim/render.sh`

- [ ] **Step 1: Add the options**

In `sim/render.cpp`:
- Add `{"detent", &p.detent}` and `{"inertia", &p.dial_inertia_s}` to `fields[]`.
- Add a `sweep` option: a `float sweep_to = NAN;` local in `main()`, set by `sweep=<Hz>` (handle it in `SetOption` with an extra `float *sweep` parameter, or parse it in `main()` before calling `SetOption`). When set, the tune ramps linearly from `tune=` at the start to `sweep=` at the end of the file. In the block loop:
```cpp
    float tune_from = p.tune_hz;
    for (size_t i = 0; i + block <= in.left.size(); i += block) {
        if (!std::isnan(sweep_to)) p.tune_hz = tune_from + (sweep_to - tune_from) * (float)i / (float)in.left.size();
        engine.Process(&in.left[i], &in.right[i], &out.left[i], &out.right[i], block, p);
    }
```
  (add `#include <cmath>`; restore `p.tune_hz = tune_from;` before the final `printf`).
- Add `[detent=0-0.95] [inertia=s] [sweep=Hz]` to the usage comment and message, and `detent %g inertia %g` (and `sweep %g` when set) to the final `printf`.
In `sim/render.sh`, add `[detent=] [inertia=] [sweep=Hz]` to the usage comment.

- [ ] **Step 2: Make a drone and render**

```bash
cd ~/local_files/daisy/bitchshifter/sim/out
python3 - <<'EOF'
# drone.wav: 10 s sawtooth at 110 Hz with a little vibrato (a clear, harmonic source)
import math, struct, wave
sr, n = 48000, 480000
w = wave.open('drone.wav', 'wb'); w.setnchannels(1); w.setsampwidth(2); w.setframerate(sr)
phase, frames = 0.0, []
for i in range(n):
    f = 110.0 * (1 + 0.003 * math.sin(2 * math.pi * 5 * i / sr))
    phase = (phase + f / sr) % 1.0
    frames.append(struct.pack('<h', int(0.3 * (2 * phase - 1) * 32767)))
w.writeframes(b''.join(frames)); w.close()
print('wrote drone.wav')
EOF
../render.sh drone.wav p3b_drone_sweep_plain.wav tune=-300 sweep=300
../render.sh drone.wav p3b_drone_sweep_detents.wav tune=-300 sweep=300 detent=0.7 inertia=0.06
../render.sh voice.wav p3b_voice_sweep_detents.wav tune=-250 sweep=250 detent=0.7 inertia=0.06 filter=1100 q=3 open=0
../render.sh drone.wav p3b_drone_hand.wav tune=60 reception=0.6 detent=0.7 inertia=0.06
```
Expected: `wrote drone.wav` and four `Wrote …` lines, no compiler warnings.

- [ ] **Step 3: User listens, then commit**

Send the four renders to the user: the same −300 → +300 Hz sweep over a 110 Hz drone without and with detents (listen for the sound lingering harmonically at 0, ±110, ±220 Hz), the voice sweeping through, and the tuning hand wandering into the detents. After they're happy:
```bash
cd ~/local_files/daisy/bitchshifter
git add sim/render.cpp sim/render.sh
git commit -m "Render tool: sweep, detent and inertia options"
```

---

### Task 6: Firmware

**Files:**
- Modify: `simple_touch.cpp`
- Modify: `README.md`

- [ ] **Step 1: Turn the dial on**

In `simple_touch.cpp`'s `AudioCallback`, after the other `p.` lines:
```cpp
    p.detent = 0.7f;          // harmonic detents, medium
    p.dial_inertia_s = 0.06f; // TUNE glides like a heavy dial (two 60 ms stages)
```
In the report, add the tracked pitch to the second line: after `blip samples %d` add ` | pitch %d Hz %d%%` with the arguments `(int)engine.Pitch(), (int)(engine.PitchConfidence() * 100.f)`. Update the file's top comment: fader 1 is the SSB dial (±400 Hz fine, swoop zones to ±2 kHz, inertia, harmonic detents).

- [ ] **Step 2: Build**

Run: `make -C ~/local_files/daisy/bitchshifter`
Expected: memory table, no errors or warnings.

- [ ] **Step 3: User flashes and plays it**

User runs `make program-dfu` in `~/local_files/daisy/bitchshifter`. Try:
- Sweep TUNE slowly around the centre on a pitched sample: the sound should catch and linger where it turns harmonic again (at multiples of the sample's pitch), with chimes in between.
- Push TUNE to an end: a big swoop, gliding rather than jumping.
- RECEPTION up: the dial wanders on its own and settles into the detents.
- Read the USB report (`cat /dev/cu.usbmodem*`): `pitch … Hz …%` should follow the sample's pitch when it's clear; CPU roughly 23–25%.

- [ ] **Step 4: Document and commit (after the user confirms)**

In `README.md`'s "Controls so far" table, replace the TUNE row with:
```markdown
| Fader 1 TUNE | The SSB dial: fine ±400 Hz over the middle 80%, swoop zones to ±2 kHz at the ends; glides like a heavy dial; harmonic detents catch the shift at multiples of the source's pitch |
```
and add to the RECEPTION row: "and the dial wanders on its own". Add `[detent=] [inertia=] [sweep=Hz]` to the render line, and update the CPU note.
```bash
cd ~/local_files/daisy/bitchshifter
git add simple_touch.cpp README.md
git commit -m "Simple Touch firmware: TUNE as the SSB dial (detents, inertia), pitch in the report"
```

---

## Later plans

- **Part 4:** pads and switches (stations, keyboard, performance), including a detent-strength control and an optional "plain shift" mode.
- **Part 5 (optional):** carrier input mode on CH; reverb; the Daisy Patch front end (and other interfaces later: the engine and `mapping.h` are hardware-free).
