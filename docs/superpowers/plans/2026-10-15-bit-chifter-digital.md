# bit-chifter Strange Pages Step 5 (Digital & damage) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build page 3's damage effects (Bitcrush, Sample-rate crush, Dropouts, Stutter, Codec warble) and the Chaos pad.

**Architecture:** Small header-only classes in `engine/`, each tested on its own on the Mac: `crush.h` (bitcrush as a function; `RateCrush` sample-and-hold), `Dropouts` (a mono gate), `Stutter` (stereo: catches and repeats fragments), `CodecWarble` (stereo: an 8-band filter bank with randomly re-levelled bands). In the engine they sit after the radio filter and before the AGC, in the order codec → rate crush → bitcrush → stutter → dropouts, each faded in and out with its pad's level (ramped across the block) and run only while that level is above 0, so with the pads off the sound is bit-for-bit unchanged. Chaos is front-end pad logic in `ui/pad_pages.h`: while its pad is held, a random set of 2–3 other effects is on.

**Tech Stack:** C++17 (also gnu++14 for the firmware), libDaisy v8.1, Apple clang for `sim/`.

**Spec:** `docs/superpowers/specs/2026-10-13-bit-chifter-strange-pages-design.md` (Page 3: Digital & damage; Chaos).

**Conventions:**
- Project root `~/local_files/daisy/bit-chifter`, git, master (public on GitHub; don't push, the user asks for that). No `Co-Authored-By` or other attribution lines in commits.
- Tests: `sim/run_tests.sh` (prints `All tests passed`). Helpers in `sim/test_engine.cpp`: `CHECK(cond, fmt, ...)`, `kSampleRate` (48000.f), `kPi`, `Magnitude(x, freq)`, `Db(ratio)`, `Window(x, from_s, to_s)`, `Peak(x)`, `Energy(x)`, `Rms(x)`, `Sine(hz, amplitude, n)`, `RunFor(engine, params, seconds, signal, n, l, r)` (48-sample blocks of `signal(n)` into both inputs, appending both outputs), `kBuiltEffects` (used by the bit-exact `TestEffectsUnbuiltDoNothing`), and the existing `TestPadPages`. New test functions go above `main()` and are called from it.
- `EffectRange(amount)` in `engine/effects.h` maps the effect amount 0.35–1 to 0–1. `Random` (`engine/random.h`) has `Seed`, `Uniform()` (0–1), `Bipolar()`. `Bandpass` (`engine/radio_filter.h`) is a unity-peak band-pass: `SetCoefficients(g = tan(pi f / sr), k = 1 / Q)`, `Process(x)`, `Reset()`.
- Engine code: single precision only, nothing allocating in `Process()`; the firmware builds as gnu++14 (no odr-used `static constexpr` members: use enums or functions). The engine is a global in DTCM (128 KB, ~37 KB used): the stutter's buffers add ~46 KB, which fits.
- If a test fails after implementing, don't change tolerances or the given algorithm to make it pass: report the printed numbers.
- Hardware steps: the user listens to renders, and flashes and confirms before the firmware commit.

## File structure

| File | Responsibility |
|---|---|
| `engine/crush.h` | `CrushBits`, `CrushStep`, `Bitcrush`; `RateCrush` |
| `engine/dropouts.h` | `Dropouts` |
| `engine/stutter.h` | `Stutter` |
| `engine/codec_warble.h` | `CodecWarble` |
| `engine/engine.h`, `engine/engine.cpp` | the five damage effects wired in after the radio filter |
| `ui/pad_pages.h` | Chaos |
| `sim/test_engine.cpp` | tests |
| `simple_touch.cpp`, `README.md` | comment and controls table |

---

### Task 1: Bitcrush and sample-rate crush

**Files:**
- Create: `engine/crush.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "crush.h"` with the engine includes, and above `main()`:
```cpp
static void TestCrush() {
    CHECK(bitch::CrushBits(0.35f) == 10.f && bitch::CrushBits(1.f) == 3.f, "crush bits %.1f %.1f",
          bitch::CrushBits(0.35f), bitch::CrushBits(1.f));
    // 3 bits: only multiples of 0.25
    float step = bitch::CrushStep(3.f);
    bool on_grid = true;
    for (int n = 0; n < 4800; n++) {
        float y = bitch::Bitcrush(Sine(440.f, 0.8f, n), step, 1.f / step);
        float k = y / 0.25f;
        on_grid = on_grid && fabsf(k - roundf(k)) < 1e-6f;
    }
    CHECK(fabsf(step - 0.25f) < 1e-7f && on_grid, "bitcrush 3 bits: step %g, on the grid %d", step, (int)on_grid);
    // 10 bits: within half a step
    step = bitch::CrushStep(10.f);
    float worst = 0.f;
    for (int n = 0; n < 4800; n++) {
        float x = Sine(440.f, 0.8f, n);
        worst = std::max(worst, fabsf(bitch::Bitcrush(x, step, 1.f / step) - x));
    }
    CHECK(worst <= 0.5f * step + 1e-7f, "bitcrush 10 bits: error %g (step %g)", worst, step);
    // Sample-and-hold: 1.5 kHz at amount 1 (about 150 new values in 0.1 s), 12 kHz at 0.35
    bitch::RateCrush rc;
    rc.Init(kSampleRate);
    for (float amount : {1.f, 0.35f}) {
        rc.Reset();
        rc.Update(amount);
        int changes = 0;
        float prev = rc.Process(Sine(440.f, 0.8f, 0));
        for (int n = 1; n < 4800; n++) {
            float y = rc.Process(Sine(440.f, 0.8f, n));
            changes += y != prev;
            prev = y;
        }
        float want = 0.1f * rc.HoldRateHz();
        CHECK(fabsf((float)changes - want) <= 2.f + 0.01f * want, "rate crush %.2f: %d new values (want %.0f)",
              amount, changes, want);
    }
    CHECK(fabsf(rc.HoldRateHz() - 12000.f) < 1.f, "rate crush at 0.35: %.0f Hz", rc.HoldRateHz());
}
```
Call `TestCrush();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: compile error, `'crush.h' file not found`.

- [ ] **Step 3: Write the crushers**

`engine/crush.h`:
```cpp
#pragma once

#include <cmath>

#include "effects.h"

namespace bitch {

// Bitcrush: the signal quantised to `bits` bits over -1..1 (no dither): 10 bits (amount 0.35)
// down to 3 bits (amount 1). A mid-tread quantiser (0 is a level), so `bits` bits give
// 2^bits + 1 levels over -1..1 (3 bits: 9, steps of 0.25). CrushStep() and its inverse once
// per block, Bitcrush() per sample.
inline float CrushBits(float amount) { return 10.f - 7.f * EffectRange(amount); }
inline float CrushStep(float bits) { return 2.f / exp2f(bits); }
inline float Bitcrush(float x, float step, float inv_step) { return step * floorf(x * inv_step + 0.5f); }

// Sample-rate crush: sample-and-hold at 12 kHz (amount 0.35) down to 1.5 kHz (amount 1), with
// no anti-aliasing (the aliasing is the sound). Update() once per block, Process() per sample.
class RateCrush {
  public:
    void Init(float sample_rate) {
        sample_rate_ = sample_rate;
        Update(0.35f);
        Reset();
    }
    void Reset() {
        phase_ = 1.f; // take the first sample at once
        held_ = 0.f;
    }
    void Update(float amount) { step_ = 12000.f * powf(1500.f / 12000.f, EffectRange(amount)) / sample_rate_; }
    float Process(float x) {
        phase_ += step_;
        if (phase_ >= 1.f) {
            phase_ -= floorf(phase_);
            held_ = x;
        }
        return held_;
    }
    float HoldRateHz() const { return step_ * sample_rate_; }

  private:
    float sample_rate_ = 48000.f, step_ = 0.25f, phase_ = 1.f, held_ = 0.f;
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/crush.h sim/test_engine.cpp
git commit -m "Engine: bitcrush and sample-rate crush"
```

---

### Task 2: Dropouts

**Files:**
- Create: `engine/dropouts.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "dropouts.h"`, and above `main()`:
```cpp
// Runs the dropouts gate for `seconds`: the number of gaps, the fraction of time fully gone, and
// the largest step of the gain between samples
static void RunDropouts(float amount, float seconds, int &gaps, float &gone, float &step) {
    bitch::Dropouts d;
    d.Init(kSampleRate, 19u);
    gaps = 0;
    gone = 0.f;
    step = 0.f;
    float prev = 1.f;
    int total = 0;
    for (int b = 0; b < (int)(seconds * kSampleRate / 48); b++) {
        d.Update(amount);
        for (int k = 0; k < 48; k++) {
            float g = d.Gain();
            gaps += prev >= 0.5f && g < 0.5f;
            gone += g == 0.f;
            step = std::max(step, fabsf(g - prev));
            prev = g;
            total++;
        }
    }
    gone /= (float)total;
}

static void TestDropouts() {
    int gaps;
    float gone, step;
    RunDropouts(1.f, 20.f, gaps, gone, step);
    printf("  dropouts at 1: %.2f gaps/s (want 4), gone %.0f%% of the time\n", gaps / 20.f, 100.f * gone);
    CHECK(gaps >= 68 && gaps <= 92, "dropouts at 1: %.2f gaps/s (want 3.4-4.6)", gaps / 20.f);
    CHECK(gone > 0.2f && gone < 0.65f, "dropouts at 1: gone %.0f%% of the time", 100.f * gone);
    CHECK(step <= 1.f / 48.f + 1e-6f, "dropouts: gain steps by %.4f (1 ms edges: 1/48 at most)", step);
    RunDropouts(0.35f, 40.f, gaps, gone, step);
    printf("  dropouts at 0.35: %.2f gaps/s (want 0.3)\n", gaps / 40.f);
    CHECK(gaps >= 6 && gaps <= 20, "dropouts at 0.35: %.2f gaps/s (want 0.15-0.5)", gaps / 40.f);
}
```
Call `TestDropouts();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: compile error, `'dropouts.h' file not found`.

- [ ] **Step 3: Write the dropouts**

`engine/dropouts.h`:
```cpp
#pragma once

#include <cmath>
#include <cstdint>

#include "effects.h"
#include "random.h"

namespace bitch {

// Dropouts: the signal cuts out, like a digital stream losing packets: random gaps of
// 20-200 ms with 1 ms edges, starting 0.3 (amount 0.35) to 4 (amount 1) times a second on
// average. Gain() per sample (1 = through, 0 = gone; mono: both sides drop together),
// Update() once per block.
class Dropouts {
  public:
    void Init(float sample_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        edge_ = 1.f / (0.001f * sample_rate);
        random_.Seed(seed);
        Update(0.35f);
        Reset();
    }
    void Reset() {
        gain_ = 1.f;
        gap_left_ = 0;
    }
    // Gaps can only start outside a gap: the chance is scaled up so the average rate is as given
    // (0.112 s: the mean gap, 0.11 s, plus its edges)
    void Update(float amount) {
        float rate = 0.3f + 3.7f * EffectRange(amount);
        chance_ = rate / (1.f - rate * 0.112f) / sample_rate_;
    }
    float Gain() {
        if (gap_left_ > 0) {
            gap_left_--;
            gain_ = fmaxf(gain_ - edge_, 0.f);
        } else {
            if (random_.Uniform() < chance_) gap_left_ = (int)((0.02f + 0.18f * random_.Uniform()) * sample_rate_);
            gain_ = fminf(gain_ + edge_, 1.f);
        }
        return gain_;
    }

  private:
    float sample_rate_ = 48000.f, edge_ = 0.f, chance_ = 0.f, gain_ = 1.f;
    int gap_left_ = 0;
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
git add engine/dropouts.h sim/test_engine.cpp
git commit -m "Engine: dropouts"
```

---

### Task 3: Stutter

**Files:**
- Create: `engine/stutter.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "stutter.h"`, and above `main()`:
```cpp
static void TestStutter() {
    static bitch::Stutter s; // ~46 KB: off the stack
    s.Init(kSampleRate, 17u);
    // A slow rising ramp in, and its negative on the right: repeats play the past (lower values)
    int starts = 0, repeating = 0, from_past = 0;
    bool was = false, symmetric = true;
    for (int b = 0; b < (int)(4.f * kSampleRate / 48); b++) {
        s.Update(1.f, 48.f / kSampleRate);
        for (int k = 0; k < 48; k++) {
            int n = b * 48 + k;
            float live = 0.5f * (float)(n % 96000) / 96000.f;
            float l = live, r = -live;
            s.Process(l, r);
            symmetric = symmetric && r == -l;
            if (s.Repeating()) {
                repeating++;
                from_past += l < live - 0.001f;
            }
        }
        starts += s.Repeating() && !was;
        was = s.Repeating();
    }
    CHECK(starts >= 3, "stutter at 1: %d stutters in 4 s", starts);
    CHECK(repeating > 0 && from_past > repeating / 2, "stutter: %d of %d repeating samples from the past", from_past,
          repeating);
    CHECK(symmetric, "stutter: the sides don't repeat together");
    // Nothing until the buffer holds a fragment: right after Reset, the first 30 ms are live
    s.Reset();
    bool live_at_start = true;
    for (int n = 0; n < (int)(0.03f * kSampleRate); n++) {
        if (n % 48 == 0) s.Update(1.f, 48.f / kSampleRate);
        float l = (float)n, r = 0.f;
        s.Process(l, r);
        live_at_start = live_at_start && l == (float)n;
    }
    CHECK(live_at_start, "stutter: repeated before it had a whole fragment");
    // Every fragment is one unbroken stretch of the input: on a ramp (l = n, r = -n), where the
    // repeat is fully in (between the 2 ms fades) the output rises by exactly 1 a sample
    s.Init(kSampleRate, 17u);
    int fragments = 0, spliced = 0, jumps = 0;
    bool in_jump = false;
    float prev_l = 0.f, prev_g = 0.f;
    for (int b = 0; b < (int)(60.f * kSampleRate / 48); b++) {
        bool was_repeating = s.Repeating();
        s.Update(1.f, 48.f / kSampleRate);
        if (s.Repeating() && !was_repeating) {
            fragments++;
            in_jump = false;
        }
        for (int k = 0; k < 48; k++) {
            int n = b * 48 + k;
            float g = s.FadeGain();
            float l = (float)n, r = -(float)n;
            s.Process(l, r);
            if (g == 1.f && prev_g == 1.f && l - prev_l != 1.f) {
                jumps++;
                if (!in_jump) spliced++;
                in_jump = true;
            }
            prev_l = l;
            prev_g = g;
        }
    }
    printf("  stutter: %d of %d fragments spliced (%d jumps)\n", spliced, fragments, jumps);
    CHECK(fragments > 50 && jumps == 0, "stutter: %d of %d fragments contain a splice (%d jumps)", spliced, fragments,
          jumps);
}
```
Call `TestStutter();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: compile error, `'stutter.h' file not found`.

- [ ] **Step 3: Write the stutter**

`engine/stutter.h`:
```cpp
#pragma once

#include <cmath>
#include <cstdint>

#include "effects.h"
#include "random.h"

namespace bitch {

// Stutter: now and then catches the last 30-120 ms and repeats it 2-8 times before letting the
// signal through again. A stutter starts 0.5 (amount 0.35) to 4 (amount 1) times a second on
// average (while not already repeating). Each repeat is faded in and out over 2 ms against the
// live signal, so the loop points don't click. Stereo: both sides repeat together. Assumes
// 48 kHz (the buffer holds 120 ms). Update() once per block, Process() per sample.
class Stutter {
  public:
    enum : int { kMax = 5760 }; // 120 ms at 48 kHz

    void Init(float sample_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        fade_ = (int)(0.002f * sample_rate);
        random_.Seed(seed);
        Reset();
    }

    void Reset() {
        write_ = filled_ = 0;
        repeats_left_ = 0;
    }

    void Update(float amount, float block_seconds) {
        float rate = 0.5f + 3.5f * EffectRange(amount);
        if (repeats_left_ > 0 || random_.Uniform() >= rate * block_seconds) return;
        int length = (int)((0.03f + 0.09f * random_.Uniform()) * sample_rate_);
        if (length > kMax) length = kMax;
        if (filled_ < length) return; // not enough recorded yet
        length_ = length;
        start_ = write_ - length;
        if (start_ < 0) start_ += kMax;
        position_ = 0;
        repeats_left_ = 2 + (int)(random_.Uniform() * 7.f); // 2-8
        // Recording pauses while repeating: the next stutter needs a fresh, unbroken fragment
        // (one spanning the pause would have a splice in it)
        filled_ = 0;
    }

    void Process(float &l, float &r) {
        if (repeats_left_ <= 0) { // recording (only while not repeating, so the fragment stays intact)
            buffer_[0][write_] = l;
            buffer_[1][write_] = r;
            if (++write_ >= kMax) write_ = 0;
            if (filled_ < kMax) filled_++;
            return;
        }
        int read = start_ + position_;
        if (read >= kMax) read -= kMax;
        float g = FadeGain();
        l += g * (buffer_[0][read] - l);
        r += g * (buffer_[1][read] - r);
        if (++position_ >= length_) {
            position_ = 0;
            repeats_left_--;
        }
    }

    bool Repeating() const { return repeats_left_ > 0; }
    // The repeat's gain against the live signal for the next sample (0 = live, 1 = all repeat)
    float FadeGain() const {
        if (repeats_left_ <= 0) return 0.f;
        return fminf(fminf((float)position_, (float)(length_ - position_)) / (float)fade_, 1.f);
    }

  private:
    float sample_rate_ = 48000.f;
    int fade_ = 96, write_ = 0, filled_ = 0, start_ = 0, length_ = 1, position_ = 0, repeats_left_ = 0;
    float buffer_[2][kMax] = {};
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
git add engine/stutter.h sim/test_engine.cpp
git commit -m "Engine: stutter"
```

---

### Task 4: Codec warble

**Files:**
- Create: `engine/codec_warble.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "codec_warble.h"`, and above `main()`:
```cpp
static std::vector<float> RunCodec(bitch::CodecWarble &c, float amount, float seconds, float hz) {
    std::vector<float> out;
    for (int b = 0; b < (int)(seconds * kSampleRate / 48); b++) {
        c.Update(amount, 48);
        for (int k = 0; k < 48; k++) {
            float l = Sine(hz, 0.25f, b * 48 + k), r = l;
            c.Process(l, r);
            out.push_back(l);
        }
    }
    return out;
}

static void TestCodecWarble() {
    bitch::CodecWarble c;
    c.Init(kSampleRate, 13u);
    // In band: through, at a lower and wobbling level
    auto x = RunCodec(c, 1.f, 2.f, 1000.f);
    float level = Db(Rms(Window(x, 0.5f, 2.f)) / (0.25f / sqrtf(2.f)));
    CHECK(level > -15.f && level < 3.f, "codec warble: 1 kHz at %.1f dB", level);
    float mean = 0.f, var = 0.f;
    std::vector<float> w;
    for (float t = 0.5f; t + 0.01f <= 2.f; t += 0.01f) w.push_back(Rms(Window(x, t, t + 0.01f)));
    for (float v : w) mean += v / (float)w.size();
    for (float v : w) var += (v - mean) * (v - mean) / (float)w.size();
    CHECK(sqrtf(var) > 0.15f * mean, "codec warble: level hardly moves (%.4f +- %.4f)", mean, sqrtf(var));
    // Nothing above ~4 kHz
    c.Init(kSampleRate, 13u);
    auto high = RunCodec(c, 0.35f, 1.f, 7000.f);
    float top = Db(Rms(Window(high, 0.5f, 1.f)) / (0.25f / sqrtf(2.f)));
    CHECK(top < -20.f, "codec warble: 7 kHz only %.1f dB down", top);
    // Both sides share the band levels
    c.Init(kSampleRate, 13u);
    bool same = true;
    for (int b = 0; b < 200; b++) {
        c.Update(1.f, 48);
        for (int k = 0; k < 48; k++) {
            float l = Sine(800.f, 0.25f, b * 48 + k), r = l;
            c.Process(l, r);
            same = same && l == r;
        }
    }
    CHECK(same, "codec warble: the sides differ");
}
```
Call `TestCodecWarble();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: compile error, `'codec_warble.h' file not found`.

- [ ] **Step 3: Write the codec warble**

`engine/codec_warble.h`:
```cpp
#pragma once

#include <cmath>
#include <cstdint>

#include "effects.h"
#include "radio_filter.h"
#include "random.h"

namespace bitch {

// Codec warble: the watery sound of a low-bitrate stream. An 8-band filter bank (band-passes from
// 150 Hz to 3.5 kHz, about 0.65 octave apart, then a 4th-order Butterworth low-pass at 4 kHz for
// the top cut) whose
// bands are each re-levelled at random every 25-66 ms (15-40 times a second): to between
// 1 - depth and 1, or now and then gated almost off. Depth 0.5 (amount 0.35) to 1 (amount 1);
// the gating rises with the amount (10% to 35% of the changes). Level changes glide over 2 ms.
// Stereo: both sides share the band levels. Update() once per block, Process() per sample.
class CodecWarble {
  public:
    enum : int { kBands = 8 };

    void Init(float sample_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        glide_ = 1.f - expf(-1.f / (0.002f * sample_rate));
        random_.Seed(seed);
        for (int k = 0; k < kBands; k++) {
            float hz = 150.f * powf(3500.f / 150.f, (float)k / (float)(kBands - 1));
            float g = tanf(3.14159265359f * hz / sample_rate);
            for (Bandpass &b : band_[k]) b.SetCoefficients(g, 1.f / 2.2f);
        }
        float g = tanf(3.14159265359f * 4000.f / sample_rate);
        for (int side = 0; side < 2; side++) {
            top_[side][0].Set(g, 1.8478f); // 4th-order Butterworth: two stages, k = 1/Q
            top_[side][1].Set(g, 0.7654f);
        }
        Reset();
    }

    void Reset() {
        for (auto &side : top_)
            for (LowPass &s : side) s.Reset();
        for (int k = 0; k < kBands; k++) {
            for (Bandpass &b : band_[k]) b.Reset();
            gain_[k] = target_[k] = 1.f;
            left_[k] = 0;
        }
    }

    void Update(float amount, int block_size) {
        float u = EffectRange(amount);
        float depth = 0.5f + 0.5f * u, gate = 0.1f + 0.25f * u;
        for (int k = 0; k < kBands; k++) {
            left_[k] -= block_size;
            if (left_[k] > 0) continue;
            left_[k] = (int)((0.025f + 0.041f * random_.Uniform()) * sample_rate_);
            target_[k] = random_.Uniform() < gate ? 0.02f : 1.f - depth * random_.Uniform();
        }
    }

    void Process(float &l, float &r) {
        float out_l = 0.f, out_r = 0.f;
        for (int k = 0; k < kBands; k++) {
            gain_[k] += glide_ * (target_[k] - gain_[k]);
            out_l += gain_[k] * band_[k][0].Process(l);
            out_r += gain_[k] * band_[k][1].Process(r);
        }
        // the 8 unity-peak bands overlap: their sum is a little above unity
        l = 0.6f * top_[0][1].Process(top_[0][0].Process(out_l));
        r = 0.6f * top_[1][1].Process(top_[1][0].Process(out_r));
    }

  private:
    // One low-pass stage: Andrew Simper's state-variable filter (g = tan(pi fc / fs), k = 1 / Q)
    struct LowPass {
        float a1 = 0.f, a2 = 0.f, a3 = 0.f, ic1 = 0.f, ic2 = 0.f;
        void Set(float g, float k) {
            a1 = 1.f / (1.f + g * (g + k));
            a2 = g * a1;
            a3 = g * a2;
        }
        void Reset() { ic1 = ic2 = 0.f; }
        float Process(float x) {
            float v3 = x - ic2;
            float v1 = a1 * ic1 + a2 * v3;
            float v2 = ic2 + a2 * ic1 + a3 * v3;
            ic1 = 2.f * v1 - ic1;
            ic2 = 2.f * v2 - ic2;
            return v2;
        }
    };

    LowPass top_[2][2];
    Bandpass band_[kBands][2];
    float gain_[kBands] = {}, target_[kBands] = {};
    int left_[kBands] = {};
    float sample_rate_ = 48000.f, glide_ = 0.f;
    Random random_;
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`. If the in-band level check fails only because the bank's sum is louder or quieter than the 0.6 make-up assumes, report the measured level (the make-up can then be set from the measurement: 0.6 × 10^(−level/20)), don't loosen the range.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/codec_warble.h sim/test_engine.cpp
git commit -m "Engine: codec warble"
```

---

### Task 5: The damage pads in the engine

**Files:**
- Modify: `engine/engine.h`
- Modify: `engine/engine.cpp`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp`, add `bitch::Bit(bitch::kBitcrush) | bitch::Bit(bitch::kRateCrush) | bitch::Bit(bitch::kDropouts) | bitch::Bit(bitch::kStutter) | bitch::Bit(bitch::kCodecWarble) |` to `kBuiltEffects`, and above `main()`:
```cpp
// Each damage pad on and off (amount 0.6, a 220 Hz tone): the worst step in the 35 ms after each
// toggle, against the steady steps with the effect on (for the crushers their own staircase is
// the sound) and off
static float DamageNoClickWorstRatio(bitch::Effect pad) {
    const float kAmount = 0.6f;
    auto tone = [](int k) { return Sine(220.f, 0.25f, k); };
    auto max_step = [](const std::vector<float> &x, size_t from, size_t to) {
        float step = 0.f;
        for (size_t k = std::max<size_t>(from, 1); k < std::min(to, x.size()); k++)
            step = std::max(step, fabsf(x[k] - x[k - 1]));
        return step;
    };
    // The steady references: the pad held for 3 s (after the first 0.5 s), and the plain tone
    float steady = 0.f;
    for (uint32_t effects : {bitch::Bit(pad), 0u}) {
        bitch::Engine e;
        e.Init(kSampleRate);
        bitch::Params p;
        p.effect_amount = kAmount;
        p.effects = effects;
        std::vector<float> l, r;
        int n = 0;
        RunFor(e, p, 3.f, tone, n, l, r);
        steady = std::max(steady, max_step(l, (size_t)(0.5f * kSampleRate), l.size()));
    }
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.effect_amount = kAmount;
    std::vector<float> l, r;
    int n = 0;
    const int kToggles = 40;
    const size_t segment = (size_t)(0.1f * kSampleRate), fade = (size_t)(0.035f * kSampleRate);
    for (int i = 0; i < kToggles; i++) {
        p.effects ^= bitch::Bit(pad);
        RunFor(e, p, 0.1f, tone, n, l, r);
    }
    float worst = 0.f;
    for (int i = 1; i < kToggles; i++) {
        size_t at = i * segment;
        worst = std::max(worst, max_step(l, at, at + fade) / steady);
    }
    return worst;
}

static void TestEffectsDamageNoClick() {
    const bitch::Effect kPads[5] = {bitch::kCodecWarble, bitch::kRateCrush, bitch::kBitcrush, bitch::kStutter,
                                    bitch::kDropouts};
    for (bitch::Effect pad : kPads) {
        float worst = DamageNoClickWorstRatio(pad);
        printf("  %s on/off: worst step %.2fx the steady ones\n", bitch::EffectName(pad), worst);
        CHECK(worst < 2.5f, "%s on/off: a step %.2fx the steady ones", bitch::EffectName(pad), worst);
    }
}

// Runs an engine (no plate) on a 1 kHz tone with the given effects, returning the left output
static std::vector<float> RunDamage(uint32_t effects, float amount, float seconds, float hz = 1000.f) {
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.effects = effects;
    p.effect_amount = amount;
    std::vector<float> l, r;
    int n = 0;
    RunFor(e, p, seconds, [hz](int k) { return Sine(hz, 0.25f, k); }, n, l, r);
    return l;
}

static void TestEffectsDamage() {
    using bitch::Bit;
    // Bitcrush at 3 bits: a 0.25 tone lands on -0.25, 0 or +0.25 (through the soft clip; the
    // dry/wet mix's rounding moves it by a few ulps, hence the tolerance)
    auto w = Window(RunDamage(Bit(bitch::kBitcrush), 1.f, 1.f), 0.5f, 1.f);
    const float kGrid[3] = {bitch::SoftClip(-0.25f), 0.f, bitch::SoftClip(0.25f)};
    int off_grid = 0;
    for (float v : w) {
        float nearest = 1e9f;
        for (float g : kGrid) nearest = std::min(nearest, fabsf(v - g));
        off_grid += nearest > 1e-5f;
    }
    CHECK(off_grid == 0, "bitcrush pad: %d samples off the 3-bit grid", off_grid);
    // Sample-rate crush at 1.5 kHz: about 150 new values in 0.1 s (a held value moves only by
    // rounding, far below 1e-4)
    w = Window(RunDamage(Bit(bitch::kRateCrush), 1.f, 1.f), 0.5f, 0.6f);
    int changes = 0;
    for (size_t k = 1; k < w.size(); k++) changes += fabsf(w[k] - w[k - 1]) > 1e-4f;
    CHECK(changes >= 140 && changes <= 160, "rate crush pad: %d new values in 0.1 s (want ~150)", changes);
    // Dropouts: silent for a good part of the time
    auto x = RunDamage(Bit(bitch::kDropouts), 1.f, 5.f);
    int quiet = 0;
    for (float v : x) quiet += fabsf(v) < 1e-4f;
    CHECK(quiet > (int)(0.15f * x.size()), "dropouts pad: silent %.0f%% of the time",
          100.f * (float)quiet / (float)x.size());
    // Stutter: differs from the plain tone's output, stays finite
    auto plain = RunDamage(0u, 1.f, 4.f, 440.f), stutter = RunDamage(Bit(bitch::kStutter), 1.f, 4.f, 440.f);
    double diff = 0.0;
    bool finite = true;
    for (size_t k = 0; k < plain.size(); k++) {
        diff += (double)(plain[k] - stutter[k]) * (plain[k] - stutter[k]);
        finite = finite && std::isfinite(stutter[k]);
    }
    CHECK(finite && sqrt(diff / plain.size()) > 0.03, "stutter pad: no stutter (rms difference %.4f)",
          sqrt(diff / plain.size()));
    // Codec warble: nothing above ~4 kHz
    float off = Rms(Window(RunDamage(0u, 0.35f, 1.f, 7000.f), 0.5f, 1.f));
    float on = Rms(Window(RunDamage(Bit(bitch::kCodecWarble), 0.35f, 1.f, 7000.f), 0.5f, 1.f));
    CHECK(Db(on / off) < -20.0, "codec warble pad: 7 kHz only %.1f dB down", Db(on / off));
}
```
Add `#include <algorithm>` if it isn't already included (it is). Call `TestEffectsDamage();` and `TestEffectsDamageNoClick();` from `main()` (before the `All tests passed` line).

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: FAIL lines from `TestEffectsDamage` (the pads do nothing yet); `TestEffectsUnbuiltDoNothing` still passes (and `TestEffectsDamageNoClick`, with nothing happening yet, may too: it guards the wiring's ramps).

- [ ] **Step 3: Wire them in**

`engine/engine.h`:
- Add includes: `#include "crush.h"`, `#include "dropouts.h"`, `#include "stutter.h"`, `#include "codec_warble.h"`.
- Extend the class comment: "Page 3's damage pads (codec warble, sample-rate crush, bitcrush, stutter, dropouts) sit after the radio filter, before the AGC."
- Add private members after the radio members:
```cpp
    // Digital damage pads (page 3), after the radio filter and before the AGC
    CodecWarble codec_;
    RateCrush rate_crush_[2];
    Stutter stutter_;
    Dropouts dropouts_;
    // Their levels at the end of the last block (ramped across each block from there), in chain
    // order: codec warble, sample-rate crush, bitcrush, stutter, dropouts
    float damage_level_[5] = {};
```

`engine/engine.cpp`:
- At the top of the file (after `Clamp`):
```cpp
// The damage pads in chain order (damage_level_ follows it)
static const Effect kDamage[5] = {kCodecWarble, kRateCrush, kBitcrush, kStutter, kDropouts};
```
- In `Init`, after the radio `Init`s:
```cpp
    codec_.Init(sample_rate, 13u);
    for (RateCrush &r : rate_crush_) r.Init(sample_rate);
    stutter_.Init(sample_rate, 17u);
    dropouts_.Init(sample_rate, 19u);
```
- In `Reset()`: `codec_.Reset(); for (RateCrush &r : rate_crush_) r.Reset(); stutter_.Reset(); dropouts_.Reset(); for (float &v : damage_level_) v = 0.f;`.
- In the first-block snap, next to `scramble_level_ = ...; selcall_level_ = ...;`: `for (int k = 0; k < 5; k++) damage_level_[k] = level_[kDamage[k]];`.
- In `Process()`, after the other radio pads' per-block block, add:
```cpp
    // Digital damage pads: run only while audible (reset as each one starts), their levels ramped
    // across the block
    float damage_from[5], damage_to[5];
    bool damaging[5];
    for (int k = 0; k < 5; k++) {
        damage_from[k] = damage_level_[k];
        damage_to[k] = level_[kDamage[k]];
        damage_level_[k] = damage_to[k];
        damaging[k] = damage_from[k] > 0.f || damage_to[k] > 0.f;
    }
    const bool starting[5] = {damaging[0] && damage_from[0] == 0.f, damaging[1] && damage_from[1] == 0.f,
                              false, damaging[3] && damage_from[3] == 0.f, damaging[4] && damage_from[4] == 0.f};
    if (starting[0]) codec_.Reset();
    if (starting[1]) for (RateCrush &rc : rate_crush_) rc.Reset();
    if (starting[3]) stutter_.Reset();
    if (starting[4]) dropouts_.Reset();
    if (damaging[0]) codec_.Update(amount_radio, (int)size);
    if (damaging[1]) for (RateCrush &rc : rate_crush_) rc.Update(amount_radio);
    const float crush_step = CrushStep(CrushBits(amount_radio)), crush_inv_step = 1.f / crush_step;
    if (damaging[3]) stutter_.Update(amount_radio, (float)size / sample_rate_);
    if (damaging[4]) dropouts_.Update(amount_radio);
```
- In the per-sample loop, split the per-side loop at the radio filter, so the stereo damage can run between the two halves. Replace:
```cpp
        float y[2];
        for (int c = 0; c < 2; c++) {
```
with
```cpp
        float y[2], dry[2], filtered[2];
        for (int c = 0; c < 2; c++) {
```
  and inside that loop rename the local `float dry = ...` to `dry[c] = ...` (and its uses `drive_.Process(dry)` → `drive_.Process(dry[c])`), then replace its last two lines
```cpp
            float wet = wet_gain * agc_[c].Process(filter_[c].Process(received));
            y[c] = dry + mix_ * (wet - dry);
        }
```
  with
```cpp
            filtered[c] = filter_[c].Process(received);
        }
        // Digital damage (page 3), after the radio filter and before the AGC (so the AGC pumps on it)
        float d[5];
        for (int k = 0; k < 5; k++) d[k] = damaging[k] ? damage_from[k] + t * (damage_to[k] - damage_from[k]) : 0.f;
        if (damaging[0]) {
            float side_l = filtered[0], side_r = filtered[1];
            codec_.Process(side_l, side_r);
            filtered[0] += d[0] * (side_l - filtered[0]);
            filtered[1] += d[0] * (side_r - filtered[1]);
        }
        for (int c = 0; c < 2; c++) {
            if (damaging[1]) filtered[c] += d[1] * (rate_crush_[c].Process(filtered[c]) - filtered[c]);
            if (damaging[2]) filtered[c] += d[2] * (Bitcrush(filtered[c], crush_step, crush_inv_step) - filtered[c]);
        }
        if (damaging[3]) {
            float side_l = filtered[0], side_r = filtered[1];
            stutter_.Process(side_l, side_r);
            filtered[0] += d[3] * (side_l - filtered[0]);
            filtered[1] += d[3] * (side_r - filtered[1]);
        }
        if (damaging[4]) {
            float drop = 1.f - d[4] * (1.f - dropouts_.Gain());
            filtered[0] *= drop;
            filtered[1] *= drop;
        }
        for (int c = 0; c < 2; c++) {
            float wet = wet_gain * agc_[c].Process(filtered[c]);
            y[c] = dry[c] + mix_ * (wet - dry[c]);
        }
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`, including `TestEffectsUnbuiltDoNothing` (bit-exact: with the pads off nothing in the damage block runs, and the split loop does the same arithmetic in the same order) and every earlier test.

- [ ] **Step 5: Check the firmware builds, and commit**

Run: `make -C ~/local_files/daisy/bit-chifter` (touch `engine/engine.cpp` first) — expected: no errors or warnings (engine.cpp is also clean with `-Wshadow`); DTCMRAM about 83 KB of 128 KB. Report the DTCMRAM/SRAM lines.
```bash
cd ~/local_files/daisy/bit-chifter
git add engine/engine.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: digital damage pads (codec warble, rate crush, bitcrush, stutter, dropouts)"
```

(Review checkpoint: the engine tasks are done; review before Task 6.)

---

### Task 6: Chaos

**Files:**
- Modify: `ui/pad_pages.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing test**

In `sim/test_engine.cpp`, at the end of `TestPadPages()` (before its closing brace), add:
```cpp
    // Chaos (page 3, bottom right): while held, 2-3 other effects (not Fade-out or Squelch, which
    // would just silence things); a new set at each touch; off at the release
    uint32_t sets[6];
    for (int touch = 0; touch < 6; touch++) {
        step(37 + 11 * touch, 1u << 9, 2);
        uint32_t fx = pp.Effects() & ~Bit(kChaos);
        int count = 0;
        for (int k = 0; k < kEffectCount; k++) count += (fx >> k) & 1u;
        CHECK(count >= 2 && count <= 3, "chaos: %d effects (%08x)", count, fx);
        CHECK(!(fx & (Bit(kFadeOut) | Bit(kSquelch))), "chaos picked fade-out or squelch (%08x)", fx);
        sets[touch] = fx;
        step(50, 0, 2);
        CHECK(pp.Effects() == 0, "chaos: still on after release (%08x)", pp.Effects());
    }
    int distinct = 0;
    for (int a = 0; a < 6; a++) {
        bool seen = false;
        for (int b = 0; b < a; b++) seen = seen || sets[b] == sets[a];
        distinct += !seen;
    }
    CHECK(distinct >= 4, "chaos: only %d different sets in 6 touches", distinct);
```

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: FAIL lines `chaos: 0 effects`.

- [ ] **Step 3: Add Chaos to the pad logic**

In `ui/pad_pages.h`:
- Extend the class comment: "- Chaos (page 3, bottom right): while its pad is held, a random set of 2-3 other effects is on (not Fade-out or Squelch, which would only silence things); each touch rolls a new set."
- In `Update`, replace
```cpp
            if (on && !was) {
                effect_[ch] = PadEffect(page, ch);
                if (effect_[ch] >= 0) held_ |= 1u << effect_[ch];
            } else if (!on && was && effect_[ch] >= 0) {
                held_ &= ~(1u << effect_[ch]);
                effect_[ch] = -1;
            }
```
with
```cpp
            if (on && !was) {
                effect_[ch] = PadEffect(page, ch);
                if (effect_[ch] >= 0) held_ |= 1u << effect_[ch];
                if (effect_[ch] == kChaos) chaos_ = RollChaos(now_ms);
            } else if (!on && was && effect_[ch] >= 0) {
                held_ &= ~(1u << effect_[ch]);
                if (effect_[ch] == kChaos) chaos_ = 0;
                effect_[ch] = -1;
            }
```
- Change `Effects()` to `uint32_t Effects() const { return held_ | chaos_; }`.
- Add private:
```cpp
    // 2-3 random effects other than Chaos, Fade-out and Squelch (the touch time stirs the generator)
    uint32_t RollChaos(uint32_t now_ms) {
        random_ ^= now_ms * 2654435761u;
        uint32_t set = 0;
        int count = 2 + (int)(NextRandom() % 2u);
        while (count > 0) {
            int e = (int)(NextRandom() % (uint32_t)kEffectCount);
            if (e == kChaos || e == kFadeOut || e == kSquelch || ((set >> e) & 1u)) continue;
            set |= 1u << e;
            count--;
        }
        return set;
    }
    uint32_t NextRandom() { // xorshift32
        random_ ^= random_ << 13;
        random_ ^= random_ >> 17;
        random_ ^= random_ << 5;
        return random_;
    }
    uint32_t chaos_ = 0, random_ = 0x9E3779B9u;
```
  (If `random_` could become 0 after the xor with the time, guard: `if (!random_) random_ = 1u;` right after it.)

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add ui/pad_pages.h sim/test_engine.cpp
git commit -m "Pad logic: chaos (2-3 random effects while held)"
```

---

### Task 7: Renders and listening

**Files:** none (renders only)

- [ ] **Step 1: Render**

```bash
cd ~/local_files/daisy/bit-chifter/sim/out
../render.sh voice.wav d5_bitcrush.wav effects=bitcrush amount=0.8 every=2
../render.sh voice.wav d5_rate_crush.wav effects=rate_crush amount=0.8 every=2
../render.sh voice.wav d5_dropouts.wav effects=dropouts amount=0.7
../render.sh voice.wav d5_stutter.wav effects=stutter amount=0.8
../render.sh voice.wav d5_codec.wav effects=codec_warble amount=0.8
../render.sh drone.wav d5_drone_stutter_crush.wav tune=-80 fb=0.5 effects=stutter,rate_crush amount=0.7 reverb=0.5
../render.sh voice.wav d5_broken_stream.wav reception=0.4 filter=1500 q=1.5 open=0 effects=codec_warble,dropouts,bitcrush amount=0.6
```
Expected: seven `Wrote …` lines, no compiler warnings.

- [ ] **Step 2: User listens**

Send the seven renders, saying what each is (bitcrush and rate crush toggling every 2 s; dropouts; stutter; codec warble; the shifted drone with stutter and crush into reverb; a "broken stream" mix). Chaos is pad logic: it's heard on the hardware.

---

### Task 8: Firmware

**Files:**
- Modify: `simple_touch.cpp`
- Modify: `README.md`

- [ ] **Step 1: Update the comment and README**

No code changes (the pads already send the effect bits; Chaos is in `ui/pad_pages.h`). In `simple_touch.cpp`'s top comment, add "the damage pads: bitcrush, sample-rate crush, dropouts, stutter, codec warble, and chaos (2-3 random effects while held)" to the built list. In `README.md`'s Pads row, add: "page 3 big left bitcrush, big right stutter, left edge sample-rate crush, right edge dropouts, centre strip codec warble, bottom right chaos". If every pad is now built except page 1's echoes, spirals, chirp and warble, say so.

- [ ] **Step 2: Build**

Run: `make -C ~/local_files/daisy/bit-chifter`
Expected: memory table, no errors or warnings.

- [ ] **Step 3: User flashes and plays it**

User runs `make program-dfu` (RESET, then BOOT while the LED pulses). Switch 1 right (page 3), hold each pad: big left bitcrush, big right stutter, left edge sample-rate crush, right edge dropouts, centre strip codec warble, bottom right chaos (each touch a new random mix). RECEPTION makes them harsher. Check the CPU in the USB report holding several damage and radio pads.

- [ ] **Step 4: Commit (after the user confirms)**

```bash
cd ~/local_files/daisy/bit-chifter
git add simple_touch.cpp README.md
git commit -m "Simple Touch firmware: the damage pads and chaos (page 3)"
```
