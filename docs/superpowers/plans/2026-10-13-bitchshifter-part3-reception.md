# Bitchshifter Part 3 (Reception) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fader 2 RECEPTION takes the sound from a clean signal (bottom) to "lost in the noise" (top): each side's shift drifts on its own random path, the signal fades and combs (multipath), static and crackles rise, the AGC pumps, and the input is driven into saturation.

**Architecture:** Each receiver effect is a small header-only class with its own amount and Mac tests: `Wander` (smoothed random walk, `random.h`), `Drive`, `Fading`, `StaticNoise`, `Agc`. The engine owns one of each per side (Drive is shared) and drives them all from a single smoothed `reception` amount. Per side the wet chain becomes: drive → ring mod → shifter (+ drift, with feedback) → fading → + static → radio filter → AGC. The filter shapes both signal and static, and the AGC boosts the static in quiet passages. At reception 0 every stage is an exact pass-through, so all part 1/2 behaviour and tests are unchanged.

**Tech Stack:** C++17 (the firmware compiles the engine as gnu++14), libDaisy v8.1, Apple clang for `sim/`.

**Spec:** `docs/superpowers/specs/2026-10-12-bitchshifter-design.md` (Faders → 2 RECEPTION; Signal chain items 1 Drive and 5 Reception; the stereo paragraph's Drift).

**Conventions:**
- Project root `~/local_files/daisy/bitchshifter`, local git, master. No `Co-Authored-By` lines.
- Tests: `sim/run_tests.sh` (prints `All tests passed`). Helpers already in `sim/test_engine.cpp`: `CHECK(cond, fmt...)`, `kSampleRate` (48000.f), `kPi`, `Magnitude(vector, freq)` (Goertzel amplitude), `Db(ratio)`, `RunEngine(params, amplitude, &right)` (1 kHz sine into both inputs, returns the last 0.5 s of OUT L), `RunEngineStereo(params, seconds, freq_l, amp_l, freq_r, amp_r, out_l, out_r)` (full outputs), `Window(x, from_s, to_s)`.
- Engine code: single precision only (`f` literals, float functions), nothing allocating in `Process()`. Real-time cost matters (Process runs in the audio interrupt): per-sample work stays cheap; random targets and gains update per block.
- At amount 0, each new stage must return its input **exactly** (so the existing tests stay valid).
- If a test fails after implementing, don't change tolerances or the given algorithm to make it pass: report the printed numbers.
- Hardware steps: the user flashes (RESET, BOOT while the LED pulses, `make program-dfu`) and confirms before the firmware commit.

## File structure

| File | Responsibility |
|---|---|
| `engine/random.h` | `Random` (xorshift32) and `Wander` (smoothed random walk, -1..1, updated per block) |
| `engine/drive.h` | `Drive`: soft-clip saturation, 0 to +24 dB, level-compensated |
| `engine/fading.h` | `Fading`: slow random level dips (to -18 dB) and a moving 0.5–3 ms comb (multipath) |
| `engine/static_noise.h` | `StaticNoise`: hiss plus random crackles |
| `engine/agc.h` | `Agc`: feed-forward gain control, up to +18 dB boost / -12 dB cut |
| `engine/mapping.h` | add `ReceptionAmount(fader)` |
| `engine/engine.h`, `engine/engine.cpp` | `reception` param, per-side stages, drift, phase-lock scaling |
| `sim/test_engine.cpp` | tests per stage and for the chain |
| `sim/render.cpp` | `reception=` option |
| `simple_touch.cpp`, `README.md` | fader 2 |

(The spec's code-structure section lists one `reception.h`; splitting it into one file per effect keeps each small and separately testable.)

---

### Task 1: Random and Wander

**Files:**
- Create: `engine/random.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "random.h"` with the other engine includes, and above `main()`:
```cpp
// Wander stays in -1..1, covers most of the range, moves smoothly, and depends on its seed
static void TestWander() {
    bitch::Wander w, other;
    w.Init(1000.f, 5);
    other.Init(1000.f, 6);
    float lo = 1.f, hi = -1.f, prev = 0.f, max_step = 0.f;
    bool bounded = true, differs = false;
    for (int b = 0; b < 100000; b++) { // 100 s of blocks at 1 kHz
        float v = w.Next(0.5f);
        bounded = bounded && v >= -1.f && v <= 1.f;
        lo = std::min(lo, v);
        hi = std::max(hi, v);
        if (b > 0) max_step = std::max(max_step, fabsf(v - prev));
        prev = v;
        if (fabsf(v - other.Next(0.5f)) > 0.1f) differs = true;
    }
    CHECK(bounded, "wander: left -1..1");
    CHECK(hi - lo > 1.f, "wander: only covered %.2f..%.2f", lo, hi);
    CHECK(max_step < 0.01f, "wander: jumped %.4f in one block", max_step);
    CHECK(differs, "wander: seeds 5 and 6 gave the same path");
}
```
Call `TestWander();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile error, `'random.h' file not found`.

- [ ] **Step 3: Write random.h**

`engine/random.h`:
```cpp
#pragma once

#include <cmath>
#include <cstdint>

namespace bitch {

// Small, fast, deterministic pseudo-random generator (xorshift32), real-time safe
class Random {
  public:
    explicit Random(uint32_t seed = 1) { Seed(seed); }

    void Seed(uint32_t seed) { state_ = seed ? seed : 1u; }

    uint32_t Next() {
        state_ ^= state_ << 13;
        state_ ^= state_ >> 17;
        state_ ^= state_ << 5;
        return state_;
    }

    float Uniform() { return (float)(Next() >> 8) * (1.f / 16777216.f); } // 0 to 1
    float Bipolar() { return 2.f * Uniform() - 1.f; }                     // -1 to 1

  private:
    uint32_t state_ = 1u;
};

// Smoothed random walk in -1..1, for drift and fading: every 0.5-1.5 periods (a period
// is 1 / rate_hz) it picks a new random target and glides towards it. Call Next() once
// per audio block.
class Wander {
  public:
    void Init(float block_rate, uint32_t seed) {
        block_rate_ = block_rate;
        random_.Seed(seed);
        value_ = target_ = 0.f;
        countdown_ = 0;
    }

    float Next(float rate_hz) {
        if (--countdown_ <= 0) {
            target_ = random_.Bipolar();
            countdown_ = 1 + (int)(block_rate_ / rate_hz * (0.5f + random_.Uniform()));
        }
        value_ += (1.f - expf(-2.f * rate_hz / block_rate_)) * (target_ - value_);
        return value_;
    }

  private:
    Random random_;
    float block_rate_ = 1000.f;
    float value_ = 0.f, target_ = 0.f;
    int countdown_ = 0;
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/random.h sim/test_engine.cpp
git commit -m "Engine: random generator and smoothed random walk"
```

---

### Task 2: Drive

**Files:**
- Create: `engine/drive.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "drive.h"`, and above `main()`:
```cpp
// Drives a 1 kHz sine (amplitude 0.25); returns the last 0.5 s
static std::vector<float> DriveSine(float amount) {
    bitch::Drive d;
    d.Set(amount);
    std::vector<float> out;
    int n = (int)kSampleRate;
    for (int t = 0; t < n; t++) {
        float x = 0.25f * sinf(2 * kPi * 1000.f * t / kSampleRate);
        float y = d.Process(x);
        if (t >= n / 2) out.push_back(y);
    }
    return out;
}

static void TestDrive() {
    bitch::Drive d;
    d.Set(0.f);
    bool identical = true;
    for (int t = 0; t < 1000; t++) {
        float x = 0.25f * sinf(0.1f * t);
        identical = identical && d.Process(x) == x;
    }
    CHECK(identical, "drive 0: not a pass-through");
    auto clean = DriveSine(0.f), fuzz = DriveSine(1.f);
    CHECK(Magnitude(clean, 3000.f) < 0.001, "drive 0: 3rd harmonic %.4f", Magnitude(clean, 3000.f));
    CHECK(Magnitude(fuzz, 3000.f) > 0.05, "drive 1: 3rd harmonic only %.4f", Magnitude(fuzz, 3000.f));
    float peak = 0.f;
    for (float v : fuzz) peak = std::max(peak, fabsf(v));
    CHECK(peak < 0.3f, "drive 1: level not compensated, peak %.3f", peak);
}
```
Call `TestDrive();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile error, `'drive.h' file not found`.

- [ ] **Step 3: Write the drive**

`engine/drive.h`:
```cpp
#pragma once

#include <cmath>

#include "softclip.h"

namespace bitch {

// Input saturation: up to +24 dB into the soft clip (fuzz at the top), scaled back so a
// nominal -12 dBFS signal comes out at about the same level. amount 0 is an exact
// pass-through. Set() once per block; Process() per sample.
class Drive {
  public:
    void Set(float amount) {
        bypass_ = !(amount > 0.f);
        float a = bypass_ ? 0.f : fminf(amount, 1.f);
        gain_ = exp2f(a * a * 3.986f); // 2^3.986 = +24 dB at amount 1; gentle low down
        makeup_ = 0.25f / SoftClip(0.25f * gain_);
    }

    float Process(float x) const { return bypass_ ? x : SoftClip(x * gain_) * makeup_; }

  private:
    bool bypass_ = true;
    float gain_ = 1.f, makeup_ = 1.f;
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/drive.h sim/test_engine.cpp
git commit -m "Engine: level-compensated drive"
```

---

### Task 3: Fading (multipath)

**Files:**
- Create: `engine/fading.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "fading.h"`, and above `main()`:
```cpp
static void TestFading() {
    const float block_rate = kSampleRate / 48.f;
    bitch::Fading off;
    off.Init(kSampleRate, block_rate, 101);
    bool identical = true;
    for (int b = 0; b < 1000; b++) {
        off.Update(0.f);
        for (int k = 0; k < 48; k++) {
            float x = 0.25f * sinf(0.13f * (b * 48 + k));
            identical = identical && off.Process(x) == x;
        }
    }
    CHECK(identical, "fading 0: not a pass-through");

    // Full depth on a 1 kHz sine for 30 s: 50 ms windows must dip well below and swell
    // close to the input level, and stay bounded
    bitch::Fading f;
    f.Init(kSampleRate, block_rate, 101);
    float lo_db = 0.f, hi_db = -100.f, peak = 0.f;
    bool finite = true;
    double sum = 0;
    int count = 0;
    for (int b = 0; b < (int)(30 * block_rate); b++) {
        f.Update(1.f);
        for (int k = 0; k < 48; k++) {
            float y = f.Process(0.25f * sinf(2 * kPi * 1000.f * (b * 48 + k) / kSampleRate));
            finite = finite && std::isfinite(y);
            peak = std::max(peak, fabsf(y));
            sum += y * y;
            count++;
        }
        if (count >= 2400) { // 50 ms
            float db = 20.f * log10f(sqrtf((float)(sum / count)) / (0.25f * 0.7071f) + 1e-9f);
            if (b > block_rate) { // after the first second
                lo_db = std::min(lo_db, db);
                hi_db = std::max(hi_db, db);
            }
            sum = 0;
            count = 0;
        }
    }
    CHECK(finite, "fading 1: non-finite output");
    CHECK(lo_db < -10.f, "fading 1: deepest dip only %.1f dB", lo_db);
    CHECK(hi_db > -4.f, "fading 1: never swells back, best %.1f dB", hi_db);
    CHECK(peak < 0.45f, "fading 1: peak %.3f", peak); // gain <= 1, echo comb up to +4 dB: 0.25 * 1.6 = 0.4
}
```
Call `TestFading();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile error, `'fading.h' file not found`.

- [ ] **Step 3: Write the fading**

`engine/fading.h`:
```cpp
#pragma once

#include <cmath>
#include <cstdint>

#include "random.h"

namespace bitch {

// Multipath fading, like a distant station: the level swells and dips (down to -18 dB)
// and a short echo (0.5-3 ms, subtracted) combs the sound, both on slow random paths
// (0.05-0.5 Hz, faster as depth rises). depth 0 is an exact pass-through.
// Update() once per block, Process() per sample.
class Fading {
  public:
    static constexpr int kSize = 256; // > 3 ms at 48 kHz, power of two
    static constexpr int kMask = kSize - 1;

    void Init(float sample_rate, float block_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        level_path_.Init(block_rate, seed);
        delay_path_.Init(block_rate, seed * 7u + 3u);
        glide_ = 1.f - expf(-1.f / (0.01f * sample_rate)); // ~10 ms
        Reset();
    }

    void Reset() {
        for (float &v : buffer_) v = 0.f;
        position_ = 0;
        gain_ = gain_target_ = 1.f;
        echo_ = echo_target_ = 0.f;
        delay_ = delay_target_ = 0.001f * sample_rate_;
    }

    void Update(float depth) {
        float d = fminf(fmaxf(depth, 0.f), 1.f);
        float rate = 0.05f + 0.45f * d;
        float level = level_path_.Next(rate); // -1..1
        float path = delay_path_.Next(1.7f * rate);
        gain_target_ = 1.f - d * 0.874f * (0.5f + 0.5f * level); // 0.126 = -18 dB at the deepest
        delay_target_ = (0.5f + 1.25f * (1.f + path)) * 0.001f * sample_rate_; // 0.5-3 ms
        echo_target_ = -0.6f * d;
    }

    float Process(float x) {
        gain_ += glide_ * (gain_target_ - gain_);
        echo_ += glide_ * (echo_target_ - echo_);
        delay_ += glide_ * (delay_target_ - delay_);
        buffer_[position_] = x;
        float read = (float)position_ - delay_;
        if (read < 0.f) read += (float)kSize;
        int i0 = (int)read;
        float frac = read - (float)i0;
        float a = buffer_[i0 & kMask], b = buffer_[(i0 + 1) & kMask];
        position_ = (position_ + 1) & kMask;
        return gain_ * (x + echo_ * (a + frac * (b - a)));
    }

  private:
    Wander level_path_, delay_path_;
    float buffer_[kSize] = {};
    int position_ = 0;
    float sample_rate_ = 48000.f, glide_ = 0.f;
    float gain_ = 1.f, gain_target_ = 1.f, echo_ = 0.f, echo_target_ = 0.f, delay_ = 48.f, delay_target_ = 48.f;
};

} // namespace bitch
```
Also add to `engine/engine.cpp`, next to the existing `constexpr float Hilbert::kA[4];` definitions (the firmware builds as C++14):
```cpp
constexpr int Fading::kSize;
constexpr int Fading::kMask;
```
and `#include "fading.h"` at the top of `engine/engine.cpp` if `engine.h` doesn't include it yet (it will after Task 7).

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/fading.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: multipath fading"
```

---

### Task 4: Static

**Files:**
- Create: `engine/static_noise.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "static_noise.h"`, and above `main()`:
```cpp
static void TestStaticNoise() {
    bitch::StaticNoise off;
    off.Init(kSampleRate, 7);
    off.Update(0.f);
    bool silent = true;
    for (int t = 0; t < (int)kSampleRate; t++) silent = silent && off.Process() == 0.f;
    CHECK(silent, "static 0: not silent");

    // Full level for 10 s: audible hiss, a few hundred crackles, bounded
    bitch::StaticNoise s;
    s.Init(kSampleRate, 7);
    s.Update(1.f);
    double sum = 0;
    float peak = 0.f;
    // A crackle = above 0.15 at least 5 ms after the last one (a decaying crackle riding on
    // hiss crosses 0.15 several times; hiss alone peaks at 0.12)
    int crackles = 0, since_last = 1 << 30;
    int n = 10 * (int)kSampleRate;
    for (int t = 0; t < n; t++) {
        float v = s.Process();
        sum += v * v;
        peak = std::max(peak, fabsf(v));
        since_last++;
        if (fabsf(v) > 0.15f && since_last >= 240) {
            crackles++;
            since_last = 0;
        }
    }
    float rms = sqrtf((float)(sum / n));
    CHECK(rms > 0.01f && rms < 0.15f, "static 1: rms %.4f", rms);
    CHECK(crackles > 30 && crackles < 1000, "static 1: %d crackles in 10 s", crackles);
    CHECK(peak < 0.6f, "static 1: peak %.3f", peak);
}
```
Call `TestStaticNoise();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile error, `'static_noise.h' file not found`.

- [ ] **Step 3: Write the static**

`engine/static_noise.h`:
```cpp
#pragma once

#include <cmath>
#include <cstdint>

#include "random.h"

namespace bitch {

// Receiver static: hiss (white noise, gently low-passed at 6 kHz) plus random crackles
// (sparse pops of random size and sign through a 2 ms decay, 2-30 per second as the
// level rises). level 0 is exactly silent. Update() once per block, Process() per sample.
class StaticNoise {
  public:
    void Init(float sample_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        random_.Seed(seed);
        hiss_coefficient_ = 1.f - expf(-6.28318530718f * 6000.f / sample_rate);
        crackle_decay_ = expf(-1.f / (0.002f * sample_rate));
        hiss_state_ = crackle_ = 0.f;
        Update(0.f);
    }

    void Update(float level) {
        float l = fminf(fmaxf(level, 0.f), 1.f);
        hiss_level_ = 0.12f * l;
        crackle_chance_ = l > 0.f ? (2.f + 28.f * l) / sample_rate_ : 0.f;
        crackle_level_ = 0.4f * l;
    }

    float Process() {
        if (hiss_level_ == 0.f && crackle_ == 0.f) return 0.f;
        hiss_state_ += hiss_coefficient_ * (random_.Bipolar() - hiss_state_);
        if (random_.Uniform() < crackle_chance_) {
            float size = crackle_level_ * (0.3f + 0.7f * random_.Uniform());
            crackle_ = random_.Uniform() < 0.5f ? -size : size;
        }
        float out = hiss_level_ * hiss_state_ + crackle_;
        crackle_ *= crackle_decay_;
        if (fabsf(crackle_) < 1e-6f) crackle_ = 0.f;
        return out;
    }

  private:
    Random random_;
    float sample_rate_ = 48000.f;
    float hiss_coefficient_ = 0.f, hiss_state_ = 0.f, hiss_level_ = 0.f;
    float crackle_chance_ = 0.f, crackle_level_ = 0.f, crackle_decay_ = 0.f, crackle_ = 0.f;
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed`. If the RMS or crackle count lands outside its range, report the numbers (the levels are a voicing choice to settle together).

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/static_noise.h sim/test_engine.cpp
git commit -m "Engine: static (hiss and crackles)"
```

---

### Task 5: AGC

**Files:**
- Create: `engine/agc.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "agc.h"`, and above `main()`:
```cpp
// Runs the AGC on a 1 kHz sine for 2 s at the given strength; returns the last 0.5 s peak
static float AgcPeak(float amplitude, float strength) {
    bitch::Agc a;
    a.Init(kSampleRate);
    float peak = 0.f;
    for (int b = 0; b < (int)(2 * kSampleRate / 48); b++) {
        a.Update(strength);
        for (int k = 0; k < 48; k++) {
            int t = b * 48 + k;
            float y = a.Process(amplitude * sinf(2 * kPi * 1000.f * t / kSampleRate));
            if (t >= (int)(1.5f * kSampleRate)) peak = std::max(peak, fabsf(y));
        }
    }
    return peak;
}

static void TestAgc() {
    bitch::Agc off;
    off.Init(kSampleRate);
    bool identical = true;
    for (int b = 0; b < 100; b++) {
        off.Update(0.f);
        for (int k = 0; k < 48; k++) {
            float x = 0.05f * sinf(0.2f * (b * 48 + k));
            identical = identical && off.Process(x) == x;
        }
    }
    CHECK(identical, "agc 0: not a pass-through");
    // Strength 1 aims for a 0.3 peak: within reach it gets there, beyond it hits the limits
    float mid = AgcPeak(0.1f, 1.f), loud = AgcPeak(0.9f, 1.f), quiet = AgcPeak(0.02f, 1.f);
    CHECK(fabsf(Db(mid / 0.3)) < 1.5, "agc: 0.1 in -> peak %.3f (want 0.3)", mid);
    CHECK(fabsf(Db(loud / 0.3)) < 1.5, "agc: 0.9 in -> peak %.3f (want 0.3)", loud);
    CHECK(fabsf(Db(quiet / (0.02 * 7.94))) < 1.5, "agc: 0.02 in -> peak %.3f (want the +18 dB limit, 0.159)", quiet);
}
```
Call `TestAgc();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile error, `'agc.h' file not found`.

- [ ] **Step 3: Write the AGC**

`engine/agc.h`:
```cpp
#pragma once

#include <cmath>

namespace bitch {

// Receiver-style automatic gain control: follows the input's peak level (attack 5 ms,
// release 300 ms) and steers the gain towards a 0.3 peak. At strength 1 it can boost by
// up to +18 dB and cut by up to -12 dB; strength 0 is an exact pass-through. In quiet
// passages the gain rises, so static swells up in the gaps. Update() once per block
// (sets the gain target from the level so far), Process() per sample.
class Agc {
  public:
    static constexpr float kTarget = 0.3f;

    void Init(float sample_rate) {
        attack_ = 1.f - expf(-1.f / (0.005f * sample_rate));
        release_ = 1.f - expf(-1.f / (0.3f * sample_rate));
        glide_ = 1.f - expf(-1.f / (0.02f * sample_rate));
        Reset();
    }

    void Reset() {
        envelope_ = 0.f;
        gain_ = gain_target_ = 1.f;
        bypass_ = true;
    }

    void Update(float strength) {
        float s = fminf(fmaxf(strength, 0.f), 1.f);
        bypass_ = !(s > 0.f) && gain_ == 1.f;
        if (!(s > 0.f)) {
            gain_target_ = 1.f;
            return;
        }
        float wanted = kTarget / fmaxf(envelope_, 1e-4f);
        float most = exp2f(2.99f * s);    // +18 dB at strength 1
        float least = exp2f(-1.993f * s); // -12 dB at strength 1
        gain_target_ = fminf(fmaxf(wanted, least), most);
    }

    float Process(float x) {
        float level = fabsf(x);
        envelope_ += (level > envelope_ ? attack_ : release_) * (level - envelope_);
        if (bypass_) return x;
        gain_ += glide_ * (gain_target_ - gain_);
        if (gain_target_ == 1.f && fabsf(gain_ - 1.f) < 1e-5f) gain_ = 1.f;
        return x * gain_;
    }

  private:
    float attack_ = 0.f, release_ = 0.f, glide_ = 0.f;
    float envelope_ = 0.f, gain_ = 1.f, gain_target_ = 1.f;
    bool bypass_ = true;
};

} // namespace bitch
```
Also add `constexpr float Agc::kTarget;` next to the other out-of-class definitions in `engine/engine.cpp`, with `#include "agc.h"` there if needed.

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/agc.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: receiver-style AGC"
```

---

### Task 6: RECEPTION mapping

**Files:**
- Modify: `engine/mapping.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing test**

In `TestMappings()` in `sim/test_engine.cpp`, append:
```cpp
    CHECK(bitch::ReceptionAmount(0.f) == 0.f, "reception bottom %.3f", bitch::ReceptionAmount(0.f));
    CHECK(bitch::ReceptionAmount(0.02f) == 0.f, "reception bottom dead zone %.3f", bitch::ReceptionAmount(0.02f));
    CHECK(bitch::ReceptionAmount(0.98f) == 1.f, "reception top %.3f", bitch::ReceptionAmount(0.98f));
    CHECK(fabsf(bitch::ReceptionAmount(0.5f) - 0.5f) < 0.01f, "reception middle %.3f", bitch::ReceptionAmount(0.5f));
```

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile error, `no member named 'ReceptionAmount' in namespace 'bitch'`.

- [ ] **Step 3: Add the mapping**

In `engine/mapping.h`, before the closing `} // namespace bitch`:
```cpp
// Fader -> reception amount 0-1, with the bottom 3% truly clean and the top 3% full
// (a fader rarely reads exactly 0 or 1)
inline float ReceptionAmount(float fader) { return fminf(fmaxf((fader - 0.03f) / 0.94f, 0.f), 1.f); }
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/mapping.h sim/test_engine.cpp
git commit -m "Engine: RECEPTION fader mapping"
```

---

### Task 7: Reception in the engine

**Files:**
- Modify: `engine/engine.h`
- Modify: `engine/engine.cpp`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp`, above `main()`:
```cpp
// Largest |L - R| over the last second of a run
static float SideDifference(const std::vector<float> &l, const std::vector<float> &r) {
    float d = 0.f;
    for (size_t i = l.size() - (size_t)kSampleRate; i < l.size(); i++) d = std::max(d, fabsf(l[i] - r[i]));
    return d;
}

// At reception 0 a mono-ish source stays identical on both sides; with reception up,
// the sides drift (and fade, and get their own static) apart
static void TestEngineReceptionSeparatesSides() {
    bitch::Params p;
    p.tune_hz = 100.f;
    std::vector<float> l, r;
    RunEngineStereo(p, 3.f, 1000.f, 0.25f, 1000.f, 0.25f, l, r);
    CHECK(SideDifference(l, r) < 1e-6f, "reception 0: sides differ by %.6f", SideDifference(l, r));
    p.reception = 0.5f;
    RunEngineStereo(p, 3.f, 1000.f, 0.25f, 1000.f, 0.25f, l, r);
    CHECK(SideDifference(l, r) > 0.05f, "reception 0.5: sides only differ by %.4f", SideDifference(l, r));
}

// Silence in: reception 0 gives silence out; full reception gives audible static
static void TestEngineReceptionStatic() {
    bitch::Params p;
    std::vector<float> l, r;
    RunEngineStereo(p, 2.f, 1000.f, 0.f, 1000.f, 0.f, l, r);
    float peak = 0.f;
    for (float v : l) peak = std::max(peak, fabsf(v));
    CHECK(peak < 1e-7f, "reception 0, silence: peak %.8f", peak);
    p.reception = 1.f;
    RunEngineStereo(p, 2.f, 1000.f, 0.f, 1000.f, 0.f, l, r);
    double sum = 0;
    peak = 0.f;
    for (size_t i = l.size() / 2; i < l.size(); i++) {
        sum += l[i] * l[i];
        peak = std::max(peak, fabsf(l[i]));
    }
    float rms = sqrtf((float)(sum / (l.size() / 2)));
    CHECK(rms > 0.005f, "reception 1, silence: static rms only %.4f", rms);
    CHECK(peak <= 1.f, "reception 1, silence: peak %.3f", peak);
}

// A quiet input comes out louder with full reception (AGC boost, plus static)
static void TestEngineReceptionLiftsQuiet() {
    bitch::Params p;
    std::vector<float> l, r;
    auto rms = [](const std::vector<float> &x) {
        double s = 0;
        for (size_t i = x.size() / 2; i < x.size(); i++) s += x[i] * x[i];
        return sqrt(s / (x.size() / 2));
    };
    RunEngineStereo(p, 2.f, 1000.f, 0.02f, 1000.f, 0.02f, l, r);
    double clean = rms(l);
    p.reception = 1.f;
    RunEngineStereo(p, 2.f, 1000.f, 0.02f, 1000.f, 0.02f, l, r);
    CHECK(Db(rms(l) / clean) > 6, "reception 1: quiet input only %.1f dB louder", Db(rms(l) / clean));
}

// Everything at once, on noise, for 20 s: finite and within the limiter
static void TestEngineReceptionStable() {
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.reception = 1.f;
    p.feedback = 0.95f;
    p.ring = 1.f;
    p.carrier_hz = 300.f;
    p.filter_q = 12.f;
    p.filter_open = 0.f;
    p.filter_hz = 1200.f;
    p.tune_hz = 37.f;
    p.stereo_mode = bitch::StereoMode::kMirror;
    uint32_t seed = 9;
    float in[48], out_l[48], out_r[48], peak = 0.f;
    bool finite = true;
    for (int b = 0; b < (int)(20 * kSampleRate / 48); b++) {
        for (float &v : in) {
            seed = seed * 1664525u + 1013904223u;
            v = ((seed >> 8) / 8388608.f - 1.f) * 0.5f;
        }
        e.Process(in, in, out_l, out_r, 48, p);
        for (int k = 0; k < 48; k++) {
            finite = finite && std::isfinite(out_l[k]) && std::isfinite(out_r[k]);
            peak = std::max(peak, std::max(fabsf(out_l[k]), fabsf(out_r[k])));
        }
    }
    CHECK(finite, "reception stress: non-finite output");
    CHECK(peak <= 1.f, "reception stress: peak %.3f", peak);
}
```
Call the four from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile error, `no member named 'reception' in 'bitch::Params'`.

- [ ] **Step 3: Wire Reception into the engine**

`engine/engine.h` — add includes after `#include "shifter.h"`:
```cpp
#include "agc.h"
#include "drive.h"
#include "fading.h"
#include "random.h"
#include "static_noise.h"
```
add to `Params` (after `stereo_mode`):
```cpp
    float reception = 0.f;    // 0 clean .. 1 lost in the noise (drift, fading, static, AGC, drive)
```
change `Init`'s declaration to:
```cpp
    // block_size: the size every Process() call will use (the firmware and sim use 48)
    void Init(float sample_rate, size_t block_size = 48);
```
add private members (after `active_[2]`):
```cpp
    // Reception (fader 2): one amount drives every receiver effect, smoothed per block
    float reception_ = 0.f;
    Drive drive_;
    Wander drift_[2];          // each side's shift wanders on its own path
    Fading fading_[2];
    StaticNoise static_[2];
    Agc agc_[2];
```
and extend the class comment with: "Reception (0-1) adds, per side: drive -> ring mod -> shifter (+ its own drift, up to ±40 Hz) -> fading -> + static -> radio filter -> AGC. At 0 every stage is an exact pass-through."

`engine/engine.cpp`:
- Out-of-class definitions (keep the Hilbert ones; add any Fading/Agc ones not added yet):
```cpp
constexpr float Hilbert::kA[4];
constexpr float Hilbert::kB[4];
constexpr int Fading::kSize;
constexpr int Fading::kMask;
constexpr float Agc::kTarget;
```
- `Init` becomes:
```cpp
void Engine::Init(float sample_rate, size_t block_size) {
    sample_rate_ = sample_rate;
    float block_rate = sample_rate / (float)block_size;
    for (RingMod &r : ring_) r.Init(sample_rate);
    for (Shifter &s : shifter_) s.Init(sample_rate);
    for (RadioFilter &f : filter_) f.Init(sample_rate);
    const uint32_t kSeeds[2] = {11u, 29u}; // left and right wander on different paths
    for (int c = 0; c < 2; c++) {
        drift_[c].Init(block_rate, kSeeds[c]);
        fading_[c].Init(sample_rate, block_rate, kSeeds[c] * 13u + 101u);
        static_[c].Init(sample_rate, kSeeds[c] * 7u + 5u);
        agc_[c].Init(sample_rate);
    }
    glide_ = 1.f - expf(-1.f / (0.02f * sample_rate));
    envelope_release_ = expf(-1.f / (0.2f * sample_rate));
    take_glide_ = 1.f - expf(-1.f / (0.05f * sample_rate));
    rotate_step_ = 1.f / (8.f * sample_rate);
    Reset();
}
```
- In `Reset()`, add:
```cpp
    reception_ = 0.f;
    for (Fading &f : fading_) f.Reset();
    for (Agc &a : agc_) a.Reset();
```
- In `Process()`:
  - In the `if (block_glide_ == 0.f)` first-block snap, add `reception_ = Clamp(params.reception, 0.f, 1.f);`.
  - After the filter `Set` loop, add the per-block Reception updates:
```cpp
    // Reception: one smoothed amount for every receiver effect
    reception_ += block_glide_ * (Clamp(params.reception, 0.f, 1.f) - reception_);
    float r = reception_ < 1e-4f ? 0.f : reception_;
    drive_.Set(r);
    float drift_hz[2];
    for (int c = 0; c < 2; c++) {
        drift_hz[c] = r > 0.f ? r * 40.f * drift_[c].Next(0.1f + 0.4f * r) : 0.f;
        fading_[c].Update(r);
        static_[c].Update(r);
        agc_[c].Update(r);
    }
```
  - In the Linked branch, scale the phase lock so the sides can drift: change `shifter_[1].NudgePhase(0.02f * err);` to `shifter_[1].NudgePhase(0.02f * (1.f - r) * err);` and update its comment ("scaled down by reception so the sides can wander").
  - In the per-side loop, replace the `ringed` / `wet` lines with:
```cpp
            float driven = drive_.Process(dry);
            float ringed = ring_[c].Process(driven, carrier_hz_, ring_amount_);
            float shifted = shifter_[c].Process(ringed, shift[c] + drift_hz[c], feedback);
            float received = fading_[c].Process(shifted) + static_[c].Process();
            float wet = agc_[c].Process(filter_[c].Process(received));
```
  (`dry` and the `y[c] = dry + mix_ * (wet - dry);` mix stay as they are: the dry path is never driven.)

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed` — including every earlier test (reception defaults to 0, where every new stage is an exact pass-through).

- [ ] **Step 5: Check the firmware still builds**

Run: `make -C ~/local_files/daisy/bitchshifter`
Expected: memory table, no errors or warnings.

- [ ] **Step 6: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/engine.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: Reception (drift, fading, static, AGC, drive) on one amount"
```

---

### Task 8: Render option, and listening

**Files:**
- Modify: `sim/render.cpp`
- Modify: `sim/render.sh`

- [ ] **Step 1: Add the option**

In `sim/render.cpp`, add `{"reception", &p.reception}` to the `fields[]` table, add `[reception=0-1]` to the usage comment at the top and to the usage message, and add `reception %g` / `p.reception` to the final `printf` (after `open %g`). In `sim/render.sh`, add `[reception=0-1]` to the usage comment.

- [ ] **Step 2: Render**

```bash
cd ~/local_files/daisy/bitchshifter/sim/out
../render.sh voice.wav p3_reception_25.wav tune=150 filter=1100 q=4 open=0 reception=0.25
../render.sh voice.wav p3_reception_60.wav tune=150 filter=1100 q=4 open=0 reception=0.6
../render.sh voice.wav p3_reception_100.wav tune=150 filter=1100 q=4 open=0 reception=1
../render.sh voice.wav p3_lost_station.wav tune=-70 ring=0.3 carrier=30 fb=0.4 filter=1500 q=6 open=0 reception=0.85
../render.sh stereo_source.wav p3_stereo_drift.wav tune=80 filter=1300 q=3 open=0 reception=0.5
```
Expected: five `Wrote …` lines, no compiler warnings.

- [ ] **Step 3: User listens, then commit**

Send the five files to the user (a light, a moderate and a full reception on the mistuned voice; a "lost station"; and the stereo source drifting apart). After they're happy:
```bash
cd ~/local_files/daisy/bitchshifter
git add sim/render.cpp sim/render.sh
git commit -m "Render tool: reception option"
```

---

### Task 9: Simple Touch: fader 2

**Files:**
- Modify: `simple_touch.cpp`
- Modify: `README.md`

- [ ] **Step 1: Wire fader 2**

In `simple_touch.cpp`:
- Add to the control map: `const int kInputReception = 7; // fader 2`
- In `AudioCallback`, after the other `p.` lines: `p.reception = bitch::ReceptionAmount(Read(kInputReception));`
- In the main loop's `PrintLine`, add `reception %d%%` after the tune/fb/mix group, with the argument `(int)(bitch::ReceptionAmount(Read(kInputReception)) * 100.f)`.
- Update the file's top comment to include fader 2 RECEPTION.

- [ ] **Step 2: Build**

Run: `make -C ~/local_files/daisy/bitchshifter`
Expected: memory table, no errors or warnings.

- [ ] **Step 3: User flashes and plays it**

User runs `make program-dfu` in `~/local_files/daisy/bitchshifter`. With a source in, TUNE a little off centre, and a narrow-ish filter (WIDTH around 10 o'clock):
- Fader 2 at the bottom: as before (clean).
- Raising it: the sides start to drift apart (headphones), static appears, the signal fades in and out, quiet moments fill with hiss, and near the top the input is driven into fuzz.
- Read the USB line (`cat /dev/cu.usbmodem*`): reception % tracks the fader; CPU (expect roughly 20–25%).

- [ ] **Step 4: Document and commit (after the user confirms)**

In `README.md`'s "Controls so far" table, add after the TUNE row:
```markdown
| Fader 2 RECEPTION | Clean (bottom) → lost in the noise (top): the sides drift apart, the signal fades, static and crackles rise, the AGC pumps, the input is driven |
```
and update the CPU note under the table to the measured figure.
```bash
cd ~/local_files/daisy/bitchshifter
git add simple_touch.cpp README.md
git commit -m "Simple Touch firmware: RECEPTION on fader 2"
```

---

## Later plans

- **Part 4:** pads and switches (stations, keyboard, performance), designed around the measured layout (3 + 5 + 2 pads plus the TOU/CH logo pads; TOU = stereo mode, CH reserved for the carrier input). Smoothing for pad-driven jumps is already in place for ring and mix.
- **Part 5 (optional):** the carrier input mode on CH; reverb; Daisy Patch front end.
