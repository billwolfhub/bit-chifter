# bit-chifter Reverb Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a light stereo plate reverb at the end of the chain, brought in by the top 15% of the MIX knob.

**Architecture:** A header-only `PlateReverb` (Dattorro's figure-of-eight plate, delay lengths scaled from his 29.761 kHz to 48 kHz) in `engine/plate_reverb.h`, fed by the mono sum of the mixed output, returning a wide stereo tail. `Params::reverb` (0–1, default 0 = off, an exact pass-through) sets the return level (up to −6 dB) and the decay (~1.2 s → ~3 s). The engine adds the return after the dry/wet mix and stereo swap, before the final soft clip. `mapping.h` gets `MixAndReverb(knob, mix, reverb)`: dry/wet over the first 85% of the turn, the reverb over the last 15% (full from 97%, like WIDTH's open end).

**Tech Stack:** C++17 (also gnu++14 for the firmware), libDaisy v8.1, Apple clang for `sim/`.

**Spec:** `docs/superpowers/specs/2026-10-12-bitchshifter-design.md` (Knobs → 6 MIX; Signal chain item 7, Reverb).

**Conventions:**
- Project root `~/local_files/daisy/bit-chifter`, git, master (public on GitHub; don't push, the user asks for that). No `Co-Authored-By` or other attribution lines in commits.
- Tests: `sim/run_tests.sh` (prints `All tests passed`). Helpers in `sim/test_engine.cpp`: `CHECK(cond, fmt, ...)`, `kSampleRate` (48000.f), `kPi`, `Magnitude(x, freq)`, `Db(ratio)`, `RunEngine`, `RunEngineStereo`, `Window(x, from_s, to_s)`. New test functions go above `main()` and are called from it.
- Engine code: single precision only (`sinf`, `1.f`), nothing allocating in `Process()`; the firmware builds as gnu++14 (odr-used `static constexpr` members would need out-of-class definitions in `engine/engine.cpp`; this plan uses an `enum` for the delay lengths, so none are needed).
- If a test fails after implementing, don't change tolerances or the given algorithm to make it pass: report the printed numbers.
- Hardware steps: the user flashes and confirms before the firmware commit.

## File structure

| File | Responsibility |
|---|---|
| `engine/mapping.h` | `MixAndReverb(knob, mix, reverb)` |
| `engine/plate_reverb.h` | `DelayLine<N>`, `PlateReverb` (`Init`, `Clear`, `SetDecay`, `Process`) |
| `engine/engine.h`, `engine/engine.cpp` | `Params::reverb`; the plate after the mix, before the soft clip |
| `sim/test_engine.cpp` | tests |
| `sim/render.cpp`, `sim/render.sh` | `reverb=` option |
| `simple_touch.cpp`, `README.md` | MIX knob → `MixAndReverb`; reverb in the USB report |

---

### Task 1: The MIX knob mapping

**Files:**
- Modify: `engine/mapping.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing test**

Above `main()` in `sim/test_engine.cpp`:
```cpp
static void TestMixAndReverb() {
    float mix, reverb;
    bitch::MixAndReverb(0.f, mix, reverb);
    CHECK(mix == 0.f && reverb == 0.f, "mix knob 0: mix %.3f reverb %.3f", mix, reverb);
    bitch::MixAndReverb(0.425f, mix, reverb);
    CHECK(fabsf(mix - 0.5f) < 1e-4f && reverb == 0.f, "mix knob 42.5%%: mix %.3f reverb %.3f", mix, reverb);
    bitch::MixAndReverb(0.85f, mix, reverb);
    CHECK(mix == 1.f && reverb == 0.f, "mix knob 85%%: mix %.3f reverb %.3f", mix, reverb);
    bitch::MixAndReverb(0.91f, mix, reverb);
    CHECK(mix == 1.f && fabsf(reverb - 0.5f) < 1e-3f, "mix knob 91%%: mix %.3f reverb %.3f", mix, reverb);
    bitch::MixAndReverb(0.97f, mix, reverb);
    CHECK(mix == 1.f && reverb == 1.f, "mix knob 97%%: mix %.3f reverb %.3f", mix, reverb);
    bitch::MixAndReverb(1.f, mix, reverb);
    CHECK(mix == 1.f && reverb == 1.f, "mix knob 100%%: mix %.3f reverb %.3f", mix, reverb);
}
```
Call `TestMixAndReverb();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: compile error, `no member named 'MixAndReverb' in namespace 'bitch'`.

- [ ] **Step 3: Write the mapping**

In `engine/mapping.h`, after `FilterWidth`:
```cpp
// Knob -> dry/wet and reverb: dry/wet 0-1 over the first 85% of the turn; the rest stays
// fully wet and brings in the reverb, full from 97% of the turn (a knob rarely reads exactly 1.0)
inline void MixAndReverb(float knob, float &mix, float &reverb) {
    mix = fminf(fmaxf(knob / 0.85f, 0.f), 1.f);
    reverb = fminf(fmaxf((knob - 0.85f) / 0.12f, 0.f), 1.f);
}
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/mapping.h sim/test_engine.cpp
git commit -m "Engine: MIX knob mapping with the reverb on its top end"
```

---

### Task 2: The plate reverb

**Files:**
- Create: `engine/plate_reverb.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "plate_reverb.h"` with the other engine includes, and above `main()`:
```cpp
// Runs a plate on an impulse (in both inputs) for `seconds`, returning both outputs
static void PlateImpulse(float decay_amount, float seconds, std::vector<float> &l, std::vector<float> &r) {
    static bitch::PlateReverb plate; // large: keep it off the stack
    plate.Init(kSampleRate);
    plate.SetDecay(decay_amount);
    l.clear();
    r.clear();
    for (int n = 0; n < (int)(seconds * kSampleRate); n++) {
        float x = n == 0 ? 1.f : 0.f, ol, orr;
        plate.Process(x, x, ol, orr);
        l.push_back(ol);
        r.push_back(orr);
    }
}

static double Energy(const std::vector<float> &x) {
    double e = 0.0;
    for (float v : x) e += (double)v * v;
    return e;
}

static void TestPlateReverb() {
    std::vector<float> l, r;
    PlateImpulse(0.f, 4.f, l, r);
    bool finite = true;
    float peak = 0.f;
    for (size_t n = 0; n < l.size(); n++) {
        finite = finite && std::isfinite(l[n]) && std::isfinite(r[n]);
        peak = std::max(peak, std::max(fabsf(l[n]), fabsf(r[n])));
    }
    CHECK(finite, "plate: non-finite output");
    CHECK(peak < 1.f, "plate: impulse peak %.3f", peak);
    // A tail on both sides, starting after the predelay
    double early_l = Energy(Window(l, 0.f, 0.009f)), tail_l = Energy(Window(l, 0.05f, 0.3f));
    double tail_r = Energy(Window(r, 0.05f, 0.3f));
    CHECK(early_l < 1e-12, "plate: output before the predelay (energy %g)", early_l);
    CHECK(tail_l > 1e-3 && tail_r > 1e-3, "plate: tail energy L %g R %g", tail_l, tail_r);
    // Wide: the sides are decorrelated
    auto wl = Window(l, 0.05f, 0.5f), wr = Window(r, 0.05f, 0.5f);
    double lr = 0.0;
    for (size_t n = 0; n < wl.size(); n++) lr += (double)wl[n] * wr[n];
    double correlation = lr / sqrt(Energy(wl) * Energy(wr));
    CHECK(fabs(correlation) < 0.5, "plate: L/R correlation %.2f", correlation);
    // It dies away, and lasts longer with a longer decay
    double short_late = Energy(Window(l, 1.5f, 2.f)) / tail_l;
    CHECK(short_late < 1e-3, "plate (short): energy at 1.5-2 s is %.1f dB of the early tail", 10 * log10(short_late));
    PlateImpulse(1.f, 4.f, l, r);
    double long_late = Energy(Window(l, 1.5f, 2.f)) / Energy(Window(l, 0.05f, 0.3f));
    CHECK(long_late > 10 * short_late, "plate: long decay %.1f dB vs short %.1f dB at 1.5-2 s",
          10 * log10(long_late), 10 * log10(short_late));
    CHECK(Energy(Window(l, 3.5f, 4.f)) < Energy(Window(l, 0.05f, 0.3f)) * 1e-2, "plate (long): still loud at 3.5 s");
}

static void TestPlateReverbClear() {
    static bitch::PlateReverb plate;
    plate.Init(kSampleRate);
    plate.SetDecay(1.f);
    float ol, orr;
    for (int n = 0; n < 4800; n++) plate.Process(0.5f * sinf(2 * kPi * 440.f * n / kSampleRate), 0.f, ol, orr);
    plate.Clear();
    float peak = 0.f;
    for (int n = 0; n < 48000; n++) {
        plate.Process(0.f, 0.f, ol, orr);
        peak = std::max(peak, std::max(fabsf(ol), fabsf(orr)));
    }
    CHECK(peak == 0.f, "plate: %g after Clear()", peak);
}
```
Call `TestPlateReverb();` and `TestPlateReverbClear();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: compile error, `'plate_reverb.h' file not found`.

- [ ] **Step 3: Write the reverb**

`engine/plate_reverb.h`:
```cpp
#pragma once

#include <cmath>

namespace bitch {

// A fixed-length delay line: Write() one sample, Tap(d) reads the sample written d
// writes ago (1 = the newest, N = the oldest).
template <int N>
class DelayLine {
  public:
    void Clear() {
        for (float &v : buffer_) v = 0.f;
        position_ = 0;
    }
    void Write(float x) {
        buffer_[position_] = x;
        if (++position_ == N) position_ = 0;
    }
    float Tap(int d) const {
        int i = position_ - d;
        if (i < 0) i += N;
        return buffer_[i];
    }
    // Fractional tap (linear interpolation), 1 <= d < N
    float Tap(float d) const {
        int whole = (int)d;
        float a = Tap(whole), b = Tap(whole + 1);
        return a + (d - (float)whole) * (b - a);
    }

  private:
    float buffer_[N] = {};
    int position_ = 0;
};

// Light stereo plate: Jon Dattorro's figure-of-eight plate ("Effect Design, Part 1",
// 1997), with his delay lengths scaled from 29.761 kHz to 48 kHz (so it assumes a
// 48 kHz sample rate). Mono in (the sum of both inputs), wide stereo out (his output
// taps). Band-limited input (~150 Hz - 8 kHz), 10 ms predelay, four allpasses for
// diffusion, then two cross-coupled loops, each: modulated allpass -> delay -> damping
// -> x decay -> allpass -> delay -> x decay -> into the other loop. Returns only the
// reverb (no dry). About 145 KB of buffers.
class PlateReverb {
  public:
    void Init(float sample_rate) {
        const float kTwoPi = 6.28318530718f;
        low_cut_ = 1.f - expf(-kTwoPi * 150.f / sample_rate);
        high_cut_ = 1.f - expf(-kTwoPi * 8000.f / sample_rate);
        damping_ = 1.f - expf(-kTwoPi * 5000.f / sample_rate);
        lfo_step_ = kTwoPi * 0.9f / sample_rate; // 0.9 Hz wobble on the loops' allpasses
        SetDecay(0.f);
        Clear();
    }

    // amount 0-1: decay ~1.2 s -> ~3 s
    void SetDecay(float amount) { decay_ = 0.35f + 0.3f * fminf(fmaxf(amount, 0.f), 1.f); }

    void Clear() {
        predelay_.Clear();
        in1_.Clear();
        in2_.Clear();
        in3_.Clear();
        in4_.Clear();
        left_ap1_.Clear();
        left_d1_.Clear();
        left_ap2_.Clear();
        left_d2_.Clear();
        right_ap1_.Clear();
        right_d1_.Clear();
        right_ap2_.Clear();
        right_d2_.Clear();
        low_ = high_ = 0.f;
        left_damp_ = right_damp_ = 0.f;
        left_end_ = right_end_ = 0.f;
        lfo_sin_ = 0.f;
        lfo_cos_ = 1.f;
    }

    void Process(float in_l, float in_r, float &out_l, float &out_r) {
        // Band-limit the mono input, predelay it, diffuse it
        float x = 0.5f * (in_l + in_r);
        high_ += high_cut_ * (x - high_);
        low_ += low_cut_ * (high_ - low_);
        predelay_.Write(high_ - low_);
        x = predelay_.Tap((int)kPredelay);
        x = Allpass(in1_, (int)kIn1, x, 0.75f);
        x = Allpass(in2_, (int)kIn2, x, 0.75f);
        x = Allpass(in3_, (int)kIn3, x, 0.625f);
        x = Allpass(in4_, (int)kIn4, x, 0.625f);

        // The loop allpasses' wobble: a quadrature oscillator (the "magic circle")
        lfo_sin_ += lfo_step_ * lfo_cos_;
        lfo_cos_ -= lfo_step_ * lfo_sin_;
        float wobble_l = (float)kExcursion * (1.f + lfo_sin_) * 0.5f;
        float wobble_r = (float)kExcursion * (1.f + lfo_cos_) * 0.5f;

        // Each loop is fed by the other's end from the previous sample
        float into_left = x + decay_ * right_end_, into_right = x + decay_ * left_end_;

        float l = ModulatedAllpass(left_ap1_, (float)kLeftAp1 + wobble_l, into_left, 0.7f);
        left_d1_.Write(l);
        l = left_d1_.Tap((int)kLeftD1);
        left_damp_ += damping_ * (l - left_damp_);
        l = Allpass(left_ap2_, (int)kLeftAp2, decay_ * left_damp_, 0.5f);
        left_d2_.Write(l);
        left_end_ = left_d2_.Tap((int)kLeftD2);

        float r = ModulatedAllpass(right_ap1_, (float)kRightAp1 + wobble_r, into_right, 0.7f);
        right_d1_.Write(r);
        r = right_d1_.Tap((int)kRightD1);
        right_damp_ += damping_ * (r - right_damp_);
        r = Allpass(right_ap2_, (int)kRightAp2, decay_ * right_damp_, 0.5f);
        right_d2_.Write(r);
        right_end_ = right_d2_.Tap((int)kRightD2);

        // Dattorro's output taps, scaled to 48 kHz
        out_l = 0.6f * (right_d1_.Tap(429) + right_d1_.Tap(4797) - right_ap2_.Tap(3085) + right_d2_.Tap(3219) -
                        left_d1_.Tap(3210) - left_ap2_.Tap(302) - left_d2_.Tap(1719));
        out_r = 0.6f * (left_d1_.Tap(569) + left_d1_.Tap(5850) - left_ap2_.Tap(1981) + left_d2_.Tap(4311) -
                        right_d1_.Tap(3405) - right_ap2_.Tap(540) - right_d2_.Tap(195));
    }

  private:
    // Delay lengths in samples at 48 kHz (Dattorro's x 48000 / 29761)
    enum : int {
        kPredelay = 480,
        kIn1 = 229,
        kIn2 = 173,
        kIn3 = 611,
        kIn4 = 447,
        kExcursion = 26, // the loop allpasses' wobble, in samples
        kLeftAp1 = 1084,
        kLeftD1 = 7182,
        kLeftAp2 = 2903,
        kLeftD2 = 6000,
        kRightAp1 = 1464,
        kRightD1 = 6801,
        kRightAp2 = 4284,
        kRightD2 = 5101,
    };

    // Schroeder allpass on a delay line of length d (the line's last tap)
    template <int N>
    static float Allpass(DelayLine<N> &line, int d, float x, float g) {
        float delayed = line.Tap(d);
        float w = x + g * delayed;
        line.Write(w);
        return delayed - g * w;
    }
    template <int N>
    static float ModulatedAllpass(DelayLine<N> &line, float d, float x, float g) {
        float delayed = line.Tap(d);
        float w = x - g * delayed; // Dattorro's decay-diffusion allpasses have the opposite sign
        line.Write(w);
        return delayed + g * w;
    }

    DelayLine<kPredelay> predelay_;
    DelayLine<kIn1> in1_;
    DelayLine<kIn2> in2_;
    DelayLine<kIn3> in3_;
    DelayLine<kIn4> in4_;
    DelayLine<kLeftAp1 + kExcursion + 2> left_ap1_;
    DelayLine<kLeftD1> left_d1_;
    DelayLine<kLeftAp2> left_ap2_;
    DelayLine<kLeftD2> left_d2_;
    DelayLine<kRightAp1 + kExcursion + 2> right_ap1_;
    DelayLine<kRightD1> right_d1_;
    DelayLine<kRightAp2> right_ap2_;
    DelayLine<kRightD2> right_d2_;
    float low_cut_ = 0.f, high_cut_ = 0.f, damping_ = 0.f, lfo_step_ = 0.f, decay_ = 0.35f;
    float low_ = 0.f, high_ = 0.f, left_damp_ = 0.f, right_damp_ = 0.f, left_end_ = 0.f, right_end_ = 0.f;
    float lfo_sin_ = 0.f, lfo_cos_ = 1.f;
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`. If a decay or correlation check fails, report the printed dB / correlation values.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/plate_reverb.h sim/test_engine.cpp
git commit -m "Engine: light stereo plate reverb (Dattorro)"
```

---

### Task 3: The reverb in the engine

**Files:**
- Modify: `engine/engine.h`
- Modify: `engine/engine.cpp`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

Above `main()` in `sim/test_engine.cpp`:
```cpp
// Runs a (static, large) engine on a 440 Hz tone for `tone_s`, then silence for `silence_s`,
// with reverb `reverb_tone` during the tone and `reverb_silence` after; returns the left output
static std::vector<float> RunToneThenSilence(float tone_s, float silence_s, float reverb_tone, float reverb_silence) {
    static bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.mix = 1.f;
    std::vector<float> out;
    float in[48], out_l[48], out_r[48];
    int tone_blocks = (int)(tone_s * kSampleRate / 48), blocks = (int)((tone_s + silence_s) * kSampleRate / 48);
    for (int b = 0; b < blocks; b++) {
        p.reverb = b < tone_blocks ? reverb_tone : reverb_silence;
        for (int k = 0; k < 48; k++)
            in[k] = b < tone_blocks ? 0.3f * sinf(2 * kPi * 440.f * (b * 48 + k) / kSampleRate) : 0.f;
        e.Process(in, in, out_l, out_r, 48, p);
        out.insert(out.end(), out_l, out_l + 48);
    }
    return out;
}

static float Peak(const std::vector<float> &x) {
    float peak = 0.f;
    for (float v : x) peak = std::max(peak, fabsf(v));
    return peak;
}

static void TestEngineReverb() {
    // Off: silence right after the tone stops
    auto dry = RunToneThenSilence(0.5f, 1.f, 0.f, 0.f);
    CHECK(Peak(Window(dry, 0.6f, 1.5f)) < 1e-4f, "reverb off: %g after the tone", Peak(Window(dry, 0.6f, 1.5f)));
    // On: a tail after the tone stops
    auto wet = RunToneThenSilence(0.5f, 1.f, 1.f, 1.f);
    CHECK(Peak(Window(wet, 0.6f, 0.9f)) > 0.01f, "reverb on: tail only %g", Peak(Window(wet, 0.6f, 0.9f)));
    // Turned down, then up again: the old tail doesn't come back
    static bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    float in[48], out_l[48], out_r[48];
    auto run = [&](float seconds, float level, float reverb) {
        p.reverb = reverb;
        float peak = 0.f;
        for (int b = 0; b < (int)(seconds * kSampleRate / 48); b++) {
            for (int k = 0; k < 48; k++) in[k] = level * sinf(2 * kPi * 440.f * (b * 48 + k) / kSampleRate);
            e.Process(in, in, out_l, out_r, 48, p);
            for (int k = 0; k < 48; k++) peak = std::max(peak, std::max(fabsf(out_l[k]), fabsf(out_r[k])));
        }
        return peak;
    };
    run(0.5f, 0.3f, 1.f);                  // tone into the reverb
    run(0.5f, 0.f, 0.f);                   // reverb down (it glides out over ~0.2 s), silence
    float replay = run(0.5f, 0.f, 1.f);    // reverb up again, still silence
    CHECK(replay < 1e-5f, "reverb: the old tail replayed (%g) after turning it down and up", replay);
}

static void TestEngineReverbStable() {
    static bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.reverb = 1.f;
    p.feedback = 0.95f;
    p.reception = 1.f;
    p.tune_hz = 37.f;
    uint32_t seed = 5;
    float in[48], out_l[48], out_r[48];
    bool finite = true;
    for (int b = 0; b < (int)(20 * kSampleRate / 48); b++) {
        for (float &v : in) {
            seed = seed * 1664525u + 1013904223u;
            v = ((seed >> 8) / 8388608.f - 1.f) * 0.5f;
        }
        e.Process(in, in, out_l, out_r, 48, p);
        for (int k = 0; k < 48; k++) finite = finite && std::isfinite(out_l[k]) && std::isfinite(out_r[k]);
    }
    CHECK(finite, "reverb stress: non-finite output");
}
```
Call `TestEngineReverb();` and `TestEngineReverbStable();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: compile error, `no member named 'reverb' in 'bitch::Params'`.

- [ ] **Step 3: Wire the reverb in**

`engine/engine.h`:
- Add `#include "plate_reverb.h"` after the other includes.
- Add to `Params` (after `dial_inertia_s`):
```cpp
    float reverb = 0.f;          // plate reverb 0-1: return level (up to -6 dB) and decay (0 = off)
```
- Extend the class comment: "A light stereo plate reverb (Params::reverb) is added after the dry/wet mix, before the soft clip."
- Add private members (after the dial members):
```cpp
    // The plate reverb, after the mix (fed by both sides, returning a wide stereo tail)
    PlateReverb plate_;
    float reverb_ = 0.f;          // glided towards params.reverb
    bool plate_running_ = false;  // false: the plate is idle (cleared before it starts again)
```

`engine/engine.cpp`:
- In `Init`, after `pitch_.Init(sample_rate);`: `plate_.Init(sample_rate);`.
- In `Reset()`: `plate_.Clear(); reverb_ = 0.f; plate_running_ = false;`.
- In `Process()`, after the per-block dial code (before the per-sample loop):
```cpp
    // The plate: idle (and skipped) at 0; cleared when it starts again, so an old tail never replays
    float reverb = Clamp(params.reverb, 0.f, 1.f);
    if (reverb > 0.f && !plate_running_) {
        plate_.Clear();
        plate_running_ = true;
    }
    if (reverb == 0.f && reverb_ < 1e-4f) {
        reverb_ = 0.f;
        plate_running_ = false;
    }
    plate_.SetDecay(reverb_);
```
- In the per-sample loop, replace:
```cpp
        out_l[i] = SoftClip(y[0] + swap_ * (y[1] - y[0]));
        out_r[i] = SoftClip(y[1] + swap_ * (y[0] - y[1]));
```
with:
```cpp
        float left = y[0] + swap_ * (y[1] - y[0]), right = y[1] + swap_ * (y[0] - y[1]);
        if (plate_running_) {
            reverb_ += glide_ * (reverb - reverb_);
            float tail_l, tail_r;
            plate_.Process(left, right, tail_l, tail_r);
            left += 0.5f * reverb_ * tail_l; // up to -6 dB
            right += 0.5f * reverb_ * tail_r;
        }
        out_l[i] = SoftClip(left);
        out_r[i] = SoftClip(right);
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`, including every earlier test (the reverb defaults to 0, where the plate is skipped).

- [ ] **Step 5: Check the firmware still builds, and the memory**

Run: `make -C ~/local_files/daisy/bit-chifter`
Expected: no errors or warnings; SRAM use rises by about 145 KB (from ~113 KB to ~260 KB of 480 KB). Report the SRAM line.

- [ ] **Step 6: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/engine.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: plate reverb after the mix"
```

---

### Task 4: Render option and listening

**Files:**
- Modify: `sim/render.cpp`
- Modify: `sim/render.sh`

- [ ] **Step 1: Add the option**

In `sim/render.cpp`: add `{"reverb", &p.reverb}` to `fields[]`; add `[reverb=0-1]` to the usage comment and message, and ` reverb %g` (with `p.reverb`) to the final `printf` after `inertia %g`. In `sim/render.sh`, add `[reverb=0-1]` to the usage comment.

- [ ] **Step 2: Render**

```bash
cd ~/local_files/daisy/bit-chifter/sim/out
../render.sh voice.wav rev_voice_half.wav reverb=0.5
../render.sh voice.wav rev_voice_full.wav reverb=1
../render.sh drone.wav rev_drone_shift_full.wav tune=-60 fb=0.6 detent=0.7 inertia=0.06 reverb=1
../render.sh voice.wav rev_voice_radio_full.wav tune=150 filter=1100 q=3 open=0 reception=0.5 reverb=1
```
Expected: four `Wrote …` lines showing `reverb 0.5` / `reverb 1`, no compiler warnings.

- [ ] **Step 3: User listens, then commit**

Send the four renders to the user (voice with half and full reverb, a shifted drone with feedback, the radio voice with reception). After they're happy:
```bash
cd ~/local_files/daisy/bit-chifter
git add sim/render.cpp sim/render.sh
git commit -m "Render tool: reverb option"
```

---

### Task 5: Firmware

**Files:**
- Modify: `simple_touch.cpp`
- Modify: `README.md`

- [ ] **Step 1: MIX knob → mix and reverb**

In `simple_touch.cpp`'s `AudioCallback`, replace `p.mix = Read(kInputMix);` with:
```cpp
    bitch::MixAndReverb(Read(kInputMix), p.mix, p.reverb);
```
In the report: compute `float mix, reverb; bitch::MixAndReverb(Read(kInputMix), mix, reverb);` next to the existing `FilterWidth` call, and change the `mix %d%%` field to `mix %d%% rev %d%%` with the arguments `(int)(mix * 100.f), (int)(reverb * 100.f)` (in place of `(int)(Read(kInputMix) * 100.f)`). In the file's top comment, change "knob 6 MIX" to "knob 6 MIX (its last 15% adds reverb)".

- [ ] **Step 2: Build**

Run: `make -C ~/local_files/daisy/bit-chifter`
Expected: memory table, no errors or warnings.

- [ ] **Step 3: User flashes and plays it**

User runs `make program-dfu` in `~/local_files/daisy/bit-chifter` (RESET, then BOOT while the LED pulses). Try: MIX up to about 85% (fully wet, no reverb), then on to the end (the reverb rises and lengthens); with feedback and TUNE off centre; with RECEPTION up. In the USB report (`cat /dev/cu.usbmodem*`): `mix 100% rev …%` follows the knob's top end; CPU should rise by only a few percent with the reverb up.

- [ ] **Step 4: Document and commit (after the user confirms)**

In `README.md`'s "Controls so far" table, replace the MIX row with:
```markdown
| Knob 6 MIX | Dry/wet over the first 85% of the turn; the last 15% stays fully wet and adds a light stereo plate reverb (longer as it rises) |
```
Add `[reverb=0-1]` to the render line, and update the CPU note with the value the user reports.
```bash
cd ~/local_files/daisy/bit-chifter
git add simple_touch.cpp README.md
git commit -m "Simple Touch firmware: reverb on the MIX knob's top end"
```
