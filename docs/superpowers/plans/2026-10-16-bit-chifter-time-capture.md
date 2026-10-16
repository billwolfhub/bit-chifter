# bit-chifter Strange Pages Steps 2–3 (Time, Motion) and Switch 2 (Capture) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build page 1's remaining pads (Echoes, Reverse echoes, Ping-pong echoes, Spiral up/down, Chirp, Warble) and switch 2's capture modes (Normal / Freeze / Loop).

**Architecture:** Two new buffer classes, `EchoLoop` (a 1 s delay with a tape-like gliding time, a low-pass and soft clip on the way in, a reverse read with two crossfading heads, and stepwise clearing) and `CaptureLoop` (a 2 s stereo recorder that freezes the last 250 ms as a drone or loops the last 2 s), share one SDRAM block, `TapeMemory`, which the front end owns and passes to `Engine::Init` (like the plate reverb); without it, the echo pads and capture do nothing. The echo replaces the shifter's 2 ms feedback while its pads are held (crossfaded by the pads' level), so each repeat is shifted again. The spiral runs a second `Shifter` per side as the second voice of a Shepard-style pair. Chirp (an envelope follower) and Warble (an LFO) add to the shift before the stereo gains. Capture sits at the very start of the chain, so everything after it, the dry signal and the pitch tracker included, hears the captured sound. With the pads off and switch 2 at Normal, the sound is bit-for-bit unchanged.

**Tech Stack:** C++17 (also gnu++14 for the firmware), libDaisy v8.1, Apple clang for `sim/`.

**Spec:** `docs/superpowers/specs/2026-10-13-bit-chifter-strange-pages-design.md` (Page 1: Shift & time; Capture (switch 2)).

**Conventions:**
- Project root `~/local_files/daisy/bit-chifter`, git, master (public on GitHub; don't push, the user asks for that). No `Co-Authored-By` or other attribution lines in commits.
- Tests: `sim/run_tests.sh`. **New test calls go in `main()` before the `if (failures == 0) printf("All tests passed\n");` line; check the script's exit code and FAIL lines, not only its last line.** Helpers in `sim/test_engine.cpp`: `CHECK(cond, fmt, ...)`, `kSampleRate`, `kPi`, `Magnitude(x, freq)`, `Db(ratio)`, `Window(x, from_s, to_s)`, `Peak(x)`, `Energy(x)`, `Rms(x)`, `Sine(hz, amplitude, n)`, `RunFor(engine, params, seconds, signal, n, l, r)`, `ShiftTrace(engine, params, seconds, signal)` (left shifter frequency per 48-sample block, ±500 Hz), `kBuiltEffects` (used by the bit-exact `TestEffectsUnbuiltDoNothing`).
- `EffectRange(amount)` maps the effect amount 0.35–1 to 0–1. `Wander` (`random.h`): `Init(block_rate, seed)`, `Next(rate_hz)` (−1..1, once per block). `SoftClip` (`softclip.h`).
- Engine code: single precision only, nothing allocating in `Process()`; gnu++14 for the firmware (enums, not odr-used `static constexpr` members). Large buffers live in SDRAM (`TapeMemory`); anything that clears them does so in steps (≤ 1024 floats per block) inside the audio callback.
- If a test fails after implementing, don't change tolerances or the given algorithm to make it pass: report the printed numbers.
- Hardware steps: the user listens to renders, and flashes and confirms before the firmware commit.

## File structure

| File | Responsibility |
|---|---|
| `engine/echo_loop.h` | `EchoLoop` |
| `engine/capture_loop.h` | `CaptureMode`, `CaptureLoop` |
| `engine/tape.h` | `TapeMemory` (the SDRAM block both use) |
| `engine/engine.h`, `engine/engine.cpp` | `Params::capture`; `Init(..., TapeMemory *)`; capture, echoes, spiral, chirp, warble |
| `sim/test_engine.cpp` | tests |
| `sim/render.cpp`, `sim/render.sh` | a `TapeMemory`; `freeze_at=`, `loop_at=` options |
| `simple_touch.cpp`, `README.md` | `TapeMemory` in SDRAM; switch 2 → capture; README |

---

### Task 1: The capture loop

**Files:**
- Create: `engine/capture_loop.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "capture_loop.h"`, and above `main()`:
```cpp
static float capture_l[bitch::CaptureLoop::kMax], capture_r[bitch::CaptureLoop::kMax]; // 2 s each

// Runs a capture loop for `seconds` in `mode` on a signal (left = signal, right = -signal)
template <class Signal>
static void RunCapture(bitch::CaptureLoop &c, bitch::CaptureMode mode, float seconds, Signal signal, int &n,
                       std::vector<float> &out, std::vector<float> &live) {
    for (int b = 0; b < (int)(seconds * kSampleRate / 48); b++) {
        c.Set(mode);
        for (int k = 0; k < 48; k++, n++) {
            float l = signal(n), r = -l;
            live.push_back(l);
            c.Process(l, r);
            out.push_back(l);
            CHECK(r == -l, "capture: the sides differ at %d", n);
        }
    }
}

static void TestCaptureLoop() {
    using bitch::CaptureMode;
    bitch::CaptureLoop c;
    c.Init(kSampleRate, capture_l, capture_r);
    uint32_t seed = 7;
    auto noise = [&seed](int) {
        seed = seed * 1664525u + 1013904223u;
        return ((seed >> 8) / 8388608.f - 1.f) * 0.25f;
    };
    auto silence = [](int) { return 0.f; };
    std::vector<float> out, live;
    int n = 0;
    // Too little recorded: Freeze stays live
    RunCapture(c, CaptureMode::kNormal, 0.01f, noise, n, out, live);
    RunCapture(c, CaptureMode::kFreeze, 0.03f, noise, n, out, live);
    CHECK(out == live, "capture: froze before it had 50 ms");
    // Normal: exactly the live input
    out.clear();
    live.clear();
    RunCapture(c, CaptureMode::kNormal, 2.5f, noise, n, out, live);
    CHECK(out == live, "capture: Normal changed the input");
    // Loop: the last 2 s repeat, exactly, every LoopSamples() (after the 30 ms fade in)
    out.clear();
    live.clear();
    RunCapture(c, CaptureMode::kLoop, 6.f, silence, n, out, live);
    int period = c.LoopSamples();
    CHECK(period > (int)(1.9f * kSampleRate) && period < (int)(2.f * kSampleRate), "loop period %d", period);
    int mismatches = 0;
    for (int k = (int)(0.1f * kSampleRate); k + period < (int)out.size(); k++) mismatches += out[k] != out[k + period];
    CHECK(mismatches == 0, "loop: %d samples don't repeat", mismatches);
    CHECK(Rms(Window(out, 0.1f, 6.f)) > 0.1f, "loop: silent (%g)", Rms(Window(out, 0.1f, 6.f)));
    float step = 0.f;
    for (size_t k = 1; k < out.size(); k++) step = std::max(step, fabsf(out[k] - out[k - 1]));
    CHECK(step < 0.55f, "loop: a step of %.3f (noise steps up to 0.5)", step);
    // Back to Normal: live again after the 30 ms fade
    out.clear();
    live.clear();
    RunCapture(c, CaptureMode::kNormal, 0.2f, silence, n, out, live);
    CHECK(Peak(Window(out, 0.05f, 0.2f)) == 0.f, "capture: still playing after Normal");
    // Freeze: a steady drone from the last 250 ms of a tone, with no clicks
    out.clear();
    live.clear();
    RunCapture(c, CaptureMode::kNormal, 1.f, [](int k) { return Sine(440.f, 0.25f, k); }, n, out, live);
    out.clear();
    live.clear();
    RunCapture(c, CaptureMode::kFreeze, 3.f, silence, n, out, live);
    float early = Rms(Window(out, 0.1f, 0.5f)), late = Rms(Window(out, 2.5f, 3.f));
    CHECK(late > 0.1f && fabsf(Db(late / early)) < 3.f, "freeze: level %.3f then %.3f", early, late);
    CHECK(Magnitude(Window(out, 1.f, 3.f), 440.f) > 0.1, "freeze: not the tone");
    step = 0.f;
    for (size_t k = 1; k < out.size(); k++) step = std::max(step, fabsf(out[k] - out[k - 1]));
    CHECK(step < 0.03f, "freeze: a step of %.4f (the tone's own is 0.0144)", step);
}
```
Call `TestCaptureLoop();` from `main()` (before the summary line).

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh; echo "exit $?"`
Expected: compile error, `'capture_loop.h' file not found`.

- [ ] **Step 3: Write the capture loop**

`engine/capture_loop.h`:
```cpp
#pragma once

#include <cmath>

namespace bitch {

// Switch 2: Normal (live input), Freeze or Loop
enum class CaptureMode { kNormal, kFreeze, kLoop };

// Records the input (2 s, stereo, into buffers the caller provides: they live in SDRAM) while
// Normal; on Freeze, holds the last 250 ms as an endless drone (two read heads half a slice
// apart, each with a smooth sin^2 window, so they sum to a constant level); on Loop, repeats
// the last 2 s with a 20 ms crossfade at the loop point. Recording stops while Freeze or Loop
// plays (moving between them plays the same captured audio). Every change crossfades over
// 30 ms against the live input; nothing is captured until 50 ms have been recorded (the
// buffers start as SDRAM garbage, so only recorded samples are ever read). Assumes 48 kHz.
// Set() once per block, Process() per sample.
class CaptureLoop {
  public:
    enum : int { kMax = 96000 }; // 2 s at 48 kHz

    void Init(float sample_rate, float *left, float *right) {
        buffer_[0] = left;
        buffer_[1] = right;
        fade_step_ = 1.f / (0.03f * sample_rate);
        slice_ = (int)(0.25f * sample_rate);
        crossfade_ = (int)(0.02f * sample_rate);
        minimum_ = (int)(0.05f * sample_rate);
        Reset();
    }

    void Reset() {
        write_ = filled_ = 0;
        playing_ = wanted_ = CaptureMode::kNormal;
        mix_ = 0.f;
    }

    void Set(CaptureMode mode) {
        if (mode != CaptureMode::kNormal && playing_ == CaptureMode::kNormal && filled_ < minimum_) return;
        wanted_ = mode;
    }

    void Process(float &l, float &r) {
        // Change what plays only at silence (mix 0), so every change crossfades through live
        if (playing_ != wanted_ && mix_ == 0.f) Start(wanted_);
        float target = playing_ != CaptureMode::kNormal && playing_ == wanted_ ? 1.f : 0.f;
        mix_ = target > mix_ ? fminf(mix_ + fade_step_, 1.f) : fmaxf(mix_ - fade_step_, 0.f);
        if (playing_ == CaptureMode::kNormal) {
            buffer_[0][write_] = l;
            buffer_[1][write_] = r;
            if (++write_ >= kMax) write_ = 0;
            if (filled_ < kMax) filled_++;
            return;
        }
        float cl, cr;
        if (playing_ == CaptureMode::kFreeze) {
            FreezeSample(cl, cr);
        } else {
            LoopSample(cl, cr);
        }
        l += mix_ * (cl - l);
        r += mix_ * (cr - r);
    }

    int LoopSamples() const { return length_ - crossfade_; } // the loop's period, in samples

  private:
    void Start(CaptureMode mode) {
        playing_ = mode;
        if (mode == CaptureMode::kNormal) return;
        end_ = write_; // one past the last recorded sample
        int slice = slice_ < filled_ ? slice_ : filled_;
        grain_ = slice;
        phase_ = 0.f;
        length_ = filled_;
        position_ = crossfade_;
    }

    int Wrap(int i) const { return i < 0 ? i + kMax : (i >= kMax ? i - kMax : i); }

    // Two heads over the last `grain_` samples, half a grain apart, sin^2 windows (sum 1)
    void FreezeSample(float &l, float &r) {
        l = r = 0.f;
        for (int head = 0; head < 2; head++) {
            float p = phase_ + 0.5f * (float)head;
            p -= floorf(p);
            int i = Wrap(end_ - grain_ + (int)(p * (float)grain_));
            float s = sinf(3.14159265359f * p), w = s * s;
            l += w * buffer_[0][i];
            r += w * buffer_[1][i];
        }
        phase_ += 1.f / (float)grain_;
        phase_ -= floorf(phase_);
    }

    // The last `length_` samples, positions crossfade_..length_; the last crossfade_ samples
    // fade into the samples just after the loop's start, then it jumps back to crossfade_
    void LoopSample(float &l, float &r) {
        int start = end_ - length_;
        int i = Wrap(start + position_);
        l = buffer_[0][i];
        r = buffer_[1][i];
        int into = position_ - (length_ - crossfade_);
        if (into >= 0) {
            float a = (float)into / (float)crossfade_;
            int j = Wrap(start + into);
            l += a * (buffer_[0][j] - l);
            r += a * (buffer_[1][j] - r);
        }
        if (++position_ >= length_) position_ = crossfade_;
    }

    float *buffer_[2] = {nullptr, nullptr};
    int write_ = 0, filled_ = 0, end_ = 0, slice_ = 12000, crossfade_ = 960, minimum_ = 2400;
    int grain_ = 12000, length_ = 1, position_ = 0;
    float fade_step_ = 0.f, mix_ = 0.f, phase_ = 0.f;
    CaptureMode playing_ = CaptureMode::kNormal, wanted_ = CaptureMode::kNormal;
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh; echo "exit $?"`
Expected: `All tests passed`, exit 0.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/capture_loop.h sim/test_engine.cpp
git commit -m "Engine: capture loop (Freeze and Loop)"
```

---

### Task 2: The echo loop

**Files:**
- Create: `engine/echo_loop.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "echo_loop.h"`, and above `main()`:
```cpp
static float echo_buffer[bitch::EchoLoop::kMax];

// An impulse through an echo loop (no feedback): where does it come back?
static int EchoArrival(bitch::EchoLoop &e, float time_s, bool reverse, std::vector<float> *out = nullptr) {
    int arrival = -1;
    float best = 0.f;
    for (int b = 0; b < (int)(2.f * kSampleRate / 48); b++) {
        e.Update(time_s, reverse, 48);
        for (int k = 0; k < 48; k++) {
            int n = b * 48 + k;
            float y = e.Read();
            e.Write(n == 0 ? 0.5f : 0.f);
            if (out) out->push_back(y);
            if (fabsf(y) > best) {
                best = fabsf(y);
                arrival = n;
            }
        }
    }
    return arrival;
}

static void TestEchoLoop() {
    bitch::EchoLoop e;
    e.Init(kSampleRate, echo_buffer);
    while (!e.ClearStep(1024)) {
    }
    // Forward: the impulse returns after the echo time (time set before the first Update: no glide)
    e.SetTimeNow(0.3f);
    int at = EchoArrival(e, 0.3f, false);
    CHECK(abs(at - (int)(0.3f * kSampleRate)) <= 2, "echo: back after %d samples (want %d)", at,
          (int)(0.3f * kSampleRate));
    // Time changes glide (~0.5 s): 0.2 s after a change from 0.3 to 0.6 s the time is in between
    for (int b = 0; b < (int)(0.2f * kSampleRate / 48); b++) { // the glide runs per sample, in Read()
        e.Update(0.6f, false, 48);
        for (int k = 0; k < 48; k++) {
            e.Read();
            e.Write(0.f);
        }
    }
    CHECK(e.TimeSeconds() > 0.35f && e.TimeSeconds() < 0.55f, "echo glide: %.3f s after 0.2 s", e.TimeSeconds());
    // Reverse: a repeat played backwards: a rising ramp written comes back falling
    bitch::EchoLoop rev;
    rev.Init(kSampleRate, echo_buffer);
    while (!rev.ClearStep(1024)) {
    }
    rev.SetTimeNow(0.4f);
    std::vector<float> out;
    int falling = 0, rising = 0;
    for (int b = 0; b < (int)(2.f * kSampleRate / 48); b++) {
        rev.Update(0.4f, true, 48);
        for (int k = 0; k < 48; k++) {
            int n = b * 48 + k;
            out.push_back(rev.Read());
            rev.Write(0.25f * (float)(n % 4800) / 4800.f); // rising 100 ms ramps
        }
    }
    for (size_t k = (size_t)(0.5f * kSampleRate); k + 1 < out.size(); k++) {
        float d = out[k + 1] - out[k];
        falling += d < -1e-6f && d > -0.001f;
        rising += d > 1e-6f && d < 0.001f;
    }
    CHECK(falling > 2 * rising, "reverse echo: %d falling steps vs %d rising (want mostly falling)", falling, rising);
    // The low-pass in the loop: 10 kHz is cut well below 1 kHz
    auto tone_through = [](float hz) {
        bitch::EchoLoop t;
        t.Init(kSampleRate, echo_buffer);
        while (!t.ClearStep(1024)) {
        }
        t.SetTimeNow(0.01f);
        std::vector<float> y;
        for (int n = 0; n < 24000; n++) {
            if (n % 48 == 0) t.Update(0.01f, false, 48);
            y.push_back(t.Read());
            t.Write(Sine(hz, 0.25f, n));
        }
        return Rms(Window(y, 0.2f, 0.5f));
    };
    CHECK(Db(tone_through(10000.f) / tone_through(1000.f)) < -3.0, "echo low-pass: 10 kHz only %.1f dB below 1 kHz",
          Db(tone_through(10000.f) / tone_through(1000.f)));
    // ClearStep: bounded, and the line reads exactly 0 afterwards
    for (int n = 0; n < 4800; n++) e.Write(0.5f);
    int steps = 0;
    while (!e.ClearStep(1024)) steps++;
    float left = 0.f;
    for (int n = 0; n < 48000; n++) {
        left = std::max(left, fabsf(e.Read()));
        e.Write(0.f);
    }
    CHECK(steps <= bitch::EchoLoop::kMax / 1024 + 1 && left == 0.f, "echo clear: %d steps, %g left", steps, left);
}
```
Call `TestEchoLoop();` from `main()` (before the summary line).

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh; echo "exit $?"`
Expected: compile error, `'echo_loop.h' file not found`.

- [ ] **Step 3: Write the echo loop**

`engine/echo_loop.h`:
```cpp
#pragma once

#include <cmath>

#include "softclip.h"

namespace bitch {

// The echoes' delay: up to 1 s in a buffer the caller provides (SDRAM). Write() the loop's
// signal (it goes in through a gentle ~6 kHz low-pass and the soft clip), Read() the echo
// (call Read() before Write() each sample). The echo time glides towards its target over
// ~0.5 s, like tape (warping the pitch). Reverse: two read heads, half a window apart and each
// with a sin^2 window, sweep backwards through the last echo time, so each repeat plays
// backwards. ClearStep() zeroes it in bounded steps (call while it isn't in use) and returns
// true once it's clean. Assumes 48 kHz. Update() once per block.
class EchoLoop {
  public:
    enum : int { kMax = 48000 }; // 1 s at 48 kHz

    void Init(float sample_rate, float *buffer) {
        sample_rate_ = sample_rate;
        buffer_ = buffer;
        low_pass_ = 1.f - expf(-6.28318530718f * 6000.f / sample_rate);
        glide_ = 1.f - expf(-1.f / (0.5f * sample_rate));
        cleared_ = 0;
        write_ = 0;
        state_ = 0.f;
        phase_ = 0.f;
        SetTimeNow(0.15f);
    }

    // Jumps to an echo time (no glide), e.g. when the echo starts
    void SetTimeNow(float seconds) { delay_ = target_ = Samples(seconds); }

    void Update(float seconds, bool reverse, int block_size) {
        (void)block_size;
        target_ = Samples(seconds);
        reverse_ = reverse;
    }

    float Read() {
        delay_ += glide_ * (target_ - delay_);
        if (!reverse_) return Tap(delay_);
        // Reverse: each head's distance grows 2 samples per sample (so it reads backwards)
        // from 0 to the echo time over half an echo time
        float out = 0.f;
        float window = 0.5f * delay_;
        for (int head = 0; head < 2; head++) {
            float p = phase_ + 0.5f * (float)head;
            p -= floorf(p);
            float s = sinf(3.14159265359f * p);
            out += s * s * Tap(1.f + 2.f * window * p);
        }
        phase_ += 1.f / window;
        phase_ -= floorf(phase_);
        return out;
    }

    void Write(float x) {
        state_ += low_pass_ * (x - state_);
        buffer_[write_] = SoftClip(state_);
        if (++write_ >= kMax) write_ = 0;
    }

    bool ClearStep(int samples) {
        int end = cleared_ + samples < kMax ? cleared_ + samples : kMax;
        for (int i = cleared_; i < end; i++) buffer_[i] = 0.f;
        cleared_ = end;
        if (cleared_ < kMax) return false;
        cleared_ = 0;
        state_ = 0.f;
        return true;
    }

    float TimeSeconds() const { return delay_ / sample_rate_; }

  private:
    float Samples(float seconds) const {
        float s = seconds * sample_rate_;
        return s < 2.f ? 2.f : (s > (float)(kMax - 2) ? (float)(kMax - 2) : s);
    }
    // The sample written `d` samples ago (fractional, linear), 1 <= d <= kMax - 2
    float Tap(float d) const {
        int whole = (int)d;
        float frac = d - (float)whole;
        int a = write_ - whole, b = a - 1;
        if (a < 0) a += kMax;
        if (b < 0) b += kMax;
        return buffer_[a] + frac * (buffer_[b] - buffer_[a]);
    }

    float *buffer_ = nullptr;
    float sample_rate_ = 48000.f, low_pass_ = 0.f, glide_ = 0.f, state_ = 0.f;
    float delay_ = 7200.f, target_ = 7200.f, phase_ = 0.f;
    bool reverse_ = false;
    int write_ = 0, cleared_ = 0;
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh; echo "exit $?"`
Expected: `All tests passed`, exit 0.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/echo_loop.h sim/test_engine.cpp
git commit -m "Engine: echo loop (tape glide, reverse, stepwise clear)"
```

---

### Task 3: The tape memory, and capture in the engine

**Files:**
- Create: `engine/tape.h`
- Modify: `engine/engine.h`, `engine/engine.cpp`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp` add `#include "tape.h"`, and above `main()`:
```cpp
static bitch::TapeMemory test_tape; // ~1.1 MB: off the stack

// Capture in the engine: Freeze holds the input (the whole chain, dry included, plays it)
static void TestEngineCapture() {
    bitch::Engine e;
    e.Init(kSampleRate, 48, nullptr, &test_tape);
    bitch::Params p;
    p.mix = 0.f; // dry: the capture is at the input, so even the dry signal is frozen
    std::vector<float> l, r;
    int n = 0;
    RunFor(e, p, 1.f, [](int k) { return Sine(440.f, 0.25f, k); }, n, l, r);
    p.capture = bitch::CaptureMode::kFreeze;
    RunFor(e, p, 2.f, [](int) { return 0.f; }, n, l, r); // the input goes silent
    CHECK(Magnitude(Window(l, 2.f, 3.f), 440.f) > 0.1, "engine freeze: the tone didn't hold");
    p.capture = bitch::CaptureMode::kNormal;
    RunFor(e, p, 0.5f, [](int) { return 0.f; }, n, l, r);
    CHECK(Peak(Window(l, 3.2f, 3.5f)) < 1e-4f, "engine capture: still playing after Normal (%g)",
          Peak(Window(l, 3.2f, 3.5f)));
    // Without tape memory: Freeze does nothing
    bitch::Engine bare;
    bare.Init(kSampleRate);
    p.capture = bitch::CaptureMode::kFreeze;
    std::vector<float> bl, br;
    n = 0;
    RunFor(bare, p, 0.5f, [](int k) { return Sine(440.f, 0.25f, k); }, n, bl, br);
    RunFor(bare, p, 0.5f, [](int) { return 0.f; }, n, bl, br);
    CHECK(Peak(Window(bl, 0.6f, 1.f)) < 1e-4f, "no tape: Freeze still froze");
}
```
Call `TestEngineCapture();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh; echo "exit $?"`
Expected: compile error, `'tape.h' file not found`.

- [ ] **Step 3: Add the tape memory and wire capture in**

`engine/tape.h`:
```cpp
#pragma once

#include "capture_loop.h"
#include "echo_loop.h"

namespace bitch {

// The big buffers for the echoes and the capture (about 1.1 MB): the front end keeps this in
// SDRAM and passes it to Engine::Init. Plain arrays, so it stays constant-initialisable (global
// constructors run before the SDRAM is set up); it starts as garbage, which the engine never
// reads before writing (the echo clears its line first, the capture reads only what it recorded).
struct TapeMemory {
    float echo[2][EchoLoop::kMax];
    float capture[2][CaptureLoop::kMax];
};

} // namespace bitch
```

`engine/engine.h`:
- Add `#include "tape.h"`.
- Add to `Params` (after `effect_amount`): `CaptureMode capture = CaptureMode::kNormal; // switch 2`.
- Change `Init` to `void Init(float sample_rate, size_t block_size = 48, PlateReverb *plate = nullptr, TapeMemory *tape = nullptr);` and extend its comment: "tape: the echoes' and the capture's buffers (~1.1 MB, owned by the front end); without it, the echo pads and capture do nothing."
- Extend the class comment: "Capture (switch 2: Normal / Freeze / Loop) sits at the very start, so everything after it hears the captured sound."
- Add private members:
```cpp
    // Switch 2 and the echoes, using the front end's tape memory (if any)
    TapeMemory *tape_ = nullptr;
    CaptureLoop capture_;
```

`engine/engine.cpp`:
- Change the `Init` signature to match, store `tape_ = tape;`, and after the damage `Init`s: `if (tape_) capture_.Init(sample_rate, tape_->capture[0], tape_->capture[1]);`.
- In `Reset()`: `if (tape_) capture_.Reset();`.
- In `Process()`, at the very top of the per-sample loop, replace `float x[2] = {in_l[i], in_r[i]};` with:
```cpp
        float x[2] = {in_l[i], in_r[i]};
        if (tape_) capture_.Process(x[0], x[1]); // switch 2: everything after hears the captured sound
```
- Before the per-sample loop (e.g. right after the damage per-block code): `if (tape_) capture_.Set(params.capture);`.

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh; echo "exit $?"`
Expected: `All tests passed`, exit 0, including `TestEffectsUnbuiltDoNothing` and every earlier test (no tape is passed there, and with switch 2 at Normal the capture passes the input through exactly).

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/tape.h engine/engine.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: tape memory and switch 2 capture (Freeze / Loop)"
```

---

### Task 4: The echoes in the engine

**Files:**
- Modify: `engine/engine.h`, `engine/engine.cpp`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp`, add `bitch::Bit(bitch::kEchoes) | bitch::Bit(bitch::kReverseEchoes) | bitch::Bit(bitch::kPingPongEchoes) |` to `kBuiltEffects`, and above `main()`:
```cpp
// The echo pads: a short tone burst comes back after the echo time, shifted further each time
static void TestEffectEchoes() {
    auto burst = [](int k) { return k < 4800 ? Sine(1000.f, 0.25f, k) : 0.f; }; // 100 ms of 1 kHz
    bitch::Engine e;
    e.Init(kSampleRate, 48, nullptr, &test_tape);
    bitch::Params p;
    p.tune_hz = 100.f;
    p.effect_amount = 0.35f; // echo time 150 ms
    p.effects = bitch::Bit(bitch::kEchoes);
    std::vector<float> l, r;
    int n = 0;
    RunFor(e, p, 0.2f, [](int) { return 0.f; }, n, l, r); // the line clears (~48 ms) before it starts
    l.clear();
    r.clear();
    int start = n;
    RunFor(e, p, 1.f, [&](int k) { return burst(k - start); }, n, l, r);
    // The first repeat (around 150-250 ms) is shifted twice: 1 kHz + 100 (direct) + 100 (repeat)
    auto first = Window(l, 0.16f, 0.24f), second = Window(l, 0.31f, 0.39f);
    CHECK(Magnitude(first, 1200.f) > 3 * Magnitude(first, 1100.f), "echo 1: 1200 Hz %.4f vs 1100 Hz %.4f",
          Magnitude(first, 1200.f), Magnitude(first, 1100.f));
    CHECK(Magnitude(second, 1300.f) > 3 * Magnitude(second, 1200.f), "echo 2: 1300 Hz %.4f vs 1200 Hz %.4f",
          Magnitude(second, 1300.f), Magnitude(second, 1200.f));
    // Without the pad: no repeats
    bitch::Engine plain;
    plain.Init(kSampleRate, 48, nullptr, &test_tape);
    p.effects = 0u;
    std::vector<float> pl, pr;
    n = 0;
    RunFor(plain, p, 1.f, burst, n, pl, pr);
    CHECK(Rms(Window(pl, 0.16f, 0.24f)) < 1e-3f, "no echo pad: repeats anyway (%g)", Rms(Window(pl, 0.16f, 0.24f)));
    // Ping-pong: the repeats cross sides (the left input's burst repeats on the right)
    bitch::Engine pp;
    pp.Init(kSampleRate, 48, nullptr, &test_tape);
    p.effects = bitch::Bit(bitch::kPingPongEchoes);
    p.tune_hz = 0.f;
    float in_l[48], in_r[48], out_l[48], out_r[48];
    std::vector<float> ppl, ppr;
    for (int b = 0; b < (int)(0.2f * kSampleRate / 48); b++) {
        for (int k = 0; k < 48; k++) {
            in_l[k] = 0.f;
            in_r[k] = Sine(5000.f, 0.002f, k);
        }
        pp.Process(in_l, in_r, out_l, out_r, 48, p);
    }
    for (int b = 0; b < (int)(1.f * kSampleRate / 48); b++) {
        for (int k = 0; k < 48; k++) {
            in_l[k] = burst(b * 48 + k);
            in_r[k] = Sine(5000.f, 0.002f, b * 48 + k); // a faint right input, above -60 dBFS so mono
                                                        // detection doesn't copy the left into it
        }
        pp.Process(in_l, in_r, out_l, out_r, 48, p);
        ppl.insert(ppl.end(), out_l, out_l + 48);
        ppr.insert(ppr.end(), out_r, out_r + 48);
    }
    CHECK(Rms(Window(ppr, 0.16f, 0.24f)) > 3 * Rms(Window(ppl, 0.16f, 0.24f)),
          "ping-pong: first repeat right %.4f vs left %.4f", Rms(Window(ppr, 0.16f, 0.24f)),
          Rms(Window(ppl, 0.16f, 0.24f)));
}
```
Call `TestEffectEchoes();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh; echo "exit $?"`
Expected: FAIL lines from `TestEffectEchoes`; `TestEffectsUnbuiltDoNothing` still passes.

- [ ] **Step 3: Wire the echoes in**

`engine/engine.h`: private members after `capture_`:
```cpp
    EchoLoop echo_[2];
    float echo_level_ = 0.f;   // the echo pads' level at the end of the last block (ramped per sample)
    bool echo_running_ = false, echo_clean_ = false; // like the plate: cleared in steps while idle
```

`engine/engine.cpp`:
- In `Init`, with the capture `Init`: `if (tape_) for (int c = 0; c < 2; c++) echo_[c].Init(sample_rate, tape_->echo[c]);`.
- In `Reset()`: `echo_level_ = 0.f; echo_running_ = false; echo_clean_ = false;`.
- In `Process()`, after the capture `Set`:
```cpp
    // The echo pads (any of the three): the shifter's 2 ms feedback grows into an echo of
    // 150 ms (amount 0.35) to 900 ms (amount 1), with at least 50% feedback; each repeat goes
    // through the shifter again. Like the plate, the lines are cleared in steps while idle and
    // the echo starts only once they're clean (a pad held meanwhile waits up to ~100 ms).
    const float echo_want = fmaxf(fmaxf(level_[kEchoes], level_[kReverseEchoes]), level_[kPingPongEchoes]);
    if (tape_ && !echo_running_ && !echo_clean_) {
        bool a = echo_[0].ClearStep(512), b = echo_[1].ClearStep(512);
        echo_clean_ = a && b;
    }
    const float echo_time = 0.15f + 0.75f * EffectRange(amount_radio);
    if (tape_ && !echo_running_ && echo_clean_ && echo_want > 0.f) {
        echo_running_ = true;
        for (EchoLoop &el : echo_) el.SetTimeNow(echo_time);
    }
    const float echo_from = echo_running_ ? echo_level_ : 0.f, echo_to = echo_running_ ? echo_want : 0.f;
    echo_level_ = echo_to;
    if (echo_running_ && echo_from == 0.f && echo_to == 0.f) {
        echo_running_ = false;
        echo_clean_ = false;
    }
    const bool reversing = Has(fx, kReverseEchoes) || level_[kReverseEchoes] > 0.f;
    const bool ping_pong = Has(fx, kPingPongEchoes) || level_[kPingPongEchoes] > 0.f;
    if (echo_running_)
        for (EchoLoop &el : echo_) el.Update(echo_time, reversing, (int)size);
```
  (`amount_radio` is defined earlier in `Process()`; this code must come after it.)
- In the stereo-mode block, after it computes `gain_target`: `if (ping_pong) gain_target[1] = -gain_target[1]; // ping-pong: the right loop shifts the other way`.
- In the per-sample loop, before the per-side loop, add:
```cpp
        const float echo = echo_running_ ? echo_from + t * (echo_to - echo_from) : 0.f;
        float echo_return[2] = {0.f, 0.f};
        if (echo_running_) {
            float a = echo_[0].Read(), b = echo_[1].Read();
            echo_return[0] = ping_pong ? b : a; // ping-pong: each side hears the other's repeats
            echo_return[1] = ping_pong ? a : b;
        }
        const float echo_feedback = echo * fmaxf(fb, 0.5f);
```
- In the per-side loop, change
```cpp
            float shifted = shifter_[c].Process(ringed, shift[c] + drift_hz_[c], fb);
```
  to
```cpp
            float shifted = echo_running_
                                ? shifter_[c].Process(ringed + echo_feedback * echo_return[c], shift[c] + drift_hz_[c],
                                                      fb * (1.f - echo))
                                : shifter_[c].Process(ringed, shift[c] + drift_hz_[c], fb);
            if (echo_running_) echo_[c].Write(shifted);
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh; echo "exit $?"`
Expected: `All tests passed`, exit 0, including the bit-exact test (with the pads off, `echo_running_` stays false and the shifter call is unchanged).

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/engine.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: echo pads (shifted echoes, reverse, ping-pong)"
```

---

### Task 5: Spiral, Chirp and Warble

**Files:**
- Modify: `engine/engine.h`, `engine/engine.cpp`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

In `sim/test_engine.cpp`, add `bitch::Bit(bitch::kSpiralUp) | bitch::Bit(bitch::kSpiralDown) | bitch::Bit(bitch::kChirp) | bitch::Bit(bitch::kWarble) |` to `kBuiltEffects`, and above `main()`:
```cpp
// Spiral up: the sound keeps rising (a Shepard-like pair of shifted voices); spiral down falls
static void TestEffectSpiral() {
    for (int up = 0; up < 2; up++) {
        bitch::Engine e;
        e.Init(kSampleRate);
        bitch::Params p;
        p.effect_amount = 1.f; // 0.5 cycles a second over 0-300 Hz
        p.effects = bitch::Bit(up ? bitch::kSpiralUp : bitch::kSpiralDown);
        std::vector<float> l, r;
        int n = 0;
        RunFor(e, p, 4.f, [](int k) { return Sine(1000.f, 0.25f, k); }, n, l, r);
        // The output's energy sits between 1000 and 1300 Hz (up) or 700 and 1000 Hz (down)
        auto w = Window(l, 1.f, 4.f);
        double above = 0.0, below = 0.0;
        for (float hz = 1050.f; hz <= 1250.f; hz += 50.f) above += Magnitude(w, hz);
        for (float hz = 750.f; hz <= 950.f; hz += 50.f) below += Magnitude(w, hz);
        if (up) CHECK(above > 3 * below, "spiral up: above %.4f vs below %.4f", above, below);
        else CHECK(below > 3 * above, "spiral down: below %.4f vs above %.4f", below, above);
        CHECK(Rms(w) > 0.1f, "spiral: level %.3f", Rms(w));
    }
}

// Chirp: loud notes push the shift up (silence: no shift)
static void TestEffectChirp() {
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.effect_amount = 0.35f; // up to +300 Hz at full level
    p.effects = bitch::Bit(bitch::kChirp);
    auto loud = ShiftTrace(e, p, 0.5f, [](int k) { return Sine(200.f, 0.5f, k); });
    CHECK(loud.back() > 200.f && loud.back() < 320.f, "chirp: %.0f Hz on a full-level tone (want ~300)", loud.back());
    auto quiet = ShiftTrace(e, p, 1.5f, [](int) { return 0.f; });
    CHECK(fabsf(quiet.back()) < 5.f, "chirp: %.1f Hz after 1.5 s of silence", quiet.back());
}

// Warble: the shift wobbles around TUNE, by up to 20 Hz (amount 0.35)
static void TestEffectWarble() {
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.tune_hz = 100.f;
    p.effect_amount = 0.35f;
    p.effects = bitch::Bit(bitch::kWarble);
    auto trace = ShiftTrace(e, p, 4.f, [](int) { return 0.f; });
    float lo = 1e9f, hi = -1e9f;
    for (size_t k = 500; k < trace.size(); k++) {
        lo = std::min(lo, trace[k]);
        hi = std::max(hi, trace[k]);
    }
    CHECK(hi - lo > 15.f && hi - lo < 60.f && lo > 60.f && hi < 140.f, "warble at 0.35: %.1f-%.1f Hz", lo, hi);
}
```
Call the three from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh; echo "exit $?"`
Expected: FAIL lines from the three tests.

- [ ] **Step 3: Add them to the engine**

`engine/engine.h`: private members after the echo members:
```cpp
    // Motion pads (page 1)
    Shifter spiral_[2];          // the spiral's second voice, per side
    float spiral_phase_ = 0.f;   // 0-1 around the spiral's cycle
    float spiral_dir_ = 1.f;     // +1 up, -1 down (the last spiral pad touched)
    float spiral_level_ = 0.f;   // at the end of the last block (ramped per sample)
    bool spiral_running_ = false;
    float chirp_envelope_ = 0.f, chirp_attack_ = 0.f, chirp_release_ = 0.f;
    float warble_phase_ = 0.f, warble_hz_ = 0.f; // warble_hz_: the last block's value (ramped)
    Wander warble_wander_;
    bool spiral_up_was_ = false, spiral_down_was_ = false;
```

`engine/engine.cpp`:
- In `Init`: `for (Shifter &s : spiral_) s.Init(sample_rate); chirp_attack_ = 1.f - expf(-1.f / (0.002f * sample_rate)); chirp_release_ = 1.f - expf(-1.f / (0.15f * sample_rate)); warble_wander_.Init(block_rate, 53u);`.
- In `Reset()`: `for (Shifter &s : spiral_) s.Reset(); spiral_phase_ = 0.f; spiral_level_ = 0.f; spiral_running_ = false; chirp_envelope_ = 0.f; warble_phase_ = 0.f; warble_hz_ = 0.f; spiral_up_was_ = spiral_down_was_ = false;`.
- In `Process()`, after the echo per-block code:
```cpp
    // Spiral: two voices (the main shifter and spiral_[c]) whose extra shifts ramp 0 -> 300 Hz
    // (down: 0 -> -300 Hz) half a cycle apart, each weighted sin^2 of its ramp's phase, so each
    // ramp restarts silently: an endless rise (or fall). 0.05 (amount 0.35) to 0.5 cycles/s.
    // The last spiral pad touched sets the direction.
    const bool up = Has(fx, kSpiralUp), down = Has(fx, kSpiralDown);
    if (up && !spiral_up_was_) spiral_dir_ = 1.f;
    if (down && !spiral_down_was_) spiral_dir_ = -1.f;
    spiral_up_was_ = up;
    spiral_down_was_ = down;
    const float spiral_want = fmaxf(level_[kSpiralUp], level_[kSpiralDown]);
    if (spiral_want > 0.f && !spiral_running_) {
        for (Shifter &s : spiral_) s.Reset();
        spiral_running_ = true;
    }
    const float spiral_from = spiral_running_ ? spiral_level_ : 0.f, spiral_to = spiral_running_ ? spiral_want : 0.f;
    spiral_level_ = spiral_to;
    if (spiral_running_ && spiral_from == 0.f && spiral_to == 0.f) spiral_running_ = false;
    float spiral_a = 0.f, spiral_b = 0.f, weight_a = 1.f, weight_b = 0.f;
    if (spiral_running_) {
        spiral_phase_ += (0.05f + 0.45f * EffectRange(amount_radio)) * (float)size / sample_rate_;
        spiral_phase_ -= floorf(spiral_phase_);
        float pb = spiral_phase_ + 0.5f;
        pb -= floorf(pb);
        spiral_a = spiral_dir_ * 300.f * spiral_phase_;
        spiral_b = spiral_dir_ * 300.f * pb;
        float sa = sinf(3.14159265359f * spiral_phase_), sb = sinf(3.14159265359f * pb);
        weight_a = sa * sa;
        weight_b = sb * sb;
    }
    // Warble: a sine LFO plus a little randomness, 0.5-8 Hz, 20-400 Hz deep (both rising with
    // the amount); ramped across the block
    const float warble_level = level_[kWarble];
    const float warble_from = warble_hz_;
    if (warble_level > 0.f) {
        float u = EffectRange(amount_radio);
        warble_phase_ += (0.5f + 7.5f * u) * (float)size / sample_rate_;
        warble_phase_ -= floorf(warble_phase_);
        float lfo = 0.8f * sinf(6.28318530718f * warble_phase_) + 0.2f * warble_wander_.Next(2.f);
        warble_hz_ = warble_level * (20.f + 380.f * u) * lfo;
    } else {
        warble_hz_ = 0.f;
    }
    const float warble_to = warble_hz_;
    // Chirp: up to +300 Hz (amount 0.35) to +1500 Hz (amount 1) at full input level
    const float chirp_level = level_[kChirp];
    const float chirp_depth = chirp_level * (300.f + 1200.f * EffectRange(amount_radio));
```
- In the per-sample loop, after `float x[2] = ...; if (tape_) capture_.Process(...)`, add the chirp's envelope (on the captured input):
```cpp
        float motion = 0.f; // Chirp + Warble, added to the shift before the stereo gains
        if (chirp_level > 0.f || chirp_envelope_ > 0.f) {
            float level = fmaxf(fabsf(x[0]), fabsf(x[1]));
            chirp_envelope_ += (level > chirp_envelope_ ? chirp_attack_ : chirp_release_) * (level - chirp_envelope_);
            if (chirp_level == 0.f && chirp_envelope_ < 1e-5f) chirp_envelope_ = 0.f;
            motion += chirp_depth * fminf(chirp_envelope_ / 0.5f, 1.f);
        }
        if (warble_from != 0.f || warble_to != 0.f) motion += warble_from + (float)(i + 1) * inv_size * (warble_to - warble_from);
```
  and change the shift computation
```cpp
            shift[c] = shift_hz_ * side_gain_[c];
```
  to
```cpp
            shift[c] = (motion != 0.f ? shift_hz_ + motion : shift_hz_) * side_gain_[c];
```
  (keeping the no-motion arithmetic exactly as before). `inv_size` already exists in `Process()`; if the declaration order puts it after this point, move the motion code below it.
- In the per-side loop, replace the shifter call's result with the spiral pair when it runs. After the echo change from Task 4, the call is the `float shifted = ...;` statement; wrap it:
```cpp
            float shifted;
            if (!spiral_running_) {
                shifted = /* the Task 4 expression, unchanged */;
            } else {
                const float spiral = spiral_from + t * (spiral_to - spiral_from);
                float in = echo_running_ ? ringed + echo_feedback * echo_return[c] : ringed;
                float own_fb = echo_running_ ? fb * (1.f - echo) : fb;
                float side = side_gain_[c];
                float a = shifter_[c].Process(in, shift[c] + drift_hz_[c] + spiral * spiral_a * side, own_fb);
                float b = spiral_[c].Process(in, shift[c] + drift_hz_[c] + spiral * spiral_b * side, own_fb);
                shifted = a * (1.f - spiral + spiral * weight_a) + b * spiral * weight_b;
            }
            if (echo_running_) echo_[c].Write(shifted);
```
  (Write the Task 4 expression out in full in the first branch; don't leave the comment.)

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh; echo "exit $?"`
Expected: `All tests passed`, exit 0, including the bit-exact test (only Chaos is now unbuilt in the engine).

- [ ] **Step 5: Build and commit**

Run: `make -C ~/local_files/daisy/bit-chifter` (touch `engine/engine.cpp` first) — expected: no errors or warnings; report the DTCMRAM/SRAM lines (the engine grows only by the two extra shifters, ~1 KB).
```bash
cd ~/local_files/daisy/bit-chifter
git add engine/engine.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: spiral up/down, chirp and warble"
```

(Review checkpoint: the engine tasks are done; review before Task 6.)

---

### Task 6: The render tool

**Files:**
- Modify: `sim/render.cpp`, `sim/render.sh`

- [ ] **Step 1: Tape memory and capture options**

In `sim/render.cpp`:
- `static bitch::TapeMemory tape;` next to the static plate, and `engine.Init((float)in.sample_rate, 48, &plate, &tape);`.
- Two options parsed like `every=`: `freeze_at=s` and `loop_at=s` (seconds; NAN = never). In the block loop, before `engine.Process(...)`: set `p.capture` to `kFreeze` from `freeze_at`, `kLoop` from `loop_at` (whichever time has passed most recently wins), else `kNormal`.
- Add `[freeze_at=s] [loop_at=s]` to the usage comment and message, and ` capture freeze@%g loop@%g` to the final printf when set. In `sim/render.sh`, add them to the usage comment.

- [ ] **Step 2: Render**

```bash
cd ~/local_files/daisy/bit-chifter/sim/out
../render.sh voice.wav t6_echoes.wav tune=120 effects=echoes amount=0.5
../render.sh voice.wav t6_reverse.wav tune=-80 effects=reverse_echoes amount=0.7 reverb=0.4
../render.sh voice.wav t6_pingpong.wav tune=150 fb=0.7 effects=ping_pong_echoes amount=0.4
../render.sh drone.wav t6_spiral_up.wav effects=spiral_up amount=0.8
../render.sh voice.wav t6_spiral_down.wav effects=spiral_down amount=0.5 fb=0.4
../render.sh voice.wav t6_chirp.wav effects=chirp amount=0.6
../render.sh drone.wav t6_warble.wav effects=warble amount=0.6
../render.sh voice.wav t6_freeze.wav freeze_at=2.5 tune=60 effects=spiral_up amount=0.5
../render.sh voice.wav t6_loop.wav loop_at=3 effects=stutter,echoes amount=0.6
```
Expected: nine `Wrote …` lines, no compiler warnings.

- [ ] **Step 3: User listens, then commit**

Send the renders with a line each. After they're happy:
```bash
cd ~/local_files/daisy/bit-chifter
git add sim/render.cpp sim/render.sh
git commit -m "Render tool: tape memory, freeze_at and loop_at"
```

---

### Task 7: Firmware

**Files:**
- Modify: `simple_touch.cpp`, `README.md`

- [ ] **Step 1: Tape memory and switch 2**

In `simple_touch.cpp`:
- Next to the plate: `DSY_SDRAM_BSS bitch::TapeMemory tape; // the echoes' and capture's buffers (~1.1 MB; see the plate's note)`.
- `engine.Init(hw.AudioSampleRate(), hw.AudioBlockSize(), &plate, &tape);`.
- A second switch: `Switch3 capture_switch; // switch 2 (right): Normal / Freeze / Loop; reads mirror-image`, `capture_switch.Init(seed::D7, seed::D6);` next to the page switch, and `volatile int capture = 0;` set in the main loop: physically left (reads `POS_RIGHT`) = Normal (0), centre = Freeze (1), right (`POS_LEFT`) = Loop (2).
- In `AudioCallback`: `p.capture = (bitch::CaptureMode)capture;`.
- In the report's second line add ` capture %s` with `{"normal", "freeze", "loop"}[capture]`.
- Update the top comment: switch 2 is Normal / Freeze / Loop; every pad is built.

- [ ] **Step 2: Build**

Run: `make -C ~/local_files/daisy/bit-chifter` — expected: no errors or warnings; SDRAM about 1.3 MB.

- [ ] **Step 3: User flashes and plays it**

Page 1 (switch 1 left): big left Echoes, left edge Reverse echoes, centre strip Ping-pong, big right Spiral up, right edge Spiral down, top left Chirp, top right Warble. Switch 2: centre = Freeze, right = Loop, left = Normal. Try Freeze/Loop with pads on top. Check the CPU with echoes + spiral + a few radio/damage pads.

- [ ] **Step 4: Document and commit (after the user confirms)**

README: add the switch 2 row, complete the Pads row (all pads built), update the SDRAM note.
```bash
cd ~/local_files/daisy/bit-chifter
git add simple_touch.cpp README.md
git commit -m "Simple Touch firmware: page 1 (echoes, spirals, chirp, warble) and switch 2 (Freeze / Loop)"
```
