# bit-chifter Strange Pages Step 1 (Pages) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the pads and switch 1 work as three pages of effects (tap latches, hold is momentary, hold TOU clears all), with the 12 cheap effects sounding: feedback burst, dial spin up/down, static burst, squelch, signal fade-out, filter slam, drift to maximum, ring 100%, carrier ×2, kill dry, hold drift.

**Architecture:** `engine/effects.h` names the 30 effects (an `Effect` enum: bit *i* of `Params::effects`) and maps (page, pad channel) to an effect. `ui/pad_pages.h` is the hardware-free pad logic (touch masks + time + page → effect bits and events), tested on the Mac. The engine gets `Params::effects` and `Params::effect_amount` (the front end passes `EffectAmount(reception)` = 0.35–1); each effect fades in and out over ~30 ms (a per-block level per effect) and is an exact pass-through when off. Effects not built yet (echoes, spirals, scrambler, …, chaos) do nothing. `simple_touch.cpp` only reads the switch and pads and calls the pad logic.

**Tech Stack:** C++17 (also gnu++14 for the firmware), libDaisy v8.1, Apple clang for `sim/`.

**Spec:** `docs/superpowers/specs/2026-10-13-bit-chifter-strange-pages-design.md` (Controls, Pad layout, the effects listed above, Build order step 1).

**Conventions:**
- Project root `~/local_files/daisy/bit-chifter`, git, master (public on GitHub; don't push, the user asks for that). No `Co-Authored-By` or other attribution lines in commits.
- Tests: `sim/run_tests.sh` (prints `All tests passed`). Helpers in `sim/test_engine.cpp`: `CHECK(cond, fmt, ...)`, `kSampleRate` (48000.f), `kPi`, `Magnitude(x, freq)`, `Db(ratio)`, `Window(x, from_s, to_s)`, `ShiftTrace(engine, params, seconds, signal)` (the left shifter's frequency at the end of each 48-sample block), `Peak(x)`, `Energy(x)`. New test functions go above `main()` and are called from it.
- Engine code: single precision only (`sinf`, `1.f`), nothing allocating in `Process()`; the firmware builds as gnu++14 (no odr-used `static constexpr` members: use enums or functions, as below).
- If a test fails after implementing, don't change tolerances or the given algorithm to make it pass: report the printed numbers.
- Hardware steps: the user listens to renders before the render-tool commit, and flashes and confirms before the firmware commit.

## File structure

| File | Responsibility |
|---|---|
| `engine/effects.h` | `Effect` enum, `Bit()`, `Has()`, `PadEffect(page, channel)`, `EffectName()` |
| `engine/mapping.h` | `EffectAmount(reception)` |
| `ui/pad_pages.h` | `PadPages`: tap/hold latching, pages, TOU tap (stereo step) and hold (clear all) |
| `engine/blips.h` | `Start(count, low)`: a low blip for clear-all |
| `engine/engine.h`, `engine/engine.cpp` | `Params::effects`, `Params::effect_amount`; the 12 effects |
| `sim/test_engine.cpp` | tests |
| `sim/render.cpp`, `sim/render.sh` | `effects=`, `amount=`, `every=` options |
| `simple_touch.cpp`, `README.md` | switch 1 → page, pads → `PadPages`, effects into the engine; README controls |

---

### Task 1: The effects list and the amount

**Files:**
- Create: `engine/effects.h`
- Modify: `engine/mapping.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing test**

In `sim/test_engine.cpp` add `#include "effects.h"` with the other engine includes, and above `main()`:
```cpp
static void TestEffectsMapping() {
    using namespace bitch;
    CHECK(PadEffect(0, 4) == kEchoes, "page 1 big left pad: %d", PadEffect(0, 4));
    CHECK(PadEffect(0, 6) == kSpiralUp, "page 1 big right pad: %d", PadEffect(0, 6));
    CHECK(PadEffect(0, 1) == kFeedbackBurst, "page 1 top centre: %d", PadEffect(0, 1));
    CHECK(PadEffect(0, 8) == kSpinDown && PadEffect(0, 9) == kSpinUp, "page 1 bottom pads");
    CHECK(PadEffect(1, 0) == kStaticBurst && PadEffect(1, 2) == kSquelch, "page 2 top pads");
    CHECK(PadEffect(1, 8) == kFadeOut && PadEffect(1, 9) == kFilterSlam && PadEffect(1, 1) == kDriftMax,
          "page 2 bottom and top centre pads");
    CHECK(PadEffect(2, 0) == kRing100 && PadEffect(2, 2) == kCarrierX2 && PadEffect(2, 8) == kKillDry,
          "page 3 pads");
    CHECK(PadEffect(2, 9) == kChaos && PadEffect(2, 1) == kHoldDrift, "page 3 chaos and hold drift");
    CHECK(PadEffect(3, 0) == -1 && PadEffect(0, 10) == -1 && PadEffect(-1, 0) == -1, "out of range");
    uint32_t seen = 0;
    for (int page = 0; page < 3; page++)
        for (int ch = 0; ch < 10; ch++) seen |= 1u << PadEffect(page, ch);
    CHECK(seen == (1u << kEffectCount) - 1 && kEffectCount == 30, "pads cover %08x of the 30 effects", seen);
    CHECK(strcmp(EffectName(kSquelch), "squelch") == 0 && strcmp(EffectName(kHoldDrift), "hold_drift") == 0,
          "effect names");
    CHECK(Has(Bit(kRing100) | Bit(kKillDry), kKillDry) && !Has(Bit(kRing100), kKillDry), "Has/Bit");
    CHECK(EffectAmount(0.f) == 0.35f && EffectAmount(1.f) == 1.f && fabsf(EffectAmount(0.5f) - 0.675f) < 1e-6f,
          "effect amount %.3f %.3f", EffectAmount(0.f), EffectAmount(1.f));
}
```
Add `#include <cstring>` at the top of the file (for `strcmp`) and call `TestEffectsMapping();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: compile error, `'effects.h' file not found`.

- [ ] **Step 3: Write the effects list and the amount**

`engine/effects.h`:
```cpp
#pragma once

#include <cstdint>

namespace bitch {

// The 30 pad effects of the strange pages (spec: 2026-10-13-bit-chifter-strange-pages-design.md),
// page by page, each page in the order of its pads from the big ones down (see PadEffect).
// Bit i of Params::effects turns Effect i on.
enum Effect : int {
    // Page 1: Shift & time
    kEchoes, kSpiralUp, kReverseEchoes, kSpiralDown, kPingPongEchoes,
    kChirp, kWarble, kSpinDown, kSpinUp, kFeedbackBurst,
    // Page 2: Radio
    kScrambler, kCarriers, kHoppingScrambler, kMorseChimes, kJammer,
    kStaticBurst, kSquelch, kFadeOut, kFilterSlam, kDriftMax,
    // Page 3: Digital & damage
    kBitcrush, kStutter, kRateCrush, kDropouts, kCodecWarble,
    kRing100, kCarrierX2, kKillDry, kChaos, kHoldDrift,
    kEffectCount
};

inline uint32_t Bit(Effect e) { return 1u << e; }
inline bool Has(uint32_t effects, Effect e) { return (effects >> e) & 1u; }

// The effect on pad `channel` (0-9, the MPR121 channels in the README's pad map) of `page`
// (0-2), or -1. The big pads (4, 6) get each page's first two effects, the narrow top
// centre (1, hard to reach) the last.
inline int PadEffect(int page, int channel) {
    static const int kPosition[10] = {5, 9, 6, 2, 0, 4, 1, 3, 7, 8}; // channel -> place in the page
    if (page < 0 || page > 2 || channel < 0 || channel > 9) return -1;
    return page * 10 + kPosition[channel];
}

// Short names for the render tool and reports
inline const char *EffectName(int e) {
    static const char *kNames[kEffectCount] = {
        "echoes",    "spiral_up", "reverse_echoes",    "spiral_down",  "ping_pong_echoes",
        "chirp",     "warble",    "spin_down",         "spin_up",      "feedback_burst",
        "scrambler", "carriers",  "hopping_scrambler", "morse_chimes", "jammer",
        "static_burst", "squelch", "fade_out",         "filter_slam",  "drift_max",
        "bitcrush",  "stutter",   "rate_crush",        "dropouts",     "codec_warble",
        "ring_100",  "carrier_x2", "kill_dry",         "chaos",        "hold_drift"};
    return e >= 0 && e < kEffectCount ? kNames[e] : "?";
}

} // namespace bitch
```
In `engine/mapping.h`, after `ReceptionAmount`:
```cpp
// Reception amount -> the pad effects' amount: clearly there with the fader at the bottom
// (0.35), extreme at the top (1)
inline float EffectAmount(float reception) { return 0.35f + 0.65f * fminf(fmaxf(reception, 0.f), 1.f); }
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/effects.h engine/mapping.h sim/test_engine.cpp
git commit -m "Engine: the 30 pad effects and their pad layout"
```

---

### Task 2: The pad logic

**Files:**
- Create: `ui/pad_pages.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing test**

In `sim/test_engine.cpp` add `#include "../ui/pad_pages.h"` after the engine includes, and above `main()`:
```cpp
static void TestPadPages() {
    using namespace bitch;
    PadPages pp;
    uint32_t t = 1000;
    auto step = [&](uint32_t ms, uint32_t touched, int page) {
        t += ms;
        pp.Update(t, touched, page);
    };
    // A tap latches; another tap unlatches; the effect starts at the touch
    step(10, 1u << 4, 0);
    CHECK(pp.Effects() == Bit(kEchoes), "tap: not on at the touch (%08x)", pp.Effects());
    step(100, 0, 0);
    CHECK(pp.Effects() == Bit(kEchoes) && pp.Latched() == Bit(kEchoes), "tap: not latched (%08x)", pp.Effects());
    step(10, 1u << 4, 0);
    CHECK(pp.Effects() == Bit(kEchoes), "second tap: off at the touch");
    step(100, 0, 0);
    CHECK(pp.Effects() == 0, "second tap: still on (%08x)", pp.Effects());
    // A hold is momentary
    step(10, 1u << 8, 1);
    CHECK(pp.Effects() == Bit(kFadeOut), "hold: not on (%08x)", pp.Effects());
    step(500, 1u << 8, 1);
    CHECK(pp.Effects() == Bit(kFadeOut), "hold: dropped while held");
    step(10, 0, 1);
    CHECK(pp.Effects() == 0, "hold: still on after release (%08x)", pp.Effects());
    // Holding a latched effect turns it off at the release
    step(10, 1u << 0, 2);
    step(50, 0, 2);
    CHECK(pp.Latched() == Bit(kRing100), "latch ring 100%%");
    step(10, 1u << 0, 2);
    step(400, 1u << 0, 2);
    CHECK(pp.Effects() == Bit(kRing100), "hold on latched: off while held");
    step(10, 0, 2);
    CHECK(pp.Effects() == 0, "hold on latched: still on after release (%08x)", pp.Effects());
    // Latches survive page changes; the same pad on another page is another effect
    step(10, 1u << 6, 0);
    step(50, 0, 0);
    step(10, 0, 2);
    CHECK(pp.Effects() == Bit(kSpiralUp), "page change dropped a latch (%08x)", pp.Effects());
    step(10, 1u << 6, 2);
    step(50, 0, 2);
    CHECK(pp.Effects() == (Bit(kSpiralUp) | Bit(kStutter)), "two pages' latches (%08x)", pp.Effects());
    // A held pad keeps the effect of the page it was touched on
    step(10, 1u << 3, 0);
    step(100, 1u << 3, 1);
    CHECK(pp.Effects() == (Bit(kSpiralUp) | Bit(kStutter) | Bit(kReverseEchoes)), "held pad changed page (%08x)",
          pp.Effects());
    step(300, 0, 1);
    CHECK(pp.Effects() == (Bit(kSpiralUp) | Bit(kStutter)), "held pad: still on after release (%08x)", pp.Effects());
    // TOU tap: one stereo step, on the release
    step(10, 1u << 10, 0);
    CHECK(pp.TakeStereoSteps() == 0, "TOU: stepped at the touch");
    step(100, 0, 0);
    CHECK(pp.TakeStereoSteps() == 1 && !pp.TakeCleared(), "TOU tap: no stereo step");
    // TOU hold 1 s: clears the latches (once), no stereo step; a held pad stays on
    for (int i = 0; i < 110; i++) step(10, (1u << 10) | (1u << 5), 0);
    CHECK(pp.TakeCleared() && !pp.TakeCleared(), "TOU hold: not cleared exactly once");
    CHECK(pp.Latched() == 0 && pp.Effects() == Bit(kPingPongEchoes), "TOU hold: left %08x", pp.Effects());
    step(10, 0, 0);
    CHECK(pp.TakeStereoSteps() == 0 && pp.Effects() == 0, "TOU hold: stepped the stereo mode, or left %08x",
          pp.Effects());
}
```
Call `TestPadPages();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: compile error, `'../ui/pad_pages.h' file not found`.

- [ ] **Step 3: Write the pad logic**

`ui/pad_pages.h`:
```cpp
#pragma once

#include <cstdint>

#include "../engine/effects.h"

namespace bitch {

// The strange pages' pad logic, without hardware (tested on the Mac): call Update() every
// few milliseconds with the time, the touched pad channels (bit per MPR121 channel) and the
// page (0-2, from switch 1).
// - Pads 0-9: a tap (released within 0.3 s) latches the pad's effect on, or off if it was
//   latched; a hold (0.3 s or more) is momentary (off at the release, even if it was
//   latched). Either way the effect starts at the touch. A held pad keeps the effect of the
//   page it was touched on; latches stay on across page changes.
// - TOU (channel 10): a tap steps the stereo mode (counted on the release, so a hold
//   doesn't); holding it 1 s clears every latched effect (held pads stay on until released).
class PadPages {
  public:
    enum : uint32_t { kTapMs = 300, kClearMs = 1000 };
    enum : int { kPads = 10, kStereoPad = 10 };

    void Update(uint32_t now_ms, uint32_t touched, int page) {
        for (int ch = 0; ch < kPads; ch++) {
            bool on = (touched >> ch) & 1u, was = (was_touched_ >> ch) & 1u;
            if (on && !was) {
                int e = PadEffect(page, ch);
                if (e < 0) continue;
                effect_[ch] = e;
                down_ms_[ch] = now_ms;
                was_latched_[ch] = (latched_ >> e) & 1u;
                held_ |= 1u << e;
                touched_[ch] = true;
            } else if (!on && was && touched_[ch]) {
                uint32_t bit = 1u << effect_[ch];
                if (now_ms - down_ms_[ch] < kTapMs && !was_latched_[ch]) {
                    latched_ |= bit;
                } else {
                    latched_ &= ~bit;
                }
                held_ &= ~bit;
                touched_[ch] = false;
            }
        }
        bool on = (touched >> kStereoPad) & 1u, was = (was_touched_ >> kStereoPad) & 1u;
        if (on && !was) {
            stereo_down_ms_ = now_ms;
            stereo_cleared_ = false;
        } else if (on && !stereo_cleared_ && now_ms - stereo_down_ms_ >= kClearMs) {
            latched_ = 0;
            stereo_cleared_ = true;
            cleared_ = true;
        } else if (!on && was && !stereo_cleared_) {
            stereo_steps_++;
        }
        was_touched_ = touched;
    }

    uint32_t Effects() const { return latched_ | held_; } // bits for Params::effects
    uint32_t Latched() const { return latched_; }
    // Events since the last call
    int TakeStereoSteps() {
        int n = stereo_steps_;
        stereo_steps_ = 0;
        return n;
    }
    bool TakeCleared() {
        bool c = cleared_;
        cleared_ = false;
        return c;
    }

  private:
    uint32_t latched_ = 0, held_ = 0, was_touched_ = 0;
    uint32_t down_ms_[kPads] = {};
    int effect_[kPads] = {};          // the effect each touched pad turned on
    bool was_latched_[kPads] = {};    // that effect was latched when the pad was touched
    bool touched_[kPads] = {};        // the pad's touch was taken (it had an effect)
    uint32_t stereo_down_ms_ = 0;
    bool stereo_cleared_ = false;     // this TOU touch already cleared the latches
    int stereo_steps_ = 0;
    bool cleared_ = false;
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add ui/pad_pages.h sim/test_engine.cpp
git commit -m "Pad logic: pages, tap to latch, hold for momentary, TOU hold clears"
```

---

### Task 3: A low blip

**Files:**
- Modify: `engine/blips.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing test**

Above `main()` in `sim/test_engine.cpp`:
```cpp
static void TestLowBlip() {
    bitch::Blips b;
    b.Init(kSampleRate);
    b.Start(1, true);
    std::vector<float> x;
    for (int n = 0; n < (int)(0.2f * kSampleRate); n++) x.push_back(b.Process());
    auto beep = Window(x, 0.f, 0.06f);
    CHECK(Magnitude(beep, 500.f) > 5 * Magnitude(beep, 1500.f), "low blip: 500 Hz %.4f vs 1500 Hz %.4f",
          Magnitude(beep, 500.f), Magnitude(beep, 1500.f));
    b.Start(1);
    x.clear();
    for (int n = 0; n < (int)(0.2f * kSampleRate); n++) x.push_back(b.Process());
    beep = Window(x, 0.f, 0.06f);
    CHECK(Magnitude(beep, 1500.f) > 5 * Magnitude(beep, 500.f), "normal blip after a low one: not 1500 Hz");
}
```
Call `TestLowBlip();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: compile error, `too many arguments to function call` (for `Start(1, true)`).

- [ ] **Step 3: Add the low blip**

In `engine/blips.h`:
- Update the class comment's first sentence to: "Short confirmation beeps (the board has no screen): Start(n) plays n beeps of a 1.5 kHz tone (Start(n, true): 500 Hz), 60 ms each with 90 ms gaps, at about -14 dBFS."
- In `Init`, replace `step_ = 1500.f / sample_rate;` with `sample_rate_ = sample_rate;` and `step_ = 1500.f / sample_rate;`.
- Replace `Start` with:
```cpp
    // Start() may be called from the main loop while Process() runs in the audio callback.
    // The count and the pitch travel in one atomic word.
    void Start(int count, bool low = false) { request_.store(count | (low ? kLow : 0), std::memory_order_relaxed); }
```
- In `Process`, replace the `if (r > 0) { ... }` block with:
```cpp
        if (r > 0) {
            remaining_ = r & ~kLow;
            step_ = ((r & kLow) ? 500.f : 1500.f) / sample_rate_;
            position_ = 0;
            phase_ = 0.f;
        }
```
- Add to the private section: `enum : int { kLow = 0x100 };` and `float sample_rate_ = 48000.f;`.

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed` (including the existing `TestBlips`).

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/blips.h sim/test_engine.cpp
git commit -m "Blips: a low blip (for clear-all)"
```

---

### Task 4: The effects in the engine (levels, and the parameter effects)

Feedback burst, ring 100%, carrier ×2, kill dry, filter slam: each changes a target that the engine already glides (or, for the feedback, is blended by the effect's own ~30 ms level).

**Files:**
- Modify: `engine/engine.h`
- Modify: `engine/engine.cpp`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

Above `main()` in `sim/test_engine.cpp`:
```cpp
static float Sine(float hz, float amplitude, int n) { return amplitude * sinf(2 * kPi * hz * (float)n / kSampleRate); }

// Runs an engine for `seconds` on a signal (of the running sample count n) into both
// inputs, appending both outputs
template <class Signal>
static void RunFor(bitch::Engine &e, const bitch::Params &p, float seconds, Signal signal, int &n,
                   std::vector<float> &l, std::vector<float> &r) {
    float in[48], ol[48], orr[48];
    for (int b = 0; b < (int)(seconds * kSampleRate / 48); b++) {
        for (int k = 0; k < 48; k++) in[k] = signal(n + k);
        e.Process(in, in, ol, orr, 48, p);
        l.insert(l.end(), ol, ol + 48);
        r.insert(r.end(), orr, orr + 48);
        n += 48;
    }
}

static float Rms(const std::vector<float> &x) { return x.empty() ? 0.f : sqrtf((float)(Energy(x) / x.size())); }

// The effects built in this step
static const uint32_t kBuiltEffects =
    bitch::Bit(bitch::kFeedbackBurst) | bitch::Bit(bitch::kSpinUp) | bitch::Bit(bitch::kSpinDown) |
    bitch::Bit(bitch::kStaticBurst) | bitch::Bit(bitch::kSquelch) | bitch::Bit(bitch::kFadeOut) |
    bitch::Bit(bitch::kFilterSlam) | bitch::Bit(bitch::kDriftMax) | bitch::Bit(bitch::kRing100) |
    bitch::Bit(bitch::kCarrierX2) | bitch::Bit(bitch::kKillDry) | bitch::Bit(bitch::kHoldDrift);

// Pads whose effects come in later steps change nothing (bit for bit)
static void TestEffectsUnbuiltDoNothing() {
    bitch::Params p;
    p.tune_hz = 120.f;
    p.feedback = 0.5f;
    p.reception = 0.5f;
    p.ring = 0.3f;
    p.mix = 0.7f;
    uint32_t seed = 3;
    auto signal = [&seed](int n) {
        seed = seed * 1664525u + 1013904223u;
        return Sine(330.f, 0.2f, n) + ((seed >> 8) / 8388608.f - 1.f) * 0.05f;
    };
    bitch::Engine a, b;
    a.Init(kSampleRate);
    b.Init(kSampleRate);
    std::vector<float> al, ar, bl, br;
    int n = 0;
    seed = 3;
    RunFor(a, p, 2.f, signal, n, al, ar);
    p.effects = ((1u << bitch::kEffectCount) - 1) & ~kBuiltEffects;
    n = 0;
    seed = 3;
    RunFor(b, p, 2.f, signal, n, bl, br);
    CHECK(al == bl && ar == br, "unbuilt effects changed the sound");
}

static void TestEffectFeedbackBurst() {
    float tail[2];
    for (int on = 0; on < 2; on++) {
        bitch::Engine e;
        e.Init(kSampleRate);
        bitch::Params p;
        p.tune_hz = 100.f;
        p.effects = on ? bitch::Bit(bitch::kFeedbackBurst) : 0u;
        std::vector<float> l, r;
        int n = 0;
        RunFor(e, p, 0.5f, [](int k) { return Sine(440.f, 0.3f, k); }, n, l, r);
        RunFor(e, p, 0.5f, [](int) { return 0.f; }, n, l, r);
        tail[on] = Rms(Window(l, 0.55f, 0.6f)); // (the wet path rings down for ~100 ms even without feedback)
    }
    CHECK(tail[1] > 30 * tail[0], "feedback burst: tail %g, without it %g", tail[1], tail[0]);
}

static void TestEffectRingAndCarrier() {
    auto run = [](uint32_t effects, float ring, float carrier) {
        bitch::Engine e;
        e.Init(kSampleRate);
        bitch::Params p;
        p.ring = ring;
        p.carrier_hz = carrier;
        p.effects = effects;
        std::vector<float> l, r;
        int n = 0;
        RunFor(e, p, 1.f, [](int k) { return Sine(1000.f, 0.25f, k); }, n, l, r);
        return Window(l, 0.5f, 1.f);
    };
    auto w = run(bitch::Bit(bitch::kRing100), 0.f, 300.f);
    CHECK(Magnitude(w, 1300.f) > 10 * Magnitude(w, 1000.f), "ring 100%%: 1300 Hz %.4f vs 1000 Hz %.4f",
          Magnitude(w, 1300.f), Magnitude(w, 1000.f));
    w = run(0u, 0.f, 300.f);
    CHECK(Magnitude(w, 1000.f) > 10 * Magnitude(w, 1300.f), "ring off: not the plain tone");
    w = run(bitch::Bit(bitch::kCarrierX2), 1.f, 300.f);
    CHECK(Magnitude(w, 1600.f) > 10 * Magnitude(w, 1300.f), "carrier x2: 1600 Hz %.4f vs 1300 Hz %.4f",
          Magnitude(w, 1600.f), Magnitude(w, 1300.f));
}

static void TestEffectKillDry() {
    for (int on = 0; on < 2; on++) {
        bitch::Engine e;
        e.Init(kSampleRate);
        bitch::Params p;
        p.tune_hz = 200.f;
        p.mix = 0.f;
        p.effects = on ? bitch::Bit(bitch::kKillDry) : 0u;
        std::vector<float> l, r;
        int n = 0;
        RunFor(e, p, 1.f, [](int k) { return Sine(1000.f, 0.25f, k); }, n, l, r);
        auto w = Window(l, 0.5f, 1.f);
        if (on) CHECK(Magnitude(w, 1200.f) > 10 * Magnitude(w, 1000.f), "kill dry: dry still there");
        else CHECK(Magnitude(w, 1000.f) > 10 * Magnitude(w, 1200.f), "mix 0: not dry");
    }
}

static void TestEffectFilterSlam() {
    for (int on = 0; on < 2; on++) {
        bitch::Engine e;
        e.Init(kSampleRate);
        bitch::Params p;
        p.filter_hz = 1000.f;
        p.filter_open = 1.f;
        p.effects = on ? bitch::Bit(bitch::kFilterSlam) : 0u;
        std::vector<float> l, r;
        int n = 0;
        RunFor(e, p, 1.f, [](int k) { return Sine(1000.f, 0.2f, k) + Sine(4000.f, 0.2f, k); }, n, l, r);
        auto w = Window(l, 0.5f, 1.f);
        double ratio = Db(Magnitude(w, 1000.f) / Magnitude(w, 4000.f));
        if (on) CHECK(ratio > 20.0, "filter slam: 1 kHz only %.1f dB above 4 kHz", ratio);
        else CHECK(fabs(ratio) < 1.0, "filter open: 1 kHz vs 4 kHz %.1f dB", ratio);
    }
}

// Switching effects on and off doesn't click (not Squelch: its opening noise burst is meant to be abrupt)
static void TestEffectsNoClick() {
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.mix = 0.5f;
    p.carrier_hz = 300.f;
    const uint32_t kToggled[4] = {bitch::Bit(bitch::kFadeOut), bitch::Bit(bitch::kKillDry),
                                  bitch::Bit(bitch::kRing100), bitch::Bit(bitch::kCarrierX2)};
    std::vector<float> l, r;
    int n = 0;
    uint32_t seed = 17;
    for (int i = 0; i < 60; i++) {
        seed = seed * 1664525u + 1013904223u;
        p.effects ^= kToggled[(seed >> 16) % 4];
        RunFor(e, p, 0.05f, [](int k) { return Sine(200.f, 0.3f, k); }, n, l, r);
    }
    float step = 0.f;
    for (size_t k = 1; k < l.size(); k++) step = std::max(step, fabsf(l[k] - l[k - 1]));
    CHECK(step < 0.06f, "effects on/off: a step of %.3f between samples", step);
}
```
Call `TestEffectsUnbuiltDoNothing();`, `TestEffectFeedbackBurst();`, `TestEffectRingAndCarrier();`, `TestEffectKillDry();`, `TestEffectFilterSlam();` and `TestEffectsNoClick();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: compile error, `no member named 'effects' in 'bitch::Params'`.

- [ ] **Step 3: Add the effects to the engine**

`engine/engine.h`:
- Add `#include "effects.h"` after the other includes.
- Add to `Params` (after `reverb`):
```cpp
    uint32_t effects = 0;        // pad effects: bit i = Effect i (effects.h); 0 = none
    float effect_amount = 0.35f; // how strong, 0.35-1 (the front end passes EffectAmount(reception))
```
- Extend the class comment: "Pad effects (Params::effects, see effects.h) each fade in and out over ~30 ms; with none on, the sound is unchanged."
- Add private members (after the plate members):
```cpp
    // Pad effects: each one's level 0-1, faded per block (~30 ms)
    float level_[kEffectCount] = {};
```

`engine/engine.cpp`:
- In `Reset()`: `for (float &v : level_) v = 0.f;`.
- In `Process()`, inside the first-block snap (`if (block_glide_ == 0.f) { ... }`), add at its end:
```cpp
        for (int k = 0; k < kEffectCount; k++) level_[k] = (params.effects >> k) & 1u ? 1.f : 0.f;
```
- Directly after that first-block block (before the `log2_filter_hz_ += ...` lines), add:
```cpp
    // Pad effects: each fades in and out over ~30 ms
    const uint32_t fx = params.effects;
    for (int k = 0; k < kEffectCount; k++) {
        float target = (fx >> k) & 1u ? 1.f : 0.f;
        level_[k] += block_glide_ * (target - level_[k]);
        if (fabsf(target - level_[k]) < 1e-4f) level_[k] = target;
    }
    // The parameter effects (ring, carrier and mix are glided per sample below, the filter per block)
    feedback += level_[kFeedbackBurst] * (0.95f - feedback);
    if (Has(fx, kRing100)) ring = 1.f;
    if (Has(fx, kCarrierX2)) carrier *= 2.f;
    if (Has(fx, kKillDry)) mix = 1.f;
    float q_target = Clamp(params.filter_q, 0.7f, 12.f), open_target = Clamp(params.filter_open, 0.f, 1.f);
    if (Has(fx, kFilterSlam)) {
        q_target = 12.f; // narrowest and most resonant
        open_target = 0.f;
    }
```
- Replace the two lines
```cpp
    filter_q_ += block_glide_ * (Clamp(params.filter_q, 0.7f, 12.f) - filter_q_);
    filter_open_ += block_glide_ * (Clamp(params.filter_open, 0.f, 1.f) - filter_open_);
```
with
```cpp
    filter_q_ += block_glide_ * (q_target - filter_q_);
    filter_open_ += block_glide_ * (open_target - filter_open_);
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`, including every earlier test (no effects = unchanged).

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/engine.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: pad effect levels; feedback burst, ring 100%, carrier x2, kill dry, filter slam"
```

---

### Task 5: The reception effects

Static burst, drift to maximum, hold drift, signal fade-out, squelch.

**Files:**
- Modify: `engine/engine.h`
- Modify: `engine/engine.cpp`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing tests**

Above `main()` in `sim/test_engine.cpp`:
```cpp
static void TestEffectStaticBurst() {
    for (int on = 0; on < 2; on++) {
        bitch::Engine e;
        e.Init(kSampleRate);
        bitch::Params p;
        p.effects = on ? bitch::Bit(bitch::kStaticBurst) : 0u;
        std::vector<float> l, r;
        int n = 0;
        RunFor(e, p, 1.f, [](int) { return 0.f; }, n, l, r);
        float rms = Rms(Window(l, 0.5f, 1.f));
        if (on) CHECK(rms > 0.01f, "static burst: only %g", rms);
        else CHECK(Peak(l) < 1e-6f, "no static burst: %g", Peak(l));
    }
}

static float Range(const std::vector<float> &x, size_t from) {
    float lo = 1e9f, hi = -1e9f;
    for (size_t i = from; i < x.size(); i++) {
        lo = std::min(lo, x[i]);
        hi = std::max(hi, x[i]);
    }
    return hi - lo;
}

static void TestEffectDriftMax() {
    bitch::Params p;
    bitch::Engine e;
    e.Init(kSampleRate);
    auto still = ShiftTrace(e, p, 5.f, [](int) { return 0.f; });
    CHECK(Range(still, 100) < 0.01f, "reception 0: the shift moved %.3f Hz", Range(still, 100));
    p.effects = bitch::Bit(bitch::kDriftMax);
    auto drift = ShiftTrace(e, p, 10.f, [](int) { return 0.f; });
    CHECK(Range(drift, 1000) > 20.f, "drift to maximum: only %.1f Hz", Range(drift, 1000));
}

static void TestEffectHoldDrift() {
    bitch::Params p;
    p.reception = 1.f;
    bitch::Engine e;
    e.Init(kSampleRate);
    auto free_run = ShiftTrace(e, p, 4.f, [](int) { return 0.f; });
    CHECK(Range(free_run, 1500) > 5.f, "reception 1: the shift only moved %.1f Hz", Range(free_run, 1500));
    p.effects = bitch::Bit(bitch::kHoldDrift);
    auto held = ShiftTrace(e, p, 3.f, [](int) { return 0.f; });
    CHECK(Range(held, 500) < 1.f, "hold drift: the shift still moved %.2f Hz", Range(held, 500));
}

static void TestEffectFadeOut() {
    auto tone = [](int k) { return Sine(1000.f, 0.25f, k); };
    bitch::Engine ref_engine;
    ref_engine.Init(kSampleRate);
    bitch::Params p;
    std::vector<float> ref_l, ref_r;
    int n = 0;
    RunFor(ref_engine, p, 1.f, tone, n, ref_l, ref_r);
    double ref = Magnitude(Window(ref_l, 0.5f, 1.f), 1000.f);
    bitch::Engine e;
    e.Init(kSampleRate);
    std::vector<float> l, r;
    n = 0;
    p.effects = bitch::Bit(bitch::kFadeOut);
    RunFor(e, p, 2.f, tone, n, l, r);
    double faded = Db(Magnitude(Window(l, 1.8f, 2.f), 1000.f) / ref);
    CHECK(faded < -35.0, "fade-out: only %.1f dB down after 1.8 s", faded);
    p.effects = 0u;
    RunFor(e, p, 2.f, tone, n, l, r);
    double back = Db(Magnitude(Window(l, 3.8f, 4.f), 1000.f) / ref);
    CHECK(fabs(back) < 1.0, "fade-out: %.1f dB, 1.8 s after it ended", back);
}

static void TestEffectSquelch() {
    auto tone = [](int k) { return Sine(1000.f, 0.25f, k); };
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    std::vector<float> l, r;
    int n = 0;
    p.effects = bitch::Bit(bitch::kSquelch);
    RunFor(e, p, 0.5f, tone, n, l, r);
    CHECK(Rms(Window(l, 0.2f, 0.5f)) < 1e-3f, "squelch: wet still %g", Rms(Window(l, 0.2f, 0.5f)));
    p.effects = 0u;
    RunFor(e, p, 1.5f, tone, n, l, r);
    double tail = Magnitude(Window(l, 0.5f, 0.6f), 3000.f), after = Magnitude(Window(l, 1.5f, 1.6f), 3000.f);
    CHECK(tail > 10 * after, "squelch: no noise burst as it opens (%.5f vs %.5f)", tail, after);
    CHECK(Rms(Window(l, 1.5f, 2.f)) > 0.15f, "squelch: the tone didn't come back (%g)", Rms(Window(l, 1.5f, 2.f)));
}
```
Call `TestEffectStaticBurst();`, `TestEffectDriftMax();`, `TestEffectHoldDrift();`, `TestEffectFadeOut();` and `TestEffectSquelch();` from `main()`.

- [ ] **Step 2: Run to see them fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: FAIL lines for static burst, drift to maximum, hold drift, fade-out and squelch (the effects don't exist yet).

- [ ] **Step 3: Add the reception effects**

`engine/engine.h`, private members (after `level_`):
```cpp
    float drift_hz_[2] = {0.f, 0.f}; // each side's drift (kept while Hold drift is on)
    float hand_hz_ = 0.f;            // the tuning hand (kept while Hold drift is on)
    float fade_db_ = 0.f;            // Signal fade-out: 0 to -40 dB
    float squelch_tail_ = 0.f;       // the noise burst as the squelch opens (1 -> 0)
    float squelch_decay_ = 0.f;      // per sample, ~150 ms
    bool squelch_was_on_ = false;
    Random squelch_noise_;
```

`engine/engine.cpp`:
- In `Init`, after `pitch_.Init(sample_rate);`: `squelch_decay_ = expf(-1.f / (0.15f * sample_rate));` and `squelch_noise_.Seed(61u);`.
- In `Reset()`: `drift_hz_[0] = drift_hz_[1] = 0.f; hand_hz_ = 0.f; fade_db_ = 0.f; squelch_tail_ = 0.f; squelch_was_on_ = false;`.
- In `Process()`, replace the Reception block's per-side loop:
```cpp
    float drift_hz[2];
    for (int c = 0; c < 2; c++) {
        drift_hz[c] = r > 0.f ? r * 40.f * drift_[c].Next(0.1f + 0.4f * r) : 0.f;
        fading_[c].Update(r);
        static_[c].Update(r);
        agc_[c].Update(r);
    }
```
with:
```cpp
    // Drift to maximum raises the drift alone; Static burst the static alone; Hold drift
    // freezes the drift and the tuning hand where they are
    float drift_amount = r + level_[kDriftMax] * (1.f - r);
    float static_level = r + level_[kStaticBurst] * (1.f - r);
    const bool hold = Has(fx, kHoldDrift);
    for (int c = 0; c < 2; c++) {
        if (!hold)
            drift_hz_[c] = drift_amount > 0.f ? drift_amount * 40.f * drift_[c].Next(0.1f + 0.4f * drift_amount) : 0.f;
        fading_[c].Update(r);
        static_[c].Update(static_level);
        agc_[c].Update(r);
    }
    // Signal fade-out: towards -40 dB over 1.5 s while on, back at the same rate
    float fade_step = 40.f / 1.5f * (float)size / sample_rate_;
    fade_db_ = Has(fx, kFadeOut) ? fmaxf(fade_db_ - fade_step, -40.f) : fminf(fade_db_ + fade_step, 0.f);
    float signal_gain = fade_db_ == 0.f ? 1.f : powf(10.f, 0.05f * fade_db_);
    // Squelch: the wet signal mutes; as it opens, a short burst of noise
    const bool squelch = Has(fx, kSquelch);
    if (squelch_was_on_ && !squelch) squelch_tail_ = 1.f;
    squelch_was_on_ = squelch;
    float wet_gain = 1.f - level_[kSquelch];
```
- In the dial code, replace
```cpp
    float hand = r > 0.f ? r * 150.f * hand_.Next(0.15f + 0.15f * r) : 0.f;
```
with
```cpp
    if (!hold) hand_hz_ = r > 0.f ? r * 150.f * hand_.Next(0.15f + 0.15f * r) : 0.f;
```
and in the `SoftQuantize` line, `dial + hand` with `dial + hand_hz_`.
- In the per-sample loop, replace `drift_hz[c]` with `drift_hz_[c]`, and change
```cpp
            float shifted = shifter_[c].Process(ringed, shift[c] + drift_hz_[c], feedback);
            float received = fading_[c].Process(shifted) + static_[c].Process();
            float wet = agc_[c].Process(filter_[c].Process(received));
```
to
```cpp
            float shifted = signal_gain * shifter_[c].Process(ringed, shift[c] + drift_hz_[c], feedback);
            float received = fading_[c].Process(shifted) + static_[c].Process();
            float wet = wet_gain * agc_[c].Process(filter_[c].Process(received));
            if (squelch_tail_ > 0.f) wet += 0.25f * squelch_tail_ * squelch_noise_.Bipolar();
```
- After the per-side loop inside the per-sample loop (after the `for (int c = 0; c < 2; c++) { ... }` that sets `y[c]`), add:
```cpp
        if (squelch_tail_ > 0.f) {
            squelch_tail_ *= squelch_decay_;
            if (squelch_tail_ < 1e-4f) squelch_tail_ = 0.f;
        }
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`, including every earlier test (the drift and hand values are the same as before when Hold drift and Drift to maximum are off).

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bit-chifter
git add engine/engine.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: static burst, drift to maximum, hold drift, signal fade-out, squelch"
```

---

### Task 6: The dial spins

**Files:**
- Modify: `engine/engine.h`
- Modify: `engine/engine.cpp`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing test**

Above `main()` in `sim/test_engine.cpp`:
```cpp
static void TestEffectSpins() {
    auto silence = [](int) { return 0.f; };
    bitch::Params p;
    p.effect_amount = 1.f;
    p.effects = bitch::Bit(bitch::kSpinUp);
    bitch::Engine e;
    e.Init(kSampleRate);
    auto up = ShiftTrace(e, p, 1.2f, silence);
    CHECK(fabsf(up[500] - 1500.f) < 150.f, "spin up: %.0f Hz after 0.5 s (want ~1500)", up[500]);
    CHECK(fabsf(up.back() - 2000.f) < 20.f, "spin up: %.0f Hz after 1.2 s (want the 2000 Hz limit)", up.back());
    p.effects = 0u;
    auto back = ShiftTrace(e, p, 5.f, silence);
    CHECK(fabsf(back.back()) < 50.f, "spin up: %.0f Hz, 5 s after release", back.back());
    bitch::Engine d;
    d.Init(kSampleRate);
    p.effects = bitch::Bit(bitch::kSpinDown);
    auto down = ShiftTrace(d, p, 0.6f, silence);
    CHECK(fabsf(down[500] + 1500.f) < 150.f, "spin down: %.0f Hz after 0.5 s (want ~-1500)", down[500]);
    bitch::Engine slow;
    slow.Init(kSampleRate);
    p.effect_amount = 0.35f;
    p.effects = bitch::Bit(bitch::kSpinUp);
    auto gentle = ShiftTrace(slow, p, 1.f, silence);
    CHECK(fabsf(gentle.back() - 150.f) < 20.f, "spin up at amount 0.35: %.0f Hz after 1 s (want ~150)",
          gentle.back());
}
```
Call `TestEffectSpins();` from `main()`.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: FAIL lines for `spin up` and `spin down`.

- [ ] **Step 3: Add the spins**

`engine/engine.h`, private members (after the squelch members):
```cpp
    float spin_hz_ = 0.f;     // Dial spin: the dial's extra turn
    float spin_return_ = 0.f; // per block, ~1 s (the dial coming back after a spin)
```

`engine/engine.cpp`:
- In `Reset()`: `spin_hz_ = 0.f;`.
- In the first-block snap, add `spin_return_ = 1.f - expf(-(float)size / (1.f * sample_rate_));`.
- In the dial code, replace the line
```cpp
    target_shift_ = SoftQuantize(dial + hand_hz_, detent_pitch_, detent_amount_);
```
with:
```cpp
    // Dial spin: the dial keeps turning, 150 Hz/s (amount 0.35) to 3000 Hz/s (amount 1),
    // within +-2 kHz in all; after the pad is released it comes back in ~1 s. Both on: back.
    float amount = Clamp(params.effect_amount, 0.35f, 1.f);
    float spin_rate = 150.f + 2850.f * (amount - 0.35f) / 0.65f;
    float spin_dir = (Has(fx, kSpinUp) ? 1.f : 0.f) - (Has(fx, kSpinDown) ? 1.f : 0.f);
    if (spin_dir != 0.f) {
        spin_hz_ += spin_dir * spin_rate * (float)size / sample_rate_;
    } else {
        spin_hz_ -= spin_return_ * spin_hz_;
        if (fabsf(spin_hz_) < 0.01f) spin_hz_ = 0.f;
    }
    float turned = dial + hand_hz_;
    // (turned can reach +-2150 with the tuning hand: clamp it for the limits, so they never cross)
    float limit_turned = Clamp(turned, -2000.f, 2000.f);
    spin_hz_ = Clamp(spin_hz_, -2000.f - limit_turned, 2000.f - limit_turned);
    target_shift_ = SoftQuantize(turned + spin_hz_, detent_pitch_, detent_amount_);
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bit-chifter/sim/run_tests.sh`
Expected: `All tests passed`, including every earlier test (with no spin, `spin_hz_` stays exactly 0).

- [ ] **Step 5: Check the firmware still builds, and commit**

Run: `make -C ~/local_files/daisy/bit-chifter` — expected: no errors or warnings.
```bash
cd ~/local_files/daisy/bit-chifter
git add engine/engine.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: dial spin up and down"
```

(Review checkpoint: the engine tasks are done; review before Task 7.)

---

### Task 7: Render options and listening

**Files:**
- Modify: `sim/render.cpp`
- Modify: `sim/render.sh`

- [ ] **Step 1: Add the options**

In `sim/render.cpp`:
- Add `#include "effects.h"` and `#include <cstdlib>` (for `atof`).
- Add `{"amount", &p.effect_amount}` to `fields[]`.
- Add two locals in `main()` next to `sweep_to`: `float every = 0.f;` (seconds; 0 = effects on all the time) and handle these arguments in the argument loop **before** calling `SetOption`:
```cpp
        if (strncmp(argv[i], "effects=", 8) == 0) {
            // A comma-separated list of effect names (effects.h), e.g. effects=squelch,spin_up
            char list[256];
            snprintf(list, sizeof(list), "%s", argv[i] + 8);
            for (char *name = strtok(list, ","); name; name = strtok(nullptr, ",")) {
                int found = -1;
                for (int e = 0; e < bitch::kEffectCount; e++)
                    if (strcmp(name, bitch::EffectName(e)) == 0) found = e;
                if (found < 0) {
                    fprintf(stderr, "unknown effect: %s\n", name);
                    return 1;
                }
                p.effects |= 1u << found;
            }
            continue;
        }
        if (strncmp(argv[i], "every=", 6) == 0) {
            every = (float)atof(argv[i] + 6); // effects on for `every` seconds, off for `every`, ...
            continue;
        }
```
- In the block loop, before `engine.Process(...)`:
```cpp
        if (every > 0.f) {
            float t = (float)i / (float)in.sample_rate;
            p.effects = fmodf(t, 2.f * every) < every ? effects_on : 0u;
        }
```
  with `const uint32_t effects_on = p.effects;` declared just before the loop (and `p.effects = effects_on;` restored after it).
- Add ` [amount=0.35-1] [effects=name,name] [every=s]` to the usage comment and message, and to the final `printf` add ` amount %g effects %s` with `p.effect_amount` and a string built from the names of the set bits (comma-separated, `none` when empty).
In `sim/render.sh`, add `[amount=] [effects=name,name] [every=s]` to the usage comment.

- [ ] **Step 2: Render**

```bash
cd ~/local_files/daisy/bit-chifter/sim/out
../render.sh voice.wav p4_spin.wav effects=spin_up every=1.5 amount=0.6
../render.sh drone.wav p4_burst_slam.wav tune=40 filter=880 effects=feedback_burst,filter_slam every=2
../render.sh voice.wav p4_squelch.wav effects=squelch every=1
../render.sh voice.wav p4_static_drift.wav tune=80 effects=static_burst,drift_max every=2.5
../render.sh drone.wav p4_ring_x2.wav carrier=330 effects=ring_100,carrier_x2 every=2
../render.sh voice.wav p4_hold_drift.wav reception=0.8 effects=hold_drift every=2
../render.sh voice.wav p4_fade_out.wav effects=fade_out every=3
```
Expected: seven `Wrote …` lines listing the effects, no compiler warnings.

- [ ] **Step 3: User listens, then commit**

Send the seven renders to the user, saying what each toggles (every N seconds on, then off). After they're happy:
```bash
cd ~/local_files/daisy/bit-chifter
git add sim/render.cpp sim/render.sh
git commit -m "Render tool: effects, amount and every options"
```

---

### Task 8: Firmware

**Files:**
- Modify: `simple_touch.cpp`
- Modify: `README.md`

- [ ] **Step 1: Pages, pads and effects**

In `simple_touch.cpp`:
- Add includes: `#include "effects.h"` and `#include "ui/pad_pages.h"`.
- Add globals after `blips`:
```cpp
Switch3 page_switch;                 // switch 1 (left): the page; reads mirror-image (README)
bitch::PadPages pad_pages;
volatile uint32_t effects = 0;       // from the pads (main loop) to the engine (audio callback)
volatile int page = 0;               // for the report
```
- In `AudioCallback`, after `p.reception = ...`:
```cpp
    p.effects = effects;
    p.effect_amount = bitch::EffectAmount(p.reception);
```
- In `main()`, after the pads are set up: `page_switch.Init(seed::D9, seed::D8);`.
- Replace the TOU block in the main loop (from `// TOU pad: each new touch steps the stereo mode` through `was_touched = touched;`, and remove `bool was_touched = false;`) with:
```cpp
        // Pads and switch 1: the strange pages (tap latches, hold is momentary); TOU tap steps
        // the stereo mode (1, 2 or 3 blips), TOU hold 1 s clears every latched effect (a low blip).
        // Switch 1 physically left reads POS_RIGHT: left = page 1, centre = page 2, right = page 3.
        int position = page_switch.Read();
        page = position == Switch3::POS_RIGHT ? 0 : (position == Switch3::POS_CENTER ? 1 : 2);
        pad_pages.Update(System::GetNow(), pads_ok ? pads.Touched() : 0u, page);
        effects = pad_pages.Effects();
        int steps = pad_pages.TakeStereoSteps();
        if (steps > 0) {
            stereo_mode = (stereo_mode + steps) % 3;
            blips.Start(stereo_mode + 1); // Blips::Start is safe from the main loop (atomic request)
        }
        if (pad_pages.TakeCleared()) blips.Start(1, true);
```
- Remove the now-unused `kPadStereoMode` constant (PadPages owns the TOU channel).
- In the report's second line, after `| pitch %s`, add ` | page %d fx %08lx` with the arguments `page + 1, (unsigned long)effects`.
- Update the file's top comment: switch 1 picks the page (Shift & time / Radio / Digital & damage); the pads: tap latches, hold is momentary; TOU tap steps the stereo mode, hold 1 s clears all; RECEPTION also sets the effects' amount. Built so far: feedback burst, dial spins, static burst, squelch, signal fade-out, filter slam, drift to maximum, ring 100%, carrier ×2, kill dry, hold drift.

- [ ] **Step 2: Build**

Run: `make -C ~/local_files/daisy/bit-chifter`
Expected: memory table, no errors or warnings.

- [ ] **Step 3: User flashes and plays it**

User runs `make program-dfu` in `~/local_files/daisy/bit-chifter` (RESET, then BOOT while the LED pulses). Try, with the readout (`cat /dev/cu.usbmodem*`) showing `page … fx …`:
- Switch 1 left (page 1): tap the bottom-right pad (dial spin up): the dial keeps turning up; tap again: it comes back. Hold the bottom-left pad (spin down): it spins down while held, comes back on release. Top centre: feedback burst.
- Switch 1 centre (page 2): top left static burst, top right squelch (it opens with a burst of noise), bottom left signal fade-out, bottom right filter slam, top centre drift to maximum.
- Switch 1 right (page 3): top left ring 100%, top right carrier ×2, bottom left kill dry, top centre hold drift (try with RECEPTION up).
- RECEPTION changes how fast the spins turn. Latch a few on different pages, then hold TOU for 1 s: everything latched stops, with a low blip. A quick TOU tap still steps the stereo mode (now on release).
- The other pads do nothing yet.

- [ ] **Step 4: Document and commit (after the user confirms)**

In `README.md`'s "Controls so far" table, replace the `"TOU" logo pad` row with:
```markdown
| Switch 1 (left switch) | Page: left = Shift & time, centre = Radio, right = Digital & damage |
| Pads | Effects on the page (see the strange pages spec): tap latches on or off, hold is momentary. Built so far: page 1 bottom left/right dial spin down/up, top centre feedback burst; page 2 top left static burst, top right squelch, bottom left signal fade-out, bottom right filter slam, top centre drift to maximum; page 3 top left ring 100%, top right carrier ×2, bottom left kill dry, top centre hold drift |
| "TOU" logo pad | Tap: stereo mode (1 blip Linked, 2 Mirror, 3 Rotate (8 s)); hold 1 s: clear every latched effect (a low blip) |
```
In the RECEPTION row add: "; also how strong the pad effects are". Add `[amount=] [effects=name,name] [every=s]` to the render line.
```bash
cd ~/local_files/daisy/bit-chifter
git add simple_touch.cpp README.md
git commit -m "Simple Touch firmware: the strange pages (switch 1, pads, TOU clear), first 12 effects"
```
