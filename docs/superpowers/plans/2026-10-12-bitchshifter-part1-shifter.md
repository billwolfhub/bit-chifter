# Bitchshifter Part 1 (Playable Shifter) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A playable frequency shifter on the Synthux Simple Touch (TUNE fader, FEEDBACK and MIX knobs), with the sound engine tested on the Mac first. This is build steps 1–3 of the spec; ring mod/filter, reception and pads are later plans.

**Architecture:** The sound engine (`engine/`) is plain C++ with no hardware code, so it compiles both for the Daisy and on the Mac. A Mac test program checks the shifter numerically (Goertzel measurements) and a render tool writes WAVs to listen to. The Simple Touch front end (`simple_touch.cpp`) reads the controls and calls the engine from the audio callback. A separate hardware-test firmware first identifies which analog inputs are the faders.

**Tech Stack:** C++17, libDaisy v8.1 (shared at `~/local_files/daisy/libDaisy`), arm-none-eabi-gcc 10.3 (`/Library/DaisyToolchain`), Apple clang for the Mac harness, Daisy bootloader v6.4 (`APP_TYPE = BOOT_SRAM`).

**Spec:** `docs/superpowers/specs/2026-10-12-bitchshifter-design.md`

**Conventions (from this user's Daisy projects):**
- Project root: `~/local_files/daisy/bitchshifter` (local git repo, already created with the spec).
- The user builds and flashes in their own Terminal: tap RESET, press BOOT while the LED pulses, then `make program-dfu`. Commit only after the user confirms hardware steps work. **No `Co-Authored-By` lines in commits.**
- newlib-nano has no float `printf`: print numbers as integers.
- Mac builds need `-isysroot` pointing at `/Library/Developer/CommandLineTools/SDKs/MacOSX26*.sdk`.

---

## File structure

| File | Responsibility |
|---|---|
| `hwtest/HwTest.cpp`, `hwtest/Makefile` | Hardware test firmware: prints analog inputs, switches and pads over USB serial |
| `engine/softclip.h` | `SoftClip()`: rational tanh approximation, used in feedback and the output limiter |
| `engine/hilbert.h` | `Hilbert`: 90° phase splitter (two allpass chains) |
| `engine/shifter.h` | `Shifter`: single-sideband shift with quadrature oscillator and feedback |
| `engine/engine.h`, `engine/engine.cpp` | `Params`, `Engine`: mono sum, shift glide, two shifters (L/R), dry/wet, limiter, NaN guard |
| `sim/test_engine.cpp` | Mac tests: Hilbert phase/level, shift accuracy and image rejection, through-zero, stability |
| `sim/wav.h` | Minimal 16-bit PCM WAV read/write for the render tool |
| `sim/render.cpp` | Renders a WAV through the engine with given settings |
| `sim/run_tests.sh`, `sim/render.sh` | Build-and-run scripts for the Mac |
| `simple_touch.cpp`, `Makefile` | Simple Touch firmware |
| `README.md` | Controls, building, flashing |

---

### Task 1: Hardware test firmware

Identifies which of A0–A7 are knobs vs faders (and their direction), and confirms the switches and pads work. No engine code yet.

**Files:**
- Create: `hwtest/HwTest.cpp`
- Create: `hwtest/Makefile`

- [ ] **Step 1: Write the firmware**

`hwtest/HwTest.cpp`:
```cpp
// Bitchshifter hardware test for the Synthux Simple Touch: prints the 8 analog
// inputs (A0-A7, as 0-100), both 3-way switches and the 12 touch pads over USB
// serial, 5 times a second. Move one control at a time to see which is which.
#include "daisy_seed.h"

using namespace daisy;

DaisySeed hw;
Switch3 switch_a, switch_b;
Mpr121I2C pads;

int main(void) {
    hw.Init();
    hw.StartLog(false);

    AdcChannelConfig cfg[8];
    Pin pins[8] = {seed::A0, seed::A1, seed::A2, seed::A3, seed::A4, seed::A5, seed::A6, seed::A7};
    for (int i = 0; i < 8; i++) cfg[i].InitSingle(pins[i]);
    hw.adc.Init(cfg, 8);
    hw.adc.Start();

    // Pin pairs as wired on the Simple Touch (from Synthux's TouchFX source)
    switch_a.Init(seed::D7, seed::D6);
    switch_b.Init(seed::D9, seed::D8);

    Mpr121I2C::Config pad_config;
    bool pads_ok = pads.Init(pad_config) == Mpr121I2C::OK;

    const char *positions[3] = {"C", "L", "R"}; // Switch3: 0 centre, 1 left/up, 2 right/down
    while (1) {
        int a[8];
        for (int i = 0; i < 8; i++) a[i] = (int)(hw.adc.GetFloat(i) * 100.f + 0.5f);
        uint16_t touched = pads_ok ? pads.Touched() : 0;
        char pad_bits[13];
        for (int i = 0; i < 12; i++) pad_bits[i] = (touched & (1 << i)) ? "123456789ABC"[i] : '.';
        pad_bits[12] = 0;
        hw.PrintLine("A0-A7: %3d %3d %3d %3d %3d %3d %3d %3d | sw A:%s B:%s | pads:%s %s",
                     a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7],
                     positions[switch_a.Read()], positions[switch_b.Read()], pad_bits,
                     pads_ok ? "" : "(MPR121 not found)");
        hw.SetLed(touched != 0);
        System::Delay(200);
    }
}
```

`hwtest/Makefile`:
```make
TARGET = HwTest
CPP_SOURCES = HwTest.cpp
LIBDAISY_DIR = ../../libDaisy
APP_TYPE = BOOT_SRAM
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
```

- [ ] **Step 2: Build**

Run: `cd ~/local_files/daisy/bitchshifter/hwtest && make`
Expected: ends with a memory table (SRAM well under 480 KB) and no errors.

- [ ] **Step 3: User flashes it and reports the readings**

The user flashes with `make program-dfu` (RESET, then BOOT while the LED pulses). Read the serial output from the Mac:
```bash
cat /dev/cu.usbmodem*
```
(Ctrl-C to stop.) Ask the user to move each knob and fader in turn, flip each switch, and touch pads 1–12. Record:
- which A-index each knob 1–6 and fader 1–2 is (spec numbering: the user decides which physical knob is "knob 1" etc.; suggest left-to-right, top-to-bottom),
- for each, whether the value rises when turned clockwise / pushed up (if not, it needs flipping),
- which switch is "switch 1" (pad mode) and "switch 2" (modifier), and which position reads L/C/R,
- that all 12 pads register, and which bit is which physical pad.

- [ ] **Step 4: Write the control map into the README**

Create `README.md`:
```markdown
# Bitchshifter

Frequency shifter / ring mod voiced as a shortwave sideband receiver, for the Synthux
Simple Touch. Design: `docs/superpowers/specs/2026-10-12-bitchshifter-design.md`.

## Control map (measured with hwtest/)

| Control | Input | Increases when |
|---|---|---|
| Fader 1 TUNE | A? | pushed up |
| Fader 2 RECEPTION | A? | pushed up |
| Knob 1 RING | A? | turned clockwise |
| Knob 2 CARRIER | A? | turned clockwise |
| Knob 3 FILTER | A? | turned clockwise |
| Knob 4 WIDTH | A? | turned clockwise |
| Knob 5 FEEDBACK | A? | turned clockwise |
| Knob 6 MIX | A? | turned clockwise |

Switch 1 (pad mode): D?/D?. Switch 2 (modifier): D?/D?. Pads: MPR121 bits 0-11 = pads 1-12 (order as measured).
```
Replace every `?` with the measured values (and note any control that reads reversed, e.g. "pushed down").

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add hwtest/HwTest.cpp hwtest/Makefile README.md
git commit -m "Hardware test firmware and measured Simple Touch control map"
```

---

### Task 2: Soft clip and Hilbert phase splitter

**Files:**
- Create: `engine/softclip.h`
- Create: `engine/hilbert.h`
- Create: `sim/test_engine.cpp`
- Create: `sim/run_tests.sh`

- [ ] **Step 1: Write the failing Hilbert test**

`sim/test_engine.cpp`:
```cpp
// Mac tests for the Bitchshifter engine. Build and run with sim/run_tests.sh.
#include <cmath>
#include <cstdio>
#include <vector>

#include "hilbert.h"

static int failures = 0;
#define CHECK(cond, ...)                                                   \
    do {                                                                   \
        if (!(cond)) {                                                     \
            failures++;                                                    \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);                    \
            printf(__VA_ARGS__);                                           \
            printf("\n");                                                  \
        }                                                                  \
    } while (0)

static const float kSampleRate = 48000.f;
static const float kPi = 3.14159265358979f;

// I and Q of a sine must have equal level and be uncorrelated (90 degrees apart)
static void TestHilbertQuadrature() {
    for (float freq : {100.f, 1000.f, 5000.f, 15000.f}) {
        bitch::Hilbert h;
        h.Reset();
        double ii = 0, qq = 0, iq = 0;
        int n = (int)kSampleRate;
        for (int t = 0; t < n; t++) {
            float x = sinf(2 * kPi * freq * t / kSampleRate);
            float i, q;
            h.Process(x, i, q);
            if (t < n / 2) continue; // let the filters settle
            ii += i * i;
            qq += q * q;
            iq += i * q;
        }
        double level_ratio = sqrt(qq / ii);
        double correlation = iq / sqrt(ii * qq);
        CHECK(fabs(level_ratio - 1.0) < 0.05, "Hilbert %g Hz: Q/I level %.3f", freq, level_ratio);
        CHECK(fabs(correlation) < 0.05, "Hilbert %g Hz: I/Q correlation %.3f", freq, correlation);
    }
}

int main() {
    TestHilbertQuadrature();
    if (failures == 0) printf("All tests passed\n");
    return failures == 0 ? 0 : 1;
}
```

`sim/run_tests.sh`:
```sh
#!/bin/sh
# Builds the engine for the Mac and runs the tests.
set -e
cd "$(dirname "$0")"
mkdir -p out
# The newest macOS SDK may not match the installed linker; use the 26.x SDK that works
SDK=$(ls -d /Library/Developer/CommandLineTools/SDKs/MacOSX26*.sdk 2>/dev/null | head -1)
SOURCES="test_engine.cpp"
[ -f ../engine/engine.cpp ] && SOURCES="$SOURCES ../engine/engine.cpp"
c++ ${SDK:+-isysroot "$SDK"} -std=c++17 -O2 -Wall -Wextra -I../engine $SOURCES -o out/test_engine
./out/test_engine
```

Run: `chmod +x ~/local_files/daisy/bitchshifter/sim/run_tests.sh`

- [ ] **Step 2: Run it to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile error, `'hilbert.h' file not found`.

- [ ] **Step 3: Write the soft clip and the Hilbert splitter**

`engine/softclip.h`:
```cpp
#pragma once

namespace bitch {

// tanh-like soft clip: slope 1 near 0, smoothly limits to +-1 (rational
// approximation, exact limit reached at +-3)
inline float SoftClip(float x) {
    if (x <= -3.f) return -1.f;
    if (x >= 3.f) return 1.f;
    float x2 = x * x;
    return x * (27.f + x2) / (27.f + 9.f * x2);
}

} // namespace bitch
```

`engine/hilbert.h`:
```cpp
#pragma once

namespace bitch {

// 90-degree phase splitter: two parallel chains of four second-order allpass
// sections (Olli Niemitalo's coefficients). The two outputs (the A chain one
// sample later (I), and the B chain (Q)) stay 90 degrees apart, within a fraction of a degree, from about 20 Hz to
// over 20 kHz at 48 kHz.
class Hilbert {
  public:
    void Reset() {
        for (State &s : a_) s = State();
        for (State &s : b_) s = State();
        a_delay_ = 0.f;
    }

    void Process(float x, float &i, float &q) {
        float a = x, b = x;
        for (int k = 0; k < 4; k++) a = Section(a, kA[k], a_[k]);
        for (int k = 0; k < 4; k++) b = Section(b, kB[k], b_[k]);
        i = a_delay_;
        a_delay_ = a;
        q = b;
    }

  private:
    struct State {
        float x1 = 0.f, x2 = 0.f, y1 = 0.f, y2 = 0.f;
    };

    // y[n] = c * (x[n] + y[n-2]) - x[n-2], c = coefficient squared
    static float Section(float x, float c, State &s) {
        float y = c * (x + s.y2) - s.x2;
        s.x2 = s.x1;
        s.x1 = x;
        s.y2 = s.y1;
        s.y1 = y;
        return y;
    }

    static constexpr float kA[4] = {0.6923878f * 0.6923878f, 0.9360654322959f * 0.9360654322959f,
                                    0.9882295226860f * 0.9882295226860f, 0.9987488452737f * 0.9987488452737f};
    static constexpr float kB[4] = {0.4021921162426f * 0.4021921162426f, 0.8561710882420f * 0.8561710882420f,
                                    0.9722909545651f * 0.9722909545651f, 0.9952884791278f * 0.9952884791278f};
    State a_[4], b_[4];
    float a_delay_ = 0.f;
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests to see them pass**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed`. If the 100 Hz or 15 kHz case fails narrowly, report the printed numbers before changing tolerances (a real phase error would show at both ends).

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/softclip.h engine/hilbert.h sim/test_engine.cpp sim/run_tests.sh
git commit -m "Engine: soft clip and Hilbert phase splitter, with Mac tests"
```

---

### Task 3: Shifter (single sideband, through zero, feedback)

**Files:**
- Create: `engine/shifter.h`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing shifter tests**

In `sim/test_engine.cpp`, add `#include "shifter.h"` after `#include "hilbert.h"`, add these functions above `main()`:
```cpp
// Magnitude of one frequency in a block of samples (Goertzel)
static double Magnitude(const std::vector<float> &x, float freq) {
    double w = 2 * M_PI * freq / kSampleRate;
    double coeff = 2 * cos(w), s1 = 0, s2 = 0;
    for (float v : x) {
        double s = v + coeff * s1 - s2;
        s2 = s1;
        s1 = s;
    }
    double power = s1 * s1 + s2 * s2 - coeff * s1 * s2;
    return 2 * sqrt(power) / x.size(); // amplitude of a sine at freq
}

// Shifts a 1 kHz sine (amplitude 0.25) and returns the last 0.5 s of output
static std::vector<float> ShiftSine(float shift_hz, float feedback) {
    bitch::Shifter s;
    s.Init(kSampleRate);
    std::vector<float> out;
    int n = (int)kSampleRate;
    for (int t = 0; t < n; t++) {
        float x = 0.25f * sinf(2 * kPi * 1000.f * t / kSampleRate);
        float y = s.Process(x, shift_hz, feedback);
        if (t >= n / 2) out.push_back(y);
    }
    return out;
}

static double Db(double ratio) { return 20 * log10(ratio); }

static void TestShiftUpAndDown() {
    auto up = ShiftSine(100.f, 0.f);
    CHECK(fabs(Magnitude(up, 1100.f) - 0.25) < 0.02, "+100 Hz: 1100 Hz level %.3f", Magnitude(up, 1100.f));
    CHECK(Db(Magnitude(up, 1100.f) / Magnitude(up, 900.f)) > 40, "+100 Hz: image only %.1f dB down",
          Db(Magnitude(up, 1100.f) / Magnitude(up, 900.f)));
    auto down = ShiftSine(-100.f, 0.f);
    CHECK(fabs(Magnitude(down, 900.f) - 0.25) < 0.02, "-100 Hz: 900 Hz level %.3f", Magnitude(down, 900.f));
    CHECK(Db(Magnitude(down, 900.f) / Magnitude(down, 1100.f)) > 40, "-100 Hz: image only %.1f dB down",
          Db(Magnitude(down, 900.f) / Magnitude(down, 1100.f)));
    auto none = ShiftSine(0.f, 0.f);
    CHECK(fabs(Magnitude(none, 1000.f) - 0.25) < 0.02, "0 Hz: 1000 Hz level %.3f", Magnitude(none, 1000.f));
}

// Shifting 1 kHz down by 1.5 kHz goes through zero and comes out at 500 Hz
static void TestThroughZero() {
    auto out = ShiftSine(-1500.f, 0.f);
    CHECK(fabs(Magnitude(out, 500.f) - 0.25) < 0.02, "-1500 Hz: 500 Hz level %.3f", Magnitude(out, 500.f));
}

// Maximum feedback on noise for 60 s stays finite and bounded
static void TestFeedbackStable() {
    bitch::Shifter s;
    s.Init(kSampleRate);
    uint32_t seed = 1;
    float peak = 0.f;
    bool finite = true;
    for (int t = 0; t < 60 * (int)kSampleRate; t++) {
        seed = seed * 1664525u + 1013904223u;
        float noise = ((seed >> 8) / 8388608.f - 1.f) * 0.5f;
        float y = s.Process(noise, 3.f, 0.95f);
        finite = finite && std::isfinite(y);
        peak = std::max(peak, fabsf(y));
    }
    CHECK(finite, "feedback 0.95: non-finite output");
    CHECK(peak < 4.f, "feedback 0.95: peak %.2f", peak);
}
```
and call them from `main()` after `TestHilbertQuadrature();`:
```cpp
    TestShiftUpAndDown();
    TestThroughZero();
    TestFeedbackStable();
```
Also add `#include <algorithm>` and `#include <cstdint>` to the includes.

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile error, `'shifter.h' file not found`.

- [ ] **Step 3: Write the shifter**

`engine/shifter.h`:
```cpp
#pragma once

#include <cmath>

#include "hilbert.h"
#include "softclip.h"

namespace bitch {

// Single-sideband frequency shifter: the Hilbert splitter's I and Q are mixed
// with a quadrature oscillator at the shift frequency. Positive shifts move the
// spectrum up, negative down; the oscillator runs backwards for negative shifts,
// so the shift sweeps smoothly through 0. Feedback returns the output to the input
// after 2 ms, through a soft clip (barberpole sweeps without running away).
class Shifter {
  public:
    static constexpr int kFeedbackDelay = 96; // 2 ms at 48 kHz

    void Init(float sample_rate) {
        sample_rate_ = sample_rate;
        Reset();
    }

    void Reset() {
        hilbert_.Reset();
        phase_ = 0.f;
        for (float &v : feedback_buffer_) v = 0.f;
        position_ = 0;
    }

    // feedback: 0 to 0.95
    float Process(float x, float shift_hz, float feedback) {
        float delayed = feedback_buffer_[position_];
        float in = x + feedback * SoftClip(delayed);
        float i, q;
        hilbert_.Process(in, i, q);
        phase_ += shift_hz / sample_rate_;
        phase_ -= floorf(phase_);
        float angle = 6.28318530718f * phase_;
        float out = i * cosf(angle) - q * sinf(angle);
        feedback_buffer_[position_] = out;
        position_ = (position_ + 1) % kFeedbackDelay;
        return out;
    }

  private:
    Hilbert hilbert_;
    float sample_rate_ = 48000.f;
    float phase_ = 0.f;
    float feedback_buffer_[kFeedbackDelay] = {};
    int position_ = 0;
};

} // namespace bitch
```

- [ ] **Step 4: Run the tests**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: `All tests passed`.
If the +100 Hz test reports the energy at 900 Hz and the −100 Hz test at 1100 Hz (up and down swapped), the I/Q sign is reversed for this splitter: change `i * cosf(angle) - q * sinf(angle)` to `i * cosf(angle) + q * sinf(angle)` and run again. If the feedback test's peak is ≥ 4, report the value rather than raising the limit.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/shifter.h sim/test_engine.cpp
git commit -m "Engine: single-sideband shifter with through-zero and feedback"
```

---

### Task 4: Engine (mono sum, glide, stereo, mix, limiter, NaN guard)

**Files:**
- Create: `engine/engine.h`
- Create: `engine/engine.cpp`
- Modify: `sim/test_engine.cpp`

- [ ] **Step 1: Add the failing engine tests**

In `sim/test_engine.cpp`, add `#include "engine.h"`, and above `main()`:
```cpp
// Runs the engine on a 1 kHz sine in both inputs; returns the last 0.5 s of OUT L
static std::vector<float> RunEngine(const bitch::Params &p, float amplitude, std::vector<float> *right = nullptr) {
    bitch::Engine e;
    e.Init(kSampleRate);
    const size_t block = 48;
    std::vector<float> in(block), out_l(block), out_r(block), result;
    int blocks = (int)kSampleRate / block;
    for (int b = 0; b < blocks; b++) {
        for (size_t k = 0; k < block; k++)
            in[k] = amplitude * sinf(2 * kPi * 1000.f * (b * block + k) / kSampleRate);
        e.Process(in.data(), in.data(), out_l.data(), out_r.data(), block, p);
        if (b >= blocks / 2) {
            result.insert(result.end(), out_l.begin(), out_l.end());
            if (right) right->insert(right->end(), out_r.begin(), out_r.end());
        }
    }
    return result;
}

static void TestEngineShiftAndMix() {
    bitch::Params p;
    p.tune_hz = 100.f;
    p.mix = 1.f;
    std::vector<float> right;
    auto wet = RunEngine(p, 0.25f, &right);
    CHECK(Magnitude(wet, 1100.f) > 0.2, "engine wet: 1100 Hz level %.3f", Magnitude(wet, 1100.f));
    CHECK(Magnitude(wet, 1000.f) < 0.01, "engine wet: dry leaking %.3f", Magnitude(wet, 1000.f));
    CHECK(fabs(Magnitude(right, 1100.f) - Magnitude(wet, 1100.f)) < 0.01, "engine: L/R differ");
    p.mix = 0.f;
    auto dry = RunEngine(p, 0.25f);
    CHECK(Magnitude(dry, 1000.f) > 0.2, "engine dry: 1000 Hz level %.3f", Magnitude(dry, 1000.f));
    CHECK(Magnitude(dry, 1100.f) < 0.01, "engine dry: wet leaking %.3f", Magnitude(dry, 1100.f));
}

static void TestEngineLimiter() {
    bitch::Params p;
    p.tune_hz = 50.f;
    p.feedback = 0.95f;
    p.mix = 1.f;
    auto loud = RunEngine(p, 4.f); // far too hot input
    float peak = 0.f;
    for (float v : loud) peak = std::max(peak, fabsf(v));
    CHECK(peak <= 1.f, "engine limiter: peak %.2f", peak);
}

static void TestEngineRecoversFromNaN() {
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.tune_hz = 100.f;
    float in[48], out_l[48], out_r[48];
    for (float &v : in) v = NAN;
    e.Process(in, in, out_l, out_r, 48, p);
    for (float v : out_l) CHECK(std::isfinite(v), "engine NaN: output not finite");
    for (int k = 0; k < 48; k++) in[k] = 0.25f * sinf(2 * kPi * 1000.f * k / kSampleRate);
    e.Process(in, in, out_l, out_r, 48, p);
    for (float v : out_l) CHECK(std::isfinite(v), "engine NaN: did not recover");
}
```
and call them from `main()`:
```cpp
    TestEngineShiftAndMix();
    TestEngineLimiter();
    TestEngineRecoversFromNaN();
```

- [ ] **Step 2: Run to see it fail**

Run: `~/local_files/daisy/bitchshifter/sim/run_tests.sh`
Expected: compile error, `'engine.h' file not found`.

- [ ] **Step 3: Write the engine**

`engine/engine.h`:
```cpp
#pragma once

#include <cstddef>

#include "shifter.h"

namespace bitch {

// Everything the front end controls. Part 1 uses the shifter controls only.
struct Params {
    float tune_hz = 0.f;  // shift, -2500 to +2500 Hz
    float feedback = 0.f; // 0 to 0.95
    float mix = 1.f;      // dry/wet, 0 to 1
};

// The sound engine: no hardware code, so it runs on the Daisy and on the Mac.
// The inputs are summed to mono; L and R each have their own shifter (they will
// drift apart once Reception is added, which is where the stereo comes from).
class Engine {
  public:
    void Init(float sample_rate);
    void Process(const float *in_l, const float *in_r, float *out_l, float *out_r, size_t size,
                 const Params &params);

  private:
    void Reset();

    Shifter shifter_[2];
    float shift_hz_ = 0.f; // glided towards params.tune_hz
    float glide_ = 0.f;    // one-pole coefficient, ~20 ms
};

} // namespace bitch
```

`engine/engine.cpp`:
```cpp
#include "engine.h"

#include <cmath>

#include "softclip.h"

namespace bitch {

void Engine::Init(float sample_rate) {
    for (Shifter &s : shifter_) s.Init(sample_rate);
    glide_ = 1.f - expf(-1.f / (0.02f * sample_rate));
    shift_hz_ = 0.f;
}

void Engine::Reset() {
    for (Shifter &s : shifter_) s.Reset();
    shift_hz_ = 0.f;
}

void Engine::Process(const float *in_l, const float *in_r, float *out_l, float *out_r, size_t size,
                     const Params &params) {
    float feedback = fminf(fmaxf(params.feedback, 0.f), 0.95f);
    float mix = fminf(fmaxf(params.mix, 0.f), 1.f);
    for (size_t i = 0; i < size; i++) {
        shift_hz_ += glide_ * (params.tune_hz - shift_hz_);
        float dry = 0.5f * (in_l[i] + in_r[i]);
        float wet_l = shifter_[0].Process(dry, shift_hz_, feedback);
        float wet_r = shifter_[1].Process(dry, shift_hz_, feedback);
        out_l[i] = SoftClip(dry + mix * (wet_l - dry));
        out_r[i] = SoftClip(dry + mix * (wet_r - dry));
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
Expected: `All tests passed`.

- [ ] **Step 5: Commit**

```bash
cd ~/local_files/daisy/bitchshifter
git add engine/engine.h engine/engine.cpp sim/test_engine.cpp
git commit -m "Engine: mono sum, shift glide, L/R shifters, mix, limiter, NaN guard"
```

---

### Task 5: Render tool (listen on the Mac)

**Files:**
- Create: `sim/wav.h`
- Create: `sim/render.cpp`
- Create: `sim/render.sh`

- [ ] **Step 1: Write the WAV helper**

`sim/wav.h`:
```cpp
#pragma once
// Minimal 16-bit PCM WAV reading (mono or stereo) and writing (stereo).
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

struct Wav {
    int sample_rate = 48000;
    std::vector<float> left, right;
};

inline bool ReadWav(const char *path, Wav &wav) {
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    char id[4];
    uint32_t size;
    if (fread(id, 1, 4, f) != 4 || memcmp(id, "RIFF", 4) || fread(&size, 4, 1, f) != 1 ||
        fread(id, 1, 4, f) != 4 || memcmp(id, "WAVE", 4)) {
        fclose(f);
        return false;
    }
    uint16_t format = 0, channels = 0, bits = 0;
    uint32_t rate = 0;
    while (fread(id, 1, 4, f) == 4 && fread(&size, 4, 1, f) == 1) {
        if (!memcmp(id, "fmt ", 4)) {
            uint8_t fmt[40] = {};
            if (fread(fmt, 1, size < 40 ? size : 40, f) < 16) break;
            if (size > 40) fseek(f, size - 40, SEEK_CUR);
            memcpy(&format, fmt, 2);
            memcpy(&channels, fmt + 2, 2);
            memcpy(&rate, fmt + 4, 4);
            memcpy(&bits, fmt + 14, 2);
        } else if (!memcmp(id, "data", 4)) {
            if (format != 1 || bits != 16 || channels < 1 || channels > 2) break;
            std::vector<int16_t> samples(size / 2);
            size_t got = fread(samples.data(), 2, samples.size(), f);
            wav.sample_rate = (int)rate;
            for (size_t i = 0; i + channels <= got; i += channels) {
                wav.left.push_back(samples[i] / 32768.f);
                wav.right.push_back(samples[i + channels - 1] / 32768.f);
            }
            fclose(f);
            return true;
        } else {
            fseek(f, size + (size & 1), SEEK_CUR);
        }
    }
    fclose(f);
    return false;
}

inline bool WriteWav(const char *path, const Wav &wav) {
    FILE *f = fopen(path, "wb");
    if (!f) return false;
    uint32_t frames = (uint32_t)wav.left.size(), data = frames * 4, riff = 36 + data;
    uint32_t rate = (uint32_t)wav.sample_rate, bytes_per_second = rate * 4, fmt_size = 16;
    uint16_t format = 1, channels = 2, block_align = 4, bits = 16;
    fwrite("RIFF", 1, 4, f);
    fwrite(&riff, 4, 1, f);
    fwrite("WAVEfmt ", 1, 8, f);
    fwrite(&fmt_size, 4, 1, f);
    fwrite(&format, 2, 1, f);
    fwrite(&channels, 2, 1, f);
    fwrite(&rate, 4, 1, f);
    fwrite(&bytes_per_second, 4, 1, f);
    fwrite(&block_align, 2, 1, f);
    fwrite(&bits, 2, 1, f);
    fwrite("data", 1, 4, f);
    fwrite(&data, 4, 1, f);
    for (uint32_t i = 0; i < frames; i++) {
        for (float v : {wav.left[i], wav.right[i]}) {
            float c = v < -1.f ? -1.f : (v > 1.f ? 1.f : v);
            int16_t s = (int16_t)(c * 32767.f);
            fwrite(&s, 2, 1, f);
        }
    }
    fclose(f);
    return true;
}
```

- [ ] **Step 2: Write the render tool**

`sim/render.cpp`:
```cpp
// Renders a 16-bit WAV through the Bitchshifter engine.
// Usage: render in.wav out.wav <tune_hz> [feedback 0-0.95] [mix 0-1]
#include <cstdio>
#include <cstdlib>

#include "engine.h"
#include "wav.h"

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "Usage: %s in.wav out.wav tune_hz [feedback] [mix]\n", argv[0]);
        return 1;
    }
    Wav in;
    if (!ReadWav(argv[1], in)) {
        fprintf(stderr, "Couldn't read %s (needs 16-bit PCM WAV, mono or stereo)\n", argv[1]);
        return 1;
    }
    bitch::Params p;
    p.tune_hz = (float)atof(argv[3]);
    p.feedback = argc > 4 ? (float)atof(argv[4]) : 0.f;
    p.mix = argc > 5 ? (float)atof(argv[5]) : 1.f;
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
    printf("Wrote %s (shift %g Hz, feedback %g, mix %g)\n", argv[2], p.tune_hz, p.feedback, p.mix);
    return 0;
}
```

`sim/render.sh`:
```sh
#!/bin/sh
# Builds the render tool and runs it: render.sh in.wav out.wav tune_hz [feedback] [mix]
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$HERE/out"
SDK=$(ls -d /Library/Developer/CommandLineTools/SDKs/MacOSX26*.sdk 2>/dev/null | head -1)
c++ ${SDK:+-isysroot "$SDK"} -std=c++17 -O2 -Wall -I"$HERE/../engine" "$HERE/render.cpp" "$HERE/../engine/engine.cpp" -o "$HERE/out/render"
"$HERE/out/render" "$@"
```

Run: `chmod +x ~/local_files/daisy/bitchshifter/sim/render.sh`

- [ ] **Step 3: Make a test voice and render it**

```bash
cd ~/local_files/daisy/bitchshifter/sim/out
say -o voice.aiff "This is station Bitchshifter, broadcasting on the forty metre band. Can you read me?"
afconvert -f WAVE -d LEI16@48000 voice.aiff voice.wav
../render.sh voice.wav shift_up_300.wav 300
../render.sh voice.wav shift_down_200.wav -200
../render.sh voice.wav barberpole.wav 4 0.85
../render.sh voice.wav detuned_8.wav 8 0 0.5
```
Expected: four `Wrote …` lines. Open the folder (`open .`) and play the files: voice shifted up (squeaky, inharmonic), down (bassy, metallic), a rising barberpole swirl, and a phasey near-detune at 50% mix.

- [ ] **Step 4: User listens, then commit**

Ask the user to listen and say whether it sounds right before committing.
```bash
cd ~/local_files/daisy/bitchshifter
git add sim/wav.h sim/render.cpp sim/render.sh
git commit -m "Mac render tool for listening to the engine"
```

---

### Task 6: Simple Touch firmware (TUNE, FEEDBACK, MIX)

**Files:**
- Create: `simple_touch.cpp`
- Create: `Makefile`
- Modify: `README.md`

- [ ] **Step 1: Write the firmware**

`simple_touch.cpp` (set the four `kInput…`/`kFlip…` values from the control map measured in Task 1, recorded in `README.md`):
```cpp
// Bitchshifter for the Synthux Simple Touch: part 1, a playable frequency shifter.
// Fader 1 TUNE (shift, 0 at the centre), knob 5 FEEDBACK, knob 6 MIX. The CPU
// load is printed over USB serial once a second.
#include "daisy_seed.h"
#include "engine.h"
#include "util/CpuLoadMeter.h"

using namespace daisy;

// Control map, from hwtest/ (see README.md): A-index of each control, and whether
// it reads reversed (needs flipping so that up/clockwise = more)
const int kInputTune = 0;     // fader 1
const int kInputFeedback = 4; // knob 5
const int kInputMix = 5;      // knob 6
const bool kFlipTune = false;
const bool kFlipFeedback = false;
const bool kFlipMix = false;

const float kMaxShiftHz = 2500.f;
const float kTuneDeadZone = 0.02f; // around the centre, so exactly 0 Hz is easy to find

DaisySeed hw;
AnalogControl controls[8];
bitch::Engine engine;
CpuLoadMeter cpu;

float Read(int input, bool flip) {
    float v = controls[input].Value();
    return flip ? 1.f - v : v;
}

// Fader 0..1 -> -2500..+2500 Hz, cubic so it's fine near 0 and wild at the ends
float TuneHz(float fader) {
    float x = 2.f * fader - 1.f;
    if (fabsf(x) < kTuneDeadZone) return 0.f;
    x = (x - copysignf(kTuneDeadZone, x)) / (1.f - kTuneDeadZone);
    return kMaxShiftHz * x * x * x;
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
    cpu.OnBlockStart();
    for (AnalogControl &c : controls) c.Process();
    bitch::Params p;
    p.tune_hz = TuneHz(Read(kInputTune, kFlipTune));
    p.feedback = 0.95f * Read(kInputFeedback, kFlipFeedback);
    p.mix = Read(kInputMix, kFlipMix);
    engine.Process(in[0], in[1], out[0], out[1], size, p);
    cpu.OnBlockEnd();
}

int main(void) {
    hw.Init(true); // 480 MHz
    hw.SetAudioBlockSize(48);
    hw.StartLog(false);

    AdcChannelConfig cfg[8];
    Pin pins[8] = {seed::A0, seed::A1, seed::A2, seed::A3, seed::A4, seed::A5, seed::A6, seed::A7};
    for (int i = 0; i < 8; i++) cfg[i].InitSingle(pins[i]);
    hw.adc.Init(cfg, 8);
    for (int i = 0; i < 8; i++) controls[i].Init(hw.adc.GetPtr(i), hw.AudioCallbackRate());
    hw.adc.Start();

    engine.Init(hw.AudioSampleRate());
    cpu.Init(hw.AudioSampleRate(), hw.AudioBlockSize());
    hw.StartAudio(AudioCallback);

    while (1) {
        System::Delay(1000);
        hw.PrintLine("CPU avg %d%% max %d%% | tune %d Hz", (int)(cpu.GetAvgCpuLoad() * 100.f),
                     (int)(cpu.GetMaxCpuLoad() * 100.f), (int)TuneHz(Read(kInputTune, kFlipTune)));
        cpu.Reset();
    }
}
```

`Makefile`:
```make
# Bitchshifter for the Synthux Simple Touch
TARGET = Bitchshifter
CPP_SOURCES = simple_touch.cpp engine/engine.cpp
LIBDAISY_DIR = ../libDaisy
APP_TYPE = BOOT_SRAM
C_INCLUDES += -Iengine
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
```

- [ ] **Step 2: Build**

Run: `cd ~/local_files/daisy/bitchshifter && make`
Expected: memory table, no errors or warnings from `simple_touch.cpp` or `engine/`.

- [ ] **Step 3: User flashes and plays it**

User runs `make program-dfu` (RESET, then BOOT while the LED pulses). Test with a source into the inputs and outputs to a mixer:
- TUNE at the centre, MIX fully up, FEEDBACK down → the input passes unchanged (0 Hz shift).
- TUNE slightly up/down → the sound goes inharmonic, squeaky above, metallic below; it sweeps smoothly through the centre.
- FEEDBACK up with TUNE just off centre → rising or falling barberpole swirl, no runaway.
- Read the CPU over USB: `cat /dev/cu.usbmodem*`. Expect a low figure (roughly 10–20%).

- [ ] **Step 4: Document and commit (after the user confirms it works)**

Append to `README.md`:
```markdown
## Part 1: playable shifter

| Control | Does |
|---|---|
| Fader 1 TUNE | Shift −2500 … 0 (centre) … +2500 Hz, fine near the centre |
| Knob 5 FEEDBACK | Feedback 0–95% (barberpole) |
| Knob 6 MIX | Dry/wet |

Build and flash: `make`, then RESET, BOOT while the LED pulses, `make program-dfu`.
CPU load over USB serial: `cat /dev/cu.usbmodem*`.
Mac tests: `sim/run_tests.sh`. Listen on the Mac: `sim/render.sh in.wav out.wav <tune_hz> [feedback] [mix]`.
```
```bash
cd ~/local_files/daisy/bitchshifter
git add simple_touch.cpp Makefile README.md
git commit -m "Simple Touch firmware: playable shifter (TUNE, FEEDBACK, MIX)"
```

---

## Later plans (not part of this one)

- **Part 2:** ring mod (knobs 1–2) and radio filter (knobs 3–4), with Mac tests.
- **Part 3:** Reception (fader 2): drift (L/R independent), fading, static, AGC, drive.
- **Part 4:** pads and switches: stations (QSPI 0x7C0000, store confirmation chirp, takeover),
  keyboard (hold, octave switch), performance pads.
- **Part 5 (optional):** reverb; Daisy Patch front end.
