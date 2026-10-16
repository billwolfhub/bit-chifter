// Mac tests for the bit-chifter engine. Build and run with sim/run_tests.sh.
#include <cmath>
#include <cstdio>
#include <vector>
#include <cstring>

#include "blips.h"
#include "engine.h"
#include "hilbert.h"
#include "softclip.h"
#include "shifter.h"
#include "mapping.h"
#include "radio_filter.h"
#include "ring_mod.h"
#include "agc.h"
#include "static_noise.h"
#include "fading.h"
#include "drive.h"
#include "detents.h"
#include "pitch_tracker.h"
#include "random.h"
#include "plate_reverb.h"
#include "../ui/pad_pages.h"
#include "effects.h"
#include "scrambler.h"
#include "carriers.h"
#include "morse_chimes.h"
#include "selcall.h"
#include "jammer.h"
#include "codec_warble.h"
#include "stutter.h"
#include "dropouts.h"
#include "crush.h"
#include <algorithm>
#include <cstdint>

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
    for (float freq : {40.f, 100.f, 1000.f, 5000.f, 15000.f}) {
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
    // L and R are identical until Reception makes them drift apart
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

static void TestEngineNonFiniteMidBlock() {
    for (float bad : {(float)NAN, (float)INFINITY}) {
        bitch::Engine e;
        e.Init(kSampleRate);
        bitch::Params p;
        p.tune_hz = 100.f;
        float in[48], out_l[48], out_r[48];
        for (int k = 0; k < 48; k++) in[k] = 0.25f * sinf(2 * kPi * 1000.f * k / kSampleRate);
        in[std::isnan(bad) ? 20 : 47] = bad;
        e.Process(in, in, out_l, out_r, 48, p);
        for (int k = 0; k < 48; k++)
            CHECK(std::isfinite(out_l[k]) && std::isfinite(out_r[k]), "mid-block %g: output %d not finite", bad, k);
        for (int k = 0; k < 48; k++) in[k] = 0.25f * sinf(2 * kPi * 1000.f * k / kSampleRate);
        e.Process(in, in, out_l, out_r, 48, p);
        for (int k = 0; k < 48; k++)
            CHECK(std::isfinite(out_l[k]) && std::isfinite(out_r[k]), "mid-block %g: next block %d not finite", bad, k);
    }
}

static void TestEngineNaNTune() {
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.tune_hz = NAN;
    float in[48], out_l[48], out_r[48];
    for (int b = 0; b < 20; b++) {
        for (int k = 0; k < 48; k++) in[k] = 0.25f * sinf(2 * kPi * 1000.f * (b * 48 + k) / kSampleRate);
        e.Process(in, in, out_l, out_r, 48, p);
        for (int k = 0; k < 48; k++)
            CHECK(std::isfinite(out_l[k]) && std::isfinite(out_r[k]), "NaN tune: block %d sample %d not finite", b, k);
    }
    float last_peak = 0.f;
    for (int k = 0; k < 48; k++) last_peak = std::max(last_peak, fabsf(out_l[k]));
    CHECK(last_peak > 0.01f, "NaN tune: output silent (peak %.4f), clamp not working", last_peak);
}

static void TestSoftClip() {
    bool bounded = true;
    for (float x = -4.f; x <= 4.f; x += 0.0001f) bounded = bounded && fabsf(bitch::SoftClip(x)) <= 1.f;
    CHECK(bounded, "softclip: output exceeds 1");
    CHECK(fabsf(bitch::SoftClip(0.1f) - 0.1f) < 0.001f, "softclip: slope near 0");
    CHECK(std::isnan(bitch::SoftClip(NAN)), "softclip: NaN must pass through");
}

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

static void TestMappings() {
    CHECK(bitch::TuneHz(0.5f) == 0.f, "tune centre %.2f", bitch::TuneHz(0.5f));
    CHECK(bitch::TuneHz(0.51f) == 0.f, "tune dead zone %.2f", bitch::TuneHz(0.51f));
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
    bitch::FilterWidth(0.98f, q, open);
    CHECK(open == 1.f, "width 0.98: open %.2f", open);
    CHECK(bitch::ReceptionAmount(0.f) == 0.f, "reception bottom %.3f", bitch::ReceptionAmount(0.f));
    CHECK(bitch::ReceptionAmount(0.02f) == 0.f, "reception bottom dead zone %.3f", bitch::ReceptionAmount(0.02f));
    CHECK(bitch::ReceptionAmount(0.98f) == 1.f, "reception top %.3f", bitch::ReceptionAmount(0.98f));
    CHECK(fabsf(bitch::ReceptionAmount(0.5f) - 0.5f) < 0.01f, "reception middle %.3f", bitch::ReceptionAmount(0.5f));
}

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
    wr = Window(r, 0.5f, 1.f);
    CHECK(fabs(Magnitude(wl, 1100.f) - 0.245) < 0.02, "mono R-only: OUT L 1100 Hz %.3f", Magnitude(wl, 1100.f));
    CHECK(fabs(Magnitude(wr, 1100.f) - 0.245) < 0.02, "mono R-only: OUT R 1100 Hz %.3f", Magnitude(wr, 1100.f));
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


// Switching stereo mode must not click: the biggest sample-to-sample step around each
// switch stays under twice the biggest step in steady Linked playback
static void TestEngineModeSwitchNoClick() {
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.tune_hz = 300.f;
    const size_t block = 48;
    std::vector<float> l(block), r(block), ol(block), orr(block), out;
    int total = (int)(2.5f * kSampleRate) / block;
    for (int b = 0; b < total; b++) {
        float t0 = b * block / kSampleRate;
        p.stereo_mode = (t0 >= 1.f && t0 < 2.f) ? bitch::StereoMode::kRotate : bitch::StereoMode::kLinked;
        for (size_t k = 0; k < block; k++) {
            float t = (b * block + k) / kSampleRate;
            l[k] = 0.25f * sinf(2 * kPi * 1000.f * t);
            r[k] = 0.25f * sinf(2 * kPi * 1500.f * t);
        }
        e.Process(l.data(), r.data(), ol.data(), orr.data(), block, p);
        out.insert(out.end(), ol.begin(), ol.end());
    }
    auto max_step = [&](float from_s, float to_s) {
        float m = 0.f;
        for (int i = (int)(from_s * kSampleRate); i < (int)(to_s * kSampleRate); i++)
            m = std::max(m, fabsf(out[i + 1] - out[i]));
        return m;
    };
    float steady = max_step(0.5f, 1.f);
    float to_rotate = max_step(0.9f, 1.1f), to_linked = max_step(1.9f, 2.1f);
    CHECK(to_rotate < 2.f * steady, "mode switch to Rotate: step %.4f vs steady %.4f", to_rotate, steady);
    CHECK(to_linked < 2.f * steady, "mode switch to Linked: step %.4f vs steady %.4f", to_linked, steady);
}

// Rotate half way round: L is shifted down and R up, and the pair is back on its own sides
static void TestEngineRotateSweep() {
    bitch::Params p;
    p.tune_hz = 100.f;
    p.stereo_mode = bitch::StereoMode::kRotate;
    std::vector<float> l, r;
    RunEngineStereo(p, 4.2f, 1000.f, 0.25f, 1000.f, 0.25f, l, r);
    auto w = Window(l, 3.95f, 4.05f);
    CHECK(Magnitude(w, 900.f) > 0.06, "rotate sweep 4 s: 900 Hz %.3f", Magnitude(w, 900.f));
    CHECK(Magnitude(w, 1100.f) > 0.06, "rotate sweep 4 s: 1100 Hz %.3f", Magnitude(w, 1100.f));
}

// Back in Linked after Mirror, the two oscillators must be back in phase: the mono sum
// keeps its level
static void TestEngineLinkedAfterMirror() {
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.tune_hz = 100.f;
    const size_t block = 48;
    std::vector<float> in(block), ol(block), orr(block), l, mono;
    int total = (int)(3.f * kSampleRate) / block;
    for (int b = 0; b < total; b++) {
        float t0 = b * block / kSampleRate;
        p.stereo_mode = t0 < 1.0025f ? bitch::StereoMode::kMirror : bitch::StereoMode::kLinked;
        for (size_t k = 0; k < block; k++) in[k] = 0.25f * sinf(2 * kPi * 1000.f * (b * block + k) / kSampleRate);
        e.Process(in.data(), in.data(), ol.data(), orr.data(), block, p);
        if (t0 >= 2.5f)
            for (size_t k = 0; k < block; k++) {
                l.push_back(ol[k]);
                mono.push_back(0.5f * (ol[k] + orr[k]));
            }
    }
    double ml = Magnitude(l, 1100.f), mm = Magnitude(mono, 1100.f);
    CHECK(fabs(Db(mm / ml)) < 1.0, "linked after mirror: mono sum %.3f vs L %.3f", mm, ml);
}

// The silent side starts taking the other side's signal within 0.2 s
static void TestEngineMonoTakeoverTiming() {
    bitch::Params p;
    p.tune_hz = 100.f;
    std::vector<float> l, r;
    RunEngineStereo(p, 0.4f, 1000.f, 0.25f, 0.f, 0.f, l, r);
    auto w = Window(r, 0.2f, 0.4f);
    CHECK(Magnitude(w, 1100.f) > 0.2, "mono takeover: OUT R 1100 Hz at 0.2 s %.3f", Magnitude(w, 1100.f));
}

// A side that is only silent between short bursts must not flip to taking the other side
static void TestEngineIntermittentSideNoFlap() {
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p; // tune 0
    const size_t block = 48;
    std::vector<float> l(block), r(block), ol(block), orr(block), out_r;
    int total = (int)(10.f * kSampleRate) / block;
    for (int b = 0; b < total; b++) {
        for (size_t k = 0; k < block; k++) {
            float t = (b * block + k) / kSampleRate;
            l[k] = 0.25f * sinf(2 * kPi * 1000.f * t);
            bool burst = fmodf(t, 2.f) < 0.05f;
            r[k] = burst ? 0.25f * sinf(2 * kPi * 1500.f * t) : 0.f;
        }
        e.Process(l.data(), r.data(), ol.data(), orr.data(), block, p);
        out_r.insert(out_r.end(), orr.begin(), orr.end());
    }
    for (float t = 0.5f; t + 0.25f <= 10.f; t += 0.25f) {
        double m = Magnitude(Window(out_r, t, t + 0.25f), 1000.f);
        CHECK(m < 0.02, "intermittent R: L's 1000 Hz on OUT R at %.2f s: %.3f", t, m);
    }
}

// Start(n) plays n separate beeps, then silence
static void TestBlips() {
    for (int n = 1; n <= 3; n++) {
        bitch::Blips b;
        b.Init(kSampleRate);
        b.Start(n);
        // A burst = sound after at least 200 silent samples (the start counts as silence)
        int bursts = 0, silent_run = 1000;
        float peak = 0.f, prev = 0.f, max_step = 0.f;
        for (int t = 0; t < (int)kSampleRate; t++) {
            float s = b.Process();
            float v = fabsf(s);
            // each beep starts at a multiple of 150 ms: its first sample must be near silent
            if (t % 7200 == 0 && t / 7200 < n) CHECK(v < 0.01f, "blips: beep %d starts at %.3f", t / 7200, v);
            if (v > 0.01f) {
                if (silent_run >= 200) bursts++;
                silent_run = 0;
            } else {
                silent_run++;
            }
            max_step = std::max(max_step, fabsf(s - prev));
            peak = std::max(peak, v);
            prev = s;
        }
        CHECK(peak > 0.18f && peak < 0.21f, "blips: peak %.3f", peak);
        CHECK(max_step < 0.045f, "blips: max step %.4f", max_step);
        CHECK(bursts == n, "blips: Start(%d) gave %d bursts", n, bursts);
        CHECK(fabsf(b.Process()) < 1e-6f, "blips: still sounding after 1 s");
    }
}

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
    int crackles = 0;
    int since_last = 1 << 30;
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
    CHECK(fabs(Db(mid / 0.3)) < 1.5, "agc: 0.1 in -> peak %.3f (want 0.3)", mid);
    CHECK(fabs(Db(loud / 0.3)) < 1.5, "agc: 0.9 in -> peak %.3f (want 0.3)", loud);
    CHECK(fabs(Db(quiet / (0.02 * 7.94))) < 1.5, "agc: 0.02 in -> peak %.3f (want the +18 dB limit, 0.159)", quiet);
}

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
    CHECK(rms < 0.5f, "reception 1, silence: static rms too loud %.4f", rms);
    int hot = 0;
    for (float v : l) hot += fabsf(v) > 0.95f;
    CHECK(hot < (int)l.size() / 100, "reception 1, silence: %d of %zu samples above 0.95", hot, l.size());
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

// Everything at once, on noise, for 60 s: finite and within the limiter
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
    float in[48], out_l[48], out_r[48];
    int hot = 0;
    long total = 0;
    bool finite = true;
    for (int b = 0; b < (int)(60 * kSampleRate / 48); b++) {
        for (float &v : in) {
            seed = seed * 1664525u + 1013904223u;
            v = ((seed >> 8) / 8388608.f - 1.f) * 0.5f;
        }
        e.Process(in, in, out_l, out_r, 48, p);
        for (int k = 0; k < 48; k++) {
            finite = finite && std::isfinite(out_l[k]) && std::isfinite(out_r[k]);
            hot += (fabsf(out_l[k]) > 0.95f) + (fabsf(out_r[k]) > 0.95f);
            total += 2;
        }
    }
    CHECK(finite, "reception stress: non-finite output");
    CHECK(hot < total / 100, "reception stress: %d of %ld samples above 0.95", hot, total);
}

static void TestDriveOnset() {
    auto run = [](float amount) {
        bitch::Drive d;
        d.Set(amount);
        std::vector<float> out;
        for (int t = 0; t < (int)kSampleRate; t++) out.push_back(d.Process(0.9f * sinf(2 * kPi * 1000.f * t / kSampleRate)));
        return out;
    };
    auto rms = [](const std::vector<float> &x) {
        double s = 0;
        for (float v : x) s += v * v;
        return sqrt(s / x.size());
    };
    auto a = run(0.f), b = run(0.001f);
    CHECK(fabs(Db(rms(b) / rms(a))) < 0.05, "drive onset: level jumps %.3f dB", Db(rms(b) / rms(a)));
    double h3 = Magnitude(b, 3000.f), f1 = Magnitude(b, 1000.f);
    CHECK(h3 < 0.9 * f1 * pow(10.0, -60.0 / 20.0), "drive onset: 3rd harmonic %.1f dB", Db(h3 / f1));
}

static void TestRandomSeedsScrambled() {
    const uint32_t seeds[] = {11, 29, 244, 478, 82, 208};
    float lo = 1.f, hi = -1.f;
    bool any_positive = false;
    for (uint32_t s : seeds) {
        bitch::Random r(s);
        float v = r.Bipolar();
        lo = std::min(lo, v);
        hi = std::max(hi, v);
        any_positive = any_positive || v > 0.f;
    }
    CHECK(any_positive, "seeds: first values all negative");
    CHECK(hi - lo > 0.5f, "seeds: first values spread only %.3f", hi - lo);
}

static void TestWanderRateChange() {
    bitch::Wander w;
    w.Init(1000.f, 5);
    for (int b = 0; b < 60000; b++) w.Next(0.05f);
    float prev = w.Next(0.5f), prev_sign = 0.f;
    int reversals = 0;
    for (int b = 0; b < 3000; b++) {
        float v = w.Next(0.5f);
        float sign = v > prev ? 1.f : (v < prev ? -1.f : 0.f);
        if (sign != 0.f) {
            if (prev_sign != 0.f && sign != prev_sign) reversals++;
            prev_sign = sign;
        }
        prev = v;
    }
    CHECK(reversals >= 1, "wander: no new target within 3 s after raising the rate");
}

static void TestFadingReturnsToExact() {
    bitch::Fading f;
    f.Init(kSampleRate, kSampleRate / 48.f, 101);
    int exact_bad = 0, n = 0;
    for (int b = 0; b < (int)(5 * kSampleRate / 48); b++) {
        f.Update(b < (int)(2 * kSampleRate / 48) ? 1.f : 0.f);
        for (int k = 0; k < 48; k++) {
            float x = 0.3f * sinf(0.05f * (b * 48 + k));
            float y = f.Process(x);
            if (b >= (int)(4.5f * kSampleRate / 48)) {
                n++;
                exact_bad += y != x;
            }
        }
    }
    CHECK(n > 0 && exact_bad == 0, "fading: %d of %d samples inexact after returning to 0", exact_bad, n);
}

static void TestEngineLinkedDrifts() {
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.tune_hz = 100.f;
    p.reception = 0.3f;
    const size_t block = 48;
    std::vector<float> in(block), ol(block), orr(block);
    int total = (int)(6.f * kSampleRate) / block, start = (int)(2.f * kSampleRate) / block;
    float prev = 0.f, unwrapped = 0.f, first = 0.f;
    for (int b = 0; b < total; b++) {
        for (size_t k = 0; k < block; k++) in[k] = 0.25f * sinf(2 * kPi * 1000.f * (b * block + k) / kSampleRate);
        e.Process(in.data(), in.data(), ol.data(), orr.data(), block, p);
        float d = e.ShifterPhase(0) - e.ShifterPhase(1);
        d -= floorf(d + 0.5f);
        if (b == start) first = unwrapped = d;
        else if (b > start) {
            float step = d - prev;
            step -= floorf(step + 0.5f);
            unwrapped += step;
        }
        prev = d;
    }
    CHECK(fabsf(unwrapped - first) > 2.f, "linked drift: phase difference moved only %.2f cycles", unwrapped - first);
}

static void TestEngineReceptionReturnsClean() {
    const size_t block = 48;
    auto run = [&](bool noisy_first, std::vector<float> &l, std::vector<float> &r) {
        bitch::Engine e;
        e.Init(kSampleRate);
        bitch::Params p;
        p.tune_hz = 100.f;
        std::vector<float> in(block), ol(block), orr(block);
        int total = (int)(6.f * kSampleRate) / block;
        l.clear();
        r.clear();
        for (int b = 0; b < total; b++) {
            float t0 = b * block / kSampleRate;
            p.reception = (noisy_first && t0 < 2.f) ? 1.f : 0.f;
            for (size_t k = 0; k < block; k++) in[k] = 0.25f * sinf(2 * kPi * 1000.f * (b * block + k) / kSampleRate);
            e.Process(in.data(), in.data(), ol.data(), orr.data(), block, p);
            if (t0 >= 5.5f)
                for (size_t k = 0; k < block; k++) {
                    l.push_back(ol[k]);
                    r.push_back(orr[k]);
                }
        }
    };
    std::vector<float> l, r, fl, fr;
    run(true, l, r);
    run(false, fl, fr);
    int diff = 0;
    for (size_t i = 0; i < l.size(); i++) diff += l[i] != r[i];
    CHECK(diff == 0, "returns clean: %d samples differ between L and R", diff);
    CHECK(fabs(Db(Magnitude(l, 1100.f) / Magnitude(fl, 1100.f))) < 0.5, "returns clean: 1100 Hz off by %.2f dB",
          Db(Magnitude(l, 1100.f) / Magnitude(fl, 1100.f)));
}

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

static float Sine(float hz, int n) { return 0.25f * sinf(2 * kPi * hz * n / kSampleRate); }
static float Saw(float hz, int n) {
    float x = hz * n / kSampleRate;
    return 0.25f * (2.f * (x - floorf(x)) - 1.f);
}

// A pitch buried in noise (no dip below the YIN threshold) still gives a partial confidence
static void TestPitchTrackerGradedConfidence() {
    bitch::PitchTracker t;
    uint32_t seed = 7;
    Track(t, 0.5f, [&seed](int n) {
        seed = seed * 1664525u + 1013904223u;
        return Sine(200.f, n) + 0.8f * ((seed >> 8) / 8388608.f - 1.f);
    });
    CHECK(t.Confidence() > 0.2f && t.Confidence() < 0.85f && fabsf(t.Pitch() - 200.f) < 6.f,
          "pitch: 200 Hz in noise -> %.2f Hz (conf %.2f)", t.Pitch(), t.Confidence());
}

// Pitches just outside 60-800 Hz must not be reported at the range edge with high confidence
static void TestPitchTrackerRangeEdges() {
    bitch::PitchTracker t;
    for (int saw = 0; saw < 2; saw++) {
        const char *name = saw ? "saw" : "sine";
        for (float hz : {820.f, 850.f}) {
            Track(t, 0.5f, [saw, hz](int n) { return saw ? Saw(hz, n) : Sine(hz, n); });
            float p = t.Pitch(), c = t.Confidence();
            bool subharmonic = fabsf(p - hz / 2.f) < 0.03f * hz / 2.f || fabsf(p - hz / 3.f) < 0.03f * hz / 3.f;
            CHECK(c <= 0.5f || subharmonic, "pitch edge: %.0f Hz %s -> %.2f Hz (conf %.2f)", hz, name, p, c);
        }
        for (float hz : {55.f, 59.f}) {
            Track(t, 0.5f, [saw, hz](int n) { return saw ? Saw(hz, n) : Sine(hz, n); });
            float p = t.Pitch(), c = t.Confidence();
            CHECK(c < 0.5f || fabsf(p - hz) < 2.f, "pitch edge: %.0f Hz %s -> %.2f Hz (conf %.2f)", hz, name, p, c);
        }
    }
}

// Runs an engine block by block on a signal into both inputs, returning each side's
// shifter frequency (from its phase advance) at the end of each block
template <class Signal>
static void ShiftTraces(bitch::Engine &e, const bitch::Params &p, float seconds, Signal signal,
                        std::vector<float> &left, std::vector<float> &right) {
    float in[48], out_l[48], out_r[48];
    float prev[2] = {e.ShifterPhase(0), e.ShifterPhase(1)};
    int blocks = (int)(seconds * kSampleRate / 48);
    for (int b = 0; b < blocks; b++) {
        for (int k = 0; k < 48; k++) in[k] = signal(b * 48 + k);
        e.Process(in, in, out_l, out_r, 48, p);
        for (int c = 0; c < 2; c++) {
            float phase = e.ShifterPhase(c), d = phase - prev[c];
            d -= floorf(d + 0.5f);
            (c == 0 ? left : right).push_back(d * kSampleRate / 48.f);
            prev[c] = phase;
        }
    }
}

// The left side's shift only
template <class Signal>
static std::vector<float> ShiftTrace(bitch::Engine &e, const bitch::Params &p, float seconds, Signal signal) {
    std::vector<float> left, right;
    ShiftTraces(e, p, seconds, signal, left, right);
    return left;
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

// Inertia from the very first block: the dial starts at TUNE, it doesn't sweep in from 0
static void TestEngineDialInertiaFromStart() {
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.tune_hz = 200.f;
    p.dial_inertia_s = 0.06f;
    auto trace = ShiftTrace(e, p, 0.1f, [](int) { return 0.f; });
    CHECK(trace[50] > 180.f, "inertia from start: only %.1f Hz after 50 ms", trace[50]);
}

// When the source stops, the detents fade out (in ~150 ms, so a dropped analysis doesn't
// pump them) and the shift returns to TUNE. Pulled to ~111.5 Hz; one block = 1 ms
static void TestEngineDetentsRelease() {
    bitch::Params p;
    p.tune_hz = 100.f;
    p.detent = 0.7f;
    bitch::Engine e;
    e.Init(kSampleRate);
    ShiftTrace(e, p, 1.5f, Sawtooth120);
    auto after = ShiftTrace(e, p, 0.75f, [](int) { return 0.f; });
    CHECK(after[100] > 105.f, "detents release: already at %.2f Hz 100 ms after the source stopped", after[100]);
    CHECK(fabsf(after.back() - 100.f) < 0.5f, "detents release: %.2f Hz 0.75 s after the source stopped",
          after.back());
}

// While the detents release, a weak, wrong pitch (noise plus a faint 530 Hz tone) must not
// take over the pull: the shift only goes from the 120 Hz detent (~111.5 Hz) back to TUNE
static void TestEngineDetentsHoldPitch() {
    bitch::Params p;
    p.tune_hz = 100.f;
    p.detent = 0.7f;
    bitch::Engine e;
    e.Init(kSampleRate);
    ShiftTrace(e, p, 1.5f, Sawtooth120);
    uint32_t seed = 7;
    auto after = ShiftTrace(e, p, 0.3f, [&seed](int n) {
        seed = seed * 1664525u + 1013904223u;
        return 0.15f * sinf(2 * kPi * 530.f * n / kSampleRate) + 0.8f * ((seed >> 8) / 8388608.f - 1.f);
    });
    float lowest = *std::min_element(after.begin(), after.end());
    CHECK(lowest > 99.f, "detents hold pitch: shift fell to %.2f Hz on a weak wrong pitch", lowest);
}

// Mirror with detents: the right side's shift is exactly the opposite of the left's
static void TestEngineMirrorDetents() {
    bitch::Params p;
    p.tune_hz = 100.f;
    p.detent = 0.7f;
    p.stereo_mode = bitch::StereoMode::kMirror;
    bitch::Engine e;
    e.Init(kSampleRate);
    std::vector<float> l, r;
    ShiftTraces(e, p, 1.5f, Sawtooth120, l, r);
    CHECK(l.back() > 104.f && l.back() < 119.f, "mirror detents: left %.2f Hz", l.back());
    float worst = 0.f;
    for (size_t i = l.size() / 2; i < l.size(); i++) worst = std::max(worst, fabsf(l[i] + r[i]));
    CHECK(worst < 0.01f, "mirror detents: right differs from -left by %.4f Hz", worst);
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

// Runs a (static, large) engine on a 440 Hz tone for `tone_s`, then silence for `silence_s`,
// with reverb `reverb_tone` during the tone and `reverb_silence` after; returns the left output
static std::vector<float> RunToneThenSilence(float tone_s, float silence_s, float reverb_tone, float reverb_silence) {
    static bitch::Engine e;
    static bitch::PlateReverb plate;
    e.Init(kSampleRate, 48, &plate);
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
    static bitch::PlateReverb plate;
    e.Init(kSampleRate, 48, &plate);
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
    // No plate given: Params::reverb is ignored (it still runs, and leaves no tail)
    static bitch::Engine bare;
    bare.Init(kSampleRate);
    p = bitch::Params();
    p.reverb = 1.f;
    float tail = 0.f;
    for (int b = 0; b < (int)(1.5f * kSampleRate / 48); b++) {
        for (int k = 0; k < 48; k++)
            in[k] = b < (int)(0.5f * kSampleRate / 48) ? 0.3f * sinf(2 * kPi * 440.f * (b * 48 + k) / kSampleRate) : 0.f;
        bare.Process(in, in, out_l, out_r, 48, p);
        if (b * 48 >= (int)(0.6f * kSampleRate))
            for (int k = 0; k < 48; k++) tail = std::max(tail, std::max(fabsf(out_l[k]), fabsf(out_r[k])));
    }
    CHECK(tail < 1e-4f, "reverb without a plate: %g after the tone", tail);
}

static void TestEngineReverbStable() {
    static bitch::Engine e;
    static bitch::PlateReverb plate;
    e.Init(kSampleRate, 48, &plate);
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

static void TestDelayLine() {
    static bitch::DelayLine<100> line;
    line.Clear();
    line.Write(1.f);
    for (int n = 0; n < 9; n++) line.Write(0.f);
    // Ten writes since the impulse's: it is the 10th newest
    bool only_at_10 = true;
    for (int d = 1; d <= 100; d++) only_at_10 = only_at_10 && line.Tap(d) == (d == 10 ? 1.f : 0.f);
    CHECK(only_at_10, "delay line: the impulse isn't (only) at Tap(10)");
    CHECK(line.Tap(9.5f) == 0.5f && line.Tap(10.5f) == 0.5f, "delay line: fractional taps %g %g (want 0.5)",
          line.Tap(9.5f), line.Tap(10.5f));
    CHECK(line.Tap(10.f) == 1.f, "delay line: Tap(10.f) = %g", line.Tap(10.f));
}

static void TestPlateReverbClearStep() {
    static bitch::PlateReverb plate;
    plate.Init(kSampleRate);
    plate.SetDecay(1.f);
    float ol, orr;
    uint32_t seed = 3;
    for (int n = 0; n < 48000; n++) {
        seed = seed * 1664525u + 1013904223u;
        plate.Process(((seed >> 8) / 8388608.f - 1.f) * 0.5f, 0.f, ol, orr);
    }
    int steps = 0, most = 0, total = 0;
    bool clean = false;
    while (!clean && steps < 1000) {
        clean = plate.ClearStep(1024);
        steps++;
        most = std::max(most, plate.LastCleared());
        total += plate.LastCleared();
    }
    int want = (bitch::PlateReverb::TotalSamples() + 1023) / 1024;
    CHECK(clean && steps == want, "plate: ClearStep took %d steps (want %d)", steps, want);
    CHECK(most <= 1024, "plate: ClearStep zeroed %d floats in one call (max 1024)", most);
    CHECK(total == bitch::PlateReverb::TotalSamples(), "plate: ClearStep zeroed %d floats of %d", total,
          bitch::PlateReverb::TotalSamples());
    float peak = 0.f;
    for (int n = 0; n < 48000; n++) {
        plate.Process(0.f, 0.f, ol, orr);
        peak = std::max(peak, std::max(fabsf(ol), fabsf(orr)));
    }
    CHECK(peak == 0.f, "plate: %g after ClearStep() finished", peak);
}

// Reverb flipped on and off every 20 ms (MIX hovering at the reverb's threshold), on a tone
static void TestEngineReverbFlicker() {
    static bitch::Engine e;
    static bitch::PlateReverb plate;
    e.Init(kSampleRate, 48, &plate);
    bitch::Params p;
    float in[48], out_l[48], out_r[48];
    bool finite = true;
    int n = 0;
    auto run = [&](float seconds, float level, bool flicker, float reverb) {
        float peak = 0.f;
        for (int b = 0; b < (int)(seconds * kSampleRate / 48); b++) {
            p.reverb = flicker ? ((b / 20) % 2 == 0 ? 1.f : 0.f) : reverb;
            for (int k = 0; k < 48; k++, n++) in[k] = level * sinf(2 * kPi * 440.f * n / kSampleRate);
            e.Process(in, in, out_l, out_r, 48, p);
            for (int k = 0; k < 48; k++) {
                finite = finite && std::isfinite(out_l[k]) && std::isfinite(out_r[k]);
                peak = std::max(peak, std::max(fabsf(out_l[k]), fabsf(out_r[k])));
            }
        }
        return peak;
    };
    run(2.f, 0.3f, true, 0.f);
    run(0.5f, 0.f, false, 0.f);
    float replay = run(0.5f, 0.f, false, 1.f);
    CHECK(finite, "reverb flicker: non-finite output");
    CHECK(replay < 1e-5f, "reverb flicker: the old tail replayed (%g)", replay);
}

// At reverb 0 a plate changes nothing: bit-exact with an engine that has none
static void TestEngineReverbOffBitExact() {
    static bitch::Engine with, without;
    static bitch::PlateReverb plate;
    with.Init(kSampleRate, 48, &plate);
    without.Init(kSampleRate);
    bitch::Params p;
    p.feedback = 0.8f;
    p.reception = 0.5f;
    p.tune_hz = 120.f;
    p.ring = 0.3f;
    uint32_t seed = 9;
    float in_l[48], in_r[48], a_l[48], a_r[48], b_l[48], b_r[48];
    bool same = true;
    int n = 0;
    for (int b = 0; b < (int)(3 * kSampleRate / 48); b++) {
        for (int k = 0; k < 48; k++, n++) {
            seed = seed * 1664525u + 1013904223u;
            float noise = ((seed >> 8) / 8388608.f - 1.f) * 0.1f;
            in_l[k] = 0.3f * sinf(2 * kPi * 330.f * n / kSampleRate) + noise;
            in_r[k] = 0.2f * sinf(2 * kPi * 220.f * n / kSampleRate) - noise;
        }
        with.Process(in_l, in_r, a_l, a_r, 48, p);
        without.Process(in_l, in_r, b_l, b_r, 48, p);
        for (int k = 0; k < 48; k++) same = same && a_l[k] == b_l[k] && a_r[k] == b_r[k];
    }
    CHECK(same, "reverb 0: output differs with a plate");
}

// NaN recovery: the plate (which had sound, and maybe the NaN) gives no stale tail once it restarts
static void TestEngineReverbNanRecovery() {
    static bitch::Engine e;
    static bitch::PlateReverb plate;
    e.Init(kSampleRate, 48, &plate);
    bitch::Params p;
    p.reverb = 1.f;
    float in[48], out_l[48], out_r[48];
    for (int b = 0; b < (int)(0.5f * kSampleRate / 48); b++) {
        for (int k = 0; k < 48; k++) in[k] = 0.3f * sinf(2 * kPi * 440.f * (b * 48 + k) / kSampleRate);
        e.Process(in, in, out_l, out_r, 48, p);
    }
    for (float &v : in) v = NAN;
    e.Process(in, in, out_l, out_r, 48, p);
    for (float &v : in) v = 0.f;
    float peak = 0.f;
    for (int b = 0; b < (int)(0.5f * kSampleRate / 48); b++) {
        e.Process(in, in, out_l, out_r, 48, p);
        for (int k = 0; k < 48; k++) peak = std::max(peak, std::max(fabsf(out_l[k]), fabsf(out_r[k])));
    }
    CHECK(std::isfinite(peak) && peak < 1e-5f, "reverb after a NaN: %g (stale tail)", peak);
}

static void TestEffectsMapping() {
    using namespace bitch;
    CHECK(PadEffect(0, 4) == kEchoes, "page 1 big left pad: %d", PadEffect(0, 4));
    CHECK(PadEffect(0, 6) == kSpiralUp, "page 1 big right pad: %d", PadEffect(0, 6));
    CHECK(PadEffect(0, 1) == kFeedbackBurst, "page 1 top centre: %d", PadEffect(0, 1));
    CHECK(PadEffect(0, 8) == kSpinDown && PadEffect(0, 9) == kSpinUp, "page 1 bottom pads");
    CHECK(PadEffect(1, 0) == kStaticBurst && PadEffect(1, 2) == kSquelch, "page 2 top pads");
    CHECK(PadEffect(1, 8) == kFadeOut && PadEffect(1, 9) == kFilterSlam && PadEffect(1, 1) == kMorseChimes,
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

static void TestPadPages() {
    using namespace bitch;
    PadPages pp;
    uint32_t t = 1000;
    auto step = [&](uint32_t ms, uint32_t touched, int page) {
        t += ms;
        pp.Update(t, touched, page);
    };
    // Every pad is momentary: on from the touch, off at the release, however short or long
    step(10, 1u << 4, 0);
    CHECK(pp.Effects() == Bit(kEchoes), "touch: not on (%08x)", pp.Effects());
    step(100, 0, 0);
    CHECK(pp.Effects() == 0, "tap: still on after release (%08x)", pp.Effects());
    step(10, 1u << 8, 1);
    step(500, 1u << 8, 1);
    CHECK(pp.Effects() == Bit(kFadeOut), "hold: dropped while held (%08x)", pp.Effects());
    step(10, 0, 1);
    CHECK(pp.Effects() == 0, "hold: still on after release (%08x)", pp.Effects());
    // Several pads at once
    step(10, (1u << 0) | (1u << 2), 2);
    CHECK(pp.Effects() == (Bit(kRing100) | Bit(kCarrierX2)), "two pads: %08x", pp.Effects());
    step(10, 1u << 2, 2);
    CHECK(pp.Effects() == Bit(kCarrierX2), "release one of two: %08x", pp.Effects());
    step(10, 0, 2);
    // A held pad keeps the effect of the page it was touched on
    step(10, 1u << 3, 0);
    step(100, 1u << 3, 1);
    CHECK(pp.Effects() == Bit(kReverseEchoes), "held pad changed page (%08x)", pp.Effects());
    step(300, 0, 1);
    CHECK(pp.Effects() == 0, "held pad: still on after release (%08x)", pp.Effects());
    // TOU tap: one stereo step, on the release; a long TOU touch does nothing
    step(10, 1u << 10, 0);
    CHECK(pp.TakeStereoSteps() == 0, "TOU: stepped at the touch");
    step(100, 0, 0);
    CHECK(pp.TakeStereoSteps() == 1, "TOU tap: no stereo step");
    for (int i = 0; i < 60; i++) step(10, 1u << 10, 0);
    step(10, 0, 0);
    CHECK(pp.TakeStereoSteps() == 0, "TOU 0.6 s: stepped the stereo mode");
}

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
    bitch::Bit(bitch::kBitcrush) | bitch::Bit(bitch::kRateCrush) | bitch::Bit(bitch::kDropouts) | bitch::Bit(bitch::kStutter) | bitch::Bit(bitch::kCodecWarble) |
    bitch::Bit(bitch::kScrambler) | bitch::Bit(bitch::kHoppingScrambler) |
    bitch::Bit(bitch::kCarriers) | bitch::Bit(bitch::kSelcall) | bitch::Bit(bitch::kMorseChimes) | bitch::Bit(bitch::kJammer) |
    bitch::Bit(bitch::kFeedbackBurst) | bitch::Bit(bitch::kSpinUp) | bitch::Bit(bitch::kSpinDown) |
    bitch::Bit(bitch::kStaticBurst) | bitch::Bit(bitch::kSquelch) | bitch::Bit(bitch::kFadeOut) |
    bitch::Bit(bitch::kFilterSlam) | bitch::Bit(bitch::kRing100) |
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

// Switching effects on and off doesn't click: in the 35 ms after each toggle (the ~30 ms fade),
// no step between samples is more than 2.5x the largest in the steady 15 ms just before and
// after it. Squelch is toggled too, but the 30 ms after each release are skipped (its opening
// noise burst is meant to be abrupt). Static burst isn't toggled: its crackles jump by design.
// Returns the worst ratio over the toggles.
static float NoClickWorstRatio(uint32_t seed) {
    const bitch::Effect kToggled[7] = {bitch::kFadeOut,   bitch::kKillDry,    bitch::kRing100,
                                       bitch::kCarrierX2, bitch::kFilterSlam, bitch::kFeedbackBurst,
                                       bitch::kSquelch};
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.mix = 0.5f;
    p.carrier_hz = 300.f;
    p.tune_hz = 50.f;
    p.filter_hz = 800.f; // closed at a low Q, so Filter slam has something to change
    p.filter_q = 0.7f;
    p.filter_open = 0.f;
    std::vector<float> l, r;
    std::vector<bool> skip;
    int n = 0;
    const int kToggles = 60;
    const size_t segment = (size_t)(0.05f * kSampleRate), fade = (size_t)(0.035f * kSampleRate);
    for (int i = 0; i < kToggles; i++) {
        seed = seed * 1664525u + 1013904223u;
        bitch::Effect fx = kToggled[(seed >> 16) % (sizeof(kToggled) / sizeof(kToggled[0]))];
        bool release = fx == bitch::kSquelch && bitch::Has(p.effects, fx);
        p.effects ^= bitch::Bit(fx);
        size_t from = l.size();
        RunFor(e, p, 0.05f, [](int k) { return Sine(200.f, 0.3f, k); }, n, l, r);
        skip.resize(l.size(), false);
        if (release)
            for (size_t k = from; k < std::min(l.size(), from + (size_t)(0.03f * kSampleRate)); k++) skip[k] = true;
    }
    auto max_step = [&](size_t from, size_t to) {
        float step = 0.f;
        for (size_t k = std::max<size_t>(from, 1); k < std::min(to, l.size()); k++)
            if (!skip[k]) step = std::max(step, fabsf(l[k] - l[k - 1]));
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

static void TestEffectsNoClick() {
    float a = NoClickWorstRatio(17u), b = NoClickWorstRatio(4242u);
    CHECK(a < 2.5f && b < 2.5f, "effects on/off: a step %.2fx / %.2fx the steady ones around it", a, b);
}

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
    // At mix 0.5 the dry signal fades as well
    bitch::Params half;
    half.mix = 0.5f;
    bitch::Engine ref_half_engine, half_engine;
    ref_half_engine.Init(kSampleRate);
    half_engine.Init(kSampleRate);
    ref_l.clear();
    ref_r.clear();
    n = 0;
    RunFor(ref_half_engine, half, 1.f, tone, n, ref_l, ref_r);
    double ref_half = Magnitude(Window(ref_l, 0.5f, 1.f), 1000.f);
    l.clear();
    r.clear();
    n = 0;
    half.effects = bitch::Bit(bitch::kFadeOut);
    RunFor(half_engine, half, 2.f, tone, n, l, r);
    double faded_half = Db(Magnitude(Window(l, 1.8f, 2.f), 1000.f) / ref_half);
    CHECK(faded_half < -35.0, "fade-out at mix 0.5: only %.1f dB down after 1.8 s", faded_half);
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
    // (at 3.5 kHz: not a harmonic of the tone, which the soft clip adds)
    double tail = Magnitude(Window(l, 0.5f, 0.6f), 3500.f), after = Magnitude(Window(l, 1.5f, 1.6f), 3500.f);
    CHECK(tail > 10 * after, "squelch: no noise burst as it opens (%.5f vs %.5f)", tail, after);
    CHECK(Rms(Window(l, 1.5f, 2.f)) > 0.15f, "squelch: the tone didn't come back (%g)", Rms(Window(l, 1.5f, 2.f)));
}

// Like ShiftTrace, with 8-sample blocks (the engine Init'ed with block size 8): the left side's
// shifter frequency per 8-sample block, measurable to +-3000 Hz (ShiftTrace's 48: +-500 Hz)
template <class Signal>
static std::vector<float> ShiftTraceFine(bitch::Engine &e, const bitch::Params &p, float seconds, Signal signal) {
    float in[8], out_l[8], out_r[8];
    std::vector<float> trace;
    float prev = e.ShifterPhase(0);
    int blocks = (int)(seconds * kSampleRate / 8);
    for (int b = 0; b < blocks; b++) {
        for (int k = 0; k < 8; k++) in[k] = signal(b * 8 + k);
        e.Process(in, in, out_l, out_r, 8, p);
        float phase = e.ShifterPhase(0), d = phase - prev;
        d -= floorf(d + 0.5f);
        trace.push_back(d * kSampleRate / 8.f);
        prev = phase;
    }
    return trace;
}

static void TestEffectSpins() {
    auto silence = [](int) { return 0.f; };
    bitch::Params p;
    p.effect_amount = 1.f;
    p.effects = bitch::Bit(bitch::kSpinUp);
    bitch::Engine e;
    e.Init(kSampleRate, 8);
    auto up = ShiftTraceFine(e, p, 1.2f, silence);
    CHECK(fabsf(up[3000] - 1500.f) < 150.f, "spin up: %.0f Hz after 0.5 s (want ~1500)", up[3000]);
    CHECK(fabsf(up.back() - 2000.f) < 20.f, "spin up: %.0f Hz after 1.2 s (want the 2000 Hz limit)", up.back());
    p.effects = 0u;
    auto back = ShiftTraceFine(e, p, 5.f, silence);
    CHECK(fabsf(back.back()) < 50.f, "spin up: %.0f Hz, 5 s after release", back.back());
    bitch::Engine d;
    d.Init(kSampleRate, 8);
    p.effects = bitch::Bit(bitch::kSpinDown);
    auto down = ShiftTraceFine(d, p, 0.6f, silence);
    CHECK(fabsf(down[3000] + 1500.f) < 150.f, "spin down: %.0f Hz after 0.5 s (want ~-1500)", down[3000]);
    bitch::Engine slow;
    slow.Init(kSampleRate, 8);
    p.effect_amount = 0.35f;
    p.effects = bitch::Bit(bitch::kSpinUp);
    auto gentle = ShiftTraceFine(slow, p, 1.f, silence);
    CHECK(fabsf(gentle.back() - 150.f) < 20.f, "spin up at amount 0.35: %.0f Hz after 1 s (want ~150)",
          gentle.back());
}

// Hold drift in Linked mode: once reception has drifted the sides apart, holding the drift
// keeps them apart even with reception turned back to 0 (the phase lock stays off)
static void TestEffectDriftLinked() {
    auto silence = [](int) { return 0.f; };
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.reception = 0.5f;
    std::vector<float> left, right;
    for (int i = 0; i < 100 && (left.empty() || fabsf(right.back() - left.back()) < 3.f); i++)
        ShiftTraces(e, p, 0.1f, silence, left, right);
    float start = right.back() - left.back();
    CHECK(fabsf(start) >= 3.f, "reception 0.5: the sides never 3 Hz apart (%.2f)", start);
    p.effects = bitch::Bit(bitch::kHoldDrift);
    p.reception = 0.f;
    left.clear();
    right.clear();
    ShiftTraces(e, p, 3.f, silence, left, right);
    float moved = 0.f;
    for (size_t k = 0; k < right.size(); k++) moved = std::max(moved, fabsf(right[k] - left[k] - start));
    CHECK(moved < 0.1f, "hold drift, Linked, reception down: the sides' difference moved %.3f Hz (from %.2f)", moved,
          start);
}

// Every effect at once (but Squelch and Fade-out) at the extremes, the spins switching every
// second: the output stays finite and within +-1
static void TestEffectsStable() {
    static bitch::Engine e;
    static bitch::PlateReverb plate;
    e.Init(kSampleRate, 48, &plate);
    bitch::Params p;
    p.feedback = 0.95f;
    p.reception = 1.f;
    p.ring = 1.f;
    p.carrier_hz = 5000.f;
    p.stereo_mode = bitch::StereoMode::kRotate;
    p.reverb = 1.f;
    p.effect_amount = 1.f;
    const uint32_t spins = bitch::Bit(bitch::kSpinUp) | bitch::Bit(bitch::kSpinDown);
    const uint32_t base = ((1u << bitch::kEffectCount) - 1) &
                          ~(bitch::Bit(bitch::kSquelch) | bitch::Bit(bitch::kFadeOut) | spins);
    const uint32_t kSpins[4] = {bitch::Bit(bitch::kSpinUp), 0u, bitch::Bit(bitch::kSpinDown), spins};
    bitch::Random noise_l(5u), noise_r(9u);
    float in_l[48], in_r[48], out_l[48], out_r[48];
    bool finite = true;
    float peak = 0.f;
    const int blocks_per_s = (int)(kSampleRate / 48);
    for (int b = 0; b < 30 * blocks_per_s; b++) {
        p.effects = base | kSpins[(b / blocks_per_s) % 4];
        for (int k = 0; k < 48; k++) {
            in_l[k] = 0.9f * noise_l.Bipolar();
            in_r[k] = 0.9f * noise_r.Bipolar();
        }
        e.Process(in_l, in_r, out_l, out_r, 48, p);
        for (int k = 0; k < 48; k++) {
            finite = finite && std::isfinite(out_l[k]) && std::isfinite(out_r[k]);
            peak = std::max(peak, std::max(fabsf(out_l[k]), fabsf(out_r[k])));
        }
    }
    CHECK(finite && peak <= 1.f, "every effect at once: finite %d, peak %g", (int)finite, peak);
}

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

// Both scrambler pads held: it still hops (Hopping adds the hops to the plain scrambler)
static void TestEffectScramblerBothPads() {
    auto run = [](uint32_t effects) {
        bitch::Engine e;
        e.Init(kSampleRate);
        bitch::Params p;
        p.effect_amount = 1.f;
        p.effects = effects;
        std::vector<float> l, r;
        int n = 0;
        RunFor(e, p, 2.f, [](int k) { return Sine(500.f, 0.25f, k); }, n, l, r);
        return Magnitude(Window(l, 1.f, 2.f), 700.f); // the plain inversion at 1.2 kHz puts 500 Hz at 700 Hz
    };
    double plain = run(bitch::Bit(bitch::kScrambler));
    double both = run(bitch::Bit(bitch::kScrambler) | bitch::Bit(bitch::kHoppingScrambler));
    CHECK(both < 0.5 * plain, "both scrambler pads: no hops (700 Hz %.4f vs plain %.4f)", both, plain);
}

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

int main() {
    TestSoftQuantize();
    TestSoftClip();
    TestHilbertQuadrature();
    TestShiftUpAndDown();
    TestThroughZero();
    TestFeedbackStable();
    TestEngineShiftAndMix();
    TestEngineLimiter();
    TestEngineRecoversFromNaN();
    TestEngineNonFiniteMidBlock();
    TestEngineNaNTune();
    TestEngineMonoInput();
    TestEngineStereoKeepsSides();
    TestEngineRingBeforeShift();
    TestEngineFilter();
    TestEngineDefaultsTransparent();
    TestEngineMirror();
    TestEngineRotateSwapsSides();
    TestEngineRotateShifts();
    TestEngineModeSwitchNoClick();
    TestEngineRotateSweep();
    TestEngineLinkedAfterMirror();
    TestEngineMonoTakeoverTiming();
    TestEngineIntermittentSideNoFlap();
    TestRingMod();
    TestRadioFilter();
    TestRadioFilterSweepStable();
    TestMappings();
    TestBlips();
    TestAgc();
    TestStaticNoise();
    TestFading();
    TestDrive();
    TestWander();
    TestEngineReceptionSeparatesSides();
    TestEngineReceptionStatic();
    TestEngineReceptionLiftsQuiet();
    TestEngineReceptionStable();
    TestDriveOnset();
    TestRandomSeedsScrambled();
    TestWanderRateChange();
    TestFadingReturnsToExact();
    TestEngineLinkedDrifts();
    TestEngineReceptionReturnsClean();
    TestPitchTracker();
    TestPitchTrackerRangeEdges();
    TestPitchTrackerGradedConfidence();
    TestEngineDialInertia();
    TestEngineDetents();
    TestEngineDialInertiaFromStart();
    TestEngineDetentsRelease();
    TestEngineDetentsHoldPitch();
    TestEngineMirrorDetents();
    TestEngineTuningHand();
    TestMixAndReverb();
    TestPlateReverb();
    TestPlateReverbClear();
    TestEngineReverb();
    TestEngineReverbStable();
    TestDelayLine();
    TestPlateReverbClearStep();
    TestEngineReverbFlicker();
    TestEngineReverbOffBitExact();
    TestEngineReverbNanRecovery();
    TestEffectsMapping();
    TestPadPages();
    TestLowBlip();
    TestEffectsUnbuiltDoNothing();
    TestEffectFeedbackBurst();
    TestEffectRingAndCarrier();
    TestEffectKillDry();
    TestEffectFilterSlam();
    TestEffectsNoClick();
    TestEffectStaticBurst();
    TestEffectHoldDrift();
    TestEffectFadeOut();
    TestEffectSquelch();
    TestEffectSpins();
    TestEffectDriftLinked();
    TestEffectsStable();
    TestScrambler();
    TestEffectScrambler();
    TestEffectScramblerBothPads();
    TestCarriers();
    TestMorseChimes();
    TestSelcall();
    TestJammer();
    TestEffectsRadioSounds();
    TestEffectSelcallMistuned();
    TestEffectCarriersFollowDial();
    TestRadioZeroBeat();
    TestEffectsRadioNoClick();
    TestCrush();
    TestDropouts();
    TestStutter();
    TestCodecWarble();
    TestEffectsDamage();
    TestEffectsDamageNoClick();
    if (failures == 0) printf("All tests passed\n");
    return failures == 0 ? 0 : 1;
}
