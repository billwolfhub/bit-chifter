// Mac tests for the Bitchshifter engine. Build and run with sim/run_tests.sh.
#include <cmath>
#include <cstdio>
#include <vector>

#include "blips.h"
#include "engine.h"
#include "hilbert.h"
#include "softclip.h"
#include "shifter.h"
#include "mapping.h"
#include "radio_filter.h"
#include "ring_mod.h"
#include "random.h"
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
    bitch::FilterWidth(0.98f, q, open);
    CHECK(open == 1.f, "width 0.98: open %.2f", open);
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

int main() {
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
    TestWander();
    if (failures == 0) printf("All tests passed\n");
    return failures == 0 ? 0 : 1;
}
