// Mac tests for the Bitchshifter engine. Build and run with sim/run_tests.sh.
#include <cmath>
#include <cstdio>
#include <vector>

#include "engine.h"
#include "hilbert.h"
#include "softclip.h"
#include "shifter.h"
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
}

static void TestEngineMonoSum() {
    bitch::Engine e;
    e.Init(kSampleRate);
    bitch::Params p;
    p.tune_hz = 100.f;
    p.mix = 1.f;
    std::vector<float> l, r;
    int n = (int)kSampleRate;
    float in[48], zeros[48] = {}, out_l[48], out_r[48];
    for (int t = 0; t < n; t += 48) {
        for (int k = 0; k < 48; k++) in[k] = 0.25f * sinf(2 * kPi * 1000.f * (t + k) / kSampleRate);
        e.Process(in, zeros, out_l, out_r, 48, p);
        if (t >= n / 2) {
            l.insert(l.end(), out_l, out_l + 48);
            r.insert(r.end(), out_r, out_r + 48);
        }
    }
    CHECK(fabs(Magnitude(l, 1100.f) - 0.125) < 0.02, "mono sum: OUT L 1100 Hz level %.4f", Magnitude(l, 1100.f));
    CHECK(fabs(Magnitude(r, 1100.f) - 0.125) < 0.02, "mono sum: OUT R 1100 Hz level %.4f", Magnitude(r, 1100.f));
}

static void TestSoftClip() {
    bool bounded = true;
    for (float x = -4.f; x <= 4.f; x += 0.0001f) bounded = bounded && fabsf(bitch::SoftClip(x)) <= 1.f;
    CHECK(bounded, "softclip: output exceeds 1");
    CHECK(fabsf(bitch::SoftClip(0.1f) - 0.1f) < 0.001f, "softclip: slope near 0");
    CHECK(std::isnan(bitch::SoftClip(NAN)), "softclip: NaN must pass through");
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
    TestEngineMonoSum();
    if (failures == 0) printf("All tests passed\n");
    return failures == 0 ? 0 : 1;
}
