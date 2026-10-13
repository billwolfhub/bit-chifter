#include "engine.h"

#include <cmath>

#include "agc.h"
#include "fading.h"
#include "softclip.h"

namespace bitch {

// Out-of-class definitions: the firmware builds as C++14, where odr-used static constexpr
// members need one (redundant but harmless in C++17)
constexpr float Hilbert::kA[4];
constexpr float Hilbert::kB[4];
constexpr int Fading::kSize;
constexpr int Fading::kMask;
constexpr float Agc::kTarget;

static const float kSilent = 0.001f;  // -60 dBFS: above this an input counts as used at once
static const float kQuiet = 0.0005f;  // -66 dBFS: an input must stay below this for 3 s to count as unused

static float Clamp(float x, float lo, float hi) { return fminf(fmaxf(x, lo), hi); }

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

void Engine::Reset() {
    for (RingMod &r : ring_) r.Reset();
    for (Shifter &s : shifter_) s.Reset();
    for (RadioFilter &f : filter_) f.Reset();
    shift_hz_ = 0.f;
    carrier_hz_ = 100.f;
    log2_filter_hz_ = log2f(1000.f);
    filter_q_ = 0.7f;
    filter_open_ = 1.f;
    block_glide_ = 0.f; // set on the first block (depends on the block size)
    rotate_phase_ = 0.f;
    side_gain_[0] = side_gain_[1] = 1.f;
    swap_ = 0.f;
    ring_amount_ = 0.f;
    mix_ = 1.f;
    quiet_blocks_[0] = quiet_blocks_[1] = 0;
    active_[0] = active_[1] = false;
    envelope_[0] = envelope_[1] = 0.f;
    take_other_[0] = take_other_[1] = 0.f;
    reception_ = 0.f;
    for (Fading &f : fading_) f.Reset();
    for (Agc &a : agc_) a.Reset();
}

void Engine::Process(const float *in_l, const float *in_r, float *out_l, float *out_r, size_t size,
                     const Params &params) {
    if (size == 0) return;
    float tune = Clamp(params.tune_hz, -2500.f, 2500.f);
    float feedback = Clamp(params.feedback, 0.f, 0.95f);
    float mix = Clamp(params.mix, 0.f, 1.f);
    float ring = Clamp(params.ring, 0.f, 1.f);
    float carrier = Clamp(params.carrier_hz, 0.1f, 5000.f);

    // Filter settings change once per block, smoothed (the centre in octaves)
    if (block_glide_ == 0.f) {
        block_glide_ = 1.f - expf(-(float)size / (0.03f * sample_rate_));
        log2_filter_hz_ = log2f(Clamp(params.filter_hz, 150.f, 5000.f));
        filter_q_ = Clamp(params.filter_q, 0.7f, 12.f);
        filter_open_ = Clamp(params.filter_open, 0.f, 1.f);
        reception_ = Clamp(params.reception, 0.f, 1.f);
    }
    log2_filter_hz_ += block_glide_ * (log2f(Clamp(params.filter_hz, 150.f, 5000.f)) - log2_filter_hz_);
    filter_q_ += block_glide_ * (Clamp(params.filter_q, 0.7f, 12.f) - filter_q_);
    filter_open_ += block_glide_ * (Clamp(params.filter_open, 0.f, 1.f) - filter_open_);
    float filter_hz = exp2f(log2_filter_hz_);
    for (RadioFilter &f : filter_) f.Set(filter_hz, filter_q_, filter_open_);

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

    // Mono detection: a side whose input is silent takes the other side's input instead
    float take_target[2] = {active_[1] && !active_[0] ? 1.f : 0.f, active_[0] && !active_[1] ? 1.f : 0.f};

    // Stereo mode: per-block targets for each side's share of the shift and for the left/right
    // swap (glided per sample below, so mode changes don't click)
    float gain_target[2] = {1.f, 1.f}, swap_target = 0.f;
    if (params.stereo_mode == StereoMode::kMirror) {
        gain_target[1] = -1.f;
    } else if (params.stereo_mode == StereoMode::kRotate) {
        rotate_phase_ += rotate_step_ * (float)size;
        rotate_phase_ -= floorf(rotate_phase_);
        float angle = 6.28318530718f * rotate_phase_;
        float sweep = cosf(angle); // left's shift goes +shift .. -shift, right's the other way
        gain_target[0] = sweep;
        gain_target[1] = -sweep;
        swap_target = 0.5f + 0.5f * sinf(angle); // the pair circles from left to right
    } else {
        // Linked: keep the two oscillators in phase (they are left at an offset after Mirror/Rotate).
        // Pulling the right one gently (<1 Hz deviation) converges in ~100 ms and is inaudible.
        // Scaled down by reception so the sides can wander.
        float err = shifter_[0].Phase() - shifter_[1].Phase();
        err -= floorf(err + 0.5f); // wrapped to -0.5..0.5
        shifter_[1].NudgePhase(0.02f * (1.f - r) * err);
    }

    for (size_t i = 0; i < size; i++) {
        float x[2] = {in_l[i], in_r[i]};
        for (int c = 0; c < 2; c++) envelope_[c] = fmaxf(fabsf(x[c]), envelope_[c] * envelope_release_);
        shift_hz_ += glide_ * (tune - shift_hz_);
        carrier_hz_ += glide_ * (carrier - carrier_hz_);
        ring_amount_ += glide_ * (ring - ring_amount_);
        mix_ += glide_ * (mix - mix_);
        swap_ += glide_ * (swap_target - swap_);
        float shift[2];
        for (int c = 0; c < 2; c++) {
            side_gain_[c] += glide_ * (gain_target[c] - side_gain_[c]);
            shift[c] = shift_hz_ * side_gain_[c];
        }
        float y[2];
        for (int c = 0; c < 2; c++) {
            take_other_[c] += take_glide_ * (take_target[c] - take_other_[c]);
            float dry = x[c] + take_other_[c] * x[1 - c];
            float driven = drive_.Process(dry);
            float ringed = ring_[c].Process(driven, carrier_hz_, ring_amount_);
            float shifted = shifter_[c].Process(ringed, shift[c] + drift_hz[c], feedback);
            float received = fading_[c].Process(shifted) + static_[c].Process();
            float wet = agc_[c].Process(filter_[c].Process(received));
            y[c] = dry + mix_ * (wet - dry);
        }
        out_l[i] = SoftClip(y[0] + swap_ * (y[1] - y[0]));
        out_r[i] = SoftClip(y[1] + swap_ * (y[0] - y[1]));
    }
    // Hold: a side counts as active at once when its envelope passes kSilent, and as unused
    // only after it has stayed below kQuiet for 3 s
    const int hold_blocks = (int)(3.f * sample_rate_ / (float)size);
    for (int c = 0; c < 2; c++) {
        if (envelope_[c] > kSilent) active_[c] = true;
        if (envelope_[c] > kQuiet) {
            quiet_blocks_[c] = 0;
        } else if (++quiet_blocks_[c] >= hold_blocks) {
            active_[c] = false;
            quiet_blocks_[c] = hold_blocks;
        }
    }
    // A NaN/Inf would stay in the feedback loop and silence everything: reset instead
    if (!std::isfinite(out_l[size - 1]) || !std::isfinite(out_r[size - 1])) {
        Reset();
        for (size_t i = 0; i < size; i++) out_l[i] = out_r[i] = 0.f;
    }
}

} // namespace bitch
