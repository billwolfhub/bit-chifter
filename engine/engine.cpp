#include "engine.h"

#include <cmath>

#include "softclip.h"

namespace bitch {

// Out-of-class definitions: the firmware builds as C++14, where odr-used static constexpr
// members need one (redundant but harmless in C++17)
constexpr float Hilbert::kA[4];
constexpr float Hilbert::kB[4];

static const float kSilent = 0.001f; // -60 dBFS: below this an input counts as unused

static float Clamp(float x, float lo, float hi) { return fminf(fmaxf(x, lo), hi); }

void Engine::Init(float sample_rate) {
    sample_rate_ = sample_rate;
    for (RingMod &r : ring_) r.Init(sample_rate);
    for (Shifter &s : shifter_) s.Init(sample_rate);
    for (RadioFilter &f : filter_) f.Init(sample_rate);
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
    envelope_[0] = envelope_[1] = 0.f;
    take_other_[0] = take_other_[1] = 0.f;
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
    }
    log2_filter_hz_ += block_glide_ * (log2f(Clamp(params.filter_hz, 150.f, 5000.f)) - log2_filter_hz_);
    filter_q_ += block_glide_ * (Clamp(params.filter_q, 0.7f, 12.f) - filter_q_);
    filter_open_ += block_glide_ * (Clamp(params.filter_open, 0.f, 1.f) - filter_open_);
    float filter_hz = exp2f(log2_filter_hz_);
    for (RadioFilter &f : filter_) f.Set(filter_hz, filter_q_, filter_open_);

    // Mono detection: a side whose input is silent takes the other side's input instead
    bool active_l = envelope_[0] > kSilent, active_r = envelope_[1] > kSilent;
    float take_target[2] = {active_r && !active_l ? 1.f : 0.f, active_l && !active_r ? 1.f : 0.f};

    for (size_t i = 0; i < size; i++) {
        float x[2] = {in_l[i], in_r[i]};
        for (int c = 0; c < 2; c++) envelope_[c] = fmaxf(fabsf(x[c]), envelope_[c] * envelope_release_);
        shift_hz_ += glide_ * (tune - shift_hz_);
        carrier_hz_ += glide_ * (carrier - carrier_hz_);
        float shift[2] = {shift_hz_, shift_hz_};
        float swap = 0.f; // 0 = sides as they are, 1 = fully swapped
        if (params.stereo_mode == StereoMode::kMirror) {
            shift[1] = -shift_hz_;
        } else if (params.stereo_mode == StereoMode::kRotate) {
            rotate_phase_ += rotate_step_;
            if (rotate_phase_ >= 1.f) rotate_phase_ -= 1.f;
            float angle = 6.28318530718f * rotate_phase_;
            float sweep = cosf(angle); // left's shift goes +shift .. -shift, right's the other way
            shift[0] = shift_hz_ * sweep;
            shift[1] = -shift_hz_ * sweep;
            swap = 0.5f + 0.5f * sinf(angle); // the pair circles from left to right
        }
        float y[2];
        for (int c = 0; c < 2; c++) {
            take_other_[c] += take_glide_ * (take_target[c] - take_other_[c]);
            float dry = x[c] + take_other_[c] * x[1 - c];
            float ringed = ring_[c].Process(dry, carrier_hz_, ring);
            float wet = filter_[c].Process(shifter_[c].Process(ringed, shift[c], feedback));
            y[c] = dry + mix * (wet - dry);
        }
        out_l[i] = SoftClip(y[0] + swap * (y[1] - y[0]));
        out_r[i] = SoftClip(y[1] + swap * (y[0] - y[1]));
    }
    // A NaN/Inf would stay in the feedback loop and silence everything: reset instead
    if (!std::isfinite(out_l[size - 1]) || !std::isfinite(out_r[size - 1])) {
        Reset();
        for (size_t i = 0; i < size; i++) out_l[i] = out_r[i] = 0.f;
    }
}

} // namespace bitch
