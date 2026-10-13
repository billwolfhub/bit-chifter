#include "engine.h"

#include <cmath>

#include "softclip.h"

namespace bitch {

// Out-of-class definitions: the firmware builds as C++14, where odr-used static constexpr
// members need one (redundant but harmless in C++17)
constexpr float Hilbert::kA[4];
constexpr float Hilbert::kB[4];

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
    if (size == 0) return;
    float tune = fminf(fmaxf(params.tune_hz, -2500.f), 2500.f);
    float feedback = fminf(fmaxf(params.feedback, 0.f), 0.95f);
    float mix = fminf(fmaxf(params.mix, 0.f), 1.f);
    for (size_t i = 0; i < size; i++) {
        shift_hz_ += glide_ * (tune - shift_hz_);
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
