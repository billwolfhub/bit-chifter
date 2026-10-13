#pragma once

#include <cmath>
#include <cstdint>

#include "random.h"

namespace bitch {

// Receiver static: hiss (white noise, gently low-passed at 6 kHz) plus random crackles
// (sparse pops of random size and sign through a 2 ms decay, 2-30 per second as the
// level rises). level 0 is exactly silent. Update() once per block, Process() per sample.
class StaticNoise {
  public:
    void Init(float sample_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        random_.Seed(seed);
        hiss_coefficient_ = 1.f - expf(-6.28318530718f * 6000.f / sample_rate);
        crackle_decay_ = expf(-1.f / (0.002f * sample_rate));
        hiss_state_ = crackle_ = 0.f;
        Update(0.f);
    }

    // Clears the filter and crackle state (the RNG keeps running)
    void Reset() { hiss_state_ = crackle_ = 0.f; }

    void Update(float level) {
        float l = fminf(fmaxf(level, 0.f), 1.f);
        hiss_level_ = 0.12f * l;
        crackle_chance_ = l > 0.f ? (2.f + 28.f * l) / sample_rate_ : 0.f;
        crackle_level_ = 0.4f * l;
    }

    float Process() {
        if (hiss_level_ == 0.f && crackle_ == 0.f) return 0.f;
        hiss_state_ += hiss_coefficient_ * (random_.Bipolar() - hiss_state_);
        if (random_.Uniform() < crackle_chance_) {
            float size = crackle_level_ * (0.3f + 0.7f * random_.Uniform());
            crackle_ = random_.Uniform() < 0.5f ? -size : size;
        }
        float out = hiss_level_ * hiss_state_ + crackle_;
        crackle_ *= crackle_decay_;
        if (fabsf(crackle_) < 1e-6f) crackle_ = 0.f;
        return out;
    }

  private:
    Random random_;
    float sample_rate_ = 48000.f;
    float hiss_coefficient_ = 0.f, hiss_state_ = 0.f, hiss_level_ = 0.f;
    float crackle_chance_ = 0.f, crackle_level_ = 0.f, crackle_decay_ = 0.f, crackle_ = 0.f;
};

} // namespace bitch
