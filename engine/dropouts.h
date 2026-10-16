#pragma once

#include <cmath>
#include <cstdint>

#include "effects.h"
#include "random.h"

namespace bitch {

// Dropouts: the signal cuts out, like a digital stream losing packets: random gaps of
// 20-200 ms with 1 ms edges, starting 0.3 (amount 0.35) to 4 (amount 1) times a second on
// average. Gain() per sample (1 = through, 0 = gone; mono: both sides drop together),
// Update() once per block.
class Dropouts {
  public:
    void Init(float sample_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        edge_ = 1.f / (0.001f * sample_rate);
        random_.Seed(seed);
        Update(0.35f);
        Reset();
    }
    void Reset() {
        gain_ = 1.f;
        gap_left_ = 0;
    }
    void Update(float amount) { chance_ = (0.3f + 3.7f * EffectRange(amount)) / sample_rate_; }
    float Gain() {
        if (gap_left_ > 0) {
            gap_left_--;
            gain_ = fmaxf(gain_ - edge_, 0.f);
        } else {
            if (random_.Uniform() < chance_) gap_left_ = (int)((0.02f + 0.18f * random_.Uniform()) * sample_rate_);
            gain_ = fminf(gain_ + edge_, 1.f);
        }
        return gain_;
    }

  private:
    float sample_rate_ = 48000.f, edge_ = 0.f, chance_ = 0.f, gain_ = 1.f;
    int gap_left_ = 0;
    Random random_;
};

} // namespace bitch
