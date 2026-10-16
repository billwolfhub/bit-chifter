#pragma once

#include <cmath>

#include "effects.h"

namespace bitch {

// Bitcrush: the signal quantised to `bits` bits over -1..1 (no dither): 10 bits (amount 0.35)
// down to 3 bits (amount 1). A mid-tread quantiser (0 is a level), so `bits` bits give
// 2^bits + 1 levels over -1..1 (3 bits: 9, steps of 0.25). CrushStep() and its inverse once
// per block, Bitcrush() per sample.
inline float CrushBits(float amount) { return 10.f - 7.f * EffectRange(amount); }
inline float CrushStep(float bits) { return 2.f / exp2f(bits); }
inline float Bitcrush(float x, float step, float inv_step) { return step * floorf(x * inv_step + 0.5f); }

// Sample-rate crush: sample-and-hold at 12 kHz (amount 0.35) down to 1.5 kHz (amount 1), with
// no anti-aliasing (the aliasing is the sound). Update() once per block, Process() per sample.
class RateCrush {
  public:
    void Init(float sample_rate) {
        sample_rate_ = sample_rate;
        Update(0.35f);
        Reset();
    }
    void Reset() {
        phase_ = 1.f; // take the first sample at once
        held_ = 0.f;
    }
    void Update(float amount) { step_ = 12000.f * powf(1500.f / 12000.f, EffectRange(amount)) / sample_rate_; }
    float Process(float x) {
        phase_ += step_;
        if (phase_ >= 1.f) {
            phase_ -= floorf(phase_);
            held_ = x;
        }
        return held_;
    }
    float HoldRateHz() const { return step_ * sample_rate_; }

  private:
    float sample_rate_ = 48000.f, step_ = 0.25f, phase_ = 1.f, held_ = 0.f;
};

} // namespace bitch
