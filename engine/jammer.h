#pragma once

#include <cmath>
#include <cstdint>

#include "effects.h"
#include "random.h"
#include "softclip.h"

namespace bitch {

// Jammer: a buzzy tone (an overdriven sine) swept from 300 Hz up to 3 kHz in a sawtooth
// (exponential sweep), 0.5 (amount 0.35) to 4 (amount 1) sweeps a second, plus bursts of
// noise (30-150 ms, 1 to 6 a second). Level -24 dBFS (amount 0.35) to -10 dBFS (amount 1).
// Update() once per block (with the pad's fade level), Process() per sample (mono).
class Jammer {
  public:
    void Init(float sample_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        random_.Seed(seed);
        sweep_ = phase_ = 0.f;
        burst_left_ = 0;
        tone_hz_ = 300.f;
    }

    void Update(float amount, float level, float block_seconds) {
        float u = EffectRange(amount);
        sweep_step_ = (0.5f + 3.5f * u) / sample_rate_;
        gain_ = powf(10.f, 0.05f * (-24.f + 14.f * u)) * level;
        if (burst_left_ <= 0 && random_.Uniform() < (1.f + 5.f * u) * block_seconds)
            burst_left_ = (int)((0.03f + 0.12f * random_.Uniform()) * sample_rate_);
    }

    float Process() {
        sweep_ += sweep_step_;
        sweep_ -= floorf(sweep_);
        tone_hz_ = 300.f * powf(10.f, sweep_);
        phase_ += tone_hz_ / sample_rate_;
        phase_ -= floorf(phase_);
        float out = SoftClip(3.f * sinf(6.28318530718f * phase_));
        if (burst_left_ > 0) {
            burst_left_--;
            out += random_.Bipolar();
        }
        return gain_ * out;
    }

    float ToneHz() const { return tone_hz_; }

  private:
    float sample_rate_ = 48000.f, sweep_ = 0.f, sweep_step_ = 0.f, phase_ = 0.f, gain_ = 0.f, tone_hz_ = 300.f;
    int burst_left_ = 0;
    Random random_;
};

} // namespace bitch
