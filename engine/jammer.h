#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "effects.h"
#include "random.h"
#include "softclip.h"

namespace bitch {

// A cheap sine of a phase 0-1 (a parabola, refined: within 0.1% of sinf)
inline float FastSine(float phase) {
    float t = phase < 0.5f ? phase : phase - 1.f; // -0.5..0.5
    float y = 8.f * t * (1.f - 2.f * fabsf(t));
    return y + 0.225f * (y * fabsf(y) - y);
}

// Jammer: a buzzy tone (an overdriven sine) swept from 300 Hz up to 3 kHz in a sawtooth
// (exponential sweep), 0.5 (amount 0.35) to 4 (amount 1) sweeps a second, plus bursts of
// noise (30-150 ms, 1 to 6 a second, with 1.5 ms edges). Level -24 dBFS (amount 0.35) to
// -10 dBFS (amount 1). Update() once per block (with the pad's fade level; the gain is
// ramped across the next block), Process() per sample (mono).
class Jammer {
  public:
    void Init(float sample_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        random_.Seed(seed);
        edge_ = (int)(0.0015f * sample_rate);
        sweep_ = phase_ = 0.f;
        tone_hz_ = 300.f;
        Reset();
    }

    // Silent: the gain at 0, no burst
    void Reset() {
        gain_ = step_ = target_ = 0.f;
        ramp_ = 0;
        burst_left_ = 0;
    }

    void Update(float amount, float level, float block_seconds) {
        float u = EffectRange(amount);
        sweep_step_ = (0.5f + 3.5f * u) / sample_rate_;
        ratio_ = powf(10.f, sweep_step_); // the tone's rise per sample (a decade per sweep)
        target_ = powf(10.f, 0.05f * (-24.f + 14.f * u)) * level;
        ramp_ = std::max((int)lrintf(block_seconds * sample_rate_), 1);
        step_ = (target_ - gain_) / (float)ramp_;
        if (burst_left_ <= 0 && random_.Uniform() < (1.f + 5.f * u) * block_seconds) {
            burst_length_ = (int)((0.03f + 0.12f * random_.Uniform()) * sample_rate_);
            burst_left_ = burst_length_;
        }
    }

    float Process() {
        if (ramp_ > 0) gain_ = --ramp_ > 0 ? gain_ + step_ : target_;
        sweep_ += sweep_step_;
        if (sweep_ >= 1.f) { // back to 300 Hz
            sweep_ -= 1.f;
            tone_hz_ = 300.f * powf(10.f, sweep_);
        } else {
            tone_hz_ = fminf(tone_hz_ * ratio_, 3000.f);
        }
        phase_ += tone_hz_ / sample_rate_;
        phase_ -= floorf(phase_);
        float out = SoftClip(3.f * FastSine(phase_));
        if (burst_left_ > 0) {
            int from_start = burst_length_ - burst_left_;
            float edge = fminf((float)std::min(from_start, burst_left_) / (float)edge_, 1.f);
            burst_left_--;
            out += edge * random_.Bipolar();
        }
        return gain_ * out;
    }

    float ToneHz() const { return tone_hz_; }

  private:
    float sample_rate_ = 48000.f, sweep_ = 0.f, sweep_step_ = 0.f, ratio_ = 1.f, phase_ = 0.f, tone_hz_ = 300.f;
    float gain_ = 0.f, step_ = 0.f, target_ = 0.f;
    int ramp_ = 0, burst_left_ = 0, burst_length_ = 1, edge_ = 1;
    Random random_;
};

} // namespace bitch
