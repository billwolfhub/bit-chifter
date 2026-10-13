#pragma once

#include <cmath>
#include <cstdint>

namespace bitch {

// Small, fast, deterministic pseudo-random generator (xorshift32), real-time safe
class Random {
  public:
    explicit Random(uint32_t seed = 1) { Seed(seed); }

    void Seed(uint32_t seed) {
        // Scramble so small or neighbouring seeds don't start with similar values
        state_ = (seed * 2654435761u) ^ 0x9E3779B9u;
        if (!state_) state_ = 1u;
        for (int i = 0; i < 4; i++) Next();
    }

    uint32_t Next() {
        state_ ^= state_ << 13;
        state_ ^= state_ >> 17;
        state_ ^= state_ << 5;
        return state_;
    }

    float Uniform() { return (float)(Next() >> 8) * (1.f / 16777216.f); } // 0 to 1
    float Bipolar() { return 2.f * Uniform() - 1.f; }                     // -1 to 1

  private:
    uint32_t state_ = 1u;
};

// Smoothed random walk in -1..1, for drift and fading: every 0.5-1.5 periods (a period
// is 1 / rate_hz) it picks a new random target and glides towards it. Call Next() once
// per audio block.
class Wander {
  public:
    void Init(float block_rate, uint32_t seed) {
        block_rate_ = block_rate;
        random_.Seed(seed);
        value_ = target_ = 0.f;
        phase_ = 0.f;
    }

    float Next(float rate_hz) {
        float rate = fmaxf(rate_hz, 0.001f);
        phase_ -= rate / block_rate_; // counts down in periods, so rate changes apply at once
        if (phase_ <= 0.f) {
            target_ = random_.Bipolar();
            phase_ += 0.5f + random_.Uniform();
        }
        value_ += (1.f - expf(-2.f * rate / block_rate_)) * (target_ - value_);
        return value_;
    }

  private:
    Random random_;
    float block_rate_ = 1000.f;
    float value_ = 0.f, target_ = 0.f;
    float phase_ = 0.f; // periods left until the next target
};

} // namespace bitch
