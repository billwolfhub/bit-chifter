#pragma once

#include <cmath>
#include <cstdint>

namespace bitch {

// Small, fast, deterministic pseudo-random generator (xorshift32), real-time safe
class Random {
  public:
    explicit Random(uint32_t seed = 1) { Seed(seed); }

    void Seed(uint32_t seed) { state_ = seed ? seed : 1u; }

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
        countdown_ = 0;
    }

    float Next(float rate_hz) {
        if (--countdown_ <= 0) {
            target_ = random_.Bipolar();
            countdown_ = 1 + (int)(block_rate_ / rate_hz * (0.5f + random_.Uniform()));
        }
        value_ += (1.f - expf(-2.f * rate_hz / block_rate_)) * (target_ - value_);
        return value_;
    }

  private:
    Random random_;
    float block_rate_ = 1000.f;
    float value_ = 0.f, target_ = 0.f;
    int countdown_ = 0;
};

} // namespace bitch
