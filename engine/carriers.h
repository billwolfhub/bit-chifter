#pragma once

#include <cmath>
#include <cstdint>

#include "effects.h"
#include "random.h"

namespace bitch {

// Quadrature sine oscillator by rotation: cheap per sample (4 multiplies), retuned per block
// (SetHz, two trig calls) and renormalised there, so its level stays exact. Frequencies may be
// negative (the rotation runs backwards; the sine is the same pitch).
class Rotor {
  public:
    void SetHz(float hz, float sample_rate) {
        float w = 6.28318530718f * hz / sample_rate;
        cos_ = cosf(w);
        sin_ = sinf(w);
        float norm = 1.5f - 0.5f * (c_ * c_ + s_ * s_);
        c_ *= norm;
        s_ *= norm;
    }
    float Next() {
        float c = c_ * cos_ - s_ * sin_;
        s_ = s_ * cos_ + c_ * sin_;
        c_ = c;
        return s_;
    }
    void Reset() {
        c_ = 1.f;
        s_ = 0.f;
    }

  private:
    float cos_ = 1.f, sin_ = 0.f, c_ = 1.f, s_ = 0.f;
};

// Passing carriers: hidden "stations", steady carriers at fixed random places within
// +-2.5 kHz (at least 150 Hz from 0). A receiver tuned to `dial_hz` hears each one as a
// whistle at its offset from the dial, so moving TUNE sweeps them through high, zero and
// back. 2 stations (amount 0.35) to 6 (amount 1), each at -30 dBFS (amount 0.35) to
// -12 dBFS (amount 1), each drifting slowly (+-20 Hz). Update() once per block (with the
// dial and the pad's fade level), Process() per sample.
class Carriers {
  public:
    enum : int { kMax = 6 };

    void Init(float sample_rate, float block_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        Random random(seed);
        for (int k = 0; k < kMax; k++) {
            float side = random.Uniform() < 0.5f ? -1.f : 1.f;
            position_[k] = side * (150.f + 2350.f * random.Uniform());
            drift_[k].Init(block_rate, seed * 31u + (uint32_t)k);
            rotor_[k].Reset();
            gain_[k] = 0.f;
        }
    }

    static int Count(float amount) { return 2 + (int)lrintf(4.f * EffectRange(amount)); }
    float Position(int k) const { return position_[k]; }

    void Update(float amount, float level, float dial_hz) {
        float u = EffectRange(amount);
        float each = powf(10.f, 0.05f * (-30.f + 18.f * u)) * level;
        int count = Count(amount);
        for (int k = 0; k < kMax; k++) {
            float hz = position_[k] + 20.f * drift_[k].Next(0.1f) - dial_hz;
            rotor_[k].SetHz(hz, sample_rate_);
            float target = k < count ? each : 0.f;
            step_[k] = (target - gain_[k]) * (1.f / 48.f); // ramped over the next 48 samples
            left_[k] = 48;
        }
    }

    float Process() {
        float out = 0.f;
        for (int k = 0; k < kMax; k++) {
            if (left_[k] > 0) {
                gain_[k] += step_[k];
                left_[k]--;
            }
            if (gain_[k] != 0.f) out += gain_[k] * rotor_[k].Next();
        }
        return out;
    }

  private:
    float sample_rate_ = 48000.f;
    float position_[kMax] = {};
    Wander drift_[kMax];
    Rotor rotor_[kMax];
    float gain_[kMax] = {}, step_[kMax] = {};
    int left_[kMax] = {};
};

} // namespace bitch
