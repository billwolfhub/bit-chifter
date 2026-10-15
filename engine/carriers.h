#pragma once

#include <cmath>
#include <cstdint>

#include "effects.h"
#include "random.h"
#include "rotor.h"

namespace bitch {

// Smooth fade to 0 as a station's heard offset nears 0 Hz (full from 200 Hz): tuned onto a
// station, the whistle falls to zero beat and fades out, rather than putting out DC and subsonics
inline float ZeroBeatTaper(float hz) {
    float t = fminf(fabsf(hz) / 200.f, 1.f);
    return t * t * (3.f - 2.f * t);
}

// Passing carriers: hidden "stations", steady carriers at fixed random places within
// +-2.5 kHz (at least 150 Hz from 0). A receiver tuned to `dial_hz` hears each one as a
// whistle at position + dial (the dial moves them the same way as the voice), so moving TUNE
// sweeps them through high, zero beat (faded out within 200 Hz of it) and back. 2 stations
// (amount 0.35) to 6 (amount 1), each at -30 dBFS (amount 0.35) to -12 dBFS (amount 1), each
// drifting slowly (+-20 Hz). Update() once per block (with the dial and the pad's fade level;
// the gains are ramped across the next block), Process() per sample.
class Carriers {
  public:
    enum : int { kMax = 6 };

    void Init(float sample_rate, float block_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        block_ = (int)lrintf(sample_rate / block_rate);
        if (block_ < 1) block_ = 1;
        Random random(seed);
        for (int k = 0; k < kMax; k++) {
            float side = random.Uniform() < 0.5f ? -1.f : 1.f;
            position_[k] = side * (150.f + 2350.f * random.Uniform());
            drift_[k].Init(block_rate, seed * 31u + (uint32_t)k);
        }
        Reset();
    }

    // Silent, with the gains at 0 (the stations stay where they are)
    void Reset() {
        for (int k = 0; k < kMax; k++) {
            rotor_[k].Reset();
            gain_[k] = step_[k] = target_[k] = 0.f;
            left_[k] = 0;
        }
    }

    static int Count(float amount) { return 2 + (int)lrintf(4.f * EffectRange(amount)); }
    float Position(int k) const { return position_[k]; }

    void Update(float amount, float level, float dial_hz) {
        float u = EffectRange(amount);
        float each = powf(10.f, 0.05f * (-30.f + 18.f * u)) * level;
        int count = Count(amount);
        for (int k = 0; k < kMax; k++) {
            float hz = position_[k] + 20.f * drift_[k].Next(0.1f) + dial_hz;
            rotor_[k].SetHz(hz, sample_rate_);
            target_[k] = k < count ? each * ZeroBeatTaper(hz) : 0.f;
            step_[k] = (target_[k] - gain_[k]) / (float)block_; // ramped over the next block
            left_[k] = block_;
        }
    }

    float Process() {
        float out = 0.f;
        for (int k = 0; k < kMax; k++) {
            if (left_[k] > 0) {
                gain_[k] = --left_[k] > 0 ? gain_[k] + step_[k] : target_[k];
            }
            if (gain_[k] != 0.f) out += gain_[k] * rotor_[k].Next();
        }
        return out;
    }

  private:
    float sample_rate_ = 48000.f;
    int block_ = 48;
    float position_[kMax] = {};
    Wander drift_[kMax];
    Rotor rotor_[kMax];
    float gain_[kMax] = {}, step_[kMax] = {}, target_[kMax] = {};
    int left_[kMax] = {};
};

} // namespace bitch
