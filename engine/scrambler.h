#pragma once

#include <cmath>
#include <cstdint>

#include "effects.h"
#include "hilbert.h"
#include "random.h"

namespace bitch {

// Voice scrambler, like old radio scramblers: the spectrum is turned upside down around an
// inversion frequency f (0..f becomes f..0). The input is low-passed at f (4th-order
// Butterworth, so nothing above f folds back), split into I and Q, and mixed with an
// oscillator at f taking the opposite sideband from the shifter's: i*cos - q*sin.
// Plain: f from 3.4 kHz (amount 0.35) down to 1.2 kHz (amount 1). Hopping: f jumps to a
// random 1-4 kHz every 1.5 s (amount 0.35) down to every 0.1 s (amount 1), gliding over
// ~5 ms. Update() once per block, Process() per sample.
class Scrambler {
  public:
    void Init(float sample_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        glide_ = 1.f - expf(-1.f / (0.005f * sample_rate));
        random_.Seed(seed);
        Reset();
    }

    void Reset() {
        hilbert_.Reset();
        for (Stage &s : low_pass_) s.ic1 = s.ic2 = 0.f;
        phase_ = 0.f;
        inversion_hz_ = target_hz_ = 3400.f;
        hop_left_s_ = 0.f;
        SetLowPass();
    }

    void Update(float amount, bool hopping, float block_seconds) {
        float u = EffectRange(amount);
        if (!hopping) {
            target_hz_ = 3400.f - 2200.f * u;
        } else {
            hop_left_s_ -= block_seconds;
            if (hop_left_s_ <= 0.f) {
                target_hz_ = 1000.f + 3000.f * random_.Uniform();
                hop_left_s_ = 1.5f * powf(0.1f / 1.5f, u);
            }
        }
        SetLowPass();
    }

    float Process(float x) {
        inversion_hz_ += glide_ * (target_hz_ - inversion_hz_);
        float y = x;
        for (Stage &s : low_pass_) y = s.Process(y);
        float i, q;
        hilbert_.Process(y, i, q);
        phase_ += inversion_hz_ / sample_rate_;
        phase_ -= floorf(phase_);
        float angle = 6.28318530718f * phase_;
        return i * cosf(angle) - q * sinf(angle);
    }

    float InversionHz() const { return target_hz_; }

  private:
    // Andrew Simper's state-variable filter, low-pass output
    struct Stage {
        float a1 = 0.f, a2 = 0.f, a3 = 0.f, ic1 = 0.f, ic2 = 0.f;
        void Set(float g, float k) {
            a1 = 1.f / (1.f + g * (g + k));
            a2 = g * a1;
            a3 = g * a2;
        }
        float Process(float x) {
            float v3 = x - ic2;
            float v1 = a1 * ic1 + a2 * v3;
            float v2 = ic2 + a2 * ic1 + a3 * v3;
            ic1 = 2.f * v1 - ic1;
            ic2 = 2.f * v2 - ic2;
            return v2;
        }
    };

    // The low-pass follows the inversion frequency (per block)
    void SetLowPass() {
        float g = tanf(3.14159265359f * fminf(inversion_hz_, 0.45f * sample_rate_) / sample_rate_);
        low_pass_[0].Set(g, 1.8478f); // Butterworth 4th order: Q 0.541 and 1.307
        low_pass_[1].Set(g, 0.7654f);
    }

    Hilbert hilbert_;
    Stage low_pass_[2];
    Random random_;
    float sample_rate_ = 48000.f, glide_ = 0.f, phase_ = 0.f;
    float inversion_hz_ = 3400.f, target_hz_ = 3400.f, hop_left_s_ = 0.f;
};

} // namespace bitch
