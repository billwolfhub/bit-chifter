#pragma once

#include <cmath>

namespace bitch {

// Ring modulator: multiplies the signal by a sine carrier, giving the sum and
// difference sidebands (metallic, bell-like, or tremolo at low carrier rates).
// amount 0 leaves the signal unchanged, 1 is fully ring-modulated.
class RingMod {
  public:
    void Init(float sample_rate) {
        inv_sample_rate_ = 1.f / sample_rate;
        Reset();
    }

    void Reset() { phase_ = 0.f; }

    float Process(float x, float carrier_hz, float amount) {
        phase_ += carrier_hz * inv_sample_rate_;
        phase_ -= floorf(phase_);
        float carrier = sinf(6.28318530718f * phase_);
        return x * (1.f - amount + amount * carrier);
    }

  private:
    float inv_sample_rate_ = 1.f / 48000.f;
    float phase_ = 0.f;
};

} // namespace bitch
