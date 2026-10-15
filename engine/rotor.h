#pragma once

#include <cmath>

namespace bitch {

// Quadrature sine oscillator by rotation: cheap per sample (4 multiplies), retuned per block
// (SetHz, two trig calls) and renormalised there, so its level stays exact. Frequencies may be
// negative (the rotation runs backwards; the sine is the same pitch). Retuning keeps the phase.
class Rotor {
  public:
    void SetHz(float hz, float sample_rate) {
        float w = 6.28318530718f * hz / sample_rate;
        cos_ = cosf(w);
        sin_ = sinf(w);
        Normalize();
    }
    // Pulls the level back to exactly 1 (SetHz does this; call it per block when not retuning)
    void Normalize() {
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
    // Both outputs: cos and sin of the same advancing angle
    void Next(float &c, float &s) {
        Next();
        c = c_;
        s = s_;
    }
    void Reset() {
        c_ = 1.f;
        s_ = 0.f;
    }

  private:
    float cos_ = 1.f, sin_ = 0.f, c_ = 1.f, s_ = 0.f;
};

} // namespace bitch
