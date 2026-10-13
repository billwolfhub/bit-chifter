#pragma once

#include <cmath>

namespace bitch {

// Short confirmation beeps (the board has no screen): Start(n) plays n beeps of a
// 1.5 kHz tone, 60 ms each with 90 ms gaps, at about -14 dBFS. Process() returns the
// next sample to add to both outputs.
class Blips {
  public:
    void Init(float sample_rate) {
        on_samples_ = (int)(0.06f * sample_rate);
        period_samples_ = (int)(0.15f * sample_rate);
        step_ = 1500.f / sample_rate;
        remaining_ = 0;
    }

    void Start(int count) {
        remaining_ = count;
        position_ = 0;
        phase_ = 0.f;
    }

    float Process() {
        if (remaining_ <= 0) return 0.f;
        float out = 0.f;
        if (position_ < on_samples_) {
            phase_ += step_;
            if (phase_ >= 1.f) phase_ -= 1.f;
            // 5 ms fade in and out so the beeps don't click
            float edge = fminf((float)position_, (float)(on_samples_ - position_)) / (0.005f * on_samples_ / 0.06f);
            out = 0.2f * fminf(edge, 1.f) * sinf(6.28318530718f * phase_);
        }
        if (++position_ >= period_samples_) {
            position_ = 0;
            remaining_--;
        }
        return out;
    }

  private:
    int on_samples_ = 0, period_samples_ = 0, position_ = 0, remaining_ = 0;
    float step_ = 0.f, phase_ = 0.f;
};

} // namespace bitch
