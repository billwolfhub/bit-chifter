#pragma once

#include <cmath>

#include "hilbert.h"
#include "softclip.h"

namespace bitch {

// Single-sideband frequency shifter: the Hilbert splitter's I and Q are mixed
// with a quadrature oscillator at the shift frequency. Positive shifts move the
// spectrum up, negative down; the oscillator runs backwards for negative shifts,
// so the shift sweeps smoothly through 0. Feedback returns the output to the input
// after 2 ms, through a soft clip (barberpole sweeps without running away).
class Shifter {
  public:
    static constexpr int kFeedbackDelay = 96; // 2 ms at 48 kHz

    void Init(float sample_rate) {
        sample_rate_ = sample_rate;
        Reset();
    }

    void Reset() {
        hilbert_.Reset();
        phase_ = 0.f;
        for (float &v : feedback_buffer_) v = 0.f;
        position_ = 0;
    }

    // feedback: 0 to 0.95
    float Process(float x, float shift_hz, float feedback) {
        float delayed = feedback_buffer_[position_];
        float in = x + feedback * SoftClip(delayed);
        float i, q;
        hilbert_.Process(in, i, q);
        phase_ += shift_hz / sample_rate_;
        phase_ -= floorf(phase_);
        float angle = 6.28318530718f * phase_;
        float out = i * cosf(angle) + q * sinf(angle);
        feedback_buffer_[position_] = out;
        position_ = (position_ + 1) % kFeedbackDelay;
        return out;
    }

  private:
    Hilbert hilbert_;
    float sample_rate_ = 48000.f;
    float phase_ = 0.f;
    float feedback_buffer_[kFeedbackDelay] = {};
    int position_ = 0;
};

} // namespace bitch
