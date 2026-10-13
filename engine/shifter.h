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
    static constexpr int kFeedbackDelay = 96; // 2 ms; assumes 48 kHz: the feedback delay is a fixed 96 samples

    // Assumes 48 kHz: the feedback delay is a fixed 96 samples.
    void Init(float sample_rate) {
        inv_sample_rate_ = 1.f / sample_rate;
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
        phase_ += shift_hz * inv_sample_rate_;
        phase_ -= floorf(phase_);
        float angle = 6.28318530718f * phase_;
        float out = i * cosf(angle) + q * sinf(angle);
        feedback_buffer_[position_] = out;
        position_ = (position_ + 1) % kFeedbackDelay;
        return out;
    }

    // The oscillator phase (0-1) and a way to nudge it, so the engine can pull two shifters back in phase
    float Phase() const { return phase_; }
    void NudgePhase(float delta) {
        phase_ += delta;
        phase_ -= floorf(phase_);
    }

  private:
    Hilbert hilbert_;
    float inv_sample_rate_ = 1.f / 48000.f;
    float phase_ = 0.f;
    float feedback_buffer_[kFeedbackDelay] = {};
    int position_ = 0;
};

} // namespace bitch
