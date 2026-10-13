#pragma once

#include <atomic>
#include <cmath>

namespace bitch {

// Short confirmation beeps (the board has no screen): Start(n) plays n beeps of a 1.5 kHz
// tone (Start(n, true): 500 Hz), 60 ms each with 90 ms gaps, at about -14 dBFS. Process()
// returns the next sample to add to both outputs.
class Blips {
  public:
    void Init(float sample_rate) {
        on_samples_ = (int)(0.06f * sample_rate);
        period_samples_ = (int)(0.15f * sample_rate);
        sample_rate_ = sample_rate;
        step_ = 1500.f / sample_rate;
        fade_samples_ = 0.005f * sample_rate; // 5 ms fade in and out so the beeps don't click
        inv_fade_samples_ = 1.f / fade_samples_;
        remaining_ = 0;
        request_.store(0, std::memory_order_relaxed);
    }

    // Start() may be called from the main loop while Process() runs in the audio callback.
    // The count and the pitch travel in one atomic word (the count masked to 0-255, so it can't
    // reach the kLow flag).
    void Start(int count, bool low = false) {
        request_.store((count & 0xFF) | (low ? kLow : 0), std::memory_order_relaxed);
    }

    float Process() {
        int r = request_.exchange(0, std::memory_order_relaxed);
        if (r > 0) {
            remaining_ = r & ~kLow;
            step_ = ((r & kLow) ? 500.f : 1500.f) / sample_rate_;
            position_ = 0;
            phase_ = 0.f;
        }
        if (remaining_ <= 0) return 0.f;
        float out = 0.f;
        if (position_ < on_samples_) {
            phase_ += step_;
            if (phase_ >= 1.f) phase_ -= 1.f;
            float edge = fminf((float)position_, (float)(on_samples_ - position_)) * inv_fade_samples_;
            out = 0.2f * fminf(edge, 1.f) * sinf(6.28318530718f * phase_);
        }
        if (++position_ >= period_samples_) {
            position_ = 0;
            remaining_--;
        }
        return out;
    }

  private:
    enum : int { kLow = 0x100 };
    int on_samples_ = 0, period_samples_ = 0, position_ = 0, remaining_ = 0;
    float sample_rate_ = 48000.f;
    float step_ = 0.f, phase_ = 0.f, fade_samples_ = 1.f, inv_fade_samples_ = 1.f;
    std::atomic<int> request_{0};
};

} // namespace bitch
