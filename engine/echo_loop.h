#pragma once

#include <cmath>

#include "softclip.h"

namespace bitch {

// The echoes' delay: up to 1 s in a buffer the caller provides (SDRAM). Write() the loop's
// signal (it goes in through a gentle ~6 kHz low-pass and the soft clip), Read() the echo
// (call Read() before Write() each sample). The echo time glides towards its target over
// ~0.5 s, like tape (warping the pitch). Reverse (Read's amount, 0-1: between them it blends
// the two, so the pad can crossfade): two read heads, half a window apart, with sin^2 and
// cos^2 windows, sweep backwards through the last echo time, so each repeat plays backwards.
// ClearStep() zeroes it in bounded steps (call while it isn't in use) and returns true once
// it's clean. Assumes 48 kHz. Update() once per block.
class EchoLoop {
  public:
    enum : int { kMax = 48000 }; // 1 s at 48 kHz

    void Init(float sample_rate, float *buffer) {
        sample_rate_ = sample_rate;
        buffer_ = buffer;
        low_pass_ = 1.f - expf(-6.28318530718f * 6000.f / sample_rate);
        glide_ = 1.f - expf(-1.f / (0.5f * sample_rate));
        cleared_ = 0;
        write_ = 0;
        state_ = 0.f;
        phase_ = 0.f;
        SetTimeNow(0.15f);
    }

    // Jumps to an echo time (no glide), e.g. when the echo starts
    void SetTimeNow(float seconds) { delay_ = target_ = Samples(seconds); }

    void Update(float seconds, int block_size) {
        (void)block_size;
        target_ = Samples(seconds);
    }

    // reverse: 0 forward, 1 backwards (in between: both, blended)
    float Read(float reverse = 0.f) {
        delay_ += glide_ * (target_ - delay_);
        if (reverse <= 0.f) return Tap(delay_);
        // Reverse: each head's distance grows 2 samples per sample (so it reads backwards)
        // from 0 to the echo time over half an echo time; the second head is half a window on
        float window = 0.5f * delay_;
        float p = phase_ + 0.5f;
        p -= floorf(p);
        float s = sinf(3.14159265359f * phase_), w = s * s; // the second head's window: 1 - w
        float back = w * Tap(1.f + 2.f * window * phase_) + (1.f - w) * Tap(1.f + 2.f * window * p);
        phase_ += 1.f / window;
        phase_ -= floorf(phase_);
        if (reverse >= 1.f) return back;
        float forward = Tap(delay_);
        return forward + reverse * (back - forward);
    }

    void Write(float x) {
        state_ += low_pass_ * (x - state_);
        buffer_[write_] = SoftClip(state_);
        if (++write_ >= kMax) write_ = 0;
    }

    bool ClearStep(int samples) {
        int end = cleared_ + samples < kMax ? cleared_ + samples : kMax;
        for (int i = cleared_; i < end; i++) buffer_[i] = 0.f;
        cleared_ = end;
        if (cleared_ < kMax) return false;
        cleared_ = 0;
        state_ = 0.f;
        return true;
    }

    float TimeSeconds() const { return delay_ / sample_rate_; }

  private:
    float Samples(float seconds) const {
        float s = seconds * sample_rate_;
        return s < 2.f ? 2.f : (s > (float)(kMax - 2) ? (float)(kMax - 2) : s);
    }
    // The sample written `d` samples ago (fractional, linear), 1 <= d <= kMax - 2
    float Tap(float d) const {
        int whole = (int)d;
        float frac = d - (float)whole;
        int a = write_ - whole, b = a - 1;
        if (a < 0) a += kMax;
        if (b < 0) b += kMax;
        return buffer_[a] + frac * (buffer_[b] - buffer_[a]);
    }

    float *buffer_ = nullptr;
    float sample_rate_ = 48000.f, low_pass_ = 0.f, glide_ = 0.f, state_ = 0.f;
    float delay_ = 7200.f, target_ = 7200.f, phase_ = 0.f;
    int write_ = 0, cleared_ = 0;
};

} // namespace bitch
