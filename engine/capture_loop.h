#pragma once

#include <cmath>

namespace bitch {

// Switch 2: Normal (live input), Freeze or Loop
enum class CaptureMode { kNormal, kFreeze, kLoop };

// Records the input (2 s, stereo, into buffers the caller provides: they live in SDRAM) while
// Normal; on Freeze, holds the last 250 ms as an endless drone (two read heads half a slice
// apart, each with a smooth sin^2 window, so they sum to a constant level); on Loop, repeats
// the last 2 s with a 20 ms crossfade at the loop point. Recording stops while Freeze or Loop
// plays (moving between them plays the same captured audio). Every change crossfades over
// 30 ms against the live input; nothing is captured until 50 ms have been recorded (the
// buffers start as SDRAM garbage, so only recorded samples are ever read). Assumes 48 kHz.
// Set() once per block, Process() per sample.
class CaptureLoop {
  public:
    enum : int { kMax = 96000 }; // 2 s at 48 kHz

    void Init(float sample_rate, float *left, float *right) {
        buffer_[0] = left;
        buffer_[1] = right;
        fade_step_ = 1.f / (0.03f * sample_rate);
        slice_ = (int)(0.25f * sample_rate);
        crossfade_ = (int)(0.02f * sample_rate);
        minimum_ = (int)(0.05f * sample_rate);
        Reset();
    }

    void Reset() {
        write_ = filled_ = 0;
        playing_ = wanted_ = CaptureMode::kNormal;
        mix_ = 0.f;
    }

    void Set(CaptureMode mode) {
        if (mode != CaptureMode::kNormal && playing_ == CaptureMode::kNormal && filled_ < minimum_) return;
        wanted_ = mode;
    }

    void Process(float &l, float &r) {
        // Change what plays only at silence (mix 0), so every change crossfades through live
        if (playing_ != wanted_ && mix_ == 0.f) Start(wanted_);
        float target = playing_ != CaptureMode::kNormal && playing_ == wanted_ ? 1.f : 0.f;
        mix_ = target > mix_ ? fminf(mix_ + fade_step_, 1.f) : fmaxf(mix_ - fade_step_, 0.f);
        if (playing_ == CaptureMode::kNormal) {
            buffer_[0][write_] = l;
            buffer_[1][write_] = r;
            if (++write_ >= kMax) write_ = 0;
            if (filled_ < kMax) filled_++;
            return;
        }
        float cl, cr;
        if (playing_ == CaptureMode::kFreeze) {
            FreezeSample(cl, cr);
        } else {
            LoopSample(cl, cr);
        }
        l += mix_ * (cl - l);
        r += mix_ * (cr - r);
    }

    int LoopSamples() const { return length_ - crossfade_; } // the loop's period, in samples

  private:
    void Start(CaptureMode mode) {
        playing_ = mode;
        if (mode == CaptureMode::kNormal) return;
        end_ = write_; // one past the last recorded sample
        int slice = slice_ < filled_ ? slice_ : filled_;
        grain_ = slice;
        phase_ = 0.f;
        length_ = filled_;
        position_ = crossfade_;
    }

    int Wrap(int i) const { return i < 0 ? i + kMax : (i >= kMax ? i - kMax : i); }

    // Two heads over the last `grain_` samples, half a grain apart, sin^2 windows (sum 1)
    void FreezeSample(float &l, float &r) {
        l = r = 0.f;
        for (int head = 0; head < 2; head++) {
            float p = phase_ + 0.5f * (float)head;
            p -= floorf(p);
            int i = Wrap(end_ - grain_ + (int)(p * (float)grain_));
            float s = sinf(3.14159265359f * p), w = s * s;
            l += w * buffer_[0][i];
            r += w * buffer_[1][i];
        }
        phase_ += 1.f / (float)grain_;
        phase_ -= floorf(phase_);
    }

    // The last `length_` samples, positions crossfade_..length_; the last crossfade_ samples
    // fade into the samples just after the loop's start, then it jumps back to crossfade_
    void LoopSample(float &l, float &r) {
        int start = end_ - length_;
        int i = Wrap(start + position_);
        l = buffer_[0][i];
        r = buffer_[1][i];
        int into = position_ - (length_ - crossfade_);
        if (into >= 0) {
            float a = (float)into / (float)crossfade_;
            int j = Wrap(start + into);
            l += a * (buffer_[0][j] - l);
            r += a * (buffer_[1][j] - r);
        }
        if (++position_ >= length_) position_ = crossfade_;
    }

    float *buffer_[2] = {nullptr, nullptr};
    int write_ = 0, filled_ = 0, end_ = 0, slice_ = 12000, crossfade_ = 960, minimum_ = 2400;
    int grain_ = 12000, length_ = 1, position_ = 0;
    float fade_step_ = 0.f, mix_ = 0.f, phase_ = 0.f;
    CaptureMode playing_ = CaptureMode::kNormal, wanted_ = CaptureMode::kNormal;
};

} // namespace bitch
