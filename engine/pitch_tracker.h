#pragma once

#include <cmath>

namespace bitch {

// Pitch tracker for the harmonic detents: YIN (de Cheveigne & Kawahara, 2002) on the
// input decimated to 8 kHz, 60-800 Hz. A new analysis starts every 32 ms; its difference
// function is spread over the following audio blocks (a few lags per Work() call), so
// there are no CPU spikes. Push() every input sample, Work() once per block.
// Confidence() is 0-1 (0 = silence or no clear pitch).
class PitchTracker {
  public:
    static constexpr int kDecimate = 6;   // 48 kHz -> 8 kHz
    static constexpr int kWindow = 256;   // 32 ms at 8 kHz
    static constexpr int kMinLag = 10;    // 800 Hz
    static constexpr int kMaxLag = 133;   // ~60 Hz
    static constexpr int kFrame = kWindow + kMaxLag + 1;
    static constexpr int kLagsPerWork = 6; // all 134 lags in ~23 blocks, well within 32 ms
    static constexpr float kThreshold = 0.15f;

    void Init(float sample_rate) {
        rate_ = sample_rate / (float)kDecimate;
        lowpass_ = 1.f - expf(-6.28318530718f * 1000.f / sample_rate);
        Reset();
    }

    void Reset() {
        state_ = sum_ = 0.f;
        count_ = write_ = filled_ = since_analysis_ = 0;
        lag_ = -1;
        pitch_ = confidence_ = 0.f;
        for (float &v : history_) v = 0.f;
    }

    void Push(float x) {
        state_ += lowpass_ * (x - state_); // 1 kHz low-pass against aliasing, then average 6
        sum_ += state_;
        if (++count_ < kDecimate) return;
        history_[write_] = sum_ * (1.f / (float)kDecimate);
        write_ = (write_ + 1) % kFrame;
        sum_ = 0.f;
        count_ = 0;
        if (filled_ < kFrame) filled_++;
        since_analysis_++;
    }

    void Work() {
        if (lag_ < 0) {
            if (filled_ < kFrame || since_analysis_ < kWindow) return;
            since_analysis_ = 0;
            float energy = 0.f;
            for (int i = 0; i < kFrame; i++) frame_[i] = history_[(write_ + i) % kFrame]; // oldest first
            for (int i = 0; i < kWindow; i++) energy += frame_[i] * frame_[i];
            if (energy < (float)kWindow * 1e-7f) { // silence (below about -70 dBFS)
                confidence_ = 0.f;
                return;
            }
            lag_ = 1;
            running_sum_ = 0.f;
            cmndf_[0] = 1.f;
        }
        for (int k = 0; k < kLagsPerWork && lag_ <= kMaxLag; k++, lag_++) {
            float d = 0.f;
            for (int i = 0; i < kWindow; i++) {
                float e = frame_[i] - frame_[i + lag_];
                d += e * e;
            }
            running_sum_ += d;
            cmndf_[lag_] = running_sum_ > 0.f ? d * (float)lag_ / running_sum_ : 1.f;
        }
        if (lag_ > kMaxLag) {
            Finish();
            lag_ = -1;
        }
    }

    float Pitch() const { return pitch_; }
    float Confidence() const { return confidence_; }

  private:
    // The first lag below the threshold (followed down to its local minimum) is the period
    void Finish() {
        int best = -1;
        for (int t = kMinLag; t <= kMaxLag; t++) {
            if (cmndf_[t] < kThreshold) {
                while (t + 1 <= kMaxLag && cmndf_[t + 1] < cmndf_[t]) t++;
                best = t;
                break;
            }
        }
        if (best < 0) {
            confidence_ = 0.f;
            return;
        }
        float period = (float)best;
        if (best > kMinLag && best < kMaxLag) { // parabolic interpolation between lags
            float a = cmndf_[best - 1], b = cmndf_[best], c = cmndf_[best + 1];
            float den = a - 2.f * b + c;
            if (fabsf(den) > 1e-9f) period += 0.5f * (a - c) / den;
        }
        pitch_ = rate_ / period;
        confidence_ = fminf(fmaxf(1.f - cmndf_[best], 0.f), 1.f);
    }

    float rate_ = 8000.f, lowpass_ = 0.f, state_ = 0.f, sum_ = 0.f;
    int count_ = 0, write_ = 0, filled_ = 0, since_analysis_ = 0, lag_ = -1;
    float history_[kFrame] = {};
    float frame_[kFrame] = {};
    float cmndf_[kMaxLag + 1] = {};
    float running_sum_ = 0.f;
    float pitch_ = 0.f, confidence_ = 0.f;
};

} // namespace bitch
