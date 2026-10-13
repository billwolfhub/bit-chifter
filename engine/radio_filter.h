#pragma once

#include <cmath>

namespace bitch {

// One band-pass stage: a trapezoidal state-variable filter (Andrew Simper's form),
// stable under fast parameter changes. The output is scaled to unity gain at the centre.
class Bandpass {
  public:
    void Reset() { ic1_ = ic2_ = 0.f; }

    // g = tan(pi * centre / sample_rate), k = 1 / Q
    void SetCoefficients(float g, float k) {
        k_ = k;
        a1_ = 1.f / (1.f + g * (g + k));
        a2_ = g * a1_;
        a3_ = g * a2_;
    }

    float Process(float x) {
        float v3 = x - ic2_;
        float v1 = a1_ * ic1_ + a2_ * v3;
        float v2 = ic2_ + a2_ * ic1_ + a3_ * v3;
        ic1_ = 2.f * v1 - ic1_;
        ic2_ = 2.f * v2 - ic2_;
        return k_ * v1;
    }

  private:
    float ic1_ = 0.f, ic2_ = 0.f;
    float a1_ = 0.f, a2_ = 0.f, a3_ = 0.f, k_ = 1.f;
};

// The receiver's narrow, boxy band: two band-pass stages in series, with partial
// make-up gain so narrowing it doesn't make it much quieter, and an "open" control
// that crossfades to the unfiltered signal (1 = fully bypassed).
class RadioFilter {
  public:
    void Init(float sample_rate) {
        sample_rate_ = sample_rate;
        Reset();
        Set(1000.f, 0.7f, 1.f);
    }

    void Reset() {
        stage_[0].Reset();
        stage_[1].Reset();
    }

    // centre_hz: 150-5000 Hz; q: 0.7 (wide) to 12 (narrow, whistly); open: 0-1
    void Set(float centre_hz, float q, float open) {
        float centre = fminf(fmaxf(centre_hz, 20.f), 0.45f * sample_rate_);
        float qq = fminf(fmaxf(q, 0.5f), 20.f);
        float g = tanf(3.14159265f * centre / sample_rate_);
        for (Bandpass &s : stage_) s.SetCoefficients(g, 1.f / qq);
        makeup_ = powf(qq / 0.7f, 0.35f);
        open_ = fminf(fmaxf(open, 0.f), 1.f);
    }

    float Process(float x) {
        float y = stage_[1].Process(stage_[0].Process(x)) * makeup_;
        return y + open_ * (x - y);
    }

  private:
    Bandpass stage_[2];
    float sample_rate_ = 48000.f;
    float makeup_ = 1.f;
    float open_ = 1.f;
};

} // namespace bitch
