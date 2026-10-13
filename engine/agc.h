#pragma once

#include <cmath>

namespace bitch {

// Receiver-style automatic gain control: follows the input's peak level (attack 5 ms,
// release 300 ms) and steers the gain towards a 0.3 peak. At strength 1 it can boost by
// up to +18 dB and cut by up to -12 dB; strength 0 is an exact pass-through. In quiet
// passages the gain rises, so static swells up in the gaps. Update() once per block
// (sets the gain target from the level so far), Process() per sample.
class Agc {
  public:
    static constexpr float kTarget = 0.3f;

    void Init(float sample_rate) {
        attack_ = 1.f - expf(-1.f / (0.005f * sample_rate));
        release_ = 1.f - expf(-1.f / (0.3f * sample_rate));
        glide_ = 1.f - expf(-1.f / (0.02f * sample_rate));
        Reset();
    }

    void Reset() {
        envelope_ = 0.f;
        gain_ = gain_target_ = 1.f;
        bypass_ = true;
    }

    void Update(float strength) {
        float s = fminf(fmaxf(strength, 0.f), 1.f);
        bypass_ = !(s > 0.f) && gain_ == 1.f;
        if (!(s > 0.f)) {
            gain_target_ = 1.f;
            return;
        }
        float wanted = kTarget / fmaxf(envelope_, 1e-4f);
        float most = exp2f(2.99f * s);    // +18 dB at strength 1
        float least = exp2f(-1.993f * s); // -12 dB at strength 1
        gain_target_ = fminf(fmaxf(wanted, least), most);
    }

    float Process(float x) {
        float level = fabsf(x);
        envelope_ += (level > envelope_ ? attack_ : release_) * (level - envelope_);
        if (bypass_) return x;
        gain_ += glide_ * (gain_target_ - gain_);
        if (gain_target_ == 1.f && fabsf(gain_ - 1.f) < 1e-5f) gain_ = 1.f;
        return x * gain_;
    }

  private:
    float attack_ = 0.f, release_ = 0.f, glide_ = 0.f;
    float envelope_ = 0.f, gain_ = 1.f, gain_target_ = 1.f;
    bool bypass_ = true;
};

} // namespace bitch
