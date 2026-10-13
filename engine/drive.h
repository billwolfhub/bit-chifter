#pragma once

#include <cmath>

#include "softclip.h"

namespace bitch {

// Input saturation: up to +24 dB into the soft clip (fuzz at the top), scaled back so a
// nominal -12 dBFS signal comes out at about the same level. amount 0 is an exact
// pass-through. Set() once per block; Process() per sample.
class Drive {
  public:
    void Set(float amount) {
        bypass_ = !(amount > 0.f);
        float a = bypass_ ? 0.f : fminf(amount, 1.f);
        gain_ = exp2f(a * a * 3.986f); // 2^3.986 = +24 dB at amount 1; gentle low down
        makeup_ = 0.25f / SoftClip(0.25f * gain_);
        blend_ = fminf(a * 10.f, 1.f); // crossfade in over the first 10%, so leaving 0 is continuous
    }

    float Process(float x) const {
        if (bypass_) return x;
        float d = SoftClip(x * gain_) * makeup_;
        return x + blend_ * (d - x);
    }

  private:
    bool bypass_ = true;
    float gain_ = 1.f, makeup_ = 1.f, blend_ = 0.f;
};

} // namespace bitch
