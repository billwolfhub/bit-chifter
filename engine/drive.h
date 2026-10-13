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
    }

    float Process(float x) const { return bypass_ ? x : SoftClip(x * gain_) * makeup_; }

  private:
    bool bypass_ = true;
    float gain_ = 1.f, makeup_ = 1.f;
};

} // namespace bitch
