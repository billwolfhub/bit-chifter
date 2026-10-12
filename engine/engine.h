#pragma once

#include <cstddef>

#include "shifter.h"

namespace bitch {

// Everything the front end controls. Part 1 uses the shifter controls only.
struct Params {
    float tune_hz = 0.f;  // shift, -2500 to +2500 Hz
    float feedback = 0.f; // 0 to 0.95
    float mix = 1.f;      // dry/wet, 0 to 1
};

// The sound engine: no hardware code, so it runs on the Daisy and on the Mac.
// The inputs are summed to mono; L and R each have their own shifter (they will
// drift apart once Reception is added, which is where the stereo comes from).
class Engine {
  public:
    void Init(float sample_rate);
    void Process(const float *in_l, const float *in_r, float *out_l, float *out_r, size_t size,
                 const Params &params);

  private:
    void Reset();

    Shifter shifter_[2];
    float shift_hz_ = 0.f; // glided towards params.tune_hz
    float glide_ = 0.f;    // one-pole coefficient, ~20 ms
};

} // namespace bitch
