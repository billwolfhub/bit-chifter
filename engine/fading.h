#pragma once

#include <cmath>
#include <cstdint>

#include "random.h"

namespace bitch {

// Multipath fading, like a distant station: the level swells and dips (down to -18 dB)
// and a short echo (0.5-3 ms, subtracted) combs the sound, both on slow random paths
// (0.05-0.5 Hz, faster as depth rises). depth 0 is an exact pass-through.
// Update() once per block, Process() per sample.
class Fading {
  public:
    static constexpr int kSize = 256; // > 3 ms at 48 kHz, power of two
    static constexpr int kMask = kSize - 1;

    void Init(float sample_rate, float block_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        level_path_.Init(block_rate, seed);
        delay_path_.Init(block_rate, seed * 7u + 3u);
        glide_ = 1.f - expf(-1.f / (0.01f * sample_rate)); // ~10 ms
        Reset();
    }

    void Reset() {
        for (float &v : buffer_) v = 0.f;
        position_ = 0;
        gain_ = gain_target_ = 1.f;
        echo_ = echo_target_ = 0.f;
        delay_ = delay_target_ = 0.001f * sample_rate_;
    }

    void Update(float depth) {
        float d = fminf(fmaxf(depth, 0.f), 1.f);
        float rate = 0.05f + 0.45f * d;
        float level = level_path_.Next(rate); // -1..1
        float path = delay_path_.Next(1.7f * rate);
        gain_target_ = 1.f - d * 0.874f * (0.5f + 0.5f * level); // 0.126 = -18 dB at the deepest
        delay_target_ = (0.5f + 1.25f * (1.f + path)) * 0.001f * sample_rate_; // 0.5-3 ms
        echo_target_ = -0.6f * d;
    }

    float Process(float x) {
        gain_ += glide_ * (gain_target_ - gain_);
        echo_ += glide_ * (echo_target_ - echo_);
        delay_ += glide_ * (delay_target_ - delay_);
        buffer_[position_] = x;
        if (gain_target_ == 1.f && echo_target_ == 0.f && fabsf(gain_ - 1.f) < 1e-4f && fabsf(echo_) < 1e-4f) {
            gain_ = 1.f; // settled back at depth 0: exact pass-through (buffer stays filled); 1e-4 (-80 dB) since the float glide stalls ~3e-5 short of 1
            echo_ = 0.f;
            position_ = (position_ + 1) & kMask;
            return x;
        }
        float read = (float)position_ - delay_;
        if (read < 0.f) read += (float)kSize;
        int i0 = (int)read;
        float frac = read - (float)i0;
        float a = buffer_[i0 & kMask], b = buffer_[(i0 + 1) & kMask];
        position_ = (position_ + 1) & kMask;
        return gain_ * (x + echo_ * (a + frac * (b - a)));
    }

  private:
    Wander level_path_, delay_path_;
    float buffer_[kSize] = {};
    int position_ = 0;
    float sample_rate_ = 48000.f, glide_ = 0.f;
    float gain_ = 1.f, gain_target_ = 1.f, echo_ = 0.f, echo_target_ = 0.f, delay_ = 48.f, delay_target_ = 48.f;
};

} // namespace bitch
