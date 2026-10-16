#pragma once

#include <cmath>
#include <cstdint>

#include "effects.h"
#include "radio_filter.h"
#include "random.h"

namespace bitch {

// Codec warble: the watery sound of a low-bitrate stream. An 8-band filter bank (band-passes from
// 150 Hz to 3.5 kHz, about 0.65 octave apart, then a 4th-order Butterworth low-pass at 4 kHz for
// the top cut) whose
// bands are each re-levelled at random every 25-66 ms (15-40 times a second): to between
// 1 - depth and 1, or now and then gated almost off. Depth 0.5 (amount 0.35) to 1 (amount 1);
// the gating rises with the amount (10% to 35% of the changes). Level changes glide over 2 ms.
// Stereo: both sides share the band levels. Update() once per block, Process() per sample.
class CodecWarble {
  public:
    enum : int { kBands = 8 };

    void Init(float sample_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        glide_ = 1.f - expf(-1.f / (0.002f * sample_rate));
        random_.Seed(seed);
        for (int k = 0; k < kBands; k++) {
            float hz = 150.f * powf(3500.f / 150.f, (float)k / (float)(kBands - 1));
            float g = tanf(3.14159265359f * hz / sample_rate);
            for (Bandpass &b : band_[k]) b.SetCoefficients(g, 1.f / 2.2f);
        }
        float g = tanf(3.14159265359f * 4000.f / sample_rate);
        for (int side = 0; side < 2; side++) {
            top_[side][0].Set(g, 1.8478f); // 4th-order Butterworth: two stages, k = 1/Q
            top_[side][1].Set(g, 0.7654f);
        }
        Reset();
    }

    void Reset() {
        for (auto &side : top_)
            for (LowPass &s : side) s.Reset();
        for (int k = 0; k < kBands; k++) {
            for (Bandpass &b : band_[k]) b.Reset();
            gain_[k] = target_[k] = 1.f;
            left_[k] = 0;
        }
    }

    void Update(float amount, int block_size) {
        float u = EffectRange(amount);
        float depth = 0.5f + 0.5f * u, gate = 0.1f + 0.25f * u;
        for (int k = 0; k < kBands; k++) {
            left_[k] -= block_size;
            if (left_[k] > 0) continue;
            left_[k] = (int)((0.025f + 0.041f * random_.Uniform()) * sample_rate_);
            target_[k] = random_.Uniform() < gate ? 0.02f : 1.f - depth * random_.Uniform();
        }
    }

    void Process(float &l, float &r) {
        float out_l = 0.f, out_r = 0.f;
        for (int k = 0; k < kBands; k++) {
            gain_[k] += glide_ * (target_[k] - gain_[k]);
            out_l += gain_[k] * band_[k][0].Process(l);
            out_r += gain_[k] * band_[k][1].Process(r);
        }
        // the 8 unity-peak bands overlap: their sum is a little above unity
        l = 0.6f * top_[0][1].Process(top_[0][0].Process(out_l));
        r = 0.6f * top_[1][1].Process(top_[1][0].Process(out_r));
    }

  private:
    // One low-pass stage: Andrew Simper's state-variable filter (g = tan(pi fc / fs), k = 1 / Q)
    struct LowPass {
        float a1 = 0.f, a2 = 0.f, a3 = 0.f, ic1 = 0.f, ic2 = 0.f;
        void Set(float g, float k) {
            a1 = 1.f / (1.f + g * (g + k));
            a2 = g * a1;
            a3 = g * a2;
        }
        void Reset() { ic1 = ic2 = 0.f; }
        float Process(float x) {
            float v3 = x - ic2;
            float v1 = a1 * ic1 + a2 * v3;
            float v2 = ic2 + a2 * ic1 + a3 * v3;
            ic1 = 2.f * v1 - ic1;
            ic2 = 2.f * v2 - ic2;
            return v2;
        }
    };

    LowPass top_[2][2];
    Bandpass band_[kBands][2];
    float gain_[kBands] = {}, target_[kBands] = {};
    int left_[kBands] = {};
    float sample_rate_ = 48000.f, glide_ = 0.f;
    Random random_;
};

} // namespace bitch
