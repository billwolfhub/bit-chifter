#pragma once

#include <cmath>
#include <cstdint>

#include "effects.h"
#include "hilbert.h"
#include "random.h"
#include "rotor.h"

namespace bitch {

// Voice scrambler, like old radio scramblers: the spectrum is turned upside down around an
// inversion frequency f (0..f becomes f..0). Band-limited in and out like a real scrambler:
// the input is high-passed at 250 Hz (2nd-order Butterworth; this also keeps DC from coming
// out as a tone at f) and low-passed at 0.75 f (8th-order Butterworth), so what's above f is
// removed rather than folded back (just above f: at least 30 dB down); then split into I and
// Q, and mixed with an oscillator at f taking the opposite sideband from the shifter's:
// i*cos - q*sin; and the output is high-passed at 250 Hz again (what's left of the fold-back
// lands low, so this takes it further down).
// Plain: f from 3.4 kHz (amount 0.35) down to 1.2 kHz (amount 1). Hopping: f jumps to a
// random 1-4 kHz every 1.5 s (amount 0.35) down to every 0.1 s (amount 1), gliding over
// ~5 ms (per block). Update() once per block, Process() per sample.
class Scrambler {
  public:
    void Init(float sample_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        random_.Seed(seed);
        float g = tanf(3.14159265359f * 250.f / sample_rate);
        high_pass_.Set(g, 1.41421356f);
        out_high_pass_.Set(g, 1.41421356f);
        Reset();
    }

    void Reset() {
        hilbert_.Reset();
        high_pass_.ic1 = high_pass_.ic2 = out_high_pass_.ic1 = out_high_pass_.ic2 = 0.f;
        for (Stage &s : low_pass_) s.ic1 = s.ic2 = 0.f;
        rotor_.Reset();
        inversion_hz_ = target_hz_ = 3400.f;
        hop_left_s_ = 0.f;
        Retune();
    }

    void Update(float amount, bool hopping, float block_seconds) {
        float u = EffectRange(amount);
        if (!hopping) {
            target_hz_ = 3400.f - 2200.f * u;
        } else {
            hop_left_s_ -= block_seconds;
            if (hop_left_s_ <= 0.f) {
                target_hz_ = 1000.f + 3000.f * random_.Uniform();
                hop_left_s_ = 1.5f * powf(0.1f / 1.5f, u);
            }
        }
        if (block_seconds != block_seconds_) { // ~5 ms glide, per block
            block_seconds_ = block_seconds;
            glide_ = 1.f - expf(-block_seconds / 0.005f);
        }
        inversion_hz_ += glide_ * (target_hz_ - inversion_hz_);
        Retune();
    }

    float Process(float x) {
        float y = high_pass_.HighPass(x);
        for (Stage &s : low_pass_) y = s.LowPass(y);
        float i, q, c, s;
        hilbert_.Process(y, i, q);
        rotor_.Next(c, s);
        return out_high_pass_.HighPass(i * c - q * s);
    }

    float InversionHz() const { return target_hz_; }

  private:
    // Andrew Simper's state-variable filter (g = tan(pi fc / fs), k = 1 / Q)
    struct Stage {
        float a1 = 0.f, a2 = 0.f, a3 = 0.f, k = 0.f, ic1 = 0.f, ic2 = 0.f;
        void Set(float g, float damping) {
            k = damping;
            a1 = 1.f / (1.f + g * (g + k));
            a2 = g * a1;
            a3 = g * a2;
        }
        float LowPass(float x) {
            float v1, v2;
            Tick(x, v1, v2);
            return v2;
        }
        float HighPass(float x) {
            float v1, v2;
            Tick(x, v1, v2);
            return x - k * v1 - v2;
        }
        void Tick(float x, float &v1, float &v2) {
            float v3 = x - ic2;
            v1 = a1 * ic1 + a2 * v3;
            v2 = ic2 + a2 * ic1 + a3 * v3;
            ic1 = 2.f * v1 - ic1;
            ic2 = 2.f * v2 - ic2;
        }
    };

    // The low-pass and the oscillator follow the inversion frequency (per block)
    void Retune() {
        float g = tanf(3.14159265359f * fminf(0.75f * inversion_hz_, 0.45f * sample_rate_) / sample_rate_);
        static const float kK[4] = {1.f / 0.5098f, 1.f / 0.6013f, 1.f / 0.9000f, 1.f / 2.5629f}; // Butterworth 8th order
        for (int n = 0; n < 4; n++) low_pass_[n].Set(g, kK[n]);
        rotor_.SetHz(inversion_hz_, sample_rate_);
    }

    Hilbert hilbert_;
    Stage high_pass_, low_pass_[4], out_high_pass_;
    Rotor rotor_;
    Random random_;
    float sample_rate_ = 48000.f, glide_ = 0.f, block_seconds_ = 0.f;
    float inversion_hz_ = 3400.f, target_hz_ = 3400.f, hop_left_s_ = 0.f;
};

} // namespace bitch
