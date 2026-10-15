#pragma once

#include <cmath>
#include <cstdint>

#include "effects.h"
#include "random.h"

namespace bitch {

// Selcall: 5-tone selective-calling bursts, as heard on shortwave and utility bands. Each
// burst is 5 random digits in the ZVEI tone set (70 ms per tone) or the CCIR set (100 ms),
// chosen at random; a digit repeating the one before uses the set's repeat tone. About
// -14 dBFS with 4 ms tone edges. While on: the first burst at the touch, then one every 2 s
// (amount 0.35) down to every 0.4 s (amount 1), counted from each burst's start (a burst
// still playing finishes first, then 50 ms of quiet). Off: the current burst finishes, no
// new ones. Update() once per block, Process() per sample (mono).
class Selcall {
  public:
    void Init(float sample_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        random_.Seed(seed);
        edge_ = (int)(0.004f * sample_rate);
        tone_ = -1;
        was_on_ = false;
        next_ = 0;
        phase_ = 0.f;
        tone_hz_ = 0.f;
    }

    void Update(bool on, float amount) {
        int interval = (int)(2.f * powf(0.2f, EffectRange(amount)) * sample_rate_);
        if (on && !was_on_) next_ = 0; // fire at the touch
        was_on_ = on;
        on_ = on;
        interval_ = interval;
    }

    float Process() {
        if (on_ && tone_ < 0) {
            if (--next_ <= 0) Start();
        } else if (on_) {
            next_--;
        }
        if (tone_ < 0) return 0.f;
        float edge = fminf(fminf((float)position_, (float)(length_ - position_)) / (float)edge_, 1.f);
        phase_ += tone_hz_ / sample_rate_;
        phase_ -= floorf(phase_);
        float out = 0.2f * edge * sinf(6.28318530718f * phase_);
        if (++position_ >= length_) {
            position_ = 0;
            if (++tone_ >= 5) {
                tone_ = -1;
                if (next_ < (int)(0.05f * sample_rate_)) next_ = (int)(0.05f * sample_rate_);
            } else {
                tone_hz_ = Tone(tone_);
            }
        }
        return out;
    }

    float ToneHz() const { return tone_hz_; } // the current (or last) tone

    // Drops a burst that was cut off (the engine calls this once the pad's fade has ended, so
    // a half-played burst doesn't resume at the next touch)
    void Stop() { tone_ = -1; }

  private:
    void Start() {
        ccir_ = random_.Uniform() < 0.5f;
        for (int k = 0; k < 5; k++) digits_[k] = (int)(random_.Uniform() * 10.f) % 10;
        length_ = (int)((ccir_ ? 0.1f : 0.07f) * sample_rate_);
        tone_ = 0;
        position_ = 0;
        tone_hz_ = Tone(0);
        next_ = interval_;
    }

    float Tone(int k) const {
        static const float kZvei[11] = {2400.f, 1060.f, 1160.f, 1270.f, 1400.f, 1530.f,
                                        1670.f, 1830.f, 2000.f, 2200.f, 2600.f};
        static const float kCcir[11] = {1981.f, 1124.f, 1197.f, 1275.f, 1358.f, 1446.f,
                                        1540.f, 1640.f, 1747.f, 1860.f, 2400.f};
        int digit = k > 0 && digits_[k] == digits_[k - 1] ? 10 : digits_[k]; // 10: the repeat tone
        return ccir_ ? kCcir[digit] : kZvei[digit];
    }

    float sample_rate_ = 48000.f, phase_ = 0.f, tone_hz_ = 0.f;
    Random random_;
    int digits_[5] = {};
    bool ccir_ = false, on_ = false, was_on_ = false;
    int tone_ = -1, position_ = 0, length_ = 1, edge_ = 1, next_ = 0, interval_ = 1;
};

} // namespace bitch
