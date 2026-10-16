#pragma once

#include <cmath>
#include <cstdint>

#include "effects.h"
#include "random.h"

namespace bitch {

// Stutter: now and then catches the last 30-120 ms and repeats it 2-8 times before letting the
// signal through again. A stutter starts 0.5 (amount 0.35) to 4 (amount 1) times a second on
// average (while not already repeating). Each repeat is faded in and out over 2 ms against the
// live signal, so the loop points don't click. Stereo: both sides repeat together. Assumes
// 48 kHz (the buffer holds 120 ms). Update() once per block, Process() per sample.
class Stutter {
  public:
    enum : int { kMax = 5760 }; // 120 ms at 48 kHz

    void Init(float sample_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        fade_ = (int)(0.002f * sample_rate);
        random_.Seed(seed);
        Reset();
    }

    void Reset() {
        write_ = filled_ = 0;
        repeats_left_ = 0;
        rate_ = 0.5f;
    }

    void Update(float amount, float block_seconds) {
        rate_ = 0.5f + 3.5f * EffectRange(amount);
        if (repeats_left_ > 0 || random_.Uniform() >= rate_ * block_seconds) return;
        int length = (int)((0.03f + 0.09f * random_.Uniform()) * sample_rate_);
        if (length > kMax) length = kMax;
        if (filled_ < length) return; // not enough recorded yet
        length_ = length;
        start_ = write_ - length;
        if (start_ < 0) start_ += kMax;
        position_ = 0;
        repeats_left_ = 2 + (int)(random_.Uniform() * 7.f); // 2-8
    }

    void Process(float &l, float &r) {
        if (repeats_left_ <= 0) { // recording (only while not repeating, so the fragment stays intact)
            buffer_[0][write_] = l;
            buffer_[1][write_] = r;
            if (++write_ >= kMax) write_ = 0;
            if (filled_ < kMax) filled_++;
            return;
        }
        int read = start_ + position_;
        if (read >= kMax) read -= kMax;
        float g = fminf(fminf((float)position_, (float)(length_ - position_)) / (float)fade_, 1.f);
        l += g * (buffer_[0][read] - l);
        r += g * (buffer_[1][read] - r);
        if (++position_ >= length_) {
            position_ = 0;
            repeats_left_--;
        }
    }

    bool Repeating() const { return repeats_left_ > 0; }

  private:
    float sample_rate_ = 48000.f, rate_ = 0.5f;
    int fade_ = 96, write_ = 0, filled_ = 0, start_ = 0, length_ = 1, position_ = 0, repeats_left_ = 0;
    float buffer_[2][kMax] = {};
    Random random_;
};

} // namespace bitch
