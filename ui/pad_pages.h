#pragma once

#include <cstdint>

#include "../engine/effects.h"

namespace bitch {

// The strange pages' pad logic, without hardware (tested on the Mac): call Update() every
// few milliseconds with the time, the touched pad channels (bit per MPR121 channel) and the
// page (0-2, from switch 1).
// - Pads 0-9 are momentary: a pad's effect is on from the touch until the release. A held
//   pad keeps the effect of the page it was touched on. (No latching: playing them is
//   easier to experiment with.)
// - Chaos (page 3, bottom right): while its pad is held, a random set of 2-3 other effects is on
//   (not Fade-out or Squelch, which would only silence things); each touch rolls a new set.
// - TOU (channel 10): a tap (released within 0.3 s) steps the stereo mode, counted on the
//   release; a longer touch does nothing.
class PadPages {
  public:
    enum : uint32_t { kTapMs = 300 };
    enum : int { kPads = 10, kStereoPad = 10 };

    void Update(uint32_t now_ms, uint32_t touched, int page) {
        for (int ch = 0; ch < kPads; ch++) {
            bool on = (touched >> ch) & 1u, was = (was_touched_ >> ch) & 1u;
            if (on && !was) {
                effect_[ch] = PadEffect(page, ch);
                if (effect_[ch] >= 0) held_ |= 1u << effect_[ch];
                if (effect_[ch] == kChaos) chaos_ = RollChaos(now_ms);
            } else if (!on && was && effect_[ch] >= 0) {
                held_ &= ~(1u << effect_[ch]);
                if (effect_[ch] == kChaos) chaos_ = 0;
                effect_[ch] = -1;
            }
        }
        bool on = (touched >> kStereoPad) & 1u, was = (was_touched_ >> kStereoPad) & 1u;
        if (on && !was) stereo_down_ms_ = now_ms;
        if (!on && was && now_ms - stereo_down_ms_ < kTapMs) stereo_steps_++;
        was_touched_ = touched;
    }

    uint32_t Effects() const { return held_ | chaos_; } // bits for Params::effects
    // Stereo-mode steps since the last call
    int TakeStereoSteps() {
        int n = stereo_steps_;
        stereo_steps_ = 0;
        return n;
    }

  private:
    // 2-3 random effects other than Chaos, Fade-out and Squelch (the touch time stirs the generator)
    uint32_t RollChaos(uint32_t now_ms) {
        random_ ^= now_ms * 2654435761u;
        if (!random_) random_ = 1u;
        uint32_t set = 0;
        int count = 2 + (int)(NextRandom() % 2u);
        while (count > 0) {
            int e = (int)(NextRandom() % (uint32_t)kEffectCount);
            if (e == kChaos || e == kFadeOut || e == kSquelch || ((set >> e) & 1u)) continue;
            set |= 1u << e;
            count--;
        }
        return set;
    }
    uint32_t NextRandom() { // xorshift32
        random_ ^= random_ << 13;
        random_ ^= random_ >> 17;
        random_ ^= random_ << 5;
        return random_;
    }
    uint32_t chaos_ = 0, random_ = 0x9E3779B9u;
    uint32_t held_ = 0, was_touched_ = 0;
    int effect_[kPads] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1}; // each held pad's effect (-1: none)
    uint32_t stereo_down_ms_ = 0;
    int stereo_steps_ = 0;
};

} // namespace bitch
