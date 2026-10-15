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
            } else if (!on && was && effect_[ch] >= 0) {
                held_ &= ~(1u << effect_[ch]);
                effect_[ch] = -1;
            }
        }
        bool on = (touched >> kStereoPad) & 1u, was = (was_touched_ >> kStereoPad) & 1u;
        if (on && !was) stereo_down_ms_ = now_ms;
        if (!on && was && now_ms - stereo_down_ms_ < kTapMs) stereo_steps_++;
        was_touched_ = touched;
    }

    uint32_t Effects() const { return held_; } // bits for Params::effects
    // Stereo-mode steps since the last call
    int TakeStereoSteps() {
        int n = stereo_steps_;
        stereo_steps_ = 0;
        return n;
    }

  private:
    uint32_t held_ = 0, was_touched_ = 0;
    int effect_[kPads] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1}; // each held pad's effect (-1: none)
    uint32_t stereo_down_ms_ = 0;
    int stereo_steps_ = 0;
};

} // namespace bitch
