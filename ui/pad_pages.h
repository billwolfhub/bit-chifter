#pragma once

#include <cstdint>

#include "../engine/effects.h"

namespace bitch {

// The strange pages' pad logic, without hardware (tested on the Mac): call Update() every
// few milliseconds with the time, the touched pad channels (bit per MPR121 channel) and the
// page (0-2, from switch 1).
// - Pads 0-9: a tap (released within 0.3 s) latches the pad's effect on, or off if it was
//   latched; a hold (0.3 s or more) is momentary (off at the release, even if it was
//   latched). Either way the effect starts at the touch. A held pad keeps the effect of the
//   page it was touched on; latches stay on across page changes.
// - TOU (channel 10): a tap steps the stereo mode (counted on the release, so a hold
//   doesn't); holding it 1 s clears every latched effect (held pads stay on until released).
class PadPages {
  public:
    enum : uint32_t { kTapMs = 300, kClearMs = 1000 };
    enum : int { kPads = 10, kStereoPad = 10 };

    void Update(uint32_t now_ms, uint32_t touched, int page) {
        for (int ch = 0; ch < kPads; ch++) {
            bool on = (touched >> ch) & 1u, was = (was_touched_ >> ch) & 1u;
            if (on && !was) {
                int e = PadEffect(page, ch);
                if (e < 0) continue;
                effect_[ch] = e;
                down_ms_[ch] = now_ms;
                was_latched_[ch] = (latched_ >> e) & 1u;
                held_ |= 1u << e;
                touched_[ch] = true;
            } else if (!on && was && touched_[ch]) {
                uint32_t bit = 1u << effect_[ch];
                if (now_ms - down_ms_[ch] < kTapMs && !was_latched_[ch]) {
                    latched_ |= bit;
                } else {
                    latched_ &= ~bit;
                }
                held_ &= ~bit;
                touched_[ch] = false;
            }
        }
        bool on = (touched >> kStereoPad) & 1u, was = (was_touched_ >> kStereoPad) & 1u;
        if (on && !was) {
            stereo_down_ms_ = now_ms;
            stereo_cleared_ = false;
        } else if (on && !stereo_cleared_ && now_ms - stereo_down_ms_ >= kClearMs) {
            latched_ = 0;
            stereo_cleared_ = true;
            cleared_ = true;
        } else if (!on && was && !stereo_cleared_) {
            stereo_steps_++;
        }
        was_touched_ = touched;
    }

    uint32_t Effects() const { return latched_ | held_; } // bits for Params::effects
    uint32_t Latched() const { return latched_; }
    // Events since the last call
    int TakeStereoSteps() {
        int n = stereo_steps_;
        stereo_steps_ = 0;
        return n;
    }
    bool TakeCleared() {
        bool c = cleared_;
        cleared_ = false;
        return c;
    }

  private:
    uint32_t latched_ = 0, held_ = 0, was_touched_ = 0;
    uint32_t down_ms_[kPads] = {};
    int effect_[kPads] = {};          // the effect each touched pad turned on
    bool was_latched_[kPads] = {};    // that effect was latched when the pad was touched
    bool touched_[kPads] = {};        // the pad's touch was taken (it had an effect)
    uint32_t stereo_down_ms_ = 0;
    bool stereo_cleared_ = false;     // this TOU touch already cleared the latches
    int stereo_steps_ = 0;
    bool cleared_ = false;
};

} // namespace bitch
