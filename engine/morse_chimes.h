#pragma once

#include <cmath>
#include <cstdint>

#include "carriers.h"
#include "effects.h"
#include "random.h"

namespace bitch {

// Morse chimes: 1 keyer (amount 0.35) to 3 (amount 1), each a carrier at a fixed random place
// within +-1.5 kHz of the dial (at least 200 Hz from 0; heard at its offset from the dial,
// like the passing carriers), keyed in random Morse-like letters at 12 (amount 0.35) to 25
// (amount 1) words per minute: dits (1 unit) and dahs (3 units), 1 unit apart; letters of
// 1-4 elements, 3 units apart; words of 2-5 letters, 7 units apart. Each key-down is a
// chime: a 3 ms attack, then a bell-like decay (120 ms), released in 20 ms at the key-up.
// About -18 dBFS per keyer. Update() once per block, Process() per sample.
class MorseChimes {
  public:
    enum : int { kMax = 3 };

    void Init(float sample_rate, float block_rate, uint32_t seed) {
        sample_rate_ = sample_rate;
        (void)block_rate;
        random_.Seed(seed);
        attack_ = 1.f / (0.003f * sample_rate);
        decay_ = expf(-1.f / (0.12f * sample_rate));
        release_ = expf(-1.f / (0.02f * sample_rate));
        for (int k = 0; k < kMax; k++) {
            Keyer &key = keyer_[k];
            float side = random_.Uniform() < 0.5f ? -1.f : 1.f;
            key.position = side * (200.f + 1300.f * random_.Uniform());
            key.rotor.Reset();
            key.envelope = 0.f;
            key.down = key.rising = false;
            key.left = (int)(random_.Uniform() * 0.3f * sample_rate); // start apart
            key.elements = key.letters = 0;
        }
    }

    static int Keyers(float amount) { return 1 + (int)lrintf(2.f * EffectRange(amount)); }
    float Position(int k) const { return keyer_[k].position; }

    void Update(float amount, float level, float dial_hz) {
        float u = EffectRange(amount);
        unit_ = (int)(1.2f / (12.f + 13.f * u) * sample_rate_);
        count_ = Keyers(amount);
        level_ = 0.125f * level;
        for (int k = 0; k < kMax; k++) keyer_[k].rotor.SetHz(keyer_[k].position - dial_hz, sample_rate_);
    }

    float Process() {
        float out = 0.f;
        for (int k = 0; k < kMax; k++) {
            Keyer &key = keyer_[k];
            if (--key.left <= 0) Advance(key, k < count_);
            if (key.down) {
                if (key.rising) {
                    key.envelope += attack_;
                    if (key.envelope >= 1.f) {
                        key.envelope = 1.f;
                        key.rising = false;
                    }
                } else {
                    key.envelope *= decay_;
                }
            } else {
                key.envelope *= release_;
                if (key.envelope < 1e-5f) key.envelope = 0.f;
            }
            if (key.envelope > 0.f) out += level_ * key.envelope * key.rotor.Next();
        }
        return out;
    }

  private:
    struct Keyer {
        float position = 0.f, envelope = 0.f;
        Rotor rotor;
        bool down = false, rising = false;
        int left = 0, elements = 0, letters = 0; // samples left in this state; elements / letters still to send
    };

    // The keyer's next state: an element, or a gap between elements, letters or words
    void Advance(Keyer &key, bool active) {
        if (key.down) { // the element ends: a gap
            key.down = false;
            if (--key.elements > 0) {
                key.left = unit_;
            } else if (--key.letters > 0) {
                key.left = 3 * unit_;
            } else {
                key.left = 7 * unit_;
            }
            return;
        }
        if (!active) { // this keyer is silent at this amount: check again later
            key.left = 7 * unit_;
            return;
        }
        if (key.letters <= 0) key.letters = 2 + (int)(random_.Uniform() * 4.f);
        if (key.elements <= 0) key.elements = 1 + (int)(random_.Uniform() * 4.f);
        key.down = key.rising = true;
        key.left = (random_.Uniform() < 0.5f ? 1 : 3) * unit_;
    }

    float sample_rate_ = 48000.f, attack_ = 0.f, decay_ = 0.f, release_ = 0.f, level_ = 0.f;
    int unit_ = 4800, count_ = 1;
    Random random_;
    Keyer keyer_[kMax];
};

} // namespace bitch
