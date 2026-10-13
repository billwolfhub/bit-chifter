#pragma once

#include <cmath>

namespace bitch {

// Knob and fader positions (0-1) to engine parameters. Shared by the Simple Touch
// front end, a future Patch front end and the tests.

// Fader -> shift: -2500..+2500 Hz, 0 in a small dead zone at the centre, cubic so
// it's fine near 0 and wild at the ends
inline float TuneHz(float fader) {
    const float kMaxShiftHz = 2500.f, kDeadZone = 0.02f;
    float x = 2.f * fader - 1.f;
    if (fabsf(x) < kDeadZone) return 0.f;
    x = (x - copysignf(kDeadZone, x)) / (1.f - kDeadZone);
    return kMaxShiftHz * x * x * x;
}

// Knob -> ring-mod carrier, 0.1 Hz (tremolo) to 5 kHz (bells), exponential
inline float CarrierHz(float knob) { return 0.1f * powf(50000.f, knob); }

// Knob -> radio filter centre, 150 Hz to 5 kHz, exponential
inline float FilterHz(float knob) { return 150.f * powf(5000.f / 150.f, knob); }

// Knob -> radio filter width: narrow and resonant (Q 12) to wide (Q 0.7) over the
// first 85% of the turn, then crossfading to fully open (bypassed) between 85% and 97% of the turn
inline void FilterWidth(float knob, float &q, float &open) {
    float narrowing = fminf(knob / 0.85f, 1.f);
    q = 12.f * powf(0.7f / 12.f, narrowing);
    open = fminf(fmaxf((knob - 0.85f) / 0.12f, 0.f), 1.f); // fully open from 97% of the turn (a knob rarely reads exactly 1.0)
}

} // namespace bitch
