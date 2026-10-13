#pragma once

#include <cmath>

namespace bitch {

// Knob and fader positions (0-1) to engine parameters. Shared by the Simple Touch
// front end, a future Patch front end and the tests.

// Fader -> the SSB dial: 0 in a small dead zone at the centre; the middle 80% of the
// travel is a fine, linear +-400 Hz (where mistuned sideband sweeps through its harmonic
// "snaps"); the last 10% at each end accelerates to +-2000 Hz for big swoops. The slope
// is continuous at the knee, so there's no flat spot.
inline float TuneHz(float fader) {
    const float kDeadZone = 0.02f, kFine = 0.8f, kFineHz = 400.f, kMaxHz = 2000.f;
    const float slope = kFineHz / kFine;                                                    // Hz per unit, fine zone
    const float curve = (kMaxHz - kFineHz - slope * (1.f - kFine)) / ((1.f - kFine) * (1.f - kFine));
    float x = 2.f * fader - 1.f;
    if (fabsf(x) < kDeadZone) return 0.f;
    float m = fminf((fabsf(x) - kDeadZone) / (1.f - kDeadZone), 1.f);
    float hz;
    if (m <= kFine) {
        hz = slope * m;
    } else {
        float d = m - kFine;
        hz = kFineHz + slope * d + curve * d * d;
    }
    return copysignf(hz, x);
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

// Fader -> reception amount 0-1, with the bottom 3% truly clean and the top 3% full
// (a fader rarely reads exactly 0 or 1)
inline float ReceptionAmount(float fader) { return fminf(fmaxf((fader - 0.03f) / 0.94f, 0.f), 1.f); }

} // namespace bitch
