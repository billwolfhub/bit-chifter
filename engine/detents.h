#pragma once

#include <cmath>

namespace bitch {

// Harmonic detents: a mistuned sideband signal is harmonic again whenever the shift is a
// multiple of the source's pitch (all components land on a new harmonic series). This
// soft-quantises the shift towards those multiples: continuous and always increasing
// (for amount < 1), so a sweep passes straight through, but flattened around each
// multiple, so the sound lingers there. amount 0 (or no pitch) leaves the shift as it is.
inline float SoftQuantize(float shift, float step, float amount) {
    if (!(step > 1.f) || !(amount > 0.f)) return shift;
    float a = fminf(amount, 0.95f);
    const float kTwoPi = 6.28318530718f;
    return shift - a * step * (1.f / kTwoPi) * sinf(kTwoPi * shift / step);
}

} // namespace bitch
