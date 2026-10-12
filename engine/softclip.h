#pragma once

namespace bitch {

// tanh-like soft clip: slope 1 near 0, smoothly limits to +-1 (rational
// approximation, exact limit reached at +-3)
inline float SoftClip(float x) {
    if (x <= -3.f) return -1.f;
    if (x >= 3.f) return 1.f;
    float x2 = x * x;
    return x * (27.f + x2) / (27.f + 9.f * x2);
}

} // namespace bitch
