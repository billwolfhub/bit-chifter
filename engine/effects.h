#pragma once

#include <cstdint>

namespace bitch {

// The 30 pad effects of the strange pages (spec: 2026-10-13-bit-chifter-strange-pages-design.md),
// page by page, each page in the order of its pads from the big ones down (see PadEffect).
// Bit i of Params::effects turns Effect i on.
enum Effect : int {
    // Page 1: Shift & time
    kEchoes, kSpiralUp, kReverseEchoes, kSpiralDown, kPingPongEchoes,
    kChirp, kWarble, kSpinDown, kSpinUp, kFeedbackBurst,
    // Page 2: Radio
    kScrambler, kCarriers, kHoppingScrambler, kSelcall, kJammer,
    kStaticBurst, kSquelch, kFadeOut, kFilterSlam, kDriftMax,
    // Page 3: Digital & damage
    kBitcrush, kStutter, kRateCrush, kDropouts, kCodecWarble,
    kRing100, kCarrierX2, kKillDry, kChaos, kHoldDrift,
    kEffectCount
};

inline uint32_t Bit(Effect e) { return 1u << e; }
inline bool Has(uint32_t effects, Effect e) { return (effects >> e) & 1u; }

// The effect on pad `channel` (0-9, the MPR121 channels in the README's pad map) of `page`
// (0-2), or -1. The big pads (4, 6) get each page's first two effects, the narrow top
// centre (1, hard to reach) the last.
inline int PadEffect(int page, int channel) {
    static const int kPosition[10] = {5, 9, 6, 2, 0, 4, 1, 3, 7, 8}; // channel -> place in the page
    if (page < 0 || page > 2 || channel < 0 || channel > 9) return -1;
    return page * 10 + kPosition[channel];
}

// Short names for the render tool and reports
inline const char *EffectName(int e) {
    static const char *kNames[kEffectCount] = {
        "echoes",    "spiral_up", "reverse_echoes",    "spiral_down",  "ping_pong_echoes",
        "chirp",     "warble",    "spin_down",         "spin_up",      "feedback_burst",
        "scrambler", "carriers",  "hopping_scrambler", "selcall",      "jammer",
        "static_burst", "squelch", "fade_out",         "filter_slam",  "drift_max",
        "bitcrush",  "stutter",   "rate_crush",        "dropouts",     "codec_warble",
        "ring_100",  "carrier_x2", "kill_dry",         "chaos",        "hold_drift"};
    return e >= 0 && e < kEffectCount ? kNames[e] : "?";
}

} // namespace bitch
