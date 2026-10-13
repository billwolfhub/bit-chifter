#pragma once

#include <cstddef>

#include "radio_filter.h"
#include "ring_mod.h"
#include "shifter.h"

namespace bitch {

// How the two sides' shifts relate (cycled with the TOU logo pad)
enum class StereoMode { kLinked, kMirror, kRotate };

// Everything the front end controls (physical values; see mapping.h for knobs).
struct Params {
    float tune_hz = 0.f;      // shift, -2500 to +2500 Hz
    float feedback = 0.f;     // 0 to 0.95
    float mix = 1.f;          // dry/wet, 0 to 1
    float ring = 0.f;         // ring-mod amount, 0 to 1
    float carrier_hz = 100.f; // ring-mod carrier, 0.1 to 5000 Hz
    float filter_hz = 1000.f; // radio filter centre, 150 to 5000 Hz
    float filter_q = 0.7f;    // 0.7 (wide) to 12 (narrow)
    float filter_open = 1.f;  // 1 = filter bypassed
    StereoMode stereo_mode = StereoMode::kLinked;
};

// The sound engine: no hardware code, so it runs on the Daisy and on the Mac.
// True stereo: each side has its own ring mod -> shifter (with feedback) -> radio
// filter -> dry/wet -> soft clip, with shared controls. If one input is silent, the
// other feeds both sides at full level.
// Stereo modes: Linked (same shift both sides), Mirror (right gets the opposite shift),
// Rotate (over 8 s the sides' shifts sweep through each other and the pair circles left to right).
class Engine {
  public:
    void Init(float sample_rate);
    void Process(const float *in_l, const float *in_r, float *out_l, float *out_r, size_t size,
                 const Params &params);

  private:
    void Reset();

    RingMod ring_[2];
    Shifter shifter_[2];
    RadioFilter filter_[2];
    float shift_hz_ = 0.f;     // glided towards params.tune_hz
    float carrier_hz_ = 100.f; // glided towards params.carrier_hz
    float glide_ = 0.f;        // per-sample one-pole coefficient, ~20 ms
    float log2_filter_hz_ = 0.f, filter_q_ = 0.7f, filter_open_ = 1.f; // smoothed per block
    // Assumes a fixed block size (the coefficient is set from the first block's size)
    float block_glide_ = 0.f;  // per-block one-pole coefficient, ~30 ms
    float sample_rate_ = 48000.f;
    // Mono detection: input envelopes, and how much of the other side each chain takes
    float envelope_[2] = {0.f, 0.f};
    float take_other_[2] = {0.f, 0.f};
    float envelope_release_ = 0.f; // per sample, ~200 ms
    float take_glide_ = 0.f;       // per sample, ~50 ms
    float rotate_phase_ = 0.f; // 0-1 around the Rotate cycle
    float rotate_step_ = 0.f;  // per sample, one cycle per 8 s (advanced per block: times the block size)
    float side_gain_[2] = {1.f, 1.f}; // multiplies shift_hz_ per side, glided towards the mode's targets
    float swap_ = 0.f;                // 0 = sides as they are, 1 = fully swapped; glided
    float ring_amount_ = 0.f, mix_ = 1.f; // glided towards the clamped params
    int quiet_blocks_[2] = {0, 0};  // consecutive blocks with the input envelope below kQuiet
    bool active_[2] = {false, false}; // side counts as having a signal (with hold)
};

} // namespace bitch
