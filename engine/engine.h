#pragma once

#include <cstddef>

#include "radio_filter.h"
#include "ring_mod.h"
#include "shifter.h"
#include "agc.h"
#include "drive.h"
#include "fading.h"
#include "random.h"
#include "static_noise.h"
#include "detents.h"
#include "pitch_tracker.h"
#include "plate_reverb.h"
#include "effects.h"

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
    float reception = 0.f;    // 0 clean .. 1 lost in the noise (drift, fading, static, AGC, drive)
    float detent = 0.f;          // harmonic detent strength 0-0.95 (0 = off; the firmware uses 0.7)
    float dial_inertia_s = 0.f;  // TUNE inertia, per stage of two (0 = off; the firmware uses 0.06)
    float reverb = 0.f;          // plate reverb 0-1: return level (up to -6 dB) and decay (0 = off)
    uint32_t effects = 0;        // pad effects: bit i = Effect i (effects.h); 0 = none
    float effect_amount = 0.35f; // how strong, 0.35-1 (the front end passes EffectAmount(reception))
};

// The sound engine: no hardware code, so it runs on the Daisy and on the Mac.
// True stereo: each side has its own ring mod -> shifter (with feedback) -> radio
// filter -> dry/wet -> soft clip, with shared controls. If one input is silent, the
// other feeds both sides at full level.
// Stereo modes: Linked (same shift both sides), Mirror (right gets the opposite shift),
// Rotate (over 8 s the sides' shifts sweep through each other and the pair circles left to right).
// Reception (0-1) adds, per side: drive -> ring mod -> shifter (+ its own drift, up to ±40 Hz) -> fading ->
// + static -> radio filter -> AGC. At 0 every stage is an exact pass-through.
// The shift (TUNE) goes through optional inertia, the tuning hand (reception x ±150 Hz) and the
// harmonic detents (pulled onto multiples of the tracked pitch) once per block.
// A light stereo plate reverb (Params::reverb) is added after the dry/wet mix, before the soft clip.
// Pad effects (Params::effects, see effects.h) each fade in and out over ~30 ms; with none on, the sound is unchanged.
class Engine {
  public:
    // block_size: the size every Process() call will use (the firmware and sim use 48).
    // plate: the reverb is ~145 KB, so the caller provides it (or none: then Params::reverb is ignored)
    void Init(float sample_rate, size_t block_size = 48, PlateReverb *plate = nullptr);
    // Test hook: a side's shifter oscillator phase (0-1)
    float ShifterPhase(int side) const { return shifter_[side].Phase(); }
    // The source's tracked fundamental, Hz (the last one found: kept when the confidence drops to 0)
    float Pitch() const { return pitch_.Pitch(); }
    float PitchConfidence() const { return pitch_.Confidence(); } // 0-1
    void Process(const float *in_l, const float *in_r, float *out_l, float *out_r, size_t size,
                 const Params &params);

  private:
    void Reset();

    RingMod ring_[2];
    Shifter shifter_[2];
    RadioFilter filter_[2];
    float shift_hz_ = 0.f;     // glided towards target_shift_ (the dial)
    float carrier_hz_ = 100.f; // glided towards params.carrier_hz
    float glide_ = 0.f;        // per-sample one-pole coefficient, ~20 ms
    float log2_filter_hz_ = 0.f, filter_q_ = 0.7f, filter_open_ = 1.f; // smoothed per block
    // Assumes a fixed block size (the coefficient is set from the first block's size)
    float block_glide_ = 0.f;  // per-block one-pole coefficient, ~30 ms
    float detent_release_ = 0.f; // per-block one-pole coefficient, ~150 ms (detent_amount_ falling)
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
    // Reception (fader 2): one amount drives every receiver effect, smoothed per block
    float reception_ = 0.f;
    Drive drive_;
    Wander drift_[2];          // each side's shift wanders on its own path
    Fading fading_[2];
    StaticNoise static_[2];
    Agc agc_[2];
    // The SSB dial: inertia on TUNE, the tuning hand, and the harmonic detents
    float dial_[2] = {0.f, 0.f}; // two-stage inertia
    Wander hand_;                // wanders the dial with reception (shared by both sides)
    PitchTracker pitch_;
    float detent_amount_ = 0.f;  // strength x confidence, smoothed per block
    float detent_pitch_ = 0.f;   // the pitch the detents pull towards (held while they release)
    float target_shift_ = 0.f;   // the dial's shift for this block; shift_hz_ glides to it
    // The plate reverb, after the mix (fed by both sides, returning a wide stereo tail)
    PlateReverb *plate_ = nullptr; // optional; owned by the front end (the firmware keeps it in SDRAM)
    float reverb_ = 0.f;          // glided towards params.reverb
    bool plate_running_ = false;  // false: the plate is idle (and cleared in steps before it starts again)
    bool plate_clean_ = true;     // the idle plate's buffers are all zero (it may start)
    // Pad effects: each one's level 0-1, faded per block (~30 ms)
    float level_[kEffectCount] = {};
};

} // namespace bitch
