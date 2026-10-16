#include "engine.h"

#include <cmath>

#include "agc.h"
#include "fading.h"
#include "softclip.h"

namespace bitch {

// Out-of-class definitions: the firmware builds as C++14, where odr-used static constexpr
// members need one (redundant but harmless in C++17)
constexpr float Hilbert::kA[4];
constexpr float Hilbert::kB[4];
constexpr int Fading::kSize;
constexpr int Fading::kMask;
constexpr float Agc::kTarget;
constexpr int PitchTracker::kDecimate;
constexpr int PitchTracker::kWindow;
constexpr int PitchTracker::kMinLag;
constexpr int PitchTracker::kMaxLag;
constexpr int PitchTracker::kFrame;
constexpr int PitchTracker::kLagsPerWork;
constexpr float PitchTracker::kThreshold;

static const float kSilent = 0.001f;  // -60 dBFS: above this an input counts as used at once
static const float kQuiet = 0.0005f;  // -66 dBFS: an input must stay below this for 3 s to count as unused

static float Clamp(float x, float lo, float hi) { return fminf(fmaxf(x, lo), hi); }

// The damage pads in chain order (damage_level_ follows it)
static const Effect kDamage[5] = {kCodecWarble, kRateCrush, kBitcrush, kStutter, kDropouts};

void Engine::Init(float sample_rate, size_t block_size, PlateReverb *plate, TapeMemory *tape) {
    sample_rate_ = sample_rate;
    plate_ = plate;
    tape_ = tape;
    float block_rate = sample_rate / (float)block_size;
    for (RingMod &r : ring_) r.Init(sample_rate);
    for (Shifter &s : shifter_) s.Init(sample_rate);
    for (RadioFilter &f : filter_) f.Init(sample_rate);
    const uint32_t kSeeds[2] = {11u, 29u}; // left and right wander on different paths
    for (int c = 0; c < 2; c++) {
        drift_[c].Init(block_rate, kSeeds[c]);
        fading_[c].Init(sample_rate, block_rate, kSeeds[c] * 13u + 101u);
        static_[c].Init(sample_rate, kSeeds[c] * 7u + 5u);
        agc_[c].Init(sample_rate);
    }
    for (Scrambler &s : scrambler_) s.Init(sample_rate, 83u); // the same seed: both sides hop together
    for (int c = 0; c < 2; c++) {
        carriers_[c].Init(sample_rate, block_rate, 3u); // same seed: the same stations both sides
        morse_[c].Init(sample_rate, block_rate, 9u);
    }
    selcall_.Init(sample_rate, 71u);
    jammer_.Init(sample_rate, 6u);
    codec_.Init(sample_rate, 13u);
    for (RateCrush &r : rate_crush_) r.Init(sample_rate);
    stutter_.Init(sample_rate, 17u);
    dropouts_.Init(sample_rate, 19u);
    if (tape_) capture_.Init(sample_rate, tape_->capture[0], tape_->capture[1]);
    hand_.Init(block_rate, 47u);
    pitch_.Init(sample_rate);
    squelch_attack_ = 1.f / (0.002f * sample_rate);
    squelch_decay_ = expf(-1.f / (0.15f * sample_rate));
    squelch_noise_.Seed(61u);
    if (plate_) plate_->Init(sample_rate);
    glide_ = 1.f - expf(-1.f / (0.02f * sample_rate));
    envelope_release_ = expf(-1.f / (0.2f * sample_rate));
    take_glide_ = 1.f - expf(-1.f / (0.05f * sample_rate));
    rotate_step_ = 1.f / (8.f * sample_rate);
    Reset();
    plate_clean_ = true; // plate_->Init() has just cleared it in full (outside the audio callback)
}

void Engine::Reset() {
    for (RingMod &r : ring_) r.Reset();
    for (Shifter &s : shifter_) s.Reset();
    for (RadioFilter &f : filter_) f.Reset();
    shift_hz_ = 0.f;
    carrier_hz_ = 100.f;
    log2_filter_hz_ = log2f(1000.f);
    filter_q_ = 0.7f;
    filter_open_ = 1.f;
    block_glide_ = 0.f; // set on the first block (depends on the block size)
    detent_release_ = 0.f;
    rotate_phase_ = 0.f;
    side_gain_[0] = side_gain_[1] = 1.f;
    swap_ = 0.f;
    ring_amount_ = 0.f;
    mix_ = 1.f;
    quiet_blocks_[0] = quiet_blocks_[1] = 0;
    active_[0] = active_[1] = false;
    envelope_[0] = envelope_[1] = 0.f;
    take_other_[0] = take_other_[1] = 0.f;
    reception_ = 0.f;
    for (Fading &f : fading_) f.Reset();
    for (Agc &a : agc_) a.Reset();
    for (StaticNoise &n : static_) n.Reset();
    dial_[0] = dial_[1] = 0.f;
    detent_amount_ = 0.f;
    detent_pitch_ = 0.f;
    target_shift_ = 0.f;
    pitch_.Reset();
    // The plate isn't cleared here (too much for one callback: this is also the NaN recovery);
    // it is cleared in steps over the next blocks, before it can start again
    reverb_ = 0.f;
    plate_running_ = false;
    plate_clean_ = false;
    for (float &v : level_) v = 0.f;
    drift_hz_[0] = drift_hz_[1] = 0.f;
    hand_hz_ = 0.f;
    fade_db_ = 0.f;
    squelch_tail_ = 0.f;
    squelch_rising_ = false;
    squelch_was_on_ = false;
    burst_level_ = squelch_level_ = 0.f;
    spin_hz_ = 0.f;
    for (Scrambler &s : scrambler_) s.Reset();
    scrambler_running_ = false;
    for (Carriers &c : carriers_) c.Reset();
    for (MorseChimes &m : morse_) m.Reset();
    selcall_.Stop();
    jammer_.Reset();
    scramble_level_ = selcall_level_ = 0.f;
    codec_.Reset();
    for (RateCrush &r : rate_crush_) r.Reset();
    stutter_.Reset();
    dropouts_.Reset();
    for (float &v : damage_level_) v = 0.f;
    if (tape_) capture_.Reset();
}

void Engine::Process(const float *in_l, const float *in_r, float *out_l, float *out_r, size_t size,
                     const Params &params) {
    if (size == 0) return;
    float tune = Clamp(params.tune_hz, -2500.f, 2500.f);
    float feedback = Clamp(params.feedback, 0.f, 0.95f);
    float mix = Clamp(params.mix, 0.f, 1.f);
    float ring = Clamp(params.ring, 0.f, 1.f);
    float carrier = Clamp(params.carrier_hz, 0.1f, 5000.f);

    // Filter settings change once per block, smoothed (the centre in octaves)
    if (block_glide_ == 0.f) {
        block_glide_ = 1.f - expf(-(float)size / (0.03f * sample_rate_));
        detent_release_ = 1.f - expf(-(float)size / (0.15f * sample_rate_));
        spin_return_ = 1.f - expf(-(float)size / (0.3f * sample_rate_));
        log2_filter_hz_ = log2f(Clamp(params.filter_hz, 150.f, 5000.f));
        filter_q_ = Clamp(params.filter_q, 0.7f, 12.f);
        filter_open_ = Clamp(params.filter_open, 0.f, 1.f);
        reception_ = Clamp(params.reception, 0.f, 1.f);
        dial_[0] = dial_[1] = tune;
        for (int k = 0; k < kEffectCount; k++) level_[k] = (params.effects >> k) & 1u ? 1.f : 0.f;
        burst_level_ = level_[kFeedbackBurst];
        squelch_level_ = level_[kSquelch];
        scramble_level_ = fmaxf(level_[kScrambler], level_[kHoppingScrambler]);
        selcall_level_ = level_[kSelcall];
        for (int k = 0; k < 5; k++) damage_level_[k] = level_[kDamage[k]];
    }
    // Pad effects: each fades in and out over ~30 ms
    const uint32_t fx = params.effects;
    for (int k = 0; k < kEffectCount; k++) {
        float target = (fx >> k) & 1u ? 1.f : 0.f;
        level_[k] += block_glide_ * (target - level_[k]);
        if (fabsf(target - level_[k]) < 1e-4f) level_[k] = target;
    }
    // The parameter effects (ring, carrier and mix are glided per sample below, the filter per
    // block; the feedback burst is ramped across the block below)
    if (Has(fx, kRing100)) ring = 1.f;
    if (Has(fx, kCarrierX2)) carrier *= 2.f; // up to 10 kHz (aliasing at the top is part of the effect)
    if (Has(fx, kKillDry)) mix = 1.f;
    float q_target = Clamp(params.filter_q, 0.7f, 12.f), open_target = Clamp(params.filter_open, 0.f, 1.f);
    if (Has(fx, kFilterSlam)) {
        q_target = 12.f; // narrowest and most resonant
        open_target = 0.f;
    }
    log2_filter_hz_ += block_glide_ * (log2f(Clamp(params.filter_hz, 150.f, 5000.f)) - log2_filter_hz_);
    filter_q_ += block_glide_ * (q_target - filter_q_);
    filter_open_ += block_glide_ * (open_target - filter_open_);
    float filter_hz = exp2f(log2_filter_hz_);
    for (RadioFilter &f : filter_) f.Set(filter_hz, filter_q_, filter_open_);

    // Reception: one smoothed amount for every receiver effect
    reception_ += block_glide_ * (Clamp(params.reception, 0.f, 1.f) - reception_);
    float r = reception_ < 1e-4f ? 0.f : reception_;
    drive_.Set(r);
    // Static burst raises the static alone; Hold drift freezes the drift and the tuning hand
    // where they are
    float static_level = r + level_[kStaticBurst] * (1.f - r);
    const bool hold = Has(fx, kHoldDrift);
    for (int c = 0; c < 2; c++) {
        if (!hold)
            drift_hz_[c] = r > 0.f ? r * 40.f * drift_[c].Next(0.1f + 0.4f * r) : 0.f;
        fading_[c].Update(r);
        static_[c].Update(static_level);
        agc_[c].Update(r);
    }
    // Signal fade-out: towards -40 dB over 1.5 s while on, back at the same rate. It fades the
    // input (dry and wet alike, at any MIX); the static etc. stay
    float fade_step = 40.f / 1.5f * (float)size / sample_rate_;
    fade_db_ = Has(fx, kFadeOut) ? fmaxf(fade_db_ - fade_step, -40.f) : fminf(fade_db_ + fade_step, 0.f);
    float signal_gain = fade_db_ == 0.f ? 1.f : powf(10.f, 0.05f * fade_db_);
    // Squelch: the wet signal mutes; as it opens, a short burst of noise into the receiver (so
    // the radio filter shapes it like its own noise)
    const bool squelch = Has(fx, kSquelch);
    if (squelch_was_on_ && !squelch) squelch_rising_ = true;
    squelch_was_on_ = squelch;
    // The feedback burst and the squelch's mute, ramped across the block from the last one's
    // levels (with neither on, both ramps are constant 0: the numbers are as without them)
    const float burst_from = burst_level_, burst_to = level_[kFeedbackBurst];
    const float squelch_from = squelch_level_, squelch_to = level_[kSquelch];
    burst_level_ = burst_to;
    squelch_level_ = squelch_to;
    const float inv_size = 1.f / (float)size;

    // The SSB dial, once per block: TUNE through the inertia, plus the tuning hand, then
    // pulled towards multiples of the source's pitch (strength x confidence, smoothed)
    float dial = tune;
    if (params.dial_inertia_s > 0.f) {
        float k = 1.f - expf(-(float)size / (params.dial_inertia_s * sample_rate_));
        dial_[0] += k * (tune - dial_[0]);
        dial_[1] += k * (dial_[0] - dial_[1]);
        dial = dial_[1];
    } else {
        dial_[0] = dial_[1] = tune;
    }
    if (!hold) hand_hz_ = r > 0.f ? r * 150.f * hand_.Next(0.15f + 0.15f * r) : 0.f;
    float detent_target = Clamp(params.detent, 0.f, 0.95f) * pitch_.Confidence();
    // Rises in ~30 ms, falls in ~150 ms, so one analysis without a pitch doesn't pump the detents
    float detent_glide = detent_target > detent_amount_ ? block_glide_ : detent_release_;
    detent_amount_ += detent_glide * (detent_target - detent_amount_);
    if (detent_amount_ < 1e-4f && detent_target == 0.f) detent_amount_ = 0.f;
    // The pull's pitch follows the tracker only for a pitch at least as sure as the current
    // pull, so a weak (often wrong) estimate can't steer the detents while they release
    if (detent_target >= detent_amount_) detent_pitch_ = pitch_.Pitch();
    // Dial spin: the dial keeps turning, 150 Hz/s (amount 0.35) to 3000 Hz/s (amount 1),
    // within +-2 kHz in all (TUNE + tuning hand + spin; the spin stops at the limit); after the
    // pad is released it comes back in ~1 s. Both on: back. The detents act on the spun dial too.
    float amount = Clamp(params.effect_amount, 0.35f, 1.f);
    float spin_rate = 150.f + 2850.f * (amount - 0.35f) / 0.65f;
    float spin_dir = (Has(fx, kSpinUp) ? 1.f : 0.f) - (Has(fx, kSpinDown) ? 1.f : 0.f);
    if (spin_dir != 0.f) {
        spin_hz_ += spin_dir * spin_rate * (float)size / sample_rate_;
    } else {
        spin_hz_ -= spin_return_ * spin_hz_;
        if (fabsf(spin_hz_) < 0.01f) spin_hz_ = 0.f;
    }
    float turned = dial + hand_hz_;
    // (turned can reach +-2150 with the tuning hand: clamp it for the limits, so they never cross)
    float limit_turned = Clamp(turned, -2000.f, 2000.f);
    spin_hz_ = Clamp(spin_hz_, -2000.f - limit_turned, 2000.f - limit_turned);
    target_shift_ = SoftQuantize(turned + spin_hz_, detent_pitch_, detent_amount_);

    // Radio pads: the scrambler (either pad), run only while it's audible (its level ramped
    // across the block). The hops go on through the Hopping pad's release, unless the plain pad
    // is held
    float amount_radio = Clamp(params.effect_amount, 0.35f, 1.f);
    const float scramble_from = scramble_level_, scramble_to = fmaxf(level_[kScrambler], level_[kHoppingScrambler]);
    scramble_level_ = scramble_to;
    const bool scrambling = scramble_from > 0.f || scramble_to > 0.f;
    // Hopping while its pad is held (with or without the plain pad), and through its fade-out
    // unless the plain pad is still held
    const bool hopping = Has(fx, kHoppingScrambler) || (level_[kHoppingScrambler] > 0.f && !Has(fx, kScrambler));
    if (scrambling) {
        if (!scrambler_running_) {
            for (Scrambler &s : scrambler_) s.Reset();
            scrambler_running_ = true;
        }
        for (Scrambler &s : scrambler_) s.Update(amount_radio, hopping, (float)size / sample_rate_);
    } else {
        scrambler_running_ = false;
    }
    // The other radio pads, run only while audible: the stations are heard against each side's
    // dial (the shift as it is now, with the side's stereo gain and drift). Selcall's level is
    // ramped across the block here; the others ramp their own gains
    const float level_carriers = level_[kCarriers], level_morse = level_[kMorseChimes];
    const float selcall_from = selcall_level_, selcall_to = level_[kSelcall], level_jammer = level_[kJammer];
    selcall_level_ = selcall_to;
    const bool calling = selcall_from > 0.f || selcall_to > 0.f;
    for (int c = 0; c < 2; c++) {
        float side_dial = shift_hz_ * side_gain_[c] + drift_hz_[c];
        if (level_carriers > 0.f) carriers_[c].Update(amount_radio, level_carriers, side_dial);
        if (level_morse > 0.f) morse_[c].Update(amount_radio, level_morse, side_dial);
    }
    if (calling || Has(fx, kSelcall)) {
        selcall_.Update(Has(fx, kSelcall), amount_radio);
    } else {
        selcall_.Stop();
    }
    if (level_jammer > 0.f) jammer_.Update(amount_radio, level_jammer, (float)size / sample_rate_);
    // Digital damage pads: run only while audible (reset as each one starts), their levels ramped
    // across the block
    float damage_from[5], damage_to[5];
    bool damaging[5];
    for (int k = 0; k < 5; k++) {
        damage_from[k] = damage_level_[k];
        damage_to[k] = level_[kDamage[k]];
        damage_level_[k] = damage_to[k];
        damaging[k] = damage_from[k] > 0.f || damage_to[k] > 0.f;
    }
    const bool starting[5] = {damaging[0] && damage_from[0] == 0.f, damaging[1] && damage_from[1] == 0.f,
                              false, damaging[3] && damage_from[3] == 0.f, damaging[4] && damage_from[4] == 0.f};
    if (starting[0]) codec_.Reset();
    if (starting[1]) for (RateCrush &rc : rate_crush_) rc.Reset();
    if (starting[3]) stutter_.Reset();
    if (starting[4]) dropouts_.Reset();
    if (damaging[0]) codec_.Update(amount_radio, (int)size);
    if (damaging[1]) for (RateCrush &rc : rate_crush_) rc.Update(amount_radio);
    const float crush_step = CrushStep(CrushBits(amount_radio)), crush_inv_step = 1.f / crush_step;
    if (damaging[3]) stutter_.Update(amount_radio, (float)size / sample_rate_);
    if (damaging[4]) dropouts_.Update(amount_radio);
    if (tape_) capture_.Set(params.capture);

    // The plate: idle (and skipped) at 0 or without a plate. While idle it is cleared in steps
    // (1024 floats a block, ~40 ms for all of it), and it starts again only once clean, so an
    // old tail never replays (until then reverb_ stays at 0, and the glide starts from there)
    float reverb = plate_ ? Clamp(params.reverb, 0.f, 1.f) : 0.f;
    if (plate_ && !plate_running_ && !plate_clean_) plate_clean_ = plate_->ClearStep(1024);
    if (reverb > 0.f && !plate_running_ && plate_clean_) plate_running_ = true;
    if (plate_running_ && reverb == 0.f && reverb_ < 1e-4f) {
        reverb_ = 0.f;
        plate_running_ = false;
        plate_clean_ = false;
    }
    if (plate_running_) plate_->SetDecay(reverb_);

    // Mono detection: a side whose input is silent takes the other side's input instead
    float take_target[2] = {active_[1] && !active_[0] ? 1.f : 0.f, active_[0] && !active_[1] ? 1.f : 0.f};

    // Stereo mode: per-block targets for each side's share of the shift and for the left/right
    // swap (glided per sample below, so mode changes don't click)
    float gain_target[2] = {1.f, 1.f}, swap_target = 0.f;
    if (params.stereo_mode == StereoMode::kMirror) {
        gain_target[1] = -1.f;
    } else if (params.stereo_mode == StereoMode::kRotate) {
        rotate_phase_ += rotate_step_ * (float)size;
        rotate_phase_ -= floorf(rotate_phase_);
        float angle = 6.28318530718f * rotate_phase_;
        float sweep = cosf(angle); // left's shift goes +shift .. -shift, right's the other way
        gain_target[0] = sweep;
        gain_target[1] = -sweep;
        swap_target = 0.5f + 0.5f * sinf(angle); // the pair circles from left to right
    } else {
        // Linked: keep the two oscillators in phase (they are left at an offset after Mirror/Rotate).
        // Pulling the right one gently (<1 Hz deviation) converges in ~100 ms and is inaudible.
        // The lock fades out as the drift rises with reception (gone above 0.05) so the sides can
        // drift apart; it re-locks within ~100 ms once the drift is back at 0. Off while Hold drift
        // keeps a drift (even with reception turned down).
        float lock = 0.02f * fmaxf(1.f - r * 20.f, 0.f);
        if (hold && (drift_hz_[0] != 0.f || drift_hz_[1] != 0.f)) lock = 0.f;
        if (lock > 0.f) {
            float err = shifter_[0].Phase() - shifter_[1].Phase();
            err -= floorf(err + 0.5f); // wrapped to -0.5..0.5
            // Within rounding of in-phase: close the gap fully, since 2% of it is below float resolution
            shifter_[1].NudgePhase(fabsf(err) < 1e-5f ? err : lock * err);
        }
    }

    for (size_t i = 0; i < size; i++) {
        float x[2] = {in_l[i], in_r[i]};
        if (tape_) capture_.Process(x[0], x[1]); // switch 2: everything after hears the captured sound
        pitch_.Push(0.5f * (x[0] + x[1]));
        for (int c = 0; c < 2; c++) envelope_[c] = fmaxf(fabsf(x[c]), envelope_[c] * envelope_release_);
        shift_hz_ += glide_ * (target_shift_ - shift_hz_);
        carrier_hz_ += glide_ * (carrier - carrier_hz_);
        ring_amount_ += glide_ * (ring - ring_amount_);
        mix_ += glide_ * (mix - mix_);
        swap_ += glide_ * (swap_target - swap_);
        float shift[2];
        for (int c = 0; c < 2; c++) {
            side_gain_[c] += glide_ * (gain_target[c] - side_gain_[c]);
            shift[c] = shift_hz_ * side_gain_[c];
        }
        float t = (float)(i + 1) * inv_size;
        float burst = burst_from + t * (burst_to - burst_from);
        float fb = feedback + burst * (0.95f - feedback);
        float wet_gain = 1.f - (squelch_from + t * (squelch_to - squelch_from));
        float squelch_noise = 0.f;
        if (squelch_rising_ || squelch_tail_ > 0.f) {
            if (squelch_rising_) {
                squelch_tail_ += squelch_attack_;
                if (squelch_tail_ >= 1.f) {
                    squelch_tail_ = 1.f;
                    squelch_rising_ = false;
                }
            } else {
                squelch_tail_ *= squelch_decay_;
                if (squelch_tail_ < 1e-4f) squelch_tail_ = 0.f;
            }
            squelch_noise = 0.25f * squelch_tail_;
        }
        float call = calling ? (selcall_from + t * (selcall_to - selcall_from)) * selcall_.Process() : 0.f;
        float jam = level_jammer > 0.f ? jammer_.Process() : 0.f;
        const float scramble = scramble_from + t * (scramble_to - scramble_from);
        float y[2], dry[2], filtered[2];
        for (int c = 0; c < 2; c++) {
            take_other_[c] += take_glide_ * (take_target[c] - take_other_[c]);
            dry[c] = signal_gain * (x[c] + take_other_[c] * x[1 - c]) + call;
            float driven = drive_.Process(dry[c]);
            if (scrambling) driven += scramble * (scrambler_[c].Process(driven) - driven);
            float ringed = ring_[c].Process(driven, carrier_hz_, ring_amount_);
            float shifted = shifter_[c].Process(ringed, shift[c] + drift_hz_[c], fb);
            float received = fading_[c].Process(shifted) + static_[c].Process();
            if (level_carriers > 0.f) received += carriers_[c].Process();
            if (level_morse > 0.f) received += morse_[c].Process();
            received += jam;
            if (squelch_noise > 0.f) received += squelch_noise * squelch_noise_.Bipolar();
            filtered[c] = filter_[c].Process(received);
        }
        for (int c = 0; c < 2; c++) {
            float wet = wet_gain * agc_[c].Process(filtered[c]);
            y[c] = dry[c] + mix_ * (wet - dry[c]);
        }
        float left = y[0] + swap_ * (y[1] - y[0]), right = y[1] + swap_ * (y[0] - y[1]);
        if (plate_running_) {
            reverb_ += glide_ * (reverb - reverb_);
            float tail_l, tail_r;
            plate_->Process(left, right, tail_l, tail_r);
            left += 0.5f * reverb_ * tail_l; // up to -6 dB
            right += 0.5f * reverb_ * tail_r;
        }
        // Digital damage (page 3), on the whole sound: after the dry/wet mix and the reverb, so it
        // bites at any MIX (like a broken stream, it breaks everything you hear)
        float out[2] = {left, right};
        float d[5];
        for (int k = 0; k < 5; k++) d[k] = damaging[k] ? damage_from[k] + t * (damage_to[k] - damage_from[k]) : 0.f;
        if (damaging[0]) {
            float side_l = out[0], side_r = out[1];
            codec_.Process(side_l, side_r);
            out[0] += d[0] * (side_l - out[0]);
            out[1] += d[0] * (side_r - out[1]);
        }
        for (int c = 0; c < 2; c++) {
            if (damaging[1]) out[c] += d[1] * (rate_crush_[c].Process(out[c]) - out[c]);
            if (damaging[2]) out[c] += d[2] * (Bitcrush(out[c], crush_step, crush_inv_step) - out[c]);
        }
        if (damaging[3]) {
            float side_l = out[0], side_r = out[1];
            stutter_.Process(side_l, side_r);
            out[0] += d[3] * (side_l - out[0]);
            out[1] += d[3] * (side_r - out[1]);
        }
        if (damaging[4]) {
            float drop = 1.f - d[4] * (1.f - dropouts_.Gain());
            out[0] *= drop;
            out[1] *= drop;
        }
        out_l[i] = SoftClip(out[0]);
        out_r[i] = SoftClip(out[1]);
    }
    pitch_.Work();
    // Hold: a side counts as active at once when its envelope passes kSilent, and as unused
    // only after it has stayed below kQuiet for 3 s
    const int hold_blocks = (int)(3.f * sample_rate_ / (float)size);
    for (int c = 0; c < 2; c++) {
        if (envelope_[c] > kSilent) active_[c] = true;
        if (envelope_[c] > kQuiet) {
            quiet_blocks_[c] = 0;
        } else if (++quiet_blocks_[c] >= hold_blocks) {
            active_[c] = false;
            quiet_blocks_[c] = hold_blocks;
        }
    }
    // A NaN/Inf would stay in the feedback loop and silence everything: reset instead
    if (!std::isfinite(out_l[size - 1]) || !std::isfinite(out_r[size - 1])) {
        Reset();
        for (size_t i = 0; i < size; i++) out_l[i] = out_r[i] = 0.f;
    }
}

} // namespace bitch
