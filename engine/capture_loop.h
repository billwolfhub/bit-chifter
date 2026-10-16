#pragma once

#include <cmath>
#include <cstdlib>

#include "random.h"

namespace bitch {

// Switch 2: Normal (live input), Freeze or Loop
enum class CaptureMode { kNormal, kFreeze, kLoop };

// Records the input (2 s, stereo, into buffers the caller provides: they live in SDRAM) while
// Normal; on Freeze, holds the last 250 ms as an endless drone; on Loop, repeats everything
// recorded since the last Normal (up to 2 s) with a 20 ms crossfade at the loop point.
//
// Freeze: three read heads, a third of a 125 ms grain apart, each with a sin^2 window (the
// windows sum to a constant 1.5, scaled by 2/3). Whenever a head's window comes back to 0 it
// jumps to a new place in the slice: a random one, refined (by a cross-correlation search over
// ~10 ms, run one candidate per sample in the ~6 ms before the jump) to the nearby place that
// best matches what the head it overlaps will play, so the heads stay in phase (a held tone
// doesn't comb or flutter) with no fixed repetition.
//
// Recording stops while Freeze or Loop plays, and moving between them plays the same capture,
// crossfading directly from one to the other over 30 ms (Loop starts where the loudest freeze
// head is reading; Freeze starts, ~5 ms after the switch, at the place in its slice that best
// matches the loop). To and from Normal it crossfades against the live input over 30 ms.
// Returning to Normal starts a new recording, so a capture only ever uses contiguous audio, and
// nothing is captured until a full slice (250 ms) has been recorded (the buffers start as SDRAM
// garbage, so only recorded samples are ever read). Assumes 48 kHz. Set() once per block,
// Process() per sample.
class CaptureLoop {
  public:
    enum : int { kMax = 96000 }; // 2 s at 48 kHz
    enum : int { kHeads = 3 };

    void Init(float sample_rate, float *left, float *right) {
        buffer_[0] = left;
        buffer_[1] = right;
        fade_step_ = 1.f / (0.03f * sample_rate);
        slice_ = (int)(0.25f * sample_rate);
        grain_ = (int)(0.125f * sample_rate);
        crossfade_ = (int)(0.02f * sample_rate);
        Reset();
    }

    void Reset() {
        write_ = filled_ = 0;
        playing_ = wanted_ = previous_ = CaptureMode::kNormal;
        mix_ = 0.f;
        swap_ = 1.f;
        searching_ = kNone;
        random_.Seed(37u);
    }

    void Set(CaptureMode mode) {
        if (mode != CaptureMode::kNormal && playing_ == CaptureMode::kNormal && filled_ < slice_) return;
        wanted_ = mode;
    }

    void Process(float &l, float &r) {
        if (searching_ == kLoopStart && wanted_ != CaptureMode::kFreeze) searching_ = kNone; // not wanted now
        if (playing_ != wanted_) {
            if (playing_ == CaptureMode::kNormal) {
                Start(wanted_, slice_ - grain_, crossfade_); // mix_ is 0 in Normal
            } else if (wanted_ == CaptureMode::kNormal) {
                if (mix_ == 0.f) { // faded out: record afresh
                    playing_ = CaptureMode::kNormal;
                    filled_ = 0;
                    searching_ = kNone;
                }
            } else if (swap_ >= 1.f) { // Freeze <-> Loop: crossfade directly
                if (wanted_ == CaptureMode::kLoop) {
                    previous_ = playing_;
                    swap_ = 0.f;
                    Start(CaptureMode::kLoop, 0, LoudestHeadPosition());
                } else if (searching_ != kLoopStart) {
                    BeginLoopStartSearch(); // Freeze starts once it has found its place
                } else if (searching_ == kLoopStart && candidate_ >= kCandidates) {
                    searching_ = kNone;
                    previous_ = playing_;
                    swap_ = 0.f;
                    Start(CaptureMode::kFreeze, best_, 0);
                }
            }
        }
        float target = playing_ != CaptureMode::kNormal && wanted_ != CaptureMode::kNormal ? 1.f : 0.f;
        mix_ = target > mix_ ? fminf(mix_ + fade_step_, 1.f) : fmaxf(mix_ - fade_step_, 0.f);
        if (playing_ == CaptureMode::kNormal) {
            buffer_[0][write_] = l;
            buffer_[1][write_] = r;
            if (++write_ >= kMax) write_ = 0;
            if (filled_ < kMax) filled_++;
            return;
        }
        SearchStep();
        float cl, cr;
        Generate(playing_, cl, cr);
        if (swap_ < 1.f) {
            swap_ = fminf(swap_ + fade_step_, 1.f);
            float pl, pr;
            Generate(previous_, pl, pr);
            cl = pl + swap_ * (cl - pl);
            cr = pr + swap_ * (cr - pr);
        }
        l += mix_ * (cl - l);
        r += mix_ * (cr - r);
    }

    int LoopSamples() const { return length_ - crossfade_; } // the loop's period, in samples

  private:
    enum : int { kRefLength = 96, kRefStride = 4 };      // the match: 96 samples over 8 ms
    enum : int { kCandidates = 256, kCandidateStep = 2 }; // searched: +-256 samples (one 94 Hz period)
    enum : int { kLead = 300 };                          // a head's search starts this long before its jump
    enum : int { kNone = -1, kLoopStart = kHeads };      // searching_: none, a head, or Freeze's start

    // Freeze: all three heads read the same place (`at` into the slice) to begin with.
    // Loop: from `position`.
    void Start(CaptureMode mode, int at, int position) {
        playing_ = mode;
        end_ = write_; // one past the last recorded sample (recording has stopped)
        if (mode == CaptureMode::kFreeze) {
            tick_ = 0;
            fresh_ = true;
            for (int h = 0; h < kHeads; h++) offset_[h] = at - HeadTick(h);
        } else {
            length_ = filled_;
            position_ = position;
        }
    }

    void Generate(CaptureMode mode, float &l, float &r) {
        if (mode == CaptureMode::kFreeze) {
            FreezeSample(l, r);
        } else {
            LoopSample(l, r);
        }
    }

    int Wrap(int i) const { return i < 0 ? i + kMax : (i >= kMax ? i - kMax : i); }
    int SliceStart() const { return end_ - slice_; }

    // Head h's sample within its grain (0 .. grain_ - 1); the heads are a third of a grain apart
    int HeadTick(int h) const {
        int t = tick_ + h * (grain_ / kHeads);
        return t >= grain_ ? t - grain_ : t;
    }

    void FreezeSample(float &l, float &r) {
        // The windows: sin^2 of x, x + pi/3 and x + 2 pi/3 (the heads are a third of a grain
        // apart, and sin^2 repeats every pi), from one sin and cos
        static_assert(kHeads == 3, "the windows below assume three heads");
        float x = 3.14159265359f * (float)tick_ / (float)grain_, s = sinf(x), c = cosf(x);
        const float sines[kHeads] = {s, 0.5f * s + 0.866025404f * c, -0.5f * s + 0.866025404f * c};
        l = r = 0.f;
        for (int h = 0; h < kHeads; h++) {
            int t = HeadTick(h);
            if (t == 0 && !fresh_) { // the window is at 0: jump (but not on the first sample)
                offset_[h] = searching_ == h && candidate_ >= kCandidates ? best_ : center_;
                if (searching_ == h) searching_ = kNone;
            } else if (t == grain_ - kLead && searching_ == kNone) {
                BeginHeadSearch(h);
            }
            int i = Wrap(SliceStart() + offset_[h] + t);
            float w = sines[h] * sines[h];
            l += w * buffer_[0][i];
            r += w * buffer_[1][i];
        }
        l *= 2.f / (float)kHeads; // the windows sum to kHeads / 2
        r *= 2.f / (float)kHeads;
        if (++tick_ >= grain_) tick_ = 0;
        fresh_ = false;
    }

    // The loop's position `ahead` samples from now
    int LoopPositionAhead(int ahead) const {
        int q = position_ + ahead;
        while (q >= length_) q -= length_ - crossfade_;
        return q;
    }

    // Loop: start in step with the loudest freeze head (the one nearest the middle of its window)
    int LoudestHeadPosition() const {
        int loudest = 0, nearest = grain_;
        for (int h = 0; h < kHeads; h++) {
            int distance = abs(HeadTick(h) - grain_ / 2);
            if (distance < nearest) {
                nearest = distance;
                loudest = h;
            }
        }
        int index = SliceStart() + offset_[loudest] + HeadTick(loudest); // where it reads now
        int position = index - (end_ - filled_); // into the loop (which is everything recorded)
        if (position < 0) position += kMax;
        return position < crossfade_ ? crossfade_ : (position >= filled_ ? filled_ - 1 : position);
    }

    // Copies the reference into ref_ (from the louder side) and starts the search around `center`
    void BeginSearch(int who, int center, const int *reference_index) {
        float energy[2] = {0.f, 0.f};
        for (int c = 0; c < 2; c++)
            for (int m = 0; m < kRefLength; m++) {
                float v = buffer_[c][reference_index[m]];
                energy[c] += v * v;
            }
        channel_ = energy[1] > energy[0] ? 1 : 0;
        for (int m = 0; m < kRefLength; m++) ref_[m] = buffer_[channel_][reference_index[m]];
        searching_ = who;
        center_ = center;
        candidate_ = 0;
        best_ = center;
        best_score_ = -1e30f;
    }

    // Head h jumps in kLead samples: match what head h + 1 (a third of a grain ahead, so it
    // overlaps h longest) will play a sixth of a grain after the jump
    void BeginHeadSearch(int h) {
        const int range = kCandidates * kCandidateStep / 2;
        int center = range + (int)(random_.Uniform() * (float)(slice_ - grain_ - 2 * range));
        int ahead = (h + 1) % kHeads;
        match_ = grain_ / 6;
        int from = SliceStart() + offset_[ahead] + grain_ / 3 + match_;
        int reference_index[kRefLength];
        for (int m = 0; m < kRefLength; m++) reference_index[m] = Wrap(from + kRefStride * m);
        BeginSearch(h, center, reference_index);
    }

    // Loop -> Freeze: the freeze starts once the search is done (kCandidates samples from now),
    // all heads reading one place near the middle of the slice: the one that matches the loop then
    void BeginLoopStartSearch() {
        match_ = 0;
        int start = end_ - length_;
        int reference_index[kRefLength];
        for (int m = 0; m < kRefLength; m++)
            reference_index[m] = Wrap(start + LoopPositionAhead(kCandidates + kRefStride * m));
        BeginSearch(kLoopStart, (slice_ - grain_) * 5 / 6, reference_index);
    }

    // One candidate a sample: candidate places center_ - range .. center_ + range
    void SearchStep() {
        if (searching_ == kNone || candidate_ >= kCandidates) return;
        const int range = kCandidates * kCandidateStep / 2;
        int place = center_ - range + kCandidateStep * candidate_;
        int from = Wrap(SliceStart() + place + match_);
        const float *line = buffer_[channel_];
        float score = 0.f;
        if (from + kRefStride * (kRefLength - 1) < kMax) { // (usually: no wrap within the match)
            for (int m = 0; m < kRefLength; m++) score += ref_[m] * line[from + kRefStride * m];
        } else {
            for (int m = 0; m < kRefLength; m++) score += ref_[m] * line[Wrap(from + kRefStride * m)];
        }
        if (score > best_score_) {
            best_score_ = score;
            best_ = place;
        }
        candidate_++;
    }

    // The last `length_` samples, positions crossfade_..length_; the last crossfade_ samples
    // fade into the samples just after the loop's start, then it jumps back to crossfade_
    void LoopSample(float &l, float &r) {
        int start = end_ - length_;
        int i = Wrap(start + position_);
        l = buffer_[0][i];
        r = buffer_[1][i];
        int into = position_ - (length_ - crossfade_);
        if (into >= 0) {
            float a = (float)into / (float)crossfade_;
            int j = Wrap(start + into);
            l += a * (buffer_[0][j] - l);
            r += a * (buffer_[1][j] - r);
        }
        if (++position_ >= length_) position_ = crossfade_;
    }

    float *buffer_[2] = {nullptr, nullptr};
    int write_ = 0, filled_ = 0, end_ = 0, slice_ = 12000, grain_ = 6000, crossfade_ = 960;
    int length_ = 1, position_ = 0;
    int offset_[kHeads] = {}; // each head's grain start, into the slice
    int tick_ = 0;            // head 0's sample within its grain
    bool fresh_ = false;      // the freeze's first sample
    float fade_step_ = 0.f, mix_ = 0.f, swap_ = 1.f;
    // The search
    int searching_ = kNone, channel_ = 0, center_ = 0, candidate_ = 0, best_ = 0, match_ = 0;
    float best_score_ = 0.f;
    float ref_[kRefLength] = {};
    Random random_;
    CaptureMode playing_ = CaptureMode::kNormal, wanted_ = CaptureMode::kNormal, previous_ = CaptureMode::kNormal;
};

} // namespace bitch
