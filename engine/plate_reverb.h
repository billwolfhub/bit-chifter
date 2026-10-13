#pragma once

#include <cmath>

namespace bitch {

// A fixed-length delay line: Write() one sample, Tap(d) reads the sample written d
// writes ago (1 = the newest, N = the oldest).
template <int N>
class DelayLine {
  public:
    void Clear() {
        for (float &v : buffer_) v = 0.f;
        position_ = 0;
    }
    void Write(float x) {
        buffer_[position_] = x;
        if (++position_ == N) position_ = 0;
    }
    float Tap(int d) const {
        int i = position_ - d;
        if (i < 0) i += N;
        return buffer_[i];
    }
    // Fractional tap (linear interpolation), 1 <= d < N
    float Tap(float d) const {
        int whole = (int)d;
        float a = Tap(whole), b = Tap(whole + 1);
        return a + (d - (float)whole) * (b - a);
    }

  private:
    float buffer_[N] = {};
    int position_ = 0;
};

// Light stereo plate: Jon Dattorro's figure-of-eight plate ("Effect Design, Part 1",
// 1997), with his delay lengths scaled from 29.761 kHz to 48 kHz (so it assumes a
// 48 kHz sample rate). Mono in (the sum of both inputs), wide stereo out (his output
// taps). Band-limited input (~150 Hz - 8 kHz), 10 ms predelay, four allpasses for
// diffusion, then two cross-coupled loops, each: modulated allpass -> delay -> damping
// -> x decay -> allpass -> delay -> x decay -> into the other loop. Returns only the
// reverb (no dry). About 145 KB of buffers.
class PlateReverb {
  public:
    void Init(float sample_rate) {
        const float kTwoPi = 6.28318530718f;
        low_cut_ = 1.f - expf(-kTwoPi * 150.f / sample_rate);
        high_cut_ = 1.f - expf(-kTwoPi * 8000.f / sample_rate);
        damping_ = 1.f - expf(-kTwoPi * 5000.f / sample_rate);
        lfo_step_ = kTwoPi * 0.9f / sample_rate; // 0.9 Hz wobble on the loops' allpasses
        SetDecay(0.f);
        Clear();
    }

    // amount 0-1: decay ~1.2 s -> ~3 s
    void SetDecay(float amount) { decay_ = 0.35f + 0.3f * fminf(fmaxf(amount, 0.f), 1.f); }

    void Clear() {
        predelay_.Clear();
        in1_.Clear();
        in2_.Clear();
        in3_.Clear();
        in4_.Clear();
        left_ap1_.Clear();
        left_d1_.Clear();
        left_ap2_.Clear();
        left_d2_.Clear();
        right_ap1_.Clear();
        right_d1_.Clear();
        right_ap2_.Clear();
        right_d2_.Clear();
        low_ = high_ = 0.f;
        left_damp_ = right_damp_ = 0.f;
        left_end_ = right_end_ = 0.f;
        lfo_sin_ = 0.f;
        lfo_cos_ = 1.f;
    }

    void Process(float in_l, float in_r, float &out_l, float &out_r) {
        // Band-limit the mono input, predelay it, diffuse it
        float x = 0.5f * (in_l + in_r);
        high_ += high_cut_ * (x - high_);
        low_ += low_cut_ * (high_ - low_);
        predelay_.Write(high_ - low_);
        x = predelay_.Tap((int)kPredelay);
        x = Allpass(in1_, (int)kIn1, x, 0.75f);
        x = Allpass(in2_, (int)kIn2, x, 0.75f);
        x = Allpass(in3_, (int)kIn3, x, 0.625f);
        x = Allpass(in4_, (int)kIn4, x, 0.625f);

        // The loop allpasses' wobble: a quadrature oscillator (the "magic circle")
        lfo_sin_ += lfo_step_ * lfo_cos_;
        lfo_cos_ -= lfo_step_ * lfo_sin_;
        float wobble_l = (float)kExcursion * (1.f + lfo_sin_) * 0.5f;
        float wobble_r = (float)kExcursion * (1.f + lfo_cos_) * 0.5f;

        // Each loop is fed by the other's end from the previous sample
        float into_left = x + decay_ * right_end_, into_right = x + decay_ * left_end_;

        float l = ModulatedAllpass(left_ap1_, (float)kLeftAp1 + wobble_l, into_left, 0.7f);
        left_d1_.Write(l);
        l = left_d1_.Tap((int)kLeftD1);
        left_damp_ += damping_ * (l - left_damp_);
        l = Allpass(left_ap2_, (int)kLeftAp2, decay_ * left_damp_, 0.5f);
        left_d2_.Write(l);
        left_end_ = left_d2_.Tap((int)kLeftD2);

        float r = ModulatedAllpass(right_ap1_, (float)kRightAp1 + wobble_r, into_right, 0.7f);
        right_d1_.Write(r);
        r = right_d1_.Tap((int)kRightD1);
        right_damp_ += damping_ * (r - right_damp_);
        r = Allpass(right_ap2_, (int)kRightAp2, decay_ * right_damp_, 0.5f);
        right_d2_.Write(r);
        right_end_ = right_d2_.Tap((int)kRightD2);

        // Dattorro's output taps, scaled to 48 kHz
        out_l = 0.6f * (right_d1_.Tap(429) + right_d1_.Tap(4797) - right_ap2_.Tap(3085) + right_d2_.Tap(3219) -
                        left_d1_.Tap(3210) - left_ap2_.Tap(302) - left_d2_.Tap(1719));
        out_r = 0.6f * (left_d1_.Tap(569) + left_d1_.Tap(5850) - left_ap2_.Tap(1981) + left_d2_.Tap(4311) -
                        right_d1_.Tap(3405) - right_ap2_.Tap(540) - right_d2_.Tap(195));
    }

  private:
    // Delay lengths in samples at 48 kHz (Dattorro's x 48000 / 29761)
    enum : int {
        kPredelay = 480,
        kIn1 = 229,
        kIn2 = 173,
        kIn3 = 611,
        kIn4 = 447,
        kExcursion = 26, // the loop allpasses' wobble, in samples
        kLeftAp1 = 1084,
        kLeftD1 = 7182,
        kLeftAp2 = 2903,
        kLeftD2 = 6000,
        kRightAp1 = 1464,
        kRightD1 = 6801,
        kRightAp2 = 4284,
        kRightD2 = 5101,
    };

    // Schroeder allpass on a delay line of length d (the line's last tap)
    template <int N>
    static float Allpass(DelayLine<N> &line, int d, float x, float g) {
        float delayed = line.Tap(d);
        float w = x + g * delayed;
        line.Write(w);
        return delayed - g * w;
    }
    template <int N>
    static float ModulatedAllpass(DelayLine<N> &line, float d, float x, float g) {
        float delayed = line.Tap(d);
        float w = x - g * delayed; // Dattorro's decay-diffusion allpasses have the opposite sign
        line.Write(w);
        return delayed + g * w;
    }

    DelayLine<kPredelay> predelay_;
    DelayLine<kIn1> in1_;
    DelayLine<kIn2> in2_;
    DelayLine<kIn3> in3_;
    DelayLine<kIn4> in4_;
    DelayLine<kLeftAp1 + kExcursion + 2> left_ap1_;
    DelayLine<kLeftD1> left_d1_;
    DelayLine<kLeftAp2> left_ap2_;
    DelayLine<kLeftD2> left_d2_;
    DelayLine<kRightAp1 + kExcursion + 2> right_ap1_;
    DelayLine<kRightD1> right_d1_;
    DelayLine<kRightAp2> right_ap2_;
    DelayLine<kRightD2> right_d2_;
    float low_cut_ = 0.f, high_cut_ = 0.f, damping_ = 0.f, lfo_step_ = 0.f, decay_ = 0.35f;
    float low_ = 0.f, high_ = 0.f, left_damp_ = 0.f, right_damp_ = 0.f, left_end_ = 0.f, right_end_ = 0.f;
    float lfo_sin_ = 0.f, lfo_cos_ = 1.f;
};

} // namespace bitch
