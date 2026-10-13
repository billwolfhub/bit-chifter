// bit-chifter level test: prints each input's peak level (dBFS) over USB serial
// twice a second, and plays, depending on switch 1 (physically left/centre/right):
//   left   = 440 Hz sine at -6 dBFS on both outputs
//   centre = straight passthrough (IN L -> OUT L, IN R -> OUT R)
//   right  = silence
// (Switch 1 is Init(D9, D8) and reads mirror-image: physical left = POS_RIGHT.)
#include <cmath>

#include "daisy_seed.h"

using namespace daisy;

DaisySeed hw;
Switch3 switch_1;
volatile float peak[2];
// Per input over each report: sum, sum of squares, min, max, count
volatile float sum[2], sum_sq[2], lo[2] = {1.f, 1.f}, hi[2] = {-1.f, -1.f};
volatile int count = 0;
float phase = 0.f;

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
    int position = switch_1.Read();
    count += size;
    for (size_t i = 0; i < size; i++) {
        for (int c = 0; c < 2; c++) {
            float x = in[c][i];
            float a = fabsf(x);
            if (a > peak[c]) peak[c] = a;
            sum[c] += x;
            sum_sq[c] += x * x;
            if (x < lo[c]) lo[c] = x;
            if (x > hi[c]) hi[c] = x;
        }
        if (position == Switch3::POS_RIGHT) { // physically left
            phase += 440.f / hw.AudioSampleRate();
            if (phase >= 1.f) phase -= 1.f;
            out[0][i] = out[1][i] = 0.5f * sinf(6.28318530718f * phase);
        } else if (position == Switch3::POS_CENTER) {
            out[0][i] = in[0][i];
            out[1][i] = in[1][i];
        } else {
            out[0][i] = out[1][i] = 0.f;
        }
    }
}

// dBFS as an integer, floored at -99
int Db(float v) { return v > 0.0000112f ? (int)lrintf(20.f * log10f(v)) : -99; }

int main(void) {
    hw.Init(true);
    hw.SetAudioBlockSize(48);
    hw.StartLog(false);
    switch_1.Init(seed::D9, seed::D8);
    hw.StartAudio(AudioCallback);
    const char *names[3] = {"passthrough", "silence", "tone -6 dBFS"}; // indexed by Read()
    while (1) {
        System::Delay(500);
        __disable_irq();
        float s[2] = {sum[0], sum[1]}, q[2] = {sum_sq[0], sum_sq[1]}, mn[2] = {lo[0], lo[1]}, mx[2] = {hi[0], hi[1]};
        int n = count;
        for (int c = 0; c < 2; c++) {
            peak[c] = sum[c] = sum_sq[c] = 0.f;
            lo[c] = 1.f;
            hi[c] = -1.f;
        }
        count = 0;
        __enable_irq();
        if (n == 0) continue;
        // Mean (DC) and min/max in percent of full scale; AC level in dBFS (RMS, DC removed)
        int dc[2], ac[2], pmin[2], pmax[2];
        for (int c = 0; c < 2; c++) {
            float mean = s[c] / n;
            float var = q[c] / n - mean * mean;
            dc[c] = (int)lrintf(mean * 100.f);
            ac[c] = Db(sqrtf(var > 0.f ? var : 0.f));
            pmin[c] = (int)lrintf(mn[c] * 100.f);
            pmax[c] = (int)lrintf(mx[c] * 100.f);
        }
        hw.PrintLine("L: dc %d%% min %d%% max %d%% audio %d dBFS | R: dc %d%% min %d%% max %d%% audio %d dBFS | %s",
                     dc[0], pmin[0], pmax[0], ac[0], dc[1], pmin[1], pmax[1], ac[1], names[switch_1.Read()]);
    }
}
