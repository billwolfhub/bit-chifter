// Bitchshifter for the Synthux Simple Touch: part 1, a playable frequency shifter.
// Fader 1 TUNE (shift, 0 at the centre), knob 5 FEEDBACK, knob 6 MIX. The CPU
// load is printed over USB serial once a second.
#include "daisy_seed.h"
#include "engine.h"
#include "util/CpuLoadMeter.h"

using namespace daisy;

// Control map, from hwtest/ (see README.md): A-index of each control, and whether
// it reads reversed (needs flipping so that up/clockwise = more)
const int kInputTune = 6;     // fader 1
const int kInputFeedback = 0; // knob 5
const int kInputMix = 5;      // knob 6
const bool kFlipTune = false;
const bool kFlipFeedback = false;
const bool kFlipMix = false;

const float kMaxShiftHz = 2500.f;
const float kTuneDeadZone = 0.02f; // around the centre, so exactly 0 Hz is easy to find

DaisySeed hw;
AnalogControl controls[8];
bitch::Engine engine;
CpuLoadMeter cpu;
volatile float in_peak[2], out_peak[2]; // diagnostic: block peaks since the last report

float Read(int input, bool flip) {
    float v = controls[input].Value();
    return flip ? 1.f - v : v;
}

// Fader 0..1 -> -2500..+2500 Hz, cubic so it's fine near 0 and wild at the ends
float TuneHz(float fader) {
    float x = 2.f * fader - 1.f;
    if (fabsf(x) < kTuneDeadZone) return 0.f;
    x = (x - copysignf(kTuneDeadZone, x)) / (1.f - kTuneDeadZone);
    return kMaxShiftHz * x * x * x;
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
    cpu.OnBlockStart();
    for (AnalogControl &c : controls) c.Process();
    bitch::Params p;
    p.tune_hz = TuneHz(Read(kInputTune, kFlipTune));
    p.feedback = 0.95f * Read(kInputFeedback, kFlipFeedback);
    p.mix = Read(kInputMix, kFlipMix);
    engine.Process(in[0], in[1], out[0], out[1], size, p);
    for (size_t i = 0; i < size; i++) {
        for (int c = 0; c < 2; c++) {
            in_peak[c] = fmaxf(in_peak[c], fabsf(in[c][i]));
            out_peak[c] = fmaxf(out_peak[c], fabsf(out[c][i]));
        }
    }
    cpu.OnBlockEnd();
}

int main(void) {
    hw.Init(true); // 480 MHz
    hw.SetAudioBlockSize(48);
    hw.StartLog(false);

    AdcChannelConfig cfg[8];
    Pin pins[8] = {seed::A0, seed::A1, seed::A2, seed::A3, seed::A4, seed::A5, seed::A6, seed::A7};
    for (int i = 0; i < 8; i++) cfg[i].InitSingle(pins[i]);
    hw.adc.Init(cfg, 8);
    for (int i = 0; i < 8; i++) controls[i].Init(hw.adc.GetPtr(i), hw.AudioCallbackRate());
    hw.adc.Start();

    // Flush denormals to zero: the feedback loop can otherwise settle into a tiny
    // subnormal limit cycle after silence
    __set_FPSCR(__get_FPSCR() | (1u << 24));
    engine.Init(hw.AudioSampleRate());
    cpu.Init(hw.AudioSampleRate(), hw.AudioBlockSize());
    hw.StartAudio(AudioCallback);

    while (1) {
        System::Delay(1000);
        auto db = [](float v) { return v > 0.0000112f ? (int)lrintf(20.f * log10f(v)) : -99; };
        hw.PrintLine("CPU avg %d%% max %d%% | tune %d Hz fb %d%% mix %d%% | in L %d R %d dB | out L %d R %d dB",
                     (int)(cpu.GetAvgCpuLoad() * 100.f), (int)(cpu.GetMaxCpuLoad() * 100.f),
                     (int)TuneHz(Read(kInputTune, kFlipTune)), (int)(Read(kInputFeedback, kFlipFeedback) * 100.f),
                     (int)(Read(kInputMix, kFlipMix) * 100.f), db(in_peak[0]), db(in_peak[1]), db(out_peak[0]),
                     db(out_peak[1]));
        in_peak[0] = in_peak[1] = out_peak[0] = out_peak[1] = 0.f;
        cpu.Reset();
    }
}
