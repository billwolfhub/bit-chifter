// Bitchshifter for the Synthux Simple Touch: ring mod, shifter, radio filter, stereo.
// Fader 1 TUNE (shift, 0 at the centre), knob 1 RING, knob 2 CARRIER, knob 3 FILTER,
// knob 4 WIDTH, knob 5 FEEDBACK, knob 6 MIX; the "TOU" logo pad steps the stereo mode
// (Linked / Mirror / Rotate, with 1/2/3 confirmation blips). The CPU load and the
// controls are printed over USB serial once a second.
#include "daisy_seed.h"
#include "engine.h"
#include "mapping.h"
#include "blips.h"
#include "util/CpuLoadMeter.h"

using namespace daisy;

// Control map, from hwtest/ and padtest/ (see README.md). None of the analog
// controls read reversed.
const int kInputTune = 6;     // fader 1
const int kInputRing = 1;     // knob 1
const int kInputCarrier = 2;  // knob 2
const int kInputFilter = 3;   // knob 3
const int kInputWidth = 4;    // knob 4
const int kInputFeedback = 0; // knob 5
const int kInputMix = 5;      // knob 6
const int kPadStereoMode = 10; // "TOU" of the TOUCH logo

DaisySeed hw;
AnalogControl controls[8];
bitch::Engine engine;
CpuLoadMeter cpu;
Mpr121I2C pads;
bitch::Blips blips;
volatile int stereo_mode = 0; // 0 Linked, 1 Mirror, 2 Rotate (cycled by the TOU pad)
volatile float in_peak[2], out_peak[2]; // diagnostic: block peaks since the last report

float Read(int input) { return controls[input].Value(); }

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
    cpu.OnBlockStart();
    for (AnalogControl &c : controls) c.Process();
    bitch::Params p;
    p.tune_hz = bitch::TuneHz(Read(kInputTune));
    p.feedback = 0.95f * Read(kInputFeedback);
    p.mix = Read(kInputMix);
    p.ring = Read(kInputRing);
    p.carrier_hz = bitch::CarrierHz(Read(kInputCarrier));
    p.filter_hz = bitch::FilterHz(Read(kInputFilter));
    bitch::FilterWidth(Read(kInputWidth), p.filter_q, p.filter_open);
    p.stereo_mode = (bitch::StereoMode)stereo_mode;
    engine.Process(in[0], in[1], out[0], out[1], size, p);
    for (size_t i = 0; i < size; i++) {
        float b = blips.Process();
        out[0][i] += b;
        out[1][i] += b;
    }
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
    Mpr121I2C::Config pad_config;
    bool pads_ok = pads.Init(pad_config) == Mpr121I2C::OK;
    blips.Init(hw.AudioSampleRate());
    hw.StartAudio(AudioCallback);

    static const char *mode_names[3] = {"linked", "mirror", "rotate"};
    bool was_touched = false;
    uint32_t last_report = System::GetNow();
    while (1) {
        System::Delay(10);
        // TOU pad: each new touch steps the stereo mode; 1, 2 or 3 blips say which
        bool touched = pads_ok && (pads.Touched() & (1 << kPadStereoMode));
        if (touched && !was_touched) {
            stereo_mode = (stereo_mode + 1) % 3;
            // Blips::Start is safe from the main loop (atomic request)
            blips.Start(stereo_mode + 1);
        }
        was_touched = touched;
        if (System::GetNow() - last_report < 1000) continue;
        last_report = System::GetNow();
        auto db = [](float v) { return v > 0.0000112f ? (int)lrintf(20.f * log10f(v)) : -99; };
        float q, open;
        bitch::FilterWidth(Read(kInputWidth), q, open);
        hw.PrintLine("CPU avg %d%% max %d%% | tune %d Hz fb %d%% mix %d%% | ring %d%% carrier %d Hz | "
                     "filter %d Hz q %d.%d open %d%% | %s%s | in L %d R %d dB | out L %d R %d dB",
                     (int)(cpu.GetAvgCpuLoad() * 100.f), (int)(cpu.GetMaxCpuLoad() * 100.f),
                     (int)bitch::TuneHz(Read(kInputTune)), (int)(Read(kInputFeedback) * 100.f),
                     (int)(Read(kInputMix) * 100.f), (int)(Read(kInputRing) * 100.f),
                     (int)bitch::CarrierHz(Read(kInputCarrier)), (int)bitch::FilterHz(Read(kInputFilter)),
                     (int)q, (int)(q * 10.f) % 10, (int)(open * 100.f), mode_names[stereo_mode],
                     pads_ok ? "" : " (pads not found)", db(in_peak[0]), db(in_peak[1]), db(out_peak[0]),
                     db(out_peak[1]));
        in_peak[0] = in_peak[1] = out_peak[0] = out_peak[1] = 0.f;
        cpu.Reset();
    }
}
