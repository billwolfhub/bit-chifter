// bit-chifter for the Synthux Simple Touch: ring mod, shifter, radio filter, stereo.
// Fader 1 TUNE (the SSB dial: fine +-400 Hz around 0 at the centre, swoop zones to +-2 kHz at the
// ends, glides like a heavy dial, harmonic detents), fader 2 RECEPTION (clean to lost in the noise), knob 1 RING, knob 2 CARRIER, knob 3 FILTER,
// knob 4 WIDTH, knob 5 FEEDBACK, knob 6 MIX (its last 15% adds reverb). Switch 1 picks the page of pad
// effects (Shift & time / Radio / Digital & damage): a pad's effect is on while it's held, and
// RECEPTION also sets how strong they are. Built so far: feedback burst, dial spins, static burst,
// squelch, signal fade-out, filter slam, ring 100%, carrier x2, kill dry, hold drift; and the radio
// pads: scrambler, hopping scrambler, passing carriers, selcall, Morse chimes, jammer; and the damage
// pads: bitcrush, sample-rate crush, dropouts, stutter, codec warble, and chaos (2-3 random effects
// while held); page 1: echoes (shifted, reverse, ping-pong), spiral up/down, chirp, warble. Switch 2
// (right switch): Normal / Freeze / Loop.
// The "TOU" logo pad: a tap steps the stereo mode (Linked / Mirror / Rotate, with 1/2/3 confirmation
// blips). The CPU load and the
// controls are printed over USB serial once a second.
#include "daisy_seed.h"
#include "engine.h"
#include "mapping.h"
#include "blips.h"
#include "effects.h"
#include "ui/pad_pages.h"
#include "util/CpuLoadMeter.h"

#include <cstdarg>
#include <cstdio>

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
const int kInputReception = 7; // fader 2

DaisySeed hw;
AnalogControl controls[8];
bitch::Engine engine;
// ~145 KB: too big for DTCM (where globals go); Init() clears it (uninitialised SDRAM holds NaNs).
// PlateReverb/DelayLine must stay constant-initialisable (no user constructors): global
// constructors run before hw.Init() sets up the SDRAM.
DSY_SDRAM_BSS bitch::PlateReverb plate;
DSY_SDRAM_BSS bitch::TapeMemory tape; // the echoes' and the capture's buffers (~1.1 MB; same rules as the plate)
CpuLoadMeter cpu;
Mpr121I2C pads;
bitch::Blips blips;
Switch3 page_switch;           // switch 1 (left): the page; reads mirror-image (README)
bitch::PadPages pad_pages;     // pads -> effects (on while held), TOU tap -> stereo mode
volatile uint32_t effects = 0; // from the pads (main loop) to the engine (audio callback)
volatile int page = 0;         // for the report
Switch3 capture_switch;        // switch 2 (right): Normal / Freeze / Loop; reads mirror-image
volatile int capture = 0;      // 0 Normal, 1 Freeze, 2 Loop
volatile int blip_samples = 0; // diagnostic: non-zero blip samples since the last report
volatile int stereo_mode = 0; // 0 Linked, 1 Mirror, 2 Rotate (cycled by the TOU pad)
volatile float in_peak[2], out_peak[2]; // diagnostic: block peaks since the last report

float Read(int input) { return controls[input].Value(); }

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
    cpu.OnBlockStart();
    for (AnalogControl &c : controls) c.Process();
    bitch::Params p;
    p.tune_hz = bitch::TuneHz(Read(kInputTune));
    p.feedback = 0.95f * Read(kInputFeedback);
    bitch::MixAndReverb(Read(kInputMix), p.mix, p.reverb);
    p.ring = Read(kInputRing);
    p.carrier_hz = bitch::CarrierHz(Read(kInputCarrier));
    p.filter_hz = bitch::FilterHz(Read(kInputFilter));
    bitch::FilterWidth(Read(kInputWidth), p.filter_q, p.filter_open);
    p.reception = bitch::ReceptionAmount(Read(kInputReception));
    p.stereo_mode = (bitch::StereoMode)stereo_mode;
    p.effects = effects;
    p.effect_amount = bitch::EffectAmount(p.reception);
    p.capture = (bitch::CaptureMode)capture;
    p.detent = 0.7f;          // harmonic detents, medium
    p.dial_inertia_s = 0.06f; // TUNE glides like a heavy dial (two 60 ms stages)
    engine.Process(in[0], in[1], out[0], out[1], size, p);
    for (size_t i = 0; i < size; i++) {
        float b = blips.Process();
        if (b != 0.f) blip_samples++;
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

// Sends a report over USB without waiting: if the computer isn't reading (or the
// previous report is still going out), this one is simply dropped.
void Report(const char *format, ...) {
    static char text[320];
    va_list args;
    va_start(args, format);
    int n = vsnprintf(text, sizeof(text) - 2, format, args);
    va_end(args);
    if (n < 0) return;
    if (n > (int)sizeof(text) - 3) n = sizeof(text) - 3;
    text[n++] = '\r';
    text[n++] = '\n';
    hw.usb_handle.TransmitInternal((uint8_t *)text, n);
}

int main(void) {
    hw.Init(true); // 480 MHz
    hw.SetAudioBlockSize(48);
    // USB serial for the once-a-second report. Not libDaisy's logger: after its first two
    // messages it waits for the computer to read each one, so with nothing reading the
    // port the main loop (pads, blips, report) would freeze.
    hw.usb_handle.Init(UsbHandle::FS_INTERNAL);

    AdcChannelConfig cfg[8];
    Pin pins[8] = {seed::A0, seed::A1, seed::A2, seed::A3, seed::A4, seed::A5, seed::A6, seed::A7};
    for (int i = 0; i < 8; i++) cfg[i].InitSingle(pins[i]);
    hw.adc.Init(cfg, 8);
    for (int i = 0; i < 8; i++) controls[i].Init(hw.adc.GetPtr(i), hw.AudioCallbackRate());
    hw.adc.Start();

    // Flush denormals to zero: the feedback loop can otherwise settle into a tiny
    // subnormal limit cycle after silence
    __set_FPSCR(__get_FPSCR() | (1u << 24));
    FPU->FPDSCR |= FPU_FPDSCR_FZ_Msk; // the audio interrupt's FP context starts from FPDSCR, not FPSCR
    engine.Init(hw.AudioSampleRate(), hw.AudioBlockSize(), &plate, &tape); // after hw.Init (which sets up the SDRAM)
    cpu.Init(hw.AudioSampleRate(), hw.AudioBlockSize());
    Mpr121I2C::Config pad_config;
    bool pads_ok = pads.Init(pad_config) == Mpr121I2C::OK;
    blips.Init(hw.AudioSampleRate());
    page_switch.Init(seed::D9, seed::D8);
    capture_switch.Init(seed::D7, seed::D6);
    hw.StartAudio(AudioCallback);

    static const char *capture_names[3] = {"normal", "freeze", "loop"};
    static const char *mode_names[3] = {"linked", "mirror", "rotate"};
    uint32_t last_report = System::GetNow();
    while (1) {
        System::Delay(10);
        // Pads and switch 1: the strange pages (each pad's effect is on while it's held); a TOU
        // tap steps the stereo mode (1, 2 or 3 blips).
        // Switch 1 physically left reads POS_RIGHT: left = page 1, centre = page 2, right = page 3.
        int position = page_switch.Read();
        page = position == Switch3::POS_RIGHT ? 0 : (position == Switch3::POS_CENTER ? 1 : 2);
        // Switch 2, also mirror-image: physically left = Normal, centre = Freeze, right = Loop
        int capture_position = capture_switch.Read();
        capture = capture_position == Switch3::POS_RIGHT ? 0 : (capture_position == Switch3::POS_CENTER ? 1 : 2);
        pad_pages.Update(System::GetNow(), pads_ok ? pads.Touched() : 0u, page);
        effects = pad_pages.Effects();
        int steps = pad_pages.TakeStereoSteps();
        if (steps > 0) {
            stereo_mode = (stereo_mode + steps) % 3;
            blips.Start(stereo_mode + 1); // Blips::Start is safe from the main loop (atomic request)
        }
        if (System::GetNow() - last_report < 1000) continue;
        last_report = System::GetNow();
        auto db = [](float v) { return v > 0.0000112f ? (int)lrintf(20.f * log10f(v)) : -99; };
        float q, open;
        bitch::FilterWidth(Read(kInputWidth), q, open);
        float mix, reverb;
        bitch::MixAndReverb(Read(kInputMix), mix, reverb);
        // The tracked pitch of the input ("--" when there's no clear pitch)
        char pitch[24] = "--";
        if (engine.PitchConfidence() > 0.f)
            snprintf(pitch, sizeof(pitch), "%d Hz %d%%", (int)engine.Pitch(), (int)(engine.PitchConfidence() * 100.f));
        // One message per second, two lines
        Report("CPU %d%% max %d%% | tune %d Hz fb %d%% mix %d%% rev %d%% rec %d%% | ring %d%% car %d Hz | "
                     "filt %d Hz q %d.%d open %d%%\r\n"
               "  %s%s | in L %d R %d dB | out L %d R %d dB | blip samples %d | pitch %s | page %d fx %08lx capture %s",
                     (int)(cpu.GetAvgCpuLoad() * 100.f), (int)(cpu.GetMaxCpuLoad() * 100.f),
                     (int)bitch::TuneHz(Read(kInputTune)), (int)(Read(kInputFeedback) * 100.f),
                     (int)(mix * 100.f), (int)(reverb * 100.f),
                     (int)(bitch::ReceptionAmount(Read(kInputReception)) * 100.f), (int)(Read(kInputRing) * 100.f),
                     (int)bitch::CarrierHz(Read(kInputCarrier)), (int)bitch::FilterHz(Read(kInputFilter)),
                     (int)q, (int)(q * 10.f) % 10, (int)(open * 100.f), mode_names[stereo_mode],
               pads_ok ? "" : " (pads not found)", db(in_peak[0]), db(in_peak[1]), db(out_peak[0]),
               db(out_peak[1]), blip_samples, pitch, page + 1, (unsigned long)effects, capture_names[capture]);
        blip_samples = 0;
        in_peak[0] = in_peak[1] = out_peak[0] = out_peak[1] = 0.f;
        cpu.Reset();
    }
}
