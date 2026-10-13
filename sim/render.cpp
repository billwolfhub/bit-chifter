// Renders a 16-bit WAV through the bit-chifter engine.
// Usage: render in.wav out.wav [tune=Hz] [fb=0-0.95] [mix=0-1] [ring=0-1] [carrier=Hz]
//                              [filter=Hz] [q=0.7-12] [open=0-1] [reception=0-1]
//                              [detent=0-0.95] [inertia=s] [reverb=0-1] [sweep=Hz] [stereo=linked|mirror|rotate]
// sweep=Hz ramps the tune linearly from tune= (at the start) to sweep= (at the end of the file).
// (defaults are the engine's: no shift, no ring mod, filter open, linked)
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "engine.h"
#include "wav.h"

static bool SetOption(bitch::Params &p, float &sweep_to, const char *arg) {
    const char *eq = strchr(arg, '=');
    if (!eq) return false;
    size_t n = (size_t)(eq - arg);
    const char *value = eq + 1;
    if (n == 6 && strncmp(arg, "stereo", 6) == 0) {
        if (!strcmp(value, "linked")) p.stereo_mode = bitch::StereoMode::kLinked;
        else if (!strcmp(value, "mirror")) p.stereo_mode = bitch::StereoMode::kMirror;
        else if (!strcmp(value, "rotate")) p.stereo_mode = bitch::StereoMode::kRotate;
        else return false;
        return true;
    }
    struct {
        const char *name;
        float *field;
    } fields[] = {{"tune", &p.tune_hz},     {"fb", &p.feedback},        {"mix", &p.mix},
                  {"ring", &p.ring},        {"carrier", &p.carrier_hz}, {"filter", &p.filter_hz},
                  {"q", &p.filter_q},       {"open", &p.filter_open},
                  {"reception", &p.reception}, {"detent", &p.detent},
                  {"inertia", &p.dial_inertia_s}, {"reverb", &p.reverb},
                  {"sweep", &sweep_to}};
    for (auto &f : fields) {
        if (strlen(f.name) == n && strncmp(arg, f.name, n) == 0) {
            *f.field = (float)atof(value);
            return true;
        }
    }
    return false;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr,
                "Usage: %s in.wav out.wav [tune=] [fb=] [mix=] [ring=] [carrier=] [filter=] [q=] [open=] "
                "[reception=] [detent=] [inertia=] [reverb=0-1] [sweep=Hz] [stereo=linked|mirror|rotate]\n",
                argv[0]);
        return 1;
    }
    bitch::Params p;
    float sweep_to = NAN;
    for (int i = 3; i < argc; i++) {
        if (!SetOption(p, sweep_to, argv[i])) {
            fprintf(stderr, "Unknown option %s\n", argv[i]);
            return 1;
        }
    }
    Wav in;
    if (!ReadWav(argv[1], in)) {
        fprintf(stderr, "Couldn't read %s (needs 16-bit PCM WAV, mono or stereo)\n", argv[1]);
        return 1;
    }
    static bitch::PlateReverb plate; // large: keep it off the stack
    static bitch::Engine engine;
    engine.Init((float)in.sample_rate, 48, &plate);
    Wav out;
    out.sample_rate = in.sample_rate;
    out.left.resize(in.left.size());
    out.right.resize(in.left.size());
    const size_t block = 48;
    const float tune_from = p.tune_hz;
    for (size_t i = 0; i + block <= in.left.size(); i += block) {
        if (!std::isnan(sweep_to)) p.tune_hz = tune_from + (sweep_to - tune_from) * (float)i / (float)in.left.size();
        engine.Process(&in.left[i], &in.right[i], &out.left[i], &out.right[i], block, p);
    }
    p.tune_hz = tune_from;
    if (!WriteWav(argv[2], out)) {
        fprintf(stderr, "Couldn't write %s\n", argv[2]);
        return 1;
    }
    static const char *modes[3] = {"linked", "mirror", "rotate"};
    printf("Wrote %s (tune %g fb %g mix %g ring %g carrier %g filter %g q %g open %g reception %g detent %g inertia %g reverb %g", argv[2],
           p.tune_hz, p.feedback, p.mix, p.ring, p.carrier_hz, p.filter_hz, p.filter_q, p.filter_open, p.reception,
           p.detent, p.dial_inertia_s, p.reverb);
    if (!std::isnan(sweep_to)) printf(" sweep %g", sweep_to);
    printf(" stereo %s)\n", modes[(int)p.stereo_mode]);
    return 0;
}
