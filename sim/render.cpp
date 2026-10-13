// Renders a 16-bit WAV through the Bitchshifter engine.
// Usage: render in.wav out.wav [tune=Hz] [fb=0-0.95] [mix=0-1] [ring=0-1] [carrier=Hz]
//                              [filter=Hz] [q=0.7-12] [open=0-1] [stereo=linked|mirror|rotate]
// (defaults are the engine's: no shift, no ring mod, filter open, linked)
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "engine.h"
#include "wav.h"

static bool SetOption(bitch::Params &p, const char *arg) {
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
                  {"q", &p.filter_q},       {"open", &p.filter_open}};
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
                "[stereo=linked|mirror|rotate]\n",
                argv[0]);
        return 1;
    }
    bitch::Params p;
    for (int i = 3; i < argc; i++) {
        if (!SetOption(p, argv[i])) {
            fprintf(stderr, "Unknown option %s\n", argv[i]);
            return 1;
        }
    }
    Wav in;
    if (!ReadWav(argv[1], in)) {
        fprintf(stderr, "Couldn't read %s (needs 16-bit PCM WAV, mono or stereo)\n", argv[1]);
        return 1;
    }
    bitch::Engine engine;
    engine.Init((float)in.sample_rate);
    Wav out;
    out.sample_rate = in.sample_rate;
    out.left.resize(in.left.size());
    out.right.resize(in.left.size());
    const size_t block = 48;
    for (size_t i = 0; i + block <= in.left.size(); i += block)
        engine.Process(&in.left[i], &in.right[i], &out.left[i], &out.right[i], block, p);
    if (!WriteWav(argv[2], out)) {
        fprintf(stderr, "Couldn't write %s\n", argv[2]);
        return 1;
    }
    static const char *modes[3] = {"linked", "mirror", "rotate"};
    printf("Wrote %s (tune %g fb %g mix %g ring %g carrier %g filter %g q %g open %g stereo %s)\n", argv[2],
           p.tune_hz, p.feedback, p.mix, p.ring, p.carrier_hz, p.filter_hz, p.filter_q, p.filter_open,
           modes[(int)p.stereo_mode]);
    return 0;
}
