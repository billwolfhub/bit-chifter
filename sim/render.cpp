// Renders a 16-bit WAV through the bit-chifter engine.
// Usage: render in.wav out.wav [tune=Hz] [fb=0-0.95] [mix=0-1] [ring=0-1] [carrier=Hz]
//                              [filter=Hz] [q=0.7-12] [open=0-1] [reception=0-1]
//                              [detent=0-0.95] [inertia=s] [reverb=0-1] [sweep=Hz] [stereo=linked|mirror|rotate]
//                              [amount=0.35-1] [effects=name,name] [every=s] [freeze_at=s] [loop_at=s]
// freeze_at=s / loop_at=s switch capture (switch 2) to Freeze / Loop from that second on; whichever time passed most recently wins.
// effects= turns on pad effects by name (effects.h); every=s toggles them on for s seconds, off for s, ...
// sweep=Hz ramps the tune linearly from tune= (at the start) to sweep= (at the end of the file).
// (defaults are the engine's: no shift, no ring mod, filter open, linked)
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "effects.h"
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
                  {"sweep", &sweep_to},   {"amount", &p.effect_amount}};
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
                "[reception=] [detent=] [inertia=] [reverb=0-1] [sweep=Hz] [stereo=linked|mirror|rotate] [amount=0.35-1] [effects=name,name] [every=s] [freeze_at=s] [loop_at=s]\n",
                argv[0]);
        return 1;
    }
    bitch::Params p;
    float sweep_to = NAN;
    float every = 0.f; // seconds; 0 = effects on all the time
    float freeze_at = NAN, loop_at = NAN; // seconds; NAN = never
    for (int i = 3; i < argc; i++) {
        if (strncmp(argv[i], "effects=", 8) == 0) {
            // A comma-separated list of effect names (effects.h), e.g. effects=squelch,spin_up
            char list[256];
            snprintf(list, sizeof(list), "%s", argv[i] + 8);
            for (char *name = strtok(list, ","); name; name = strtok(nullptr, ",")) {
                int found = -1;
                for (int e = 0; e < bitch::kEffectCount; e++)
                    if (strcmp(name, bitch::EffectName(e)) == 0) found = e;
                if (found < 0) {
                    fprintf(stderr, "unknown effect: %s\n", name);
                    return 1;
                }
                p.effects |= 1u << found;
            }
            continue;
        }
        if (strncmp(argv[i], "every=", 6) == 0) {
            every = (float)atof(argv[i] + 6); // effects on for `every` seconds, off for `every`, ...
            continue;
        }
        if (strncmp(argv[i], "freeze_at=", 10) == 0) {
            freeze_at = (float)atof(argv[i] + 10);
            continue;
        }
        if (strncmp(argv[i], "loop_at=", 8) == 0) {
            loop_at = (float)atof(argv[i] + 8);
            continue;
        }
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
    static bitch::TapeMemory tape;   // ~1.1 MB: likewise
    static bitch::Engine engine;
    engine.Init((float)in.sample_rate, 48, &plate, &tape);
    Wav out;
    out.sample_rate = in.sample_rate;
    out.left.resize(in.left.size());
    out.right.resize(in.left.size());
    const size_t block = 48;
    const float tune_from = p.tune_hz;
    const uint32_t effects_on = p.effects;
    for (size_t i = 0; i + block <= in.left.size(); i += block) {
        if (!std::isnan(sweep_to)) p.tune_hz = tune_from + (sweep_to - tune_from) * (float)i / (float)in.left.size();
        if (every > 0.f) {
            float t = (float)i / (float)in.sample_rate;
            p.effects = fmodf(t, 2.f * every) < every ? effects_on : 0u;
        }
        float now = (float)i / (float)in.sample_rate;
        bool frozen = !std::isnan(freeze_at) && now >= freeze_at;
        bool looped = !std::isnan(loop_at) && now >= loop_at;
        if (frozen && looped) frozen = freeze_at > loop_at; // the most recent wins
        p.capture = frozen ? bitch::CaptureMode::kFreeze : looped ? bitch::CaptureMode::kLoop : bitch::CaptureMode::kNormal;
        engine.Process(&in.left[i], &in.right[i], &out.left[i], &out.right[i], block, p);
    }
    p.tune_hz = tune_from;
    p.effects = effects_on;
    if (!WriteWav(argv[2], out)) {
        fprintf(stderr, "Couldn't write %s\n", argv[2]);
        return 1;
    }
    char names[512] = "";
    for (int e = 0; e < bitch::kEffectCount; e++)
        if (p.effects & (1u << e)) snprintf(names + strlen(names), sizeof(names) - strlen(names), "%s%s", names[0] ? "," : "", bitch::EffectName(e));
    if (!names[0]) snprintf(names, sizeof(names), "none");
    static const char *modes[3] = {"linked", "mirror", "rotate"};
    printf("Wrote %s (tune %g fb %g mix %g ring %g carrier %g filter %g q %g open %g reception %g detent %g inertia %g reverb %g amount %g effects %s", argv[2],
           p.tune_hz, p.feedback, p.mix, p.ring, p.carrier_hz, p.filter_hz, p.filter_q, p.filter_open, p.reception,
           p.detent, p.dial_inertia_s, p.reverb, p.effect_amount, names);
    if (!std::isnan(sweep_to)) printf(" sweep %g", sweep_to);
    if (!std::isnan(freeze_at) || !std::isnan(loop_at)) printf(" capture freeze@%g loop@%g", freeze_at, loop_at);
    printf(" stereo %s)\n", modes[(int)p.stereo_mode]);
    return 0;
}
