// Renders a 16-bit WAV through the Bitchshifter engine.
// Usage: render in.wav out.wav <tune_hz> [feedback 0-0.95] [mix 0-1]
#include <cstdio>
#include <cstdlib>

#include "engine.h"
#include "wav.h"

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "Usage: %s in.wav out.wav tune_hz [feedback] [mix]\n", argv[0]);
        return 1;
    }
    Wav in;
    if (!ReadWav(argv[1], in)) {
        fprintf(stderr, "Couldn't read %s (needs 16-bit PCM WAV, mono or stereo)\n", argv[1]);
        return 1;
    }
    bitch::Params p;
    p.tune_hz = (float)atof(argv[3]);
    p.feedback = argc > 4 ? (float)atof(argv[4]) : 0.f;
    p.mix = argc > 5 ? (float)atof(argv[5]) : 1.f;
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
    printf("Wrote %s (shift %g Hz, feedback %g, mix %g)\n", argv[2], p.tune_hz, p.feedback, p.mix);
    return 0;
}
