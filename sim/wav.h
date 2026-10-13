#pragma once
// Minimal 16-bit PCM WAV reading (mono or stereo) and writing (stereo).
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

struct Wav {
    int sample_rate = 48000;
    std::vector<float> left, right;
};

inline bool ReadWav(const char *path, Wav &wav) {
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    char id[4];
    uint32_t size;
    if (fread(id, 1, 4, f) != 4 || memcmp(id, "RIFF", 4) || fread(&size, 4, 1, f) != 1 ||
        fread(id, 1, 4, f) != 4 || memcmp(id, "WAVE", 4)) {
        fclose(f);
        return false;
    }
    uint16_t format = 0, channels = 0, bits = 0;
    uint32_t rate = 0;
    while (fread(id, 1, 4, f) == 4 && fread(&size, 4, 1, f) == 1) {
        if (!memcmp(id, "fmt ", 4)) {
            uint8_t fmt[40] = {};
            if (fread(fmt, 1, size < 40 ? size : 40, f) < 16) break;
            if (size > 40) fseek(f, size - 40, SEEK_CUR);
            memcpy(&format, fmt, 2);
            memcpy(&channels, fmt + 2, 2);
            memcpy(&rate, fmt + 4, 4);
            memcpy(&bits, fmt + 14, 2);
        } else if (!memcmp(id, "data", 4)) {
            if (format != 1 || bits != 16 || channels < 1 || channels > 2) break;
            std::vector<int16_t> samples(size / 2);
            size_t got = fread(samples.data(), 2, samples.size(), f);
            wav.sample_rate = (int)rate;
            for (size_t i = 0; i + channels <= got; i += channels) {
                wav.left.push_back(samples[i] / 32768.f);
                wav.right.push_back(samples[i + channels - 1] / 32768.f);
            }
            fclose(f);
            return true;
        } else {
            fseek(f, size + (size & 1), SEEK_CUR);
        }
    }
    fclose(f);
    return false;
}

inline bool WriteWav(const char *path, const Wav &wav) {
    FILE *f = fopen(path, "wb");
    if (!f) return false;
    uint32_t frames = (uint32_t)wav.left.size(), data = frames * 4, riff = 36 + data;
    uint32_t rate = (uint32_t)wav.sample_rate, bytes_per_second = rate * 4, fmt_size = 16;
    uint16_t format = 1, channels = 2, block_align = 4, bits = 16;
    fwrite("RIFF", 1, 4, f);
    fwrite(&riff, 4, 1, f);
    fwrite("WAVEfmt ", 1, 8, f);
    fwrite(&fmt_size, 4, 1, f);
    fwrite(&format, 2, 1, f);
    fwrite(&channels, 2, 1, f);
    fwrite(&rate, 4, 1, f);
    fwrite(&bytes_per_second, 4, 1, f);
    fwrite(&block_align, 2, 1, f);
    fwrite(&bits, 2, 1, f);
    fwrite("data", 1, 4, f);
    fwrite(&data, 4, 1, f);
    for (uint32_t i = 0; i < frames; i++) {
        for (float v : {wav.left[i], wav.right[i]}) {
            float c = v < -1.f ? -1.f : (v > 1.f ? 1.f : v);
            int16_t s = (int16_t)(c * 32767.f);
            fwrite(&s, 2, 1, f);
        }
    }
    fclose(f);
    return true;
}
