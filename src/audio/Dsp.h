#pragma once
// Small DSP toolkit for the procedural car audio (header-only, allocation free after setup).
#include <cmath>
#include <cstdint>
#include <vector>

namespace apex::dsp {

constexpr float kPi = 3.14159265358979323846f;

struct Noise {
    uint32_t s = 0x12345678u;
    explicit Noise(uint32_t seed = 0x12345678u) : s(seed ? seed : 1u) {}
    float white() {  // [-1, 1]
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        return static_cast<int32_t>(s) * (1.0f / 2147483648.0f);
    }
};

// Pink-ish noise (Paul Kellet's economy filter).
struct PinkNoise {
    Noise n;
    float b0 = 0, b1 = 0, b2 = 0;
    explicit PinkNoise(uint32_t seed = 99) : n(seed) {}
    float next() {
        float w = n.white();
        b0 = 0.99765f * b0 + w * 0.0990460f;
        b1 = 0.96300f * b1 + w * 0.2965164f;
        b2 = 0.57000f * b2 + w * 1.0526913f;
        return (b0 + b1 + b2 + w * 0.1848f) * 0.25f;
    }
};

struct OnePole {
    float a = 0.0f, z = 0.0f;
    void setLowpass(float cutoff, float sr) { a = std::exp(-2.0f * kPi * cutoff / sr); }
    float lp(float x) { return z = x + (z - x) * a; }
    float hp(float x) { return x - lp(x); }
};

// Smoothed parameter (one-pole towards a target).
struct Smoother {
    float v = 0.0f, k = 0.001f;
    void setTime(float seconds, float sr) { k = 1.0f - std::exp(-1.0f / (seconds * sr)); }
    float next(float target) { return v += (target - v) * k; }
};

// RBJ biquad.
struct Biquad {
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    float z1 = 0, z2 = 0;
    float process(float x) {
        float y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
    void set(float nb0, float nb1, float nb2, float na0, float na1, float na2) {
        b0 = nb0 / na0;
        b1 = nb1 / na0;
        b2 = nb2 / na0;
        a1 = na1 / na0;
        a2 = na2 / na0;
    }
    void bandpass(float f, float q, float sr) {
        f = std::fmin(f, sr * 0.45f);
        float w = 2 * kPi * f / sr, al = std::sin(w) / (2 * q);
        set(al, 0, -al, 1 + al, -2 * std::cos(w), 1 - al);
    }
    void lowpass(float f, float q, float sr) {
        f = std::fmin(f, sr * 0.45f);
        float w = 2 * kPi * f / sr, al = std::sin(w) / (2 * q), c = std::cos(w);
        set((1 - c) / 2, 1 - c, (1 - c) / 2, 1 + al, -2 * c, 1 - al);
    }
    void highpass(float f, float q, float sr) {
        f = std::fmin(f, sr * 0.45f);
        float w = 2 * kPi * f / sr, al = std::sin(w) / (2 * q), c = std::cos(w);
        set((1 + c) / 2, -(1 + c), (1 + c) / 2, 1 + al, -2 * c, 1 - al);
    }
    void peak(float f, float q, float gainDb, float sr) {
        f = std::fmin(f, sr * 0.45f);
        float A = std::pow(10.0f, gainDb / 40.0f);
        float w = 2 * kPi * f / sr, al = std::sin(w) / (2 * q), c = std::cos(w);
        set(1 + al * A, -2 * c, 1 - al * A, 1 + al / A, -2 * c, 1 - al / A);
    }
};

// Fractional delay line with linear interpolation.
struct Delay {
    std::vector<float> buf;
    size_t w = 0;
    void init(size_t maxSamples) {
        buf.assign(maxSamples + 2, 0.0f);
        w = 0;
    }
    void push(float x) {
        buf[w] = x;
        w = (w + 1) % buf.size();
    }
    float tap(float delaySamples) const {
        float d = std::fmin(std::fmax(delaySamples, 1.0f), static_cast<float>(buf.size() - 2));
        float rp = static_cast<float>(w) - d;
        while (rp < 0) rp += static_cast<float>(buf.size());
        size_t i0 = static_cast<size_t>(rp);
        size_t i1 = (i0 + 1) % buf.size();
        float t = rp - static_cast<float>(i0);
        return buf[i0] + (buf[i1] - buf[i0]) * t;
    }
};

// Compact stereo reverb (Freeverb-style combs + allpasses), used for the forest ambience.
struct Reverb {
    struct Comb {
        std::vector<float> b;
        size_t i = 0;
        float store = 0;
        float process(float x, float fb, float damp) {
            float y = b[i];
            store = y * (1 - damp) + store * damp;
            b[i] = x + store * fb;
            i = (i + 1) % b.size();
            return y;
        }
    };
    struct Allpass {
        std::vector<float> b;
        size_t i = 0;
        float process(float x) {
            float y = b[i];
            b[i] = x + y * 0.5f;
            i = (i + 1) % b.size();
            return y - x;
        }
    };
    Comb cl[4], cr[4];
    Allpass al[2], ar[2];
    float feedback = 0.80f, damp = 0.35f;
    void init(float sr) {
        const int combs[4] = {1116, 1188, 1277, 1356};
        const int aps[2] = {556, 441};
        float k = sr / 44100.0f;
        for (int i = 0; i < 4; ++i) {
            cl[i].b.assign(static_cast<size_t>(combs[i] * k), 0.0f);
            cr[i].b.assign(static_cast<size_t>((combs[i] + 23) * k), 0.0f);
        }
        for (int i = 0; i < 2; ++i) {
            al[i].b.assign(static_cast<size_t>(aps[i] * k), 0.0f);
            ar[i].b.assign(static_cast<size_t>((aps[i] + 23) * k), 0.0f);
        }
    }
    void process(float in, float& outL, float& outR) {
        float l = 0, r = 0;
        for (int i = 0; i < 4; ++i) {
            l += cl[i].process(in, feedback, damp);
            r += cr[i].process(in, feedback, damp);
        }
        for (int i = 0; i < 2; ++i) {
            l = al[i].process(l);
            r = ar[i].process(r);
        }
        outL = l * 0.25f;
        outR = r * 0.25f;
    }
};

inline float softClip(float x) { return std::tanh(x); }

}  // namespace apex::dsp
