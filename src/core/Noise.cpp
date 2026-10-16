#include "core/Noise.h"

#include <cmath>

namespace apex {

uint32_t hash32(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

uint32_t hash2(int x, int y, uint32_t seed) {
    return hash32(static_cast<uint32_t>(x) * 0x8da6b343U ^ hash32(static_cast<uint32_t>(y) * 0xd8163841U ^ seed));
}

float hashFloat(uint32_t h) { return (h & 0xFFFFFF) / 16777216.0f; }

static inline float fade(float t) { return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); }
static inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }

float valueNoise1(float x, uint32_t seed) {
    float fx = std::floor(x);
    int ix = static_cast<int>(fx);
    float t = fade(x - fx);
    float a = hashFloat(hash32(static_cast<uint32_t>(ix) ^ seed * 0x9E3779B9U)) * 2.0f - 1.0f;
    float b = hashFloat(hash32(static_cast<uint32_t>(ix + 1) ^ seed * 0x9E3779B9U)) * 2.0f - 1.0f;
    return lerpf(a, b, t);
}

static inline float grad(uint32_t h, float x, float y) {
    switch (h & 7u) {
        case 0: return x + y;
        case 1: return x - y;
        case 2: return -x + y;
        case 3: return -x - y;
        case 4: return x;
        case 5: return -x;
        case 6: return y;
        default: return -y;
    }
}

static float perlinImpl(float x, float y, int period, uint32_t seed) {
    float fx = std::floor(x), fy = std::floor(y);
    int ix = static_cast<int>(fx), iy = static_cast<int>(fy);
    float dx = x - fx, dy = y - fy;
    auto wrap = [period](int v) { return period > 0 ? ((v % period) + period) % period : v; };
    int x0 = wrap(ix), x1 = wrap(ix + 1), y0 = wrap(iy), y1 = wrap(iy + 1);
    float n00 = grad(hash2(x0, y0, seed), dx, dy);
    float n10 = grad(hash2(x1, y0, seed), dx - 1, dy);
    float n01 = grad(hash2(x0, y1, seed), dx, dy - 1);
    float n11 = grad(hash2(x1, y1, seed), dx - 1, dy - 1);
    float u = fade(dx), v = fade(dy);
    return lerpf(lerpf(n00, n10, u), lerpf(n01, n11, u), v) * 0.7071f;
}

float perlin2(float x, float y, uint32_t seed) { return perlinImpl(x, y, 0, seed); }

float perlin2Tiled(float x, float y, int period, uint32_t seed) { return perlinImpl(x, y, period, seed); }

float fbm2(float x, float y, int octaves, float lacunarity, float gain, uint32_t seed) {
    float sum = 0, amp = 1, norm = 0;
    for (int i = 0; i < octaves; ++i) {
        sum += amp * perlin2(x, y, seed + static_cast<uint32_t>(i) * 131u);
        norm += amp;
        x *= lacunarity;
        y *= lacunarity;
        amp *= gain;
    }
    return sum / norm;
}

float fbm2Tiled(float x, float y, int period, int octaves, uint32_t seed) {
    float sum = 0, amp = 1, norm = 0;
    for (int i = 0; i < octaves; ++i) {
        sum += amp * perlin2Tiled(x, y, period, seed + static_cast<uint32_t>(i) * 131u);
        norm += amp;
        x *= 2;
        y *= 2;
        period *= 2;
        amp *= 0.5f;
    }
    return sum / norm;
}

}  // namespace apex
