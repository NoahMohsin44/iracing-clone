#pragma once
#include <cstdint>

namespace apex {

// Deterministic hashing / gradient noise used by procedural content (textures, terrain detail, placement).
uint32_t hash32(uint32_t x);
uint32_t hash2(int x, int y, uint32_t seed = 0);
float hashFloat(uint32_t h);  // [0, 1)

float valueNoise1(float x, uint32_t seed = 0);
float perlin2(float x, float y, uint32_t seed = 0);          // ~[-1, 1]
float fbm2(float x, float y, int octaves, float lacunarity = 2.0f, float gain = 0.5f, uint32_t seed = 0);
float perlin2Tiled(float x, float y, int period, uint32_t seed = 0);  // tiles every `period` units
float fbm2Tiled(float x, float y, int period, int octaves, uint32_t seed = 0);

struct Rng {
    uint64_t state;
    explicit Rng(uint64_t seed = 0x9E3779B97F4A7C15ull) : state(seed ? seed : 1) {}
    uint32_t next() {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return static_cast<uint32_t>(state >> 16);
    }
    float uniform() { return (next() & 0xFFFFFF) / 16777216.0f; }
    float range(float a, float b) { return a + (b - a) * uniform(); }
};

}  // namespace apex
