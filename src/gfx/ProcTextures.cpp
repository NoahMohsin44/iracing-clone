#include "gfx/ProcTextures.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <glm/glm.hpp>

#include "core/Noise.h"

namespace apex::proctex {

namespace {
constexpr float kPi = 3.14159265358979f;

// Premultiplied RGBA float canvas with simple anti-aliased primitives.
struct Canvas {
    int w, h;
    std::vector<glm::vec4> px;
    Canvas(int w_, int h_) : w(w_), h(h_), px(static_cast<size_t>(w_) * h_, glm::vec4(0.0f)) {}

    void blend(int x, int y, const glm::vec3& c, float a) {
        if (x < 0 || y < 0 || x >= w || y >= h || a <= 0.0f) return;
        glm::vec4& d = px[static_cast<size_t>(y) * w + x];
        d = glm::vec4(c * a, a) + d * (1.0f - a);
    }

    void line(glm::vec2 a, glm::vec2 b, float width, const glm::vec3& c, float alpha = 1.0f) {
        int x0 = static_cast<int>(std::floor(std::min(a.x, b.x) - width)), x1 = static_cast<int>(std::ceil(std::max(a.x, b.x) + width));
        int y0 = static_cast<int>(std::floor(std::min(a.y, b.y) - width)), y1 = static_cast<int>(std::ceil(std::max(a.y, b.y) + width));
        glm::vec2 ab = b - a;
        float len2 = std::max(glm::dot(ab, ab), 1e-6f);
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                glm::vec2 p(x + 0.5f, y + 0.5f);
                float t = std::clamp(glm::dot(p - a, ab) / len2, 0.0f, 1.0f);
                float d = glm::length(p - (a + ab * t));
                float wt = width * (1.0f - 0.55f * t);  // tapering
                float cov = std::clamp(wt * 0.5f - d + 0.5f, 0.0f, 1.0f);
                blend(x, y, c, cov * alpha);
            }
    }

    void ellipse(glm::vec2 c, float rx, float ry, float ang, const glm::vec3& col, float alpha = 1.0f) {
        float r = std::max(rx, ry) + 1.0f;
        float ca = std::cos(ang), sa = std::sin(ang);
        for (int y = static_cast<int>(c.y - r); y <= static_cast<int>(c.y + r); ++y)
            for (int x = static_cast<int>(c.x - r); x <= static_cast<int>(c.x + r); ++x) {
                glm::vec2 p(x + 0.5f - c.x, y + 0.5f - c.y);
                glm::vec2 q(ca * p.x + sa * p.y, -sa * p.x + ca * p.y);
                // pointed leaf: narrower towards the tips
                float taper = 1.0f - 0.35f * std::abs(q.x / rx);
                float e = std::sqrt((q.x * q.x) / (rx * rx) + (q.y * q.y) / (ry * ry * taper * taper));
                float cov = std::clamp((1.0f - e) * std::min(rx, ry) + 0.5f, 0.0f, 1.0f);
                blend(x, y, col, cov * alpha);
            }
    }
};

glm::vec3 needleColour(Rng& r, float shade) {
    glm::vec3 base(0.055f, 0.105f, 0.045f);
    glm::vec3 tip(0.10f, 0.17f, 0.065f);
    return glm::mix(base, tip, r.uniform()) * shade;
}

// One spruce spray: stem with drooping twigs, all covered in needles.
void paintSpruceBranch(Canvas& cv, glm::vec2 origin, float length, float scale, Rng& r) {
    auto needles = [&](glm::vec2 a, glm::vec2 b, float density, float nlen, float shade) {
        glm::vec2 d = b - a;
        float L = glm::length(d);
        glm::vec2 t = d / L, n(-t.y, t.x);
        int count = static_cast<int>(L / density);
        for (int i = 0; i < count; ++i) {
            float f = (i + r.uniform()) / count;
            glm::vec2 p = a + d * f;
            float len = nlen * (0.75f + 0.5f * r.uniform()) * (1.0f - 0.35f * f);
            for (int side = -1; side <= 1; side += 2) {
                float ang = (0.85f + 0.35f * r.uniform()) * side;
                glm::vec2 dir = t * std::cos(ang) + n * std::sin(ang);
                cv.line(p, p + dir * len, 2.2f * scale, needleColour(r, shade * (0.8f + 0.4f * r.uniform())));
            }
        }
    };
    glm::vec2 end = origin + glm::vec2(length, 0.0f);
    // stem (slight curve)
    glm::vec2 prev = origin;
    const int segs = 12;
    for (int i = 1; i <= segs; ++i) {
        float f = static_cast<float>(i) / segs;
        glm::vec2 p = origin + glm::vec2(length * f, std::sin(f * kPi) * 10.0f * scale);
        cv.line(prev, p, 4.0f * scale * (1.0f - 0.6f * f), glm::vec3(0.16f, 0.10f, 0.06f));
        prev = p;
    }
    // twigs
    for (float f = 0.08f; f < 0.9f; f += 0.075f + 0.03f * r.uniform()) {
        glm::vec2 p = origin + glm::vec2(length * f, std::sin(f * kPi) * 10.0f * scale);
        for (int side = -1; side <= 1; side += 2) {
            float tl = (1.0f - f) * length * 0.32f + 25.0f * scale;
            float ang = (0.55f + 0.25f * r.uniform()) * side;
            glm::vec2 tip = p + glm::vec2(std::cos(ang), std::sin(ang)) * tl;
            cv.line(p, tip, 2.0f * scale, glm::vec3(0.14f, 0.09f, 0.05f));
            needles(p, tip, 2.6f * scale, 15.0f * scale, side < 0 ? 0.85f : 1.0f);
        }
    }
    needles(origin, end, 2.4f * scale, 22.0f * scale, 1.0f);
}

void paintBeechCluster(Canvas& cv, glm::vec2 c, glm::vec2 radius, int leaves, float scale, Rng& r) {
    // twigs
    for (int i = 0; i < 9; ++i) {
        float a = r.range(0, 2 * kPi);
        cv.line(c, c + glm::vec2(std::cos(a) * radius.x, std::sin(a) * radius.y) * 0.8f, 3.0f * scale,
                glm::vec3(0.18f, 0.13f, 0.08f));
    }
    for (int i = 0; i < leaves; ++i) {
        float a = r.range(0, 2 * kPi);
        float d = std::sqrt(r.uniform());
        glm::vec2 p = c + glm::vec2(std::cos(a) * radius.x, std::sin(a) * radius.y) * d;
        float shade = 0.75f + 0.45f * r.uniform();
        glm::vec3 col = glm::mix(glm::vec3(0.11f, 0.22f, 0.045f), glm::vec3(0.20f, 0.32f, 0.07f), r.uniform()) * shade;
        float ang = a + r.range(-0.8f, 0.8f);
        float rx = r.range(22.0f, 34.0f) * scale, ry = rx * r.range(0.45f, 0.6f);
        cv.ellipse(p, rx, ry, ang, col);
        glm::vec2 dir(std::cos(ang), std::sin(ang));
        cv.line(p - dir * rx * 0.85f, p + dir * rx * 0.85f, 1.2f * scale, col * 0.65f, 0.7f);
    }
}

void paintSpruceImpostor(Canvas& cv, glm::ivec2 org, int size, Rng& r) {
    const float cx = org.x + size * 0.5f;
    const float bottom = org.y + size - 6.0f, top = org.y + 10.0f;
    const float H = bottom - top;
    cv.line({cx, bottom}, {cx, top + 20}, 7.0f, glm::vec3(0.18f, 0.12f, 0.08f));
    for (float f = 0.12f; f < 0.98f; f += 0.018f) {
        float y = bottom - H * f;
        float half = (1.0f - f) * size * 0.40f + 6.0f;
        for (int side = -1; side <= 1; side += 2) {
            float len = half * r.range(0.75f, 1.05f);
            glm::vec2 a(cx, y), b(cx + side * len, y + len * 0.35f);
            float shade = 0.55f + 0.5f * f;
            int blobs = static_cast<int>(len / 3.5f);
            for (int i = 0; i < blobs; ++i) {
                float t = (i + r.uniform()) / blobs;
                glm::vec2 p = a + (b - a) * t + glm::vec2(r.range(-2, 2), r.range(-3, 3));
                float rad = 3.5f * (1.0f - 0.4f * t) + r.uniform() * 2.0f;
                cv.ellipse(p, rad, rad * 0.7f, r.range(0, kPi), needleColour(r, shade * (0.7f + 0.6f * t)));
            }
        }
    }
}

void paintBeechImpostor(Canvas& cv, glm::ivec2 org, int size, Rng& r) {
    const float cx = org.x + size * 0.5f;
    const float bottom = org.y + size - 6.0f;
    cv.line({cx, bottom}, {cx, bottom - size * 0.5f}, 10.0f, glm::vec3(0.30f, 0.27f, 0.22f));
    glm::vec2 crown(cx, bottom - size * 0.56f);
    for (int i = 0; i < 1400; ++i) {
        float a = r.range(0, 2 * kPi);
        float d = std::sqrt(r.uniform());
        glm::vec2 p = crown + glm::vec2(std::cos(a) * size * 0.40f, std::sin(a) * size * 0.40f) * d;
        float lit = 0.6f + 0.6f * std::clamp((crown.y - p.y) / (size * 0.4f) * 0.5f + 0.5f, 0.0f, 1.0f);
        glm::vec3 col = glm::mix(glm::vec3(0.10f, 0.20f, 0.04f), glm::vec3(0.18f, 0.30f, 0.06f), r.uniform()) * lit;
        cv.ellipse(p, 5.0f, 3.0f, r.range(0, kPi), col);
    }
}
}  // namespace

Texture foliageAtlas() {
    const int W = 2048, H = 1024;
    Canvas cv(W, H);
    Rng r(1234);
    // spruce branch spray (0..1024, 0..512)
    paintSpruceBranch(cv, {20.0f, 256.0f}, 980.0f, 1.0f, r);
    // beech leaf cluster (1024..2048, 0..512)
    paintBeechCluster(cv, {1536.0f, 256.0f}, {470.0f, 230.0f}, 520, 1.0f, r);
    paintSpruceImpostor(cv, {0, 512}, 512, r);
    paintBeechImpostor(cv, {512, 512}, 512, r);

    // un-premultiply; give transparent texels the average foliage colour so filtering is clean
    glm::vec3 fill(0.07f, 0.13f, 0.05f);
    std::vector<glm::vec4> level(cv.px.size());
    for (size_t i = 0; i < cv.px.size(); ++i) {
        const glm::vec4& p = cv.px[i];
        glm::vec3 c = p.w > 1e-3f ? glm::vec3(p) / p.w : fill;
        level[i] = glm::vec4(c, p.w);
    }

    Texture tex;
    const int mips = Texture::mipCount(W, H);
    tex.create2D(W, H, GL_SRGB8_ALPHA8, mips);
    auto upload = [&](const std::vector<glm::vec4>& data, int lw, int lh, int lvl) {
        std::vector<uint8_t> bytes(static_cast<size_t>(lw) * lh * 4);
        for (size_t i = 0; i < data.size(); ++i) {
            glm::vec3 c = glm::pow(glm::clamp(glm::vec3(data[i]), 0.0f, 1.0f), glm::vec3(1.0f / 2.2f));
            bytes[i * 4 + 0] = static_cast<uint8_t>(c.x * 255.0f + 0.5f);
            bytes[i * 4 + 1] = static_cast<uint8_t>(c.y * 255.0f + 0.5f);
            bytes[i * 4 + 2] = static_cast<uint8_t>(c.z * 255.0f + 0.5f);
            bytes[i * 4 + 3] = static_cast<uint8_t>(std::clamp(data[i].w, 0.0f, 1.0f) * 255.0f + 0.5f);
        }
        glTextureSubImage2D(tex.id(), lvl, 0, 0, lw, lh, GL_RGBA, GL_UNSIGNED_BYTE, bytes.data());
    };
    // coverage of each atlas quadrant at level 0 (alpha >= 0.5)
    auto coverage = [](const std::vector<glm::vec4>& d, int lw, int x0, int y0, int x1, int y1, float scale) {
        size_t n = 0, c = 0;
        for (int y = y0; y < y1; ++y)
            for (int x = x0; x < x1; ++x) {
                ++n;
                if (d[static_cast<size_t>(y) * lw + x].w * scale >= 0.5f) ++c;
            }
        return n ? static_cast<float>(c) / n : 0.0f;
    };
    const int regions[4][4] = {{0, 0, 1024, 512}, {1024, 0, 2048, 512}, {0, 512, 512, 1024}, {512, 512, 1024, 1024}};
    float target[4];
    for (int k = 0; k < 4; ++k) target[k] = coverage(level, W, regions[k][0], regions[k][1], regions[k][2], regions[k][3], 1.0f);
    upload(level, W, H, 0);

    int lw = W, lh = H;
    for (int m = 1; m < mips; ++m) {
        int nw = std::max(1, lw / 2), nh = std::max(1, lh / 2);
        std::vector<glm::vec4> next(static_cast<size_t>(nw) * nh);
        for (int y = 0; y < nh; ++y)
            for (int x = 0; x < nw; ++x) {
                glm::vec4 acc(0.0f);
                float wa = 0.0f;
                for (int dy = 0; dy < 2; ++dy)
                    for (int dx = 0; dx < 2; ++dx) {
                        const glm::vec4& p = level[static_cast<size_t>(std::min(y * 2 + dy, lh - 1)) * lw + std::min(x * 2 + dx, lw - 1)];
                        acc += glm::vec4(glm::vec3(p) * p.w, p.w);
                        wa += p.w;
                    }
                glm::vec3 c = wa > 1e-4f ? glm::vec3(acc) / wa : fill;
                next[static_cast<size_t>(y) * nw + x] = glm::vec4(c, acc.w * 0.25f);
            }
        // preserve alpha-test coverage per region
        int div = 1 << m;
        for (int k = 0; k < 4; ++k) {
            int x0 = regions[k][0] / div, y0 = regions[k][1] / div, x1 = std::max(x0 + 1, regions[k][2] / div),
                y1 = std::max(y0 + 1, regions[k][3] / div);
            float lo = 0.5f, hi = 4.0f;
            for (int it = 0; it < 12; ++it) {
                float mid = 0.5f * (lo + hi);
                if (coverage(next, nw, x0, y0, x1, y1, mid) < target[k]) lo = mid;
                else hi = mid;
            }
            float sc = 0.5f * (lo + hi);
            for (int y = y0; y < y1 && y < nh; ++y)
                for (int x = x0; x < x1 && x < nw; ++x) {
                    float& a = next[static_cast<size_t>(y) * nw + x].w;
                    a = std::min(1.0f, a * sc);
                }
        }
        upload(next, nw, nh, m);
        level.swap(next);
        lw = nw;
        lh = nh;
    }
    tex.setFilter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
    tex.setWrap(GL_CLAMP_TO_EDGE);
    tex.setAnisotropy(8.0f);
    return tex;
}

}  // namespace apex::proctex
