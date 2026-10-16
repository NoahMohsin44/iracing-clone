#include "world/Vegetation.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "core/Log.h"
#include "core/Noise.h"
#include "game/CameraRig.h"
#include "gfx/ProcTextures.h"

namespace apex {

namespace {
constexpr float kPi = 3.14159265358979f;
constexpr float kSpruceH = 24.0f;
constexpr float kBeechH = 22.0f;

void addCylinder(MeshData& m, glm::vec3 a, glm::vec3 b, float ra, float rb, int sides, float aoA, float aoB) {
    glm::vec3 axis = glm::normalize(b - a);
    glm::vec3 t = std::abs(axis.y) < 0.95f ? glm::normalize(glm::cross(axis, glm::vec3(0, 1, 0))) : glm::vec3(1, 0, 0);
    glm::vec3 bt = glm::cross(axis, t);
    float len = glm::length(b - a);
    uint32_t base = static_cast<uint32_t>(m.vertices.size());
    for (int i = 0; i <= sides; ++i) {
        float ang = 2.0f * kPi * i / sides;
        glm::vec3 n = t * std::cos(ang) + bt * std::sin(ang);
        for (int e = 0; e < 2; ++e) {
            Vertex v;
            v.pos = (e == 0 ? a : b) + n * (e == 0 ? ra : rb);
            v.normal = n;
            v.uv = glm::vec2(static_cast<float>(i) / sides * 2.0f, (e == 0 ? 0.0f : len) / 2.5f);
            v.color = glm::vec4(0.0f, e == 0 ? 0.0f : 0.15f, e == 0 ? aoA : aoB, 0.0f);
            m.vertices.push_back(v);
        }
    }
    for (int i = 0; i < sides; ++i) {
        uint32_t i0 = base + i * 2;
        m.addQuad(i0, i0 + 2, i0 + 3, i0 + 1);
    }
}

void addCard(MeshData& m, const glm::vec3 p[4], const glm::vec2 uv[4], const glm::vec3 n[4], const float sway[4],
             const float ao[4]) {
    uint32_t base = static_cast<uint32_t>(m.vertices.size());
    for (int i = 0; i < 4; ++i) {
        Vertex v;
        v.pos = p[i];
        v.normal = n[i];
        v.uv = uv[i];
        v.color = glm::vec4(1.0f, sway[i], ao[i], 0.0f);
        m.vertices.push_back(v);
    }
    m.addQuad(base, base + 1, base + 2, base + 3);
}

glm::vec2 atlas(const float r[4], float u, float v) { return {r[0] + (r[2] - r[0]) * u, r[1] + (r[3] - r[1]) * v}; }

MeshData buildSpruce(int lod) {
    MeshData m;
    Rng r(77 + lod);
    const float H = kSpruceH;
    addCylinder(m, {0, -0.5f, 0}, {0, H * 0.97f, 0}, 0.36f, 0.03f, lod == 0 ? 7 : 5, 0.55f, 0.85f);
    const int whorls = lod == 0 ? 46 : 16;
    const int per = lod == 0 ? 6 : 5;
    for (int i = 0; i < whorls; ++i) {
        float y = 1.6f + (H - 2.2f) * i / (whorls - 1);
        float frac = y / H;
        float L = std::max(0.9f, (H - y) * 0.34f + 0.4f) * (lod == 0 ? 1.0f : 1.15f);
        float offset = i * 2.39996f;
        for (int j = 0; j < per; ++j) {
            float a = offset + j * 2.0f * kPi / per + r.range(-0.25f, 0.25f);
            glm::vec3 dir(std::cos(a), 0.0f, std::sin(a));
            glm::vec3 perp(-std::sin(a), 0.0f, std::cos(a));
            float tilt = r.range(-0.55f, 0.55f);
            glm::vec3 side = glm::normalize(perp * std::cos(tilt) + glm::vec3(0, 1, 0) * std::sin(tilt));
            float droop = L * r.range(0.22f, 0.38f);
            glm::vec3 base = glm::vec3(0, y, 0) + dir * 0.12f;
            glm::vec3 tip = base + dir * L - glm::vec3(0, droop, 0);
            float w = L * 0.75f;
            glm::vec3 p[4] = {base - side * w * 0.3f, tip - side * w * 0.5f, tip + side * w * 0.5f, base + side * w * 0.3f};
            glm::vec2 uv[4] = {atlas(proctex::kSpruceBranch, 0.02f, 0.0f), atlas(proctex::kSpruceBranch, 0.98f, 0.0f),
                               atlas(proctex::kSpruceBranch, 0.98f, 1.0f), atlas(proctex::kSpruceBranch, 0.02f, 1.0f)};
            glm::vec3 nb = glm::normalize(glm::vec3(0, 1, 0) * 0.9f + dir * 0.25f);
            glm::vec3 nt = glm::normalize(dir * 0.85f + glm::vec3(0, 0.55f, 0));
            glm::vec3 n[4] = {nb, nt, nt, nb};
            float swayBase = std::pow(frac, 1.5f) * 0.3f, swayTip = std::pow(frac, 1.2f) * 0.6f + 0.4f;
            float sway[4] = {swayBase, swayTip, swayTip, swayBase};
            float aoIn = 0.25f + 0.45f * frac, aoOut = 0.65f + 0.35f * frac;
            float ao[4] = {aoIn, aoOut, aoOut, aoIn};
            addCard(m, p, uv, n, sway, ao);
        }
    }
    // leader at the top
    for (int k = 0; k < 2; ++k) {
        float a = k * kPi * 0.5f;
        glm::vec3 side(std::cos(a), 0, std::sin(a));
        glm::vec3 p[4] = {glm::vec3(0, H - 1.6f, 0) - side * 0.5f, glm::vec3(0, H + 0.4f, 0) - side * 0.15f,
                          glm::vec3(0, H + 0.4f, 0) + side * 0.15f, glm::vec3(0, H - 1.6f, 0) + side * 0.5f};
        glm::vec2 uv[4] = {atlas(proctex::kSpruceBranch, 0.1f, 0.0f), atlas(proctex::kSpruceBranch, 0.9f, 0.0f),
                           atlas(proctex::kSpruceBranch, 0.9f, 1.0f), atlas(proctex::kSpruceBranch, 0.1f, 1.0f)};
        glm::vec3 n[4] = {glm::vec3(0, 1, 0), glm::vec3(0, 1, 0), glm::vec3(0, 1, 0), glm::vec3(0, 1, 0)};
        float sway[4] = {0.5f, 1.0f, 1.0f, 0.5f}, ao[4] = {1, 1, 1, 1};
        addCard(m, p, uv, n, sway, ao);
    }
    return m;
}

MeshData buildBeech(int lod) {
    MeshData m;
    Rng r(991 + lod);
    const glm::vec3 crown(0.0f, 14.5f, 0.0f), radii(5.6f, 6.4f, 5.6f);
    addCylinder(m, {0, -0.5f, 0}, {0, 9.5f, 0}, 0.32f, 0.2f, lod == 0 ? 7 : 5, 0.5f, 0.6f);
    if (lod == 0) {
        for (int k = 0; k < 4; ++k) {
            float a = k * 1.7f + 0.3f;
            glm::vec3 d(std::cos(a) * 0.6f, 1.0f, std::sin(a) * 0.6f);
            addCylinder(m, {0, 9.0f, 0}, glm::vec3(0, 9.0f, 0) + glm::normalize(d) * 6.5f, 0.17f, 0.05f, 5, 0.5f, 0.7f);
        }
    }
    const int cards = lod == 0 ? 150 : 42;
    const float size = lod == 0 ? 3.8f : 5.6f;
    for (int i = 0; i < cards; ++i) {
        glm::vec3 d;
        do {
            d = glm::vec3(r.range(-1, 1), r.range(-0.7f, 1), r.range(-1, 1));
        } while (glm::dot(d, d) > 1.0f || glm::dot(d, d) < 0.05f);
        d = glm::normalize(d);
        glm::vec3 c = crown + d * radii * r.range(0.35f, 0.98f);
        glm::vec3 nrm = glm::normalize(d + glm::vec3(r.range(-0.4f, 0.4f), 0.3f, r.range(-0.4f, 0.4f)));
        glm::vec3 t = glm::normalize(glm::cross(nrm, std::abs(nrm.y) < 0.9f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0)));
        glm::vec3 b = glm::cross(nrm, t);
        float rot = r.range(0, 2 * kPi);
        glm::vec3 tt = t * std::cos(rot) + b * std::sin(rot);
        glm::vec3 bb = glm::cross(nrm, tt);
        float hs = size * 0.5f * r.range(0.8f, 1.2f);
        glm::vec3 p[4] = {c - tt * hs - bb * hs * 0.55f, c + tt * hs - bb * hs * 0.55f, c + tt * hs + bb * hs * 0.55f,
                          c - tt * hs + bb * hs * 0.55f};
        glm::vec2 uv[4] = {atlas(proctex::kBeechCluster, 0.02f, 0.02f), atlas(proctex::kBeechCluster, 0.98f, 0.02f),
                           atlas(proctex::kBeechCluster, 0.98f, 0.98f), atlas(proctex::kBeechCluster, 0.02f, 0.98f)};
        glm::vec3 n[4];
        float sway[4], ao[4];
        for (int k = 0; k < 4; ++k) {
            n[k] = glm::normalize((p[k] - crown) / radii + glm::vec3(0, 0.25f, 0));
            sway[k] = 0.5f + 0.5f * std::clamp((p[k].y - 9.0f) / 12.0f, 0.0f, 1.0f);
            ao[k] = 0.4f + 0.6f * (0.5f + 0.5f * n[k].y) * (0.6f + 0.4f * glm::length((p[k] - crown) / radii));
        }
        addCard(m, p, uv, n, sway, ao);
    }
    return m;
}

MeshData buildImpostor(const float region[4], float height, float halfWidthFrac, float vBottom, float vTop) {
    MeshData m;
    const float hw = height * halfWidthFrac;
    for (int k = 0; k < 2; ++k) {
        float a = k * kPi * 0.5f;
        glm::vec3 side(std::cos(a), 0, std::sin(a));
        glm::vec3 p[4] = {-side * hw, side * hw, side * hw + glm::vec3(0, height, 0), -side * hw + glm::vec3(0, height, 0)};
        glm::vec2 uv[4] = {atlas(region, 0.0f, vBottom), atlas(region, 1.0f, vBottom), atlas(region, 1.0f, vTop),
                           atlas(region, 0.0f, vTop)};
        glm::vec3 fwd = glm::cross(side, glm::vec3(0, 1, 0));
        glm::vec3 n[4] = {glm::normalize(-side + fwd * 0.3f + glm::vec3(0, 0.3f, 0)),
                          glm::normalize(side + fwd * 0.3f + glm::vec3(0, 0.3f, 0)),
                          glm::normalize(side + fwd * 0.3f + glm::vec3(0, 1.0f, 0)),
                          glm::normalize(-side + fwd * 0.3f + glm::vec3(0, 1.0f, 0))};
        float sway[4] = {0, 0, 0.6f, 0.6f}, ao[4] = {0.7f, 0.7f, 1, 1};
        addCard(m, p, uv, n, sway, ao);
    }
    return m;
}
}  // namespace

void Vegetation::build(const TrackData& track, int quality) {
    quality_ = quality;
    float density = 1.0f, spacing = 5.2f;
    switch (quality) {
        case 0: band_ = 55.0f; density = 0.55f; lod0_ = 35.0f; lod1_ = 160.0f; break;
        case 1: band_ = 95.0f; density = 0.8f; lod0_ = 55.0f; lod1_ = 220.0f; break;
        case 2: band_ = 140.0f; density = 1.0f; lod0_ = 75.0f; lod1_ = 300.0f; break;
        default: band_ = 210.0f; density = 1.0f; lod0_ = 110.0f; lod1_ = 420.0f; spacing = 4.8f; break;
    }

    // ---- meshes (impostor uvs measured from the painted atlas canvas)
    const float atlasH = 1024.0f;
    meshes_[0][0].upload(buildSpruce(0));
    meshes_[0][1].upload(buildSpruce(1));
    meshes_[0][2].upload(buildImpostor(proctex::kSpruceImpostor, kSpruceH, 0.516f, (1018.0f - 512.0f) / 512.0f,
                                       (522.0f - 512.0f) / 512.0f));
    meshes_[1][0].upload(buildBeech(0));
    meshes_[1][1].upload(buildBeech(1));
    meshes_[1][2].upload(buildImpostor(proctex::kBeechImpostor, kBeechH, 0.52f, (1018.0f - 512.0f) / 512.0f,
                                       (527.0f - 512.0f) / 512.0f));
    (void)atlasH;

    // ---- placement
    const Heightmap& hm = track.nearTerrain();
    const float cellSize = 64.0f;
    const int gw = static_cast<int>(hm.width() / cellSize) + 1;
    const int gh = static_cast<int>(hm.depth() / cellSize) + 1;
    std::vector<std::vector<Instance>> buckets[2];
    buckets[0].resize(static_cast<size_t>(gw) * gh);
    buckets[1].resize(static_cast<size_t>(gw) * gh);
    std::unordered_set<uint64_t> occupied;
    std::vector<glm::vec3> tvSpots;
    for (int i = 0; i < CameraRig::tvCameraCount(track); ++i) {
        int side = 0;
        tvSpots.push_back(CameraRig::tvCameraSpot(track, i, side));
    }
    auto key = [](float x, float z) {
        int64_t ix = static_cast<int64_t>(std::floor(x / 3.6f)), iz = static_cast<int64_t>(std::floor(z / 3.6f));
        return static_cast<uint64_t>((ix + 100000) * 1000003LL + (iz + 100000));
    };
    Rng rng(4711);
    int count = 0;
    for (float s = 0.0f; s < track.length(); s += spacing) {
        for (int side = -1; side <= 1; side += 2) {
            const float start = track.barrierAt(s, side) + 2.5f;
            for (float lat = start; lat < start + band_; lat += spacing) {
                float ss = s + rng.range(-0.5f, 0.5f) * spacing;
                float ll = lat + rng.range(-0.5f, 0.5f) * spacing;
                glm::vec3 p = track.surfacePoint(ss, side * ll);
                if (!hm.contains(p.x, p.z)) continue;
                glm::vec3 lc = track.landcoverAt(p.x, p.z);
                float forest = lc.x;
                float prob = forest * density;
                if (forest < 0.15f) prob = lc.y > 0.5f ? 0.006f : 0.02f;  // lone trees and hedges
                if (lc.z > 0.5f) prob *= 0.15f;                          // villages
                if (rng.uniform() > prob) continue;
                uint64_t k = key(p.x, p.z);
                if (occupied.count(k)) continue;
                bool nearCamera = false;
                for (const auto& c : tvSpots) {
                    float dx = c.x - p.x, dz = c.z - p.z;
                    if (dx * dx + dz * dz < 11.0f * 11.0f) nearCamera = true;
                }
                if (nearCamera) continue;
                // not too close to any other part of the circuit
                TrackLocation loc = track.locate(p, -1);
                int lside = loc.lateral >= 0 ? 1 : -1;
                if (std::abs(loc.lateral) < track.barrierAt(loc.s, lside) + 2.0f) continue;
                occupied.insert(k);
                float patch = fbm2(p.x * 0.004f, p.z * 0.004f, 3, 2.0f, 0.5f, 21);
                int type = (patch + rng.range(-0.15f, 0.15f) > -0.08f) ? 0 : 1;
                float scale = rng.range(0.72f, 1.22f);
                float rot = rng.range(0.0f, 2.0f * kPi);
                Instance inst;
                inst.posScale = glm::vec4(p.x, track.terrainHeight(p.x, p.z) - 0.2f, p.z, scale);
                inst.rotTint = glm::vec4(std::cos(rot), std::sin(rot), rng.uniform(), 0.0f);
                int cx = std::clamp(static_cast<int>((p.x - hm.x0) / cellSize), 0, gw - 1);
                int cz = std::clamp(static_cast<int>((p.z - hm.z0) / cellSize), 0, gh - 1);
                buckets[type][static_cast<size_t>(cz) * gw + cx].push_back(inst);
                ++count;
            }
        }
    }

    // ---- pack per cell
    std::vector<Instance> all;
    all.reserve(count);
    cells_.clear();
    for (int cz = 0; cz < gh; ++cz) {
        for (int cx = 0; cx < gw; ++cx) {
            size_t idx = static_cast<size_t>(cz) * gw + cx;
            if (buckets[0][idx].empty() && buckets[1][idx].empty()) continue;
            Cell c;
            float ysum = 0.0f;
            int n = 0;
            for (int t = 0; t < 2; ++t) {
                c.start[t] = static_cast<uint32_t>(all.size());
                c.count[t] = static_cast<uint32_t>(buckets[t][idx].size());
                for (auto& inst : buckets[t][idx]) {
                    all.push_back(inst);
                    ysum += inst.posScale.y;
                    ++n;
                }
            }
            c.centre = glm::vec3(hm.x0 + (cx + 0.5f) * cellSize, ysum / std::max(n, 1) + 12.0f, hm.z0 + (cz + 0.5f) * cellSize);
            c.radius = cellSize * 0.75f + 22.0f;
            cells_.push_back(c);
        }
    }
    if (instanceBuffer_) glDeleteBuffers(1, &instanceBuffer_);
    glCreateBuffers(1, &instanceBuffer_);
    glNamedBufferStorage(instanceBuffer_, std::max<size_t>(all.size(), 1) * sizeof(Instance), all.empty() ? nullptr : all.data(), 0);
    for (auto& type : meshes_)
        for (auto& mesh : type) mesh.setInstanceBuffer(instanceBuffer_, 2);
    stats_.instances = static_cast<int>(all.size());
    LOGI("Vegetation: %d trees in %zu cells (band %.0f m)", stats_.instances, cells_.size(), band_);
}

void Vegetation::draw(const glm::dvec3& camPos, const Frustum& frustum, float maxDistance, int minLod) {
    stats_.drawnCells = 0;
    stats_.drawCalls = 0;
    for (const Cell& c : cells_) {
        glm::vec3 rel = glm::vec3(glm::dvec3(c.centre) - camPos);
        float d = glm::length(rel);
        if (d - c.radius > maxDistance) continue;
        if (!frustum.sphere(rel, c.radius)) continue;
        int lod = d < lod0_ ? 0 : (d < lod1_ ? 1 : 2);
        lod = std::max(lod, minLod);
        ++stats_.drawnCells;
        for (int t = 0; t < 2; ++t) {
            if (!c.count[t]) continue;
            meshes_[t][lod].drawInstanced(static_cast<GLsizei>(c.count[t]), c.start[t]);
            ++stats_.drawCalls;
        }
    }
}

}  // namespace apex
