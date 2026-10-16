#include "world/TrackMesh.h"

#include <algorithm>
#include <cmath>

#include "core/Log.h"

namespace apex {

namespace {
constexpr float kChunkLength = 200.0f;
constexpr float kStep = 1.0f;

float groundAt(const TrackData& t, float s, float lat) {
    int side = lat >= 0 ? 1 : -1;
    float hw = t.halfWidthAt(s, side);
    float aw = std::abs(lat);
    if (aw <= hw) return t.roadHeight(s, lat);
    if (aw <= hw + TrackData::kVergeWidth) {
        float u = (aw - hw) / TrackData::kVergeWidth;
        const TrackSample& a = t.sample(static_cast<int>(t.wrapS(s) / t.spacing()));
        const TrackSample& b = t.sample(static_cast<int>(t.wrapS(s) / t.spacing()) + 1);
        float f = t.wrapS(s) / t.spacing() - std::floor(t.wrapS(s) / t.spacing());
        float outer = side > 0 ? glm::mix(a.vergeOuterRightY, b.vergeOuterRightY, f) : glm::mix(a.vergeOuterLeftY, b.vergeOuterLeftY, f);
        return glm::mix(t.roadHeight(s, side * hw), outer, u);
    }
    glm::vec3 p = t.surfacePoint(s, lat);
    return t.terrainHeight(p.x, p.z);
}

void addBox(MeshData& m, const glm::vec3& c, const glm::vec3& ax, const glm::vec3& ay, const glm::vec3& az) {
    // ax/ay/az are half-extent vectors
    const glm::vec3 n[6] = {glm::normalize(ax), -glm::normalize(ax), glm::normalize(ay), -glm::normalize(ay), glm::normalize(az), -glm::normalize(az)};
    const glm::vec3 u[6] = {ay, ay, az, az, ax, ax};
    const glm::vec3 v[6] = {az, -az, ax, -ax, ay, -ay};
    const glm::vec3 o[6] = {ax, -ax, ay, -ay, az, -az};
    for (int f = 0; f < 6; ++f) {
        uint32_t b = static_cast<uint32_t>(m.vertices.size());
        glm::vec3 corners[4] = {c + o[f] - u[f] - v[f], c + o[f] + u[f] - v[f], c + o[f] + u[f] + v[f], c + o[f] - u[f] + v[f]};
        for (int k = 0; k < 4; ++k) {
            Vertex vx;
            vx.pos = corners[k];
            vx.normal = n[f];
            vx.uv = glm::vec2(k == 1 || k == 2, k >= 2);
            vx.tangent = glm::vec4(glm::normalize(u[f]), 1.0f);
            m.vertices.push_back(vx);
        }
        m.addQuad(b, b + 1, b + 2, b + 3);
    }
}
}  // namespace

void TrackMesh::build(const TrackData& t) {
    chunks_.clear();
    const float L = t.length();
    const int nChunks = static_cast<int>(std::ceil(L / kChunkLength));
    size_t triCount = 0;
    for (int ci = 0; ci < nChunks; ++ci) {
        Chunk ch;
        ch.s0 = ci * kChunkLength;
        const float s1 = std::min(L, ch.s0 + kChunkLength);
        ch.origin = glm::dvec3(t.centreAt(ch.s0));
        const glm::vec3 org(ch.origin);
        MeshData road, verge, rails, posts;

        const int stations = static_cast<int>(std::round((s1 - ch.s0) / kStep)) + 1;
        const float latFrac[] = {-1.0f, -0.66f, -0.33f, 0.0f, 0.33f, 0.66f, 1.0f};
        const int cols = 7;
        std::vector<bool> kerbL(stations), kerbR(stations);
        for (int k = 0; k < stations; ++k) {
            const float s = ch.s0 + k * kStep;
            const glm::vec3 r = t.rightAt(s);
            const float hwL = t.halfWidthAt(s, -1), hwR = t.halfWidthAt(s, 1);
            const float line = t.racingLineAt(s);
            // ---- tarmac
            for (int c = 0; c < cols; ++c) {
                float lat = latFrac[c] < 0 ? latFrac[c] * hwL : latFrac[c] * hwR;
                Vertex v;
                v.pos = t.surfacePoint(s, lat) - org;
                v.uv = glm::vec2(lat, s - ch.s0);
                v.color = glm::vec4(line, hwL, hwR, 0.0f);
                v.tangent = glm::vec4(r, 1.0f);
                road.vertices.push_back(v);
            }
            kerbL[k] = t.kerbAt(s, -1);
            kerbR[k] = t.kerbAt(s, 1);
        }
        for (int k = 0; k + 1 < stations; ++k) {
            for (int c = 0; c + 1 < cols; ++c) {
                uint32_t a = k * cols + c, b = a + 1, d = (k + 1) * cols + c, e = d + 1;
                road.addTri(a, b, e);
                road.addTri(a, e, d);
            }
        }
        road.computeNormals();

        // ---- kerbs (separate strips laid over the tarmac edge)
        const float kerbU[] = {0.0f, 0.12f, 0.28f, 0.6f, 0.88f, 1.0f};
        for (int side : {-1, 1}) {
            const auto& mask = side < 0 ? kerbL : kerbR;
            for (int k = 0; k + 1 < stations; ++k) {
                if (!mask[k] || !mask[k + 1]) continue;
                uint32_t base = static_cast<uint32_t>(road.vertices.size());
                for (int kk = 0; kk < 2; ++kk) {
                    float s = ch.s0 + (k + kk) * kStep;
                    float hw = t.halfWidthAt(s, side);
                    glm::vec3 r = t.rightAt(s);
                    for (float u : kerbU) {
                        float aw = hw - TrackData::kKerbInner + u * (TrackData::kKerbInner + TrackData::kKerbOuter);
                        float lat = side * aw;
                        glm::vec3 p = t.surfacePoint(s, side * std::min(aw, hw));
                        p += r * (lat - side * std::min(aw, hw));
                        if (aw > hw) p.y -= (aw - hw) * 0.02f;
                        p.y += t.kerbProfile(s, u) * 0.85f + 0.004f;
                        Vertex v;
                        v.pos = p - org;
                        v.uv = glm::vec2(lat, s - ch.s0);
                        v.color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
                        v.tangent = glm::vec4(r, 1.0f);
                        road.vertices.push_back(v);
                    }
                }
                const uint32_t n = 6;
                for (uint32_t c = 0; c + 1 < n; ++c) {
                    uint32_t a = base + c, b = a + 1, d = base + n + c, e = d + 1;
                    if (side > 0) {
                        road.addTri(a, b, e);
                        road.addTri(a, e, d);
                    } else {
                        road.addTri(a, e, b);
                        road.addTri(a, d, e);
                    }
                }
            }
        }
        // kerb normals: recompute only for the kerb vertices (tarmac already has smooth normals)
        {
            MeshData tmp = road;
            tmp.computeNormals();
            for (size_t i = stations * cols; i < road.vertices.size(); ++i) {
                road.vertices[i].normal = tmp.vertices[i].normal;
                if (road.vertices[i].normal.y < 0) road.vertices[i].normal = -road.vertices[i].normal;
            }
        }
        road.computeTangents();

        // ---- verges (grass between tarmac and terrain)
        const float vergeU[] = {0.0f, 0.15f, 0.4f, 0.7f, 1.0f, 1.15f};
        const int vc = 6;
        for (int side : {-1, 1}) {
            uint32_t base = static_cast<uint32_t>(verge.vertices.size());
            for (int k = 0; k < stations; ++k) {
                float s = ch.s0 + k * kStep;
                float hw = t.halfWidthAt(s, side);
                for (float u : vergeU) {
                    float aw = hw + u * TrackData::kVergeWidth;
                    float y = groundAt(t, s, side * std::min(aw, hw + TrackData::kVergeWidth));
                    if (u > 1.0f) y -= 0.25f;
                    glm::vec3 p = t.surfacePoint(s, side * aw);
                    p.y = y - (u == 0.0f ? 0.01f : 0.0f);
                    Vertex v;
                    v.pos = p - org;
                    v.uv = glm::vec2(side * aw, s - ch.s0);
                    verge.vertices.push_back(v);
                }
            }
            for (int k = 0; k + 1 < stations; ++k) {
                for (int c = 0; c + 1 < vc; ++c) {
                    uint32_t a = base + k * vc + c, b = a + 1, d = base + (k + 1) * vc + c, e = d + 1;
                    if (side > 0) {
                        verge.addTri(a, b, e);
                        verge.addTri(a, e, d);
                    } else {
                        verge.addTri(a, e, b);
                        verge.addTri(a, d, e);
                    }
                }
            }
        }
        verge.computeNormals();
        for (auto& v : verge.vertices)
            if (v.normal.y < 0) v.normal = -v.normal;

        // ---- armco: two W-beam rails per side, posts every 4 m
        for (int side : {-1, 1}) {
            uint32_t base = static_cast<uint32_t>(rails.vertices.size());
            const float heights[2][2] = {{0.42f, 0.73f}, {0.80f, 1.11f}};
            const int perStation = 2 * 3;  // two beams x (bottom, top, back-top)
            for (int k = 0; k < stations; ++k) {
                float s = ch.s0 + k * kStep;
                float b = t.barrierAt(s, side);
                glm::vec3 r = t.rightAt(s);
                glm::vec3 p = t.surfacePoint(s, side * b);
                float gy = groundAt(t, s, side * b);
                glm::vec3 face = -r * static_cast<float>(side);  // faces the track
                for (int beam = 0; beam < 2; ++beam) {
                    for (int e = 0; e < 3; ++e) {
                        Vertex v;
                        float h = e == 0 ? heights[beam][0] : heights[beam][1];
                        glm::vec3 pos(p.x, gy + h, p.z);
                        if (e == 2) pos -= face * 0.07f;
                        v.pos = pos - org;
                        v.normal = e == 2 ? glm::vec3(0, 1, 0) : face;
                        v.uv = glm::vec2(s, e == 0 ? 0.0f : 1.0f);
                        v.tangent = glm::vec4(glm::cross(glm::vec3(0, 1, 0), face), 1.0f);
                        rails.vertices.push_back(v);
                    }
                }
                if (static_cast<int>(s) % 4 == 0 && std::abs(s - std::round(s)) < 0.01f) {
                    glm::vec3 pc(p.x, gy + 0.6f, p.z);
                    pc -= face * 0.12f;
                    glm::vec3 tdir = glm::normalize(glm::cross(glm::vec3(0, 1, 0), face));
                    addBox(posts, pc - org, face * 0.05f, glm::vec3(0, 0.62f, 0), tdir * 0.05f);
                }
            }
            for (int k = 0; k + 1 < stations; ++k) {
                for (int beam = 0; beam < 2; ++beam) {
                    uint32_t a = base + k * perStation + beam * 3;
                    uint32_t d = base + (k + 1) * perStation + beam * 3;
                    rails.addQuad(a, d, d + 1, a + 1);      // front face
                    rails.addQuad(a + 1, d + 1, d + 2, a + 2);  // top lip
                }
            }
        }

        ch.road.upload(road);
        ch.verge.upload(verge);
        ch.rails.upload(rails);
        ch.posts.upload(posts);
        glm::vec3 mn = road.boundsMin(), mx = road.boundsMax();
        mn = glm::min(mn, rails.boundsMin());
        mx = glm::max(mx, rails.boundsMax());
        ch.centre = glm::vec3(ch.origin) + (mn + mx) * 0.5f;
        ch.radius = glm::length(mx - mn) * 0.5f + 2.0f;
        triCount += (road.indices.size() + verge.indices.size() + rails.indices.size() + posts.indices.size()) / 3;
        chunks_.push_back(std::move(ch));
    }
    LOGI("Track mesh: %zu chunks, %zu triangles", chunks_.size(), triCount);
}

void TrackMesh::visible(const glm::dvec3& camPos, const Frustum& f, float maxDistance, std::vector<int>& out) const {
    out.clear();
    for (int i = 0; i < static_cast<int>(chunks_.size()); ++i) {
        const Chunk& c = chunks_[i];
        glm::vec3 rel = glm::vec3(glm::dvec3(c.centre) - camPos);
        if (glm::length(rel) - c.radius > maxDistance) continue;
        if (!f.sphere(rel, c.radius)) continue;
        out.push_back(i);
    }
}

}  // namespace apex
