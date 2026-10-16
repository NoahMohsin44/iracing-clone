#include "world/Scenery.h"

#include <algorithm>
#include <cmath>
#include <map>

#include "core/Json.h"
#include "core/Log.h"
#include "core/Paths.h"

namespace apex {

namespace {
constexpr float kCell = 400.0f;

float hashf(float x, float z) {
    float h = std::sin(x * 12.9898f + z * 78.233f) * 43758.5453f;
    return h - std::floor(h);
}

struct Builder {
    std::map<std::pair<int, int>, MeshData> cells;

    MeshData& cellFor(const glm::vec3& p) {
        return cells[{static_cast<int>(std::floor(p.x / kCell)), static_cast<int>(std::floor(p.z / kCell))}];
    }

    // quad a-b-c-d (counter-clockwise seen from the front), flat shaded
    void quad(MeshData& m, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d, glm::vec3 col, float kind,
              glm::vec2 uva = {0, 0}, glm::vec2 uvb = {0, 0}, glm::vec2 uvc = {0, 0}, glm::vec2 uvd = {0, 0}) {
        glm::vec3 n = glm::normalize(glm::cross(b - a, d - a));
        glm::vec3 t = glm::normalize(b - a);
        uint32_t base = static_cast<uint32_t>(m.vertices.size());
        const glm::vec3 p[4] = {a, b, c, d};
        const glm::vec2 uv[4] = {uva, uvb, uvc, uvd};
        for (int k = 0; k < 4; ++k) {
            Vertex v;
            v.pos = p[k];
            v.normal = n;
            v.uv = uv[k];
            v.tangent = glm::vec4(t, 1.0f);
            v.color = glm::vec4(col, kind);
            m.vertices.push_back(v);
        }
        m.addQuad(base, base + 1, base + 2, base + 3);
    }

    // same, but wound so the face looks along `facing`
    void quadFacing(MeshData& m, glm::vec3 facing, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d, glm::vec3 col, float kind,
                    glm::vec2 uva = {0, 0}, glm::vec2 uvb = {0, 0}, glm::vec2 uvc = {0, 0}, glm::vec2 uvd = {0, 0}) {
        if (glm::dot(glm::cross(b - a, d - a), facing) < 0.0f) quad(m, b, a, d, c, col, kind, uvb, uva, uvd, uvc);
        else quad(m, a, b, c, d, col, kind, uva, uvb, uvc, uvd);
    }

    void tri(MeshData& m, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 col, float kind, glm::vec3 facing = glm::vec3(0)) {
        if (glm::dot(facing, facing) > 0.0f && glm::dot(glm::cross(b - a, c - a), facing) < 0.0f) std::swap(b, c);
        glm::vec3 n = glm::normalize(glm::cross(b - a, c - a));
        uint32_t base = static_cast<uint32_t>(m.vertices.size());
        for (const glm::vec3& p : {a, b, c}) {
            Vertex v;
            v.pos = p;
            v.normal = n;
            v.uv = glm::vec2(p.x, p.z);
            v.tangent = glm::vec4(glm::normalize(b - a), 1.0f);
            v.color = glm::vec4(col, kind);
            m.vertices.push_back(v);
        }
        m.addTri(base, base + 1, base + 2);
    }

    // axis-aligned-ish box from a centre, an along-track direction and half extents
    void box(MeshData& m, glm::vec3 c, glm::vec3 fwd, glm::vec3 half, glm::vec3 col, float kind) {
        glm::vec3 up(0, 1, 0);
        glm::vec3 f = glm::normalize(glm::vec3(fwd.x, 0, fwd.z)) * half.z;
        glm::vec3 r = glm::normalize(glm::cross(f, up)) * half.x;
        glm::vec3 u = up * half.y;
        glm::vec3 v[8];
        for (int i = 0; i < 8; ++i)
            v[i] = c + r * ((i & 1) ? 1.0f : -1.0f) + u * ((i & 2) ? 1.0f : -1.0f) + f * ((i & 4) ? 1.0f : -1.0f);
        quad(m, v[0], v[1], v[3], v[2], col, kind);  // back (-f)
        quad(m, v[5], v[4], v[6], v[7], col, kind);  // front (+f)
        quad(m, v[4], v[0], v[2], v[6], col, kind);  // left
        quad(m, v[1], v[5], v[7], v[3], col, kind);  // right
        quad(m, v[2], v[3], v[7], v[6], col, kind);  // top
        quad(m, v[4], v[5], v[1], v[0], col, kind);  // bottom
    }
};

// Ear clipping for a simple polygon given counter-clockwise (seen from above, x east, z south ->
// CCW means the signed area of (x, -z) is positive).
std::vector<int> triangulate(const std::vector<glm::vec2>& poly) {
    std::vector<int> idx(poly.size());
    for (size_t i = 0; i < poly.size(); ++i) idx[i] = static_cast<int>(i);
    std::vector<int> out;
    auto cross = [](glm::vec2 a, glm::vec2 b, glm::vec2 c) { return (b.x - a.x) * (-(c.y - a.y)) - (-(b.y - a.y)) * (c.x - a.x); };
    int guard = 0;
    while (idx.size() > 3 && guard++ < 10000) {
        bool clipped = false;
        for (size_t i = 0; i < idx.size(); ++i) {
            int ia = idx[(i + idx.size() - 1) % idx.size()], ib = idx[i], ic = idx[(i + 1) % idx.size()];
            glm::vec2 a = poly[ia], b = poly[ib], c = poly[ic];
            if (cross(a, b, c) <= 1e-6f) continue;  // reflex
            bool inside = false;
            for (int j : idx) {
                if (j == ia || j == ib || j == ic) continue;
                glm::vec2 p = poly[j];
                if (cross(a, b, p) >= 0 && cross(b, c, p) >= 0 && cross(c, a, p) >= 0) {
                    inside = true;
                    break;
                }
            }
            if (inside) continue;
            out.insert(out.end(), {ia, ib, ic});
            idx.erase(idx.begin() + static_cast<long>(i));
            clipped = true;
            break;
        }
        if (!clipped) break;  // degenerate polygon: give up on the rest
    }
    if (idx.size() == 3) out.insert(out.end(), {idx[0], idx[1], idx[2]});
    return out;
}

const glm::vec3 kHouseWalls[] = {{0.80f, 0.76f, 0.66f}, {0.86f, 0.85f, 0.81f}, {0.72f, 0.71f, 0.68f},
                                 {0.82f, 0.75f, 0.56f}, {0.56f, 0.36f, 0.28f}, {0.62f, 0.60f, 0.55f}};
const glm::vec3 kHouseRoofs[] = {{0.10f, 0.10f, 0.11f}, {0.17f, 0.17f, 0.18f}, {0.33f, 0.14f, 0.09f}};
const glm::vec3 kHallWalls[] = {{0.62f, 0.64f, 0.66f}, {0.34f, 0.40f, 0.48f}, {0.82f, 0.83f, 0.84f}, {0.55f, 0.56f, 0.52f}};
}  // namespace

void Scenery::build(const TrackData& t) {
    cells_.clear();
    buildings_ = 0;
    Builder b;

    // ------------------------------------------------------------------ buildings (OSM footprints)
    std::string text;
    if (!t.directory().empty() && paths::readTextFile(t.directory() + "/buildings.json", text)) {
        bool ok = false;
        Json j = Json::parse(text, &ok);
        for (const Json& rec : j["buildings"].array()) {
            std::vector<glm::vec2> pts;
            for (const Json& p : rec["pts"].array()) pts.push_back({p[0].f(), p[1].f()});
            if (pts.size() < 3) continue;
            const float h = rec["h"].f(6.0f);
            const int roofType = rec["roof"].i();
            const int kind = rec["kind"].i();
            glm::vec2 c(0.0f);
            for (auto& p : pts) c += p / static_cast<float>(pts.size());
            float gmin = t.terrainHeight(c.x, c.y), gmax = gmin;
            for (auto& p : pts) {
                float g = t.terrainHeight(p.x, p.y);
                gmin = std::min(gmin, g);
                gmax = std::max(gmax, g);
            }
            const float base = gmin - 0.4f, top = gmax + h;
            const float hsh = hashf(c.x, c.y);
            const glm::vec3 wall = kind == 1 ? kHallWalls[static_cast<int>(hsh * 4) % 4] : kHouseWalls[static_cast<int>(hsh * 6) % 6];
            const glm::vec3 roofCol = kHouseRoofs[static_cast<int>(hashf(c.y, c.x) * 3) % 3];
            const float wallKind = kind == 1 ? IndustrialWall : (kind == 2 ? Plain : HouseWall);
            MeshData& m = b.cellFor(glm::vec3(c.x, 0, c.y));
            // walls
            float along = 0.0f;
            for (size_t i = 0; i < pts.size(); ++i) {
                glm::vec2 p0 = pts[i], p1 = pts[(i + 1) % pts.size()];
                float len = glm::length(p1 - p0);
                b.quad(m, {p0.x, base, p0.y}, {p1.x, base, p1.y}, {p1.x, top, p1.y}, {p0.x, top, p0.y}, wall, wallKind,
                       {along, base - gmax}, {along + len, base - gmax}, {along + len, top - gmax}, {along, top - gmax});
                along += len;
            }
            if (roofType == 1 && pts.size() >= 4) {
                // pitched roof over the footprint's oriented rectangle: ridge along the long side
                glm::vec2 axis(1, 0);
                float bestLen = 0.0f;
                for (size_t i = 0; i < pts.size(); ++i) {
                    glm::vec2 e = pts[(i + 1) % pts.size()] - pts[i];
                    if (glm::length(e) > bestLen) bestLen = glm::length(e), axis = glm::normalize(e);
                }
                glm::vec2 perp(-axis.y, axis.x);
                float a0 = 1e9f, a1 = -1e9f, p0 = 1e9f, p1 = -1e9f;
                for (auto& p : pts) {
                    a0 = std::min(a0, glm::dot(p - c, axis)), a1 = std::max(a1, glm::dot(p - c, axis));
                    p0 = std::min(p0, glm::dot(p - c, perp)), p1 = std::max(p1, glm::dot(p - c, perp));
                }
                const float ov = 0.35f;
                a0 -= ov, a1 += ov, p0 -= ov, p1 += ov;
                const float pm = 0.5f * (p0 + p1), half = 0.5f * (p1 - p0);
                const float ridgeH = top + half * 0.75f;  // ~37 degree pitch
                auto P = [&](float a, float p, float y) {
                    glm::vec2 q = c + axis * a + perp * p;
                    return glm::vec3(q.x, y, q.y);
                };
                glm::vec3 e00 = P(a0, p0, top - 0.25f), e10 = P(a1, p0, top - 0.25f);
                glm::vec3 e01 = P(a0, p1, top - 0.25f), e11 = P(a1, p1, top - 0.25f);
                glm::vec3 r0 = P(a0, pm, ridgeH), r1 = P(a1, pm, ridgeH);
                const glm::vec3 ax3(axis.x, 0, axis.y), pp3(perp.x, 0, perp.y), up(0, 1, 0);
                b.quadFacing(m, up - pp3, e00, e10, r1, r0, roofCol, TileRoof, {a0, 0}, {a1, 0}, {a1, half}, {a0, half});
                b.quadFacing(m, up + pp3, e11, e01, r0, r1, roofCol, TileRoof, {a1, 0}, {a0, 0}, {a0, half}, {a1, half});
                // gable ends (wall coloured), slightly inset
                b.tri(m, P(a0 + ov, p1 - ov, top), P(a0 + ov, p0 + ov, top), P(a0 + ov, pm, ridgeH - 0.2f), wall, Plain, -ax3);
                b.tri(m, P(a1 - ov, p0 + ov, top), P(a1 - ov, p1 - ov, top), P(a1 - ov, pm, ridgeH - 0.2f), wall, Plain, ax3);
            } else {
                std::vector<int> tris = triangulate(pts);
                const glm::vec3 flat = kind == 1 ? glm::vec3(0.24f, 0.24f, 0.25f) : roofCol;
                for (size_t k = 0; k + 2 < tris.size(); k += 3) {
                    glm::vec2 a = pts[tris[k]], bb = pts[tris[k + 1]], cc = pts[tris[k + 2]];
                    b.tri(m, {a.x, top, a.y}, {bb.x, top, bb.y}, {cc.x, top, cc.y}, flat, FlatRoof, glm::vec3(0, 1, 0));
                }
            }
            ++buildings_;
        }
    }

    // ------------------------------------------------------------------ marshal posts
    // On the TV cameras' 260 m grid (CameraRig::tvCameraSpot) but half a spacing out of phase, so a
    // post never stands in front of a broadcast camera.
    const float L = t.length();
    int side = 1;
    for (float s = 260.0f * 0.15f + 130.0f; s < L - 100.0f; s += 260.0f) {
        side = -side;
        if (t.onBridge(s, 30.0f)) continue;
        const float lat = side * (t.barrierAt(s, side) + 2.0f);
        glm::vec3 p = t.surfacePoint(s, lat);
        p.y = t.terrainHeight(p.x, p.z);
        glm::vec3 fwd = t.tangentAt(s);
        MeshData& m = b.cellFor(p);
        b.box(m, p + glm::vec3(0, 1.0f, 0), fwd, {0.70f, 1.4f, 0.90f}, {0.86f, 0.86f, 0.84f}, Plain);        // hut
        b.box(m, p + glm::vec3(0, 1.75f, 0), fwd, {0.72f, 0.18f, 0.92f}, {0.95f, 0.38f, 0.04f}, Plain);      // orange band
        b.box(m, p + glm::vec3(0, 2.45f, 0), fwd, {0.85f, 0.06f, 1.05f}, {0.15f, 0.15f, 0.16f}, FlatRoof);   // roof
        glm::vec3 pole = p + glm::normalize(glm::vec3(fwd.x, 0, fwd.z)) * 1.3f;
        b.box(m, pole + glm::vec3(0, 1.6f, 0), fwd, {0.03f, 1.6f, 0.03f}, {0.7f, 0.7f, 0.72f}, Plain);
        b.box(m, pole + glm::vec3(0, 2.9f, 0.0f) + glm::normalize(t.rightAt(s)) * (-side * 0.3f), fwd,
              {0.28f, 0.2f, 0.01f}, {0.95f, 0.85f, 0.05f}, Plain);                                          // yellow flag
    }

    // ------------------------------------------------------------------ bridge parapets
    for (const glm::vec2& br : t.bridges()) {
        for (int sd : {-1, 1}) {
            for (float s = br.x - 4.0f; s < br.y + 4.0f; s += 1.0f) {
                glm::vec3 r = t.rightAt(s), r2 = t.rightAt(s + 1.0f);
                float in0 = t.halfWidthAt(s, sd) + 0.6f, in1 = t.halfWidthAt(s + 1.0f, sd) + 0.6f;
                glm::vec3 a = t.surfacePoint(s, sd * in0), c2 = t.surfacePoint(s + 1.0f, sd * in1);
                a.y = t.roadHeight(s, sd * (in0 - 0.6f));
                c2.y = t.roadHeight(s + 1.0f, sd * (in1 - 0.6f));
                glm::vec3 ao = a + r * (sd * 0.35f), co = c2 + r2 * (sd * 0.35f);
                const glm::vec3 up(0, 0.95f, 0), down(0, -1.3f, 0);
                const glm::vec3 conc(0.58f, 0.57f, 0.54f);
                MeshData& m = b.cellFor(a);
                const glm::vec3 out = r * static_cast<float>(sd);
                b.quadFacing(m, -out, a, c2, c2 + up, a + up, conc, Concrete);                // inner face
                b.quadFacing(m, glm::vec3(0, 1, 0), a + up, c2 + up, co + up, ao + up, conc, Concrete);  // coping
                b.quadFacing(m, out, ao + down, co + down, co + up, ao + up, conc, Concrete);  // outer face + deck edge
            }
        }
    }

    // ------------------------------------------------------------------ upload per cell
    size_t tris = 0;
    for (auto& [key, m] : b.cells) {
        if (m.vertices.empty()) continue;
        Cell cell;
        glm::vec3 mn = m.boundsMin(), mx = m.boundsMax();
        cell.origin = glm::dvec3((mn + mx) * 0.5f);
        for (auto& v : m.vertices) v.pos -= glm::vec3(cell.origin);
        cell.centre = glm::vec3(cell.origin);
        cell.radius = glm::length(mx - mn) * 0.5f;
        tris += m.indices.size() / 3;
        cell.mesh.upload(m);
        cells_.push_back(std::move(cell));
    }
    LOGI("Scenery: %d buildings, %zu cells, %zu triangles", buildings_, cells_.size(), tris);
}

void Scenery::visible(const glm::dvec3& camPos, const Frustum& f, float maxDistance, std::vector<int>& out) const {
    out.clear();
    for (int i = 0; i < static_cast<int>(cells_.size()); ++i) {
        const Cell& c = cells_[i];
        glm::vec3 rel = glm::vec3(glm::dvec3(c.centre) - camPos);
        if (glm::length(rel) - c.radius > maxDistance) continue;
        if (!f.sphere(rel, c.radius)) continue;
        out.push_back(i);
    }
}

}  // namespace apex
