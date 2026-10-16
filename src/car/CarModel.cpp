#include "car/CarModel.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>

namespace apex {

namespace {
constexpr float kPi = 3.14159265358979f;

// Monotone cubic interpolation through (z, value) keys.
struct Profile {
    std::vector<std::pair<float, float>> k;
    Profile(std::initializer_list<std::pair<float, float>> keys) : k(keys) {}
    float operator()(float z) const {
        if (z <= k.front().first) return k.front().second;
        if (z >= k.back().first) return k.back().second;
        size_t i = 0;
        while (i + 1 < k.size() && k[i + 1].first < z) ++i;
        auto slope = [&](size_t j) {
            if (j == 0 || j + 1 >= k.size()) return 0.0f;
            float d0 = (k[j].second - k[j - 1].second) / (k[j].first - k[j - 1].first);
            float d1 = (k[j + 1].second - k[j].second) / (k[j + 1].first - k[j].first);
            if (d0 * d1 <= 0.0f) return 0.0f;
            return 2.0f / (1.0f / d0 + 1.0f / d1);
        };
        float z0 = k[i].first, z1 = k[i + 1].first, h = z1 - z0;
        float t = (z - z0) / h;
        float t2 = t * t, t3 = t2 * t;
        float m0 = slope(i) * h, m1 = slope(i + 1) * h;
        return (2 * t3 - 3 * t2 + 1) * k[i].second + (t3 - 2 * t2 + t) * m0 + (-2 * t3 + 3 * t2) * k[i + 1].second +
               (t3 - t2) * m1;
    }
};

Vertex vtx(const glm::vec3& p, const glm::vec3& n, float mat, glm::vec2 uv = glm::vec2(0.0f), float extra = 0.0f) {
    Vertex v;
    v.pos = p;
    v.normal = n;
    v.uv = uv;
    v.color = glm::vec4(0.0f, 0.0f, extra, mat);
    return v;
}

// Oriented box: centre + rotation matrix columns scaled by half extents.
void box(MeshData& m, const glm::vec3& c, const glm::mat3& axes, const glm::vec3& half, float mat, float extra = 0.0f) {
    for (int f = 0; f < 6; ++f) {
        int a = f / 2;
        float sgn = (f % 2 == 0) ? 1.0f : -1.0f;
        glm::vec3 n = axes[a] * sgn;
        glm::vec3 u = axes[(a + 1) % 3] * half[(a + 1) % 3];
        glm::vec3 v = axes[(a + 2) % 3] * half[(a + 2) % 3];
        glm::vec3 o = c + n * half[a];
        uint32_t b = static_cast<uint32_t>(m.vertices.size());
        glm::vec3 p[4] = {o - u - v, o + u - v, o + u + v, o - u + v};
        if (sgn < 0) std::swap(p[1], p[3]);
        for (int i = 0; i < 4; ++i) m.vertices.push_back(vtx(p[i], n, mat, glm::vec2(i == 1 || i == 2, i >= 2), extra));
        m.addQuad(b, b + 1, b + 2, b + 3);
    }
}

void box(MeshData& m, const glm::vec3& c, const glm::vec3& half, float mat, float extra = 0.0f) {
    box(m, c, glm::mat3(1.0f), half, mat, extra);
}

void tube(MeshData& m, const glm::vec3& a, const glm::vec3& b, float r, int sides, float mat) {
    glm::vec3 ax = glm::normalize(b - a);
    glm::vec3 t = std::abs(ax.y) < 0.9f ? glm::normalize(glm::cross(ax, glm::vec3(0, 1, 0))) : glm::vec3(1, 0, 0);
    glm::vec3 bt = glm::cross(ax, t);
    uint32_t base = static_cast<uint32_t>(m.vertices.size());
    for (int i = 0; i <= sides; ++i) {
        float ang = 2 * kPi * i / sides;
        glm::vec3 n = t * std::cos(ang) + bt * std::sin(ang);
        m.vertices.push_back(vtx(a + n * r, n, mat));
        m.vertices.push_back(vtx(b + n * r, n, mat));
    }
    for (int i = 0; i < sides; ++i) m.addQuad(base + i * 2, base + i * 2 + 2, base + i * 2 + 3, base + i * 2 + 1);
}

void ellipsoid(MeshData& m, const glm::vec3& c, const glm::vec3& r, int seg, float mat) {
    uint32_t base = static_cast<uint32_t>(m.vertices.size());
    for (int j = 0; j <= seg; ++j) {
        float v = kPi * j / seg;
        for (int i = 0; i <= seg * 2; ++i) {
            float u = 2 * kPi * i / (seg * 2);
            glm::vec3 d(std::sin(v) * std::cos(u), std::cos(v), std::sin(v) * std::sin(u));
            m.vertices.push_back(vtx(c + d * r, glm::normalize(d / r), mat));
        }
    }
    int row = seg * 2 + 1;
    for (int j = 0; j < seg; ++j)
        for (int i = 0; i < seg * 2; ++i) {
            uint32_t a = base + j * row + i;
            m.addQuad(a, a + 1, a + row + 1, a + row);
        }
}

// Revolve a profile (x along the axle, r radius) around the x axis.
void revolve(MeshData& m, const std::vector<glm::vec2>& prof, int seg, float mat, bool smooth = true) {
    uint32_t base = static_cast<uint32_t>(m.vertices.size());
    const int np = static_cast<int>(prof.size());
    for (int i = 0; i <= seg; ++i) {
        float a = 2 * kPi * i / seg;
        float c = std::cos(a), s = std::sin(a);
        for (int k = 0; k < np; ++k) {
            glm::vec2 p = prof[k];
            // profile tangent for the normal
            glm::vec2 d = prof[std::min(k + 1, np - 1)] - prof[std::max(k - 1, 0)];
            glm::vec2 n2 = glm::normalize(glm::vec2(d.y, -d.x));  // (dx, dr) rotated
            glm::vec3 n(n2.x, n2.y * c, n2.y * s);
            m.vertices.push_back(vtx(glm::vec3(p.x, p.y * c, p.y * s), smooth ? n : glm::vec3(0), mat,
                                     glm::vec2(static_cast<float>(i) / seg, static_cast<float>(k) / (np - 1))));
        }
    }
    for (int i = 0; i < seg; ++i)
        for (int k = 0; k + 1 < np; ++k) {
            uint32_t a = base + i * np + k;
            m.addQuad(a, a + 1, a + np + 1, a + np);
        }
}
}  // namespace

void CarModel::build(const Geometry& g) {
    geo = g;
    const float zNose = -2.48f, zTail = 2.23f;

    // ------------------------------------------------------------------ body shell
    const Profile W({{-2.48f, 0.66f}, {-2.36f, 0.86f}, {-2.10f, 0.97f}, {-1.75f, 1.02f}, {-1.45f, 1.035f},
                     {-1.10f, 1.01f}, {-0.70f, 0.965f}, {0.00f, 0.95f}, {0.60f, 0.97f}, {1.00f, 1.025f},
                     {1.30f, 1.045f}, {1.70f, 1.035f}, {2.00f, 0.99f}, {2.23f, 0.93f}});
    const Profile Yb({{-2.48f, -0.32f}, {-2.25f, -0.352f}, {1.55f, -0.355f}, {2.23f, -0.27f}});
    const Profile Yc({{-2.48f, -0.19f}, {-2.20f, -0.12f}, {-1.45f, -0.03f}, {-0.70f, -0.05f}, {0.30f, -0.04f},
                      {1.30f, -0.01f}, {2.23f, 0.03f}});
    const Profile Yt({{-2.48f, -0.11f}, {-2.36f, -0.02f}, {-2.00f, 0.07f}, {-1.45f, 0.13f}, {-1.00f, 0.165f},
                      {-0.78f, 0.205f}, {-0.45f, 0.47f}, {-0.14f, 0.735f}, {0.05f, 0.79f}, {0.45f, 0.79f},
                      {0.80f, 0.67f}, {1.15f, 0.47f}, {1.45f, 0.345f}, {1.80f, 0.305f}, {2.05f, 0.30f},
                      {2.23f, 0.275f}});
    const Profile Wt({{-2.48f, 0.74f}, {-1.45f, 0.80f}, {-0.80f, 0.80f}, {-0.45f, 0.71f}, {-0.10f, 0.67f},
                      {0.50f, 0.67f}, {0.90f, 0.71f}, {1.30f, 0.82f}, {2.23f, 0.86f}});
    const Profile Nup({{-2.48f, 3.4f}, {-1.00f, 4.6f}, {-0.70f, 4.0f}, {-0.30f, 2.8f}, {0.60f, 2.8f}, {1.20f, 4.0f},
                       {2.23f, 4.8f}});

    MeshData shell;
    const int NZ = 120, NH = 46, R = 2 * NH - 2;
    std::vector<float> zs(NZ);
    for (int i = 0; i < NZ; ++i) zs[i] = zNose + (zTail - zNose) * (0.5f - 0.5f * std::cos(kPi * i / (NZ - 1)));
    for (int si = 0; si < NZ; ++si) {
        float z = zs[si];
        float w = W(z), yb = Yb(z), yc = Yc(z), yt = Yt(z), wt = Wt(z), nup = Nup(z), ndn = 7.0f;
        float hup = yt - yc, hdn = yc - yb;
        float dn = z - zNose;
        if (dn < 0.26f) {
            float f = std::sqrt(std::max(0.0f, 1.0f - std::pow(1.0f - dn / 0.26f, 2.0f)));
            w *= 0.45f + 0.55f * f;
            hup *= 0.55f + 0.45f * f;
            hdn *= 0.55f + 0.45f * f;
        }
        float dt = zTail - z;
        if (dt < 0.07f) {
            float f = std::sqrt(std::max(0.0f, 1.0f - std::pow(1.0f - dt / 0.07f, 2.0f)));
            w *= 0.93f + 0.07f * f;
            hup *= 0.93f + 0.07f * f;
        }
        float bulgeF = 0.21f * std::exp(-std::pow((z - g.frontAxleZ) / 0.50f, 2.0f));
        float bulgeR = 0.15f * std::exp(-std::pow((z - g.rearAxleZ) / 0.52f, 2.0f));
        std::vector<glm::vec3> half(NH);
        for (int k = 0; k < NH; ++k) {
            float th = -kPi * 0.5f + kPi * k / (NH - 1);
            float c = std::cos(th), s = std::sin(th);
            float x, y;
            if (s >= 0.0f) {
                x = w * std::pow(std::max(c, 0.0f), 2.0f / nup);
                float t = std::pow(s, 2.0f / nup);
                y = yc + hup * t;
                x *= glm::mix(1.0f, wt, std::pow(t, 1.3f));
                float xb = w - 0.21f;
                float gb = std::exp(-std::pow((x - xb) / 0.24f, 2.0f)) * glm::smoothstep(0.0f, 0.25f, t);
                y += (bulgeF + bulgeR) * gb;
            } else {
                x = w * std::pow(std::max(c, 0.0f), 2.0f / ndn);
                y = yc - hdn * std::pow(-s, 2.0f / ndn);
            }
            half[k] = glm::vec3(x, y, z);
        }
        for (int j = 0; j < R; ++j) {
            glm::vec3 p = j < NH ? half[j] : half[R - j];
            if (j >= NH) p.x = -p.x;
            shell.vertices.push_back(vtx(p, glm::vec3(0, 1, 0), carmat::Body, glm::vec2(z, static_cast<float>(j) / R)));
        }
    }
    for (int si = 0; si + 1 < NZ; ++si)
        for (int j = 0; j < R; ++j) {
            uint32_t a = si * R + j, b = si * R + (j + 1) % R, c = (si + 1) * R + (j + 1) % R, d = (si + 1) * R + j;
            shell.addTri(a, b, c);
            shell.addTri(a, c, d);
        }
    // nose cap (shared vertices -> rounded)
    {
        glm::vec3 centre(0.0f, Yc(zNose), zNose - 0.004f);
        uint32_t ci = shell.addVertex(vtx(centre, glm::vec3(0, 0, -1), carmat::Body));
        for (int j = 0; j < R; ++j) shell.addTri(ci, (j + 1) % R, j);
    }
    // tail cap (own vertices -> crisp Kamm tail)
    {
        uint32_t base = static_cast<uint32_t>(shell.vertices.size());
        glm::vec3 centre(0.0f, Yc(zTail) + 0.05f, zTail);
        for (int j = 0; j < R; ++j) {
            Vertex v = shell.vertices[(NZ - 1) * R + j];
            shell.vertices.push_back(v);
        }
        uint32_t ci = shell.addVertex(vtx(centre, glm::vec3(0, 0, 1), carmat::Body));
        for (int j = 0; j < R; ++j) shell.addTri(ci, base + j, base + (j + 1) % R);
    }
    shell.computeNormals();
    // flat normals on the tail cap
    for (size_t i = static_cast<size_t>(NZ) * R + 1; i < shell.vertices.size(); ++i) shell.vertices[i].normal = glm::vec3(0, 0, 1);
    shell.computeTangents();
    body.upload(shell);

    arches[0] = glm::vec4(g.frontAxleZ, g.groundY + g.frontRadius, g.frontRadius + 0.045f, 0.57f);
    arches[1] = glm::vec4(g.rearAxleZ, g.groundY + g.rearRadius, g.rearRadius + 0.045f, 0.57f);

    // ------------------------------------------------------------------ aero & detail parts
    MeshData p;
    // front splitter + dive planes
    box(p, {0.0f, -0.366f, -2.30f}, {0.95f, 0.008f, 0.22f}, carmat::Carbon);
    for (int s = -1; s <= 1; s += 2) {
        glm::mat3 r = glm::mat3(glm::rotate(glm::mat4(1.0f), s * 0.18f, glm::vec3(0, 0, 1)));
        box(p, {s * 0.93f, -0.16f, -2.26f}, r, {0.09f, 0.006f, 0.10f}, carmat::Carbon);
        box(p, {s * 0.95f, -0.25f, -2.30f}, r, {0.07f, 0.006f, 0.09f}, carmat::Carbon);
    }
    // rear wing: cambered (inverted) aerofoil with endplates and swan-neck mounts
    {
        const float span = 0.90f, chord = 0.36f;
        const glm::vec3 le(0.0f, 0.70f, 1.80f);
        const float aoa = 0.16f;
        std::vector<glm::vec2> foil;  // (z, y) around the section, chord-normalised
        const int n = 16;
        for (int i = 0; i <= n; ++i) {  // lower (suction) surface, front to back
            float x = 0.5f - 0.5f * std::cos(kPi * i / n);
            float camber = -0.05f * 4 * x * (1 - x);
            float th = 0.11f * (0.2969f * std::sqrt(x) - 0.126f * x - 0.3516f * x * x + 0.2843f * x * x * x - 0.1036f * x * x * x * x);
            foil.push_back({x, camber - th});
        }
        for (int i = n - 1; i > 0; --i) {  // upper surface back to front
            float x = 0.5f - 0.5f * std::cos(kPi * i / n);
            float camber = -0.05f * 4 * x * (1 - x);
            float th = 0.11f * (0.2969f * std::sqrt(x) - 0.126f * x - 0.3516f * x * x + 0.2843f * x * x * x - 0.1036f * x * x * x * x);
            foil.push_back({x, camber + th});
        }
        const int fn = static_cast<int>(foil.size());
        uint32_t base = static_cast<uint32_t>(p.vertices.size());
        for (int side = 0; side < 2; ++side) {
            float x = side == 0 ? -span : span;
            for (int i = 0; i < fn; ++i) {
                glm::vec2 f = foil[i] * chord;
                float zz = le.z + f.x * std::cos(aoa) - f.y * std::sin(aoa);
                float yy = le.y + f.x * std::sin(aoa) + f.y * std::cos(aoa);
                p.vertices.push_back(vtx({x, yy, zz}, glm::vec3(0, 1, 0), carmat::Carbon));
            }
        }
        for (int i = 0; i < fn; ++i) {
            uint32_t a = base + i, b = base + (i + 1) % fn, c = base + fn + (i + 1) % fn, d = base + fn + i;
            p.addTri(a, c, b);
            p.addTri(a, d, c);
        }
        for (int s = -1; s <= 1; s += 2) {
            box(p, {s * (span + 0.012f), 0.66f, 1.97f}, {0.006f, 0.17f, 0.27f}, carmat::Carbon);
            // swan neck: from the deck up behind the wing and over onto its top surface
            glm::vec3 a(s * 0.36f, 0.29f, 2.08f), b(s * 0.36f, 0.62f, 2.12f), c(s * 0.36f, 0.80f, 2.02f), d(s * 0.36f, 0.77f, 1.93f);
            for (auto seg : {std::make_pair(a, b), std::make_pair(b, c), std::make_pair(c, d)}) {
                glm::vec3 m = (seg.first + seg.second) * 0.5f;
                glm::vec3 dir = glm::normalize(seg.second - seg.first);
                glm::vec3 side(1, 0, 0), nrm = glm::normalize(glm::cross(dir, side));
                box(p, m, glm::mat3(side, dir, nrm), {0.008f, glm::length(seg.second - seg.first) * 0.5f + 0.01f, 0.03f}, carmat::Carbon);
            }
        }
    }
    // diffuser with strakes
    {
        glm::vec3 a(-0.82f, -0.352f, 1.66f), b(0.82f, -0.352f, 1.66f), c(0.82f, -0.20f, 2.24f), d(-0.82f, -0.20f, 2.24f);
        glm::vec3 n = glm::normalize(glm::cross(b - a, d - a));
        uint32_t base = static_cast<uint32_t>(p.vertices.size());
        for (auto q : {a, b, c, d}) p.vertices.push_back(vtx(q, -n, carmat::Carbon));
        p.addQuad(base, base + 3, base + 2, base + 1);
        for (float x : {-0.62f, -0.24f, 0.24f, 0.62f}) {
            uint32_t s0 = static_cast<uint32_t>(p.vertices.size());
            p.vertices.push_back(vtx({x, -0.352f, 1.66f}, {1, 0, 0}, carmat::Carbon));
            p.vertices.push_back(vtx({x, -0.20f, 2.24f}, {1, 0, 0}, carmat::Carbon));
            p.vertices.push_back(vtx({x, -0.36f, 2.24f}, {1, 0, 0}, carmat::Carbon));
            p.addTri(s0, s0 + 1, s0 + 2);
        }
    }
    // mirrors
    for (int s = -1; s <= 1; s += 2) {
        ellipsoid(p, {s * 1.04f, 0.36f, -0.47f}, {0.055f, 0.048f, 0.11f}, 8, carmat::Carbon);
        box(p, {s * 0.97f, 0.33f, -0.47f}, {0.05f, 0.012f, 0.03f}, carmat::Carbon);
    }
    // side exhaust exits ahead of the rear wheels
    for (int s = -1; s <= 1; s += 2)
        for (float z : {0.62f, 0.76f}) tube(p, {s * 0.94f, -0.29f, z}, {s * 1.035f, -0.29f, z}, 0.042f, 12, carmat::Chrome);
    // wheel arch liners and inner walls
    for (int axle = 0; axle < 2; ++axle) {
        float zw = arches[axle].x, yw = arches[axle].y, r = arches[axle].z - 0.004f;
        float outer = W(zw) - 0.006f, inner = arches[axle].w;
        for (int s = -1; s <= 1; s += 2) {
            uint32_t base = static_cast<uint32_t>(p.vertices.size());
            const int seg = 20;
            for (int i = 0; i <= seg; ++i) {
                float a = -0.35f + (kPi + 0.7f) * i / seg;
                glm::vec3 radial(0.0f, std::sin(a), std::cos(a));
                for (float x : {inner, outer}) p.vertices.push_back(vtx({s * x, yw + radial.y * r, zw + radial.z * r}, -radial, carmat::ArchLiner));
            }
            for (int i = 0; i < seg; ++i) p.addQuad(base + i * 2, base + i * 2 + 1, base + i * 2 + 3, base + i * 2 + 2);
            uint32_t c = p.addVertex(vtx({s * inner, yw, zw}, {-static_cast<float>(s), 0, 0}, carmat::ArchLiner));
            for (int i = 0; i < seg; ++i) p.addTri(c, base + i * 2, base + i * 2 + 2);
        }
    }

    // ------------------------------------------------------------------ interior
    box(p, {0.0f, -0.30f, 0.15f}, {0.82f, 0.01f, 1.05f}, carmat::Interior);
    box(p, {0.0f, 0.215f, -0.63f}, glm::mat3(glm::rotate(glm::mat4(1.0f), 0.18f, glm::vec3(1, 0, 0))), {0.80f, 0.065f, 0.13f}, carmat::Interior);
    box(p, {-0.37f, 0.275f, -0.515f}, glm::mat3(glm::rotate(glm::mat4(1.0f), -0.6f, glm::vec3(1, 0, 0))), {0.11f, 0.008f, 0.06f}, carmat::Display);
    box(p, {0.0f, -0.17f, -0.12f}, {0.12f, 0.12f, 0.6f}, carmat::Interior);
    tube(p, {-0.37f, 0.25f, -0.56f}, g.steeringWheelCentre + glm::vec3(0, -0.02f, 0.03f), 0.024f, 8, carmat::Interior);
    // racing seat
    box(p, {-0.37f, -0.21f, 0.30f}, {0.24f, 0.06f, 0.26f}, carmat::Seat);
    box(p, {-0.37f, 0.16f, 0.56f}, glm::mat3(glm::rotate(glm::mat4(1.0f), -0.22f, glm::vec3(1, 0, 0))), {0.24f, 0.36f, 0.05f}, carmat::Seat);
    for (int s = -1; s <= 1; s += 2) box(p, {-0.37f + s * 0.22f, 0.30f, 0.50f}, {0.03f, 0.14f, 0.10f}, carmat::Seat);
    // roll cage
    {
        const float r = 0.022f;
        glm::vec3 hoop[] = {{-0.84f, -0.30f, 0.66f}, {-0.52f, 0.63f, 0.66f}, {0.52f, 0.63f, 0.66f}, {0.84f, -0.30f, 0.66f}};
        for (int i = 0; i < 3; ++i) tube(p, hoop[i], hoop[i + 1], r, 8, carmat::Cage);
        tube(p, hoop[0], hoop[2], r, 8, carmat::Cage);
        for (int s = -1; s <= 1; s += 2) {
            glm::vec3 a(s * 0.80f, -0.26f, -0.74f), b(s * 0.50f, 0.66f, -0.12f), c(s * 0.50f, 0.64f, 0.64f);
            tube(p, a, b, r, 8, carmat::Cage);
            tube(p, b, c, r, 8, carmat::Cage);
            tube(p, {s * 0.86f, -0.15f, -0.70f}, {s * 0.86f, 0.20f, 0.60f}, r, 8, carmat::Cage);
            tube(p, {s * 0.86f, 0.22f, -0.65f}, {s * 0.86f, -0.20f, 0.60f}, r, 8, carmat::Cage);
        }
        tube(p, {-0.50f, 0.665f, -0.08f}, {0.50f, 0.665f, -0.08f}, r, 8, carmat::Cage);
    }
    p.computeTangents();
    parts.upload(p);

    // ------------------------------------------------------------------ driver (hidden in cockpit view)
    {
        MeshData d;
        ellipsoid(d, {-0.37f, 0.59f, 0.22f}, {0.135f, 0.15f, 0.15f}, 12, carmat::Helmet);
        ellipsoid(d, {-0.37f, 0.585f, 0.135f}, {0.11f, 0.06f, 0.07f}, 10, carmat::Visor);
        box(d, {-0.37f, 0.22f, 0.36f}, glm::mat3(glm::rotate(glm::mat4(1.0f), -0.2f, glm::vec3(1, 0, 0))), {0.21f, 0.24f, 0.11f}, carmat::Suit);
        for (int s = -1; s <= 1; s += 2) {
            glm::vec3 shoulder(-0.37f + s * 0.20f, 0.40f, 0.30f);
            glm::vec3 hand = g.steeringWheelCentre + glm::vec3(s * 0.15f, 0.0f, 0.02f);
            glm::vec3 elbow = glm::mix(shoulder, hand, 0.5f) + glm::vec3(s * 0.07f, -0.12f, 0.0f);
            tube(d, shoulder, elbow, 0.045f, 8, carmat::Suit);
            tube(d, elbow, hand, 0.04f, 8, carmat::Suit);
        }
        d.computeTangents();
        driver.upload(d);
    }

    // ------------------------------------------------------------------ steering wheel (local)
    {
        MeshData s;
        const int seg = 28;
        uint32_t base = 0;
        for (int i = 0; i <= seg; ++i) {
            float a = 2 * kPi * i / seg;
            glm::vec3 c(0.155f * std::cos(a), 0.115f * std::sin(a), 0.0f);
            glm::vec3 radial = glm::normalize(glm::vec3(c.x / 0.155f, c.y / 0.115f, 0.0f));
            for (int k = 0; k <= 8; ++k) {
                float b = 2 * kPi * k / 8;
                glm::vec3 n = radial * std::cos(b) + glm::vec3(0, 0, 1) * std::sin(b);
                s.vertices.push_back(vtx(c + n * 0.019f, n, carmat::Interior));
            }
        }
        for (int i = 0; i < seg; ++i)
            for (int k = 0; k < 8; ++k) {
                uint32_t a = base + i * 9 + k;
                s.addQuad(a, a + 9, a + 10, a + 1);
            }
        box(s, {0.0f, -0.01f, 0.012f}, {0.11f, 0.075f, 0.015f}, carmat::Carbon);
        box(s, {0.0f, -0.02f, 0.03f}, {0.045f, 0.03f, 0.003f}, carmat::Display);
        for (int i = 0; i < 10; ++i)
            box(s, {-0.072f + i * 0.016f, 0.048f, 0.03f}, {0.006f, 0.005f, 0.004f}, carmat::ShiftLeds, i / 9.0f);
        for (int sd = -1; sd <= 1; sd += 2) box(s, {sd * 0.125f, 0.0f, 0.006f}, {0.03f, 0.014f, 0.012f}, carmat::Interior);
        s.computeTangents();
        steeringWheel.upload(s);
    }

    // ------------------------------------------------------------------ wheels
    for (int axle = 0; axle < 2; ++axle) {
        MeshData w;
        const float R = axle == 0 ? g.frontRadius : g.rearRadius;
        const float hw = axle == 0 ? 0.150f : 0.158f;
        const float rimR = 0.232f;
        // tyre: tread, rounded shoulders, bulging sidewalls down to the bead
        std::vector<glm::vec2> tyre;
        tyre.push_back({-hw + 0.012f, rimR + 0.004f});
        tyre.push_back({-hw - 0.004f, rimR + 0.03f});
        tyre.push_back({-hw - 0.006f, (rimR + R) * 0.5f});
        tyre.push_back({-hw + 0.002f, R - 0.035f});
        tyre.push_back({-hw + 0.016f, R - 0.010f});
        tyre.push_back({-hw + 0.040f, R});
        tyre.push_back({hw - 0.040f, R});
        tyre.push_back({hw - 0.016f, R - 0.010f});
        tyre.push_back({hw - 0.002f, R - 0.035f});
        tyre.push_back({hw + 0.006f, (rimR + R) * 0.5f});
        tyre.push_back({hw + 0.004f, rimR + 0.03f});
        tyre.push_back({hw - 0.012f, rimR + 0.004f});
        revolve(w, tyre, 56, carmat::Rubber);
        // rim barrel (seen from inside) and outer lip
        revolve(w, {{hw - 0.012f, rimR}, {-hw + 0.012f, rimR}}, 48, carmat::Rim);
        revolve(w, {{hw - 0.004f, rimR - 0.016f}, {hw + 0.002f, rimR - 0.006f}, {hw - 0.004f, rimR + 0.004f}}, 48, carmat::Rim);
        // brake disc
        revolve(w, {{-0.035f, 0.115f}, {-0.035f, 0.19f}, {0.0f, 0.19f}, {0.0f, 0.115f}}, 40, carmat::BrakeDisc);
        // 10 spokes (5 doubles), dished towards the hub, centre-lock nut
        const float faceX = hw - 0.02f, hubX = hw - 0.06f;
        for (int i = 0; i < 10; ++i) {
            float a = 2 * kPi * (i / 2) / 5 + (i % 2 ? 0.16f : -0.16f);
            glm::vec3 radial(0.0f, std::cos(a), std::sin(a));
            glm::vec3 inner = glm::vec3(hubX, 0, 0) + radial * 0.06f;
            glm::vec3 outer = glm::vec3(faceX, 0, 0) + radial * (rimR - 0.012f);
            glm::vec3 mid = (inner + outer) * 0.5f;
            glm::vec3 dir = glm::normalize(outer - inner);
            glm::vec3 tang = glm::normalize(glm::cross(dir, glm::vec3(1, 0, 0)));
            glm::vec3 nrm = glm::normalize(glm::cross(tang, dir));
            box(w, mid, glm::mat3(dir, tang, nrm), {glm::length(outer - inner) * 0.5f, 0.012f, 0.014f}, carmat::Rim);
        }
        tube(w, {hubX - 0.02f, 0, 0}, {hubX + 0.025f, 0, 0}, 0.065f, 16, carmat::Rim);
        tube(w, {hubX + 0.02f, 0, 0}, {hubX + 0.06f, 0, 0}, 0.04f, 6, carmat::Caliper);
        w.computeTangents();
        wheel[axle].upload(w);

        MeshData c;
        const float caliperAngle = axle == 0 ? 2.2f : 0.9f;  // behind the axle, upper half
        for (int k = 0; k < 5; ++k) {
            float a = caliperAngle - 0.35f + 0.7f * k / 4;
            glm::vec3 radial(0.0f, std::cos(a), std::sin(a));
            glm::vec3 tang(0.0f, -std::sin(a), std::cos(a));
            box(c, glm::vec3(-0.0175f, 0, 0) + radial * 0.17f, glm::mat3(glm::vec3(1, 0, 0), radial, tang), {0.05f, 0.04f, 0.045f}, carmat::Caliper);
        }
        c.computeTangents();
        caliper[axle].upload(c);
    }
}

}  // namespace apex
