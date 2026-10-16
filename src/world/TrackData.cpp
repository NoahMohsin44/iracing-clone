#include "world/TrackData.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

#include "core/Json.h"
#include "core/Log.h"
#include "core/Noise.h"
#include "core/Paths.h"
#include "stb_image.h"

namespace apex {

namespace {
const SurfaceInfo kSurfaces[] = {
    // name       grip   rollRes bumpAmp bumpWave drag
    {"asphalt",   1.00f, 0.000f, 0.000f, 1.0f, 0.00f},
    {"kerb",      0.93f, 0.002f, 0.000f, 1.0f, 0.00f},
    {"concrete",  0.96f, 0.000f, 0.000f, 1.0f, 0.00f},
    {"grass",     0.52f, 0.030f, 0.012f, 1.6f, 0.02f},
    {"gravel",    0.48f, 0.150f, 0.020f, 0.7f, 0.12f},
    {"dirt",      0.58f, 0.050f, 0.018f, 1.2f, 0.03f},
};

inline float lerp(float a, float b, float t) { return a + (b - a) * t; }

glm::vec3 catmullRom(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, float t) {
    float t2 = t * t, t3 = t2 * t;
    return 0.5f * ((2.0f * p1) + (-p0 + p2) * t + (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                   (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
}
}  // namespace

const SurfaceInfo& surfaceInfo(Surface s) { return kSurfaces[static_cast<int>(s)]; }

// ============================================================================ Heightmap
bool Heightmap::load(const std::string& path, float hmin, float hmax) {
    int cw = 0, ch = 0, comp = 0;
    stbi_us* px = stbi_load_16(path.c_str(), &cw, &ch, &comp, 1);
    if (!px) {
        LOGE("Failed to load heightmap %s", path.c_str());
        return false;
    }
    if (cw != w || ch != h) {
        LOGW("Heightmap %s is %dx%d, metadata says %dx%d", path.c_str(), cw, ch, w, h);
        w = cw;
        h = ch;
    }
    data.resize(static_cast<size_t>(w) * h);
    const float scale = (hmax - hmin) / 65535.0f;
    for (size_t i = 0; i < data.size(); ++i) data[i] = hmin + px[i] * scale;
    stbi_image_free(px);
    return true;
}

bool Heightmap::contains(float x, float z) const {
    return x >= x0 && z >= z0 && x <= x0 + width() && z <= z0 + depth();
}

float Heightmap::at(int i, int j) const {
    i = std::clamp(i, 0, w - 1);
    j = std::clamp(j, 0, h - 1);
    return data[static_cast<size_t>(j) * w + i];
}

float Heightmap::sample(float x, float z) const {
    float gx = (x - x0) / res, gz = (z - z0) / res;
    gx = std::clamp(gx, 0.0f, static_cast<float>(w - 1) - 1e-4f);
    gz = std::clamp(gz, 0.0f, static_cast<float>(h - 1) - 1e-4f);
    int i = static_cast<int>(gx), j = static_cast<int>(gz);
    float fx = gx - i, fz = gz - j;
    float h00 = at(i, j), h10 = at(i + 1, j), h01 = at(i, j + 1), h11 = at(i + 1, j + 1);
    // Triangles (i,j)-(i+1,j)-(i+1,j+1) and (i,j)-(i+1,j+1)-(i,j+1), same as the terrain mesh.
    if (fx > fz) return h00 + (h10 - h00) * fx + (h11 - h10) * fz;
    return h00 + (h11 - h01) * fx + (h01 - h00) * fz;
}

glm::vec3 Heightmap::normal(float x, float z) const {
    const float e = res * 0.5f;
    float hl = sample(x - e, z), hr = sample(x + e, z);
    float hd = sample(x, z - e), hu = sample(x, z + e);
    return glm::normalize(glm::vec3(hl - hr, 2.0f * e, hd - hu));
}

// ============================================================================ loading
bool TrackData::load(const std::string& dir) {
    std::string text;
    if (!paths::readTextFile(dir + "/track.json", text)) {
        LOGE("Missing %s/track.json", dir.c_str());
        return false;
    }
    bool ok = false;
    Json meta = Json::parse(text, &ok);
    if (!ok) {
        LOGE("Invalid track.json");
        return false;
    }
    name_ = meta["name"].str();
    spacing_ = meta["sampleSpacing"].f(2.0f);
    for (auto& s : meta["sections"].array()) sections_.push_back({s["name"].str(), s["start"].f()});
    for (auto& a : meta["attribution"].array()) attribution_.push_back(a.str());

    std::string csv;
    if (!paths::readTextFile(dir + "/centerline.csv", csv)) {
        LOGE("Missing centerline.csv");
        return false;
    }
    std::istringstream in(csv);
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        TrackSample smp;
        float s, x, y, z, hl, hr, bank;
        if (sscanf(line.c_str(), "%f,%f,%f,%f,%f,%f,%f", &s, &x, &y, &z, &hl, &hr, &bank) != 7) continue;
        smp.s = s;
        smp.pos = {x, y, z};
        smp.halfLeft = hl;
        smp.halfRight = hr;
        smp.bank = bank;
        samples_.push_back(smp);
    }
    if (samples_.size() < 16) {
        LOGE("Centreline too short");
        return false;
    }
    length_ = samples_.size() * spacing_;

    auto loadHm = [&](Heightmap& hm, const Json& j) {
        hm.w = j["w"].i();
        hm.h = j["h"].i();
        hm.x0 = j["x0"].f();
        hm.z0 = j["z0"].f();
        hm.res = j["res"].f(1.0f);
        return hm.load(dir + "/" + j["file"].str(), j["min"].f(), j["max"].f());
    };
    if (!loadHm(near_, meta["heightNear"]) || !loadHm(far_, meta["heightFar"])) return false;

    int lw = 0, lh = 0, lc = 0;
    unsigned char* lcpx = stbi_load((dir + "/" + meta["landcover"]["file"].str()).c_str(), &lw, &lh, &lc, 3);
    if (lcpx && lw == near_.w && lh == near_.h) {
        landcover_.assign(lcpx, lcpx + static_cast<size_t>(lw) * lh * 3);
    } else {
        LOGW("Landcover missing or mismatched; using defaults");
        landcover_.assign(static_cast<size_t>(near_.w) * near_.h * 3, 0);
    }
    if (lcpx) stbi_image_free(lcpx);

    buildFrames();
    buildGrid();
    buildBarriersAndVerge();
    buildKerbs();
    buildRacingLine();
    buildSpeedProfile();
    LOGI("Track '%s': %.0f m, %d samples, %zu sections, %zu kerbs", name_.c_str(), length_, sampleCount(),
         sections_.size(), kerbs_.size());
    return true;
}

bool TrackData::createSynthetic(const std::vector<glm::vec2>& loop, float halfWidth, float spacing,
                                float barrierOffset) {
    name_ = "Test track";
    spacing_ = spacing;
    samples_.clear();
    float minx = 1e9f, minz = 1e9f, maxx = -1e9f, maxz = -1e9f;
    for (size_t i = 0; i < loop.size(); ++i) {
        TrackSample smp;
        smp.s = i * spacing;
        smp.pos = {loop[i].x, 0.0f, loop[i].y};
        smp.halfLeft = smp.halfRight = halfWidth;
        samples_.push_back(smp);
        minx = std::min(minx, loop[i].x);
        maxx = std::max(maxx, loop[i].x);
        minz = std::min(minz, loop[i].y);
        maxz = std::max(maxz, loop[i].y);
    }
    length_ = samples_.size() * spacing_;
    sections_ = {{"Test", 0.0f}};
    const float margin = barrierOffset + 200.0f;
    for (Heightmap* hm : {&near_, &far_}) {
        hm->res = 8.0f;
        hm->x0 = minx - margin;
        hm->z0 = minz - margin;
        hm->w = static_cast<int>((maxx - minx + 2 * margin) / hm->res) + 2;
        hm->h = static_cast<int>((maxz - minz + 2 * margin) / hm->res) + 2;
        hm->data.assign(static_cast<size_t>(hm->w) * hm->h, -0.3f);
    }
    landcover_.assign(static_cast<size_t>(near_.w) * near_.h * 3, 0);
    buildFrames();
    buildGrid();
    buildBarriersAndVerge();
    for (auto& smp : samples_) {
        smp.barrierLeft = smp.halfLeft + barrierOffset;
        smp.barrierRight = smp.halfRight + barrierOffset;
    }
    buildKerbs();
    buildRacingLine();
    buildSpeedProfile();
    return true;
}

void TrackData::buildFrames() {
    const int n = sampleCount();
    for (int i = 0; i < n; ++i) {
        glm::vec3 d = sample(i + 1).pos - sample(i - 1).pos;
        TrackSample& smp = samples_[i];
        smp.tangent = glm::normalize(d);
        glm::vec3 f = glm::normalize(glm::vec3(d.x, 0.0f, d.z));
        smp.right = glm::vec3(-f.z, 0.0f, f.x);
    }
    std::vector<float> curv(n);
    for (int i = 0; i < n; ++i) {
        const glm::vec3& a = sample(i).right;
        const glm::vec3& b = sample(i + 1).right;
        // signed angle between consecutive right vectors about +Y; positive when turning right
        float cross = a.z * b.x - a.x * b.z;
        float dot = a.x * b.x + a.z * b.z;
        curv[i] = -std::atan2(cross, dot) / spacing_;
    }
    for (int i = 0; i < n; ++i) {
        float acc = 0.0f;
        for (int k = -4; k <= 4; ++k) acc += curv[wrapIndex(i + k)];
        samples_[i].curvature = acc / 9.0f;
    }
}

void TrackData::buildGrid() {
    float minx = 1e9f, minz = 1e9f, maxx = -1e9f, maxz = -1e9f;
    for (auto& s : samples_) {
        minx = std::min(minx, s.pos.x);
        minz = std::min(minz, s.pos.z);
        maxx = std::max(maxx, s.pos.x);
        maxz = std::max(maxz, s.pos.z);
    }
    gridX0_ = minx - gridCell_;
    gridZ0_ = minz - gridCell_;
    gridW_ = static_cast<int>((maxx - gridX0_) / gridCell_) + 2;
    gridH_ = static_cast<int>((maxz - gridZ0_) / gridCell_) + 2;
    grid_.assign(static_cast<size_t>(gridW_) * gridH_, {});
    for (int i = 0; i < sampleCount(); ++i) {
        int cx = static_cast<int>((samples_[i].pos.x - gridX0_) / gridCell_);
        int cz = static_cast<int>((samples_[i].pos.z - gridZ0_) / gridCell_);
        grid_[static_cast<size_t>(cz) * gridW_ + cx].push_back(i);
    }
}

void TrackData::buildBarriersAndVerge() {
    const int n = sampleCount();
    std::vector<float> bl(n), br(n);
    for (int i = 0; i < n; ++i) {
        const TrackSample& smp = samples_[i];
        float base = 2.5f + 1.7f * (valueNoise1(smp.s / 170.0f, 11) * 0.5f + 0.5f);
        float extra = std::clamp(std::abs(smp.curvature) * 140.0f, 0.0f, 2.8f);
        bl[i] = smp.halfLeft + base + (smp.curvature > 0 ? extra : 0.0f);
        br[i] = smp.halfRight + base + (smp.curvature < 0 ? extra : 0.0f);
    }
    for (int i = 0; i < n; ++i) {
        float al = 0, ar = 0;
        for (int k = -12; k <= 12; ++k) {
            al += bl[wrapIndex(i + k)];
            ar += br[wrapIndex(i + k)];
        }
        TrackSample& smp = samples_[i];
        smp.barrierLeft = al / 25.0f;
        smp.barrierRight = ar / 25.0f;
        glm::vec3 pl = smp.pos - smp.right * (smp.halfLeft + kVergeWidth);
        glm::vec3 pr = smp.pos + smp.right * (smp.halfRight + kVergeWidth);
        smp.vergeOuterLeftY = terrainHeight(pl.x, pl.z);
        smp.vergeOuterRightY = terrainHeight(pr.x, pr.z);
    }
}

void TrackData::buildKerbs() {
    const int n = sampleCount();
    kerbMaskLeft_.assign(n, 0);
    kerbMaskRight_.assign(n, 0);
    std::vector<float> k(n);
    for (int i = 0; i < n; ++i) {
        float acc = 0;
        for (int j = -5; j <= 5; ++j) acc += samples_[wrapIndex(i + j)].curvature;
        k[i] = acc / 11.0f;
    }
    const float kThresh = 1.0f / 190.0f;
    auto mark = [&](int side, int from, int to) {
        auto& mask = side > 0 ? kerbMaskRight_ : kerbMaskLeft_;
        for (int i = from; i <= to; ++i) mask[wrapIndex(i)] = 1;
    };
    // find a starting point outside any corner so ranges don't straddle the seam
    int start = 0;
    while (start < n && std::abs(k[start]) > kThresh) ++start;
    int i = 0;
    while (i < n) {
        int idx = wrapIndex(start + i);
        if (std::abs(k[idx]) <= kThresh) {
            ++i;
            continue;
        }
        float sign = k[idx] > 0 ? 1.0f : -1.0f;
        int a = i;
        int apex = i;
        float peak = 0;
        while (i < n && std::abs(k[wrapIndex(start + i)]) > kThresh && k[wrapIndex(start + i)] * sign > 0) {
            float v = std::abs(k[wrapIndex(start + i)]);
            if (v > peak) {
                peak = v;
                apex = i;
            }
            ++i;
        }
        int b = i;
        float lenM = (b - a) * spacing_;
        int inside = sign > 0 ? 1 : -1;
        int halfL = static_cast<int>(std::clamp(lenM * 0.30f, 7.0f, 32.0f) / spacing_);
        int apexIdx = start + apex;
        mark(inside, apexIdx - halfL, apexIdx + halfL);
        int exitLen = static_cast<int>(std::clamp(lenM * 0.6f, 14.0f, 46.0f) / spacing_);
        int exitStart = apexIdx + halfL / 2;
        mark(-inside, exitStart, exitStart + exitLen);
        if (peak > 1.0f / 70.0f) {
            int entry = start + a;
            mark(-inside, entry - static_cast<int>(10 / spacing_), entry + static_cast<int>(6 / spacing_));
        }
    }
    // spans for rendering
    kerbs_.clear();
    for (int side : {-1, 1}) {
        auto& mask = side > 0 ? kerbMaskRight_ : kerbMaskLeft_;
        int s0 = 0;
        while (s0 < n && mask[s0]) ++s0;  // begin outside a span
        if (s0 == n) continue;
        for (int c = 0; c < n; ++c) {
            int idx = wrapIndex(s0 + c);
            if (mask[idx] && !mask[wrapIndex(idx - 1)]) {
                int e = c;
                while (e < n && mask[wrapIndex(s0 + e)]) ++e;
                KerbSpan span;
                span.side = side;
                span.s0 = (s0 + c) * spacing_;
                span.s1 = (s0 + e) * spacing_;
                kerbs_.push_back(span);
                c = e;
            }
        }
    }
}

void TrackData::buildRacingLine() {
    const int n = sampleCount();
    const float margin = 1.35f;
    std::vector<float> d(n, 0.0f), lo(n), hi(n);
    for (int i = 0; i < n; ++i) {
        lo[i] = -(samples_[i].halfLeft - margin);
        hi[i] = samples_[i].halfRight - margin;
    }
    auto pos = [&](int i) {
        const TrackSample& s = sample(i);
        return glm::vec2(s.pos.x, s.pos.z) + glm::vec2(s.right.x, s.right.z) * d[wrapIndex(i)];
    };
    // Multi-scale elastic band: pulling each point toward the midpoint of its neighbours minimises
    // curvature, the clamp keeps the line on the road.
    const int strides[] = {48, 32, 20, 12, 7, 4, 2};
    for (int stride : strides) {
        for (int iter = 0; iter < 120; ++iter) {
            for (int i = 0; i < n; ++i) {
                glm::vec2 mid = 0.5f * (pos(i - stride) + pos(i + stride));
                const TrackSample& s = samples_[i];
                glm::vec2 c(s.pos.x, s.pos.z), r(s.right.x, s.right.z);
                float target = glm::dot(mid - c, r);
                d[i] = std::clamp(d[i] + 0.6f * (target - d[i]), lo[i], hi[i]);
            }
        }
    }
    for (int i = 0; i < n; ++i) samples_[i].racingLine = d[i];
    // curvature of the line from 3-point circles (6 m stride), smoothed
    std::vector<float> kc(n);
    const int st = 3;
    for (int i = 0; i < n; ++i) {
        glm::vec2 a = pos(i - st), b = pos(i), c = pos(i + st);
        glm::vec2 ab = b - a, bc = c - b, ac = c - a;
        float cross = ab.x * bc.y - ab.y * bc.x;
        float denom = glm::length(ab) * glm::length(bc) * glm::length(ac);
        kc[i] = denom > 1e-6f ? 2.0f * cross / denom : 0.0f;
    }
    for (int i = 0; i < n; ++i) {
        float acc = 0;
        for (int j = -3; j <= 3; ++j) acc += kc[wrapIndex(i + j)];
        samples_[i].lineCurvature = acc / 7.0f;
    }
}

void TrackData::buildSpeedProfile() {
    // Point-mass speed profile for the GT3 car: grip with aero and crest/compression loading,
    // braking and power limited, including the effect of gradients.
    const int n = sampleCount();
    const float g = 9.81f, mu = 1.50f, mass = 1310.0f, rho = 1.225f;
    const float clA = 3.0f, cdA = 1.25f, power = 370e3f;
    const float ka = 0.5f * rho * clA / mass, kd = 0.5f * rho * cdA / mass;
    std::vector<float> grade(n), vcurv(n), v(n);
    for (int i = 0; i < n; ++i) {
        grade[i] = (sample(i + 1).pos.y - sample(i - 1).pos.y) / (2.0f * spacing_);
    }
    for (int i = 0; i < n; ++i) {
        float acc = 0;
        for (int k = -4; k <= 4; ++k) {
            int j = wrapIndex(i + k);
            acc += (grade[wrapIndex(j + 1)] - grade[wrapIndex(j - 1)]) / (2.0f * spacing_);
        }
        vcurv[i] = acc / 9.0f;  // + = compression (sag), - = crest
    }
    // normal acceleration available per unit speed^2 (aero + vertical curvature)
    auto vertK = [&](int i) { return ka + vcurv[i]; };
    for (int i = 0; i < n; ++i) {
        float k = std::abs(samples_[i].lineCurvature);
        float denom = k - mu * vertK(i);
        v[i] = denom > 1e-6f ? std::sqrt(mu * g / denom) : 95.0f;
        // crests: keep at least ~35% of the car's weight on the tyres
        if (vcurv[i] < 0.0f) v[i] = std::min(v[i], std::sqrt(0.65f * g / -vcurv[i]));
        v[i] = std::min(v[i], 95.0f);
    }
    for (int pass = 0; pass < 2; ++pass) {
        for (int c = 2 * n; c > 0; --c) {  // braking (backwards)
            int i = wrapIndex(c), j = wrapIndex(c + 1);
            float vj = v[j];
            float normal = std::max(0.3f * g, g + vertK(j) * vj * vj);
            // friction circle: grip used for cornering is not available for braking
            float aMax = mu * normal * 0.90f;
            float aLat = vj * vj * std::abs(samples_[j].lineCurvature);
            float aLong = std::sqrt(std::max(0.0f, aMax * aMax - aLat * aLat));
            float decel = std::max(aLong, 0.15f * aMax) + kd * vj * vj + g * grade[j];
            v[i] = std::min(v[i], std::sqrt(vj * vj + 2.0f * std::max(decel, 1.0f) * spacing_));
        }
        for (int c = 0; c < 2 * n; ++c) {  // acceleration (forwards)
            int i = wrapIndex(c), j = wrapIndex(c + 1);
            float vi = std::max(v[i], 3.0f);
            float normal = std::max(0.3f * g, g + vertK(i) * vi * vi);
            float aLat = vi * vi * std::abs(samples_[i].lineCurvature);
            float aMax = mu * normal;
            float traction = std::min(mu * 0.56f * normal, std::sqrt(std::max(0.0f, aMax * aMax - aLat * aLat)));
            float acc = std::min(traction, power / (mass * vi)) - kd * vi * vi - 0.015f * g - g * grade[i];
            v[j] = std::min(v[j], std::sqrt(std::max(0.0f, vi * vi + 2.0f * acc * spacing_)));
        }
    }
    for (int i = 0; i < n; ++i) samples_[i].speedHint = v[i];
}

// ============================================================================ queries
float TrackData::wrapS(float s) const {
    s = std::fmod(s, length_);
    return s < 0 ? s + length_ : s;
}

TrackLocation TrackData::locate(const glm::vec3& p, int hint) const {
    const int n = sampleCount();
    int i = hint;
    if (i < 0 || i >= n) {
        // global: nearest sample via the grid
        int cx = static_cast<int>(std::floor((p.x - gridX0_) / gridCell_));
        int cz = static_cast<int>(std::floor((p.z - gridZ0_) / gridCell_));
        float best = 1e30f;
        for (int r = 1; r <= 64 && i < 0; r *= 2) {
            for (int z = cz - r; z <= cz + r; ++z) {
                for (int x = cx - r; x <= cx + r; ++x) {
                    if (x < 0 || z < 0 || x >= gridW_ || z >= gridH_) continue;
                    for (int idx : grid_[static_cast<size_t>(z) * gridW_ + x]) {
                        glm::vec2 d(samples_[idx].pos.x - p.x, samples_[idx].pos.z - p.z);
                        float dd = glm::dot(d, d);
                        if (dd < best) {
                            best = dd;
                            i = idx;
                        }
                    }
                }
            }
        }
        if (i < 0) {
            for (int idx = 0; idx < n; ++idx) {
                glm::vec2 d(samples_[idx].pos.x - p.x, samples_[idx].pos.z - p.z);
                float dd = glm::dot(d, d);
                if (dd < best) {
                    best = dd;
                    i = idx;
                }
            }
        }
    }
    TrackLocation loc;
    int lastDir = 0;
    float t = 0;
    for (int iter = 0; iter < 400; ++iter) {
        const glm::vec3& a = samples_[i].pos;
        const glm::vec3& b = sample(i + 1).pos;
        glm::vec2 ab(b.x - a.x, b.z - a.z), ap(p.x - a.x, p.z - a.z);
        t = glm::dot(ap, ab) / glm::dot(ab, ab);
        if (t < 0.0f && lastDir <= 0) {
            i = wrapIndex(i - 1);
            lastDir = -1;
            continue;
        }
        if (t > 1.0f && lastDir >= 0) {
            i = wrapIndex(i + 1);
            lastDir = 1;
            continue;
        }
        break;
    }
    t = std::clamp(t, 0.0f, 1.0f);
    const TrackSample& sa = samples_[i];
    const TrackSample& sb = sample(i + 1);
    glm::vec3 foot = sa.pos + (sb.pos - sa.pos) * t;
    glm::vec3 r = glm::normalize(sa.right + (sb.right - sa.right) * t);
    glm::vec2 fp(p.x - foot.x, p.z - foot.z);
    loc.seg = i;
    loc.t = t;
    loc.s = i * spacing_ + t * spacing_;
    loc.lateral = fp.x * r.x + fp.y * r.z;
    loc.distance = glm::length(fp);
    return loc;
}

float TrackData::interpScalar(float s, float TrackSample::*field) const {
    float fs = wrapS(s) / spacing_;
    int i = static_cast<int>(fs);
    float t = fs - i;
    return lerp(sample(i).*field, sample(i + 1).*field, t);
}

glm::vec3 TrackData::centreAt(float s) const {
    float fs = wrapS(s) / spacing_;
    int i = static_cast<int>(fs);
    float t = fs - i;
    return catmullRom(sample(i - 1).pos, sample(i).pos, sample(i + 1).pos, sample(i + 2).pos, t);
}

glm::vec3 TrackData::tangentAt(float s) const {
    return glm::normalize(centreAt(s + 0.5f) - centreAt(s - 0.5f));
}

glm::vec3 TrackData::rightAt(float s) const {
    float fs = wrapS(s) / spacing_;
    int i = static_cast<int>(fs);
    float t = fs - i;
    return glm::normalize(glm::mix(sample(i).right, sample(i + 1).right, t));
}

float TrackData::bankAt(float s) const { return interpScalar(s, &TrackSample::bank); }
float TrackData::halfWidthAt(float s, int side) const {
    return interpScalar(s, side > 0 ? &TrackSample::halfRight : &TrackSample::halfLeft);
}
float TrackData::barrierAt(float s, int side) const {
    return interpScalar(s, side > 0 ? &TrackSample::barrierRight : &TrackSample::barrierLeft);
}
float TrackData::racingLineAt(float s) const { return interpScalar(s, &TrackSample::racingLine); }
float TrackData::speedHintAt(float s) const { return interpScalar(s, &TrackSample::speedHint); }

float TrackData::roadHeight(float s, float lateral) const {
    return centreAt(s).y - std::tan(bankAt(s)) * lateral;
}

glm::vec3 TrackData::surfacePoint(float s, float lateral) const {
    glm::vec3 c = centreAt(s);
    glm::vec3 p = c + rightAt(s) * lateral;
    p.y = c.y - std::tan(bankAt(s)) * lateral;
    return p;
}

bool TrackData::kerbAt(float s, int side) const {
    int i = wrapIndex(static_cast<int>(std::lround(wrapS(s) / spacing_)));
    return (side > 0 ? kerbMaskRight_ : kerbMaskLeft_)[i] != 0;
}

float TrackData::kerbProfile(float s, float u) const {
    // raised profile with ridges across the outer part (gives the rumble)
    float rise = std::clamp(u / 0.25f, 0.0f, 1.0f);
    float fall = std::clamp((1.0f - u) / 0.12f, 0.0f, 1.0f);
    float base = kKerbHeight * rise * fall;
    float ridge = 0.5f + 0.5f * std::sin(s * (2.0f * 3.14159265f / 0.6f));
    return base * (0.75f + 0.25f * ridge);
}

float TrackData::terrainHeight(float x, float z) const {
    if (near_.contains(x, z)) return near_.sample(x, z);
    return far_.sample(x, z);
}

glm::vec3 TrackData::terrainNormal(float x, float z) const {
    if (near_.contains(x, z)) return near_.normal(x, z);
    return far_.normal(x, z);
}

glm::vec3 TrackData::landcoverAt(float x, float z) const {
    if (!near_.contains(x, z) || landcover_.empty()) return glm::vec3(0.0f, 1.0f, 0.0f);
    float gx = (x - near_.x0) / near_.res, gz = (z - near_.z0) / near_.res;
    int i = std::clamp(static_cast<int>(gx), 0, near_.w - 2);
    int j = std::clamp(static_cast<int>(gz), 0, near_.h - 2);
    float fx = gx - i, fz = gz - j;
    auto px = [&](int a, int b) {
        const uint8_t* p = &landcover_[(static_cast<size_t>(b) * near_.w + a) * 3];
        return glm::vec3(p[0], p[1], p[2]) / 255.0f;
    };
    return glm::mix(glm::mix(px(i, j), px(i + 1, j), fx), glm::mix(px(i, j + 1), px(i + 1, j + 1), fx), fz);
}

GroundInfo TrackData::ground(const glm::vec3& p, int& hint) const {
    GroundInfo gi;
    TrackLocation loc = locate(p, hint);
    hint = loc.seg;
    gi.loc = loc;
    const float s = loc.s;
    const int side = loc.lateral >= 0.0f ? 1 : -1;
    const float hw = halfWidthAt(s, side);
    const bool kerb = kerbAt(s, side);

    auto heightAt = [&](float ss, float lat, Surface* surf) -> float {
        float aw = std::abs(lat);
        float edge = hw;
        if (aw <= edge + (kerb ? kKerbOuter : 0.0f) && (aw <= edge || kerb)) {
            float y = roadHeight(ss, std::clamp(lat, -edge, edge));
            if (aw > edge) y -= (aw - edge) * 0.02f;  // kerb outer part follows a gentle slope
            Surface sf = Surface::Asphalt;
            if (kerb && aw > edge - kKerbInner) {
                float u = (aw - (edge - kKerbInner)) / (kKerbInner + kKerbOuter);
                y += kerbProfile(ss, u);
                sf = Surface::Kerb;
            }
            if (surf) *surf = sf;
            return y;
        }
        if (aw <= edge + kVergeWidth) {
            float u = (aw - edge) / kVergeWidth;
            float edgeY = roadHeight(ss, side * edge);
            float outer = interpScalar(ss, side > 0 ? &TrackSample::vergeOuterRightY : &TrackSample::vergeOuterLeftY);
            if (surf) *surf = Surface::Grass;
            return lerp(edgeY, outer, u);
        }
        glm::vec3 q = surfacePoint(ss, lat);
        if (surf) {
            glm::vec3 lc = landcoverAt(q.x, q.z);
            *surf = lc.x > 0.5f ? Surface::Dirt : Surface::Grass;
        }
        return terrainHeight(q.x, q.z);
    };

    // Off the ribbon (far from the centreline) use the terrain directly at p.
    if (std::abs(loc.lateral) > hw + kVergeWidth + 0.5f) {
        gi.height = terrainHeight(p.x, p.z);
        gi.normal = terrainNormal(p.x, p.z);
        glm::vec3 lc = landcoverAt(p.x, p.z);
        gi.surface = lc.x > 0.5f ? Surface::Dirt : Surface::Grass;
        gi.onRoad = false;
        return gi;
    }

    Surface surf;
    const float e = 0.08f;
    float h0 = heightAt(s, loc.lateral, &surf);
    float hl = heightAt(s, loc.lateral + e, nullptr);
    float hs = heightAt(s + e, loc.lateral, nullptr);
    glm::vec3 r = rightAt(s);
    glm::vec3 f(r.z, 0.0f, -r.x);  // forward along the track (horizontal)
    glm::vec3 A = r * e + glm::vec3(0, hl - h0, 0);
    glm::vec3 B = f * e + glm::vec3(0, hs - h0, 0);
    gi.height = h0;
    gi.normal = glm::normalize(glm::cross(A, B));
    gi.surface = surf;
    gi.onRoad = surf == Surface::Asphalt || surf == Surface::Kerb || surf == Surface::Concrete;
    return gi;
}

int TrackData::sectionIndex(float s) const {
    if (sections_.empty()) return -1;
    s = wrapS(s);
    int lo = 0, hi = static_cast<int>(sections_.size()) - 1, ans = hi;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        if (sections_[mid].start <= s) {
            ans = mid;
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }
    return ans;
}

const char* TrackData::sectionName(float s) const {
    int i = sectionIndex(s);
    return i < 0 ? "" : sections_[i].name.c_str();
}

}  // namespace apex
