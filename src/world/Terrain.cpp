#include "world/Terrain.h"

#include <algorithm>
#include <cmath>

#include "core/Log.h"

namespace apex {

namespace {
// Grid of (n+1)^2 vertices with unit cell size scaled at draw time, plus a skirt ring.
MeshData makeGrid(int n, float cell) {
    MeshData m;
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i) {
            Vertex v;
            v.pos = glm::vec3(i * cell, 0.0f, j * cell);
            m.vertices.push_back(v);
        }
    auto idx = [n](int i, int j) { return static_cast<uint32_t>(j * (n + 1) + i); };
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            // same diagonal as Heightmap::sample, wound counter-clockwise seen from above
            m.addTri(idx(i, j), idx(i + 1, j + 1), idx(i + 1, j));
            m.addTri(idx(i, j), idx(i, j + 1), idx(i + 1, j + 1));
        }
    // skirts along the four borders
    auto skirt = [&](int i0, int j0, int di, int dj) {
        for (int k = 0; k < n; ++k) {
            int ia = i0 + di * k, ja = j0 + dj * k, ib = ia + di, jb = ja + dj;
            uint32_t a = idx(ia, ja), b = idx(ib, jb);
            Vertex va = m.vertices[a], vb = m.vertices[b];
            va.color.w = vb.color.w = 1.0f;
            uint32_t sa = m.addVertex(va), sb = m.addVertex(vb);
            m.addTri(a, b, sb);
            m.addTri(a, sb, sa);
        }
    };
    skirt(0, 0, 1, 0);
    skirt(0, n, 1, 0);
    skirt(0, 0, 0, 1);
    skirt(n, 0, 0, 1);
    return m;
}

void uploadHeight(Texture& tex, const Heightmap& hm) {
    tex.create2D(hm.w, hm.h, GL_R32F, 1);
    tex.upload(hm.data.data(), GL_RED, GL_FLOAT);
    tex.setFilter(GL_LINEAR, GL_LINEAR);
    tex.setWrap(GL_CLAMP_TO_EDGE);
}
}  // namespace

void Terrain::build(const TrackData& track) {
    const Heightmap& hn = track.nearTerrain();
    const Heightmap& hf = track.farTerrain();
    uploadHeight(heightNear_, hn);
    uploadHeight(heightFar_, hf);
    nearHm_ = glm::vec4(hn.x0, hn.z0, hn.res, 0.0f);
    farHm_ = glm::vec4(hf.x0, hf.z0, hf.res, 0.0f);
    nearSize_ = glm::vec2(hn.w, hn.h);
    farSize_ = glm::vec2(hf.w, hf.h);
    const float holeMargin = hf.res * 2.5f;
    hole_ = glm::vec4(hn.x0 + holeMargin, hn.z0 + holeMargin, hn.x0 + hn.width() - holeMargin, hn.z0 + hn.depth() - holeMargin);

    // land cover (RGB -> RGBA)
    const auto& lc = track.landcover();
    std::vector<uint8_t> rgba(static_cast<size_t>(hn.w) * hn.h * 4, 255);
    for (size_t i = 0; i < static_cast<size_t>(hn.w) * hn.h && i * 3 + 2 < lc.size(); ++i) {
        rgba[i * 4 + 0] = lc[i * 3 + 0];
        rgba[i * 4 + 1] = lc[i * 3 + 1];
        rgba[i * 4 + 2] = lc[i * 3 + 2];
    }
    landcover_.create2D(hn.w, hn.h, GL_RGBA8, Texture::mipCount(hn.w, hn.h));
    landcover_.upload(rgba.data(), GL_RGBA, GL_UNSIGNED_BYTE);
    landcover_.generateMips();
    landcover_.setFilter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);

    // distance to the centreline (two-pass chamfer transform), for the canopy shell
    {
        const int W = hn.w, H = hn.h;
        std::vector<float> d(static_cast<size_t>(W) * H, 1e9f);
        for (const auto& s : track.samples()) {
            int i = static_cast<int>(std::round((s.pos.x - hn.x0) / hn.res));
            int j = static_cast<int>(std::round((s.pos.z - hn.z0) / hn.res));
            if (i >= 0 && j >= 0 && i < W && j < H) d[static_cast<size_t>(j) * W + i] = 0.0f;
        }
        const float a = hn.res, b = hn.res * 1.41421f;
        auto at = [&](int i, int j) -> float& { return d[static_cast<size_t>(j) * W + i]; };
        for (int j = 0; j < H; ++j)
            for (int i = 0; i < W; ++i) {
                float& v = at(i, j);
                if (i > 0) v = std::min(v, at(i - 1, j) + a);
                if (j > 0) v = std::min(v, at(i, j - 1) + a);
                if (i > 0 && j > 0) v = std::min(v, at(i - 1, j - 1) + b);
                if (i + 1 < W && j > 0) v = std::min(v, at(i + 1, j - 1) + b);
            }
        for (int j = H - 1; j >= 0; --j)
            for (int i = W - 1; i >= 0; --i) {
                float& v = at(i, j);
                if (i + 1 < W) v = std::min(v, at(i + 1, j) + a);
                if (j + 1 < H) v = std::min(v, at(i, j + 1) + a);
                if (i + 1 < W && j + 1 < H) v = std::min(v, at(i + 1, j + 1) + b);
                if (i > 0 && j + 1 < H) v = std::min(v, at(i - 1, j + 1) + b);
            }
        trackDist_.create2D(W, H, GL_R32F, 1);
        trackDist_.upload(d.data(), GL_RED, GL_FLOAT);
        trackDist_.setFilter(GL_LINEAR, GL_LINEAR);
    }

    for (int l = 0; l < 4; ++l) grid_[l].upload(makeGrid(kChunkCells >> l, static_cast<float>(1 << l)));

    auto makeChunks = [&](const Heightmap& hm, std::vector<Chunk>& out) {
        out.clear();
        const float chunkSize = kChunkCells * hm.res;
        int nx = static_cast<int>(std::ceil((hm.w - 1) / static_cast<float>(kChunkCells)));
        int nz = static_cast<int>(std::ceil((hm.h - 1) / static_cast<float>(kChunkCells)));
        for (int cz = 0; cz < nz; ++cz)
            for (int cx = 0; cx < nx; ++cx) {
                Chunk c;
                c.origin = glm::vec2(hm.x0 + cx * chunkSize, hm.z0 + cz * chunkSize);
                float mn = 1e9f, mx = -1e9f;
                for (int j = cz * kChunkCells; j <= std::min(hm.h - 1, (cz + 1) * kChunkCells); j += 4)
                    for (int i = cx * kChunkCells; i <= std::min(hm.w - 1, (cx + 1) * kChunkCells); i += 4) {
                        mn = std::min(mn, hm.at(i, j));
                        mx = std::max(mx, hm.at(i, j));
                    }
                c.centre = glm::vec3(c.origin.x + chunkSize * 0.5f, (mn + mx) * 0.5f, c.origin.y + chunkSize * 0.5f);
                c.radius = std::sqrt(2.0f * (chunkSize * 0.5f) * (chunkSize * 0.5f) + (mx - mn) * (mx - mn) * 0.25f) + 30.0f;
                out.push_back(c);
            }
    };
    makeChunks(hn, near_);
    makeChunks(hf, far_);
    LOGI("Terrain: %zu near chunks (%dx%d @ %.0f m), %zu far chunks", near_.size(), hn.w, hn.h, hn.res, far_.size());
}

void Terrain::bindTextures() const {
    heightNear_.bind(0);
    landcover_.bind(13);
    trackDist_.bind(14);
}

void Terrain::setNearUniforms(const Shader& sh) const {
    sh.set("uHm", nearHm_);
    sh.set("uHmSize", nearSize_);
    sh.set("uHmLand", nearHm_);
    sh.set("uHmLandSize", nearSize_);
}

void Terrain::drawChunks(const Shader& sh, const std::vector<Chunk>& chunks, float cellBase, const glm::dvec3& camPos,
                         const Frustum& f, float maxDist, int minLod, float lodScale) const {
    for (const Chunk& c : chunks) {
        glm::vec3 rel = glm::vec3(glm::dvec3(c.centre) - camPos);
        float d = glm::length(rel);
        if (d - c.radius > maxDist) continue;
        if (!f.sphere(rel, c.radius)) continue;
        float dd = std::max(0.0f, d - c.radius * 0.5f) / lodScale;
        int lod = dd < 260.0f ? 0 : dd < 700.0f ? 1 : dd < 1600.0f ? 2 : 3;
        lod = std::max(lod, minLod);
        sh.set("uChunkOrigin", c.origin);
        // grid meshes are built with unit cells times 2^lod; scale to world via the model grid size
        sh.set("uCellScale", cellBase);
        grid_[lod].draw();
    }
}

void Terrain::drawNear(const Shader& sh, const glm::dvec3& camPos, const Frustum& f, float maxDist, int minLod) const {
    heightNear_.bind(0);
    setNearUniforms(sh);
    sh.set("uHole", glm::vec4(0.0f));
    drawChunks(sh, near_, nearHm_.z, camPos, f, maxDist, minLod, 1.0f);
}

void Terrain::drawFar(const Shader& sh, const glm::dvec3& camPos, const Frustum& f, float maxDist) const {
    heightFar_.bind(0);
    sh.set("uHm", farHm_);
    sh.set("uHmSize", farSize_);
    sh.set("uHmLand", nearHm_);
    sh.set("uHmLandSize", nearSize_);
    sh.set("uHole", hole_);
    drawChunks(sh, far_, farHm_.z, camPos, f, maxDist, 1, farHm_.z / nearHm_.z * 0.5f);
    heightNear_.bind(0);
}

void Terrain::drawCanopy(const Shader& sh, const glm::dvec3& camPos, const Frustum& f, float maxDist, float canopyStart) const {
    heightNear_.bind(0);
    setNearUniforms(sh);
    sh.set("uHole", glm::vec4(0.0f));
    sh.set("uCanopyHeight", 23.0f);
    sh.set("uCanopyStart", canopyStart);
    drawChunks(sh, near_, nearHm_.z, camPos, f, maxDist, 1, 1.0f);
}

}  // namespace apex
