#pragma once
#include <vector>

#include "gfx/Camera.h"
#include "gfx/Mesh.h"
#include "gfx/Shader.h"
#include "gfx/Texture.h"
#include "world/TrackData.h"

namespace apex {

// GPU-displaced heightmap terrain: a 4 m grid around the circuit (chunked with LODs and skirts),
// a 48 m horizon terrain around it, and the forest canopy shell beyond the modelled trees.
class Terrain {
public:
    void build(const TrackData& track);
    void bindTextures() const;  // height near(0)/landcover(13)/track distance(14)
    // shader must be terrain.vert based; sets per-chunk uniforms
    void drawNear(const Shader& sh, const glm::dvec3& camPos, const Frustum& f, float maxDist, int minLod = 0) const;
    void drawFar(const Shader& sh, const glm::dvec3& camPos, const Frustum& f, float maxDist) const;
    void drawCanopy(const Shader& sh, const glm::dvec3& camPos, const Frustum& f, float maxDist, float canopyStart) const;
    void setNearUniforms(const Shader& sh) const;

private:
    struct Chunk {
        glm::vec2 origin;
        glm::vec3 centre;
        float radius;
    };
    void drawChunks(const Shader& sh, const std::vector<Chunk>& chunks, float cellBase, const glm::dvec3& camPos,
                    const Frustum& f, float maxDist, int minLod, float lodScale) const;

    Texture heightNear_, heightFar_, landcover_, trackDist_;
    GpuMesh grid_[4];
    std::vector<Chunk> near_, far_;
    glm::vec4 nearHm_, farHm_;
    glm::vec2 nearSize_, farSize_;
    glm::vec4 hole_;
    static constexpr int kChunkCells = 64;
};

}  // namespace apex
