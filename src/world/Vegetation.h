#pragma once
#include <vector>

#include "gfx/Camera.h"
#include "gfx/Mesh.h"
#include "world/TrackData.h"

namespace apex {

// Forest along the circuit: spruce and beech trees placed from the OpenStreetMap forest mask,
// drawn instanced with three LODs (branch cards, coarse cards, impostor) and per-cell culling.
class Vegetation {
public:
    struct Stats {
        int instances = 0;
        int drawnCells = 0;
        int drawCalls = 0;
    };

    void build(const TrackData& track, int quality);
    // Draws with the currently bound tree shader. camPos for camera-relative culling.
    void draw(const glm::dvec3& camPos, const Frustum& frustum, float maxDistance, int minLod = 0);
    float bandWidth() const { return band_; }
    const Stats& stats() const { return stats_; }
    int quality() const { return quality_; }

private:
    struct Instance {
        glm::vec4 posScale;
        glm::vec4 rotTint;
    };
    struct Cell {
        glm::vec3 centre;
        float radius = 0.0f;
        uint32_t start[2] = {0, 0};
        uint32_t count[2] = {0, 0};
    };
    std::vector<Cell> cells_;
    GLuint instanceBuffer_ = 0;
    GpuMesh meshes_[2][3];
    float lod0_ = 70.0f, lod1_ = 260.0f;
    float band_ = 140.0f;
    int quality_ = -1;
    Stats stats_;
};

}  // namespace apex
