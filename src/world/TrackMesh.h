#pragma once
#include <vector>

#include "gfx/Camera.h"
#include "gfx/Mesh.h"
#include "world/TrackData.h"

namespace apex {

// Render geometry generated from the track definition, in 200 m chunks for culling:
// tarmac + kerbs, grass verges (terrain shader), double armco rails and posts.
class TrackMesh {
public:
    struct Chunk {
        glm::dvec3 origin{0.0};
        float s0 = 0.0f;
        glm::vec3 centre{0.0f};  // world
        float radius = 0.0f;
        GpuMesh road;    // tarmac (w=0) and kerbs (w=1)
        GpuMesh verge;
        GpuMesh rails;
        GpuMesh posts;
    };

    void build(const TrackData& track);
    const std::vector<Chunk>& chunks() const { return chunks_; }
    // indices of chunks visible from the camera within maxDistance
    void visible(const glm::dvec3& camPos, const Frustum& f, float maxDistance, std::vector<int>& out) const;

private:
    std::vector<Chunk> chunks_;
};

}  // namespace apex
