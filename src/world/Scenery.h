#pragma once
#include <vector>

#include "gfx/Camera.h"
#include "gfx/Mesh.h"
#include "world/TrackData.h"

namespace apex {

// Static trackside scenery built from the track data: OSM buildings near the circuit (houses
// with pitched roofs, industrial halls), marshal posts and the parapets on the bridge decks.
// Vertex colour = albedo, colour.w = surface kind (see object.frag, mode 3); uv = (distance
// along the wall, height above the base) for windows and cladding.
class Scenery {
public:
    struct Cell {
        glm::dvec3 origin{0.0};
        glm::vec3 centre{0.0f};
        float radius = 0.0f;
        GpuMesh mesh;
    };
    enum Kind { Plain = 0, HouseWall = 1, IndustrialWall = 2, FlatRoof = 3, TileRoof = 4, Concrete = 5 };

    void build(const TrackData& track);
    const std::vector<Cell>& cells() const { return cells_; }
    void visible(const glm::dvec3& camPos, const Frustum& f, float maxDistance, std::vector<int>& out) const;
    int buildingCount() const { return buildings_; }

private:
    std::vector<Cell> cells_;
    int buildings_ = 0;
};

}  // namespace apex
