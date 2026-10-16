#pragma once
#include <string>

#include <glm/glm.hpp>

#include "gfx/Mesh.h"

namespace apex {

// Material ids stored in Vertex::color.w for the car shader.
namespace carmat {
constexpr float Body = 0, Carbon = 1, BlackPlastic = 2, Glass = 3, Headlight = 4, Taillight = 5, Chrome = 6,
                Rubber = 7, Rim = 8, BrakeDisc = 9, Caliper = 10, Interior = 11, Seat = 12, Display = 13,
                ArchLiner = 14, ShiftLeds = 15, Cage = 16, Helmet = 17, Visor = 18, Suit = 19, Trim = 20;
}

// Procedurally modelled GT3 coupe (front-mid engine, fastback, big rear wing), built in the
// physics body frame: x right, y up, z back, origin at the centre of gravity.
class CarModel {
public:
    struct Geometry {
        float groundY = -0.43f;
        float frontAxleZ = -1.4445f, rearAxleZ = 1.2555f;
        float frontTrack = 1.66f, rearTrack = 1.62f;
        float frontRadius = 0.340f, rearRadius = 0.355f;
        glm::vec3 steeringWheelCentre{-0.37f, 0.33f, -0.29f};
        float steeringWheelTilt = 0.42f;  // rad from vertical
    };

    void build(const Geometry& g);
    // Loads the Blender-built model (tools/build_car.py). Returns false if the file is missing or
    // incomplete; build() is the procedural fallback.
    bool load(const std::string& glbPath, const Geometry& g);
    bool loaded() const { return loaded_; }

    GpuMesh body;          // painted shell (opaque + glass regions resolved in the shader)
    GpuMesh parts;         // splitter, wing, diffuser, mirrors, liners, exhausts, interior
    GpuMesh driver;        // helmet, torso, arms (hidden in the cockpit view)
    GpuMesh steeringWheel; // local to the steering wheel centre, column along -z
    GpuMesh wheel[2];      // front, rear: tyre + rim + disc, axle along x, outward = +x
    GpuMesh caliper[2];    // non-spinning
    Geometry geo;

    // wheel arch openings (z, y, radius) used by the shader to cut the body
    glm::vec4 arches[2];

private:
    bool loaded_ = false;
};

}  // namespace apex
