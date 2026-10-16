#pragma once
#include "car/Car.h"
#include "core/Input.h"
#include "gfx/Camera.h"

namespace apex {

enum class CameraMode : int { Cockpit = 0, Hood, Chase, FarChase, TV, Orbit, Count };
const char* cameraModeName(CameraMode m);

class CameraRig {
public:
    // Trackside TV camera positions (also kept clear of trees by the vegetation builder).
    static constexpr float kTvSpacing = 260.0f;
    static glm::vec3 tvCameraSpot(const TrackData& track, int index, int& side);
    static int tvCameraCount(const TrackData& track) { return static_cast<int>(track.length() / kTvSpacing) + 1; }

    void reset() { initialised_ = false; }
    void update(double dt, const Car& car, const TrackData& track, CameraMode mode, const DriverControls& look,
                float fovDeg);
    const Camera& camera() const { return cam_; }
    bool inside() const { return inside_; }

private:
    Camera cam_;
    bool initialised_ = false;
    bool inside_ = true;
    glm::dvec3 chasePos_{0.0};
    glm::dvec3 chaseVel_{0.0};
    double chaseYaw_ = 0.0;
    glm::dvec3 headOffset_{0.0};
    double lookYaw_ = 0.0;
    double orbitAngle_ = 0.0;
    float tvS_ = -1.0f;
    int tvSide_ = 1;
    CameraMode lastMode_ = CameraMode::Count;
};

}  // namespace apex
