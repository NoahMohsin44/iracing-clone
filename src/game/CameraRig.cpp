#include "game/CameraRig.h"

#include <algorithm>
#include <cmath>

namespace apex {

namespace {
constexpr double kPi = 3.14159265358979323846;
double wrapAngle(double a) {
    while (a > kPi) a -= 2 * kPi;
    while (a < -kPi) a += 2 * kPi;
    return a;
}
glm::dmat3 yawMatrix(double a) {
    // rotation about +Y (positive = turn left)
    double c = std::cos(a), s = std::sin(a);
    return glm::dmat3(glm::dvec3(c, 0, -s), glm::dvec3(0, 1, 0), glm::dvec3(s, 0, c));
}
}  // namespace

glm::vec3 CameraRig::tvCameraSpot(const TrackData& track, int index, int& side) {
    float camS = track.wrapS(index * kTvSpacing + kTvSpacing * 0.15f);
    side = (index % 2 == 0) ? 1 : -1;
    // on the grass verge inside the armco, raised like a broadcast camera platform
    float lat = side * (track.halfWidthAt(camS, side) + 2.2f);
    glm::vec3 p = track.surfacePoint(camS, lat);
    p.y += 3.2f;
    return p;
}

const char* cameraModeName(CameraMode m) {
    switch (m) {
        case CameraMode::Cockpit: return "Cockpit";
        case CameraMode::Hood: return "Hood";
        case CameraMode::Chase: return "Chase";
        case CameraMode::FarChase: return "Far chase";
        case CameraMode::TV: return "TV";
        case CameraMode::Orbit: return "Orbit";
        default: return "";
    }
}

void CameraRig::update(double dt, const Car& car, const TrackData& track, CameraMode mode, const DriverControls& look,
                       float fovDeg) {
    if (mode != lastMode_) {
        initialised_ = false;
        lastMode_ = mode;
    }
    const glm::dmat3 R = car.rotation();
    const glm::dvec3 pos = car.position();
    const glm::dvec3 up = R[1];
    const glm::dvec3 fwd = -R[2];
    cam_.fovY = glm::radians(fovDeg);
    cam_.nearZ = 0.05f;

    double targetLook = look.lookBack ? kPi * 0.97 : (look.lookLeft ? 1.25 : (look.lookRight ? -1.25 : 0.0));
    lookYaw_ += (targetLook - lookYaw_) * std::min(1.0, dt * 10.0);

    glm::dvec3 hf = glm::dvec3(fwd.x, 0.0, fwd.z);
    double hlen = glm::length(hf);
    hf = hlen > 1e-3 ? hf / hlen : glm::dvec3(0, 0, -1);
    const double carYaw = std::atan2(hf.x, -hf.z);

    switch (mode) {
        case CameraMode::Cockpit: {
            inside_ = true;
            glm::dvec3 acc = car.localAcceleration();
            acc.y -= 9.81;
            glm::dvec3 target = glm::clamp(-acc * 0.0032, glm::dvec3(-0.035), glm::dvec3(0.035));
            target.y *= 0.5;
            if (!initialised_) headOffset_ = target;
            headOffset_ += (target - headOffset_) * std::min(1.0, dt * 7.0);
            glm::dvec3 eye = glm::dvec3(car.params().driverEye) + headOffset_;
            cam_.position = pos + R * eye;
            cam_.basis = glm::mat3(R * yawMatrix(lookYaw_));
            cam_.nearZ = 0.03f;
            break;
        }
        case CameraMode::Hood: {
            inside_ = false;
            cam_.position = pos + R * glm::dvec3(0.0, 0.33, -0.95);
            cam_.basis = glm::mat3(R * yawMatrix(lookYaw_));
            break;
        }
        case CameraMode::Chase:
        case CameraMode::FarChase: {
            inside_ = false;
            const double dist = mode == CameraMode::Chase ? 6.0 : 10.5;
            const double height = mode == CameraMode::Chase ? 1.65 : 2.9;
            if (!initialised_) chaseYaw_ = carYaw;
            chaseYaw_ += wrapAngle(carYaw - chaseYaw_) * std::min(1.0, dt * 3.5);
            double yaw = chaseYaw_ + (look.lookBack ? kPi : 0.0);
            glm::dvec3 dir(std::sin(yaw), 0.0, -std::cos(yaw));
            glm::dvec3 desired = pos - dir * dist + glm::dvec3(0.0, height, 0.0);
            if (!initialised_) chasePos_ = desired;
            chasePos_ = desired + (chasePos_ - desired) * std::exp(-dt * 14.0);
            double ground = track.terrainHeight(static_cast<float>(chasePos_.x), static_cast<float>(chasePos_.z));
            int hint = -1;
            GroundInfo g = track.ground(glm::vec3(chasePos_), hint);
            ground = std::max(ground, static_cast<double>(g.height));
            chasePos_.y = std::max(chasePos_.y, ground + 0.6);
            cam_.lookAt(chasePos_, pos + up * 0.45 + dir * 1.5);
            break;
        }
        case CameraMode::TV: {
            inside_ = false;
            float s = car.trackLocation().s;
            int idx = static_cast<int>(std::floor((s + kTvSpacing * 0.45f) / kTvSpacing));
            idx %= tvCameraCount(track);
            int side = 1;
            glm::dvec3 camPos(tvCameraSpot(track, idx, side));
            glm::dvec3 target = pos + up * 0.4;
            double d = glm::length(target - camPos);
            cam_.lookAt(camPos, target);
            cam_.fovY = static_cast<float>(std::clamp(2.0 * std::atan(5.0 / std::max(d, 1.0)), glm::radians(6.0), glm::radians(55.0)));
            break;
        }
        case CameraMode::Orbit:
        default: {
            inside_ = false;
            orbitAngle_ += dt * 0.10;
            double a = carYaw + kPi * 0.75 + std::sin(orbitAngle_) * 0.9;
            glm::dvec3 dir(std::sin(a), 0.0, -std::cos(a));
            glm::dvec3 desired = pos + dir * 6.8 + glm::dvec3(0.0, 1.35, 0.0);
            if (!initialised_) chasePos_ = desired;
            chasePos_ = desired + (chasePos_ - desired) * std::exp(-dt * 6.0);
            int hint = -1;
            GroundInfo g = track.ground(glm::vec3(chasePos_), hint);
            chasePos_.y = std::max(chasePos_.y, static_cast<double>(g.height) + 0.5);
            cam_.lookAt(chasePos_, pos + up * 0.35);
            cam_.fovY = glm::radians(45.0f);
            break;
        }
    }
    initialised_ = true;
}

}  // namespace apex
