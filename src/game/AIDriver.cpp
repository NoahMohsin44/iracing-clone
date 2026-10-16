#include "game/AIDriver.h"

#include <algorithm>
#include <cmath>

namespace apex {

CarInput AIDriver::update(const Car& car, double dt) {
    CarInput in;
    const TrackLocation& loc = car.trackLocation();
    const double v = std::max(0.0, car.forwardSpeed());
    const glm::dmat3 R = car.rotation();
    const glm::dmat3 Rt = glm::transpose(R);

    // ---- steering: pure pursuit on the racing line
    const double look = std::clamp(5.0 + 0.42 * v, 7.0, 42.0);
    const float sTarget = loc.s + static_cast<float>(look);
    glm::dvec3 target(track_->surfacePoint(sTarget, track_->racingLineAt(sTarget)));
    glm::dvec3 d = Rt * (target - car.position());
    const double lat = d.x, fwd = -d.z;
    const double L2 = std::max(lat * lat + fwd * fwd, 1.0);
    const double kappa = 2.0 * lat / L2;
    const double wheelbase = car.params().wheelbase;
    double steerAngle = std::atan(kappa * wheelbase);
    // yaw-rate feedback: correct oversteer/understeer (yaw about +Y is positive to the left)
    const double yawRate = -glm::dot(car.angularVelocity(), car.up());  // + = turning right
    const double desiredYaw = v * kappa;
    steerAngle += 0.10 * (desiredYaw - yawRate);
    double steer = steerAngle / car.params().front.maxSteer;
    const double alpha = 1.0 - std::exp(-dt / 0.035);
    steerFiltered_ += (steer - steerFiltered_) * alpha;
    in.steer = static_cast<float>(std::clamp(steerFiltered_, -1.0, 1.0));

    // ---- speed: follow the profile, anticipate actuator lag
    const double vTarget = track_->speedHintAt(loc.s + static_cast<float>(v * 0.30)) * pace_;
    const double err = vTarget - v;
    double th = 0.0, br = 0.0;
    if (err > 0.0) {
        th = std::clamp(0.35 + err * 0.45, 0.0, 1.0);
    } else if (err > -1.2) {
        th = 0.25;
    } else {
        br = std::clamp((-err - 1.2) * 0.22, 0.0, 1.0);
    }
    // stability: ease off the brakes when the rear steps out
    const double yawErr = yawRate - desiredYaw;
    if (std::abs(yawErr) > 0.25 && br > 0.0) br *= std::clamp(1.0 - (std::abs(yawErr) - 0.25) * 2.5, 0.2, 1.0);
    // lift when sliding
    const glm::dvec3 vl = Rt * car.velocity();
    const double slip = v > 5.0 ? std::abs(std::atan2(vl.x, -vl.z)) : 0.0;
    if (slip > 0.10) th *= std::clamp(1.0 - (slip - 0.10) * 8.0, 0.0, 1.0);
    const double rate = dt / 0.06;
    throttle_ += std::clamp(th - throttle_, -rate * 2.0, rate);
    brake_ += std::clamp(br - brake_, -rate * 2.0, rate * 2.0);
    in.throttle = static_cast<float>(throttle_);
    in.brake = static_cast<float>(brake_);
    return in;
}

}  // namespace apex
