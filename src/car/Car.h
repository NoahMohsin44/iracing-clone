#pragma once
#include <array>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "car/CarParams.h"
#include "car/Drivetrain.h"
#include "car/Tire.h"
#include "world/TrackData.h"

namespace apex {

struct CarInput {
    float steer = 0.0f;     // -1 full left .. +1 full right (fraction of the car's steering lock)
    float throttle = 0.0f;  // 0..1
    float brake = 0.0f;     // 0..1
    float clutch = 0.0f;    // 0..1 pedal travel (1 = fully disengaged)
    bool shiftUp = false;   // edge events
    bool shiftDown = false;
};

struct Wheel {
    // configuration
    glm::dvec3 hardpoint{0.0};  // body frame (x right, y up, z back), relative to CG
    int axle = 0;               // 0 front, 1 rear
    int side = -1;              // -1 left, +1 right
    double fullDroopLength = 0.3;
    double springPreload = 0.0;

    // state
    double travel = 0.0;        // suspension compression (m), 0 = full droop
    double wheelVel = 0.0;      // absolute velocity of the wheel along the body up axis
    double omega = 0.0;         // spin rate (rad/s)
    double spinAngle = 0.0;
    double slipX = 0.0, slipY = 0.0;  // transient (relaxed) slip ratio and tan(slip angle)
    double steerAngle = 0.0;
    double absFactor = 1.0;
    double brakeTemp = 60.0;    // deg C
    double twist = 0.0;         // contact patch torsion about the vertical (rad), front wheels
    double kickLowpass[2] = {0.0, 0.0};  // slow part of the contact normal tilt (wheel frame)
    int trackHint = -1;

    // per-step outputs
    bool contact = false;
    double load = 0.0;          // tyre normal force (N)
    double fx = 0.0, fy = 0.0;  // ground-plane tyre forces (N)
    double suspensionForce = 0.0;
    double combinedSlip = 0.0;
    double slideSpeed = 0.0;    // m/s of contact patch sliding (audio, smoke)
    double camber = 0.0;
    double brakeTorque = 0.0;
    double driveTorque = 0.0;
    double kingpinTorque = 0.0; // torque about the steering axis (Nm, + steers right)
    glm::dvec3 centre{0.0};
    glm::dvec3 contactPoint{0.0};
    glm::dvec3 contactNormal{0.0, 1.0, 0.0};
    Surface surface = Surface::Asphalt;
};

class Car {
public:
    void init(const CarParams& params, const TrackData* track);
    void resetTo(float s, float lateral);
    void step(double dt, const CarInput& input);

    // Assists / setup
    bool absEnabled = true;
    int tcLevel = 3;
    float brakeBias = 0.575f;
    void setAutoClutch(bool on) { drivetrain_.setAutoClutch(on); }
    void setAutoShift(bool on) { drivetrain_.setAutoShift(on); }

    // State
    const CarParams& params() const { return p_; }
    const glm::dvec3& position() const { return pos_; }
    const glm::dquat& orientation() const { return rot_; }
    const glm::dvec3& velocity() const { return vel_; }
    const glm::dvec3& angularVelocity() const { return angVel_; }
    glm::dmat3 rotation() const { return glm::mat3_cast(rot_); }
    glm::dvec3 forward() const { return -rotation()[2]; }
    glm::dvec3 up() const { return rotation()[1]; }
    glm::dvec3 right() const { return rotation()[0]; }
    double speed() const { return glm::length(vel_); }
    double forwardSpeed() const { return glm::dot(vel_, forward()); }
    const std::array<Wheel, 4>& wheels() const { return wheels_; }
    const Drivetrain& drivetrain() const { return drivetrain_; }
    Drivetrain& drivetrain() { return drivetrain_; }
    const TrackLocation& trackLocation() const { return loc_; }
    double steeringAngle() const { return steer_; }    // front wheel angle (rad)
    double steeringWheelDeg() const;                   // driver steering wheel angle
    // Torque at the steering column (Nm, + turns the wheel clockwise): the force feedback signal.
    double steeringTorque() const { return columnTorque_; }
    glm::dvec3 localAcceleration() const { return accelLocal_; }  // m/s^2 in body frame, excludes gravity
    bool absActive() const { return absActive_; }
    bool tcActive() const { return tcActive_; }
    double appliedThrottle() const { return throttle_; }
    double appliedBrake() const { return brake_; }
    double appliedClutch() const { return clutch_; }
    double lastImpact() const { return impact_; }       // m/s, decays
    double scrape() const { return scrape_; }           // 0..1 body/ground scraping
    bool needsReset() const { return needsReset_; }
    double downforce() const { return downforce_; }

private:
    void computeAero(const glm::dmat3& R, glm::dvec3& F, glm::dvec3& T);
    void collide(const glm::dmat3& R, glm::dvec3& F, glm::dvec3& T, double dt);

    CarParams p_;
    const TrackData* track_ = nullptr;
    TireModel tires_[2];  // front, rear

    glm::dvec3 pos_{0.0};
    glm::dquat rot_{1.0, 0.0, 0.0, 0.0};
    glm::dvec3 vel_{0.0};
    glm::dvec3 angVel_{0.0};
    glm::dvec3 accelLocal_{0.0};
    std::array<Wheel, 4> wheels_{};
    Drivetrain drivetrain_;
    TrackLocation loc_;
    int cgHint_ = -1;
    std::array<int, 24> bodyHints_{};

    double massSprung_ = 1000.0;
    double massUnsprung_ = 0.0;
    double steer_ = 0.0;
    double prevSteer_ = 0.0;
    double columnTorque_ = 0.0;
    double throttle_ = 0.0, brake_ = 0.0, clutch_ = 0.0;
    double tcFactor_ = 1.0;
    bool absActive_ = false, tcActive_ = false;
    double impact_ = 0.0, scrape_ = 0.0;
    double downforce_ = 0.0;
    bool needsReset_ = false;
    bool prevShiftUp_ = false, prevShiftDown_ = false;
};

}  // namespace apex
