#pragma once
#include "car/CarParams.h"

namespace apex {

// Engine -> clutch -> sequential gearbox -> clutch-pack LSD -> rear wheels.
// Clutch and differential are solved as friction constraints each step (lock if the required
// torque is within capacity), which stays stable at stiff couplings.
class Drivetrain {
public:
    struct WheelSide {
        double omega = 0.0;        // wheel angular velocity (rad/s)
        double inertia = 1.0;      // wheel inertia
        double externalTorque = 0; // predicted torque from tyre/rolling (excluding brakes)
        double brakeTorque = 0;    // brake torque magnitude this step (opposes the rotation)
    };

    void init(const CarParams& p);
    void reset(double wheelOmega);

    // Driver controls
    void requestShift(int dir);          // +1 up / -1 down
    void setAutoClutch(bool on) { autoClutch_ = on; }
    void setAutoShift(bool on) { autoShift_ = on; }

    // Solve this step. Returns drive torques for the two driven wheels.
    void step(double dt, double throttle, double clutchPedal, WheelSide& left, WheelSide& right,
              double& torqueLeft, double& torqueRight);
    // After the wheels have been integrated: keeps engine and wheels exactly in sync when locked.
    void postWheelUpdate(double omegaLeft, double omegaRight);

    // Telemetry
    double rpm() const { return omegaEngine_ * 60.0 / (2.0 * 3.14159265358979323846); }
    double engineOmega() const { return omegaEngine_; }
    int gear() const { return gear_; }                 // -1 R, 0 N, 1..n
    int targetGear() const { return shiftTimer_ > 0 ? targetGear_ : gear_; }
    bool shifting() const { return shiftTimer_ > 0; }
    double engineTorque() const { return engineTorque_; }
    double effectiveThrottle() const { return effThrottle_; }
    bool limiterActive() const { return limiterCut_; }
    bool clutchLocked() const { return clutchLocked_; }
    double clutchEngagement() const { return engagement_; }
    double load() const { return load_; }              // 0..1 engine load for audio
    int gearCount() const { return static_cast<int>(p_.gearbox.ratios.size()); }
    double totalRatio(int gear) const;
    int lastShiftEvent() const { return shiftEvent_; } // increments every completed shift
    double tcFactor = 1.0;                             // traction control torque cut (1 = none)

private:
    double fullTorque(double rpm) const;

    CarParams p_;
    double omegaEngine_ = 200.0;
    int gear_ = 0;
    int targetGear_ = 0;
    double shiftTimer_ = 0.0;
    bool autoClutch_ = true;
    bool autoShift_ = false;
    bool limiterCut_ = false;
    bool clutchLocked_ = false;
    double engagement_ = 0.0;
    double engineTorque_ = 0.0;
    double effThrottle_ = 0.0;
    double load_ = 0.0;
    double clutchTorque_ = 0.0;
    double autoShiftCooldown_ = 0.0;
    int shiftEvent_ = 0;
    double lastWheelAvg_ = 0.0;  // driven-wheel average, used for shift protection
};

}  // namespace apex
