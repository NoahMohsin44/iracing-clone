#include "car/Drivetrain.h"

#include <algorithm>
#include <cmath>

namespace apex {

namespace {
constexpr double kRpmToRad = 2.0 * 3.14159265358979323846 / 60.0;

double smoothstep(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}
}  // namespace

void Drivetrain::init(const CarParams& p) {
    p_ = p;
    reset(0.0);
}

void Drivetrain::reset(double wheelOmega) {
    gear_ = 0;
    targetGear_ = 0;
    shiftTimer_ = 0.0;
    omegaEngine_ = p_.engine.idleRpm * kRpmToRad;
    limiterCut_ = false;
    clutchLocked_ = false;
    tcFactor = 1.0;
    lastWheelAvg_ = wheelOmega;
}

double Drivetrain::totalRatio(int gear) const {
    if (gear == 0) return 0.0;
    if (gear < 0) return -p_.gearbox.reverse * p_.gearbox.finalDrive;
    int idx = std::min(gear, gearCount()) - 1;
    return p_.gearbox.ratios[idx] * p_.gearbox.finalDrive;
}

double Drivetrain::fullTorque(double rpm) const {
    const auto& c = p_.engine.torqueCurve;
    if (c.empty()) return 0.0;
    if (rpm <= c.front().first) return c.front().second;
    for (size_t i = 1; i < c.size(); ++i) {
        if (rpm <= c[i].first) {
            double t = (rpm - c[i - 1].first) / (c[i].first - c[i - 1].first);
            return c[i - 1].second + (c[i].second - c[i - 1].second) * t;
        }
    }
    // beyond the table: torque collapses quickly
    double over = rpm - c.back().first;
    return std::max(0.0, c.back().second - over * 0.8);
}

void Drivetrain::requestShift(int dir) {
    if (shiftTimer_ > 0.0 || dir == 0) return;
    int target = gear_ + (dir > 0 ? 1 : -1);
    if (target > gearCount()) return;
    if (target < -1) return;
    if (target == -1 && std::abs(lastWheelAvg_) > 6.0) return;  // reverse only when (almost) stopped
    if (dir < 0 && target >= 1) {
        double predicted = std::abs(lastWheelAvg_ * totalRatio(target)) / kRpmToRad;
        if (predicted > p_.engine.maxRpm - 150.0) return;  // money-shift protection
    }
    targetGear_ = target;
    bool toFromNeutral = (gear_ <= 0 || target <= 0);
    shiftTimer_ = toFromNeutral ? 0.04 : (dir > 0 ? p_.gearbox.upshiftTime : p_.gearbox.downshiftTime);
}

void Drivetrain::step(double dt, double throttle, double clutchPedal, WheelSide& left, WheelSide& right,
                      double& torqueLeft, double& torqueRight) {
    const double wheelAvg = 0.5 * (left.omega + right.omega);
    lastWheelAvg_ = wheelAvg;

    // ------------------------------------------------------------ gear changes
    bool upshifting = false, downshifting = false;
    if (shiftTimer_ > 0.0) {
        upshifting = targetGear_ > gear_;
        downshifting = targetGear_ < gear_;
        shiftTimer_ -= dt;
        if (shiftTimer_ <= 0.0) {
            shiftTimer_ = 0.0;
            gear_ = targetGear_;
            ++shiftEvent_;
            upshifting = downshifting = false;
        }
    }
    autoShiftCooldown_ = std::max(0.0, autoShiftCooldown_ - dt);
    double engineRpm = rpm();
    if (autoShift_ && shiftTimer_ <= 0.0 && autoShiftCooldown_ <= 0.0) {
        if (gear_ == 0 && throttle > 0.1) {
            requestShift(+1);
            autoShiftCooldown_ = 0.3;
        } else if (gear_ >= 1 && gear_ < gearCount() && engineRpm > p_.engine.limiterRpm - 120.0) {
            requestShift(+1);
            autoShiftCooldown_ = 0.35;
        } else if (gear_ > 1) {
            double ratioDown = totalRatio(gear_ - 1) / totalRatio(gear_);
            double rpmAfter = engineRpm * ratioDown;
            bool safe = rpmAfter < p_.engine.limiterRpm - 900.0;
            if (safe && engineRpm < 5300.0) {
                requestShift(-1);
                autoShiftCooldown_ = 0.25;
            }
        }
    }

    const bool inShift = shiftTimer_ > 0.0;
    const double G = inShift ? 0.0 : totalRatio(gear_);

    // ------------------------------------------------------------ engine torque
    double th = std::clamp(throttle, 0.0, 1.0) * std::clamp(tcFactor, 0.0, 1.0);
    if (upshifting) th = 0.0;  // ignition cut
    if (downshifting) {
        double targetOmega = std::abs(wheelAvg * totalRatio(targetGear_));
        th = std::clamp((targetOmega - omegaEngine_) * 0.03, 0.0, 1.0);  // auto-blip
    }
    double idleTh = std::clamp((p_.engine.idleRpm - engineRpm) / 450.0, 0.0, 0.35);
    th = std::max(th, idleTh);
    if (engineRpm > p_.engine.limiterRpm) limiterCut_ = true;
    if (engineRpm < p_.engine.limiterRpm - 140.0) limiterCut_ = false;
    if (limiterCut_) th = 0.0;
    double friction = p_.engine.frictionTorque + p_.engine.frictionPerRpm * engineRpm;
    double tEngine = th * fullTorque(engineRpm) - (1.0 - th) * friction;
    if (engineRpm < 350.0) tEngine += 120.0;  // starter / anti-stall restart
    engineTorque_ = tEngine;
    effThrottle_ = th;
    load_ = th;

    // ------------------------------------------------------------ clutch
    double userEng = smoothstep((1.0 - std::clamp(clutchPedal, 0.0, 1.0) - 0.12) / 0.7);
    double eng = userEng;
    if (autoClutch_) {
        double launch = p_.engine.idleRpm + 450.0 + 2600.0 * std::clamp(throttle, 0.0, 1.0);
        double lo = p_.engine.idleRpm + 80.0;
        eng = std::min(eng, smoothstep((engineRpm - lo) / (launch - lo)));
    }
    // anti-stall: always open the clutch before the engine dies
    double stall = p_.engine.idleRpm * 0.72;
    eng = std::min(eng, smoothstep((engineRpm - 650.0) / (stall - 650.0)));
    engagement_ = eng;
    const double capacity = p_.gearbox.clutchMaxTorque * eng;
    const double eff = p_.gearbox.efficiency;
    const double Ie = p_.engine.inertia;
    const double Iw = 0.5 * (left.inertia + right.inertia);

    double tc = 0.0;
    clutchLocked_ = false;
    if (G != 0.0 && capacity > 1e-3) {
        double omegaD = G * wheelAvg;
        double extAvg = 0.5 * (left.externalTorque + right.externalTorque);
        double num = (omegaEngine_ - omegaD) + dt * (tEngine / Ie - G * extAvg / Iw);
        double den = dt * (1.0 / Ie + G * G * eff / (2.0 * Iw));
        tc = num / den;
        if (std::abs(tc) <= capacity) {
            clutchLocked_ = true;
        } else {
            tc = std::copysign(capacity, tc);
        }
    }
    clutchTorque_ = tc;
    omegaEngine_ += dt * (tEngine - tc) / Ie;
    omegaEngine_ = std::clamp(omegaEngine_, 0.0, (p_.engine.maxRpm + 600.0) * kRpmToRad);

    // ------------------------------------------------------------ differential (clutch-pack LSD)
    const double tin = tc * G * eff;
    const double cap = p_.diff.preload + (tin >= 0.0 ? p_.diff.powerRamp : p_.diff.coastRamp) * std::abs(tin);
    double num = (right.omega - left.omega) +
                 dt * ((right.externalTorque + 0.5 * tin) / right.inertia - (left.externalTorque + 0.5 * tin) / left.inertia);
    double den = dt * (1.0 / left.inertia + 1.0 / right.inertia);
    double bias = std::clamp(num / den, -cap, cap);
    torqueLeft = 0.5 * tin + bias;
    torqueRight = 0.5 * tin - bias;
}

void Drivetrain::postWheelUpdate(double omegaLeft, double omegaRight) {
    const double G = shiftTimer_ > 0.0 ? 0.0 : totalRatio(gear_);
    if (clutchLocked_ && G != 0.0) omegaEngine_ = std::max(0.0, G * 0.5 * (omegaLeft + omegaRight));
}

}  // namespace apex
