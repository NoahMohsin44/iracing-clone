#include "car/Car.h"

#include <algorithm>
#include <cmath>

#include "core/Noise.h"

namespace apex {

namespace {
constexpr double kG = 9.81;
constexpr double kRho = 1.225;

glm::dvec3 toD(const glm::vec3& v) { return glm::dvec3(v); }

// Body points used for barrier contact (body frame, relative to CG).
const glm::dvec3 kBarrierPoints[] = {
    {-0.97, -0.12, -2.18}, {0.97, -0.12, -2.18}, {-1.00, -0.12, -1.20}, {1.00, -0.12, -1.20},
    {-1.01, -0.12, 0.10},  {1.01, -0.12, 0.10},  {-1.00, -0.12, 1.40},  {1.00, -0.12, 1.40},
    {-0.92, -0.12, 2.32},  {0.92, -0.12, 2.32},  {0.0, -0.12, -2.38},   {0.0, -0.12, 2.42},
};
// Floor points (splitter, plank corners) for scraping/bottoming.
const glm::dvec3 kFloorPoints[] = {
    {-0.80, 0.0, -2.20}, {0.80, 0.0, -2.20}, {-0.85, 0.0, -0.4}, {0.85, 0.0, -0.4},
    {-0.85, 0.0, 1.6},   {0.85, 0.0, 1.6},
};

double damperForce(const AxleParams& a, double v) {
    // v > 0 compression (bump), v < 0 extension (rebound); digressive with a knee.
    const double av = std::abs(v);
    const double slow = v > 0 ? a.bumpSlow : a.reboundSlow;
    const double fast = v > 0 ? a.bumpFast : a.reboundFast;
    double f = av < a.damperKnee ? slow * av : slow * a.damperKnee + fast * (av - a.damperKnee);
    return v > 0 ? f : -f;
}
}  // namespace

void Car::init(const CarParams& params, const TrackData* track) {
    p_ = params;
    track_ = track;
    tires_[0] = TireModel(p_.front.tire);
    tires_[1] = TireModel(p_.rear.tire);
    drivetrain_.init(p_);
    brakeBias = p_.brakeBias;

    massUnsprung_ = 2.0 * (p_.front.unsprungMass + p_.rear.unsprungMass);
    massSprung_ = p_.mass - massUnsprung_;

    for (int i = 0; i < 4; ++i) {
        Wheel& w = wheels_[i];
        w = Wheel{};
        w.axle = i / 2;
        w.side = (i % 2 == 0) ? -1 : 1;
        const AxleParams& ax = w.axle == 0 ? p_.front : p_.rear;
        const double axleLoad = p_.mass * kG * (w.axle == 0 ? p_.frontWeight : 1.0 - p_.frontWeight);
        const double cornerLoad = 0.5 * axleLoad;
        const double sprungCorner = cornerLoad - ax.unsprungMass * kG;
        const double tyreDeflection = cornerLoad / ax.tire.verticalStiffness;
        const double wheelCentreY = -p_.cgHeight + ax.tire.radius - tyreDeflection;
        w.fullDroopLength = 0.30;
        const double hpY = wheelCentreY + (w.fullDroopLength - ax.staticTravel);
        const double z = w.axle == 0 ? p_.frontAxleZ() : p_.rearAxleZ();
        w.hardpoint = glm::dvec3(w.side * ax.track * 0.5, hpY, z);
        w.springPreload = sprungCorner / ax.springRate - ax.staticTravel;
    }
    bodyHints_.fill(-1);
}

void Car::resetTo(float s, float lateral) {
    glm::vec3 c = track_->surfacePoint(s, lateral);
    glm::dvec3 fwd = glm::normalize(toD(track_->tangentAt(s)));
    glm::dvec3 rightH = toD(track_->rightAt(s));
    double bank = track_->bankAt(s);
    glm::dvec3 rgt = glm::normalize(rightH * std::cos(bank) - glm::dvec3(0, 1, 0) * std::sin(bank));
    glm::dvec3 up = glm::normalize(glm::cross(rgt, fwd));
    rgt = glm::normalize(glm::cross(fwd, up));
    glm::dmat3 R(rgt, up, -fwd);
    rot_ = glm::normalize(glm::quat_cast(R));
    pos_ = toD(c) + up * static_cast<double>(p_.cgHeight + 0.01);
    vel_ = glm::dvec3(0.0);
    angVel_ = glm::dvec3(0.0);
    for (auto& w : wheels_) {
        const AxleParams& ax = w.axle == 0 ? p_.front : p_.rear;
        w.travel = ax.staticTravel;
        w.wheelVel = 0.0;
        w.omega = 0.0;
        w.slipX = w.slipY = 0.0;
        w.absFactor = 1.0;
        w.trackHint = -1;
        w.twist = 0.0;
        w.kickLowpass[0] = w.kickLowpass[1] = 0.0;
    }
    prevSteer_ = steer_;
    columnTorque_ = 0.0;
    bodyHints_.fill(-1);
    cgHint_ = -1;
    loc_ = track_->locate(glm::vec3(pos_), -1);
    cgHint_ = loc_.seg;
    drivetrain_.reset(0.0);
    tcFactor_ = 1.0;
    needsReset_ = false;
    impact_ = scrape_ = 0.0;
}

double Car::steeringWheelDeg() const {
    double frac = p_.front.maxSteer > 0 ? steer_ / p_.front.maxSteer : 0.0;
    return frac * p_.steeringWheelLock * 0.5;
}

void Car::computeAero(const glm::dmat3& R, glm::dvec3& F, glm::dvec3& T) {
    const glm::dvec3 up = R[1];
    const double v2 = glm::dot(vel_, vel_);
    const double v = std::sqrt(v2);
    const glm::dvec3 fwd = -R[2];
    const double u = std::max(0.0, glm::dot(vel_, fwd));
    // drag at the centre of pressure (slightly above the CG -> nose-up moment)
    if (v > 0.1) {
        glm::dvec3 drag = -0.5 * kRho * p_.aero.dragArea * v * vel_;
        glm::dvec3 at = up * static_cast<double>(p_.aero.centrePressureHeight);
        F += drag;
        T += glm::cross(at, drag);
    }
    // downforce, reduced when the car is sideways (yaw sensitivity)
    const double lift = 0.5 * kRho * p_.aero.liftArea * u * u;
    downforce_ = lift;
    const double front = lift * p_.aero.frontBalance;
    const double rear = lift - front;
    glm::dvec3 pf = R * glm::dvec3(0.0, 0.0, p_.frontAxleZ());
    glm::dvec3 pr = R * glm::dvec3(0.0, 0.0, p_.rearAxleZ());
    F += -up * (front + rear);
    T += glm::cross(pf, -up * front) + glm::cross(pr, -up * rear);
}

void Car::collide(const glm::dmat3& R, glm::dvec3& F, glm::dvec3& T, double dt) {
    // ---- armco barriers
    double impactNow = 0.0;
    int k = 0;
    for (const auto& bp : kBarrierPoints) {
        glm::dvec3 r = R * bp;
        glm::dvec3 P = pos_ + r;
        TrackLocation loc = track_->locate(glm::vec3(P), bodyHints_[k]);
        bodyHints_[k] = loc.seg;
        ++k;
        int side = loc.lateral >= 0 ? 1 : -1;
        double b = track_->barrierAt(loc.s, side);
        double pen = std::abs(loc.lateral) - b;
        if (pen <= 0.0) continue;
        glm::dvec3 n = -toD(track_->rightAt(loc.s)) * static_cast<double>(side);  // points back to the track
        glm::dvec3 vP = vel_ + glm::cross(angVel_, r);
        double vn = glm::dot(vP, n);
        double fn = std::max(0.0, 1.6e6 * pen - 4.5e4 * vn);
        glm::dvec3 vt = vP - n * vn;
        double vtLen = glm::length(vt);
        glm::dvec3 ft = vtLen > 1e-4 ? -vt / std::max(vtLen, 0.4) * (0.32 * fn) : glm::dvec3(0.0);
        glm::dvec3 f = n * fn + ft;
        F += f;
        T += glm::cross(r, f);
        impactNow = std::max(impactNow, -vn);
    }
    impact_ = std::max(impact_ * std::exp(-dt * 3.0), impactNow);

    // ---- floor / splitter scraping
    double scrapeNow = 0.0;
    for (const auto& fp0 : kFloorPoints) {
        glm::dvec3 fp = fp0;
        fp.y = -p_.cgHeight + 0.055;
        glm::dvec3 r = R * fp;
        glm::dvec3 P = pos_ + r;
        GroundInfo g = track_->ground(glm::vec3(P), bodyHints_[k]);
        ++k;
        double pen = g.height - P.y;
        if (pen <= 0.0) continue;
        glm::dvec3 n = toD(g.normal);
        glm::dvec3 vP = vel_ + glm::cross(angVel_, r);
        double vn = glm::dot(vP, n);
        double fn = std::max(0.0, 3.5e5 * pen - 1.2e4 * vn);
        glm::dvec3 vt = vP - n * vn;
        double vtLen = glm::length(vt);
        glm::dvec3 ft = vtLen > 1e-4 ? -vt / std::max(vtLen, 0.5) * (0.45 * fn) : glm::dvec3(0.0);
        F += n * fn + ft;
        T += glm::cross(r, n * fn + ft);
        scrapeNow = std::max(scrapeNow, std::min(1.0, fn / 8000.0) * std::min(1.0, vtLen / 5.0));
    }
    scrape_ = std::max(scrape_ * std::exp(-dt * 8.0), scrapeNow);
}

void Car::step(double dt, const CarInput& in) {
    const glm::dmat3 R = rotation();
    const glm::dvec3 up = R[1];
    const glm::dvec3 gravity(0.0, -kG, 0.0);

    // ------------------------------------------------------------------ driver inputs
    throttle_ = std::clamp(static_cast<double>(in.throttle), 0.0, 1.0);
    brake_ = std::clamp(static_cast<double>(in.brake), 0.0, 1.0);
    clutch_ = std::clamp(static_cast<double>(in.clutch), 0.0, 1.0);
    if (in.shiftUp && !prevShiftUp_) drivetrain_.requestShift(+1);
    if (in.shiftDown && !prevShiftDown_) drivetrain_.requestShift(-1);
    prevShiftUp_ = in.shiftUp;
    prevShiftDown_ = in.shiftDown;
    steer_ = std::clamp(static_cast<double>(in.steer), -1.0, 1.0) * p_.front.maxSteer;

    glm::dvec3 F = gravity * massSprung_;
    glm::dvec3 T(0.0);
    computeAero(R, F, T);

    const double speed = glm::length(vel_);

    // ------------------------------------------------------------------ wheels: geometry & tyre forces
    struct Scratch {
        glm::dvec3 H, C, fg, rg, n, patch;
        double vLong = 0, vLat = 0, radius = 0, fxSlip = 0, fySlip = 0, cDamp = 0, peak = 0;
        double extTorque = 0;
        double mz = 0, grip = 1, nLong = 0, nLat = 0;  // steering feel
    } sc[4];

    for (int i = 0; i < 4; ++i) {
        Wheel& w = wheels_[i];
        Scratch& s = sc[i];
        const AxleParams& ax = w.axle == 0 ? p_.front : p_.rear;
        const TireParams& tp = ax.tire;

        s.H = pos_ + R * w.hardpoint;
        s.C = s.H - up * (w.fullDroopLength - w.travel);
        w.centre = s.C;

        double steer = (w.axle == 0 ? steer_ : 0.0) - w.side * ax.toe;
        w.steerAngle = steer;

        GroundInfo g = track_->ground(glm::vec3(s.C), w.trackHint);
        double gh = g.height;
        const SurfaceInfo& surf = surfaceInfo(g.surface);
        if (surf.bumpAmplitude > 0.0f) {
            gh += surf.bumpAmplitude * fbm2(float(s.C.x) / surf.bumpWavelength, float(s.C.z) / surf.bumpWavelength, 2, 2.0f, 0.5f, 7);
        } else {
            // Nordschleife tarmac is famously uneven: long and short wavelength undulations.
            gh += 0.0045 * perlin2(float(s.C.x) * 0.16f, float(s.C.z) * 0.16f, 3) +
                  0.0018 * perlin2(float(s.C.x) * 0.65f, float(s.C.z) * 0.65f, 5);
        }
        s.n = toD(g.normal);
        w.surface = g.surface;
        w.contactNormal = s.n;

        glm::dvec3 groundPt(s.C.x, gh, s.C.z);
        double dist = glm::dot(s.C - groundPt, s.n);
        double pen = tp.radius - dist;

        glm::dvec3 fwLocal(std::sin(steer), 0.0, -std::cos(steer));
        glm::dvec3 fw = R * fwLocal;
        s.fg = glm::normalize(fw - s.n * glm::dot(fw, s.n));
        s.rg = glm::normalize(glm::cross(s.fg, s.n));
        s.patch = s.C - s.n * dist;
        // ground normal seen from the wheel (tilts on kerbs and bump faces)
        s.nLong = glm::dot(s.n, fw);
        s.nLat = glm::dot(s.n, R * glm::dvec3(std::cos(steer), 0.0, std::sin(steer)));
        s.grip = surf.grip;
        w.contactPoint = s.patch;

        glm::dvec3 vPatch = vel_ + glm::cross(angVel_, s.patch - pos_);
        s.vLong = glm::dot(vPatch, s.fg);
        s.vLat = glm::dot(vPatch, s.rg);

        // tyre vertical force (spring + damper on deflection)
        double vh = glm::dot(vel_ + glm::cross(angVel_, s.H - pos_), up);
        double tDot = w.wheelVel - vh;
        glm::dvec3 vC = vel_ + glm::cross(angVel_, s.C - pos_) + up * tDot;
        double penRate = -glm::dot(vC, s.n);
        double fz = 0.0;
        if (pen > 0.0) fz = std::max(0.0, tp.verticalStiffness * pen + tp.verticalDamping * penRate);
        w.contact = fz > 0.0;
        w.load = fz;

        // transient slip (relaxation length) - implicit update, stable down to standstill
        s.radius = tp.radius - std::max(0.0, pen) * 0.35;
        const double absV = std::abs(s.vLong);
        w.slipX = (w.slipX + dt * (w.omega * s.radius - s.vLong) / tp.relaxLong) / (1.0 + dt * absV / tp.relaxLong);
        w.slipY = (w.slipY + dt * s.vLat / tp.relaxLat) / (1.0 + dt * absV / tp.relaxLat);
        w.slipX = std::clamp(w.slipX, -1.2, 4.0);
        w.slipY = std::clamp(w.slipY, -4.0, 4.0);
        if (!w.contact) {
            w.slipX *= std::exp(-dt * 40.0);
            w.slipY *= std::exp(-dt * 40.0);
        }

        // camber relative to the road
        glm::dvec3 wupLocal(w.side * std::sin(ax.camber + ax.camberGain * (w.travel - ax.staticTravel)),
                            std::cos(ax.camber), 0.0);
        glm::dvec3 wup = R * wupLocal;
        w.camber = w.side * std::asin(std::clamp(glm::dot(wup, s.rg), -1.0, 1.0));

        TireOutput to = tires_[w.axle].compute(fz, w.slipX, w.slipY, surf.grip, w.camber);
        // low-speed damping keeps the bristle model from oscillating at standstill
        const double wLow = std::clamp(1.0 - absV / 4.0, 0.0, 1.0);
        s.cDamp = 3500.0 * wLow * (w.contact ? 1.0 : 0.0);
        s.fxSlip = to.fx;
        s.fySlip = to.fy - 7000.0 * wLow * s.vLat * (w.contact ? 1.0 : 0.0);
        s.peak = to.peakForce;
        s.mz = to.mz;
        w.combinedSlip = to.combinedSlip;

        double rr = (tp.rollingResistance + surf.rollingResistance) * fz;
        double rrTorque = -std::copysign(std::min(rr * s.radius, std::abs(w.omega) * 50.0), w.omega);
        s.extTorque = -s.fxSlip * s.radius + rrTorque;

        // surface drag (gravel etc.) acts on the body directly
        if (surf.drag > 0.0f && w.contact) {
            glm::dvec3 vh2 = vPatch - s.n * glm::dot(vPatch, s.n);
            F += -vh2 * (surf.drag * fz * 0.02);
        }
    }

    // ------------------------------------------------------------------ ABS / TC
    // Both target a slip ratio that shrinks with lateral acceleration, like motorsport ABS/TC maps
    // ("cornering" control): in a corner the tyres need part of their grip for cornering.
    const double latLoad = std::clamp(std::abs(accelLocal_.x) / (1.5 * kG), 0.0, 1.0);
    absActive_ = false;
    {
        const double peakF = p_.front.tire.peakSlipRatio, peakR = p_.rear.tire.peakSlipRatio;
        const bool on = absEnabled && brake_ > 0.05 && speed > 2.5;
        auto modulate = [&](double& factor, double slip, double target) {
            if (slip < -target) factor -= dt * 32.0;
            else factor += dt * 11.0;
            factor = std::clamp(factor, 0.12, 1.0);
        };
        // fronts individually, close to the peak (most of the braking)
        for (int i = 0; i < 2; ++i) {
            Wheel& w = wheels_[i];
            if (on && w.contact) modulate(w.absFactor, w.slipX, peakF * (1.10 - 0.35 * latLoad));
            else w.absFactor = 1.0;
        }
        // rear axle select-low: both rear brakes follow the wheel that slips most, with a lower
        // target, so an unloaded inside rear can't take the rear end's lateral grip away
        Wheel& rl = wheels_[2];
        Wheel& rr = wheels_[3];
        if (on && (rl.contact || rr.contact)) {
            double factor = std::min(rl.absFactor, rr.absFactor);
            modulate(factor, std::min(rl.slipX, rr.slipX), peakR * (0.85 - 0.35 * latLoad));
            rl.absFactor = rr.absFactor = factor;
        } else {
            rl.absFactor = rr.absFactor = 1.0;
        }
        for (const auto& w : wheels_)
            if (w.absFactor < 0.97) absActive_ = true;
    }
    // traction control: driven-wheel slip target per level (1 loosest .. 6 tightest)
    static const double kTcSlip[] = {1e9, 0.20, 0.16, 0.13, 0.11, 0.09, 0.075};
    const int tcIdx = std::clamp(tcLevel, 0, 6);
    const double tcTarget = kTcSlip[tcIdx] * (1.0 - 0.45 * latLoad);
    const double driveSlip = std::max(wheels_[2].slipX, wheels_[3].slipX);
    if (tcIdx > 0 && throttle_ > 0.05 && speed > 1.0 && driveSlip > tcTarget) {
        tcFactor_ -= dt * (14.0 + 160.0 * (driveSlip - tcTarget));
    } else {
        // give torque back quickly while the tyres are well inside the target, gently near it
        tcFactor_ += dt * (3.0 + 60.0 * std::clamp(tcTarget * 0.85 - driveSlip, 0.0, 0.1));
    }
    tcFactor_ = std::clamp(tcFactor_, 0.05, 1.0);
    tcActive_ = tcFactor_ < 0.97;
    drivetrain_.tcFactor = tcFactor_;

    for (Wheel& w : wheels_) {
        const double share = w.axle == 0 ? brakeBias : 1.0 - brakeBias;
        w.brakeTorque = brake_ * p_.maxBrakeTorque * share * 0.5 * w.absFactor;
    }

    // ------------------------------------------------------------------ drivetrain
    Drivetrain::WheelSide dl{wheels_[2].omega, p_.rear.tire.inertia, sc[2].extTorque, wheels_[2].brakeTorque};
    Drivetrain::WheelSide dr{wheels_[3].omega, p_.rear.tire.inertia, sc[3].extTorque, wheels_[3].brakeTorque};
    double tqL = 0.0, tqR = 0.0;
    drivetrain_.step(dt, throttle_, clutch_, dl, dr, tqL, tqR);
    wheels_[2].driveTorque = tqL;
    wheels_[3].driveTorque = tqR;
    wheels_[0].driveTorque = wheels_[1].driveTorque = 0.0;

    // ------------------------------------------------------------------ wheel spin integration
    for (int i = 0; i < 4; ++i) {
        Wheel& w = wheels_[i];
        Scratch& s = sc[i];
        const TireParams& tp = (w.axle == 0 ? p_.front : p_.rear).tire;
        const double I = tp.inertia;
        const double c = s.cDamp;
        const double denom = I + dt * c * s.radius * s.radius;
        double omegaFree = (I * w.omega + dt * (w.driveTorque + s.extTorque + c * s.radius * s.vLong)) / denom;
        const double dBrake = dt * w.brakeTorque / denom;
        if (std::abs(omegaFree) <= dBrake) w.omega = 0.0;
        else w.omega = omegaFree - std::copysign(dBrake, omegaFree);
        w.spinAngle = std::fmod(w.spinAngle + w.omega * dt, 2.0 * 3.14159265358979323846);

        double fx = s.fxSlip + c * (w.omega * s.radius - s.vLong);
        double fy = s.fySlip;
        double mag = std::sqrt(fx * fx + fy * fy);
        double limit = s.peak * 1.08 + 1.0;
        if (mag > limit) {
            fx *= limit / mag;
            fy *= limit / mag;
            s.mz *= limit / mag;
        }
        w.fx = fx;
        w.fy = fy;
        w.slideSpeed = w.contact ? std::sqrt(std::pow(w.omega * s.radius - s.vLong, 2) + s.vLat * s.vLat) : 0.0;

        // brake temperature (glowing discs)
        w.brakeTemp += (w.brakeTorque * std::abs(w.omega)) * dt / 9000.0;
        w.brakeTemp -= (w.brakeTemp - 25.0) * (0.015 + 0.0025 * speed) * dt;
    }
    drivetrain_.postWheelUpdate(wheels_[2].omega, wheels_[3].omega);

    // ------------------------------------------------------------------ steering torque (force feedback)
    // Torque about each front kingpin axis, through the rack ratio and power assist to the column.
    {
        const SteeringParams& sp = p_.steering;
        const double dSteer = steer_ - prevSteer_;
        prevSteer_ = steer_;
        const double kickAlpha = 1.0 - std::exp(-dt * 2.0 * 3.14159265358979323846 * 2.5);
        double kingpinSum = 0.0;
        for (int i = 0; i < 2; ++i) {
            Wheel& w = wheels_[i];
            const Scratch& s = sc[i];
            // fast part of the contact normal tilt: kerb ridges and bump faces push the patch
            w.kickLowpass[0] += (s.nLong - w.kickLowpass[0]) * kickAlpha;
            w.kickLowpass[1] += (s.nLat - w.kickLowpass[1]) * kickAlpha;
            if (!w.contact) {
                w.twist = 0.0;
                w.kingpinTorque = 0.0;
                continue;
            }
            const TireParams& tp = p_.front.tire;
            // contact patch torsion: turning the wheel winds the patch up until it slides (parking
            // effort), rolling unwinds it within a fraction of a tyre revolution
            w.twist = (w.twist + dSteer) / (1.0 + dt * std::abs(s.vLong) / sp.twistRelaxation);
            const double twistLimit = s.grip * tp.mu * w.load * sp.boreLever / sp.twistStiffness;
            w.twist = std::clamp(w.twist, -twistLimit, twistLimit);
            double t = -sp.twistStiffness * w.twist;
            // aligning moment from the pneumatic trail (fades past the peak slip angle) + caster trail
            t += s.mz - w.fy * sp.casterTrail;
            // left/right differences in longitudinal force act through the scrub radius
            t += w.side * (-w.fx) * sp.scrubRadius;
            // kingpin inclination and caster lift the car as the wheels turn: centring
            t -= w.load * sp.jackingLever * std::sin(steer_);
            // kerb / bump impacts
            const double fxKick = w.load * (s.nLong - w.kickLowpass[0]);
            const double fyKick = w.load * (s.nLat - w.kickLowpass[1]);
            t += w.side * (-fxKick) * sp.scrubRadius - fyKick * sp.casterTrail;
            w.kingpinTorque = t;
            kingpinSum += t;
        }
        columnTorque_ = kingpinSum / p_.steeringRatio() * (1.0 - sp.powerAssist);
    }

    // ------------------------------------------------------------------ suspension + forces on body
    double travelAxle[2][2] = {{wheels_[0].travel, wheels_[1].travel}, {wheels_[2].travel, wheels_[3].travel}};
    for (int i = 0; i < 4; ++i) {
        Wheel& w = wheels_[i];
        Scratch& s = sc[i];
        const AxleParams& ax = w.axle == 0 ? p_.front : p_.rear;

        // tyre ground-plane forces + the part of the normal force not carried by the spring
        glm::dvec3 fh = s.fg * w.fx + s.rg * w.fy;
        glm::dvec3 fn = s.n * w.load;
        glm::dvec3 fnPerp = fn - up * glm::dot(fn, up);
        glm::dvec3 apply = s.patch + s.n * static_cast<double>(ax.rollCentre);
        F += fh + fnPerp;
        T += glm::cross(apply - pos_, fh + fnPerp);

        // spring / damper / bump stop / anti-roll bar
        const double vh = glm::dot(vel_ + glm::cross(angVel_, s.H - pos_), up);
        const double tDot = w.wheelVel - vh;
        double fs = std::max(0.0, ax.springRate * (w.travel + w.springPreload));
        double fd = damperForce(ax, tDot);
        double bumpStart = ax.travel - ax.bumpStopGap;
        double fb = w.travel > bumpStart ? ax.bumpStopRate * (w.travel - bumpStart) : 0.0;
        double other = travelAxle[w.axle][w.side < 0 ? 1 : 0];
        double farb = ax.antiRoll * (w.travel - other);
        double fsusp = fs + fd + fb + farb;
        w.suspensionForce = fsusp;
        F += up * fsusp;
        T += glm::cross(s.H - pos_, up * fsusp);

        // unsprung mass along the suspension axis
        double fWheel = glm::dot(fn, up) - fsusp + ax.unsprungMass * glm::dot(gravity, up);
        w.wheelVel += dt * fWheel / ax.unsprungMass;
        w.travel += (w.wheelVel - vh) * dt;
        if (w.travel < 0.0) {
            w.travel = 0.0;
            w.wheelVel = vh;
        } else if (w.travel > ax.travel) {
            // hard stop: transfer the wheel's relative momentum to the body
            double J = ax.unsprungMass * (w.wheelVel - vh);
            F += up * (J / dt);
            T += glm::cross(s.H - pos_, up * (J / dt));
            w.travel = ax.travel;
            w.wheelVel = vh;
        }
    }

    collide(R, F, T, dt);

    // ------------------------------------------------------------------ integrate the body
    // Vertical (suspension axis) dynamics use the sprung mass; in-plane dynamics move the whole car.
    const double massTotal = massSprung_ + massUnsprung_;
    glm::dvec3 Fup = up * glm::dot(F, up);
    glm::dvec3 Fplane = F - Fup;
    glm::dvec3 gPlane = gravity - up * glm::dot(gravity, up);
    glm::dvec3 acc = Fup / massSprung_ + (Fplane + gPlane * massUnsprung_) / massTotal;
    vel_ += acc * dt;
    pos_ += vel_ * dt;

    const glm::dvec3 Ib(p_.inertia);
    glm::dvec3 wLocal = glm::transpose(R) * angVel_;
    glm::dvec3 tLocal = glm::transpose(R) * T;
    glm::dvec3 Lw = Ib * wLocal;
    glm::dvec3 alpha = (tLocal - glm::cross(wLocal, Lw)) / Ib;
    wLocal += alpha * dt;
    angVel_ = R * wLocal;
    glm::dquat dq(0.0, angVel_.x, angVel_.y, angVel_.z);
    rot_ = glm::normalize(rot_ + (dq * rot_) * (0.5 * dt));

    accelLocal_ = glm::transpose(R) * (acc - gravity);

    // ------------------------------------------------------------------ track position & sanity
    loc_ = track_->locate(glm::vec3(pos_), cgHint_);
    cgHint_ = loc_.seg;
    double side = loc_.lateral >= 0 ? 1.0 : -1.0;
    double barrier = track_->barrierAt(loc_.s, static_cast<int>(side));
    double ground = track_->terrainHeight(float(pos_.x), float(pos_.z));
    if (std::abs(loc_.lateral) > barrier + 25.0 || pos_.y < ground - 4.0 || !std::isfinite(pos_.x)) {
        needsReset_ = true;
    }
}

}  // namespace apex
