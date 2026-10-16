#pragma once
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

namespace apex {

struct TireParams {
    float radius = 0.34f;            // loaded-free outer radius (m)
    float width = 0.30f;
    float mu = 1.75f;                // peak lateral friction at nominal load
    float muLongFactor = 1.04f;      // longitudinal peak relative to lateral
    float nominalLoad = 4000.0f;     // N
    float loadSensitivity = 0.13f;   // fractional mu drop per unit of (Fz/Fz0 - 1)
    float peakSlipAngle = 0.115f;    // rad at nominal load
    float peakSlipRatio = 0.095f;
    float shapeLat = 1.35f;          // Magic Formula C (lateral)
    float shapeLong = 1.55f;         // Magic Formula C (longitudinal)
    float relaxLat = 0.32f;          // relaxation lengths (m)
    float relaxLong = 0.14f;
    float verticalStiffness = 310000.0f;  // N/m
    float verticalDamping = 650.0f;       // N s/m
    float rollingResistance = 0.011f;
    float camberOptimum = -0.035f;   // rad (relative to the road)
    float camberSensitivity = 6.0f;  // grip loss per rad^2 away from optimum
    float inertia = 1.25f;           // wheel + tyre + brake disc, kg m^2
};

struct AxleParams {
    float track = 1.64f;             // tyre centre to tyre centre (m)
    float springRate = 160000.0f;    // wheel rate (N/m)
    float bumpSlow = 6500.0f, reboundSlow = 11000.0f;  // N s/m (wheel)
    float bumpFast = 2800.0f, reboundFast = 5500.0f;
    float damperKnee = 0.08f;        // m/s
    float antiRoll = 90000.0f;       // N/m of travel difference (wheel)
    float travel = 0.085f;           // total wheel travel (m)
    float staticTravel = 0.045f;     // travel used at static ride height
    float bumpStopRate = 900000.0f;
    float bumpStopGap = 0.012f;      // engages this close to full bump
    float rollCentre = 0.05f;        // height above ground where tyre forces act on the body
    float camber = -0.06f;           // static camber (rad, negative = top inward)
    float camberGain = -0.6f;        // rad of camber per metre of compression
    float toe = 0.0f;                // rad, positive = toe-in
    float unsprungMass = 46.0f;
    float maxSteer = 0.0f;           // wheel angle at full lock (rad), 0 for non-steered
    float brakeShare = 0.5f;         // filled from car brake bias
    TireParams tire;
};

struct EngineParams {
    std::vector<std::pair<float, float>> torqueCurve;  // rpm, Nm at full throttle (at the flywheel)
    float idleRpm = 1900.0f;
    float limiterRpm = 8000.0f;
    float maxRpm = 8600.0f;
    float inertia = 0.17f;           // kg m^2 (crank + flywheel + clutch)
    float frictionTorque = 18.0f;    // Nm at 0 rpm (closed throttle)
    float frictionPerRpm = 0.0068f;  // Nm per rpm (engine braking, GT3 engine-brake map)
    int cylinders = 8;
};

struct GearboxParams {
    std::vector<float> ratios = {3.10f, 2.30f, 1.84f, 1.52f, 1.30f, 1.14f};
    float reverse = 3.0f;
    float finalDrive = 3.36f;
    float efficiency = 0.93f;
    float upshiftTime = 0.055f;
    float downshiftTime = 0.075f;
    float clutchMaxTorque = 950.0f;
};

struct DiffParams {
    float preload = 70.0f;   // Nm
    float powerRamp = 0.40f;  // fraction of input torque available as locking torque on power
    float coastRamp = 0.30f;
};

struct AeroParams {
    float dragArea = 1.25f;   // Cd * A (m^2)
    float liftArea = 3.0f;    // Cl * A (m^2), downforce
    float frontBalance = 0.39f;
    float centrePressureHeight = 0.12f;  // drag application point above CG (m)
};

struct CarParams {
    std::string name = "GT3";
    float mass = 1310.0f;               // with driver and some fuel (kg)
    float cgHeight = 0.43f;             // above ground at static ride height
    float wheelbase = 2.70f;
    float frontWeight = 0.465f;         // static front weight fraction
    glm::vec3 inertia{2150.0f, 2350.0f, 520.0f};  // pitch (x), yaw (y), roll (z) about CG (kg m^2)
    AxleParams front, rear;
    EngineParams engine;
    GearboxParams gearbox;
    DiffParams diff;
    AeroParams aero;
    float steeringWheelLock = 540.0f;   // lock-to-lock of the car's steering wheel (deg)
    float brakeBias = 0.575f;           // front fraction
    float maxBrakeTorque = 12400.0f;    // sum of all four corners at 100% pedal (Nm)
    glm::vec3 driverEye{-0.37f, 0.56f, 0.18f};  // relative to CG in body frame (x right, y up, z back)
    glm::vec3 bodyHalfExtents{1.0f, 0.62f, 2.3f};
    float bodyCentreZ = 0.12f;          // geometric body centre relative to CG along z (positive = behind)

    float frontAxleZ() const { return -wheelbase * (1.0f - frontWeight); }  // ahead of the CG
    float rearAxleZ() const { return wheelbase * frontWeight; }

    static CarParams gt3();
};

}  // namespace apex
