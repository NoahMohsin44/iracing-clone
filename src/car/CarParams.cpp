#include "car/CarParams.h"

#include <cmath>

namespace apex {

CarParams CarParams::gt3() {
    // A generic front-mid-engined GT3 car: ~550 hp naturally aspirated V8, 1310 kg with driver,
    // sequential 6-speed, clutch-pack LSD, ABS/TC, ~600 kg of downforce at 200 km/h.
    CarParams p;
    p.name = "GT3 V8";
    p.engine.torqueCurve = {
        {0.0f, 180.0f},    {1000.0f, 270.0f}, {2000.0f, 395.0f}, {3000.0f, 470.0f},
        {4000.0f, 535.0f}, {4750.0f, 572.0f}, {5500.0f, 585.0f}, {6250.0f, 575.0f},
        {7000.0f, 556.0f}, {7500.0f, 535.0f}, {8000.0f, 500.0f}, {8600.0f, 420.0f},
    };

    p.front.track = 1.66f;
    p.front.springRate = 175000.0f;
    p.front.antiRoll = 125000.0f;
    p.front.camber = -0.0611f;   // -3.5 deg
    p.front.toe = -0.0017f;      // slight toe-out
    p.front.rollCentre = 0.04f;
    p.front.unsprungMass = 44.0f;
    p.front.maxSteer = 0.384f;   // 22 deg at the wheel (270 deg of steering wheel, ratio ~12.3)
    p.front.tire.radius = 0.340f;
    p.front.tire.width = 0.30f;
    p.front.tire.inertia = 1.15f;

    p.rear.track = 1.62f;
    p.rear.springRate = 160000.0f;
    p.rear.antiRoll = 45000.0f;
    p.rear.camber = -0.0436f;    // -2.5 deg
    p.rear.toe = 0.0035f;        // toe-in for stability
    p.rear.rollCentre = 0.075f;
    p.rear.unsprungMass = 48.0f;
    p.rear.bumpSlow = 6000.0f;
    p.rear.reboundSlow = 10000.0f;
    p.rear.tire.radius = 0.355f;
    p.rear.tire.width = 0.31f;
    p.rear.tire.inertia = 1.40f;
    p.rear.tire.mu = 1.77f;
    // GT3 rears are the bigger tyre (31/71-18 vs 30/68-18): a larger, stiffer contact patch builds
    // cornering force at a smaller slip angle, which gives the car its mild, stable understeer.
    p.rear.tire.peakSlipAngle = 0.100f;
    return p;
}

}  // namespace apex
