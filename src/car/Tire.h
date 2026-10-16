#pragma once
#include "car/CarParams.h"

namespace apex {

struct TireOutput {
    double fx = 0.0;          // longitudinal force (N), + forward
    double fy = 0.0;          // lateral force (N), + to the right of the wheel
    double mz = 0.0;          // self-aligning torque (Nm)
    double combinedSlip = 0;  // normalised slip (1 = peak)
    double peakForce = 0;     // available force at this load (N)
};

// Combined-slip Magic Formula tyre: a single normalised slip vector drives one MF curve
// (friction-ellipse direction), with load sensitivity, camber window and slip-dependent shape.
class TireModel {
public:
    TireModel() = default;
    explicit TireModel(const TireParams& p) : p_(p) {}
    const TireParams& params() const { return p_; }

    // slipRatio: (wR - v)/|v| (transient), tanSlipAngle: v_lat/|v_long| (transient)
    TireOutput compute(double fz, double slipRatio, double tanSlipAngle, double gripScale, double camber) const;

    double peakSlipAngleAt(double fz) const;

private:
    TireParams p_;
};

}  // namespace apex
