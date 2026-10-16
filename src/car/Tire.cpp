#include "car/Tire.h"

#include <algorithm>
#include <cmath>

namespace apex {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

double TireModel::peakSlipAngleAt(double fz) const {
    double load = fz / p_.nominalLoad;
    return p_.peakSlipAngle * (0.85 + 0.15 * load);
}

TireOutput TireModel::compute(double fz, double slipRatio, double tanSlipAngle, double gripScale,
                              double camber) const {
    TireOutput out;
    if (fz <= 0.0) return out;

    const double load = fz / p_.nominalLoad;
    double mu = p_.mu * (1.0 - p_.loadSensitivity * (load - 1.0));
    mu = std::clamp(mu, p_.mu * 0.55, p_.mu * 1.25);
    const double dc = camber - p_.camberOptimum;
    mu *= std::max(0.6, 1.0 - p_.camberSensitivity * dc * dc);
    mu *= gripScale;

    const double peakAlpha = std::tan(peakSlipAngleAt(fz));
    const double peakKappa = p_.peakSlipRatio * (0.9 + 0.1 * load);
    const double nx = slipRatio / peakKappa;
    const double ny = tanSlipAngle / peakAlpha;
    const double rho = std::sqrt(nx * nx + ny * ny);

    const double muX = mu * p_.muLongFactor, muY = mu;
    double ux = 1.0, uy = 0.0;
    if (rho > 1e-9) {
        ux = nx / rho;
        uy = ny / rho;
    }
    // friction ellipse in the direction of the slip vector
    const double muDir = 1.0 / std::sqrt((ux / muX) * (ux / muX) + (uy / muY) * (uy / muY));
    const double C = p_.shapeLong * ux * ux + p_.shapeLat * uy * uy;
    const double B = std::tan(kPi / (2.0 * C));  // places the MF peak at rho = 1
    const double D = muDir * fz;

    double fOverRho;  // F(rho)/rho, well defined at rho -> 0
    if (rho < 1e-6) {
        fOverRho = D * B * C;
    } else {
        fOverRho = D * std::sin(C * std::atan(B * rho)) / rho;
    }
    out.fx = fOverRho * nx;
    out.fy = -fOverRho * ny;
    out.combinedSlip = rho;
    out.peakForce = D;

    // Pneumatic trail collapses as the tyre approaches the limit.
    const double trail0 = 0.045 * std::sqrt(std::max(load, 0.1));
    const double trail = trail0 * std::max(0.0, 1.0 - std::abs(ny) * 0.85);
    out.mz = -out.fy * trail;
    return out;
}

}  // namespace apex
