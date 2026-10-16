#pragma once
#include "car/Car.h"
#include "world/TrackData.h"

namespace apex {

// Racing-line follower: pure-pursuit steering with yaw-rate damping and a speed controller that
// tracks the track's precomputed speed profile. Used for the menu demo and for physics tests.
class AIDriver {
public:
    void init(const TrackData* track, float pace = 0.96f) {
        track_ = track;
        pace_ = pace;
    }
    CarInput update(const Car& car, double dt);
    float pace() const { return pace_; }
    void setPace(float p) { pace_ = p; }

private:
    const TrackData* track_ = nullptr;
    float pace_ = 0.96f;
    double steerFiltered_ = 0.0;
    double throttle_ = 0.0, brake_ = 0.0;
};

}  // namespace apex
