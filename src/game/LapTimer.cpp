#include "game/LapTimer.h"

#include <cmath>
#include <cstdio>

namespace apex {

void LapTimer::init(float trackLength, float resolution) {
    length_ = trackLength;
    res_ = resolution;
    reset();
    best_ = 0.0;
    bestTrace_.clear();
}

void LapTimer::reset() {
    started_ = false;
    valid_ = true;
    lapStart_ = 0.0;
    prevS_ = -1.0f;
    trace_.assign(static_cast<size_t>(length_ / res_) + 2, -1.0f);
}

void LapTimer::update(double simTime, float s) {
    if (prevS_ >= 0.0f) {
        bool crossed = prevS_ > length_ - 300.0f && s < 300.0f;
        if (crossed) {
            // interpolate the exact crossing moment would need dt; the 1 kHz step makes it moot
            if (started_) {
                double lap = simTime - lapStart_;
                last_ = lap;
                lastValid_ = valid_;
                ++laps_;
                ++lapEvent_;
                if (valid_ && (best_ <= 0.0 || lap < best_)) {
                    best_ = lap;
                    bestTrace_ = trace_;
                }
            }
            started_ = true;
            valid_ = true;
            lapStart_ = simTime;
            trace_.assign(trace_.size(), -1.0f);
        }
    }
    prevS_ = s;
    if (started_) {
        size_t idx = static_cast<size_t>(s / res_);
        if (idx < trace_.size() && trace_[idx] < 0.0f) trace_[idx] = static_cast<float>(simTime - lapStart_);
    }
}

double LapTimer::delta(double simTime, float s) const {
    if (!hasDelta()) return 0.0;
    float f = s / res_;
    size_t i = static_cast<size_t>(f);
    if (i + 1 >= bestTrace_.size() || bestTrace_[i] < 0.0f || bestTrace_[i + 1] < 0.0f) return 0.0;
    double t = f - i;
    double bestAt = bestTrace_[i] + (bestTrace_[i + 1] - bestTrace_[i]) * t;
    return currentLap(simTime) - bestAt;
}

std::string LapTimer::format(double seconds, bool sign) {
    char buf[32];
    const char* sg = "";
    if (sign) sg = seconds < 0 ? "-" : "+";
    double a = std::abs(seconds);
    int m = static_cast<int>(a / 60.0);
    double sec = a - m * 60.0;
    if (m > 0) snprintf(buf, sizeof(buf), "%s%d:%06.3f", sg, m, sec);
    else snprintf(buf, sizeof(buf), "%s%.3f", sg, sec);
    return buf;
}

}  // namespace apex
