#pragma once
#include <string>
#include <vector>

namespace apex {

class LapTimer {
public:
    void init(float trackLength, float resolution = 10.0f);
    void reset();
    // Call every physics step with the car's distance along the track.
    void update(double simTime, float s);
    void invalidate() { valid_ = false; }

    bool timing() const { return started_; }
    double currentLap(double simTime) const { return started_ ? simTime - lapStart_ : 0.0; }
    double lastLap() const { return last_; }
    double bestLap() const { return best_; }
    int laps() const { return laps_; }
    bool currentValid() const { return valid_; }
    bool lastValid() const { return lastValid_; }
    bool hasDelta() const { return !bestTrace_.empty() && started_; }
    double delta(double simTime, float s) const;  // + = slower than best
    int lapCompletedEvent() const { return lapEvent_; }

    static std::string format(double seconds, bool sign = false);

private:
    float length_ = 1.0f;
    float res_ = 10.0f;
    bool started_ = false;
    bool valid_ = true;
    bool lastValid_ = true;
    double lapStart_ = 0.0;
    double last_ = 0.0;
    double best_ = 0.0;
    int laps_ = 0;
    int lapEvent_ = 0;
    float prevS_ = -1.0f;
    std::vector<float> trace_;      // time at each resolution step of the current lap
    std::vector<float> bestTrace_;
};

}  // namespace apex
