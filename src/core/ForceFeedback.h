#pragma once
#include <memory>
#include <string>
#include <vector>

#include "core/FfbSignal.h"

struct GLFWwindow;

namespace apex {

// Which controller to drive, and how to read its steering axis (for the soft lock).
struct FfbDeviceHint {
    std::string guid;   // GLFW joystick GUID of the steering binding (empty: first FFB device)
    std::string name;
    int axis = -1;      // GLFW axis index of the steering binding
    float rest = 0.0f;  // calibration of that axis (centre and full-right values)
    float full = 1.0f;
    float deadzone = 0.0f;   // the same shaping Input applies, so the soft lock sits exactly
    float linearity = 1.0f;  // where the car's steering reaches full lock
};

struct FfbStatus {
    bool supported = false;   // platform backend exists
    bool device = false;      // a force feedback wheel is open
    bool acquired = false;    // we currently own it (window focused)
    std::string deviceName;
    float output = 0.0f;      // last sent force, -1..1
    float torqueNm = 0.0f;
    bool clipping = false;
    float clipFraction = 0.0f;
    float peakNm = 0.0f;
    bool live = false;
    int safetyTrips = 0;      // runaway guard trips (see FfbSignal::safetyTrips)
};

// Force feedback output for steering wheels. On Windows this drives a DirectInput constant
// force effect from a dedicated ~1 kHz thread; elsewhere it is a no-op.
class ForceFeedback {
public:
    ForceFeedback();
    ~ForceFeedback();
    ForceFeedback(const ForceFeedback&) = delete;
    ForceFeedback& operator=(const ForceFeedback&) = delete;

    void init(GLFWwindow* window);
    void shutdown();

    // Main thread, once per frame: settings and which device to use (re-opens on change).
    void configure(const FfbConfig& cfg, const FfbDeviceHint& hint);
    // Main thread: the steering column torque of this frame's physics steps.
    void push(const std::vector<float>& samples);
    void rescan();
    void startTest();
    void resetPeak();
    FfbStatus status() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace apex
