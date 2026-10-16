#pragma once
#include <string>
#include <vector>

#include "core/Settings.h"

struct GLFWwindow;

namespace apex {

// Processed driver controls for one frame.
struct DriverControls {
    float steer = 0.0f;     // -1..1 fraction of the car's lock (already mapped through wheel rotation)
    float throttle = 0.0f;
    float brake = 0.0f;
    float clutch = 0.0f;
    bool shiftUp = false, shiftDown = false;          // edge (pressed this frame)
    bool nextCamera = false, pause = false, reset = false, toggleHud = false;
    bool absToggle = false, tcToggle = false, biasFwd = false, biasRwd = false;
    bool lookLeft = false, lookRight = false, lookBack = false;  // held
    bool analogSteer = false;  // came from a wheel/stick
};

struct JoystickInfo {
    int jid = -1;
    std::string name;
    std::string guid;
    bool gamepad = false;
    int axes = 0, buttons = 0, hats = 0;
};

class Input {
public:
    void init(GLFWwindow* window, Settings* settings);
    // Poll devices; carSteerLockDeg is the car's steering wheel lock-to-lock range.
    DriverControls update(float dt, float carSpeed, float carSteerLockDeg);
    std::vector<JoystickInfo> joysticks() const;

    // Binding capture (settings menu): call begin, then poll until it returns true.
    void beginCapture(bool analog);
    bool pollCapture(Binding& out);
    void cancelCapture() { capturing_ = false; }
    bool capturing() const { return capturing_; }

    // Live value of a binding (for the settings UI meters), 0..1 or -1..1 for steering.
    float bindingValue(const Binding& b, bool bipolar) const;
    std::string describe(const Binding& b) const;

    // Raw key state for UI/menu navigation
    bool keyDown(int key) const;

private:
    int findJoystick(const std::string& guid) const;
    float axisValue(const Binding& b) const;
    bool buttonDown(const Binding& b) const;
    bool actionDown(Action a) const;
    float analog(Action a, bool& has, bool bipolar) const;
    static float speedSteerScale(float speed);

    GLFWwindow* window_ = nullptr;
    Settings* settings_ = nullptr;
    bool prev_[kActionCount] = {};
    float kbSteer_ = 0.0f, kbThrottle_ = 0.0f, kbBrake_ = 0.0f, kbClutch_ = 0.0f;

    bool capturing_ = false;
    bool captureAnalog_ = false;
    std::vector<std::vector<float>> captureBaseline_;
    std::vector<std::vector<unsigned char>> captureButtons_;
    std::vector<std::string> captureGuids_;
    std::vector<float> captureExtreme_;
    int captureJoy_ = -1, captureAxis_ = -1;
    double captureStart_ = 0.0;
    bool prevKeys_[400] = {};
};

}  // namespace apex
