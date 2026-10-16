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

// What the controller/keyboard steering assist needs to know about the car.
struct SteeringGeometry {
    float speed = 0.0f;          // m/s
    float lockDeg = 540.0f;      // steering wheel lock-to-lock
    float maxRoadWheel = 0.384f; // road-wheel angle at full lock (rad)
    float wheelbase = 2.7f;
    float peakSlipAngle = 0.115f;  // front tyre peak slip angle (rad)
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
    // Poll devices. Wheels steer 1:1 with the car's steering wheel; gamepads and keys go through the
    // steering assist (speed-sensitive lock up to the front tyres' peak, rate limit, gamma).
    DriverControls update(float dt, const SteeringGeometry& geo);
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
    // Largest useful steer fraction at this speed: the angle that puts the front tyres at their peak
    // slip angle while cornering at the grip limit (more only scrubs speed, then snaps back).
    static float steerLimit(const SteeringGeometry& g);

    GLFWwindow* window_ = nullptr;
    Settings* settings_ = nullptr;
    bool prev_[kActionCount] = {};
    float kbSteer_ = 0.0f, kbThrottle_ = 0.0f, kbBrake_ = 0.0f, kbClutch_ = 0.0f;
    float padSteer_ = 0.0f;

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
