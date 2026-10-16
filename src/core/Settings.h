#pragma once
#include <array>
#include <string>

namespace apex {

// A physical input bound to a game action. Controllers are identified by GUID so bindings
// survive re-plugging / reboots; the joystick slot is resolved at runtime.
struct Binding {
    enum class Type : int { None = 0, Key = 1, JoyButton = 2, JoyAxis = 3, JoyHat = 4 };
    Type type = Type::None;
    int index = -1;          // key code / button / axis / hat index
    float rest = 0.0f;       // axis value when released (pedals) or centre (steering)
    float full = 1.0f;       // axis value at full deflection
    std::string guid;        // controller GUID (empty for keyboard)
    bool valid() const { return type != Type::None && index >= 0; }
};

enum class Action : int {
    Steer = 0,       // analog bipolar (controller)
    SteerLeft,       // keyboard
    SteerRight,      // keyboard
    Throttle,
    Brake,
    Clutch,
    ShiftUp,
    ShiftDown,
    NextCamera,
    LookLeft,
    LookRight,
    LookBack,
    Pause,
    ResetCar,
    ToggleHud,
    AbsToggle,
    TcToggle,
    BrakeBiasFwd,
    BrakeBiasRwd,
    Count
};
constexpr int kActionCount = static_cast<int>(Action::Count);
const char* actionName(Action a);
const char* actionKey(Action a);  // ini key

struct GraphicsSettings {
    int width = 1920;
    int height = 1080;
    int windowMode = 0;          // 0 windowed, 1 borderless, 2 fullscreen
    bool vsync = true;
    int fpsLimit = 0;            // 0 = unlimited
    int msaa = 4;                // 0, 2, 4, 8
    float renderScale = 1.0f;
    int shadowQuality = 2;       // 0 low .. 3 ultra
    int vegetationQuality = 2;   // 0 low .. 3 ultra
    int textureAnisotropy = 16;
    bool bloom = true;
    bool ambientOcclusion = true;
    float exposure = 0.0f;       // EV offset
    float fov = 60.0f;           // vertical field of view (degrees)
    float drawDistance = 4000.0f;
    float timeOfDay = 14.5f;     // hours
    bool showFps = true;
    bool motionBlur = false;
};

struct AudioSettings {
    float master = 0.8f;
    float engine = 1.0f;
    float tyres = 0.8f;
    float wind = 0.6f;
    float effects = 0.8f;
};

struct ControlSettings {
    std::array<Binding, kActionCount> primary{};    // keyboard
    std::array<Binding, kActionCount> secondary{};  // wheel / pedals / gamepad
    float wheelRotation = 900.0f;     // physical wheel range (lock to lock, degrees)
    float steerDeadzone = 0.02f;
    float steerLinearity = 1.0f;      // 1 = linear, >1 less sensitive around centre
    float pedalDeadzone = 0.02f;
    float keyboardSteerSpeed = 2.2f;  // full-lock per second at standstill
    bool keyboardSpeedSensitive = true;
    bool useGamepadDefaults = true;   // use SDL gamepad mapping if no explicit controller bindings

    // Force feedback (steering wheels). Strength is the steering column torque, in Nm, that drives
    // the wheel at 100 %: lower = stronger forces but more clipping, like iRacing's "max force".
    bool ffbEnabled = true;
    float ffbMaxForce = 22.0f;
    float ffbMinForce = 0.0f;         // 0..0.3, lifts small forces above gear/belt friction
    float ffbDamping = 0.05f;         // 0..1, wheel-side damper
    bool ffbSoftLock = true;          // stop the wheel at the car's 540 deg lock
    bool ffbInvert = false;           // for wheels that push the wrong way
};

struct GameplaySettings {
    bool metric = true;
    bool autoClutch = true;
    bool autoShift = false;
    bool absEnabled = true;
    int tcLevel = 3;            // 0 off .. 6
    float brakeBias = 0.575f;   // front fraction
    bool showRacingLine = false;
    int defaultCamera = 0;      // 0 cockpit, 1 hood, 2 chase, 3 far chase
    bool showDelta = true;
};

struct Settings {
    GraphicsSettings graphics;
    AudioSettings audio;
    ControlSettings controls;
    GameplaySettings gameplay;

    void setDefaultBindings();
    bool load(const std::string& path);
    bool save(const std::string& path) const;
};

}  // namespace apex
