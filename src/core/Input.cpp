#include "core/Input.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace apex {

namespace {
float applyDeadzone(float v, float dz) {
    float a = std::abs(v);
    if (a <= dz) return 0.0f;
    return std::copysign((a - dz) / (1.0f - dz), v);
}

const char* keyLabel(int key) {
    switch (key) {
        case GLFW_KEY_UP: return "Up";
        case GLFW_KEY_DOWN: return "Down";
        case GLFW_KEY_LEFT: return "Left";
        case GLFW_KEY_RIGHT: return "Right";
        case GLFW_KEY_SPACE: return "Space";
        case GLFW_KEY_ESCAPE: return "Esc";
        case GLFW_KEY_ENTER: return "Enter";
        case GLFW_KEY_TAB: return "Tab";
        case GLFW_KEY_BACKSPACE: return "Backspace";
        case GLFW_KEY_LEFT_SHIFT: return "L-Shift";
        case GLFW_KEY_RIGHT_SHIFT: return "R-Shift";
        case GLFW_KEY_LEFT_CONTROL: return "L-Ctrl";
        case GLFW_KEY_RIGHT_CONTROL: return "R-Ctrl";
        case GLFW_KEY_LEFT_ALT: return "L-Alt";
        case GLFW_KEY_RIGHT_ALT: return "R-Alt";
        case GLFW_KEY_PAGE_UP: return "PgUp";
        case GLFW_KEY_PAGE_DOWN: return "PgDn";
        case GLFW_KEY_HOME: return "Home";
        case GLFW_KEY_END: return "End";
        case GLFW_KEY_INSERT: return "Insert";
        case GLFW_KEY_DELETE: return "Delete";
        default: return nullptr;
    }
}
}  // namespace

void Input::init(GLFWwindow* window, Settings* settings) {
    window_ = window;
    settings_ = settings;
}

bool Input::keyDown(int key) const { return window_ && key >= 0 && glfwGetKey(window_, key) == GLFW_PRESS; }

std::vector<JoystickInfo> Input::joysticks() const {
    std::vector<JoystickInfo> out;
    for (int jid = GLFW_JOYSTICK_1; jid <= GLFW_JOYSTICK_LAST; ++jid) {
        if (!glfwJoystickPresent(jid)) continue;
        JoystickInfo j;
        j.jid = jid;
        const char* n = glfwGetJoystickName(jid);
        const char* g = glfwGetJoystickGUID(jid);
        j.name = n ? n : "Controller";
        j.guid = g ? g : "";
        j.gamepad = glfwJoystickIsGamepad(jid);
        glfwGetJoystickAxes(jid, &j.axes);
        glfwGetJoystickButtons(jid, &j.buttons);
        glfwGetJoystickHats(jid, &j.hats);
        out.push_back(j);
    }
    return out;
}

int Input::findJoystick(const std::string& guid) const {
    for (int jid = GLFW_JOYSTICK_1; jid <= GLFW_JOYSTICK_LAST; ++jid) {
        if (!glfwJoystickPresent(jid)) continue;
        const char* g = glfwGetJoystickGUID(jid);
        if (g && guid == g) return jid;
    }
    return -1;
}

float Input::axisValue(const Binding& b) const {
    int jid = findJoystick(b.guid);
    if (jid < 0) return b.rest;
    int count = 0;
    const float* axes = glfwGetJoystickAxes(jid, &count);
    if (!axes || b.index >= count) return b.rest;
    return axes[b.index];
}

bool Input::buttonDown(const Binding& b) const {
    switch (b.type) {
        case Binding::Type::Key: return keyDown(b.index);
        case Binding::Type::JoyButton: {
            int jid = findJoystick(b.guid);
            if (jid < 0) return false;
            int count = 0;
            const unsigned char* bt = glfwGetJoystickButtons(jid, &count);
            return bt && b.index < count && bt[b.index] == GLFW_PRESS;
        }
        case Binding::Type::JoyHat: {
            int jid = findJoystick(b.guid);
            if (jid < 0) return false;
            int count = 0;
            const unsigned char* h = glfwGetJoystickHats(jid, &count);
            int hat = b.index / 4, bit = b.index % 4;
            return h && hat < count && (h[hat] & (1 << bit));
        }
        case Binding::Type::JoyAxis: {
            float v = (axisValue(b) - b.rest) / (b.full - b.rest + 1e-6f);
            return v > 0.6f;
        }
        default: return false;
    }
}

float Input::bindingValue(const Binding& b, bool bipolar) const {
    if (!b.valid()) return 0.0f;
    if (b.type == Binding::Type::JoyAxis) {
        float v = (axisValue(b) - b.rest) / (b.full - b.rest + (b.full >= b.rest ? 1e-6f : -1e-6f));
        return bipolar ? std::clamp(v, -1.0f, 1.0f) : std::clamp(v, 0.0f, 1.0f);
    }
    return buttonDown(b) ? 1.0f : 0.0f;
}

bool Input::actionDown(Action a) const {
    const auto& c = settings_->controls;
    int i = static_cast<int>(a);
    if (c.primary[i].valid() && buttonDown(c.primary[i])) return true;
    if (c.secondary[i].valid() && buttonDown(c.secondary[i])) return true;
    // gamepad defaults
    if (c.useGamepadDefaults && !c.secondary[i].valid()) {
        for (int jid = GLFW_JOYSTICK_1; jid <= GLFW_JOYSTICK_LAST; ++jid) {
            if (!glfwJoystickPresent(jid) || !glfwJoystickIsGamepad(jid)) continue;
            GLFWgamepadstate st;
            if (!glfwGetGamepadState(jid, &st)) continue;
            auto bt = [&](int k) { return st.buttons[k] == GLFW_PRESS; };
            switch (a) {
                case Action::ShiftUp: if (bt(GLFW_GAMEPAD_BUTTON_A) || bt(GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER)) return true; break;
                case Action::ShiftDown: if (bt(GLFW_GAMEPAD_BUTTON_X) || bt(GLFW_GAMEPAD_BUTTON_LEFT_BUMPER)) return true; break;
                case Action::NextCamera: if (bt(GLFW_GAMEPAD_BUTTON_Y)) return true; break;
                case Action::Pause: if (bt(GLFW_GAMEPAD_BUTTON_START)) return true; break;
                case Action::ResetCar: if (bt(GLFW_GAMEPAD_BUTTON_BACK)) return true; break;
                case Action::LookBack: if (bt(GLFW_GAMEPAD_BUTTON_RIGHT_THUMB)) return true; break;
                case Action::LookLeft: if (bt(GLFW_GAMEPAD_BUTTON_DPAD_LEFT)) return true; break;
                case Action::LookRight: if (bt(GLFW_GAMEPAD_BUTTON_DPAD_RIGHT)) return true; break;
                case Action::BrakeBiasFwd: if (bt(GLFW_GAMEPAD_BUTTON_DPAD_UP)) return true; break;
                case Action::BrakeBiasRwd: if (bt(GLFW_GAMEPAD_BUTTON_DPAD_DOWN)) return true; break;
                default: break;
            }
        }
    }
    return false;
}

float Input::analog(Action a, bool& has, bool bipolar) const {
    const auto& c = settings_->controls;
    const Binding& b = c.secondary[static_cast<int>(a)];
    has = false;
    if (b.valid() && b.type == Binding::Type::JoyAxis && findJoystick(b.guid) >= 0) {
        has = true;
        return bindingValue(b, bipolar);
    }
    if (c.useGamepadDefaults && !b.valid()) {
        for (int jid = GLFW_JOYSTICK_1; jid <= GLFW_JOYSTICK_LAST; ++jid) {
            if (!glfwJoystickPresent(jid) || !glfwJoystickIsGamepad(jid)) continue;
            GLFWgamepadstate st;
            if (!glfwGetGamepadState(jid, &st)) continue;
            has = true;
            switch (a) {
                case Action::Steer: return st.axes[GLFW_GAMEPAD_AXIS_LEFT_X];
                case Action::Throttle: return (st.axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER] + 1.0f) * 0.5f;
                case Action::Brake: return (st.axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER] + 1.0f) * 0.5f;
                default: has = false; return 0.0f;
            }
        }
    }
    return 0.0f;
}

DriverControls Input::update(float dt, const SteeringGeometry& geo) {
    DriverControls out;
    const auto& c = settings_->controls;

    // ---------------------------------------------------------------- steering
    bool hasAxis = false;
    float steerAxis = analog(Action::Steer, hasAxis, true);
    const Binding& sb = c.secondary[static_cast<int>(Action::Steer)];
    const bool isWheel = hasAxis && sb.valid();
    const float limit = c.keyboardSpeedSensitive ? steerLimit(geo) : 1.0f;
    // assisted steering moves at most as fast as a driver's hands would turn a wheel: quick at
    // parking speeds, a few degrees per tenth of a second at 250 km/h; faster back to centre so a
    // slide can still be caught
    const float roadRate = std::clamp(1.3f - geo.speed * 0.012f, 0.42f, 1.3f);  // rad/s at the road wheels
    auto rateLimited = [&](float current, float target) {
        float rate = roadRate / std::max(0.05f, geo.maxRoadWheel) * c.keyboardSteerSpeed / 2.2f;
        if (std::abs(target) < std::abs(current) || target * current < 0.0f) rate *= 1.8f;
        return current + std::clamp(target - current, -rate * dt, rate * dt);
    };
    float kbTarget = (actionDown(Action::SteerRight) ? 1.0f : 0.0f) - (actionDown(Action::SteerLeft) ? 1.0f : 0.0f);
    if (std::abs(kbTarget) > 0.0f || !hasAxis) {
        kbSteer_ = rateLimited(kbSteer_, kbTarget * limit);
    }
    if (hasAxis && std::abs(kbTarget) == 0.0f) {
        float v = applyDeadzone(steerAxis, c.steerDeadzone);
        if (isWheel) {
            // 1:1 steering: physical wheel degrees -> car steering wheel degrees
            v = std::copysign(std::pow(std::abs(v), c.steerLinearity), v);
            float deg = v * c.wheelRotation * 0.5f;
            out.steer = std::clamp(deg / (geo.lockDeg * 0.5f), -1.0f, 1.0f);
        } else {
            // gamepad stick: gamma for fine control around centre, speed-sensitive lock, filter
            v = std::copysign(std::pow(std::abs(v), std::max(1.0f, c.padSteerGamma)), v);
            padSteer_ = c.padSteerFilter ? rateLimited(padSteer_, v * limit) : v * limit;
            out.steer = padSteer_;
        }
        out.analogSteer = true;
        kbSteer_ = out.steer;
    } else {
        out.steer = kbSteer_;
        padSteer_ = kbSteer_;
    }

    // ---------------------------------------------------------------- pedals
    auto pedal = [&](Action a, float& kbState) {
        bool has = false;
        float v = analog(a, has, false);
        if (has) v = std::clamp((v - c.pedalDeadzone) / (1.0f - c.pedalDeadzone), 0.0f, 1.0f);
        bool key = actionDown(a);
        float kbTarget2 = key ? 1.0f : 0.0f;
        float rate = key ? 1.0f / 0.10f : 1.0f / 0.07f;
        kbState += std::clamp(kbTarget2 - kbState, -rate * dt, rate * dt);
        return std::max(has ? v : 0.0f, kbState);
    };
    out.throttle = pedal(Action::Throttle, kbThrottle_);
    out.brake = pedal(Action::Brake, kbBrake_);
    out.clutch = pedal(Action::Clutch, kbClutch_);

    // ---------------------------------------------------------------- buttons (edges)
    auto edge = [&](Action a) {
        int i = static_cast<int>(a);
        bool d = actionDown(a);
        bool e = d && !prev_[i];
        prev_[i] = d;
        return e;
    };
    out.shiftUp = edge(Action::ShiftUp);
    out.shiftDown = edge(Action::ShiftDown);
    out.nextCamera = edge(Action::NextCamera);
    out.pause = edge(Action::Pause);
    out.reset = edge(Action::ResetCar);
    out.toggleHud = edge(Action::ToggleHud);
    out.absToggle = edge(Action::AbsToggle);
    out.tcToggle = edge(Action::TcToggle);
    out.biasFwd = edge(Action::BrakeBiasFwd);
    out.biasRwd = edge(Action::BrakeBiasRwd);
    out.lookLeft = actionDown(Action::LookLeft);
    out.lookRight = actionDown(Action::LookRight);
    out.lookBack = actionDown(Action::LookBack);
    return out;
}

float Input::steerLimit(const SteeringGeometry& g) {
    // Ackermann angle for the tightest corner the car can take at this speed (~1.7 g), plus the
    // front slip angle at peak grip
    const float v2 = std::max(g.speed * g.speed, 1.0f);
    const float road = std::atan(g.wheelbase * 16.5f / v2) + g.peakSlipAngle * 1.1f;
    return std::clamp(road / std::max(0.05f, g.maxRoadWheel), 0.12f, 1.0f);
}

void Input::beginCapture(bool analog) {
    capturing_ = true;
    captureAnalog_ = analog;
    captureBaseline_.clear();
    captureButtons_.clear();
    captureGuids_.clear();
    captureExtreme_.clear();
    captureJoy_ = captureAxis_ = -1;
    captureStart_ = glfwGetTime();
    for (int jid = GLFW_JOYSTICK_1; jid <= GLFW_JOYSTICK_LAST; ++jid) {
        std::vector<float> axes;
        std::vector<unsigned char> buttons;
        std::string guid;
        if (glfwJoystickPresent(jid)) {
            int n = 0;
            const float* a = glfwGetJoystickAxes(jid, &n);
            if (a) axes.assign(a, a + n);
            const unsigned char* b = glfwGetJoystickButtons(jid, &n);
            if (b) buttons.assign(b, b + n);
            int h = 0;
            const unsigned char* hats = glfwGetJoystickHats(jid, &h);
            for (int k = 0; k < h; ++k) buttons.push_back(hats[k]);
            const char* g = glfwGetJoystickGUID(jid);
            guid = g ? g : "";
        }
        captureBaseline_.push_back(axes);
        captureButtons_.push_back(buttons);
        captureGuids_.push_back(guid);
    }
    for (int k = GLFW_KEY_SPACE; k <= GLFW_KEY_LAST; ++k) prevKeys_[k] = keyDown(k);
}

bool Input::pollCapture(Binding& out) {
    if (!capturing_) return false;
    const double now = glfwGetTime();
    if (now - captureStart_ < 0.25) return false;  // ignore the click that started the capture
    if (!captureAnalog_) {
        for (int k = GLFW_KEY_SPACE; k <= GLFW_KEY_LAST; ++k) {
            bool d = keyDown(k);
            if (d && !prevKeys_[k]) {
                out = Binding{};
                out.type = Binding::Type::Key;
                out.index = k;
                capturing_ = false;
                return true;
            }
            prevKeys_[k] = d;
        }
    }
    for (int jid = GLFW_JOYSTICK_1; jid <= GLFW_JOYSTICK_LAST; ++jid) {
        if (!glfwJoystickPresent(jid) || jid >= static_cast<int>(captureBaseline_.size())) continue;
        int n = 0;
        if (captureAnalog_) {
            const float* a = glfwGetJoystickAxes(jid, &n);
            const auto& base = captureBaseline_[jid];
            if (!a) continue;
            if (captureJoy_ < 0) {
                for (int k = 0; k < n && k < static_cast<int>(base.size()); ++k) {
                    if (std::abs(a[k] - base[k]) > 0.45f) {
                        captureJoy_ = jid;
                        captureAxis_ = k;
                        captureExtreme_.assign(1, a[k]);
                        break;
                    }
                }
            } else if (jid == captureJoy_ && captureAxis_ < n) {
                float v = a[captureAxis_];
                float b = base[captureAxis_];
                if (std::abs(v - b) > std::abs(captureExtreme_[0] - b)) captureExtreme_[0] = v;
                // released (back near the rest position) -> done
                if (std::abs(v - b) < 0.15f) {
                    out = Binding{};
                    out.type = Binding::Type::JoyAxis;
                    out.index = captureAxis_;
                    out.rest = b;
                    out.full = captureExtreme_[0];
                    out.guid = captureGuids_[jid];
                    capturing_ = false;
                    return true;
                }
            }
        } else {
            const unsigned char* b = glfwGetJoystickButtons(jid, &n);
            const auto& base = captureButtons_[jid];
            for (int k = 0; b && k < n && k < static_cast<int>(base.size()); ++k) {
                if (b[k] == GLFW_PRESS && base[k] != GLFW_PRESS) {
                    out = Binding{};
                    out.type = Binding::Type::JoyButton;
                    out.index = k;
                    out.guid = captureGuids_[jid];
                    capturing_ = false;
                    return true;
                }
            }
            int h = 0;
            const unsigned char* hats = glfwGetJoystickHats(jid, &h);
            for (int k = 0; hats && k < h && n + k < static_cast<int>(base.size()); ++k) {
                unsigned char changed = hats[k] & ~base[n + k];
                for (int bit = 0; bit < 4; ++bit) {
                    if (changed & (1 << bit)) {
                        out = Binding{};
                        out.type = Binding::Type::JoyHat;
                        out.index = k * 4 + bit;
                        out.guid = captureGuids_[jid];
                        capturing_ = false;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

std::string Input::describe(const Binding& b) const {
    char buf[160];
    switch (b.type) {
        case Binding::Type::Key: {
            if (const char* l = keyLabel(b.index)) return l;
            if (b.index >= GLFW_KEY_F1 && b.index <= GLFW_KEY_F25) {
                snprintf(buf, sizeof(buf), "F%d", b.index - GLFW_KEY_F1 + 1);
                return buf;
            }
            const char* n = glfwGetKeyName(b.index, 0);
            if (n) {
                std::string s = n;
                for (auto& ch : s) ch = static_cast<char>(toupper(ch));
                return s;
            }
            snprintf(buf, sizeof(buf), "Key %d", b.index);
            return buf;
        }
        case Binding::Type::JoyButton:
        case Binding::Type::JoyAxis:
        case Binding::Type::JoyHat: {
            int jid = findJoystick(b.guid);
            const char* name = jid >= 0 ? glfwGetJoystickName(jid) : "Disconnected";
            const char* what = b.type == Binding::Type::JoyButton ? "Button" : (b.type == Binding::Type::JoyAxis ? "Axis" : "Hat");
            snprintf(buf, sizeof(buf), "%s %s %d", name ? name : "Controller", what, b.index);
            return buf;
        }
        default: return "-";
    }
}

}  // namespace apex
