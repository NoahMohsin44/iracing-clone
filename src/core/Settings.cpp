#include "core/Settings.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <vector>

#include "core/Log.h"

namespace apex {

namespace {
// GLFW key codes (kept numeric so the simulation library does not depend on GLFW headers).
constexpr int KEY_UP = 265, KEY_DOWN = 264, KEY_LEFT = 263, KEY_RIGHT = 262;
constexpr int KEY_E = 69, KEY_Q = 81, KEY_Z = 90, KEY_C = 67, KEY_B = 66, KEY_R = 82, KEY_H = 72;
constexpr int KEY_COMMA = 44, KEY_PERIOD = 46, KEY_ESCAPE = 256, KEY_F7 = 296, KEY_F8 = 297;
constexpr int KEY_LBRACKET = 91, KEY_RBRACKET = 93;

struct ActionInfo {
    const char* name;
    const char* key;
};
constexpr ActionInfo kActions[kActionCount] = {
    {"Steering", "steer"},          {"Steer left", "steer_left"},   {"Steer right", "steer_right"},
    {"Throttle", "throttle"},       {"Brake", "brake"},             {"Clutch", "clutch"},
    {"Shift up", "shift_up"},       {"Shift down", "shift_down"},   {"Next camera", "next_camera"},
    {"Look left", "look_left"},     {"Look right", "look_right"},   {"Look back", "look_back"},
    {"Pause", "pause"},             {"Reset car", "reset_car"},     {"Toggle HUD", "toggle_hud"},
    {"ABS on/off", "abs_toggle"},   {"TC level", "tc_toggle"},      {"Brake bias +", "bias_fwd"},
    {"Brake bias -", "bias_rwd"},
};

std::string bindingToString(const Binding& b) {
    char buf[256];
    switch (b.type) {
        case Binding::Type::Key: snprintf(buf, sizeof(buf), "key:%d", b.index); break;
        case Binding::Type::JoyButton: snprintf(buf, sizeof(buf), "button:%s:%d", b.guid.c_str(), b.index); break;
        case Binding::Type::JoyHat: snprintf(buf, sizeof(buf), "hat:%s:%d", b.guid.c_str(), b.index); break;
        case Binding::Type::JoyAxis:
            snprintf(buf, sizeof(buf), "axis:%s:%d:%.4f:%.4f", b.guid.c_str(), b.index, b.rest, b.full);
            break;
        default: return "none";
    }
    return buf;
}

std::vector<std::string> split(const std::string& s, char sep) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == sep) {
            out.push_back(cur);
            cur.clear();
        } else {
            cur += c;
        }
    }
    out.push_back(cur);
    return out;
}

Binding bindingFromString(const std::string& s) {
    Binding b;
    auto parts = split(s, ':');
    if (parts.empty()) return b;
    const std::string& t = parts[0];
    if (t == "key" && parts.size() >= 2) {
        b.type = Binding::Type::Key;
        b.index = atoi(parts[1].c_str());
    } else if (t == "button" && parts.size() >= 3) {
        b.type = Binding::Type::JoyButton;
        b.guid = parts[1];
        b.index = atoi(parts[2].c_str());
    } else if (t == "hat" && parts.size() >= 3) {
        b.type = Binding::Type::JoyHat;
        b.guid = parts[1];
        b.index = atoi(parts[2].c_str());
    } else if (t == "axis" && parts.size() >= 5) {
        b.type = Binding::Type::JoyAxis;
        b.guid = parts[1];
        b.index = atoi(parts[2].c_str());
        b.rest = static_cast<float>(atof(parts[3].c_str()));
        b.full = static_cast<float>(atof(parts[4].c_str()));
    }
    return b;
}

std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(a, e - a + 1);
}

// Visitor used for both loading and saving so the list of keys exists exactly once.
template <class V>
void visitSettings(V& v, Settings& s) {
    auto& g = s.graphics;
    v("graphics", "width", g.width);
    v("graphics", "height", g.height);
    v("graphics", "window_mode", g.windowMode);
    v("graphics", "vsync", g.vsync);
    v("graphics", "fps_limit", g.fpsLimit);
    v("graphics", "msaa", g.msaa);
    v("graphics", "render_scale", g.renderScale);
    v("graphics", "shadow_quality", g.shadowQuality);
    v("graphics", "vegetation_quality", g.vegetationQuality);
    v("graphics", "anisotropy", g.textureAnisotropy);
    v("graphics", "bloom", g.bloom);
    v("graphics", "ambient_occlusion", g.ambientOcclusion);
    v("graphics", "exposure", g.exposure);
    v("graphics", "fov", g.fov);
    v("graphics", "draw_distance", g.drawDistance);
    v("graphics", "time_of_day", g.timeOfDay);
    v("graphics", "show_fps", g.showFps);
    v("graphics", "motion_blur", g.motionBlur);

    auto& a = s.audio;
    v("audio", "master", a.master);
    v("audio", "engine", a.engine);
    v("audio", "tyres", a.tyres);
    v("audio", "wind", a.wind);
    v("audio", "effects", a.effects);

    auto& c = s.controls;
    v("controls", "wheel_rotation", c.wheelRotation);
    v("controls", "steer_deadzone", c.steerDeadzone);
    v("controls", "steer_linearity", c.steerLinearity);
    v("controls", "pedal_deadzone", c.pedalDeadzone);
    v("controls", "keyboard_steer_speed", c.keyboardSteerSpeed);
    v("controls", "keyboard_speed_sensitive", c.keyboardSpeedSensitive);
    v("controls", "use_gamepad_defaults", c.useGamepadDefaults);
    v("controls", "ffb_enabled", c.ffbEnabled);
    v("controls", "ffb_max_force", c.ffbMaxForce);
    v("controls", "ffb_min_force", c.ffbMinForce);
    v("controls", "ffb_damping", c.ffbDamping);
    v("controls", "ffb_soft_lock", c.ffbSoftLock);
    v("controls", "ffb_invert", c.ffbInvert);

    auto& p = s.gameplay;
    v("gameplay", "metric", p.metric);
    v("gameplay", "auto_clutch", p.autoClutch);
    v("gameplay", "auto_shift", p.autoShift);
    v("gameplay", "abs", p.absEnabled);
    v("gameplay", "tc_level", p.tcLevel);
    v("gameplay", "brake_bias", p.brakeBias);
    v("gameplay", "racing_line", p.showRacingLine);
    v("gameplay", "default_camera", p.defaultCamera);
    v("gameplay", "show_delta", p.showDelta);
}

struct Loader {
    std::map<std::string, std::string>& kv;
    const std::string* find(const char* sec, const char* key) {
        auto it = kv.find(std::string(sec) + "." + key);
        return it == kv.end() ? nullptr : &it->second;
    }
    void operator()(const char* sec, const char* key, int& v) {
        if (auto* s = find(sec, key)) v = atoi(s->c_str());
    }
    void operator()(const char* sec, const char* key, float& v) {
        if (auto* s = find(sec, key)) v = static_cast<float>(atof(s->c_str()));
    }
    void operator()(const char* sec, const char* key, bool& v) {
        if (auto* s = find(sec, key)) v = (*s == "1" || *s == "true" || *s == "yes");
    }
};

struct Saver {
    std::map<std::string, std::vector<std::pair<std::string, std::string>>> sections;
    std::vector<std::string> order;
    void put(const char* sec, const char* key, const std::string& val) {
        if (!sections.count(sec)) order.push_back(sec);
        sections[sec].push_back({key, val});
    }
    void operator()(const char* sec, const char* key, int& v) { put(sec, key, std::to_string(v)); }
    void operator()(const char* sec, const char* key, float& v) {
        char buf[64];
        snprintf(buf, sizeof(buf), "%.4f", v);
        put(sec, key, buf);
    }
    void operator()(const char* sec, const char* key, bool& v) { put(sec, key, v ? "true" : "false"); }
};
}  // namespace

const char* actionName(Action a) { return kActions[static_cast<int>(a)].name; }
const char* actionKey(Action a) { return kActions[static_cast<int>(a)].key; }

void Settings::setDefaultBindings() {
    auto key = [](int code) {
        Binding b;
        b.type = Binding::Type::Key;
        b.index = code;
        return b;
    };
    auto& p = controls.primary;
    p.fill(Binding{});
    p[(int)Action::SteerLeft] = key(KEY_LEFT);
    p[(int)Action::SteerRight] = key(KEY_RIGHT);
    p[(int)Action::Throttle] = key(KEY_UP);
    p[(int)Action::Brake] = key(KEY_DOWN);
    p[(int)Action::Clutch] = key(KEY_Z);
    p[(int)Action::ShiftUp] = key(KEY_E);
    p[(int)Action::ShiftDown] = key(KEY_Q);
    p[(int)Action::NextCamera] = key(KEY_C);
    p[(int)Action::LookLeft] = key(KEY_COMMA);
    p[(int)Action::LookRight] = key(KEY_PERIOD);
    p[(int)Action::LookBack] = key(KEY_B);
    p[(int)Action::Pause] = key(KEY_ESCAPE);
    p[(int)Action::ResetCar] = key(KEY_R);
    p[(int)Action::ToggleHud] = key(KEY_H);
    p[(int)Action::AbsToggle] = key(KEY_F7);
    p[(int)Action::TcToggle] = key(KEY_F8);
    p[(int)Action::BrakeBiasFwd] = key(KEY_RBRACKET);
    p[(int)Action::BrakeBiasRwd] = key(KEY_LBRACKET);
    controls.secondary.fill(Binding{});
}

bool Settings::load(const std::string& path) {
    setDefaultBindings();
    std::ifstream f(path);
    if (!f) return false;
    std::map<std::string, std::string> kv;
    std::string line, section;
    while (std::getline(f, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#' || line[0] == ';') continue;
        if (line.front() == '[' && line.back() == ']') {
            section = line.substr(1, line.size() - 2);
            continue;
        }
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        kv[section + "." + trim(line.substr(0, eq))] = trim(line.substr(eq + 1));
    }
    Loader loader{kv};
    visitSettings(loader, *this);
    for (int i = 0; i < kActionCount; ++i) {
        std::string k = std::string("bindings.") + kActions[i].key;
        auto it = kv.find(k + ".primary");
        if (it != kv.end()) controls.primary[i] = bindingFromString(it->second);
        it = kv.find(k + ".secondary");
        if (it != kv.end()) controls.secondary[i] = bindingFromString(it->second);
    }
    LOGI("Loaded settings from %s", path.c_str());
    return true;
}

bool Settings::save(const std::string& path) const {
    Saver saver;
    visitSettings(saver, const_cast<Settings&>(*this));
    for (int i = 0; i < kActionCount; ++i) {
        saver.put("bindings", (std::string(kActions[i].key) + ".primary").c_str(),
                  bindingToString(controls.primary[i]));
        saver.put("bindings", (std::string(kActions[i].key) + ".secondary").c_str(),
                  bindingToString(controls.secondary[i]));
    }
    std::ofstream f(path);
    if (!f) {
        LOGE("Could not write settings to %s", path.c_str());
        return false;
    }
    f << "# ApexSim settings\n";
    for (auto& sec : saver.order) {
        f << "\n[" << sec << "]\n";
        for (auto& [k, v] : saver.sections[sec]) f << k << " = " << v << "\n";
    }
    return true;
}

}  // namespace apex
