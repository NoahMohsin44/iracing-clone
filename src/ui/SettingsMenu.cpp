#include "ui/SettingsMenu.h"

#include <algorithm>
#include <cstdio>
#include <string>

#include "imgui.h"
#include "ui/UiTheme.h"

namespace apex {

namespace {
bool isAnalog(Action a) { return a == Action::Steer || a == Action::Throttle || a == Action::Brake || a == Action::Clutch; }

void meter(float v, bool bipolar, float width, float height) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 p = ImGui::GetCursorScreenPos();
    dl->AddRectFilled(p, ImVec2(p.x + width, p.y + height), IM_COL32(40, 40, 46, 255), 3);
    if (bipolar) {
        float mid = p.x + width * 0.5f;
        float x = mid + v * width * 0.5f;
        dl->AddRectFilled(ImVec2(std::min(mid, x), p.y), ImVec2(std::max(mid, x), p.y + height), ui::kAccent, 3);
    } else {
        dl->AddRectFilled(p, ImVec2(p.x + width * v, p.y + height), ui::kAccent, 3);
    }
    ImGui::Dummy(ImVec2(width, height));
}
}  // namespace

SettingsResult SettingsMenu::draw(Settings& s, Input& input, const std::vector<std::pair<int, int>>& resolutions) {
    SettingsResult res;
    if (!open_) return res;
    const float sc = ui::scale();
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 size(std::min(1180 * sc, io.DisplaySize.x - 40), std::min(860 * sc, io.DisplaySize.y - 40));
    ImGui::SetNextWindowPos(ImVec2((io.DisplaySize.x - size.x) * 0.5f, (io.DisplaySize.y - size.y) * 0.5f));
    ImGui::SetNextWindowSize(size);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.045f, 0.045f, 0.055f, 0.97f));
    ImGui::Begin("##settings", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
    ImGui::PushFont(ui::fonts().heading);
    ImGui::TextUnformatted("SETTINGS");
    ImGui::PopFont();

    if (ImGui::BeginTabBar("tabs")) {
        auto& g = s.graphics;
        if (ImGui::BeginTabItem("Graphics", nullptr, forceTab_ == 0 ? ImGuiTabItemFlags_SetSelected : 0)) {
            ImGui::BeginChild("gfx", ImVec2(0, -70 * sc));
            ui::sectionHeader("DISPLAY");
            const char* modes[] = {"Windowed", "Borderless fullscreen", "Exclusive fullscreen"};
            ImGui::SetNextItemWidth(360 * sc);
            if (ImGui::Combo("Window mode", &g.windowMode, modes, 3)) res.windowChanged = true;
            char cur[32];
            snprintf(cur, sizeof(cur), "%d x %d", g.width, g.height);
            ImGui::SetNextItemWidth(360 * sc);
            if (ImGui::BeginCombo("Resolution", cur)) {
                for (auto& r : resolutions) {
                    char b[32];
                    snprintf(b, sizeof(b), "%d x %d", r.first, r.second);
                    if (ImGui::Selectable(b, r.first == g.width && r.second == g.height)) {
                        g.width = r.first;
                        g.height = r.second;
                        res.windowChanged = true;
                    }
                }
                ImGui::EndCombo();
            }
            if (ImGui::Checkbox("V-Sync", &g.vsync)) res.windowChanged = true;
            ImGui::SetNextItemWidth(360 * sc);
            ImGui::SliderInt("FPS limit (0 = off)", &g.fpsLimit, 0, 360);
            ImGui::SetNextItemWidth(360 * sc);
            ImGui::SliderFloat("Field of view (vertical)", &g.fov, 30.0f, 90.0f, "%.0f deg");
            ImGui::TextDisabled("Tip: for a single 27\" monitor ~60 cm away, a vertical FOV of 35-45 deg is life-like.");
            ui::sectionHeader("QUALITY");
            const char* msaa[] = {"Off", "2x", "4x", "8x"};
            int mi = g.msaa >= 8 ? 3 : g.msaa >= 4 ? 2 : g.msaa >= 2 ? 1 : 0;
            ImGui::SetNextItemWidth(360 * sc);
            if (ImGui::Combo("Anti-aliasing (MSAA)", &mi, msaa, 4)) {
                g.msaa = mi == 0 ? 1 : (1 << mi);
                res.graphicsChanged = true;
            }
            ImGui::SetNextItemWidth(360 * sc);
            if (ImGui::SliderFloat("Render scale", &g.renderScale, 0.5f, 2.0f, "%.2fx")) res.graphicsChanged = true;
            const char* q[] = {"Low", "Medium", "High", "Ultra"};
            ImGui::SetNextItemWidth(360 * sc);
            if (ImGui::Combo("Shadows", &g.shadowQuality, q, 4)) res.graphicsChanged = true;
            ImGui::SetNextItemWidth(360 * sc);
            if (ImGui::Combo("Trees / vegetation", &g.vegetationQuality, q, 4)) res.graphicsChanged = true;
            ImGui::SetNextItemWidth(360 * sc);
            if (ImGui::SliderFloat("Draw distance", &g.drawDistance, 1000.0f, 8000.0f, "%.0f m")) res.graphicsChanged = true;
            if (ImGui::Checkbox("Bloom", &g.bloom)) res.graphicsChanged = true;
            ImGui::SetNextItemWidth(360 * sc);
            if (ImGui::SliderFloat("Exposure", &g.exposure, -2.0f, 2.0f, "%+.1f EV")) res.graphicsChanged = true;
            ImGui::Checkbox("Show FPS", &g.showFps);
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Audio", nullptr, forceTab_ == 1 ? ImGuiTabItemFlags_SetSelected : 0)) {
            ImGui::BeginChild("aud", ImVec2(0, -70 * sc));
            auto& a = s.audio;
            ui::sectionHeader("VOLUME");
            ImGui::SetNextItemWidth(420 * sc);
            ImGui::SliderFloat("Master", &a.master, 0.0f, 1.0f, "%.2f");
            ImGui::SetNextItemWidth(420 * sc);
            ImGui::SliderFloat("Engine", &a.engine, 0.0f, 1.5f, "%.2f");
            ImGui::SetNextItemWidth(420 * sc);
            ImGui::SliderFloat("Tyres & road", &a.tyres, 0.0f, 1.5f, "%.2f");
            ImGui::SetNextItemWidth(420 * sc);
            ImGui::SliderFloat("Wind", &a.wind, 0.0f, 1.5f, "%.2f");
            ImGui::SetNextItemWidth(420 * sc);
            ImGui::SliderFloat("Effects (impacts, gearbox)", &a.effects, 0.0f, 1.5f, "%.2f");
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Controls", nullptr, forceTab_ == 2 ? ImGuiTabItemFlags_SetSelected : 0)) {
            ImGui::BeginChild("ctl", ImVec2(0, -70 * sc));
            controlsTab(s, input);
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Gameplay", nullptr, forceTab_ == 3 ? ImGuiTabItemFlags_SetSelected : 0)) {
            ImGui::BeginChild("gp", ImVec2(0, -70 * sc));
            auto& p = s.gameplay;
            ui::sectionHeader("UNITS & HUD");
            ImGui::Checkbox("Metric units (km/h)", &p.metric);
            ImGui::Checkbox("Show live delta to best lap", &p.showDelta);
            ui::sectionHeader("DRIVING AIDS");
            ImGui::Checkbox("ABS", &p.absEnabled);
            ImGui::SetNextItemWidth(360 * sc);
            ImGui::SliderInt("Traction control", &p.tcLevel, 0, 6, p.tcLevel ? "%d" : "Off");
            ImGui::Checkbox("Auto clutch (launch / anti-stall)", &p.autoClutch);
            ImGui::Checkbox("Automatic gearbox", &p.autoShift);
            ImGui::SetNextItemWidth(360 * sc);
            float bias = p.brakeBias * 100.0f;
            if (ImGui::SliderFloat("Brake bias (front)", &bias, 45.0f, 70.0f, "%.1f %%")) p.brakeBias = bias / 100.0f;
            const char* cams[] = {"Cockpit", "Hood", "Chase", "Far chase"};
            ImGui::SetNextItemWidth(360 * sc);
            ImGui::Combo("Default camera", &p.defaultCamera, cams, 4);
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    forceTab_ = -1;

    // footer
    ImGui::SetCursorPosY(ImGui::GetWindowHeight() - 60 * sc);
    ImGui::Separator();
    if (ImGui::Button("Defaults", ImVec2(160 * sc, 0))) {
        Settings d;
        d.setDefaultBindings();
        d.graphics.width = s.graphics.width;
        d.graphics.height = s.graphics.height;
        s = d;
        res.graphicsChanged = res.windowChanged = true;
    }
    ImGui::SameLine(ImGui::GetWindowWidth() - 360 * sc);
    if (ImGui::Button("Cancel", ImVec2(160 * sc, 0))) {
        s = backup_;
        res.graphicsChanged = res.windowChanged = true;
        res.close = true;
        open_ = false;
        input.cancelCapture();
    }
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.90f, 0.23f, 0.13f, 1.0f));
    if (ImGui::Button("Save", ImVec2(160 * sc, 0))) {
        res.close = true;
        open_ = false;
        input.cancelCapture();
    }
    ImGui::PopStyleColor();
    ImGui::End();
    ImGui::PopStyleColor();
    return res;
}

void SettingsMenu::controlsTab(Settings& s, Input& input) {
    const float sc = ui::scale();
    auto& c = s.controls;
    ui::sectionHeader("DEVICES");
    auto joys = input.joysticks();
    if (joys.empty()) ImGui::TextDisabled("No wheels, pedals or gamepads detected.");
    for (auto& j : joys) ImGui::Text("%s  (%d axes, %d buttons%s)", j.name.c_str(), j.axes, j.buttons, j.gamepad ? ", gamepad" : "");
    ImGui::Checkbox("Use standard gamepad layout when no controller binding is set", &c.useGamepadDefaults);

    ui::sectionHeader("STEERING");
    ImGui::SetNextItemWidth(360 * sc);
    ImGui::SliderFloat("Wheel rotation (lock to lock)", &c.wheelRotation, 180.0f, 1080.0f, "%.0f deg");
    ImGui::TextDisabled("Set this to your wheel's rotation (e.g. G29 900, T300 1080 set in driver). Steering is 1:1 with the car's 540 deg.");
    ImGui::SetNextItemWidth(360 * sc);
    ImGui::SliderFloat("Steering deadzone", &c.steerDeadzone, 0.0f, 0.2f, "%.2f");
    ImGui::SetNextItemWidth(360 * sc);
    ImGui::SliderFloat("Steering linearity", &c.steerLinearity, 1.0f, 2.5f, "%.2f");
    ImGui::SetNextItemWidth(360 * sc);
    ImGui::SliderFloat("Pedal deadzone", &c.pedalDeadzone, 0.0f, 0.2f, "%.2f");
    ImGui::SetNextItemWidth(360 * sc);
    ImGui::SliderFloat("Keyboard steering speed", &c.keyboardSteerSpeed, 0.8f, 5.0f, "%.1f");
    ImGui::Checkbox("Speed-sensitive keyboard / gamepad steering", &c.keyboardSpeedSensitive);

    ui::sectionHeader("FORCE FEEDBACK");
    FfbStatus st = ffb_ ? ffb_->status() : FfbStatus{};
    if (!st.supported) {
        ImGui::TextDisabled("Force feedback needs Windows (DirectInput).");
    } else if (!c.ffbEnabled) {
        ImGui::TextDisabled("Off - the wheel is free for other apps.");
    } else if (!st.device) {
        ImGui::TextDisabled("No force feedback wheel found.");
    } else {
        ImGui::Text("Wheel: %s%s", st.deviceName.c_str(), st.acquired ? "" : "  (waiting for window focus)");
    }
    if (st.supported) {
        ImGui::SameLine();
        if (ImGui::SmallButton("Rescan") && ffb_) ffb_->rescan();
    }
    ImGui::Checkbox("Enable force feedback", &c.ffbEnabled);
    ImGui::SetNextItemWidth(360 * sc);
    ImGui::SliderFloat("Strength (max force)", &c.ffbMaxForce, 5.0f, 60.0f, "%.1f Nm");
    ImGui::TextDisabled("Steering torque that drives your wheel at 100%%. Lower = stronger forces, but more clipping.\n"
                        "The GT3 makes about 15-30 Nm at the rim in fast corners and ~13 Nm when parked.");
    ImGui::BeginDisabled(st.peakNm < 2.0f);
    if (ImGui::Button("Auto strength") && ffb_) c.ffbMaxForce = std::clamp(st.peakNm * 1.05f, 5.0f, 60.0f);
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextDisabled("highest sustained torque so far: %.1f Nm  (drive a lap first)", st.peakNm);
    ImGui::SameLine();
    if (ImGui::SmallButton("Reset") && ffb_) ffb_->resetPeak();
    float minPct = c.ffbMinForce * 100.0f;
    ImGui::SetNextItemWidth(360 * sc);
    if (ImGui::SliderFloat("Minimum force", &minPct, 0.0f, 30.0f, "%.0f %%")) c.ffbMinForce = minPct / 100.0f;
    float dampPct = c.ffbDamping * 100.0f;
    ImGui::SetNextItemWidth(360 * sc);
    if (ImGui::SliderFloat("Damping", &dampPct, 0.0f, 50.0f, "%.0f %%")) c.ffbDamping = dampPct / 100.0f;
    ImGui::Checkbox("Soft lock at the car's steering lock (540 deg)", &c.ffbSoftLock);
    ImGui::Checkbox("Invert force direction", &c.ffbInvert);
    ImGui::BeginDisabled(!st.device || !st.acquired);
    if (ImGui::Button("Test direction") && ffb_) ffb_->startTest();
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextDisabled("The wheel should turn RIGHT, then LEFT. If it goes left first, tick Invert.");
    ImGui::TextUnformatted("Output");
    ImGui::SameLine();
    {
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 p = ImGui::GetCursorScreenPos();
        const float w = 360 * sc, h = 12 * sc;
        dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), IM_COL32(40, 40, 46, 255), 3);
        const float mid = p.x + w * 0.5f, x = mid + std::clamp(st.output, -1.0f, 1.0f) * w * 0.5f;
        dl->AddRectFilled(ImVec2(std::min(mid, x), p.y), ImVec2(std::max(mid, x), p.y + h),
                          st.clipping ? ui::kRed : ui::kYellow, 3);
        ImGui::Dummy(ImVec2(w, h));
    }
    if (st.live) {
        ImGui::SameLine();
        ImGui::Text("%.1f Nm   clipping %.0f%% of the time", st.torqueNm, st.clipFraction * 100.0f);
    }

    ui::sectionHeader("BINDINGS");
    ImGui::TextDisabled("Click a slot, then press a key / button. For axes: turn the wheel fully RIGHT or press the pedal fully, then release.");
    if (ImGui::BeginTable("binds", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Action", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("Keyboard", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("Wheel / controller", ImGuiTableColumnFlags_WidthStretch, 1.6f);
        ImGui::TableSetupColumn("Live", ImGuiTableColumnFlags_WidthStretch, 0.8f);
        ImGui::TableHeadersRow();
        for (int i = 0; i < kActionCount; ++i) {
            Action a = static_cast<Action>(i);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(actionName(a));
            for (int slot = 0; slot < 2; ++slot) {
                ImGui::TableNextColumn();
                bool secondary = slot == 1;
                if (!secondary && a == Action::Steer) {
                    ImGui::TextDisabled("(use Steer left/right)");
                    continue;
                }
                if (secondary && (a == Action::SteerLeft || a == Action::SteerRight)) {
                    ImGui::TextDisabled("-");
                    continue;
                }
                Binding& b = secondary ? c.secondary[i] : c.primary[i];
                ImGui::PushID(i * 2 + slot);
                bool capturingThis = captureAction_ == i && captureSecondary_ == secondary;
                std::string label = capturingThis ? (isAnalog(a) && secondary ? "Move the axis..." : "Press...") : input.describe(b);
                if (ImGui::Button(label.c_str(), ImVec2(-40 * sc, 0))) {
                    captureAction_ = i;
                    captureSecondary_ = secondary;
                    input.beginCapture(secondary && isAnalog(a));
                }
                ImGui::SameLine();
                if (ImGui::Button("x", ImVec2(30 * sc, 0))) b = Binding{};
                ImGui::PopID();
            }
            ImGui::TableNextColumn();
            bool bip = a == Action::Steer;
            float v = std::max(input.bindingValue(c.primary[i], bip), input.bindingValue(c.secondary[i], bip));
            if (bip) v = input.bindingValue(c.secondary[i], true);
            meter(v, bip, ImGui::GetContentRegionAvail().x, 12 * sc);
        }
        ImGui::EndTable();
    }
    if (captureAction_ >= 0) {
        Binding nb;
        if (input.pollCapture(nb)) {
            Binding& target = captureSecondary_ ? c.secondary[captureAction_] : c.primary[captureAction_];
            // keyboard slot only accepts keys, controller slot only controller input
            bool isKey = nb.type == Binding::Type::Key;
            if (isKey != captureSecondary_) target = nb;
            captureAction_ = -1;
        } else if (!input.capturing()) {
            captureAction_ = -1;
        }
    }
}

}  // namespace apex
