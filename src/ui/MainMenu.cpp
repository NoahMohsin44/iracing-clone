#include "ui/MainMenu.h"

#include <algorithm>
#include <cstdio>

#include "imgui.h"
#include "ui/UiTheme.h"

namespace apex {

const std::vector<Livery>& liveries() {
    static const std::vector<Livery> l = {
        {"Eifel White", {0.90f, 0.90f, 0.88f}, {0.04f, 0.16f, 0.50f}, {0.95f, 0.35f, 0.02f}},
        {"Ring Green", {0.02f, 0.22f, 0.10f}, {0.85f, 0.85f, 0.80f}, {0.95f, 0.75f, 0.05f}},
        {"Racing Yellow", {0.95f, 0.70f, 0.03f}, {0.02f, 0.02f, 0.02f}, {0.85f, 0.05f, 0.03f}},
        {"Heritage Blue", {0.35f, 0.62f, 0.85f}, {0.97f, 0.45f, 0.03f}, {0.05f, 0.05f, 0.05f}},
        {"Rosso", {0.60f, 0.02f, 0.02f}, {0.95f, 0.95f, 0.92f}, {0.05f, 0.05f, 0.05f}},
        {"Carbon Black", {0.025f, 0.025f, 0.03f}, {0.75f, 0.10f, 0.05f}, {0.85f, 0.85f, 0.85f}},
    };
    return l;
}

MenuAction MainMenu::draw(Settings& settings, const std::vector<JoystickInfo>& joysticks, int& liveryIndex,
                          const std::vector<std::string>& attribution, float trackLengthM) {
    MenuAction action = MenuAction::None;
    const float s = ui::scale();
    ImGuiIO& io = ImGui::GetIO();
    const ImVec2 ds = io.DisplaySize;
    auto& f = ui::fonts();

    // left gradient panel
    ImDrawList* bg = ImGui::GetBackgroundDrawList();
    bg->AddRectFilledMultiColor(ImVec2(0, 0), ImVec2(ds.x * 0.42f, ds.y), IM_COL32(6, 6, 8, 235), IM_COL32(6, 6, 8, 0),
                                IM_COL32(6, 6, 8, 0), IM_COL32(6, 6, 8, 235));
    bg->AddRectFilledMultiColor(ImVec2(0, ds.y * 0.8f), ImVec2(ds.x, ds.y), IM_COL32(0, 0, 0, 0), IM_COL32(0, 0, 0, 0),
                                IM_COL32(0, 0, 0, 200), IM_COL32(0, 0, 0, 200));

    ImGui::SetNextWindowPos(ImVec2(70 * s, 70 * s));
    ImGui::SetNextWindowSize(ImVec2(560 * s, ds.y - 140 * s));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0, 0, 0, 0));
    ImGui::Begin("##menu", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);
    ImGui::PushFont(f.title);
    ImGui::TextUnformatted("APEX");
    ImGui::PopFont();
    ImGui::SameLine();
    ImGui::PushFont(f.title);
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.90f, 0.23f, 0.13f, 1.0f));
    ImGui::TextUnformatted("SIM");
    ImGui::PopStyleColor();
    ImGui::PopFont();
    ImGui::PushFont(f.bold);
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.75f, 0.76f, 0.80f, 1.0f));
    ImGui::TextUnformatted("NÜRBURGRING NORDSCHLEIFE  ·  GT3");
    ImGui::PopStyleColor();
    ImGui::PopFont();
    ImGui::Dummy(ImVec2(0, 40 * s));

    const float bw = 420 * s;
    if (ui::menuButton("DRIVE", bw, page_ == 1)) page_ = page_ == 1 ? 0 : 1;
    if (ui::menuButton("SETTINGS", bw)) action = MenuAction::OpenSettings;
    if (ui::menuButton("CREDITS", bw, page_ == 2)) page_ = page_ == 2 ? 0 : 2;
    if (ui::menuButton("QUIT", bw)) action = MenuAction::Quit;

    ImGui::SetCursorPosY(ImGui::GetWindowHeight() - 150 * s);
    ImGui::PushFont(f.hudSmall);
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.62f, 0.63f, 0.67f, 1.0f));
    ImGui::TextUnformatted("CONTROLLERS");
    ImGui::PopStyleColor();
    if (joysticks.empty()) ImGui::TextDisabled("Keyboard (no wheel or gamepad detected)");
    for (const auto& j : joysticks) {
        ImGui::Text("%s  %s", j.gamepad ? "[gamepad]" : "[wheel/joystick]", j.name.c_str());
    }
    ImGui::PopFont();
    ImGui::End();
    ImGui::PopStyleColor();

    // ---------------------------------------------------------------- drive setup
    if (page_ == 1) {
        ImGui::SetNextWindowPos(ImVec2(ds.x - 640 * s, 70 * s));
        ImGui::SetNextWindowSize(ImVec2(570 * s, 0));
        ImGui::Begin("##drive", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
        ImGui::PushFont(f.heading);
        ImGui::TextUnformatted("Practice session");
        ImGui::PopFont();
        ImGui::TextDisabled("Nürburgring Nordschleife  ·  %.2f km  ·  73 corners", trackLengthM / 1000.0f);
        ui::sectionHeader("CAR");
        ImGui::TextUnformatted("GT3 V8  -  5.4 l NA V8, ~560 hp, 1310 kg, 6-speed sequential");
        ImGui::Spacing();
        ImGui::TextUnformatted("Livery");
        const auto& lv = liveries();
        for (int i = 0; i < static_cast<int>(lv.size()); ++i) {
            ImGui::PushID(i);
            ImVec4 c(lv[i].paint.x, lv[i].paint.y, lv[i].paint.z, 1.0f);
            ImVec4 c2(lv[i].paint2.x, lv[i].paint2.y, lv[i].paint2.z, 1.0f);
            ImVec2 p = ImGui::GetCursorScreenPos();
            const float sz = 54 * s;
            if (ImGui::InvisibleButton("##liv", ImVec2(sz, sz))) liveryIndex = i;
            ImDrawList* dl = ImGui::GetWindowDrawList();
            dl->AddRectFilled(p, ImVec2(p.x + sz, p.y + sz), ImGui::GetColorU32(c), 6 * s);
            dl->AddRectFilled(ImVec2(p.x + sz * 0.35f, p.y), ImVec2(p.x + sz * 0.65f, p.y + sz), ImGui::GetColorU32(c2));
            if (liveryIndex == i) dl->AddRect(p, ImVec2(p.x + sz, p.y + sz), ui::kAccent, 6 * s, 0, 3 * s);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", lv[i].name);
            ImGui::PopID();
            if (i + 1 < static_cast<int>(lv.size())) ImGui::SameLine();
        }
        ui::sectionHeader("CONDITIONS");
        ImGui::SetNextItemWidth(300 * s);
        int minutes = static_cast<int>(settings.graphics.timeOfDay * 60.0f + 0.5f);
        char tbuf[16];
        snprintf(tbuf, sizeof(tbuf), "%02d:%02d", minutes / 60, minutes % 60);
        ImGui::SliderFloat("Time of day", &settings.graphics.timeOfDay, 7.0f, 20.5f, tbuf);
        ui::sectionHeader("ASSISTS");
        ImGui::Checkbox("ABS", &settings.gameplay.absEnabled);
        ImGui::SameLine(200 * s);
        ImGui::Checkbox("Auto clutch", &settings.gameplay.autoClutch);
        ImGui::SameLine(380 * s);
        ImGui::Checkbox("Auto shift", &settings.gameplay.autoShift);
        ImGui::SetNextItemWidth(300 * s);
        ImGui::SliderInt("Traction control", &settings.gameplay.tcLevel, 0, 6, settings.gameplay.tcLevel ? "%d" : "Off");
        ImGui::SetNextItemWidth(300 * s);
        float bias = settings.gameplay.brakeBias * 100.0f;
        if (ImGui::SliderFloat("Brake bias (front)", &bias, 45.0f, 70.0f, "%.1f %%")) settings.gameplay.brakeBias = bias / 100.0f;
        const char* cams[] = {"Cockpit", "Hood", "Chase", "Far chase"};
        ImGui::SetNextItemWidth(300 * s);
        ImGui::Combo("Camera", &settings.gameplay.defaultCamera, cams, 4);
        ImGui::Dummy(ImVec2(0, 14 * s));
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.90f, 0.23f, 0.13f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.0f, 0.35f, 0.22f, 1.0f));
        ImGui::PushFont(f.heading);
        if (ImGui::Button("START PRACTICE", ImVec2(-1, 66 * s))) action = MenuAction::StartPractice;
        ImGui::PopFont();
        ImGui::PopStyleColor(2);
        ImGui::End();
    } else if (page_ == 2) {
        ImGui::SetNextWindowPos(ImVec2(ds.x - 760 * s, 70 * s));
        ImGui::SetNextWindowSize(ImVec2(690 * s, 0));
        ImGui::Begin("##credits", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
        ImGui::PushFont(f.heading);
        ImGui::TextUnformatted("Credits & data sources");
        ImGui::PopFont();
        ImGui::Spacing();
        for (const auto& a : attribution) ImGui::TextWrapped("%s", a.c_str());
        ImGui::TextWrapped("Ground, asphalt and bark textures: Poly Haven (polyhaven.com), CC0.");
        ImGui::TextWrapped("Font: Titillium Web, SIL Open Font License.");
        ImGui::TextWrapped("Libraries: GLFW, Dear ImGui, GLM, miniaudio, stb, glad.");
        ImGui::TextWrapped("Car, trees, sky and all sound are generated procedurally at runtime.");
        ImGui::End();
    }

    // bottom right hint
    ImGui::PushFont(f.hudSmall);
    const char* hint = "Arrow keys / gamepad / wheel  ·  E/Q shift  ·  C camera  ·  Esc pause";
    ImVec2 ts = ImGui::CalcTextSize(hint);
    ImGui::GetForegroundDrawList()->AddText(ImVec2(ds.x - ts.x - 30 * s, ds.y - ts.y - 24 * s), ui::kTextDim, hint);
    ImGui::PopFont();
    return action;
}

}  // namespace apex
