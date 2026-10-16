#include "ui/UiTheme.h"

#include <string>

#include "core/Log.h"
#include "core/Paths.h"
#include "imgui_impl_opengl3.h"

namespace apex::ui {

namespace {
Fonts g_fonts;
float g_scale = 1.0f;
}  // namespace

Fonts& fonts() { return g_fonts; }
float scale() { return g_scale; }

void setupTheme(float s) {
    g_scale = s;
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->Clear();
    const std::string regular = paths::asset("fonts/TitilliumWeb-Regular.ttf");
    const std::string semi = paths::asset("fonts/TitilliumWeb-SemiBold.ttf");
    const std::string bold = paths::asset("fonts/TitilliumWeb-Bold.ttf");
    ImFontConfig cfg;
    cfg.OversampleH = 2;
    cfg.OversampleV = 2;
    static const ImWchar ranges[] = {0x0020, 0x00FF, 0x2010, 0x2027, 0x2190, 0x2193, 0};
    auto load = [&](const std::string& f, float px) {
        ImFont* font = io.Fonts->AddFontFromFileTTF(f.c_str(), px * s, &cfg, ranges);
        if (!font) font = io.Fonts->AddFontDefault();
        return font;
    };
    g_fonts.body = load(regular, 21.0f);
    g_fonts.bold = load(semi, 21.0f);
    g_fonts.heading = load(bold, 32.0f);
    g_fonts.title = load(bold, 84.0f);
    g_fonts.hudHuge = load(bold, 92.0f);
    g_fonts.hudLarge = load(semi, 40.0f);
    g_fonts.hudSmall = load(semi, 18.0f);
    io.FontDefault = g_fonts.body;
    io.Fonts->Build();
    ImGui_ImplOpenGL3_DestroyFontsTexture();
    ImGui_ImplOpenGL3_CreateFontsTexture();

    ImGuiStyle& st = ImGui::GetStyle();
    st = ImGuiStyle();
    st.WindowRounding = 6.0f;
    st.FrameRounding = 4.0f;
    st.GrabRounding = 4.0f;
    st.TabRounding = 4.0f;
    st.ChildRounding = 4.0f;
    st.PopupRounding = 4.0f;
    st.WindowBorderSize = 0.0f;
    st.FrameBorderSize = 0.0f;
    st.WindowPadding = ImVec2(22, 20);
    st.FramePadding = ImVec2(12, 7);
    st.ItemSpacing = ImVec2(12, 10);
    st.ItemInnerSpacing = ImVec2(10, 6);
    st.ScrollbarSize = 14.0f;
    st.GrabMinSize = 14.0f;
    st.ScaleAllSizes(s);

    ImVec4* c = st.Colors;
    const ImVec4 accent(0.90f, 0.23f, 0.13f, 1.0f);
    c[ImGuiCol_Text] = ImVec4(0.95f, 0.95f, 0.96f, 1.0f);
    c[ImGuiCol_TextDisabled] = ImVec4(0.55f, 0.56f, 0.60f, 1.0f);
    c[ImGuiCol_WindowBg] = ImVec4(0.05f, 0.05f, 0.065f, 0.90f);
    c[ImGuiCol_ChildBg] = ImVec4(0.09f, 0.09f, 0.11f, 0.65f);
    c[ImGuiCol_PopupBg] = ImVec4(0.07f, 0.07f, 0.09f, 0.97f);
    c[ImGuiCol_Border] = ImVec4(0.25f, 0.25f, 0.28f, 0.6f);
    c[ImGuiCol_FrameBg] = ImVec4(0.14f, 0.14f, 0.17f, 1.0f);
    c[ImGuiCol_FrameBgHovered] = ImVec4(0.20f, 0.20f, 0.24f, 1.0f);
    c[ImGuiCol_FrameBgActive] = ImVec4(0.26f, 0.26f, 0.30f, 1.0f);
    c[ImGuiCol_TitleBg] = ImVec4(0.05f, 0.05f, 0.06f, 1.0f);
    c[ImGuiCol_TitleBgActive] = ImVec4(0.08f, 0.08f, 0.10f, 1.0f);
    c[ImGuiCol_CheckMark] = accent;
    c[ImGuiCol_SliderGrab] = accent;
    c[ImGuiCol_SliderGrabActive] = ImVec4(1.0f, 0.38f, 0.25f, 1.0f);
    c[ImGuiCol_Button] = ImVec4(0.16f, 0.16f, 0.19f, 1.0f);
    c[ImGuiCol_ButtonHovered] = ImVec4(0.75f, 0.20f, 0.12f, 1.0f);
    c[ImGuiCol_ButtonActive] = accent;
    c[ImGuiCol_Header] = ImVec4(0.18f, 0.18f, 0.22f, 1.0f);
    c[ImGuiCol_HeaderHovered] = ImVec4(0.75f, 0.20f, 0.12f, 0.8f);
    c[ImGuiCol_HeaderActive] = accent;
    c[ImGuiCol_Separator] = ImVec4(0.3f, 0.3f, 0.34f, 0.6f);
    c[ImGuiCol_Tab] = ImVec4(0.12f, 0.12f, 0.15f, 1.0f);
    c[ImGuiCol_TabHovered] = ImVec4(0.75f, 0.20f, 0.12f, 1.0f);
    c[ImGuiCol_TabSelected] = accent;
    c[ImGuiCol_TabSelectedOverline] = accent;
    c[ImGuiCol_PlotHistogram] = accent;
    c[ImGuiCol_ModalWindowDimBg] = ImVec4(0, 0, 0, 0.6f);
}

bool menuButton(const char* label, float width, bool selected) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float h = 56.0f * g_scale;
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImGui::PushID(label);
    bool clicked = ImGui::InvisibleButton("##btn", ImVec2(width, h));
    bool hovered = ImGui::IsItemHovered();
    ImGui::PopID();
    ImU32 bg = selected ? IM_COL32(230, 58, 34, 70) : (hovered ? IM_COL32(255, 255, 255, 22) : IM_COL32(0, 0, 0, 0));
    dl->AddRectFilled(p, ImVec2(p.x + width, p.y + h), bg, 3.0f * g_scale);
    if (hovered || selected) dl->AddRectFilled(p, ImVec2(p.x + 5.0f * g_scale, p.y + h), kAccent);
    ImGui::PushFont(g_fonts.heading);
    ImVec2 ts = ImGui::CalcTextSize(label);
    dl->AddText(ImVec2(p.x + 24.0f * g_scale, p.y + (h - ts.y) * 0.5f), hovered || selected ? IM_COL32_WHITE : IM_COL32(220, 220, 225, 255), label);
    ImGui::PopFont();
    return clicked;
}

void sectionHeader(const char* text) {
    ImGui::Spacing();
    ImGui::PushFont(g_fonts.bold);
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.45f, 0.30f, 1.0f));
    ImGui::TextUnformatted(text);
    ImGui::PopStyleColor();
    ImGui::PopFont();
    ImGui::Separator();
}

}  // namespace apex::ui
