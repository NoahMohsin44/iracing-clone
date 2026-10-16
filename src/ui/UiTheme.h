#pragma once
#include "imgui.h"

namespace apex::ui {

struct Fonts {
    ImFont* body = nullptr;
    ImFont* bold = nullptr;
    ImFont* heading = nullptr;
    ImFont* title = nullptr;
    ImFont* hudHuge = nullptr;
    ImFont* hudLarge = nullptr;
    ImFont* hudSmall = nullptr;
};

Fonts& fonts();
// (Re)loads fonts for the given UI scale (1.0 at 1080p) and applies the theme.
void setupTheme(float scale);
float scale();

constexpr ImU32 kAccent = IM_COL32(230, 58, 34, 255);
constexpr ImU32 kAccentDim = IM_COL32(150, 38, 22, 255);
constexpr ImU32 kGreen = IM_COL32(70, 210, 90, 255);
constexpr ImU32 kRed = IM_COL32(235, 60, 50, 255);
constexpr ImU32 kBlue = IM_COL32(60, 120, 255, 255);
constexpr ImU32 kYellow = IM_COL32(250, 200, 40, 255);
constexpr ImU32 kTextDim = IM_COL32(170, 172, 180, 255);
constexpr ImU32 kPanel = IM_COL32(12, 13, 16, 215);

// Large flat menu button with an accent bar; returns true when clicked.
bool menuButton(const char* label, float width, bool selected = false);
void sectionHeader(const char* text);

}  // namespace apex::ui
