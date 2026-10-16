#pragma once
#include "core/Input.h"
#include "core/Settings.h"

namespace apex {

struct SettingsResult {
    bool close = false;
    bool graphicsChanged = false;  // renderer settings (msaa, shadows, vegetation...)
    bool windowChanged = false;    // resolution / window mode / vsync
};

class SettingsMenu {
public:
    void open(const Settings& current) {
        backup_ = current;
        open_ = true;
        tab_ = 0;
    }
    bool isOpen() const { return open_; }
    void selectTab(int t) { forceTab_ = t; }
    SettingsResult draw(Settings& s, Input& input, const std::vector<std::pair<int, int>>& resolutions);

private:
    void controlsTab(Settings& s, Input& input);
    bool open_ = false;
    int tab_ = 0;
    Settings backup_;
    int forceTab_ = -1;
    int captureAction_ = -1;
    bool captureSecondary_ = false;
};

}  // namespace apex
