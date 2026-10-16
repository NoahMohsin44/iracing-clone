#pragma once
#include <string>
#include <vector>

#include "audio/AudioDevice.h"
#include "audio/CarAudio.h"
#include "core/Input.h"
#include "core/Settings.h"
#include "game/Session.h"
#include "gfx/Renderer.h"
#include "ui/Hud.h"
#include "ui/MainMenu.h"
#include "ui/SettingsMenu.h"
#include "world/TrackData.h"

struct GLFWwindow;

namespace apex {

class App {
public:
    int run(int argc, char** argv);

private:
    enum class State { Menu, Driving };

    bool createWindow();
    void applyWindowSettings();
    void loadingFrame(float progress, const char* text);
    void frame(double dt);
    void beginUiFrame();
    void endFrame();
    void startPractice();
    void backToMenu();
    void drawPauseMenu();
    void screenshot(const std::string& path);
    RenderSettings renderSettings() const;
    std::vector<std::pair<int, int>> resolutions() const;
    int autotest(const std::string& dir);

    GLFWwindow* window_ = nullptr;
    Settings settings_;
    std::string settingsPath_;
    Input input_;
    TrackData track_;
    Renderer renderer_;
    CarAudio audio_;
    AudioDevice audioDevice_;
    Session player_, demo_;
    State state_ = State::Menu;
    MainMenu menu_;
    SettingsMenu settingsMenu_;
    Hud hud_;
    DriverControls controls_;
    int livery_ = 0;
    bool pauseMenu_ = false;
    bool quit_ = false;
    float appliedTimeOfDay_ = -1.0f;
    double fps_ = 0.0;
    double time_ = 0.0;
    bool wantScreenshot_ = false;
    int fbW_ = 0, fbH_ = 0;
    float uiScale_ = 0.0f;
};

}  // namespace apex
