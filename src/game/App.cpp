#include "game/App.h"

#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <thread>

#include "core/Log.h"
#include "core/Paths.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "stb_image_write.h"
#include "ui/UiTheme.h"

// Prefer the discrete GPU on hybrid laptops.
#ifdef _WIN32
extern "C" {
__declspec(dllexport) unsigned long NvOptimusEnablement = 0x00000001;
__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

namespace apex {

namespace {
void glfwError(int code, const char* desc) { LOGE("GLFW error %d: %s", code, desc); }
}  // namespace

RenderSettings App::renderSettings() const {
    RenderSettings r;
    const auto& g = settings_.graphics;
    r.msaa = g.msaa;
    r.renderScale = g.renderScale;
    r.shadowQuality = g.shadowQuality;
    r.vegetationQuality = g.vegetationQuality;
    r.bloom = g.bloom;
    r.exposureEv = g.exposure;
    r.drawDistance = g.drawDistance;
    r.anisotropy = static_cast<float>(g.textureAnisotropy);
    return r;
}

std::vector<std::pair<int, int>> App::resolutions() const {
    std::vector<std::pair<int, int>> out;
    GLFWmonitor* mon = glfwGetPrimaryMonitor();
    int count = 0;
    const GLFWvidmode* modes = mon ? glfwGetVideoModes(mon, &count) : nullptr;
    for (int i = 0; i < count; ++i) {
        std::pair<int, int> r(modes[i].width, modes[i].height);
        if (r.first >= 1024 && std::find(out.begin(), out.end(), r) == out.end()) out.push_back(r);
    }
    if (out.empty()) out = {{1280, 720}, {1600, 900}, {1920, 1080}, {2560, 1440}, {3840, 2160}};
    return out;
}

bool App::createWindow() {
    glfwSetErrorCallback(glfwError);
    if (!glfwInit()) return false;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#ifndef NDEBUG
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
#endif
    glfwWindowHint(GLFW_SAMPLES, 0);
    glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_FALSE);
    const auto& g = settings_.graphics;
    window_ = glfwCreateWindow(g.width, g.height, "ApexSim - Nordschleife", nullptr, nullptr);
    if (!window_) {
        LOGE("Could not create an OpenGL 4.5 window. Update your graphics driver.");
        return false;
    }
    glfwMakeContextCurrent(window_);
    if (!gladLoadGL(reinterpret_cast<GLADloadfunc>(glfwGetProcAddress))) {
        LOGE("Failed to load OpenGL functions");
        return false;
    }
    LOGI("OpenGL %s on %s (%s)", glGetString(GL_VERSION), glGetString(GL_RENDERER), glGetString(GL_VENDOR));
    gl::installDebugCallback();
    applyWindowSettings();
    return true;
}

void App::applyWindowSettings() {
    const auto& g = settings_.graphics;
    GLFWmonitor* mon = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = mon ? glfwGetVideoMode(mon) : nullptr;
    if (g.windowMode == 2 && mon && mode) {
        glfwSetWindowMonitor(window_, mon, 0, 0, g.width, g.height, mode->refreshRate);
    } else if (g.windowMode == 1 && mon && mode) {
        glfwSetWindowMonitor(window_, nullptr, 0, 0, mode->width, mode->height, 0);
        glfwSetWindowAttrib(window_, GLFW_DECORATED, GLFW_FALSE);
        glfwSetWindowPos(window_, 0, 0);
        glfwSetWindowSize(window_, mode->width, mode->height);
    } else {
        glfwSetWindowAttrib(window_, GLFW_DECORATED, GLFW_TRUE);
        int x = 80, y = 60;
        if (mode) {
            x = std::max(0, (mode->width - g.width) / 2);
            y = std::max(30, (mode->height - g.height) / 2);
        }
        glfwSetWindowMonitor(window_, nullptr, x, y, g.width, g.height, 0);
    }
    glfwSwapInterval(g.vsync ? 1 : 0);
}

void App::beginUiFrame() {
    glfwGetFramebufferSize(window_, &fbW_, &fbH_);
    float s = std::clamp(fbH_ / 1080.0f, 0.7f, 2.5f);
    if (std::abs(s - uiScale_) > 0.05f) {
        uiScale_ = s;
        ui::setupTheme(s);
    }
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void App::endFrame() {
    ImGui::Render();
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, fbW_, fbH_);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    if (wantScreenshot_) {
        wantScreenshot_ = false;
        char name[64];
        std::time_t t = std::time(nullptr);
        std::strftime(name, sizeof(name), "shot_%Y%m%d_%H%M%S.png", std::localtime(&t));
        screenshot(paths::userDir() + "screenshots/" + name);
    }
    glfwSwapBuffers(window_);
}

void App::loadingFrame(float progress, const char* text) {
    glfwPollEvents();
    beginUiFrame();
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, fbW_, fbH_);
    glClearColor(0.03f, 0.03f, 0.04f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    const ImVec2 ds = ImGui::GetIO().DisplaySize;
    const float s = ui::scale();
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    auto& f = ui::fonts();
    ImGui::PushFont(f.title);
    dl->AddText(ImVec2(90 * s, ds.y * 0.5f - 120 * s), IM_COL32_WHITE, "APEX");
    ImVec2 w = ImGui::CalcTextSize("APEX ");
    dl->AddText(ImVec2(90 * s + w.x, ds.y * 0.5f - 120 * s), ui::kAccent, "SIM");
    ImGui::PopFont();
    ImGui::PushFont(f.bold);
    dl->AddText(ImVec2(92 * s, ds.y * 0.5f + 10 * s), ui::kTextDim, text);
    ImGui::PopFont();
    ImVec2 b0(92 * s, ds.y * 0.5f + 50 * s), b1(ds.x - 92 * s, ds.y * 0.5f + 56 * s);
    dl->AddRectFilled(b0, b1, IM_COL32(40, 40, 46, 255));
    dl->AddRectFilled(b0, ImVec2(b0.x + (b1.x - b0.x) * progress, b1.y), ui::kAccent);
    endFrame();
}

void App::startPractice() {
    player_.paint = liveries()[livery_].paint;
    player_.paint2 = liveries()[livery_].paint2;
    player_.accent = liveries()[livery_].accent;
    player_.start(&track_, &settings_, false);
    state_ = State::Driving;
    pauseMenu_ = false;
    settings_.save(settingsPath_);
}

void App::backToMenu() {
    state_ = State::Menu;
    pauseMenu_ = false;
    player_.paused = false;
}

void App::screenshot(const std::string& path) {
    std::vector<unsigned char> px(static_cast<size_t>(fbW_) * fbH_ * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, fbW_, fbH_, GL_RGB, GL_UNSIGNED_BYTE, px.data());
    std::vector<unsigned char> flipped(px.size());
    const size_t row = static_cast<size_t>(fbW_) * 3;
    for (int y = 0; y < fbH_; ++y) memcpy(&flipped[y * row], &px[(fbH_ - 1 - y) * row], row);
    if (stbi_write_png(path.c_str(), fbW_, fbH_, 3, flipped.data(), static_cast<int>(row)))
        LOGI("Saved screenshot %s", path.c_str());
}

void App::drawPauseMenu() {
    const float s = ui::scale();
    ImGuiIO& io = ImGui::GetIO();
    ImGui::GetBackgroundDrawList()->AddRectFilled(ImVec2(0, 0), io.DisplaySize, IM_COL32(0, 0, 0, 110));
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f - 230 * s, io.DisplaySize.y * 0.5f - 220 * s));
    ImGui::SetNextWindowSize(ImVec2(460 * s, 0));
    ImGui::Begin("##pause", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
    ImGui::PushFont(ui::fonts().heading);
    ImGui::TextUnformatted("PAUSED");
    ImGui::PopFont();
    ImGui::Spacing();
    const float bw = ImGui::GetContentRegionAvail().x;
    if (ui::menuButton("RESUME", bw)) {
        pauseMenu_ = false;
        player_.paused = false;
    }
    if (ui::menuButton("RESTART", bw)) {
        player_.restart();
        pauseMenu_ = false;
    }
    if (ui::menuButton("SETTINGS", bw)) settingsMenu_.open(settings_);
    if (ui::menuButton("MAIN MENU", bw)) backToMenu();
    if (ui::menuButton("QUIT", bw)) quit_ = true;
    ImGui::End();
}

void App::frame(double dt) {
    beginUiFrame();
    const Car& activeCar = state_ == State::Driving ? player_.car() : demo_.car();
    if (!settingsMenu_.isOpen() || !input_.capturing()) {
        controls_ = input_.update(static_cast<float>(dt), static_cast<float>(activeCar.speed()),
                                  activeCar.params().steeringWheelLock);
    }
    if (glfwGetKey(window_, GLFW_KEY_F12) == GLFW_PRESS) {
        static double last = 0;
        if (time_ - last > 0.5) {
            wantScreenshot_ = true;
            last = time_;
        }
    }
    if (settings_.graphics.timeOfDay != appliedTimeOfDay_) {
        appliedTimeOfDay_ = settings_.graphics.timeOfDay;
        renderer_.setTimeOfDay(appliedTimeOfDay_);
    }

    Session& session = state_ == State::Driving ? player_ : demo_;
    CameraMode mode = session.camera;
    if (state_ == State::Driving) {
        if (controls_.pause && !settingsMenu_.isOpen()) {
            pauseMenu_ = !pauseMenu_;
            player_.paused = pauseMenu_;
        }
        if (controls_.nextCamera) {
            player_.camera = static_cast<CameraMode>((static_cast<int>(player_.camera) + 1) % 5);
            player_.flash(std::string("Camera: ") + cameraModeName(player_.camera));
        }
        if (controls_.toggleHud) hud_.visible = !hud_.visible;
        mode = player_.camera;
    }
    session.update(dt, controls_);
    if (state_ == State::Menu) {
        mode = CameraMode::Orbit;
        // cycle the menu background between orbit and trackside shots
        int phase = static_cast<int>(time_ / 14.0) % 3;
        mode = phase == 0 ? CameraMode::Orbit : (phase == 1 ? CameraMode::TV : CameraMode::Chase);
    }
    session.rig.update(dt, session.car(), track_, mode, controls_, settings_.graphics.fov);

    std::vector<CarVisual> cars;
    const bool cockpit = mode == CameraMode::Cockpit && !session.useOverrideCamera;
    session.fillVisuals(cars, cockpit);
    const Camera& cam = session.useOverrideCamera ? session.overrideCamera : session.rig.camera();
    renderer_.render(cam, cars, fbW_, fbH_, static_cast<float>(time_), cockpit && session.rig.inside());

    // audio
    CarAudioParams ap;
    session.fillAudio(ap);
    ap.interior = cockpit ? 1.0f : (mode == CameraMode::Hood ? 0.6f : 0.0f);
    ap.master = settings_.audio.master * (state_ == State::Menu ? 0.45f : 1.0f);
    ap.engine = settings_.audio.engine;
    ap.tyres = settings_.audio.tyres;
    ap.wind = settings_.audio.wind;
    ap.effects = settings_.audio.effects;
    ap.paused = (state_ == State::Driving && player_.paused) || settingsMenu_.isOpen();
    audio_.setParams(ap);

    // UI
    if (state_ == State::Menu && !settingsMenu_.isOpen()) {
        MenuAction a = menu_.draw(settings_, input_.joysticks(), livery_, track_.attribution(), track_.length());
        if (a == MenuAction::StartPractice) startPractice();
        if (a == MenuAction::OpenSettings) settingsMenu_.open(settings_);
        if (a == MenuAction::Quit) quit_ = true;
    }
    if (state_ == State::Driving) {
        hud_.draw(player_, track_, settings_, static_cast<float>(fps_), renderer_.stats(), controls_);
        if (pauseMenu_ && !settingsMenu_.isOpen()) drawPauseMenu();
    }
    if (settingsMenu_.isOpen()) {
        SettingsResult r = settingsMenu_.draw(settings_, input_, resolutions());
        if (r.windowChanged) applyWindowSettings();
        if (r.graphicsChanged) renderer_.applySettings(renderSettings());
        if (r.close) settings_.save(settingsPath_);
    }
    endFrame();
}

int App::autotest(const std::string& dir) {
    std::filesystem::create_directories(dir);
    struct Shot {
        const char* name;
        float s;
        CameraMode cam;
        float seconds;
        float showcaseAngle = -1.0f;  // degrees around the car (0 = front), < 0 = use the camera mode
        float showcaseDist = 5.0f;
        float showcaseHeight = 1.1f;
    };
    const Shot shots[] = {
        {"01_t13_chase", 20.0f, CameraMode::Chase, 4.0f},       {"02_t13_cockpit", 20.0f, CameraMode::Cockpit, 1.0f},
        {"03_flugplatz_tv", 2230.0f, CameraMode::TV, 3.0f},     {"04_fuchsroehre_chase", 4150.0f, CameraMode::FarChase, 3.0f},
        {"05_karussell_tv", 11905.0f, CameraMode::TV, 3.0f},    {"06_pflanzgarten_hood", 14800.0f, CameraMode::Hood, 3.0f},
        {"07_doettinger_cockpit", 18500.0f, CameraMode::Cockpit, 3.0f}, {"08_orbit", 9000.0f, CameraMode::Orbit, 3.0f},
        {"09_car_front34", 300.0f, CameraMode::Chase, 2.0f, 35.0f, 5.2f, 1.0f},
        {"10_car_side", 300.0f, CameraMode::Chase, 2.0f, 270.0f, 5.8f, 0.9f},
        {"11_car_rear34", 300.0f, CameraMode::Chase, 2.0f, 145.0f, 5.5f, 1.3f},
    };
    // menu shot with the demo running
    // software GL is slow: simulate without drawing, then render a couple of frames per shot
    for (int i = 0; i < 90; ++i) demo_.update(1.0 / 30.0, controls_);
    for (int i = 0; i < 3; ++i) frame(1.0 / 30.0);
    screenshot(dir + "/00_menu.png");
    menu_.setPage(1);
    for (int i = 0; i < 2; ++i) frame(1.0 / 30.0);
    screenshot(dir + "/00b_drive_setup.png");
    menu_.setPage(0);
    settingsMenu_.open(settings_);
    for (int tab : {0, 2}) {
        settingsMenu_.selectTab(tab);
        for (int i = 0; i < 2; ++i) frame(1.0 / 30.0);
        screenshot(dir + (tab == 0 ? "/00c_settings_graphics.png" : "/00d_settings_controls.png"));
    }
    settingsMenu_ = SettingsMenu{};
    state_ = State::Driving;
    for (const Shot& shot : shots) {
        Session& sess = player_;
        sess.start(&track_, &settings_, true);  // AI driven, HUD visible
        sess.car().resetTo(shot.s, track_.racingLineAt(shot.s));
        sess.camera = shot.cam;
        auto t0 = std::chrono::steady_clock::now();
        const int frames = static_cast<int>(shot.seconds * 30.0f);
        sess.useOverrideCamera = false;
        for (int i = 0; i < frames; ++i) {
            sess.update(1.0 / 30.0, controls_);
            sess.rig.update(1.0 / 30.0, sess.car(), track_, shot.cam, controls_, settings_.graphics.fov);
        }
        for (int i = 0; i < 3; ++i) frame(1.0 / 30.0);
        if (shot.showcaseAngle >= 0.0f) {
            // freeze the car and frame it from a fixed angle
            sess.paused = true;
            const Car& car = sess.car();
            double a = glm::radians(static_cast<double>(shot.showcaseAngle));
            glm::dvec3 f = car.forward(), r = car.right();
            glm::dvec3 dir = f * std::cos(a) + r * std::sin(a);
            sess.overrideCamera = Camera{};
            sess.overrideCamera.fovY = glm::radians(40.0f);
            sess.overrideCamera.lookAt(car.position() + dir * static_cast<double>(shot.showcaseDist) +
                                           glm::dvec3(0, shot.showcaseHeight, 0),
                                       car.position() + glm::dvec3(0, 0.05, 0));
            sess.useOverrideCamera = true;
            for (int i = 0; i < 2; ++i) frame(1.0 / 30.0);
        }
        auto t1 = std::chrono::steady_clock::now();
        screenshot(dir + "/" + shot.name + ".png");
        LOGI("autotest %s: %.1f ms/frame, gpu %.1f ms", shot.name,
             std::chrono::duration<double, std::milli>(t1 - t0).count() / frames, renderer_.stats().gpuMs);
    }
    // pause menu over the last shot
    pauseMenu_ = true;
    player_.paused = true;
    player_.useOverrideCamera = false;
    player_.camera = CameraMode::Chase;
    for (int i = 0; i < 2; ++i) frame(1.0 / 30.0);
    screenshot(dir + "/12_pause.png");
    return 0;
}

int App::run(int argc, char** argv) {
    paths::init();
    logInit(paths::userDir() + "apexsim.log");
    settingsPath_ = paths::userDir() + "settings.ini";
    std::string autotestDir;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--autotest") == 0 && i + 1 < argc) autotestDir = argv[++i];
        if (strcmp(argv[i], "--settings") == 0 && i + 1 < argc) settingsPath_ = argv[++i];
    }
    if (!settings_.load(settingsPath_)) {
        settings_.setDefaultBindings();
        LOGI("Using default settings");
    }
    if (!autotestDir.empty()) {
        settings_.graphics.windowMode = 0;
        settings_.graphics.vsync = false;
    }
    LOGI("Assets: %s  User data: %s", paths::assetRoot().c_str(), paths::userDir().c_str());
    if (!createWindow()) return 1;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 450 core");
    input_.init(window_, &settings_);

    loadingFrame(0.0f, "Loading the Nordschleife");
    if (!track_.load(paths::asset("tracks/nordschleife"))) {
        LOGE("Track data missing - run tools/build_track.py");
        return 1;
    }
    hud_.init(track_);
    if (!renderer_.init(track_, renderSettings(), [this](float p, const char* t) { loadingFrame(p, t); })) {
        LOGE("Renderer initialisation failed");
        return 1;
    }
    appliedTimeOfDay_ = -1.0f;
    audio_.init(48000.0f);
    audioDevice_.init(&audio_);
    demo_.start(&track_, &settings_, true);

    if (!autotestDir.empty()) {
        int rc = autotest(autotestDir);
        audioDevice_.shutdown();
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwTerminate();
        return rc;
    }

    auto last = std::chrono::steady_clock::now();
    double fpsAcc = 0;
    int fpsFrames = 0;
    while (!glfwWindowShouldClose(window_) && !quit_) {
        glfwPollEvents();
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last).count();
        last = now;
        dt = std::clamp(dt, 0.0, 0.1);
        time_ += dt;
        fpsAcc += dt;
        if (++fpsFrames >= 20) {
            fps_ = fpsFrames / fpsAcc;
            fpsAcc = 0;
            fpsFrames = 0;
        }
        frame(dt);
        if (settings_.graphics.fpsLimit > 0 && !settings_.graphics.vsync) {
            double target = 1.0 / settings_.graphics.fpsLimit;
            auto end = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(target));
            while (std::chrono::steady_clock::now() < end) std::this_thread::yield();
        }
    }
    settings_.save(settingsPath_);
    audioDevice_.shutdown();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window_);
    glfwTerminate();
    logShutdown();
    return 0;
}

}  // namespace apex
