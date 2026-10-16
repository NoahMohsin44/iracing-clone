#include "ui/Hud.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "imgui.h"
#include "ui/UiTheme.h"

namespace apex {

void Hud::init(const TrackData& track) {
    map_.clear();
    mapMin_ = glm::vec2(1e9f);
    mapMax_ = glm::vec2(-1e9f);
    for (int i = 0; i < track.sampleCount(); i += 10) {
        glm::vec2 p(track.sample(i).pos.x, track.sample(i).pos.z);
        map_.push_back(p);
        mapMin_ = glm::min(mapMin_, p);
        mapMax_ = glm::max(mapMax_, p);
    }
    glm::vec2 e = mapMax_ - mapMin_;
    mapAspect_ = e.x / std::max(e.y, 1.0f);
}

void Hud::draw(const Session& s, const TrackData& track, const Settings& settings, float fps, const Renderer::Stats& rs,
               const DriverControls& in) {
    const float sc = ui::scale();
    auto& f = ui::fonts();
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    const ImVec2 ds = ImGui::GetIO().DisplaySize;
    const Car& car = s.car();
    const Drivetrain& dt = car.drivetrain();
    const LapTimer& lt = s.timer();
    const double now = s.simTime();

    if (visible) {
        // ------------------------------------------------------------ centre bottom: gear, speed, revs
        const float cx = ds.x * 0.5f, by = ds.y - 36 * sc;
        const float w = 520 * sc, h = 150 * sc;
        ImVec2 p0(cx - w * 0.5f, by - h), p1(cx + w * 0.5f, by);
        dl->AddRectFilled(p0, p1, ui::kPanel, 10 * sc);
        // rpm bar with shift lights
        const double rpm = dt.rpm();
        const auto& eng = car.params().engine;
        float frac = static_cast<float>(std::clamp(rpm / eng.limiterRpm, 0.0, 1.0));
        ImVec2 r0(p0.x + 18 * sc, p0.y + 16 * sc), r1(p1.x - 18 * sc, p0.y + 34 * sc);
        dl->AddRectFilled(r0, r1, IM_COL32(40, 40, 46, 255), 4 * sc);
        ImU32 rc = frac > 0.97f ? ui::kBlue : (frac > 0.88f ? ui::kRed : (frac > 0.75f ? ui::kYellow : ui::kGreen));
        if (frac > 0.985f && std::fmod(now * 8.0, 1.0) < 0.5) rc = IM_COL32(255, 255, 255, 255);
        dl->AddRectFilled(r0, ImVec2(r0.x + (r1.x - r0.x) * frac, r1.y), rc, 4 * sc);
        for (int k = 1000; k < eng.limiterRpm; k += 1000) {
            float x = r0.x + (r1.x - r0.x) * (k / eng.limiterRpm);
            dl->AddLine(ImVec2(x, r0.y), ImVec2(x, r1.y), IM_COL32(0, 0, 0, 140), 1.5f * sc);
        }
        // gear
        char gear[8];
        int g = dt.targetGear();
        snprintf(gear, sizeof(gear), "%s", g < 0 ? "R" : (g == 0 ? "N" : std::to_string(g).c_str()));
        ImGui::PushFont(f.hudHuge);
        ImVec2 gs = ImGui::CalcTextSize(gear);
        dl->AddText(ImVec2(cx - gs.x * 0.5f, p0.y + 32 * sc), dt.shifting() ? ui::kTextDim : IM_COL32_WHITE, gear);
        ImGui::PopFont();
        // speed + rpm
        const bool metric = settings.gameplay.metric;
        double v = car.speed() * (metric ? 3.6 : 2.23694);
        char spd[32];
        snprintf(spd, sizeof(spd), "%3.0f", v);
        ImGui::PushFont(f.hudLarge);
        ImVec2 ss = ImGui::CalcTextSize(spd);
        dl->AddText(ImVec2(p0.x + 150 * sc - ss.x, p0.y + 58 * sc), IM_COL32_WHITE, spd);
        ImGui::PopFont();
        ImGui::PushFont(f.hudSmall);
        dl->AddText(ImVec2(p0.x + 156 * sc, p0.y + 78 * sc), ui::kTextDim, metric ? "km/h" : "mph");
        char rbuf[32];
        snprintf(rbuf, sizeof(rbuf), "%5.0f", rpm);
        dl->AddText(ImGui::GetFont(), 0, ImVec2(p1.x - 160 * sc, p0.y + 64 * sc), IM_COL32_WHITE, rbuf);
        dl->AddText(ImVec2(p1.x - 80 * sc, p0.y + 64 * sc), ui::kTextDim, "rpm");
        // assists row
        char ab[96];
        snprintf(ab, sizeof(ab), "ABS %s   TC %s   BB %.1f%%", car.absEnabled ? "ON" : "OFF",
                 car.tcLevel ? std::to_string(car.tcLevel).c_str() : "OFF", car.brakeBias * 100.0);
        ImVec2 as = ImGui::CalcTextSize(ab);
        dl->AddText(ImVec2(cx - as.x * 0.5f, by - 30 * sc), ui::kTextDim, ab);
        if (car.absActive()) dl->AddText(ImVec2(p0.x + 20 * sc, by - 30 * sc), ui::kYellow, "ABS");
        if (car.tcActive()) dl->AddText(ImVec2(p1.x - 50 * sc, by - 30 * sc), ui::kYellow, "TC");
        ImGui::PopFont();

        // ------------------------------------------------------------ pedals + steering (left of centre)
        {
            ImVec2 q0(p0.x - 120 * sc, p0.y), q1(p0.x - 14 * sc, by);
            dl->AddRectFilled(q0, q1, ui::kPanel, 10 * sc);
            float bh = (q1.y - q0.y) - 30 * sc;
            auto bar = [&](int i, float val, ImU32 col) {
                float x = q0.x + 16 * sc + i * 30 * sc;
                ImVec2 a(x, q0.y + 15 * sc), b(x + 20 * sc, q0.y + 15 * sc + bh);
                dl->AddRectFilled(a, b, IM_COL32(40, 40, 46, 255), 3 * sc);
                dl->AddRectFilled(ImVec2(a.x, b.y - bh * val), b, col, 3 * sc);
            };
            bar(0, static_cast<float>(car.appliedClutch()), ui::kBlue);
            bar(1, static_cast<float>(car.appliedBrake()), ui::kRed);
            bar(2, static_cast<float>(car.appliedThrottle()), ui::kGreen);
            // steering indicator under the panel
            float st = static_cast<float>(car.steeringAngle() / car.params().front.maxSteer);
            ImVec2 c0(q0.x, by + 8 * sc), c1(q1.x, by + 14 * sc);
            dl->AddRectFilled(c0, c1, IM_COL32(40, 40, 46, 200), 2 * sc);
            float mx = (c0.x + c1.x) * 0.5f;
            dl->AddRectFilled(ImVec2(std::min(mx, mx + st * (c1.x - c0.x) * 0.5f), c0.y),
                              ImVec2(std::max(mx, mx + st * (c1.x - c0.x) * 0.5f), c1.y), IM_COL32_WHITE, 2 * sc);
        }

        // ------------------------------------------------------------ top left: timing
        {
            ImVec2 t0(28 * sc, 28 * sc), t1(t0.x + 380 * sc, t0.y + 200 * sc);
            dl->AddRectFilled(t0, t1, ui::kPanel, 10 * sc);
            float y = t0.y + 14 * sc;
            ImGui::PushFont(f.hudSmall);
            dl->AddText(ImVec2(t0.x + 18 * sc, y), ui::kTextDim, lt.timing() ? (lt.currentValid() ? "LAP TIME" : "LAP TIME  (INVALID)") : "OUT LAP");
            ImGui::PopFont();
            y += 22 * sc;
            ImGui::PushFont(f.hudLarge);
            std::string cur = lt.timing() ? LapTimer::format(lt.currentLap(now)) : "--:--.---";
            dl->AddText(ImVec2(t0.x + 18 * sc, y), lt.currentValid() ? IM_COL32_WHITE : ui::kRed, cur.c_str());
            if (settings.gameplay.showDelta && lt.hasDelta()) {
                double d = lt.delta(now, car.trackLocation().s);
                std::string ds2 = LapTimer::format(d, true);
                ImVec2 dsz = ImGui::CalcTextSize(ds2.c_str());
                dl->AddText(ImVec2(t1.x - dsz.x - 18 * sc, y), d <= 0 ? ui::kGreen : ui::kRed, ds2.c_str());
            }
            ImGui::PopFont();
            y += 50 * sc;
            ImGui::PushFont(f.bold);
            char line[96];
            snprintf(line, sizeof(line), "Last   %s%s", lt.lastLap() > 0 ? LapTimer::format(lt.lastLap()).c_str() : "--",
                     lt.lastLap() > 0 && !lt.lastValid() ? "  (inv)" : "");
            dl->AddText(ImVec2(t0.x + 18 * sc, y), IM_COL32_WHITE, line);
            y += 28 * sc;
            snprintf(line, sizeof(line), "Best   %s", lt.bestLap() > 0 ? LapTimer::format(lt.bestLap()).c_str() : "--");
            dl->AddText(ImVec2(t0.x + 18 * sc, y), IM_COL32(200, 120, 255, 255), line);
            y += 28 * sc;
            snprintf(line, sizeof(line), "Laps   %d", lt.laps());
            dl->AddText(ImVec2(t0.x + 18 * sc, y), ui::kTextDim, line);
            ImGui::PopFont();
        }

        // ------------------------------------------------------------ top right: section + map
        {
            const float mw = 300 * sc;
            const float mh = mw / std::max(mapAspect_, 0.5f);
            ImVec2 m0(ds.x - mw - 48 * sc, 28 * sc), m1(ds.x - 28 * sc, 28 * sc + mh + 80 * sc);
            dl->AddRectFilled(m0, m1, ui::kPanel, 10 * sc);
            ImGui::PushFont(f.bold);
            const char* sec = track.sectionName(car.trackLocation().s);
            dl->AddText(ImVec2(m0.x + 18 * sc, m0.y + 12 * sc), IM_COL32_WHITE, sec);
            ImGui::PopFont();
            ImGui::PushFont(f.hudSmall);
            char dist[64];
            snprintf(dist, sizeof(dist), "%.2f / %.2f km", car.trackLocation().s / 1000.0f, track.length() / 1000.0f);
            dl->AddText(ImVec2(m0.x + 18 * sc, m0.y + 40 * sc), ui::kTextDim, dist);
            ImGui::PopFont();
            glm::vec2 e = mapMax_ - mapMin_;
            float scale = std::min((mw - 20 * sc) / e.x, mh / e.y);
            ImVec2 org(m0.x + 10 * sc + (mw - 20 * sc - e.x * scale) * 0.5f, m0.y + 68 * sc);
            auto toScreen = [&](glm::vec2 p) { return ImVec2(org.x + (p.x - mapMin_.x) * scale, org.y + (p.y - mapMin_.y) * scale); };
            std::vector<ImVec2> pts;
            pts.reserve(map_.size());
            for (auto& p : map_) pts.push_back(toScreen(p));
            dl->AddPolyline(pts.data(), static_cast<int>(pts.size()), IM_COL32(200, 200, 205, 220), ImDrawFlags_Closed, 3.0f * sc);
            glm::vec2 sf(track.sample(0).pos.x, track.sample(0).pos.z);
            ImVec2 sfp = toScreen(sf);
            dl->AddRectFilled(ImVec2(sfp.x - 4 * sc, sfp.y - 4 * sc), ImVec2(sfp.x + 4 * sc, sfp.y + 4 * sc), IM_COL32_WHITE);
            ImVec2 cp = toScreen(glm::vec2(car.position().x, car.position().z));
            dl->AddCircleFilled(cp, 6.5f * sc, ui::kAccent);
            dl->AddCircle(cp, 6.5f * sc, IM_COL32_WHITE, 0, 1.5f * sc);
        }
    }

    // ------------------------------------------------------------ messages
    if (s.messageTime > 0.0 && !s.message.empty()) {
        ImGui::PushFont(f.bold);
        ImVec2 ms = ImGui::CalcTextSize(s.message.c_str());
        float a = static_cast<float>(std::min(1.0, s.messageTime / 0.5));
        ImVec2 m0(ds.x * 0.5f - ms.x * 0.5f - 22 * sc, 120 * sc), m1(ds.x * 0.5f + ms.x * 0.5f + 22 * sc, 120 * sc + ms.y + 20 * sc);
        dl->AddRectFilled(m0, m1, IM_COL32(12, 13, 16, static_cast<int>(215 * a)), 8 * sc);
        dl->AddRectFilled(m0, ImVec2(m0.x + 5 * sc, m1.y), IM_COL32(230, 58, 34, static_cast<int>(255 * a)));
        dl->AddText(ImVec2(m0.x + 22 * sc, m0.y + 10 * sc), IM_COL32(255, 255, 255, static_cast<int>(255 * a)), s.message.c_str());
        ImGui::PopFont();
    }

    if (settings.graphics.showFps) {
        char b[96];
        snprintf(b, sizeof(b), "%.0f fps  %.1f ms GPU  %d trees", fps, rs.gpuMs, rs.trees);
        ImGui::PushFont(f.hudSmall);
        ImVec2 bs = ImGui::CalcTextSize(b);
        dl->AddText(ImVec2(ds.x - bs.x - 30 * sc, ds.y - bs.y - 14 * sc), ui::kTextDim, b);
        ImGui::PopFont();
    }
    (void)in;
}

}  // namespace apex
