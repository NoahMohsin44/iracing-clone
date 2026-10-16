// Headless physics validation: runs the GT3 car through standard tests and an AI lap of the
// Nordschleife, printing figures that can be compared with real GT3 data.
//   apexsim_tests            run everything
//   apexsim_tests quick      skip the full lap
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "audio/CarAudio.h"
#include "car/Car.h"
#include "core/FfbSignal.h"
#include "core/Log.h"
#include "core/Paths.h"
#include "game/AIDriver.h"
#include "game/LapTimer.h"
#include "world/TrackData.h"

using namespace apex;

namespace {
constexpr double kDt = 1.0 / 1000.0;
int g_failures = 0;

void check(bool ok, const char* what) {
    printf("  [%s] %s\n", ok ? " OK " : "FAIL", what);
    if (!ok) ++g_failures;
}

std::vector<glm::vec2> stadium(float straight, float radius, float spacing) {
    std::vector<glm::vec2> pts;
    auto addLine = [&](glm::vec2 a, glm::vec2 b) {
        float len = glm::length(b - a);
        int n = static_cast<int>(len / spacing);
        for (int i = 0; i < n; ++i) pts.push_back(a + (b - a) * (i / static_cast<float>(n)));
    };
    auto addArc = [&](glm::vec2 c, float a0, float a1) {
        float len = std::abs(a1 - a0) * radius;
        int n = static_cast<int>(len / spacing);
        for (int i = 0; i < n; ++i) {
            float a = a0 + (a1 - a0) * (i / static_cast<float>(n));
            pts.push_back(c + radius * glm::vec2(std::cos(a), std::sin(a)));
        }
    };
    // drive north (-z) on x=0, right-hand semicircle around (radius, -straight), back south on x=2r
    addLine({0, 0}, {0, -straight});
    addArc({radius, -straight}, 3.14159265f, 2.0f * 3.14159265f);
    addLine({2 * radius, -straight}, {2 * radius, 0});
    addArc({radius, 0}, 0.0f, 3.14159265f);
    return pts;
}

std::vector<glm::vec2> circle(float radius, float spacing) {
    std::vector<glm::vec2> pts;
    int n = static_cast<int>(2 * 3.14159265f * radius / spacing);
    for (int i = 0; i < n; ++i) {
        float a = 2.0f * 3.14159265f * i / n;  // clockwise seen from above = right-hand corner
        pts.push_back(radius * glm::vec2(std::cos(a), std::sin(a)));
    }
    return pts;
}

void testStandstill(const TrackData& track) {
    printf("\n== Standstill stability\n");
    Car car;
    car.init(CarParams::gt3(), &track);
    car.resetTo(20.0f, 0.0f);
    CarInput in;
    double maxV = 0, settleY = 0;
    for (int i = 0; i < 10000; ++i) {
        car.step(kDt, in);
        if (i > 3000) maxV = std::max(maxV, car.speed());
    }
    settleY = car.position().y;
    printf("  max speed after settling: %.4f m/s, CG height %.3f m\n", maxV, settleY);
    for (int i = 0; i < 4; ++i)
        printf("  wheel %d load %.0f N travel %.1f mm\n", i, car.wheels()[i].load, car.wheels()[i].travel * 1000);
    check(maxV < 0.02, "car sits still without creeping or oscillating");
    check(std::abs(settleY - 0.43) < 0.03, "static CG height ~0.43 m");
}

void testAcceleration(const TrackData& track) {
    printf("\n== Acceleration / top speed / braking (flat straight)\n");
    Car car;
    car.init(CarParams::gt3(), &track);
    car.setAutoShift(true);
    car.tcLevel = 3;
    car.resetTo(5.0f, 0.0f);
    AIDriver ai;
    ai.init(&track);
    double t = 0, t100 = -1, t200 = -1, t250 = -1, vmax = 0;
    double brakeStart = -1, brakeDist = -1;
    bool braking = false;
    glm::dvec3 brakePos;
    for (int i = 0; i < 120000; ++i) {
        CarInput in = ai.update(car, kDt);
        in.throttle = braking ? 0.0f : 1.0f;
        in.brake = braking ? 1.0f : 0.0f;
        car.step(kDt, in);
        t += kDt;
        double kmh = car.speed() * 3.6;
        if (t100 < 0 && kmh >= 100) t100 = t;
        if (t200 < 0 && kmh >= 200) t200 = t;
        if (t250 < 0 && kmh >= 250) t250 = t;
        vmax = std::max(vmax, kmh);
        if (!braking && car.trackLocation().s > 4300.0f) {
            braking = true;
            brakeStart = kmh;
            brakePos = car.position();
        }
        if (braking && car.speed() < 0.3) {
            brakeDist = glm::length(car.position() - brakePos);
            break;
        }
    }
    printf("  0-100 km/h: %.2f s   0-200 km/h: %.2f s   0-250 km/h: %.2f s\n", t100, t200, t250);
    printf("  top speed on 4.3 km straight: %.1f km/h\n", vmax);
    printf("  braking from %.0f km/h to stop: %.1f m\n", brakeStart, brakeDist);
    check(t100 > 2.6 && t100 < 4.2, "0-100 km/h in 2.6..4.2 s (GT3: ~3.3 s)");
    check(t200 > 7.0 && t200 < 11.5, "0-200 km/h in 7..11.5 s (GT3: ~9 s)");
    check(vmax > 255 && vmax < 295, "top speed 255..295 km/h");
}

void testBraking(const TrackData& track) {
    printf("\n== Braking 200-0 km/h (ABS on/off)\n");
    for (int abs = 1; abs >= 0; --abs) {
        Car car;
        car.init(CarParams::gt3(), &track);
        car.setAutoShift(true);
        car.absEnabled = abs != 0;
        car.resetTo(5.0f, 0.0f);
        AIDriver ai;
        ai.init(&track);
        bool braking = false;
        glm::dvec3 p0;
        double dist = -1;
        double peakG = 0;
        for (int i = 0; i < 60000; ++i) {
            CarInput in = ai.update(car, kDt);
            if (!braking && car.speed() * 3.6 >= 200.0) {
                braking = true;
                p0 = car.position();
            }
            in.throttle = braking ? 0.0f : 1.0f;
            in.brake = braking ? 1.0f : 0.0f;
            car.step(kDt, in);
            if (braking) peakG = std::max(peakG, car.localAcceleration().z / 9.81);
            if (braking && car.speed() < 0.3) {
                dist = glm::length(car.position() - p0);
                break;
            }
        }
        printf("  ABS %s: 200-0 in %.1f m, peak decel %.2f g\n", abs ? "on " : "off", dist, peakG);
        if (abs) check(dist > 75 && dist < 125, "200-0 km/h in 75..125 m with ABS (GT3: ~95 m)");
    }
}

void testSkidpad(float radius, float startKmh) {
    printf("\n== Skidpad R=%.0f m\n", radius);
    TrackData track;
    track.createSynthetic(circle(radius, 2.0f), 12.0f, 2.0f, 60.0f);
    Car car;
    car.init(CarParams::gt3(), &track);
    car.setAutoShift(true);
    car.resetTo(0.0f, 0.0f);
    // simple line follower at constant radius with a slowly increasing speed target
    double target = startKmh / 3.6;
    double steerI = 0, best = 0, bestV = 0, accLat = 0, colTq = 0, bestTq = 0, peakTq = 0, steerSum = 0, bestSteer = 0;
    int window = 0;
    double drift = 0;
    for (int i = 0; i < 240000; ++i) {
        const auto& loc = car.trackLocation();
        glm::dvec3 tangent(track.tangentAt(loc.s));
        glm::dvec3 fwd = car.forward();
        double headingErr = std::atan2(glm::dot(glm::cross(fwd, tangent), glm::dvec3(0, 1, 0)), glm::dot(fwd, tangent));
        double ff = std::atan(car.params().wheelbase / radius);
        steerI += -loc.lateral * 0.00002;
        double steer = (ff - headingErr * 1.2 - loc.lateral * 0.05 + steerI) / car.params().front.maxSteer;
        CarInput in;
        in.steer = static_cast<float>(std::clamp(steer, -1.0, 1.0));
        double err = target - car.forwardSpeed();
        in.throttle = static_cast<float>(std::clamp(0.3 + err * 0.3, 0.0, 1.0));
        car.step(kDt, in);
        if (i > 15000) target += kDt * 0.25;
        accLat += car.localAcceleration().x;
        colTq += car.steeringTorque();
        steerSum += in.steer;
        if (++window == 1000) {
            double g = std::abs(accLat / window) / 9.81;
            if (std::abs(loc.lateral) < 2.5) peakTq = std::max(peakTq, std::abs(colTq / window));
            if (std::abs(loc.lateral) < 2.5 && g > best) {
                best = g;
                bestV = car.speed() * 3.6;
                bestTq = colTq / window;
                bestSteer = steerSum / window;
            }
            colTq = 0;
            steerSum = 0;
            if (std::abs(loc.lateral) > 6.0) drift += 1;
            accLat = 0;
            window = 0;
            if (drift > 3) break;
        }
    }
    printf("  max steady lateral acceleration: %.2f g at %.0f km/h\n", best, bestV);
    printf("  steering: %.0f deg at the wheel, column torque %.1f Nm at the limit, peak %.1f Nm\n",
           bestSteer * car.params().steeringWheelLock * 0.5, bestTq, peakTq);
    check(bestTq * bestSteer < 0.0, "steering torque centres the wheel in a steady corner");
    check(peakTq > 6.0 && peakTq < 40.0, "cornering steering torque in the GT3 range (6-40 Nm)");
    // (the big skidpad runs out of engine before it runs out of grip)
    if (radius < 100.0f) check(std::abs(bestTq) < peakTq * 0.98, "steering goes light past the front tyres' peak slip angle");
}

void testSteeringFeel(const TrackData& track) {
    printf("\n== Steering feel (column torque)\n");
    Car car;
    car.init(CarParams::gt3(), &track);
    car.resetTo(20.0f, 0.0f);
    CarInput in;
    for (int i = 0; i < 3000; ++i) car.step(kDt, in);
    // parking: turn a quarter of the lock in 0.5 s, then hold
    double turning = 0, holding = 0;
    for (int i = 0; i < 1500; ++i) {
        in.steer = static_cast<float>(std::min(1.0, i / 500.0) * 0.25);
        car.step(kDt, in);
        if (i == 450) turning = car.steeringTorque();
        if (i == 1499) holding = car.steeringTorque();
    }
    printf("  standstill: %.1f Nm while turning, %.1f Nm holding\n", turning, holding);
    check(turning < -3.0 && turning > -30.0, "parking effort resists the driver (3-30 Nm)");
    check(holding < -1.0, "wound-up tyres keep pushing the wheel back at standstill");

    // on-centre feel at 150 km/h: a small steering input gives a linear, centring torque
    Car c2;
    c2.init(CarParams::gt3(), &track);
    c2.setAutoShift(true);
    c2.resetTo(20.0f, 0.0f);
    AIDriver ai;
    ai.init(&track);
    double t1 = 0, t2 = 0;
    for (int i = 0; i < 60000; ++i) {
        CarInput ci = ai.update(c2, kDt);
        double err = 150.0 / 3.6 - c2.forwardSpeed();
        ci.throttle = static_cast<float>(std::clamp(err * 0.5, 0.0, 1.0));
        ci.brake = 0.0f;
        if (c2.speed() > 150.0 / 3.6 - 0.5) {
            // two short steering holds (2 and 4 deg of steering wheel) with the AI otherwise driving
            double base = ci.steer;
            for (int k = 0; k < 200; ++k) {
                CarInput h = ci;
                h.steer = static_cast<float>(base + 2.0 / 270.0);
                c2.step(kDt, h);
            }
            t1 = c2.steeringTorque();
            for (int k = 0; k < 200; ++k) {
                CarInput h = ci;
                h.steer = static_cast<float>(base + 4.0 / 270.0);
                c2.step(kDt, h);
            }
            t2 = c2.steeringTorque();
            break;
        }
        c2.step(kDt, ci);
    }
    printf("  150 km/h on-centre: %.2f Nm at +2 deg, %.2f Nm at +4 deg\n", t1, t2);
    check(t1 < 0.0 && t2 < t1, "on-centre torque centres the wheel and builds with steering angle");
}

void testForceFeedbackSignal() {
    printf("\n== Force feedback signal path\n");
    // 60 fps frames each deliver ~16-17 physics samples of a 20 Hz kerb-like torque; the device
    // thread ticks every ~1 ms with jitter. Playback should be smooth and only ~1 frame late.
    FfbSignal sig;
    FfbConfig cfg;
    cfg.maxForceNm = 20.0f;
    const double amp = 8.0, hz = 20.0;
    auto torqueAt = [&](double t) { return amp * std::sin(2.0 * 3.14159265358979 * hz * t); };
    double simT = 0.0, realT = 0.0, nextFrame = 0.0, acc = 0.0;
    std::vector<std::pair<double, double>> outT;  // (real time, played torque)
    uint32_t rng = 12345;
    auto jitter = [&]() { rng = rng * 1664525u + 1013904223u; return ((rng >> 8) & 0xffff) / 65535.0; };
    while (realT < 3.0) {
        if (realT >= nextFrame) {
            acc += 1.0 / 60.0;
            std::vector<float> burst;
            while (acc >= 0.001) {
                simT += 0.001;
                acc -= 0.001;
                burst.push_back(static_cast<float>(torqueAt(simT)));
            }
            sig.push(burst.data(), static_cast<int>(burst.size()));
            nextFrame += 1.0 / 60.0;
        }
        const double dt = 0.0008 + 0.0006 * jitter();
        realT += dt;
        FfbSignal::Output o = sig.tick(dt, std::nan(""), cfg);
        if (realT > 1.0) outT.push_back({realT, o.torqueNm});
    }
    // best lag and residual
    double bestErr = 1e9, bestLag = 0.0, maxStep = 0.0;
    for (double lag = 0.0; lag < 0.06; lag += 0.0005) {
        double e = 0.0;
        for (auto& [t, v] : outT) e += (v - torqueAt(t - lag)) * (v - torqueAt(t - lag));
        e = std::sqrt(e / outT.size());
        if (e < bestErr) {
            bestErr = e;
            bestLag = lag;
        }
    }
    for (size_t i = 1; i < outT.size(); ++i) maxStep = std::max(maxStep, std::abs(outT[i].second - outT[i - 1].second));
    printf("  playback: %.1f ms latency, rms error %.2f Nm of %.0f, largest step %.2f Nm (60 fps -> 1 kHz)\n",
           bestLag * 1000.0, bestErr, amp, maxStep);
    check(bestLag < 0.035, "playback latency under ~2 frames");
    check(bestErr < amp * 0.15, "20 Hz detail survives 60 fps frames (no staircase)");
    check(maxStep < amp * 0.25, "no frame-rate steps in the output");

    // shaping
    FfbSignal s2;
    std::vector<float> ten(20, 10.0f), thirty(20, 30.0f);
    s2.push(ten.data(), 20);
    FfbSignal::Output o;
    for (int i = 0; i < 1000; ++i) {  // past the 0.6 s fade-in
        if (i % 16 == 0) s2.push(ten.data(), 16);
        o = s2.tick(0.001, std::nan(""), cfg);
    }
    printf("  10 Nm with 20 Nm strength -> %.2f output\n", o.force);
    check(std::abs(o.force - 0.5f) < 0.02f && !o.clipping, "strength maps column torque linearly");
    for (int i = 0; i < 100; ++i) {
        if (i % 16 == 0) s2.push(thirty.data(), 16);
        o = s2.tick(0.001, std::nan(""), cfg);
    }
    check(std::abs(o.force - 1.0f) < 1e-3f && o.clipping, "torque beyond the strength clips (and is reported)");
    // soft lock: 10 deg past the car's 270 deg (half lock) to the right -> full force to the left
    for (int i = 0; i < 20; ++i) {
        if (i % 16 == 0) s2.push(ten.data(), 16);
        o = s2.tick(0.001, 280.0, cfg);
    }
    check(o.force < -0.99f, "soft lock pushes the wheel back from past the lock");
    FfbConfig cfgMin = cfg;
    cfgMin.minForce = 0.1f;
    std::vector<float> tiny(16, 0.4f);  // 2 % of the strength
    for (int i = 0; i < 50; ++i) {
        if (i % 16 == 0) s2.push(tiny.data(), 16);
        o = s2.tick(0.001, 0.0, cfgMin);
    }
    check(std::abs(o.force - (0.1f + 0.9f * 0.02f)) < 0.01f, "minimum force lifts small forces");
    // stream stops (pause): fade out
    for (int i = 0; i < 600; ++i) o = s2.tick(0.001, 0.0, cfg);
    check(std::abs(o.force) < 1e-3f && !o.live, "force fades out when the simulation pauses");
    // runaway guard: wheel 50 deg past the lock while pushing back at full force
    for (int i = 0; i < 700; ++i) {
        if (i % 16 == 0) s2.push(ten.data(), 16);
        o = s2.tick(0.001, i < 650 ? 0.0 : 320.0, cfg);
    }
    check(s2.safetyTrips() == 1 && o.safetyCut && o.force == 0.0f, "runaway guard mutes a wheel that runs far past the lock");
}

void testLap() {
    printf("\n== AI lap of the Nordschleife\n");
    TrackData track;
    if (!track.load(paths::asset("tracks/nordschleife"))) {
        check(false, "load Nordschleife");
        return;
    }
    Car car;
    car.init(CarParams::gt3(), &track);
    car.setAutoShift(true);
    car.resetTo(track.length() - 250.0f, track.racingLineAt(track.length() - 250.0f));
    AIDriver ai;
    ai.init(&track, 0.96f);
    LapTimer timer;
    timer.init(track.length());
    double t = 0, maxLat = 0, maxV = 0;
    int resets = 0, laps = 0;
    double offTrack = 0;
    int lastSection = -1;
    for (long i = 0; i < 1000L * 60 * 20; ++i) {
        CarInput in = ai.update(car, kDt);
        car.step(kDt, in);
        t += kDt;
        timer.update(t, car.trackLocation().s);
        maxLat = std::max(maxLat, std::abs(car.localAcceleration().x) / 9.81);
        maxV = std::max(maxV, car.speed() * 3.6);
        int wheelsOff = 0;
        for (auto& w : car.wheels()) wheelsOff += (w.surface == Surface::Grass || w.surface == Surface::Dirt);
        if (wheelsOff == 4) offTrack += kDt;
        if (getenv("APEX_TRACE") && i % 100 == 0 && t > atof(getenv("APEX_TRACE")) && t < atof(getenv("APEX_TRACE")) + 4) {
            const auto& L = car.trackLocation();
            glm::dvec3 tg(track.tangentAt(L.s));
            double yawErr = std::atan2(glm::dot(glm::cross(car.forward(), tg), glm::dvec3(0, 1, 0)), glm::dot(car.forward(), tg));
            glm::dvec3 vl = glm::transpose(car.rotation()) * car.velocity();
            printf("    t=%.2f s=%.0f lat=%.2f v=%.0f g=%d rpm=%.0f st=%.2f th=%.2f br=%.2f hint=%.0f line=%.2f hdg=%.1f beta=%.1f yaw=%.2f ax=%.2f ay=%.2f sx=%.2f/%.2f/%.2f/%.2f sy=%.2f/%.2f/%.2f/%.2f fz=%.0f/%.0f/%.0f/%.0f\n", t, L.s, L.lateral,
                   car.speed() * 3.6, car.drivetrain().gear(), car.drivetrain().rpm(), in.steer, in.throttle, in.brake,
                   track.speedHintAt(L.s) * 3.6, track.racingLineAt(L.s), yawErr * 57.3, std::atan2(vl.x, -vl.z) * 57.3,
                   -glm::dot(car.angularVelocity(), car.up()), car.localAcceleration().x / 9.81, car.localAcceleration().z / 9.81,
                   car.wheels()[0].slipX, car.wheels()[1].slipX, car.wheels()[2].slipX, car.wheels()[3].slipX,
                   car.wheels()[0].slipY, car.wheels()[1].slipY, car.wheels()[2].slipY, car.wheels()[3].slipY,
                   car.wheels()[0].load, car.wheels()[1].load, car.wheels()[2].load, car.wheels()[3].load);
        }
        if (timer.timing() && i % 1000 == 0) {
            int sec = track.sectionIndex(car.trackLocation().s);
            if (sec != lastSection && laps == 0) {
                lastSection = sec;
                printf("    %7.1fs  %-26s %5.0f km/h  gear %d\n", timer.currentLap(t), track.sectionName(car.trackLocation().s),
                       car.speed() * 3.6, car.drivetrain().gear());
            }
        }
        static double stuckTime = 0;
        glm::dvec3 tan(track.tangentAt(car.trackLocation().s));
        bool wrongWay = glm::dot(car.forward(), tan) < 0.2;
        stuckTime = (car.speed() < 2.0 || wrongWay) ? stuckTime + kDt : 0.0;
        if (car.needsReset() || stuckTime > 3.0) {
            stuckTime = 0;
            ++resets;
            printf("  reset at s=%.0f (%s) speed %.0f km/h\n", car.trackLocation().s,
                   track.sectionName(car.trackLocation().s), car.speed() * 3.6);
            float s = car.trackLocation().s;
            car.resetTo(s, track.racingLineAt(s));
        }
        if (timer.laps() > laps) {
            laps = timer.laps();
            printf("  lap %d: %s\n", laps, LapTimer::format(timer.lastLap()).c_str());
            if (laps >= 1) break;
        }
    }
    printf("  max lateral %.2f g, max speed %.0f km/h, %.1f s with all wheels off, %d resets\n", maxLat, maxV,
           offTrack, resets);
    check(laps >= 1, "AI completes a lap");
    check(resets == 0, "AI lap without leaving the circuit");
    if (laps >= 1) check(timer.lastLap() > 380 && timer.lastLap() < 520, "lap time 6:20..8:40 (GT3 ~7:00)");
}
}  // namespace

void writeWav(const char* path, const std::vector<float>& stereo, int sr) {
    FILE* f = fopen(path, "wb");
    if (!f) return;
    auto u32 = [&](uint32_t v) { fwrite(&v, 4, 1, f); };
    auto u16 = [&](uint16_t v) { fwrite(&v, 2, 1, f); };
    uint32_t bytes = static_cast<uint32_t>(stereo.size() * 2);
    fwrite("RIFF", 1, 4, f);
    u32(36 + bytes);
    fwrite("WAVEfmt ", 1, 8, f);
    u32(16);
    u16(1);
    u16(2);
    u32(sr);
    u32(sr * 4);
    u16(4);
    u16(16);
    fwrite("data", 1, 4, f);
    u32(bytes);
    for (float v : stereo) {
        int16_t s = static_cast<int16_t>(std::clamp(v, -1.0f, 1.0f) * 32767.0f);
        fwrite(&s, 2, 1, f);
    }
    fclose(f);
}

// Drives a scripted sequence (idle, blips, full-throttle run, ABS-off braking, slalom) and renders
// the car audio to a WAV file so the synthesis can be checked without running the game.
void renderAudioPreview(const char* path, float interior) {
    TrackData track;
    track.createSynthetic(stadium(5000.0f, 300.0f, 2.0f), 8.0f, 2.0f, 40.0f);
    Car car;
    car.init(CarParams::gt3(), &track);
    car.setAutoShift(true);
    car.resetTo(5.0f, 0.0f);
    AIDriver ai;
    ai.init(&track);
    const int sr = 48000;
    CarAudio audio;
    audio.init(static_cast<float>(sr));
    CarAudioParams params;
    params.interior = interior;
    std::vector<float> wav;
    std::vector<float> block(480 * 2);
    const double total = 34.0;
    for (double t = 0; t < total; t += 0.01) {
        for (int k = 0; k < 10; ++k) {
            CarInput in = ai.update(car, kDt);
            double tt = t + k * kDt;
            if (tt < 3.0) {
                in.throttle = 0.0f;
            } else if (tt < 6.0) {
                double ph = std::fmod(tt - 3.0, 1.0);
                in.throttle = ph < 0.35 ? 1.0f : 0.0f;  // blips in neutral
            } else if (tt < 21.0) {
                in.throttle = 1.0f;
            } else if (tt < 25.5) {
                in.throttle = 0.0f;
                in.brake = car.speed() > 3 ? 1.0f : 0.0f;
                car.absEnabled = false;
            } else {
                car.absEnabled = true;
                in.throttle = 0.75f;
                in.steer += static_cast<float>(0.45 * std::sin((tt - 25.5) * 3.2));
            }
            if (tt < 6.0) car.drivetrain().setAutoShift(false);
            else car.drivetrain().setAutoShift(true);
            car.step(kDt, in);
        }
        fillCarAudioParams(car, params);
        audio.setParams(params);
        audio.render(block.data(), 480);
        wav.insert(wav.end(), block.begin(), block.end());
    }
    writeWav(path, wav, sr);
    float peak = 0;
    double rms = 0;
    for (float v : wav) {
        peak = std::max(peak, std::abs(v));
        rms += v * v;
    }
    printf("wrote %s (%.1f s, peak %.2f, rms %.3f)\n", path, wav.size() / 2.0 / sr, peak, std::sqrt(rms / wav.size()));
}

int main(int argc, char** argv) {
    paths::init();
    if (argc > 2 && strcmp(argv[1], "audio") == 0) {
        renderAudioPreview(argv[2], argc > 3 ? static_cast<float>(atof(argv[3])) : 0.0f);
        return 0;
    }
    bool quick = argc > 1 && strcmp(argv[1], "quick") == 0;
    TrackData straight;
    straight.createSynthetic(stadium(5000.0f, 300.0f, 2.0f), 8.0f, 2.0f, 40.0f);
    testStandstill(straight);
    testAcceleration(straight);
    testBraking(straight);
    testSteeringFeel(straight);
    testForceFeedbackSignal();
    testSkidpad(80.0f, 60.0f);
    testSkidpad(300.0f, 130.0f);
    if (!quick) testLap();
    printf("\n%s (%d failures)\n", g_failures ? "SOME TESTS FAILED" : "ALL TESTS PASSED", g_failures);
    return g_failures ? 1 : 0;
}
