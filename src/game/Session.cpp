#include "game/Session.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace apex {

void Session::start(const TrackData* track, Settings* settings, bool demo) {
    track_ = track;
    settings_ = settings;
    demo_ = demo;
    car_.init(CarParams::gt3(), track);
    ai_.init(track, demo ? 0.93f : 0.96f);
    timer_.init(track->length());
    camera = demo ? CameraMode::Orbit : static_cast<CameraMode>(std::clamp(settings->gameplay.defaultCamera, 0, 3));
    restart();
}

void Session::applyGameplaySettings() {
    const auto& g = settings_->gameplay;
    car_.absEnabled = g.absEnabled;
    car_.tcLevel = g.tcLevel;
    car_.brakeBias = g.brakeBias;
    car_.setAutoClutch(demo_ || g.autoClutch);
    car_.setAutoShift(demo_ || g.autoShift);
}

void Session::restart() {
    // grid spot on the T13 straight just before the line (standing start)
    const float s = demo_ ? 600.0f : track_->length() - 160.0f;
    car_.resetTo(s, demo_ ? track_->racingLineAt(s) : 2.0f);
    applyGameplaySettings();
    timer_.reset();
    simTime_ = 0.0;
    accumulator_ = 0.0;
    offTrackTime_ = 0.0;
    stuckTime_ = 0.0;
    paused = false;
    rig.reset();
    if (!demo_) flash("Practice - Nordschleife. Engine running, select 1st gear and go!");
}

void Session::resetCarToTrack() {
    float s = car_.trackLocation().s;
    car_.resetTo(s, track_->racingLineAt(s));
    applyGameplaySettings();
    timer_.invalidate();
    rig.reset();
    flash("Car reset - lap invalidated");
}

void Session::update(double frameDt, const DriverControls& in) {
    if (messageTime > 0.0) messageTime -= frameDt;
    if (paused) return;

    if (!demo_) {
        auto& g = settings_->gameplay;
        if (in.absToggle) {
            g.absEnabled = !g.absEnabled;
            flash(g.absEnabled ? "ABS on" : "ABS off");
        }
        if (in.tcToggle) {
            g.tcLevel = (g.tcLevel + 1) % 7;
            char b[32];
            snprintf(b, sizeof(b), "TC %d", g.tcLevel);
            flash(g.tcLevel ? b : "TC off");
        }
        if (in.biasFwd) g.brakeBias = std::min(0.70f, g.brakeBias + 0.005f);
        if (in.biasRwd) g.brakeBias = std::max(0.45f, g.brakeBias - 0.005f);
        if (in.reset) resetCarToTrack();
        applyGameplaySettings();
        // shift requests are latched until the next physics step consumes them
        pendingShiftUp_ |= in.shiftUp;
        pendingShiftDown_ |= in.shiftDown;
    }

    accumulator_ += std::min(frameDt, 0.1);
    int steps = 0;
    while (accumulator_ >= kPhysicsDt && steps < 120) {
        CarInput ci;
        if (demo_) {
            ci = ai_.update(car_, kPhysicsDt);
        } else {
            ci.steer = in.steer;
            ci.throttle = in.throttle;
            ci.brake = in.brake;
            ci.clutch = in.clutch;
            ci.shiftUp = pendingShiftUp_;
            ci.shiftDown = pendingShiftDown_;
            pendingShiftUp_ = pendingShiftDown_ = false;
        }
        car_.step(kPhysicsDt, ci);  // the car detects shift edges itself
        simTime_ += kPhysicsDt;
        accumulator_ -= kPhysicsDt;
        timer_.update(simTime_, car_.trackLocation().s);
        ++steps;

        int off = 0;
        for (const auto& w : car_.wheels())
            if (w.contact && (w.surface == Surface::Grass || w.surface == Surface::Dirt || w.surface == Surface::Gravel)) ++off;
        wheelsOff_ = off;
        if (off == 4) {
            offTrackTime_ += kPhysicsDt;
            if (offTrackTime_ > 0.25 && timer_.currentValid() && timer_.timing() && !demo_) {
                timer_.invalidate();
                flash("4 wheels off - lap invalid");
            }
        } else {
            offTrackTime_ = 0.0;
        }
    }
    if (car_.needsReset()) {
        float s = car_.trackLocation().s;
        car_.resetTo(s, track_->racingLineAt(s));
        applyGameplaySettings();
        timer_.invalidate();
        rig.reset();
        if (!demo_) flash("Recovered to track");
    }
    if (demo_) {
        // keep the demo running forever: recover if the AI gets stuck or turned around
        glm::dvec3 tan(track_->tangentAt(car_.trackLocation().s));
        bool wrongWay = glm::dot(car_.forward(), tan) < 0.2;
        stuckTime_ = (car_.speed() < 2.0 || wrongWay) ? stuckTime_ + frameDt : 0.0;
        if (stuckTime_ > 3.0) {
            float s = car_.trackLocation().s + 20.0f;
            car_.resetTo(s, track_->racingLineAt(s));
            applyGameplaySettings();
            stuckTime_ = 0.0;
        }
    }
    if (timer_.lapCompletedEvent() != lastLapEvent_) {
        lastLapEvent_ = timer_.lapCompletedEvent();
        if (!demo_) {
            std::string m = "Lap " + std::to_string(timer_.laps()) + ": " + LapTimer::format(timer_.lastLap());
            if (!timer_.lastValid()) m += " (invalid)";
            else if (timer_.lastLap() <= timer_.bestLap() + 1e-6) m += "  - personal best!";
            flash(m);
        }
    }
}

void Session::fillVisuals(std::vector<CarVisual>& out, bool cockpitCamera) const {
    CarVisual v;
    v.position = car_.position();
    v.orientation = car_.orientation();
    for (int i = 0; i < 4; ++i) {
        const Wheel& w = car_.wheels()[i];
        v.wheelCentre[i] = w.centre;
        v.wheelSpin[i] = w.spinAngle;
        v.wheelSteer[i] = w.steerAngle;
        v.brakeGlow[i] = static_cast<float>(std::clamp((w.brakeTemp - 380.0) / 450.0, 0.0, 1.0));
    }
    v.steeringWheelAngle = static_cast<float>(glm::radians(car_.steeringWheelDeg()));
    v.brakeLight = car_.appliedBrake() > 0.05 ? 1.0f : 0.0f;
    const auto& e = car_.params().engine;
    v.rpmFrac = static_cast<float>(car_.drivetrain().rpm() / e.limiterRpm);
    v.paint = paint;
    v.paint2 = paint2;
    v.accent = accent;
    v.hideDriver = cockpitCamera;
    out.clear();
    out.push_back(v);
}

void Session::fillAudio(CarAudioParams& p) const {
    fillCarAudioParams(car_, p);
    p.paused = paused;
}

}  // namespace apex
