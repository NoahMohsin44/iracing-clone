#pragma once
#include <string>
#include <vector>

#include "audio/CarAudio.h"
#include "car/Car.h"
#include "core/Input.h"
#include "core/Settings.h"
#include "game/AIDriver.h"
#include "game/CameraRig.h"
#include "game/LapTimer.h"
#include "gfx/Renderer.h"
#include "world/TrackData.h"

namespace apex {

// A practice session on the Nordschleife: fixed-rate physics (1 kHz), lap timing with
// four-wheels-off invalidation, assists, camera and audio parameter plumbing.
class Session {
public:
    static constexpr double kPhysicsDt = 1.0 / 1000.0;

    void start(const TrackData* track, Settings* settings, bool demo);
    void update(double frameDt, const DriverControls& in);
    void restart();
    void resetCarToTrack();
    void applyGameplaySettings();

    void fillVisuals(std::vector<CarVisual>& out, bool cockpitCamera) const;
    void fillAudio(CarAudioParams& p) const;

    Car& car() { return car_; }
    const Car& car() const { return car_; }
    const LapTimer& timer() const { return timer_; }
    double simTime() const { return simTime_; }
    bool demo() const { return demo_; }
    bool paused = false;
    CameraMode camera = CameraMode::Cockpit;
    CameraRig rig;
    bool useOverrideCamera = false;  // autotest / photo shots
    Camera overrideCamera;
    glm::vec3 paint{0.92f, 0.92f, 0.90f}, paint2{0.05f, 0.18f, 0.55f}, accent{0.95f, 0.35f, 0.02f};

    // HUD messages
    std::string message;
    double messageTime = 0.0;
    void flash(const std::string& m) {
        message = m;
        messageTime = 3.0;
    }
    int lastWheelsOff() const { return wheelsOff_; }
    // Steering column torque (Nm) of every physics step of the last update, for force feedback.
    const std::vector<float>& ffbSamples() const { return ffbSamples_; }

private:
    const TrackData* track_ = nullptr;
    Settings* settings_ = nullptr;
    Car car_;
    AIDriver ai_;
    LapTimer timer_;
    bool demo_ = false;
    double simTime_ = 0.0;
    double accumulator_ = 0.0;
    double offTrackTime_ = 0.0;
    double stuckTime_ = 0.0;
    int wheelsOff_ = 0;
    int lastLapEvent_ = 0;
    bool pendingShiftUp_ = false, pendingShiftDown_ = false;
    float prevSteerIn_ = 0.0f;
    std::vector<float> ffbSamples_;
};

}  // namespace apex
