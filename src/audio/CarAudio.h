#pragma once
#include <mutex>

#include "audio/Dsp.h"
#include "audio/EngineSynth.h"

namespace apex {

struct CarAudioParams {
    float rpm = 0.0f;
    float load = 0.0f;           // engine load / effective throttle 0..1
    bool limiter = false;
    float speed = 0.0f;          // m/s
    int gear = 0;
    int shiftCount = 0;          // increments on every completed shift
    bool lastShiftUp = true;
    float wheelSlip[4] = {};     // normalised combined slip (1 = peak grip)
    float wheelSlide[4] = {};    // m/s contact patch sliding speed
    float wheelLoad[4] = {};     // N
    float wheelLock[4] = {};     // 0..1 locked / heavy negative slip
    int surface[4] = {};         // Surface enum
    float impact = 0.0f;         // m/s
    float scrape = 0.0f;         // 0..1
    float interior = 1.0f;       // 1 cockpit, 0 outside
    bool paused = false;
    // mixer
    float master = 0.8f, engine = 1.0f, tyres = 0.8f, wind = 0.6f, effects = 0.8f;
};

// All car sounds, rendered on the audio thread. setParams() is called by the game thread.
class CarAudio {
public:
    void init(float sampleRate);
    void setParams(const CarAudioParams& p);
    void render(float* interleavedStereo, int frames);

private:
    std::mutex mutex_;
    CarAudioParams pending_;
    bool hasPending_ = false;
    CarAudioParams p_;

    float sr_ = 48000.0f;
    EngineSynth engine_;
    dsp::Smoother rpmS_, loadS_, speedS_, interiorS_, fadeS_;
    dsp::Smoother squealS_[4], lockS_[4], offroadS_[4], kerbS_[4];
    dsp::Biquad squealRes_[4], squealBody_[4], lockBand_[4], offroadLp_[4];
    dsp::Biquad roadLp_, roarBand_, windBand_, windLow_, scrapeBand_, crashRing_, crashLp_;
    dsp::Biquad whineBand_;
    dsp::Noise noise_{0xBEEFu};
    dsp::PinkNoise pink_{777}, pink2_{999};
    dsp::Reverb reverb_;
    float kerbPhase_[4] = {};
    float squealPhaseMod_[4] = {};
    float whinePhase_ = 0.0f;
    float clunkEnv_ = 0.0f, clunkPhase_ = 0.0f;
    float crashEnv_ = 0.0f;
    float lastImpact_ = 0.0f;
    int lastShiftCount_ = 0;
    float gravelEnv_[4] = {};
};

class Car;
// Fills the physics-derived fields of the audio parameters from the car state.
void fillCarAudioParams(const Car& car, CarAudioParams& p);

}  // namespace apex
