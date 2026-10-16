#pragma once
#include "audio/Dsp.h"

namespace apex {

// Physically-inspired engine sound: individual exhaust pulses from each cylinder's firing event
// (crossplane V8 firing order -> uneven pulses per bank = the characteristic burble), fed through
// per-bank exhaust waveguides, plus intake roar and mechanical noise. Overrun crackle, limiter
// cut and shift pops come out of the same pulse model.
class EngineSynth {
public:
    void init(float sampleRate);
    // Per-sample parameters (smoothed by the caller): rpm, load 0..1, limiter active.
    void process(float rpm, float load, bool limiter, float interior, float& outL, float& outR);
    void triggerPop(float strength) { pendingPop_ = strength; }

private:
    struct Pulse {
        float age = 0.0f;
        float tau = 0.001f;
        float amp = 0.0f;
        float rough = 0.3f;
        bool active = false;
    };
    struct Bank {
        Pulse pulses[8];
        dsp::Delay pipe;
        float pipeDelay = 380.0f;
        dsp::OnePole loopLp;
        dsp::OnePole roughLp;
        dsp::Biquad body, mid, bite;
        float collector = 0.0f;
    };
    void spawn(Bank& b, float amp, float tau, float rough);
    float bankSample(Bank& b, float dt);

    float sr_ = 48000.0f;
    float crankDeg_ = 0.0f;
    float nextFireDeg_ = 0.0f;
    float baseFireDeg_ = 0.0f;
    int nextFiring_ = 0;
    Bank banks_[2];
    float cylVar_[8] = {};
    dsp::Noise noise_{0xC0FFEEu};
    dsp::PinkNoise pink_{4242};
    dsp::Biquad intakeRes_, intakeNoise_, mechBand_, dcBlockL_, dcBlockR_, interiorLp_[2], exteriorTilt_[2];
    dsp::OnePole envFollow_;
    float pendingPop_ = 0.0f;
    float lastRpm_ = 0.0f;
    float intakeEnv_ = 0.0f;
};

}  // namespace apex
