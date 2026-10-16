#include "audio/EngineSynth.h"

#include <algorithm>
#include <cmath>

namespace apex {

namespace {
// Crossplane V8 (firing order 1-8-4-3-6-5-7-2, odd cylinders on the left bank):
// bank receiving each of the 8 evenly spaced firings in a 720 degree cycle.
constexpr int kBankOfFiring[8] = {0, 1, 1, 0, 1, 0, 0, 1};
}  // namespace

void EngineSynth::init(float sampleRate) {
    sr_ = sampleRate;
    const float pipeLen[2] = {1.85f, 2.02f};  // metres, slightly unequal banks
    const float soundSpeed = 470.0f;            // hot exhaust gas
    for (int b = 0; b < 2; ++b) {
        Bank& bank = banks_[b];
        bank.pipe.init(static_cast<size_t>(sr_ * 0.03f));
        bank.pipeDelay = 2.0f * pipeLen[b] / soundSpeed * sr_;
        bank.loopLp.setLowpass(3200.0f, sr_);
        bank.roughLp.setLowpass(2600.0f, sr_);
        bank.body.peak(165.0f + b * 12.0f, 0.9f, 5.0f, sr_);
        bank.mid.peak(1150.0f, 1.2f, 2.5f, sr_);
        bank.bite.peak(2900.0f, 1.4f, 1.5f, sr_);
    }
    dsp::Noise r(777);
    for (float& v : cylVar_) v = 0.9f + 0.2f * (r.white() * 0.5f + 0.5f);
    intakeRes_.bandpass(230.0f, 1.8f, sr_);
    intakeNoise_.bandpass(1350.0f, 0.9f, sr_);
    mechBand_.bandpass(3600.0f, 0.8f, sr_);
    dcBlockL_.highpass(28.0f, 0.707f, sr_);
    dcBlockR_.highpass(28.0f, 0.707f, sr_);
    for (int i = 0; i < 2; ++i) {
        interiorLp_[i].lowpass(1900.0f, 0.6f, sr_);
        exteriorTilt_[i].peak(4200.0f, 0.7f, 2.0f, sr_);
    }
    envFollow_.setLowpass(30.0f, sr_);
    crankDeg_ = 0.0f;
    nextFireDeg_ = 0.0f;
    baseFireDeg_ = 0.0f;
    nextFiring_ = 0;
}

void EngineSynth::spawn(Bank& b, float amp, float tau, float rough) {
    Pulse* slot = &b.pulses[0];
    for (auto& p : b.pulses) {
        if (!p.active) {
            slot = &p;
            break;
        }
        if (p.age / p.tau > slot->age / slot->tau) slot = &p;  // recycle the oldest
    }
    slot->active = true;
    slot->age = 0.0f;
    slot->tau = tau;
    slot->amp = amp;
    slot->rough = rough;
}

float EngineSynth::bankSample(Bank& b, float dt) {
    const float n = b.roughLp.lp(noise_.white()) * 2.2f;
    float s = 0.0f;
    for (auto& p : b.pulses) {
        if (!p.active) continue;
        float x = p.age / p.tau;
        float env = x * std::exp(1.0f - x);
        s += p.amp * env * (1.0f + p.rough * n);
        p.age += dt;
        if (x > 9.0f) p.active = false;
    }
    b.collector = s;
    // exhaust pipe: open-ended waveguide (inverted reflection, damped in the loop)
    float fb = b.loopLp.lp(b.pipe.tap(b.pipeDelay));
    float y = s - 0.58f * fb;
    b.pipe.push(y);
    y = b.body.process(y);
    y = b.mid.process(y);
    y = b.bite.process(y);
    return y;
}

void EngineSynth::process(float rpm, float load, bool limiter, float interior, float& outL, float& outR) {
    const float dt = 1.0f / sr_;
    rpm = std::max(rpm, 0.0f);
    load = std::clamp(load, 0.0f, 1.0f);

    // ---------------------------------------------------------------- firing events
    crankDeg_ += rpm * 6.0f * dt;
    while (crankDeg_ >= nextFireDeg_) {
        const int k = nextFiring_;
        nextFiring_ = (nextFiring_ + 1) % 8;
        baseFireDeg_ += 90.0f;
        nextFireDeg_ = baseFireDeg_ + 0.8f * noise_.white();  // slight cycle-to-cycle variation
        if (rpm < 120.0f) continue;

        float amp = cylVar_[k] * (0.24f + 0.76f * std::pow(load, 0.75f)) * (1.0f + 0.07f * noise_.white());
        float tau = std::max(0.00022f, 19.0f / (rpm * 6.0f));
        float rough = 0.22f + 0.35f * load;
        if (limiter && noise_.white() > -0.35f) amp *= 0.04f;  // fuel cut: most firings missing
        // overrun: unburnt fuel igniting in the hot exhaust
        if (load < 0.08f && rpm > 3200.0f) {
            float p = 0.028f * (rpm - 3200.0f) / 4800.0f;
            if (noise_.white() * 0.5f + 0.5f < p) {
                amp = 1.6f + 1.2f * (noise_.white() * 0.5f + 0.5f);
                tau *= 2.6f;
                rough = 1.1f;
            }
        }
        if (pendingPop_ > 0.0f) {
            amp = 1.5f + 2.0f * pendingPop_;
            tau *= 3.0f;
            rough = 1.2f;
            pendingPop_ = 0.0f;
        }
        spawn(banks_[kBankOfFiring[k]], amp, tau, rough);
    }
    if (crankDeg_ >= 720.0f) {
        crankDeg_ -= 720.0f;
        nextFireDeg_ -= 720.0f;
        baseFireDeg_ -= 720.0f;
    }

    // ---------------------------------------------------------------- exhaust
    float left = bankSample(banks_[0], dt);
    float right = bankSample(banks_[1], dt);
    float raw = banks_[0].collector + banks_[1].collector;
    float env = envFollow_.lp(std::abs(raw));

    const float drive = 0.9f + 1.6f * load;
    float exL = dsp::softClip((left * 0.78f + right * 0.30f) * drive * 0.55f);
    float exR = dsp::softClip((right * 0.78f + left * 0.30f) * drive * 0.55f);

    // ---------------------------------------------------------------- intake + mechanical
    float rpmN = std::min(rpm / 8000.0f, 1.2f);
    float intake = intakeRes_.process(raw) * (0.35f + 0.65f * load) * 0.9f;
    intake += intakeNoise_.process(pink_.next()) * env * (0.2f + 1.3f * load) * rpmN * 1.6f;
    float mech = mechBand_.process(noise_.white()) * rpmN * rpmN * 0.09f;

    // ---------------------------------------------------------------- interior / exterior perspective
    float inL = interiorLp_[0].process(exL), inR = interiorLp_[1].process(exR);
    float outExtL = exteriorTilt_[0].process(exL), outExtR = exteriorTilt_[1].process(exR);
    float mixL = outExtL * (1.0f - interior) + inL * interior * 0.85f;
    float mixR = outExtR * (1.0f - interior) + inR * interior * 0.85f;
    float extra = intake * (0.35f + 0.65f * interior) + mech * (0.4f + 0.6f * interior);
    outL = dcBlockL_.process(mixL + extra) * 0.55f;
    outR = dcBlockR_.process(mixR + extra) * 0.55f;
    lastRpm_ = rpm;
}

}  // namespace apex
