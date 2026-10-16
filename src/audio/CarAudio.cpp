#include "audio/CarAudio.h"

#include <algorithm>
#include <cmath>

#include "car/Car.h"

namespace apex {

namespace {
float smoothstep(float a, float b, float x) {
    float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}
bool paved(int surface) { return surface <= 2; }  // asphalt, kerb, concrete
constexpr int kKerb = 1;
}  // namespace

void CarAudio::init(float sampleRate) {
    sr_ = sampleRate;
    engine_.init(sr_);
    rpmS_.setTime(0.012f, sr_);
    loadS_.setTime(0.010f, sr_);
    speedS_.setTime(0.05f, sr_);
    interiorS_.setTime(0.15f, sr_);
    fadeS_.setTime(0.08f, sr_);
    for (int i = 0; i < 4; ++i) {
        squealS_[i].setTime(0.06f, sr_);
        lockS_[i].setTime(0.03f, sr_);
        offroadS_[i].setTime(0.05f, sr_);
        kerbS_[i].setTime(0.015f, sr_);
        squealRes_[i].bandpass(800.0f, 14.0f, sr_);
        squealBody_[i].bandpass(1900.0f, 0.8f, sr_);
        lockBand_[i].bandpass(420.0f, 2.5f, sr_);
        offroadLp_[i].lowpass(260.0f, 0.7f, sr_);
    }
    roadLp_.lowpass(140.0f, 0.8f, sr_);
    roarBand_.bandpass(850.0f, 0.9f, sr_);
    windBand_.bandpass(1100.0f, 0.5f, sr_);
    windLow_.lowpass(90.0f, 0.9f, sr_);
    scrapeBand_.bandpass(2800.0f, 1.5f, sr_);
    crashRing_.bandpass(1450.0f, 7.0f, sr_);
    crashLp_.lowpass(500.0f, 0.8f, sr_);
    whineBand_.bandpass(2200.0f, 4.0f, sr_);
    reverb_.init(sr_);
    reverb_.feedback = 0.78f;
    reverb_.damp = 0.45f;
}

void CarAudio::setParams(const CarAudioParams& p) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = p;
    hasPending_ = true;
}

void CarAudio::render(float* out, int frames) {
    {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (lock.owns_lock() && hasPending_) {
            p_ = pending_;
            hasPending_ = false;
        }
    }
    const float dt = 1.0f / sr_;

    // ---- block-rate events
    if (p_.shiftCount != lastShiftCount_) {
        lastShiftCount_ = p_.shiftCount;
        clunkEnv_ = 1.0f;
        clunkPhase_ = 0.0f;
        if (p_.lastShiftUp && p_.rpm > 4500.0f && noise_.white() > -0.4f) engine_.triggerPop(0.3f + 0.5f * (noise_.white() * 0.5f + 0.5f));
    }
    if (p_.impact > lastImpact_ + 1.5f) crashEnv_ = std::min(1.0f, 0.25f + p_.impact / 14.0f);
    lastImpact_ = p_.impact;

    float squealFreq[4];
    for (int w = 0; w < 4; ++w) {
        squealFreq[w] = 640.0f + 38.0f * std::min(p_.wheelSlide[w], 25.0f) + w * 37.0f;
        squealRes_[w].bandpass(squealFreq[w] * (1.0f + 0.03f * std::sin(squealPhaseMod_[w])), 14.0f, sr_);
        squealPhaseMod_[w] += frames * dt * 7.0f * (1.0f + 0.3f * w);
    }
    const float whineFreq = std::max(p_.rpm, 0.0f) / 60.0f * 19.0f;
    whineBand_.bandpass(std::clamp(whineFreq, 200.0f, 9000.0f), 6.0f, sr_);

    for (int i = 0; i < frames; ++i) {
        const float rpm = rpmS_.next(p_.rpm);
        const float load = loadS_.next(p_.load);
        const float speed = speedS_.next(p_.speed);
        const float interior = interiorS_.next(p_.interior);
        const float fade = fadeS_.next(p_.paused ? 0.0f : 1.0f);

        // ------------------------------------------------ engine
        float eL = 0, eR = 0;
        engine_.process(rpm, load, p_.limiter, interior, eL, eR);

        // sequential gearbox: clunk on every shift, straight-cut gear whine
        float clunk = clunkEnv_ * (std::sin(clunkPhase_) * 0.8f + noise_.white() * 0.25f * clunkEnv_);
        clunkPhase_ += 2.0f * dsp::kPi * 75.0f * dt;
        clunkEnv_ *= std::exp(-dt / 0.045f);
        whinePhase_ += 2.0f * dsp::kPi * whineFreq * dt;
        if (whinePhase_ > 2.0f * dsp::kPi) whinePhase_ -= 2.0f * dsp::kPi;
        float whineAmp = (p_.gear != 0 ? 1.0f : 0.0f) * std::min(speed / 15.0f, 1.0f) *
                         (0.010f + 0.030f * (1.0f - load)) * (0.35f + 0.65f * interior);
        float whine = (std::sin(whinePhase_) + 0.3f * whineBand_.process(noise_.white())) * whineAmp;

        // ------------------------------------------------ tyres
        float tyL = 0, tyR = 0;
        for (int w = 0; w < 4; ++w) {
            const bool pv = paved(p_.surface[w]);
            const float loadN = std::clamp(p_.wheelLoad[w] / 4000.0f, 0.0f, 1.6f);
            float sqT = pv ? smoothstep(0.80f, 1.40f, p_.wheelSlip[w]) * std::min(p_.wheelSlide[w] / 2.5f, 1.0f) * loadN : 0.0f;
            float sq = squealS_[w].next(sqT);
            float s = 0.0f;
            if (sq > 1e-4f) {
                float n = noise_.white();
                s += (squealRes_[w].process(n) * 2.2f + squealBody_[w].process(n) * 0.35f) * sq;
            }
            float lk = lockS_[w].next(pv ? p_.wheelLock[w] * std::min(speed / 8.0f, 1.0f) : 0.0f);
            if (lk > 1e-4f) s += lockBand_[w].process(noise_.white()) * lk * 0.9f;
            float off = offroadS_[w].next(!pv ? std::min(speed / 14.0f, 1.0f) : 0.0f);
            if (off > 1e-4f) {
                if (noise_.white() > 0.9992f - 0.003f * off) gravelEnv_[w] = 1.0f;
                gravelEnv_[w] *= std::exp(-dt / 0.012f);
                s += (offroadLp_[w].process(noise_.white()) * 0.9f + noise_.white() * gravelEnv_[w] * 0.35f) * off;
            }
            float kb = kerbS_[w].next(p_.surface[w] == kKerb ? std::min(speed / 8.0f, 1.0f) : 0.0f);
            if (kb > 1e-4f) {
                kerbPhase_[w] += speed / 0.6f * dt;
                kerbPhase_[w] -= std::floor(kerbPhase_[w]);
                float saw = kerbPhase_[w] < 0.5f ? 1.0f : -1.0f;
                s += (saw * 0.6f + std::sin(kerbPhase_[w] * 2.0f * dsp::kPi) * 0.5f) * kb * 0.55f;
            }
            if (w % 2 == 0) {
                tyL += s * 0.7f;
                tyR += s * 0.3f;
            } else {
                tyL += s * 0.3f;
                tyR += s * 0.7f;
            }
        }

        // ------------------------------------------------ road, wind
        const float sp = speed / 60.0f;
        float road = roadLp_.process(pink_.next()) * std::min(speed / 35.0f, 1.4f) * (0.5f + 0.8f * interior) * 0.55f;
        float roar = roarBand_.process(pink2_.next()) * std::pow(sp, 1.3f) * 0.22f;
        float windAmt = std::pow(std::min(speed / 70.0f, 1.5f), 2.2f);
        float wind = windBand_.process(pink_.next()) * windAmt * (0.55f - 0.25f * interior) +
                     windLow_.process(pink2_.next()) * windAmt * 0.5f;

        // ------------------------------------------------ contact effects
        float scrape = scrapeBand_.process(noise_.white()) * p_.scrape * (0.6f + 0.4f * std::abs(noise_.white()));
        float crash = 0.0f;
        if (crashEnv_ > 1e-4f) {
            float n = noise_.white();
            crash = (crashRing_.process(n) * 1.4f + crashLp_.process(n) * 1.2f) * crashEnv_;
            crashEnv_ *= std::exp(-dt / 0.22f);
        }

        // ------------------------------------------------ mix
        const float tyreVol = p_.tyres, fxVol = p_.effects;
        float L = eL * p_.engine + whine * p_.engine + tyL * tyreVol + (road + roar) * tyreVol + wind * p_.wind +
                  (scrape + crash + clunk * 0.5f) * fxVol;
        float R = eR * p_.engine + whine * p_.engine + tyR * tyreVol + (road + roar) * tyreVol + wind * p_.wind +
                  (scrape + crash + clunk * 0.5f) * fxVol;
        float revL = 0, revR = 0;
        float send = (eL + eR) * 0.5f * p_.engine + (tyL + tyR) * 0.3f * tyreVol;
        reverb_.process(send * 0.5f, revL, revR);
        const float wet = 0.05f + 0.22f * (1.0f - interior);
        L += revL * wet;
        R += revR * wet;
        const float g = p_.master * fade;
        out[2 * i] = dsp::softClip(L * g * 0.9f);
        out[2 * i + 1] = dsp::softClip(R * g * 0.9f);
    }
}

void fillCarAudioParams(const Car& car, CarAudioParams& p) {
    const Drivetrain& dt = car.drivetrain();
    p.rpm = static_cast<float>(dt.rpm());
    p.load = static_cast<float>(dt.effectiveThrottle());
    p.limiter = dt.limiterActive();
    p.speed = static_cast<float>(car.speed());
    p.gear = dt.gear();
    if (dt.lastShiftEvent() != p.shiftCount) {
        p.lastShiftUp = dt.gear() > 0 && car.appliedThrottle() > 0.2;
        p.shiftCount = dt.lastShiftEvent();
    }
    for (int i = 0; i < 4; ++i) {
        const Wheel& w = car.wheels()[i];
        p.wheelSlip[i] = static_cast<float>(w.combinedSlip);
        p.wheelSlide[i] = static_cast<float>(w.slideSpeed);
        p.wheelLoad[i] = static_cast<float>(w.load);
        p.wheelLock[i] = static_cast<float>(std::clamp((-w.slipX - 0.25) / 0.5, 0.0, 1.0)) * (w.contact ? 1.0f : 0.0f);
        p.surface[i] = w.contact ? static_cast<int>(w.surface) : 0;
        if (!w.contact) {
            p.wheelSlip[i] = 0.0f;
            p.wheelLoad[i] = 0.0f;
        }
    }
    p.impact = static_cast<float>(car.lastImpact());
    p.scrape = static_cast<float>(car.scrape());
}

}  // namespace apex
