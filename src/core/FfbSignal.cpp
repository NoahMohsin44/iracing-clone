#include "core/FfbSignal.h"

#include <algorithm>
#include <cmath>

namespace apex {

void FfbSignal::push(const float* samples, int count) {
    if (count <= 0) return;
    size_t head = head_.load(std::memory_order_relaxed);
    const size_t tail = tail_.load(std::memory_order_acquire);
    for (int i = 0; i < count; ++i) {
        if (head - tail >= kCapacity) break;  // consumer stalled; drop the rest of this burst
        ring_[head & (kCapacity - 1)] = samples[i];
        ++head;
    }
    head_.store(head, std::memory_order_release);
    burst_.store(count, std::memory_order_relaxed);
}

FfbSignal::Output FfbSignal::tick(double dt, double wheelDeg, const FfbConfig& cfg) {
    Output out;
    dt = std::clamp(dt, 1e-4, 0.1);

    // ------------------------------------------------------------------ playback
    const size_t head = head_.load(std::memory_order_acquire);
    size_t tail = tail_.load(std::memory_order_relaxed);
    size_t avail = head - tail;
    burstAvg_ += (burst_.load(std::memory_order_relaxed) - burstAvg_) * std::min(1.0, dt * 4.0);
    // keep about one frame of samples buffered: enough to bridge the gap between bursts
    const double target = std::clamp(burstAvg_ + 2.0, 3.0, 120.0);
    if (static_cast<double>(avail) > target * 3.0 + 40.0) {
        // hitch (or the thread was descheduled): skip ahead instead of lagging behind
        tail = head - static_cast<size_t>(target);
        avail = static_cast<size_t>(target);
    }
    // play at the simulation rate, nudged faster/slower to hold the buffer at the target
    const double nominal = dt / std::max(1e-5, static_cast<double>(cfg.physicsDt));
    const double rate = std::clamp(1.0 + 0.5 * (static_cast<double>(avail) - target) / target, 0.25, 3.0);
    owed_ = std::min(owed_ + nominal * rate, nominal * 2.0 + 1.0);
    const size_t n = std::min(static_cast<size_t>(owed_), avail);
    if (n > 0) {
        double sum = 0.0;
        for (size_t i = 0; i < n; ++i) sum += ring_[(tail + i) & (kCapacity - 1)];
        tail += n;
        owed_ -= static_cast<double>(n);
        torque_ = sum / static_cast<double>(n);
        idle_ = 0.0;
    } else {
        idle_ += dt;
    }
    tail_.store(tail, std::memory_order_release);
    out.live = idle_ < 0.1;

    // fade out when the stream stops (pause, menus) and back in when driving resumes
    const double fadeTarget = (out.live && cfg.enabled) ? 1.0 : 0.0;
    fade_ += std::clamp(fadeTarget - fade_, -dt / 0.25, dt / 0.6);
    out.torqueNm = static_cast<float>(torque_);

    // ------------------------------------------------------------------ statistics
    if (resetPeak_.exchange(false)) {
        peak_.store(0.0f);
        sustained_ = 0.0;
    }
    if (out.live) {
        // 50 ms average so single kerb spikes don't set the auto strength
        sustained_ += (std::abs(torque_) - sustained_) * std::min(1.0, dt / 0.05);
        if (sustained_ > peak_.load()) peak_.store(static_cast<float>(sustained_));
    }

    // ------------------------------------------------------------------ shaping
    const double maxForce = std::max(1.0, static_cast<double>(cfg.maxForceNm));
    double x = torque_ / maxForce;
    out.clipping = out.live && std::abs(x) > 1.0;
    x = std::clamp(x, -1.0, 1.0);
    if (cfg.minForce > 0.0f && x != 0.0) {
        // lift small forces above the wheel's friction, ramping in so there is no notch at centre
        const double ramp = 0.01;
        const double ax = std::abs(x);
        double y = cfg.minForce + (1.0 - cfg.minForce) * ax;
        if (ax < ramp) y *= ax / ramp;
        x = std::copysign(y, x);
    }

    // soft lock: past the car's lock the wheel meets a stiff, damped stop
    if (std::isfinite(wheelDeg)) {
        if (havePrevWheel_) {
            const double v = (wheelDeg - prevWheelDeg_) / dt;
            wheelVel_ += (v - wheelVel_) * std::min(1.0, dt / 0.01);
        }
        prevWheelDeg_ = wheelDeg;
        havePrevWheel_ = true;
        const double half = 0.5 * cfg.carLockDeg;
        const double excess = std::abs(wheelDeg) - half;
        if (cfg.softLock && cfg.wheelRotationDeg > cfg.carLockDeg + 1.0f && excess > 0.0) {
            const double dir = wheelDeg > 0.0 ? 1.0 : -1.0;
            const double spring = std::min(1.0, excess / 2.5);
            const double damper = std::clamp(wheelVel_ * 0.004, -1.0, 1.0);
            // the stop overrides the car's torque (full push-back 2.5 deg past the lock)
            x = std::clamp(x + (-dir - x) * spring - damper, -1.0, 1.0);
            // Runaway guard: a wheel that keeps going far past a full-strength stop is being pushed
            // the wrong way by the device. Mute everything for a few seconds.
            if (excess > 45.0 && fade_ > 0.5 && cutTime_ <= 0.0) {
                cutTime_ = 3.0;
                safetyTrips_.fetch_add(1);
            }
        }
    } else {
        havePrevWheel_ = false;
    }
    x *= fade_;
    if (cutTime_ > 0.0) {
        cutTime_ -= dt;
        x = 0.0;
        out.safetyCut = true;
    }

    // ------------------------------------------------------------------ direction test
    if (testRequest_.exchange(false)) testTime_ = 0.0;
    if (testTime_ >= 0.0) {
        testTime_ += dt;
        const double t = testTime_;
        x = t < 0.6 ? 0.3 : (t < 1.2 ? -0.3 : 0.0);
        x *= std::min(1.0, t / 0.05) * std::clamp((1.25 - t) / 0.05, 0.0, 1.0);
        if (t > 1.3) testTime_ = -1.0;
    }

    out.force = static_cast<float>(x);
    const float clipNow = out.clipping ? 1.0f : 0.0f;
    if (out.live) clipFraction_.store(clipFraction_.load() + (clipNow - clipFraction_.load()) * static_cast<float>(std::min(1.0, dt / 2.0)));
    return out;
}

}  // namespace apex
