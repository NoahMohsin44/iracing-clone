#pragma once
#include <atomic>
#include <cstddef>

namespace apex {

struct FfbConfig {
    bool enabled = true;
    float maxForceNm = 22.0f;       // steering column torque that drives the wheel at 100 %
    float minForce = 0.0f;          // output floor (0..0.3) that overcomes gear/belt friction
    float damping = 0.05f;          // wheel-side damper (0..1), applied by the device firmware
    bool invert = false;            // flip the device sign (for wheels with a reversed axis)
    bool softLock = true;           // stop the wheel at the car's steering lock
    float carLockDeg = 540.0f;      // car steering wheel lock-to-lock
    float wheelRotationDeg = 900.0f;  // physical wheel range (lock-to-lock)
    float physicsDt = 0.001f;
};

// Carries the steering column torque from the simulation to the force feedback device thread.
//
// The simulation produces 1 kHz samples in bursts (one frame's worth at a time); the device
// thread plays them back at a steady rate, with about one frame of latency, so the wheel gets
// the full detail (kerbs, bumps, the tyres letting go) instead of a staircase at the frame
// rate. The played-back torque is then shaped for the wheel: strength, minimum force, soft lock
// at the car's steering lock, and a fade in/out when the stream starts or stops (pause, menus).
class FfbSignal {
public:
    struct Output {
        float force = 0.0f;     // -1..1, + turns the wheel clockwise (device sign applied later)
        float torqueNm = 0.0f;  // played-back column torque
        bool clipping = false;  // the requested torque exceeds the wheel's range
        bool live = false;      // samples are arriving (driving)
        bool safetyCut = false; // output muted by the runaway guard
    };

    // ---- simulation thread (single producer)
    void push(const float* samples, int count);

    // ---- device thread (single consumer)
    // dt: real seconds since the previous tick. wheelDeg: physical wheel angle, NaN if unknown.
    Output tick(double dt, double wheelDeg, const FfbConfig& cfg);

    // ---- any thread
    void startTest() { testRequest_.store(true); }  // short push right, then left
    float clipFraction() const { return clipFraction_.load(); }
    float peakTorque() const { return peak_.load(); }  // highest sustained torque since resetPeak()
    void resetPeak() { resetPeak_.store(true); }
    size_t buffered() const { return head_.load() - tail_.load(); }
    // Counts runaway-guard trips: the wheel ran far past the soft lock, which usually means the
    // device pushes the opposite way to what we ask (needs the Invert option).
    int safetyTrips() const { return safetyTrips_.load(); }

private:
    static constexpr size_t kCapacity = 8192;  // power of two
    float ring_[kCapacity] = {};
    std::atomic<size_t> head_{0}, tail_{0};
    std::atomic<int> burst_{16};

    // consumer state
    double burstAvg_ = 16.0;
    double owed_ = 0.0;
    double idle_ = 1.0;
    double torque_ = 0.0;
    double fade_ = 0.0;
    double sustained_ = 0.0;
    double prevWheelDeg_ = 0.0, wheelVel_ = 0.0;
    bool havePrevWheel_ = false;
    double testTime_ = -1.0;
    double cutTime_ = 0.0;
    std::atomic<int> safetyTrips_{0};
    std::atomic<bool> testRequest_{false}, resetPeak_{false};
    std::atomic<float> clipFraction_{0.0f}, peak_{0.0f};
};

}  // namespace apex
