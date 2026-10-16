#include "core/ForceFeedback.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <mutex>
#include <thread>

#include "core/Log.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <mmsystem.h>
#ifndef DIRECTINPUT_VERSION
#define DIRECTINPUT_VERSION 0x0800
#endif
#include <dinput.h>

#include <GLFW/glfw3.h>
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#endif

namespace apex {

#ifdef _WIN32
// ============================================================================ DirectInput backend
namespace {

std::string utf8(const wchar_t* w) {
    const int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
    if (n <= 1) return {};
    std::string s(static_cast<size_t>(n), '\0');
    WideCharToMultiByte(CP_UTF8, 0, w, -1, s.data(), n, nullptr, nullptr);
    s.resize(static_cast<size_t>(n - 1));
    return s;
}

// The GUID string GLFW (and SDL) derive for DirectInput devices, so a wheel can be matched with
// the joystick the steering axis is bound to.
std::string glfwStyleGuid(const DIDEVICEINSTANCEW& d) {
    char g[64] = {};
    if (std::memcmp(&d.guidProduct.Data4[2], "PIDVID", 6) == 0) {
        const uint32_t v = d.guidProduct.Data1;
        snprintf(g, sizeof(g), "03000000%02x%02x0000%02x%02x000000000000", v & 0xffu, (v >> 8) & 0xffu,
                 (v >> 16) & 0xffu, (v >> 24) & 0xffu);
    } else {
        char name[16] = {};
        const std::string n = utf8(d.tszInstanceName);
        std::memcpy(name, n.data(), std::min<size_t>(n.size(), 11));
        snprintf(g, sizeof(g), "05000000%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x00", (uint8_t)name[0],
                 (uint8_t)name[1], (uint8_t)name[2], (uint8_t)name[3], (uint8_t)name[4], (uint8_t)name[5],
                 (uint8_t)name[6], (uint8_t)name[7], (uint8_t)name[8], (uint8_t)name[9], (uint8_t)name[10]);
    }
    return g;
}

struct DeviceEntry {
    GUID instance;
    std::string name, guid;
};

BOOL CALLBACK enumDevicesCallback(const DIDEVICEINSTANCEW* d, void* user) {
    auto* list = static_cast<std::vector<DeviceEntry>*>(user);
    list->push_back({d->guidInstance, utf8(d->tszInstanceName), glfwStyleGuid(*d)});
    return DIENUM_CONTINUE;
}

constexpr DWORD kNoAxis = 0xffffffffu;

struct AxisEnum {
    IDirectInputDevice8W* dev = nullptr;
    std::vector<std::pair<int, DWORD>> axes;  // (0 axis / 1 slider, offset in DIJOYSTATE2)
    int sliders = 0;
    DWORD ffAxis = kNoAxis;
};

// Mirrors GLFW's object enumeration so GLFW axis indices map to the same DirectInput axes.
BOOL CALLBACK enumAxesCallback(const DIDEVICEOBJECTINSTANCEW* o, void* user) {
    auto* e = static_cast<AxisEnum*>(user);
    DWORD ofs = 0;
    bool slider = false;
    if (o->guidType == GUID_Slider) {
        if (e->sliders >= 2) return DIENUM_CONTINUE;
        ofs = static_cast<DWORD>(DIJOFS_SLIDER(e->sliders));
        slider = true;
    } else if (o->guidType == GUID_XAxis) ofs = DIJOFS_X;
    else if (o->guidType == GUID_YAxis) ofs = DIJOFS_Y;
    else if (o->guidType == GUID_ZAxis) ofs = DIJOFS_Z;
    else if (o->guidType == GUID_RxAxis) ofs = DIJOFS_RX;
    else if (o->guidType == GUID_RyAxis) ofs = DIJOFS_RY;
    else if (o->guidType == GUID_RzAxis) ofs = DIJOFS_RZ;
    else return DIENUM_CONTINUE;

    DIPROPRANGE r = {};
    r.diph.dwSize = sizeof(r);
    r.diph.dwHeaderSize = sizeof(r.diph);
    r.diph.dwObj = o->dwType;
    r.diph.dwHow = DIPH_BYID;
    r.lMin = -32768;
    r.lMax = 32767;
    if (FAILED(e->dev->SetProperty(DIPROP_RANGE, &r.diph))) return DIENUM_CONTINUE;
    if (slider) ++e->sliders;
    e->axes.push_back({slider ? 1 : 0, ofs});
    if ((o->dwFlags & DIDOI_FFACTUATOR) && (e->ffAxis == kNoAxis || ofs == DIJOFS_X)) e->ffAxis = ofs;
    return DIENUM_CONTINUE;
}

bool lostDevice(HRESULT hr) {
    return hr == static_cast<HRESULT>(DIERR_INPUTLOST) || hr == static_cast<HRESULT>(DIERR_NOTACQUIRED) ||
           hr == static_cast<HRESULT>(DIERR_NOTEXCLUSIVEACQUIRED);
}

}  // namespace

struct ForceFeedback::Impl {
    HWND hwnd = nullptr;
    IDirectInput8W* di = nullptr;
    IDirectInputDevice8W* dev = nullptr;
    IDirectInputEffect* constant = nullptr;
    IDirectInputEffect* damper = nullptr;
    DWORD ffAxis = DIJOFS_X;
    std::vector<DWORD> axisOffsets;  // GLFW axis index -> DIJOYSTATE2 offset
    bool matchedBinding = false;
    std::string deviceName;

    FfbSignal signal;
    std::thread thread;
    std::atomic<bool> running{false};

    std::mutex cfgMutex;
    FfbConfig cfg;
    FfbDeviceHint hint;
    std::string openedFor = "\x01";  // hint GUID the device was opened for ("\x01" = never tried)
    bool rescan = false;

    std::atomic<bool> acquired{false};
    std::atomic<float> output{0.0f}, torque{0.0f};
    std::atomic<bool> clipping{false}, live{false};

    bool open(const FfbDeviceHint& h) {
        std::vector<DeviceEntry> list;
        di->EnumDevices(DI8DEVCLASS_GAMECTRL, enumDevicesCallback, &list, DIEDFL_ATTACHEDONLY | DIEDFL_FORCEFEEDBACK);
        if (list.empty()) {
            LOGI("Force feedback: no force feedback controllers found");
            return false;
        }
        const DeviceEntry* pick = nullptr;
        for (const auto& d : list)
            if (!h.guid.empty() && d.guid == h.guid) pick = &d;
        if (!pick)
            for (const auto& d : list)
                if (!h.name.empty() && d.name == h.name) pick = &d;
        matchedBinding = pick != nullptr;
        if (!pick) pick = &list.front();

        if (FAILED(di->CreateDevice(pick->instance, &dev, nullptr))) {
            LOGW("Force feedback: could not open %s", pick->name.c_str());
            dev = nullptr;
            return false;
        }
        if (FAILED(dev->SetDataFormat(&c_dfDIJoystick2)) ||
            FAILED(dev->SetCooperativeLevel(hwnd, DISCL_EXCLUSIVE | DISCL_FOREGROUND))) {
            LOGW("Force feedback: could not configure %s", pick->name.c_str());
            close();
            return false;
        }
        // the game provides all forces: turn off the driver's centring spring
        DIPROPDWORD ac = {};
        ac.diph.dwSize = sizeof(ac);
        ac.diph.dwHeaderSize = sizeof(ac.diph);
        ac.diph.dwObj = 0;
        ac.diph.dwHow = DIPH_DEVICE;
        ac.dwData = DIPROPAUTOCENTER_OFF;
        dev->SetProperty(DIPROP_AUTOCENTER, &ac.diph);

        AxisEnum e;
        e.dev = dev;
        dev->EnumObjects(enumAxesCallback, &e, DIDFT_AXIS);
        std::sort(e.axes.begin(), e.axes.end());
        axisOffsets.clear();
        for (const auto& a : e.axes) axisOffsets.push_back(a.second);
        ffAxis = e.ffAxis != kNoAxis ? e.ffAxis : DIJOFS_X;
        deviceName = pick->name;
        LOGI("Force feedback: using %s (%s)%s", pick->name.c_str(), pick->guid.c_str(),
             matchedBinding ? "" : " - not the bound steering controller");
        return true;
    }

    void createEffects() {
        DWORD axes[1] = {ffAxis};
        LONG dir[1] = {0};
        DICONSTANTFORCE cf = {0};
        DIEFFECT eff = {};
        eff.dwSize = sizeof(DIEFFECT);
        eff.dwFlags = DIEFF_CARTESIAN | DIEFF_OBJECTOFFSETS;
        eff.dwDuration = INFINITE;
        eff.dwSamplePeriod = 0;
        eff.dwGain = DI_FFNOMINALMAX;
        eff.dwTriggerButton = DIEB_NOTRIGGER;
        eff.dwTriggerRepeatInterval = 0;
        eff.cAxes = 1;
        eff.rgdwAxes = axes;
        eff.rglDirection = dir;
        eff.lpEnvelope = nullptr;
        eff.cbTypeSpecificParams = sizeof(DICONSTANTFORCE);
        eff.lpvTypeSpecificParams = &cf;
        eff.dwStartDelay = 0;
        if (!constant && FAILED(dev->CreateEffect(GUID_ConstantForce, &eff, &constant, nullptr))) {
            LOGW("Force feedback: constant force effect not supported");
            constant = nullptr;
        }
        DICONDITION cond = {};
        cond.dwPositiveSaturation = cond.dwNegativeSaturation = DI_FFNOMINALMAX;
        eff.cbTypeSpecificParams = sizeof(DICONDITION);
        eff.lpvTypeSpecificParams = &cond;
        if (!damper && FAILED(dev->CreateEffect(GUID_Damper, &eff, &damper, nullptr))) damper = nullptr;
    }

    void close() {
        if (constant) {
            constant->Stop();
            constant->Release();
            constant = nullptr;
        }
        if (damper) {
            damper->Stop();
            damper->Release();
            damper = nullptr;
        }
        if (dev) {
            dev->Unacquire();
            dev->Release();
            dev = nullptr;
        }
        acquired = false;
        deviceName.clear();
    }

    double wheelDegrees(const DIJOYSTATE2& js, const FfbConfig& c, const FfbDeviceHint& h) const {
        DWORD ofs = DIJOFS_X;
        float rest = 0.0f, full = 1.0f;
        if (matchedBinding && h.axis >= 0 && h.axis < static_cast<int>(axisOffsets.size())) {
            ofs = axisOffsets[static_cast<size_t>(h.axis)];
            rest = h.rest;
            full = h.full;
        }
        if (ofs + sizeof(LONG) > 8 * sizeof(LONG)) return std::numeric_limits<double>::quiet_NaN();
        LONG raw = 0;
        std::memcpy(&raw, reinterpret_cast<const char*>(&js) + ofs, sizeof(LONG));
        const float v = (static_cast<float>(raw) + 0.5f) / 32767.5f;  // GLFW's normalisation
        const float span = full - rest;
        if (std::abs(span) < 0.05f) return std::numeric_limits<double>::quiet_NaN();
        // same mapping as Input::update for a wheel: deadzone, linearity, then 1:1 degrees
        const float n = std::clamp((v - rest) / span, -1.0f, 1.0f);
        const float dz = std::clamp(h.deadzone, 0.0f, 0.5f);
        const float a = std::max(0.0f, (std::abs(n) - dz) / (1.0f - dz));
        const float shaped = std::copysign(std::pow(a, std::max(0.1f, h.linearity)), n);
        return static_cast<double>(shaped) * c.wheelRotationDeg * 0.5;
    }

    void run() {
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
        timeBeginPeriod(1);
        using clock = std::chrono::steady_clock;
        auto prev = clock::now();
        double sinceAcquire = 1.0, sinceSend = 0.0;
        LONG lastMag = std::numeric_limits<LONG>::min();
        LONG lastDamp = -1;
        while (running.load()) {
            Sleep(1);
            const auto now = clock::now();
            const double dt = std::chrono::duration<double>(now - prev).count();
            prev = now;
            FfbConfig c;
            FfbDeviceHint h;
            {
                std::lock_guard<std::mutex> lk(cfgMutex);
                c = cfg;
                h = hint;
            }

            // (re)acquire: exclusive + foreground, so we lose the wheel when the window loses focus
            if (!acquired.load()) {
                sinceAcquire += dt;
                if (sinceAcquire > 0.3) {
                    sinceAcquire = 0.0;
                    if (SUCCEEDED(dev->Acquire())) {
                        createEffects();
                        if (constant) constant->Start(1, 0);
                        if (damper) damper->Start(1, 0);
                        acquired = true;
                        lastMag = std::numeric_limits<LONG>::min();
                        lastDamp = -1;
                    }
                }
            }

            double wheelDeg = std::numeric_limits<double>::quiet_NaN();
            if (acquired.load()) {
                dev->Poll();
                DIJOYSTATE2 js = {};
                const HRESULT hr = dev->GetDeviceState(sizeof(js), &js);
                if (lostDevice(hr)) acquired = false;
                else if (SUCCEEDED(hr)) wheelDeg = wheelDegrees(js, c, h);
            }

            const FfbSignal::Output o = signal.tick(dt, wheelDeg, c);
            output = o.force;
            torque = o.torqueNm;
            clipping = o.clipping;
            live = o.live;

            if (!acquired.load()) continue;
            // DirectInput: a positive magnitude pushes the wheel towards -X (counter-clockwise)
            const LONG mag = static_cast<LONG>(std::lround(std::clamp(o.force, -1.0f, 1.0f) * DI_FFNOMINALMAX)) *
                             (c.invert ? 1 : -1);
            sinceSend += dt;
            if (constant && mag != lastMag && (sinceSend >= 0.0019 || mag == 0)) {
                DICONSTANTFORCE cf = {mag};
                DIEFFECT eff = {};
                eff.dwSize = sizeof(DIEFFECT);
                eff.dwFlags = DIEFF_CARTESIAN | DIEFF_OBJECTOFFSETS;
                eff.cbTypeSpecificParams = sizeof(cf);
                eff.lpvTypeSpecificParams = &cf;
                const HRESULT hr = constant->SetParameters(&eff, DIEP_TYPESPECIFICPARAMS);
                if (lostDevice(hr)) {
                    acquired = false;
                    continue;
                }
                lastMag = mag;
                sinceSend = 0.0;
            }
            const LONG damp = (o.live && c.enabled) ? static_cast<LONG>(std::clamp(c.damping, 0.0f, 1.0f) * DI_FFNOMINALMAX) : 0;
            if (damper && damp != lastDamp) {
                DICONDITION cond = {};
                cond.lPositiveCoefficient = cond.lNegativeCoefficient = damp;
                cond.dwPositiveSaturation = cond.dwNegativeSaturation = DI_FFNOMINALMAX;
                DIEFFECT eff = {};
                eff.dwSize = sizeof(DIEFFECT);
                eff.dwFlags = DIEFF_CARTESIAN | DIEFF_OBJECTOFFSETS;
                eff.cbTypeSpecificParams = sizeof(cond);
                eff.lpvTypeSpecificParams = &cond;
                if (SUCCEEDED(damper->SetParameters(&eff, DIEP_TYPESPECIFICPARAMS))) lastDamp = damp;
            }
        }
        // leave the wheel limp
        if (acquired.load() && constant) {
            DICONSTANTFORCE cf = {0};
            DIEFFECT eff = {};
            eff.dwSize = sizeof(DIEFFECT);
            eff.dwFlags = DIEFF_CARTESIAN | DIEFF_OBJECTOFFSETS;
            eff.cbTypeSpecificParams = sizeof(cf);
            eff.lpvTypeSpecificParams = &cf;
            constant->SetParameters(&eff, DIEP_TYPESPECIFICPARAMS);
        }
        timeEndPeriod(1);
    }

    void start() {
        running = true;
        thread = std::thread([this] { run(); });
    }

    void stop() {
        if (running.exchange(false) && thread.joinable()) thread.join();
        close();
    }
};

ForceFeedback::ForceFeedback() : impl_(std::make_unique<Impl>()) {}
ForceFeedback::~ForceFeedback() { shutdown(); }

void ForceFeedback::init(GLFWwindow* window) {
    impl_->hwnd = glfwGetWin32Window(window);
    if (FAILED(DirectInput8Create(GetModuleHandleW(nullptr), DIRECTINPUT_VERSION, IID_IDirectInput8W,
                                  reinterpret_cast<void**>(&impl_->di), nullptr))) {
        LOGW("Force feedback: DirectInput unavailable");
        impl_->di = nullptr;
    }
}

void ForceFeedback::shutdown() {
    if (!impl_) return;
    impl_->stop();
    if (impl_->di) {
        impl_->di->Release();
        impl_->di = nullptr;
    }
}

void ForceFeedback::configure(const FfbConfig& cfg, const FfbDeviceHint& hint) {
    Impl& m = *impl_;
    if (!m.di) return;
    bool reopen = false;
    {
        std::lock_guard<std::mutex> lk(m.cfgMutex);
        m.cfg = cfg;
        m.hint = hint;
        reopen = m.rescan || (cfg.enabled && m.openedFor != hint.guid);
        m.rescan = false;
    }
    if (!cfg.enabled) {
        // hand the wheel back (other apps may want it) until FFB is switched on again
        if (m.dev) m.stop();
        m.openedFor = "\x01";
        return;
    }
    if (!reopen) return;
    m.stop();
    m.openedFor = hint.guid;
    if (m.open(hint)) m.start();
}

void ForceFeedback::push(const std::vector<float>& samples) {
    if (impl_->running.load()) impl_->signal.push(samples.data(), static_cast<int>(samples.size()));
}

void ForceFeedback::rescan() {
    std::lock_guard<std::mutex> lk(impl_->cfgMutex);
    impl_->rescan = true;
}

void ForceFeedback::startTest() { impl_->signal.startTest(); }
void ForceFeedback::resetPeak() { impl_->signal.resetPeak(); }

FfbStatus ForceFeedback::status() const {
    const Impl& m = *impl_;
    FfbStatus s;
    s.supported = m.di != nullptr;
    s.device = m.dev != nullptr;
    s.acquired = m.acquired.load();
    s.deviceName = m.deviceName;
    s.output = m.output.load();
    s.torqueNm = m.torque.load();
    s.clipping = m.clipping.load();
    s.clipFraction = m.signal.clipFraction();
    s.peakNm = m.signal.peakTorque();
    s.live = m.live.load();
    s.safetyTrips = m.signal.safetyTrips();
    return s;
}

#else
// ============================================================================ other platforms
struct ForceFeedback::Impl {
    FfbSignal signal;
};

ForceFeedback::ForceFeedback() : impl_(std::make_unique<Impl>()) {}
ForceFeedback::~ForceFeedback() = default;
void ForceFeedback::init(GLFWwindow*) {}
void ForceFeedback::shutdown() {}
void ForceFeedback::configure(const FfbConfig&, const FfbDeviceHint&) {}
void ForceFeedback::push(const std::vector<float>&) {}
void ForceFeedback::rescan() {}
void ForceFeedback::startTest() {}
void ForceFeedback::resetPeak() {}
FfbStatus ForceFeedback::status() const { return FfbStatus{}; }
#endif

}  // namespace apex
