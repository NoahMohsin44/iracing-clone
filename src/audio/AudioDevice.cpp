#include "audio/AudioDevice.h"

#include "audio/CarAudio.h"
#include "core/Log.h"
#include "miniaudio.h"

namespace apex {

namespace {
void dataCallback(ma_device* device, void* output, const void*, ma_uint32 frames) {
    auto* audio = static_cast<CarAudio*>(device->pUserData);
    if (audio) audio->render(static_cast<float*>(output), static_cast<int>(frames));
}
}  // namespace

bool AudioDevice::init(CarAudio* source) {
    source_ = source;
    auto* dev = new ma_device();
    ma_device_config cfg = ma_device_config_init(ma_device_type_playback);
    cfg.playback.format = ma_format_f32;
    cfg.playback.channels = 2;
    cfg.sampleRate = 48000;
    cfg.periodSizeInFrames = 256;  // ~5 ms: low latency so the engine follows the throttle
    cfg.dataCallback = dataCallback;
    cfg.pUserData = source;
    if (ma_device_init(nullptr, &cfg, dev) != MA_SUCCESS) {
        LOGW("No audio output device available");
        delete dev;
        return false;
    }
    if (ma_device_start(dev) != MA_SUCCESS) {
        LOGW("Failed to start audio device");
        ma_device_uninit(dev);
        delete dev;
        return false;
    }
    device_ = dev;
    LOGI("Audio: %s, %u Hz", dev->playback.name, dev->sampleRate);
    return true;
}

void AudioDevice::shutdown() {
    if (!device_) return;
    auto* dev = static_cast<ma_device*>(device_);
    ma_device_uninit(dev);
    delete dev;
    device_ = nullptr;
}

}  // namespace apex
