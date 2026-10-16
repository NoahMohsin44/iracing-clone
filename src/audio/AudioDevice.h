#pragma once

namespace apex {

class CarAudio;

// Output device (miniaudio, 48 kHz stereo float) pulling samples from the car audio synth.
class AudioDevice {
public:
    ~AudioDevice() { shutdown(); }
    bool init(CarAudio* source);
    void shutdown();
    bool running() const { return device_ != nullptr; }
    float sampleRate() const { return 48000.0f; }

private:
    void* device_ = nullptr;
    CarAudio* source_ = nullptr;
};

}  // namespace apex
