#pragma once
#include "gfx/Framebuffer.h"
#include "gfx/Shader.h"

namespace apex {

// MSAA resolve (tonemap-weighted), bloom (dual filter chain) and the final composite
// (exposure, ACES filmic tonemapping, vignette, dithering, upscale to the window).
class PostFX {
public:
    bool init();
    void resize(int w, int h);
    void resolve(const Framebuffer& scene);
    void bloom();
    void composite(int outW, int outH, float exposure, bool bloom, float vignette);
    const Framebuffer& hdr() const { return hdr_; }

private:
    static constexpr int kBloomLevels = 6;
    Shader resolve_, down_, up_, composite_;
    Framebuffer hdr_;
    Framebuffer bloom_[kBloomLevels];
    int w_ = 0, h_ = 0;
    bool bloomValid_ = false;
};

}  // namespace apex
