#include "gfx/PostFX.h"

#include <algorithm>
#include <cstdlib>

namespace apex {

bool PostFX::init() {
    return resolve_.load("fullscreen.vert", "resolve.frag") && down_.load("fullscreen.vert", "bloom_down.frag") &&
           up_.load("fullscreen.vert", "bloom_up.frag") && composite_.load("fullscreen.vert", "composite.frag") &&
           aoDepth_.load("fullscreen.vert", "ao_depth.frag") && ao_.load("fullscreen.vert", "ao.frag") &&
           aoBlur_.load("fullscreen.vert", "ao_blur.frag");
}

void PostFX::resize(int w, int h) {
    if (w == w_ && h == h_) return;
    w_ = w;
    h_ = h;
    hdr_.create(w, h, {GL_RGBA16F}, 0, 1);
    hdr_.color(0).setFilter(GL_LINEAR, GL_LINEAR);
    int bw = w, bh = h;
    for (int i = 0; i < kBloomLevels; ++i) {
        bw = std::max(1, bw / 2);
        bh = std::max(1, bh / 2);
        bloom_[i].create(bw, bh, {GL_R11F_G11F_B10F}, 0, 1);
        bloom_[i].color(0).setFilter(GL_LINEAR, GL_LINEAR);
    }
    const int hw = std::max(1, w / 2), hh = std::max(1, h / 2);
    aoDist_.create(hw, hh, {GL_R32F}, 0, 1);
    aoDist_.color(0).setFilter(GL_NEAREST, GL_NEAREST);
    aoDist_.color(0).setWrap(GL_CLAMP_TO_EDGE);
    for (auto& t : aoTex_) {
        t.create(hw, hh, {GL_R8}, 0, 1);
        t.color(0).setFilter(GL_LINEAR, GL_LINEAR);
        t.color(0).setWrap(GL_CLAMP_TO_EDGE);
    }
}

void PostFX::ambientOcclusion(const Framebuffer& scene, float nearZ) {
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    const int hw = aoDist_.width(), hh = aoDist_.height();
    aoDist_.bind();
    aoDepth_.use();
    const bool ms = scene.samples() > 1;
    glBindTextureUnit(ms ? 0 : 1, scene.depth().id());
    aoDepth_.set("uMultisample", ms ? 1 : 0);
    aoDepth_.set("uNear", nearZ);
    gl::drawFullscreenTriangle();

    aoTex_[0].bind();
    ao_.use();
    aoDist_.color(0).bind(0);
    ao_.set("uTexel", glm::vec2(1.0f / hw, 1.0f / hh));
    ao_.set("uRadius", 0.9f);
    gl::drawFullscreenTriangle();

    aoBlur_.use();
    aoDist_.color(0).bind(1);
    aoTex_[1].bind();
    aoTex_[0].color(0).bind(0);
    aoBlur_.set("uDir", glm::vec2(1.0f / hw, 0.0f));
    gl::drawFullscreenTriangle();
    aoTex_[0].bind();
    aoTex_[1].color(0).bind(0);
    aoBlur_.set("uDir", glm::vec2(0.0f, 1.0f / hh));
    gl::drawFullscreenTriangle();
    aoValid_ = true;
}

void PostFX::resolve(const Framebuffer& scene) {
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    if (scene.samples() > 1) {
        hdr_.bind();
        resolve_.use();
        glBindTextureUnit(0, scene.color(0).id());
        resolve_.set("uSamples", scene.samples());
        gl::drawFullscreenTriangle();
    } else {
        glBlitNamedFramebuffer(scene.id(), hdr_.id(), 0, 0, scene.width(), scene.height(), 0, 0, w_, h_,
                               GL_COLOR_BUFFER_BIT, GL_NEAREST);
    }
}

void PostFX::bloom() {
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    down_.use();
    for (int i = 0; i < kBloomLevels; ++i) {
        const Texture& src = i == 0 ? hdr_.color(0) : bloom_[i - 1].color(0);
        bloom_[i].bind();
        src.bind(0);
        down_.set("uTexel", glm::vec2(1.0f / src.width(), 1.0f / src.height()));
        down_.set("uFirst", i == 0 ? 1 : 0);
        gl::drawFullscreenTriangle();
    }
    up_.use();
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE);
    for (int i = kBloomLevels - 1; i > 0; --i) {
        bloom_[i - 1].bind();
        bloom_[i].color(0).bind(0);
        up_.set("uTexel", glm::vec2(1.0f / bloom_[i].width(), 1.0f / bloom_[i].height()));
        gl::drawFullscreenTriangle();
    }
    glDisable(GL_BLEND);
    bloomValid_ = true;
}

void PostFX::composite(int outW, int outH, float exposure, bool bloomOn, float vignette) {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, outW, outH);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    composite_.use();
    hdr_.color(0).bind(0);
    bloom_[0].color(0).bind(1);
    aoTex_[0].color(0).bind(2);
    static const bool debugAO = std::getenv("APEX_DEBUG_AO") != nullptr;
    composite_.set("uAO", aoValid_ ? (debugAO ? -1.0f : 0.75f) : 0.0f);
    composite_.set("uExposure", exposure);
    composite_.set("uBloom", bloomOn && bloomValid_ ? 0.045f : 0.0f);
    composite_.set("uVignette", vignette);
    gl::drawFullscreenTriangle();
}

}  // namespace apex
