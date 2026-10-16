#include "gfx/Framebuffer.h"

#include "core/Log.h"

namespace apex {

void Framebuffer::create(int w, int h, const std::vector<GLenum>& colorFormats, GLenum depthFormat, int samples) {
    release();
    w_ = w;
    h_ = h;
    samples_ = samples;
    glCreateFramebuffers(1, &fbo_);
    colors_.resize(colorFormats.size());
    std::vector<GLenum> drawBuffers;
    for (size_t i = 0; i < colorFormats.size(); ++i) {
        if (samples > 1) colors_[i].create2DMultisample(w, h, colorFormats[i], samples);
        else colors_[i].create2D(w, h, colorFormats[i], 1);
        glNamedFramebufferTexture(fbo_, GL_COLOR_ATTACHMENT0 + static_cast<GLenum>(i), colors_[i].id(), 0);
        drawBuffers.push_back(GL_COLOR_ATTACHMENT0 + static_cast<GLenum>(i));
    }
    if (depthFormat) {
        if (samples > 1) depth_.create2DMultisample(w, h, depthFormat, samples);
        else depth_.create2D(w, h, depthFormat, 1);
        glNamedFramebufferTexture(fbo_, GL_DEPTH_ATTACHMENT, depth_.id(), 0);
    }
    if (drawBuffers.empty()) glNamedFramebufferDrawBuffer(fbo_, GL_NONE);
    else glNamedFramebufferDrawBuffers(fbo_, static_cast<GLsizei>(drawBuffers.size()), drawBuffers.data());
    GLenum status = glCheckNamedFramebufferStatus(fbo_, GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) LOGE("Framebuffer incomplete: 0x%x", status);
}

void Framebuffer::createEmpty() {
    release();
    glCreateFramebuffers(1, &fbo_);
}

void Framebuffer::attachColor(GLuint tex, int level, int layer) {
    if (layer >= 0) glNamedFramebufferTextureLayer(fbo_, GL_COLOR_ATTACHMENT0, tex, level, layer);
    else glNamedFramebufferTexture(fbo_, GL_COLOR_ATTACHMENT0, tex, level);
    glNamedFramebufferDrawBuffer(fbo_, GL_COLOR_ATTACHMENT0);
}

void Framebuffer::attachDepth(GLuint tex, int level, int layer) {
    if (layer >= 0) glNamedFramebufferTextureLayer(fbo_, GL_DEPTH_ATTACHMENT, tex, level, layer);
    else glNamedFramebufferTexture(fbo_, GL_DEPTH_ATTACHMENT, tex, level);
}

void Framebuffer::bind() const {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    if (w_ > 0) glViewport(0, 0, w_, h_);
}

void Framebuffer::release() {
    if (fbo_) glDeleteFramebuffers(1, &fbo_);
    fbo_ = 0;
    colors_.clear();
    depth_.release();
}

}  // namespace apex
