#pragma once
#include <vector>

#include "gfx/Texture.h"

namespace apex {

// Framebuffer owning its attachments (colour textures + optional depth texture).
class Framebuffer {
public:
    Framebuffer() = default;
    ~Framebuffer() { release(); }
    Framebuffer(const Framebuffer&) = delete;
    Framebuffer& operator=(const Framebuffer&) = delete;

    // samples > 1 creates multisample attachments.
    void create(int w, int h, const std::vector<GLenum>& colorFormats, GLenum depthFormat, int samples = 1);
    // Attach external textures (e.g. a cubemap face / mip or array layer).
    void createEmpty();
    void attachColor(GLuint tex, int level, int layer = -1);
    void attachDepth(GLuint tex, int level, int layer = -1);
    void bind() const;
    void release();

    GLuint id() const { return fbo_; }
    const Texture& color(int i = 0) const { return colors_[i]; }
    const Texture& depth() const { return depth_; }
    int width() const { return w_; }
    int height() const { return h_; }
    int samples() const { return samples_; }

private:
    GLuint fbo_ = 0;
    std::vector<Texture> colors_;
    Texture depth_;
    int w_ = 0, h_ = 0, samples_ = 1;
};

}  // namespace apex
