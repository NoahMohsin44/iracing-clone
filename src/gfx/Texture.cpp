#include "gfx/Texture.h"

#include <algorithm>
#include <cmath>

#include "core/Log.h"
#include "stb_image.h"

namespace apex {

Texture& Texture::operator=(Texture&& o) noexcept {
    if (this != &o) {
        release();
        id_ = o.id_;
        target_ = o.target_;
        w_ = o.w_;
        h_ = o.h_;
        layers_ = o.layers_;
        levels_ = o.levels_;
        o.id_ = 0;
    }
    return *this;
}

void Texture::release() {
    if (id_) glDeleteTextures(1, &id_);
    id_ = 0;
}

int Texture::mipCount(int w, int h) {
    return 1 + static_cast<int>(std::floor(std::log2(static_cast<float>(std::max(w, h)))));
}

bool Texture::loadFile(const std::string& path, bool srgb, float anisotropy) {
    int w, h, comp;
    // flip so +v is "up" in the image (OpenGL normal-map convention, consistent tangent frames)
    stbi_set_flip_vertically_on_load(1);
    stbi_uc* px = stbi_load(path.c_str(), &w, &h, &comp, 4);
    stbi_set_flip_vertically_on_load(0);
    if (!px) {
        LOGE("Failed to load texture %s", path.c_str());
        return false;
    }
    create2D(w, h, srgb ? GL_SRGB8_ALPHA8 : GL_RGBA8, mipCount(w, h));
    upload(px, GL_RGBA, GL_UNSIGNED_BYTE);
    stbi_image_free(px);
    generateMips();
    setFilter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
    setWrap(GL_REPEAT);
    setAnisotropy(anisotropy);
    return true;
}

void Texture::create2D(int w, int h, GLenum internalFormat, int levels) {
    release();
    target_ = GL_TEXTURE_2D;
    w_ = w;
    h_ = h;
    levels_ = levels;
    glCreateTextures(GL_TEXTURE_2D, 1, &id_);
    glTextureStorage2D(id_, levels, internalFormat, w, h);
    setFilter(levels > 1 ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR, GL_LINEAR);
    setWrap(GL_CLAMP_TO_EDGE);
}

void Texture::create2DMultisample(int w, int h, GLenum internalFormat, int samples) {
    release();
    target_ = GL_TEXTURE_2D_MULTISAMPLE;
    w_ = w;
    h_ = h;
    glCreateTextures(GL_TEXTURE_2D_MULTISAMPLE, 1, &id_);
    glTextureStorage2DMultisample(id_, samples, internalFormat, w, h, GL_TRUE);
}

void Texture::create2DArray(int w, int h, int layers, GLenum internalFormat, int levels) {
    release();
    target_ = GL_TEXTURE_2D_ARRAY;
    w_ = w;
    h_ = h;
    layers_ = layers;
    levels_ = levels;
    glCreateTextures(GL_TEXTURE_2D_ARRAY, 1, &id_);
    glTextureStorage3D(id_, levels, internalFormat, w, h, layers);
    setFilter(levels > 1 ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR, GL_LINEAR);
    setWrap(GL_CLAMP_TO_EDGE);
}

void Texture::createCube(int size, GLenum internalFormat, int levels) {
    release();
    target_ = GL_TEXTURE_CUBE_MAP;
    w_ = h_ = size;
    levels_ = levels;
    glCreateTextures(GL_TEXTURE_CUBE_MAP, 1, &id_);
    glTextureStorage2D(id_, levels, internalFormat, size, size);
    setFilter(levels > 1 ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR, GL_LINEAR);
    setWrap(GL_CLAMP_TO_EDGE);
}

void Texture::upload(const void* data, GLenum format, GLenum type, int level) {
    int w = std::max(1, w_ >> level), h = std::max(1, h_ >> level);
    if (target_ == GL_TEXTURE_2D_ARRAY) glTextureSubImage3D(id_, level, 0, 0, 0, w, h, layers_, format, type, data);
    else glTextureSubImage2D(id_, level, 0, 0, w, h, format, type, data);
}

void Texture::setFilter(GLenum minF, GLenum magF) const {
    glTextureParameteri(id_, GL_TEXTURE_MIN_FILTER, minF);
    glTextureParameteri(id_, GL_TEXTURE_MAG_FILTER, magF);
}

void Texture::setWrap(GLenum wrap) const {
    glTextureParameteri(id_, GL_TEXTURE_WRAP_S, wrap);
    glTextureParameteri(id_, GL_TEXTURE_WRAP_T, wrap);
    glTextureParameteri(id_, GL_TEXTURE_WRAP_R, wrap);
}

void Texture::setAnisotropy(float a) const {
    if (a <= 1.0f) return;
    if (GLAD_GL_ARB_texture_filter_anisotropic || GLAD_GL_EXT_texture_filter_anisotropic) {
        GLfloat maxA = 1.0f;
        glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY, &maxA);
        glTextureParameterf(id_, GL_TEXTURE_MAX_ANISOTROPY, std::min(a, maxA));
    }
}

}  // namespace apex
