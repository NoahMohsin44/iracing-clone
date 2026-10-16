#pragma once
#include <string>

#include "gfx/GLUtil.h"

namespace apex {

class Texture {
public:
    Texture() = default;
    ~Texture() { release(); }
    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;
    Texture(Texture&& o) noexcept { *this = std::move(o); }
    Texture& operator=(Texture&& o) noexcept;

    // Image file (jpg/png). sRGB for colour data, linear for normals/masks.
    bool loadFile(const std::string& path, bool srgb, float anisotropy = 8.0f);
    void create2D(int w, int h, GLenum internalFormat, int levels = 1);
    void create2DMultisample(int w, int h, GLenum internalFormat, int samples);
    void create2DArray(int w, int h, int layers, GLenum internalFormat, int levels = 1);
    void createCube(int size, GLenum internalFormat, int levels = 1);
    void upload(const void* data, GLenum format, GLenum type, int level = 0);
    void generateMips() const { glGenerateTextureMipmap(id_); }
    void setFilter(GLenum minF, GLenum magF) const;
    void setWrap(GLenum wrap) const;
    void setAnisotropy(float a) const;
    void bind(int unit) const { glBindTextureUnit(unit, id_); }
    void release();

    GLuint id() const { return id_; }
    int width() const { return w_; }
    int height() const { return h_; }
    int levels() const { return levels_; }
    bool valid() const { return id_ != 0; }

    static int mipCount(int w, int h);

private:
    GLuint id_ = 0;
    GLenum target_ = GL_TEXTURE_2D;
    int w_ = 0, h_ = 0, layers_ = 1, levels_ = 1;
};

}  // namespace apex
