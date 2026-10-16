#pragma once
#include <glm/glm.hpp>

#include "gfx/Framebuffer.h"
#include "gfx/Shader.h"
#include "gfx/Texture.h"

namespace apex {

// Physically based sky for a given time of day at the Nurburgring (50.35 N, mid-July):
// renders the atmosphere into a cubemap, then derives specular IBL mips, SH irradiance,
// sun colour and fog colour from it.
class Sky {
public:
    bool init();
    void update(float timeOfDayHours, float cloudCover = 0.42f);

    glm::vec3 sunDirection() const { return sunDir_; }
    glm::vec3 sunIrradiance() const { return sunColor_; }
    glm::vec3 fogColor() const { return fogColor_; }
    const glm::vec4* sh() const { return sh_; }
    const Texture& skyCube() const { return sky_; }
    const Texture& specular() const { return spec_; }
    const Texture& brdfLut() const { return lut_; }
    int specularMips() const { return spec_.levels(); }

    static glm::vec3 solarDirection(float hours, float latitudeDeg = 50.35f, int dayOfYear = 196);
    static glm::vec3 transmittance(const glm::vec3& dir);

private:
    void renderCube(Texture& target, int size, int level);
    void computeSH(const Texture& cube, int size);

    Shader gen_, prefilter_, brdf_;
    Texture sky_, spec_, lut_, small_;
    Framebuffer fbo_;
    glm::vec3 sunDir_{0.3f, 0.8f, 0.2f};
    glm::vec3 sunColor_{3.0f};
    glm::vec3 fogColor_{0.5f, 0.6f, 0.7f};
    glm::vec4 sh_[9];
    float sunIntensity_ = 22.0f;
    float cloudCover_ = 0.4f;
};

}  // namespace apex
