#include "gfx/Sky.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "core/Log.h"

namespace apex {

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr int kSkySize = 512;
constexpr int kSpecSize = 128;
constexpr int kSmallSize = 32;

glm::vec3 cubeDir(int face, float u, float v) {
    float cx = u * 2.0f - 1.0f, cy = v * 2.0f - 1.0f;
    glm::vec3 d;
    switch (face) {
        case 0: d = {1.0f, -cy, -cx}; break;
        case 1: d = {-1.0f, -cy, cx}; break;
        case 2: d = {cx, 1.0f, cy}; break;
        case 3: d = {cx, -1.0f, -cy}; break;
        case 4: d = {cx, -cy, 1.0f}; break;
        default: d = {-cx, -cy, -1.0f}; break;
    }
    return glm::normalize(d);
}
}  // namespace

glm::vec3 Sky::solarDirection(float hours, float latitudeDeg, int dayOfYear) {
    const float lat = glm::radians(latitudeDeg);
    const float decl = glm::radians(23.44f) * std::sin(2.0f * kPi * (284.0f + dayOfYear) / 365.0f);
    // local solar time ~ clock time - 1.5 h in CEST at 7 E
    const float solar = hours - 1.5f + 0.47f;
    const float H = glm::radians(15.0f * (solar - 12.0f));
    float sinEl = std::sin(lat) * std::sin(decl) + std::cos(lat) * std::cos(decl) * std::cos(H);
    float el = std::asin(std::clamp(sinEl, -1.0f, 1.0f));
    float cosAz = (std::sin(decl) - std::sin(el) * std::sin(lat)) / (std::cos(el) * std::cos(lat));
    float az = std::acos(std::clamp(cosAz, -1.0f, 1.0f));
    if (H > 0.0f) az = 2.0f * kPi - az;  // afternoon: sun in the west
    // world: +X east, -Z north
    return glm::normalize(glm::vec3(std::sin(az) * std::cos(el), std::sin(el), -std::cos(az) * std::cos(el)));
}

glm::vec3 Sky::transmittance(const glm::vec3& dir) {
    // Same atmosphere as skygen.frag: optical depth from 550 m altitude towards dir.
    const double Re = 6360e3, Ra = 6420e3, HR = 8000.0, HM = 1200.0;
    const glm::dvec3 bR(5.802e-6, 13.558e-6, 33.1e-6), bOz(0.650e-6, 1.881e-6, 0.085e-6);
    const double bMe = 4.44e-6;
    glm::dvec3 ro(0.0, Re + 550.0, 0.0), rd(dir);
    double b = glm::dot(ro, rd), c = glm::dot(ro, ro) - Ra * Ra;
    double t = -b + std::sqrt(std::max(0.0, b * b - c));
    double c2 = glm::dot(ro, ro) - Re * Re;
    double disc = b * b - c2;
    if (disc > 0 && -b - std::sqrt(disc) > 0) return glm::vec3(0.0f);  // below the horizon
    double odR = 0, odM = 0;
    const int N = 64;
    double dt = t / N;
    for (int i = 0; i < N; ++i) {
        glm::dvec3 p = ro + rd * ((i + 0.5) * dt);
        double h = glm::length(p) - Re;
        odR += std::exp(-h / HR) * dt;
        odM += std::exp(-h / HM) * dt;
    }
    glm::dvec3 tau = bR * odR + bOz * odR * 0.6 + glm::dvec3(bMe * odM);
    return glm::vec3(glm::exp(-tau));
}

bool Sky::init() {
    bool ok = gen_.load("fullscreen.vert", "skygen.frag") && prefilter_.load("fullscreen.vert", "prefilter.frag") &&
              brdf_.load("fullscreen.vert", "brdf.frag");
    if (!ok) return false;
    sky_.createCube(kSkySize, GL_RGBA16F, Texture::mipCount(kSkySize, kSkySize));
    spec_.createCube(kSpecSize, GL_RGBA16F, 6);
    small_.createCube(kSmallSize, GL_RGBA32F, 1);
    lut_.create2D(128, 128, GL_RG16F, 1);
    fbo_.createEmpty();

    // BRDF LUT (static)
    fbo_.attachColor(lut_.id(), 0);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_.id());
    glViewport(0, 0, 128, 128);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    brdf_.use();
    gl::drawFullscreenTriangle();
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return true;
}

void Sky::renderCube(Texture& target, int size, int level) {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_.id());
    glViewport(0, 0, size, size);
    for (int face = 0; face < 6; ++face) {
        fbo_.attachColor(target.id(), level, face);
        gen_.set("uFace", face);
        gen_.set("uSize", size);
        gl::drawFullscreenTriangle();
    }
}

void Sky::update(float hours, float cloudCover) {
    cloudCover_ = cloudCover;
    sunDir_ = solarDirection(hours);
    if (sunDir_.y < 0.03f) sunDir_ = glm::normalize(glm::vec3(sunDir_.x, 0.03f, sunDir_.z));
    const glm::vec3 trans = transmittance(sunDir_);
    // Scale chosen so a white Lambertian surface under the noon sun reads ~1.0 before exposure.
    const float sunIrr = 3.6f;
    sunColor_ = trans * sunIrr;
    sunIntensity_ = sunIrr * 6.5f;

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    gen_.use();
    gen_.set("uSun", sunDir_);
    gen_.set("uSunIntensity", sunIntensity_);
    gen_.set("uSunLight", sunColor_);
    gen_.set("uCloudCover", cloudCover_);
    gen_.set("uCloudSeed", 3.7f);
    renderCube(sky_, kSkySize, 0);
    sky_.generateMips();
    renderCube(small_, kSmallSize, 0);

    // specular prefilter
    prefilter_.use();
    sky_.bind(0);
    prefilter_.set("uSrcSize", static_cast<float>(kSkySize));
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_.id());
    for (int mip = 0; mip < spec_.levels(); ++mip) {
        int size = kSpecSize >> mip;
        glViewport(0, 0, size, size);
        prefilter_.set("uRoughness", static_cast<float>(mip) / (spec_.levels() - 1));
        prefilter_.set("uSize", size);
        for (int face = 0; face < 6; ++face) {
            fbo_.attachColor(spec_.id(), mip, face);
            prefilter_.set("uFace", face);
            gl::drawFullscreenTriangle();
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    computeSH(small_, kSmallSize);
    glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
    LOGI("Sky: sun dir (%.2f %.2f %.2f) colour (%.2f %.2f %.2f)", sunDir_.x, sunDir_.y, sunDir_.z, sunColor_.x,
         sunColor_.y, sunColor_.z);
}

void Sky::computeSH(const Texture& cube, int size) {
    std::vector<float> px(static_cast<size_t>(size) * size * 4 * 6);
    glGetTextureImage(cube.id(), 0, GL_RGBA, GL_FLOAT, static_cast<GLsizei>(px.size() * sizeof(float)), px.data());
    glm::dvec3 L[9] = {};
    double wsum = 0;
    glm::dvec3 horizon(0.0);
    double hw = 0;
    for (int face = 0; face < 6; ++face) {
        for (int y = 0; y < size; ++y) {
            for (int x = 0; x < size; ++x) {
                float u = (x + 0.5f) / size, v = (y + 0.5f) / size;
                glm::vec3 d = cubeDir(face, u, v);
                float cu = u * 2 - 1, cv = v * 2 - 1;
                double dw = 4.0 / (std::pow(1.0 + cu * cu + cv * cv, 1.5) * size * size);
                const float* p = &px[((static_cast<size_t>(face) * size + y) * size + x) * 4];
                glm::dvec3 c(p[0], p[1], p[2]);
                const double Y[9] = {0.282095,
                                     0.488603 * d.y,
                                     0.488603 * d.z,
                                     0.488603 * d.x,
                                     1.092548 * d.x * d.y,
                                     1.092548 * d.y * d.z,
                                     0.315392 * (3.0 * d.z * d.z - 1.0),
                                     1.092548 * d.x * d.z,
                                     0.546274 * (d.x * d.x - d.y * d.y)};
                for (int i = 0; i < 9; ++i) L[i] += c * Y[i] * dw;
                wsum += dw;
                if (d.y > 0.0f && d.y < 0.08f) {
                    horizon += c * dw;
                    hw += dw;
                }
            }
        }
    }
    const double A[9] = {kPi, 2.0 * kPi / 3.0, 2.0 * kPi / 3.0, 2.0 * kPi / 3.0, kPi / 4.0,
                         kPi / 4.0, kPi / 4.0, kPi / 4.0, kPi / 4.0};
    for (int i = 0; i < 9; ++i) sh_[i] = glm::vec4(glm::vec3(L[i] * (A[i] / kPi)), 0.0f);
    if (hw > 0) fogColor_ = glm::vec3(horizon / hw);
}

}  // namespace apex
