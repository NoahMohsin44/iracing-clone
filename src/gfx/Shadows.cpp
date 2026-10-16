#include "gfx/Shadows.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/matrix_transform.hpp>

namespace apex {

void ShadowMaps::init(int quality) {
    if (quality == quality_ && depth_.valid()) return;
    quality_ = quality;
    switch (quality) {
        case 0:
            cascades_ = 2;
            size_ = 1024;
            splits_[0] = 18.0f;
            splits_[1] = 90.0f;
            break;
        case 1:
            cascades_ = 3;
            size_ = 2048;
            splits_[0] = 10.0f;
            splits_[1] = 45.0f;
            splits_[2] = 170.0f;
            break;
        case 2:
            cascades_ = 4;
            size_ = 2048;
            splits_[0] = 8.0f;
            splits_[1] = 30.0f;
            splits_[2] = 95.0f;
            splits_[3] = 280.0f;
            break;
        default:
            cascades_ = 4;
            size_ = 4096;
            splits_[0] = 9.0f;
            splits_[1] = 34.0f;
            splits_[2] = 110.0f;
            splits_[3] = 360.0f;
            break;
    }
    for (int i = cascades_; i < 4; ++i) splits_[i] = splits_[cascades_ - 1];
    depth_.create2DArray(size_, size_, 4, GL_DEPTH_COMPONENT32F, 1);
    depth_.setFilter(GL_LINEAR, GL_LINEAR);
    glTextureParameteri(depth_.id(), GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTextureParameteri(depth_.id(), GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    if (!fbo_) glCreateFramebuffers(1, &fbo_);
    glNamedFramebufferDrawBuffer(fbo_, GL_NONE);
    glNamedFramebufferReadBuffer(fbo_, GL_NONE);
}

void ShadowMaps::update(const Camera& cam, const glm::vec3& sunDir) {
    const float tanY = std::tan(cam.fovY * 0.5f);
    const float tanX = tanY * cam.aspect;
    const glm::dvec3 camPos = cam.position;
    for (int i = 0; i < cascades_; ++i) {
        float n = i == 0 ? cam.nearZ : splits_[i - 1];
        float f = splits_[i];
        // bounding sphere of the frustum slice (camera-relative)
        glm::vec3 corners[8];
        int k = 0;
        for (float d : {n, f})
            for (float sx : {-1.0f, 1.0f})
                for (float sy : {-1.0f, 1.0f})
                    corners[k++] = cam.basis * glm::vec3(sx * tanX * d, sy * tanY * d, -d);
        glm::vec3 c(0.0f);
        for (auto& p : corners) c += p;
        c /= 8.0f;
        float r = 0.0f;
        for (auto& p : corners) r = std::max(r, glm::length(p - c));
        r = std::ceil(r * 16.0f) / 16.0f;  // quantise to avoid size flicker

        // world-anchored light frame for texel snapping
        glm::dvec3 L = glm::normalize(glm::dvec3(sunDir));
        glm::dvec3 up = std::abs(L.y) > 0.99 ? glm::dvec3(0, 0, 1) : glm::dvec3(0, 1, 0);
        glm::dvec3 lx = glm::normalize(glm::cross(up, L));
        glm::dvec3 ly = glm::cross(L, lx);
        glm::dvec3 centreW = camPos + glm::dvec3(c);
        double texel = 2.0 * r / size_;
        double cx = std::floor(glm::dot(centreW, lx) / texel) * texel;
        double cy = std::floor(glm::dot(centreW, ly) / texel) * texel;
        double cz = glm::dot(centreW, L);
        glm::dvec3 snapped = lx * cx + ly * cy + L * cz;
        const double back = r + 600.0;  // include tall casters (trees, hills) towards the sun
        glm::dvec3 eye = snapped + L * back;
        glm::dmat4 view = glm::lookAt(eye, snapped, up);
        glm::dmat4 proj = glm::orthoRH_ZO(-double(r), double(r), -double(r), double(r), 0.0, back + r + 50.0);
        glm::dmat4 toRel = glm::translate(glm::dmat4(1.0), camPos);  // rel -> world
        glm::dmat4 vp = proj * view * toRel;
        lightVP_[i] = glm::mat4(vp);
        glm::dmat4 bias(0.5, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 1, 0, 0.5, 0.5, 0, 1);
        shadowMat_[i] = glm::mat4(bias * vp);
        centre_[i] = c;
        radius_[i] = r;
    }
}

void ShadowMaps::bindCascade(int i) const {
    glNamedFramebufferTextureLayer(fbo_, GL_DEPTH_ATTACHMENT, depth_.id(), 0, i);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glViewport(0, 0, size_, size_);
}

Frustum ShadowMaps::cascadeFrustum(int i) const {
    glm::mat4 m = glm::transpose(lightVP_[i]);
    Frustum f;
    f.planes[0] = m[3] + m[0];
    f.planes[1] = m[3] - m[0];
    f.planes[2] = m[3] + m[1];
    f.planes[3] = m[3] - m[1];
    for (int k = 0; k < 4; ++k) f.planes[k] /= glm::length(glm::vec3(f.planes[k]));
    f.planes[4] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);  // casters between the light and the slice stay (depth clamp)
    return f;
}

}  // namespace apex
