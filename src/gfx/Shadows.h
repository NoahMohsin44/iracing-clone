#pragma once
#include <glm/glm.hpp>

#include "gfx/Camera.h"
#include "gfx/Framebuffer.h"

namespace apex {

// Cascaded shadow maps for the sun. Cascades are stabilised (texel snapped in world space)
// so shadows don't shimmer while driving.
class ShadowMaps {
public:
    void init(int quality);  // 0 low .. 3 ultra
    void update(const Camera& cam, const glm::vec3& sunDir);
    void bindCascade(int i) const;

    int quality() const { return quality_; }
    int cascades() const { return cascades_; }
    int size() const { return size_; }
    const Texture& depth() const { return depth_; }
    const glm::mat4& lightViewProj(int i) const { return lightVP_[i]; }  // camera-relative
    const glm::mat4& shadowMatrix(int i) const { return shadowMat_[i]; }
    float split(int i) const { return splits_[i]; }
    float texelSize() const { return 1.0f / size_; }
    Frustum cascadeFrustum(int i) const;
    glm::vec3 cascadeCentre(int i) const { return centre_[i]; }
    float cascadeRadius(int i) const { return radius_[i]; }

private:
    int quality_ = -1;
    int cascades_ = 4;
    int size_ = 2048;
    float splits_[4] = {8.0f, 30.0f, 95.0f, 280.0f};
    Texture depth_;
    GLuint fbo_ = 0;
    glm::mat4 lightVP_[4];
    glm::mat4 shadowMat_[4];
    glm::vec3 centre_[4];
    float radius_[4] = {};
};

}  // namespace apex
