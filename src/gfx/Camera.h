#pragma once
#include <glm/glm.hpp>

#include "gfx/GLUtil.h"

namespace apex {

struct Frustum {
    glm::vec4 planes[5];  // left, right, bottom, top, near (infinite far)
    bool sphere(const glm::vec3& c, float r) const {
        for (const auto& p : planes)
            if (glm::dot(glm::vec3(p), c) + p.w < -r) return false;
        return true;
    }
    bool box(const glm::vec3& mn, const glm::vec3& mx) const {
        for (const auto& p : planes) {
            glm::vec3 v(p.x > 0 ? mx.x : mn.x, p.y > 0 ? mx.y : mn.y, p.z > 0 ? mx.z : mn.z);
            if (glm::dot(glm::vec3(p), v) + p.w < 0) return false;
        }
        return true;
    }
};

// Camera in world space (double position). Rendering uses camera-relative coordinates, so
// view() contains only the rotation.
struct Camera {
    glm::dvec3 position{0.0};
    glm::mat3 basis{1.0f};  // columns: right, up, back (camera to world)
    float fovY = glm::radians(60.0f);
    float aspect = 16.0f / 9.0f;
    float nearZ = 0.04f;

    glm::vec3 forward() const { return -basis[2]; }
    glm::mat4 view() const { return glm::mat4(glm::transpose(basis)); }
    glm::mat4 proj() const { return gl::perspectiveReversedZ(fovY, aspect, nearZ); }
    glm::mat4 viewProj() const { return proj() * view(); }

    void lookAt(const glm::dvec3& eye, const glm::dvec3& target, const glm::vec3& upHint = {0, 1, 0}) {
        position = eye;
        glm::vec3 f = glm::normalize(glm::vec3(target - eye));
        glm::vec3 r = glm::normalize(glm::cross(f, upHint));
        glm::vec3 u = glm::cross(r, f);
        basis = glm::mat3(r, u, -f);
    }

    Frustum frustum() const {
        glm::mat4 m = glm::transpose(viewProj());  // rows as columns
        Frustum f;
        f.planes[0] = m[3] + m[0];
        f.planes[1] = m[3] - m[0];
        f.planes[2] = m[3] + m[1];
        f.planes[3] = m[3] - m[1];
        f.planes[4] = m[3] - m[2];
        for (auto& p : f.planes) p /= glm::length(glm::vec3(p));
        return f;
    }
};

}  // namespace apex
