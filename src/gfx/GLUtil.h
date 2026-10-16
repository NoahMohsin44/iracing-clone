#pragma once
#include <glad/gl.h>

#include <glm/glm.hpp>

namespace apex::gl {

void installDebugCallback();
const char* errorString(GLenum err);
void checkErrors(const char* where);

// Shared empty VAO for attribute-less full-screen triangles.
GLuint emptyVao();
void drawFullscreenTriangle();

// Reversed-Z infinite perspective (depth 1 at the near plane, 0 at infinity, clip space 0..1).
glm::mat4 perspectiveReversedZ(float fovY, float aspect, float zNear);

}  // namespace apex::gl
