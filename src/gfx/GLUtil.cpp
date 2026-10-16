#include "gfx/GLUtil.h"

#include <cmath>
#include <cstring>

#include "core/Log.h"

namespace apex::gl {

namespace {
void GLAD_API_PTR debugCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei, const GLchar* message,
                            const void*) {
    if (severity == GL_DEBUG_SEVERITY_NOTIFICATION) return;
    if (id == 131185 || id == 131218 || id == 131204 || id == 131169) return;  // NVIDIA buffer/shader info spam
    if (type == GL_DEBUG_TYPE_ERROR) LOGE("GL: %s", message);
    else if (severity == GL_DEBUG_SEVERITY_HIGH) LOGW("GL: %s", message);
}
}  // namespace

void installDebugCallback() {
    if (!GLAD_GL_VERSION_4_3) return;
    GLint flags = 0;
    glGetIntegerv(GL_CONTEXT_FLAGS, &flags);
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(debugCallback, nullptr);
}

const char* errorString(GLenum err) {
    switch (err) {
        case GL_INVALID_ENUM: return "GL_INVALID_ENUM";
        case GL_INVALID_VALUE: return "GL_INVALID_VALUE";
        case GL_INVALID_OPERATION: return "GL_INVALID_OPERATION";
        case GL_OUT_OF_MEMORY: return "GL_OUT_OF_MEMORY";
        case GL_INVALID_FRAMEBUFFER_OPERATION: return "GL_INVALID_FRAMEBUFFER_OPERATION";
        default: return "unknown";
    }
}

void checkErrors(const char* where) {
    for (GLenum e = glGetError(); e != GL_NO_ERROR; e = glGetError()) LOGE("GL error %s at %s", errorString(e), where);
}

GLuint emptyVao() {
    static GLuint vao = 0;
    if (!vao) glCreateVertexArrays(1, &vao);
    return vao;
}

void drawFullscreenTriangle() {
    glBindVertexArray(emptyVao());
    glDrawArrays(GL_TRIANGLES, 0, 3);
}

glm::mat4 perspectiveReversedZ(float fovY, float aspect, float zNear) {
    const float f = 1.0f / std::tan(fovY * 0.5f);
    glm::mat4 p(0.0f);
    p[0][0] = f / aspect;
    p[1][1] = f;
    p[2][3] = -1.0f;
    p[3][2] = zNear;
    return p;
}

}  // namespace apex::gl
