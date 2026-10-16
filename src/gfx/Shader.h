#pragma once
#include <string>
#include <unordered_map>
#include <vector>

#include <glm/glm.hpp>

#include "gfx/GLUtil.h"

namespace apex {

// GLSL program loaded from assets/shaders with support for `#include "file"` and injected defines.
class Shader {
public:
    Shader() = default;
    ~Shader();
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&& o) noexcept { *this = std::move(o); }
    Shader& operator=(Shader&& o) noexcept;

    bool load(const std::string& vertFile, const std::string& fragFile, const std::vector<std::string>& defines = {});
    bool loadCompute(const std::string& compFile, const std::vector<std::string>& defines = {});
    void use() const { glUseProgram(program_); }
    GLuint id() const { return program_; }
    bool valid() const { return program_ != 0; }

    GLint loc(const char* name) const;
    void set(const char* name, int v) const { glProgramUniform1i(program_, loc(name), v); }
    void set(const char* name, float v) const { glProgramUniform1f(program_, loc(name), v); }
    void set(const char* name, const glm::vec2& v) const { glProgramUniform2fv(program_, loc(name), 1, &v.x); }
    void set(const char* name, const glm::vec3& v) const { glProgramUniform3fv(program_, loc(name), 1, &v.x); }
    void set(const char* name, const glm::vec4& v) const { glProgramUniform4fv(program_, loc(name), 1, &v.x); }
    void set(const char* name, const glm::mat4& m) const {
        glProgramUniformMatrix4fv(program_, loc(name), 1, GL_FALSE, &m[0][0]);
    }
    void set(const char* name, const glm::mat3& m) const {
        glProgramUniformMatrix3fv(program_, loc(name), 1, GL_FALSE, &m[0][0]);
    }

private:
    GLuint program_ = 0;
    mutable std::unordered_map<std::string, GLint> cache_;
};

}  // namespace apex
