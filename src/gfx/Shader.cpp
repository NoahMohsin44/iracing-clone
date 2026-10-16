#include "gfx/Shader.h"

#include <sstream>

#include "core/Log.h"
#include "core/Paths.h"

namespace apex {

namespace {
bool preprocess(const std::string& file, std::string& out, int depth = 0) {
    if (depth > 8) return false;
    std::string src;
    if (!paths::readTextFile(paths::asset("shaders/" + file), src)) {
        LOGE("Shader file not found: %s", file.c_str());
        return false;
    }
    std::istringstream in(src);
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("#include", 0) == 0) {
            size_t a = line.find('"'), b = line.rfind('"');
            if (a != std::string::npos && b > a) {
                if (!preprocess(line.substr(a + 1, b - a - 1), out, depth + 1)) return false;
                continue;
            }
        }
        out += line;
        out += '\n';
    }
    return true;
}

GLuint compile(GLenum stage, const std::string& file, const std::vector<std::string>& defines) {
    std::string body;
    if (!preprocess(file, body)) return 0;
    std::string src = "#version 450 core\n";
    for (auto& d : defines) src += "#define " + d + "\n";
    src += "#line 1\n" + body;
    GLuint sh = glCreateShader(stage);
    const char* p = src.c_str();
    glShaderSource(sh, 1, &p, nullptr);
    glCompileShader(sh);
    GLint ok = 0;
    glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[8192];
        glGetShaderInfoLog(sh, sizeof(log), nullptr, log);
        LOGE("Compile error in %s:\n%s", file.c_str(), log);
        glDeleteShader(sh);
        return 0;
    }
    return sh;
}

GLuint link(const std::vector<GLuint>& stages, const std::string& name) {
    GLuint prog = glCreateProgram();
    for (GLuint s : stages) glAttachShader(prog, s);
    glLinkProgram(prog);
    for (GLuint s : stages) glDeleteShader(s);
    GLint ok = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[8192];
        glGetProgramInfoLog(prog, sizeof(log), nullptr, log);
        LOGE("Link error in %s:\n%s", name.c_str(), log);
        glDeleteProgram(prog);
        return 0;
    }
    return prog;
}
}  // namespace

Shader::~Shader() {
    if (program_) glDeleteProgram(program_);
}

Shader& Shader::operator=(Shader&& o) noexcept {
    if (this != &o) {
        if (program_) glDeleteProgram(program_);
        program_ = o.program_;
        cache_ = std::move(o.cache_);
        o.program_ = 0;
    }
    return *this;
}

bool Shader::load(const std::string& vertFile, const std::string& fragFile, const std::vector<std::string>& defines) {
    GLuint vs = compile(GL_VERTEX_SHADER, vertFile, defines);
    GLuint fs = compile(GL_FRAGMENT_SHADER, fragFile, defines);
    if (!vs || !fs) {
        if (vs) glDeleteShader(vs);
        if (fs) glDeleteShader(fs);
        return false;
    }
    GLuint prog = link({vs, fs}, vertFile + "+" + fragFile);
    if (!prog) return false;
    if (program_) glDeleteProgram(program_);
    program_ = prog;
    cache_.clear();
    return true;
}

bool Shader::loadCompute(const std::string& compFile, const std::vector<std::string>& defines) {
    GLuint cs = compile(GL_COMPUTE_SHADER, compFile, defines);
    if (!cs) return false;
    GLuint prog = link({cs}, compFile);
    if (!prog) return false;
    if (program_) glDeleteProgram(program_);
    program_ = prog;
    cache_.clear();
    return true;
}

GLint Shader::loc(const char* name) const {
    auto it = cache_.find(name);
    if (it != cache_.end()) return it->second;
    GLint l = glGetUniformLocation(program_, name);
    cache_.emplace(name, l);
    return l;
}

}  // namespace apex
