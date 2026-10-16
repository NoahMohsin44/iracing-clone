#pragma once
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "gfx/GLUtil.h"

namespace apex {

struct Vertex {
    glm::vec3 pos{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    glm::vec2 uv{0.0f};
    glm::vec4 tangent{1.0f, 0.0f, 0.0f, 1.0f};  // xyz tangent, w = bitangent sign
    glm::vec4 color{1.0f};                       // free per-vertex data (material id, masks...)
};

struct MeshData {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    uint32_t addVertex(const Vertex& v) {
        vertices.push_back(v);
        return static_cast<uint32_t>(vertices.size() - 1);
    }
    void addTri(uint32_t a, uint32_t b, uint32_t c) {
        indices.push_back(a);
        indices.push_back(b);
        indices.push_back(c);
    }
    void addQuad(uint32_t a, uint32_t b, uint32_t c, uint32_t d) {  // a-b-c-d counter-clockwise
        addTri(a, b, c);
        addTri(a, c, d);
    }
    void append(const MeshData& other, const glm::mat4& xf = glm::mat4(1.0f));
    void computeNormals();
    void computeTangents();
    glm::vec3 boundsMin() const;
    glm::vec3 boundsMax() const;
};

// GPU mesh with an optional per-instance buffer (attribute locations 5..8 as vec4s).
class GpuMesh {
public:
    GpuMesh() = default;
    ~GpuMesh() { release(); }
    GpuMesh(const GpuMesh&) = delete;
    GpuMesh& operator=(const GpuMesh&) = delete;
    GpuMesh(GpuMesh&& o) noexcept { *this = std::move(o); }
    GpuMesh& operator=(GpuMesh&& o) noexcept;

    void upload(const MeshData& data);
    void setInstanceBuffer(GLuint buffer, int vec4PerInstance);
    void draw() const;
    void drawInstanced(GLsizei count, GLuint baseInstance = 0) const;
    void drawRange(uint32_t firstIndex, uint32_t count) const;
    void release();
    GLuint vao() const { return vao_; }
    uint32_t indexCount() const { return indexCount_; }
    bool valid() const { return vao_ != 0; }
    glm::vec3 boundsMin{0.0f}, boundsMax{0.0f};

private:
    GLuint vao_ = 0, vbo_ = 0, ebo_ = 0;
    uint32_t indexCount_ = 0;
};

}  // namespace apex
