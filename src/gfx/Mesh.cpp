#include "gfx/Mesh.h"

#include <algorithm>
#include <cstddef>

namespace apex {

void MeshData::append(const MeshData& other, const glm::mat4& xf) {
    const uint32_t base = static_cast<uint32_t>(vertices.size());
    const glm::mat3 nx = glm::transpose(glm::inverse(glm::mat3(xf)));
    for (Vertex v : other.vertices) {
        v.pos = glm::vec3(xf * glm::vec4(v.pos, 1.0f));
        v.normal = glm::normalize(nx * v.normal);
        glm::vec3 t = glm::mat3(xf) * glm::vec3(v.tangent);
        if (glm::dot(t, t) > 1e-12f) t = glm::normalize(t);
        v.tangent = glm::vec4(t, v.tangent.w);
        vertices.push_back(v);
    }
    for (uint32_t i : other.indices) indices.push_back(base + i);
}

void MeshData::computeNormals() {
    for (auto& v : vertices) v.normal = glm::vec3(0.0f);
    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        Vertex& a = vertices[indices[i]];
        Vertex& b = vertices[indices[i + 1]];
        Vertex& c = vertices[indices[i + 2]];
        glm::vec3 n = glm::cross(b.pos - a.pos, c.pos - a.pos);
        a.normal += n;
        b.normal += n;
        c.normal += n;
    }
    for (auto& v : vertices) {
        float l = glm::length(v.normal);
        v.normal = l > 1e-12f ? v.normal / l : glm::vec3(0, 1, 0);
    }
}

void MeshData::computeTangents() {
    std::vector<glm::vec3> tan(vertices.size(), glm::vec3(0.0f)), bit(vertices.size(), glm::vec3(0.0f));
    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        uint32_t ia = indices[i], ib = indices[i + 1], ic = indices[i + 2];
        const Vertex &a = vertices[ia], &b = vertices[ib], &c = vertices[ic];
        glm::vec3 e1 = b.pos - a.pos, e2 = c.pos - a.pos;
        glm::vec2 d1 = b.uv - a.uv, d2 = c.uv - a.uv;
        float det = d1.x * d2.y - d2.x * d1.y;
        if (std::abs(det) < 1e-12f) continue;
        float r = 1.0f / det;
        glm::vec3 t = (e1 * d2.y - e2 * d1.y) * r;
        glm::vec3 bt = (e2 * d1.x - e1 * d2.x) * r;
        for (uint32_t k : {ia, ib, ic}) {
            tan[k] += t;
            bit[k] += bt;
        }
    }
    for (size_t i = 0; i < vertices.size(); ++i) {
        glm::vec3 n = vertices[i].normal;
        glm::vec3 t = tan[i] - n * glm::dot(n, tan[i]);
        if (glm::dot(t, t) < 1e-12f) {
            t = std::abs(n.x) < 0.9f ? glm::cross(n, glm::vec3(1, 0, 0)) : glm::cross(n, glm::vec3(0, 0, 1));
        }
        t = glm::normalize(t);
        float w = glm::dot(glm::cross(n, t), bit[i]) < 0.0f ? -1.0f : 1.0f;
        vertices[i].tangent = glm::vec4(t, w);
    }
}

glm::vec3 MeshData::boundsMin() const {
    glm::vec3 m(1e30f);
    for (auto& v : vertices) m = glm::min(m, v.pos);
    return m;
}

glm::vec3 MeshData::boundsMax() const {
    glm::vec3 m(-1e30f);
    for (auto& v : vertices) m = glm::max(m, v.pos);
    return m;
}

GpuMesh& GpuMesh::operator=(GpuMesh&& o) noexcept {
    if (this != &o) {
        release();
        vao_ = o.vao_;
        vbo_ = o.vbo_;
        ebo_ = o.ebo_;
        indexCount_ = o.indexCount_;
        boundsMin = o.boundsMin;
        boundsMax = o.boundsMax;
        o.vao_ = o.vbo_ = o.ebo_ = 0;
        o.indexCount_ = 0;
    }
    return *this;
}

void GpuMesh::upload(const MeshData& data) {
    release();
    if (data.indices.empty()) return;
    glCreateVertexArrays(1, &vao_);
    glCreateBuffers(1, &vbo_);
    glCreateBuffers(1, &ebo_);
    glNamedBufferStorage(vbo_, data.vertices.size() * sizeof(Vertex), data.vertices.data(), 0);
    glNamedBufferStorage(ebo_, data.indices.size() * sizeof(uint32_t), data.indices.data(), 0);
    glVertexArrayVertexBuffer(vao_, 0, vbo_, 0, sizeof(Vertex));
    glVertexArrayElementBuffer(vao_, ebo_);
    auto attrib = [&](GLuint idx, int comps, size_t offset) {
        glEnableVertexArrayAttrib(vao_, idx);
        glVertexArrayAttribFormat(vao_, idx, comps, GL_FLOAT, GL_FALSE, static_cast<GLuint>(offset));
        glVertexArrayAttribBinding(vao_, idx, 0);
    };
    attrib(0, 3, offsetof(Vertex, pos));
    attrib(1, 3, offsetof(Vertex, normal));
    attrib(2, 2, offsetof(Vertex, uv));
    attrib(3, 4, offsetof(Vertex, tangent));
    attrib(4, 4, offsetof(Vertex, color));
    indexCount_ = static_cast<uint32_t>(data.indices.size());
    boundsMin = data.boundsMin();
    boundsMax = data.boundsMax();
}

void GpuMesh::setInstanceBuffer(GLuint buffer, int vec4PerInstance) {
    glVertexArrayVertexBuffer(vao_, 1, buffer, 0, static_cast<GLsizei>(vec4PerInstance * 16));
    glVertexArrayBindingDivisor(vao_, 1, 1);
    for (int i = 0; i < vec4PerInstance; ++i) {
        GLuint idx = 5 + i;
        glEnableVertexArrayAttrib(vao_, idx);
        glVertexArrayAttribFormat(vao_, idx, 4, GL_FLOAT, GL_FALSE, i * 16);
        glVertexArrayAttribBinding(vao_, idx, 1);
    }
}

void GpuMesh::draw() const {
    if (!vao_) return;
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indexCount_), GL_UNSIGNED_INT, nullptr);
}

void GpuMesh::drawInstanced(GLsizei count, GLuint baseInstance) const {
    if (!vao_ || count <= 0) return;
    glBindVertexArray(vao_);
    glDrawElementsInstancedBaseInstance(GL_TRIANGLES, static_cast<GLsizei>(indexCount_), GL_UNSIGNED_INT, nullptr,
                                        count, baseInstance);
}

void GpuMesh::drawRange(uint32_t firstIndex, uint32_t count) const {
    if (!vao_ || count == 0) return;
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(count), GL_UNSIGNED_INT,
                   reinterpret_cast<const void*>(static_cast<uintptr_t>(firstIndex) * sizeof(uint32_t)));
}

void GpuMesh::release() {
    if (vao_) glDeleteVertexArrays(1, &vao_);
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (ebo_) glDeleteBuffers(1, &ebo_);
    vao_ = vbo_ = ebo_ = 0;
    indexCount_ = 0;
}

}  // namespace apex
