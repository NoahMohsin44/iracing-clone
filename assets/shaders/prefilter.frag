// GGX prefiltered specular environment (split-sum), importance sampled with PDF-based mip lookup.
#include "common.glsl"
#include "cubeface.glsl"

in vec2 vUV;
out vec4 fragColor;
layout(binding = 0) uniform samplerCube uSrc;
uniform int uFace;
uniform int uSize;
uniform float uRoughness;
uniform float uSrcSize;

float radicalInverse(uint bits) {
    bits = (bits << 16u) | (bits >> 16u);
    bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
    bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
    bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
    bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
    return float(bits) * 2.3283064365386963e-10;
}

vec3 importanceSampleGGX(vec2 xi, vec3 n, float a) {
    float phi = 2.0 * PI * xi.x;
    float cosT = sqrt((1.0 - xi.y) / (1.0 + (a * a - 1.0) * xi.y));
    float sinT = sqrt(1.0 - cosT * cosT);
    vec3 h = vec3(cos(phi) * sinT, sin(phi) * sinT, cosT);
    vec3 up = abs(n.z) < 0.999 ? vec3(0, 0, 1) : vec3(1, 0, 0);
    vec3 tx = normalize(cross(up, n));
    vec3 ty = cross(n, tx);
    return normalize(tx * h.x + ty * h.y + n * h.z);
}

void main() {
    vec2 uv = gl_FragCoord.xy / float(uSize);
    vec3 N = cubeDir(uFace, uv);
    if (uRoughness < 0.01) {
        fragColor = vec4(textureLod(uSrc, N, 0.0).rgb, 1.0);
        return;
    }
    float a = uRoughness * uRoughness;
    const uint COUNT = 96u;
    vec3 sum = vec3(0.0);
    float w = 0.0;
    for (uint i = 0u; i < COUNT; ++i) {
        vec2 xi = vec2(float(i) / float(COUNT), radicalInverse(i));
        vec3 H = importanceSampleGGX(xi, N, a);
        vec3 L = normalize(2.0 * dot(N, H) * H - N);
        float NoL = dot(N, L);
        if (NoL > 0.0) {
            float NoH = saturate(dot(N, H));
            float a2 = a * a;
            float d = NoH * NoH * (a2 - 1.0) + 1.0;
            float D = a2 / (PI * d * d);
            float pdf = D * 0.25;
            float saTexel = 4.0 * PI / (6.0 * uSrcSize * uSrcSize);
            float saSample = 1.0 / (float(COUNT) * pdf + 1e-4);
            float mip = 0.5 * log2(saSample / saTexel) + 1.0;
            sum += textureLod(uSrc, L, max(mip, 0.0)).rgb * NoL;
            w += NoL;
        }
    }
    fragColor = vec4(sum / max(w, 1e-4), 1.0);
}
