#include "common.glsl"

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUV;
layout(location = 4) in vec4 aColor;   // r foliage, g sway weight, b ambient occlusion
layout(location = 5) in vec4 iPosScale;
layout(location = 6) in vec4 iRotTint;  // cos, sin, tint, unused
#ifdef SHADOW
uniform mat4 uLightVP;
#endif

out vec3 vRel;
out vec3 vNormal;
out vec2 vUV;
out vec4 vColor;
out float vTint;

void main() {
    float c = iRotTint.x, s = iRotTint.y, sc = iPosScale.w;
    vec3 p = aPos * sc;
    p = vec3(c * p.x - s * p.z, p.y, s * p.x + c * p.z);
    vec3 n = vec3(c * aNormal.x - s * aNormal.z, aNormal.y, s * aNormal.x + c * aNormal.z);
    float t = uMisc.x;
    float ph = dot(iPosScale.xz, vec2(0.071, 0.113));
    float sway = aColor.g * (sin(t * 1.1 + ph) * 0.7 + sin(t * 2.9 + ph * 1.7) * 0.3) * 0.18;
    p.x += sway;
    p.z += sway * 0.55;
    vec3 rel = iPosScale.xyz + p - uCamPos.xyz;
    vRel = rel;
    vNormal = n;
    vUV = aUV;
    vColor = aColor;
    vTint = iRotTint.z;
#ifdef SHADOW
    gl_Position = uLightVP * vec4(rel, 1.0);
#else
    gl_Position = uViewProj * vec4(rel, 1.0);
#endif
}
