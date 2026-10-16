#include "common.glsl"

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUV;
layout(location = 3) in vec4 aTangent;
layout(location = 4) in vec4 aColor;

uniform mat4 uModel;  // object -> camera-relative world
#ifdef SHADOW
uniform mat4 uLightVP;
#endif

out vec3 vRel;
out vec3 vNormal;
out vec4 vTangent;
out vec2 vUV;
out vec4 vColor;
out vec3 vLocal;

void main() {
    vec4 rel = uModel * vec4(aPos, 1.0);
    mat3 nm = mat3(uModel);
    vRel = rel.xyz;
    vNormal = normalize(nm * aNormal);
    vTangent = vec4(normalize(nm * aTangent.xyz), aTangent.w);
    vUV = aUV;
    vColor = aColor;
    vLocal = aPos;
#ifdef SHADOW
    gl_Position = uLightVP * rel;
#else
    gl_Position = uViewProj * rel;
#endif
}
