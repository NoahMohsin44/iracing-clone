// GPU heightmap terrain: a shared grid mesh is displaced per chunk from the height texture.
#include "common.glsl"

layout(location = 0) in vec3 aPos;    // local grid position (x, z)
layout(location = 4) in vec4 aColor;  // a = skirt vertex

layout(binding = 0) uniform sampler2D uHeight;
layout(binding = 13) uniform sampler2D uLandcover;
layout(binding = 14) uniform sampler2D uTrackDist;

uniform vec2 uChunkOrigin;
uniform float uCellScale;  // metres per grid unit
uniform vec4 uHm;      // x0, z0, res, unused
uniform vec2 uHmSize;  // texels
uniform vec4 uHole;    // xmin, zmin, xmax, zmax: vertices inside are sunk (far terrain under near)
uniform float uCanopyHeight;
uniform float uCanopyStart;
#ifdef SHADOW
uniform mat4 uLightVP;
#endif

out vec3 vRel;
out vec3 vNormal;
out float vCanopy;

vec2 hmUV(vec2 xz) { return ((xz - uHm.xy) / uHm.z + 0.5) / uHmSize; }
float H(vec2 xz) { return textureLod(uHeight, hmUV(xz), 0.0).r; }

void main() {
    vec2 xz = uChunkOrigin + aPos.xz * uCellScale;
    float e = uHm.z;
    float h = H(xz);
    vec3 n = normalize(vec3(H(xz - vec2(e, 0.0)) - H(xz + vec2(e, 0.0)), 2.0 * e,
                            H(xz - vec2(0.0, e)) - H(xz + vec2(0.0, e))));
    vCanopy = 0.0;
#ifdef CANOPY
    float forest = textureLod(uLandcover, hmUV(xz), 0.0).r;
    float d = textureLod(uTrackDist, hmUV(xz), 0.0).r;
    float band = smoothstep(uCanopyStart, uCanopyStart + 45.0, d);
    float c = smoothstep(0.35, 0.75, forest) * band;
    float lump = fbm(xz * 0.045, 3);
    h += c * uCanopyHeight * (0.82 + 0.35 * lump) - (1.0 - c) * 3.0;
    vCanopy = c;
    n = normalize(n + vec3(lump - 0.5, 0.0, fbm(xz * 0.045 + 7.0, 3) - 0.5) * 1.2 * c);
#endif
    if (aColor.a > 0.5) h -= 4.0;  // skirt hides LOD cracks
    if (uHole.z > uHole.x && xz.x > uHole.x && xz.x < uHole.z && xz.y > uHole.y && xz.y < uHole.w) h -= 90.0;
    vec3 rel = vec3(xz.x, h, xz.y) - uCamPos.xyz;
    vRel = rel;
    vNormal = n;
#ifdef SHADOW
    gl_Position = uLightVP * vec4(rel, 1.0);
#else
    gl_Position = uViewProj * vec4(rel, 1.0);
#endif
}
