// Generic PBR object (armco barriers, posts, signs) with optional albedo map.
#include "common.glsl"
#include "lighting.glsl"

in vec3 vRel;
in vec3 vNormal;
in vec4 vTangent;
in vec2 vUV;
in vec4 vColor;
in vec3 vLocal;

uniform vec3 uAlbedo;
uniform float uRoughness;
uniform float uMetallic;
uniform int uMode;  // 0 plain, 1 armco (galvanised, corrugated), 2 vertex colour
out vec4 fragColor;

void main() {
    vec3 N = normalize(vNormal);
    if (!gl_FrontFacing) N = -N;
    vec3 world = vRel + uCamPos.xyz;
    Surface s;
    s.albedo = uAlbedo;
    s.roughness = uRoughness;
    s.metallic = uMetallic;
    s.normal = N;
    s.ao = 1.0;
    s.translucency = 0.0;
    s.clearcoat = 0.0;
    s.specular = 0.5;
    if (uMode == 1) {
        float n = fbm(world.xz * 0.9 + world.y, 3);
        float stain = smoothstep(0.55, 0.8, fbm(world.xz * 0.05, 3));
        s.albedo = mix(uAlbedo, uAlbedo * vec3(0.75, 0.7, 0.62), stain) * mix(0.85, 1.1, n);
        s.roughness = clamp(uRoughness + (n - 0.5) * 0.3 + stain * 0.25, 0.2, 0.9);
        s.metallic = mix(uMetallic, 0.3, stain);
        // W-beam corrugation along the rail (vUV.y = height across the profile)
        float w = sin(vUV.y * 2.0 * PI * 2.0);
        vec3 T = normalize(vTangent.xyz);
        vec3 B = normalize(cross(N, T));
        s.normal = normalize(N + B * w * 0.45);
    } else if (uMode == 2) {
        s.albedo = vColor.rgb;
    }
    float sh = sampleShadow(vRel, N);
    vec3 col = shadeSurface(s, vRel, sh);
    fragColor = vec4(applyFog(col, vRel), 1.0);
}
