// Trees: alpha-tested foliage cards (alpha to coverage with MSAA), translucent leaves, bark.
#include "common.glsl"
#include "lighting.glsl"

in vec3 vRel;
in vec3 vNormal;
in vec2 vUV;
in vec4 vColor;
in float vTint;

layout(binding = 0) uniform sampler2D uFoliage;
layout(binding = 1) uniform sampler2D uBark;
uniform int uAlphaToCoverage;
out vec4 fragColor;

void main() {
    vec3 N = normalize(vNormal);
    Surface s;
    s.metallic = 0.0;
    s.clearcoat = 0.0;
    s.specular = 0.35;
    float alpha = 1.0;
    if (vColor.r > 0.5) {
        vec4 f = texture(uFoliage, vUV);
        if (uAlphaToCoverage == 1) {
            alpha = (f.a - 0.5) / max(fwidth(f.a), 1e-4) + 0.5;
            if (alpha <= 0.0) discard;
        } else if (f.a < 0.5) {
            discard;
        }
        vec3 tint = mix(vec3(0.88, 0.95, 0.85), vec3(1.12, 1.05, 0.9), vTint);
        s.albedo = f.rgb * tint;
        s.normal = N;
        s.roughness = 0.72;
        s.ao = vColor.b;
        s.translucency = 0.55;
    } else {
        s.albedo = texture(uBark, vUV).rgb * vec3(1.05, 0.98, 0.9);
        s.normal = gl_FrontFacing ? N : -N;
        s.roughness = 0.92;
        s.ao = vColor.b;
        s.translucency = 0.0;
    }
    float sh = sampleShadow(vRel, N);
    vec3 col = shadeSurface(s, vRel, sh);
    fragColor = vec4(applyFog(col, vRel), clamp(alpha, 0.0, 1.0));
}
