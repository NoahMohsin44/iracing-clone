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
uniform int uMode;  // 0 plain, 1 armco (galvanised, corrugated), 2 vertex colour, 3 scenery (see Scenery.h)
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
    } else if (uMode == 3) {
        // vColor.w: 0 plain, 1 house wall, 2 industrial wall, 3 flat roof, 4 tiled roof, 5 concrete
        int kind = int(vColor.w + 0.5);
        float grime = fbm(world.xz * 0.35 + world.y * 0.2, 3);
        s.albedo = vColor.rgb * mix(0.85, 1.05, grime);
        s.roughness = 0.85;
        vec2 w = vUV;  // (metres along the wall, metres above the ground)
        if (kind == 1) {
            // two storeys of windows with white frames, door-height gap at the bottom
            vec2 cell = vec2(fract(w.x / 2.7), fract(w.y / 2.8));
            float floorIdx = floor(w.y / 2.8);
            bool win = abs(cell.x - 0.5) < 0.22 && cell.y > 0.32 && cell.y < 0.82 && floorIdx >= 0.0 && floorIdx < 2.0;
            bool frame = abs(cell.x - 0.5) < 0.26 && cell.y > 0.28 && cell.y < 0.86 && floorIdx >= 0.0 && floorIdx < 2.0;
            if (frame) s.albedo = vec3(0.85);
            if (win) {
                s.albedo = vec3(0.02, 0.025, 0.03);
                s.roughness = 0.08;
                s.specular = 0.9;
            }
            // dark plinth
            if (w.y < 0.4) s.albedo *= 0.55;
        } else if (kind == 2) {
            // profiled cladding with a band of ribbon windows
            s.albedo *= 0.9 + 0.1 * step(0.5, fract(w.x * 3.0));
            s.metallic = 0.4;
            s.roughness = 0.5;
            if (w.y > 3.2 && w.y < 4.4 && fract(w.x / 6.0) > 0.08) {
                s.albedo = vec3(0.03, 0.035, 0.04);
                s.roughness = 0.1;
                s.metallic = 0.0;
            }
        } else if (kind == 3) {
            s.roughness = 0.95;
        } else if (kind == 4) {
            // tile courses running across the slope
            float course = fract(w.y * 3.2);
            s.albedo *= 0.75 + 0.35 * smoothstep(0.0, 0.8, course);
            s.roughness = 0.7;
        } else if (kind == 5) {
            s.albedo *= mix(0.8, 1.05, fbm(world.xz * 2.0 + world.y * 3.0, 3));
            s.roughness = 0.9;
        }
    }
    float sh = sampleShadow(vRel, N);
    vec3 col = shadeSurface(s, vRel, sh);
    fragColor = vec4(applyFog(col, vRel), 1.0);
}
