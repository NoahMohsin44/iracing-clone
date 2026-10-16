// Tarmac and kerbs: scanned asphalt with rubbered-in racing line, tyre marks, painted edge
// lines, the start/finish line and red/white kerbs with ridges.
#include "common.glsl"
#include "lighting.glsl"

in vec3 vRel;
in vec3 vNormal;
in vec4 vTangent;
in vec2 vUV;      // x: lateral offset (m, + right), y: distance along the chunk (m)
in vec4 vColor;   // x: racing line offset, y: half width left, z: half width right, w: material (0 road, 1 kerb)
in vec3 vLocal;

layout(binding = 1) uniform sampler2D uAsphaltA;
layout(binding = 2) uniform sampler2D uAsphaltN;
layout(binding = 3) uniform sampler2D uAsphaltARM;
uniform float uChunkS;      // distance along the track at the chunk start
uniform float uTrackLength;

out vec4 fragColor;

void main() {
    vec3 N = normalize(vNormal);
    vec3 T = normalize(vTangent.xyz - N * dot(N, vTangent.xyz));
    vec3 B = cross(N, T) * vTangent.w;
    float lat = vUV.x;
    float s = uChunkS + vUV.y;
    vec3 world = vRel + uCamPos.xyz;
    float dist = length(vRel);

    Surface surf;
    surf.metallic = 0.0;
    surf.translucency = 0.0;
    surf.clearcoat = 0.0;
    surf.specular = 0.5;

    if (vColor.w < 0.5) {
        vec2 uv = vec2(lat, vUV.y) / 3.2;
        vec3 albedo = texture(uAsphaltA, uv).rgb;
        vec3 n = texture(uAsphaltN, uv).xyz * 2.0 - 1.0;
        vec3 arm = texture(uAsphaltARM, uv).rgb;
        // large-scale patches: resurfaced sections, staining
        float patchN = fbm(vec2(s * 0.004, lat * 0.03), 3);
        albedo *= mix(0.78, 1.18, patchN);
        // rubbered-in racing line
        float dl = abs(lat - vColor.x);
        float rubber = (1.0 - smoothstep(0.5, 1.7, dl)) * (0.55 + 0.45 * fbm(vec2(s * 0.05, lat * 2.0), 3));
        albedo *= 1.0 - 0.38 * rubber;
        float roughness = clamp(arm.g - 0.08 * rubber, 0.35, 0.95);
        // tyre marks (braking zones / kerb exits): long dark streaks
        float marks = smoothstep(0.62, 0.85, fbm(vec2(lat * 1.6, s * 0.012), 4)) * (1.0 - smoothstep(1.0, 3.0, dl));
        albedo *= 1.0 - 0.35 * marks;
        // painted white edge lines
        float hw = lat < 0.0 ? vColor.y : vColor.z;
        float edge = abs(abs(lat) - (hw - 0.22));
        float paint = 1.0 - smoothstep(0.055, 0.075, edge);
        paint *= 0.75 + 0.25 * valueNoise(vec2(s * 0.8, lat * 9.0));
        // start / finish line
        float sw = min(s, uTrackLength - s);
        paint = max(paint, (1.0 - smoothstep(0.30, 0.34, abs(sw - 0.3))) * step(abs(lat), hw - 0.2));
        albedo = mix(albedo, vec3(0.72), paint);
        roughness = mix(roughness, 0.55, paint);
        n = normalize(mix(n, vec3(0, 0, 1), paint * 0.8 + smoothstep(60.0, 400.0, dist) * 0.6));
        surf.albedo = albedo;
        surf.normal = normalize(T * n.x + B * n.y + N * n.z);
        surf.roughness = roughness;
        surf.ao = arm.r;
    } else {
        // kerb: 1 m red/white blocks with transverse ridges
        float block = mod(floor(s / 1.0), 2.0);
        vec3 red = vec3(0.50, 0.025, 0.02);
        vec3 white = vec3(0.70, 0.70, 0.68);
        vec3 albedo = mix(red, white, block);
        float dirt = fbm(world.xz * 0.6, 3);
        albedo *= mix(0.75, 1.0, dirt);
        float ridge = sin(s * 2.0 * PI / 0.6);
        vec3 n = normalize(vec3(0.0, ridge * 0.35, 1.0));
        surf.albedo = albedo;
        surf.normal = normalize(T * n.x + B * n.y + N * n.z);
        surf.roughness = 0.5 + 0.2 * dirt;
        surf.ao = 1.0;
    }
    float sh = sampleShadow(vRel, N);
    vec3 col = shadeSurface(surf, vRel, sh);
    fragColor = vec4(applyFog(col, vRel), 1.0);
}
