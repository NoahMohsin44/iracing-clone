// Terrain / verge shading: grass, forest floor and gravel PBR sets blended by OSM land cover,
// macro variation against tiling, and the distant forest canopy shell.
#include "common.glsl"
#include "lighting.glsl"

in vec3 vRel;
in vec3 vNormal;
#ifdef VERGE
in vec4 vTangent;
in vec2 vUV;
in vec4 vColor;
in vec3 vLocal;
#else
in float vCanopy;
#endif

layout(binding = 1) uniform sampler2D uGrassA;
layout(binding = 2) uniform sampler2D uGrassN;
layout(binding = 3) uniform sampler2D uGrassARM;
layout(binding = 4) uniform sampler2D uForestA;
layout(binding = 5) uniform sampler2D uForestN;
layout(binding = 6) uniform sampler2D uForestARM;
layout(binding = 7) uniform sampler2D uGravelA;
layout(binding = 8) uniform sampler2D uGravelN;
layout(binding = 9) uniform sampler2D uGravelARM;
layout(binding = 13) uniform sampler2D uLandcoverF;
uniform vec4 uHmLand;      // x0, z0, res, unused (landcover grid = near heightmap grid)
uniform vec2 uHmLandSize;

out vec4 fragColor;

vec3 nmap(sampler2D t, vec2 uv) { return texture(t, uv).xyz * 2.0 - 1.0; }

void main() {
    vec3 world = vRel + uCamPos.xyz;
    vec3 N = normalize(vNormal);
    float dist = length(vRel);

    vec2 luv = ((world.xz - uHmLand.xy) / uHmLand.z + 0.5) / uHmLandSize;
    bool inside = luv.x > 0.0 && luv.y > 0.0 && luv.x < 1.0 && luv.y < 1.0;
    float macro = fbm(world.xz * 0.012, 4);
    float micro = valueNoise(world.xz * 0.35);
    // beyond the mapped area: Eifel patchwork of forest and fields from noise
    float farForest = smoothstep(0.46, 0.56, fbm(world.xz * 0.0011 + 3.0, 5));
    vec3 lc = inside ? texture(uLandcoverF, luv).rgb : vec3(farForest, 1.0 - farForest, 0.0);
    float canopy = 0.0;
#ifndef VERGE
    canopy = max(vCanopy, inside ? 0.0 : farForest);
#endif

#ifndef VERGE
    if (canopy > 0.02) {
        // distant forest canopy: dark, lumpy, slightly translucent green
        float var = fbm(world.xz * 0.11, 4);
        vec3 albedo = mix(vec3(0.020, 0.040, 0.016), vec3(0.045, 0.070, 0.028), var);
        albedo = mix(albedo, vec3(0.06, 0.07, 0.03), smoothstep(0.6, 0.9, macro) * 0.5);
        Surface s;
        s.albedo = albedo;
        s.normal = N;
        s.roughness = 0.85;
        s.metallic = 0.0;
        s.ao = mix(0.55, 1.0, var);
        s.translucency = 0.4;
        s.clearcoat = 0.0;
        s.specular = 0.3;
        float sh = sampleShadow(vRel, N);
        vec3 col = shadeSurface(s, vRel, sh);
        fragColor = vec4(applyFog(col, vRel), 1.0);
        return;
    }
#endif

    float forestW = smoothstep(0.35, 0.75, lc.r + (macro - 0.5) * 0.35);
    float builtW = smoothstep(0.45, 0.85, lc.b) * (1.0 - forestW);
    float slope = 1.0 - N.y;
    forestW = max(forestW, smoothstep(0.25, 0.45, slope) * 0.7);
#ifdef VERGE
    forestW *= 0.0;  // verges next to the tarmac are mown grass
    builtW *= 0.0;
#endif

    vec2 uvG = world.xz / 2.6;
    vec2 uvF = world.xz / 3.4;
    vec2 uvR = world.xz / 2.2;
    // second, rotated sample breaks up tiling
    vec2 uvG2 = mat2(0.8, -0.6, 0.6, 0.8) * world.xz / 7.3;

    vec3 aG = mix(texture(uGrassA, uvG).rgb, texture(uGrassA, uvG2).rgb, 0.35);
    vec3 nG = nmap(uGrassN, uvG);
    vec3 mG = texture(uGrassARM, uvG).rgb;
    vec3 aF = texture(uForestA, uvF).rgb;
    vec3 nF = nmap(uForestN, uvF);
    vec3 mF = texture(uForestARM, uvF).rgb;
    vec3 aR = texture(uGravelA, uvR).rgb;
    vec3 nR = nmap(uGravelN, uvR);
    vec3 mR = texture(uGravelARM, uvR).rgb;

    // grass colour variation: drier/yellower patches on farmland, lusher near forests
    // the scanned grass reads a bit dry: push it towards the lush Eifel summer green
    aG = mix(aG, aG * vec3(0.72, 1.05, 0.55), 0.75);
    vec3 tint = mix(vec3(1.0), vec3(1.15, 1.06, 0.78), smoothstep(0.5, 0.85, macro) * (0.3 + 0.5 * lc.g));
    tint *= mix(0.85, 1.12, micro);
    aG *= tint;

    float wG = max(0.0, 1.0 - forestW - builtW);
    vec3 albedo = aG * wG + aF * forestW + aR * builtW;
    vec3 ntex = normalize(nG * wG + nF * forestW + nR * builtW + vec3(0, 0, 1e-3));
    vec3 arm = mG * wG + mF * forestW + mR * builtW;

    // far away the textures would tile visibly: fade to their average colour
    float far = smoothstep(120.0, 900.0, dist);
    vec3 avg = vec3(0.07, 0.12, 0.035) * tint * wG + vec3(0.13, 0.10, 0.07) * forestW + vec3(0.25) * builtW;
    albedo = mix(albedo, avg * mix(0.75, 1.2, macro), far);
    ntex = normalize(mix(ntex, vec3(0, 0, 1), far));

    vec3 T = normalize(vec3(1, 0, 0) - N * N.x);
    vec3 B = normalize(cross(T, N));
    vec3 Nw = normalize(T * ntex.x + B * ntex.y + N * ntex.z);

    Surface s;
    s.albedo = albedo;
    s.normal = Nw;
    s.roughness = clamp(arm.g, 0.45, 1.0);
    s.metallic = 0.0;
    s.ao = mix(arm.r, 1.0, far);
    s.translucency = 0.12 * wG;
    s.clearcoat = 0.0;
    s.specular = 0.5;
    float sh = sampleShadow(vRel, N);
    vec3 col = shadeSurface(s, vRel, sh);
    fragColor = vec4(applyFog(col, vRel), 1.0);
}
