// Tarmac and kerbs: scanned asphalt with rubbered-in racing line, tyre marks, painted edge
// lines, the start/finish line and red/white kerbs with ridges.
#include "common.glsl"
#include "lighting.glsl"

in vec3 vRel;
in vec3 vNormal;
in vec4 vTangent;
in vec2 vUV;      // x: lateral offset (m, + right), y: distance along the chunk (m)
in vec4 vColor;   // x: racing line offset, y: half width left, z: half width right,
                  // w: kerb >= 0.5, road < 0.5 (road: braking-zone strength 0..0.45)
in vec3 vLocal;

layout(binding = 1) uniform sampler2D uAsphaltA;
layout(binding = 2) uniform sampler2D uAsphaltN;
layout(binding = 3) uniform sampler2D uAsphaltARM;
layout(binding = 4) uniform sampler2D uGraffiti;   // 8x8 cells of painted words and symbols
uniform float uChunkS;      // distance along the track at the chunk start
uniform float uTrackLength;

out vec4 fragColor;

float hash1(float n) { return fract(sin(n * 127.1 + 311.7) * 43758.5453); }

// Fans' graffiti painted across the tarmac: one candidate per stretch of road, readable when
// approaching (letters' tops point down the road). Returns paint coverage; col is the paint.
float graffiti(float s, float lat, float hw, out vec3 col) {
    col = vec3(0.0);
    const float cellLen = 26.0;
    float best = 0.0;
    float idx = floor(s / cellLen);
    for (float k = -1.0; k <= 0.0; k += 1.0) {   // neighbour cell: long words spill over
        float c = idx + k;
        if (hash1(c * 1.31) > 0.42) continue;                    // ~42% of stretches painted
        float len = mix(3.2, 6.5, hash1(c * 2.17));             // along the road (letter height)
        float wid = min(mix(4.0, 8.5, hash1(c * 3.71)), 2.0 * hw - 0.8);
        float s0 = c * cellLen + mix(len, cellLen - len, hash1(c * 5.13));
        float lat0 = (hash1(c * 7.77) - 0.5) * max(0.0, 2.0 * hw - wid - 0.6);
        float ang = (hash1(c * 9.03) - 0.5) * 0.5;
        vec2 q = vec2(lat - lat0, s - s0);
        q = mat2(cos(ang), -sin(ang), sin(ang), cos(ang)) * q;
        vec2 uv = vec2(q.x / wid + 0.5, 0.5 - q.y / len);
        if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0)))) continue;
        float cellId = floor(hash1(c * 11.3) * 64.0);
        vec2 cell = vec2(mod(cellId, 8.0), floor(cellId / 8.0));
        float a = texture(uGraffiti, (cell + uv) / vec2(8.0, 8.0)).r;
        if (a > best) {
            best = a;
            float pick = hash1(c * 13.7);
            col = pick < 0.62 ? vec3(0.78, 0.78, 0.75) : (pick < 0.85 ? vec3(0.80, 0.62, 0.08)
                                                          : (pick < 0.93 ? vec3(0.65, 0.06, 0.04) : vec3(0.08, 0.22, 0.60)));
            // older paint is greyer
            col = mix(col, vec3(0.35), hash1(c * 17.1) * 0.5);
        }
    }
    return best;
}

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
        float brakeZone = vColor.w / 0.45;
        vec2 uv = vec2(lat, vUV.y) / 3.2;
        vec3 albedo = texture(uAsphaltA, uv).rgb;
        vec3 n = texture(uAsphaltN, uv).xyz * 2.0 - 1.0;
        vec3 arm = texture(uAsphaltARM, uv).rgb;
        float hwSide = lat < 0.0 ? vColor.y : vColor.z;
        // large-scale patches: resurfaced sections, staining
        float patchN = fbm(vec2(s * 0.004, lat * 0.03), 3);
        albedo *= mix(0.95, 1.45, patchN);  // aged, light grey Nordschleife tarmac with resurfaced patches
        // repair patches with sealed seams: rectangles over one lane or the full width
        {
            float pc = floor(s / 47.0);
            if (hash1(pc * 3.3) < 0.35) {
                float p0 = pc * 47.0 + hash1(pc * 4.1) * 20.0, p1 = p0 + mix(4.0, 24.0, hash1(pc * 5.9));
                float lane = hash1(pc * 6.7);
                float l0 = lane < 0.33 ? -hwSide : (lane < 0.66 ? -0.3 : -hwSide);
                float l1 = lane < 0.33 ? 0.3 : (lane < 0.66 ? hwSide : hwSide);
                float d = min(min(s - p0, p1 - s), min(lat - l0, l1 - lat));
                if (d > 0.0) {
                    float fresh = hash1(pc * 8.3);
                    albedo *= fresh < 0.5 ? 0.72 : 1.18;     // newer (darker) or older (bleached) tarmac
                    n.xy *= 0.7;
                }
                float seam = 1.0 - smoothstep(0.02, 0.06, abs(d));
                albedo *= 1.0 - 0.6 * seam * step(-0.1, d);
            }
        }
        // rubbered-in racing line
        float dl = abs(lat - vColor.x);
        float rubber = (1.0 - smoothstep(0.5, 1.7, dl)) * (0.55 + 0.45 * fbm(vec2(s * 0.05, lat * 2.0), 3));
        albedo *= 1.0 - 0.38 * rubber;
        float roughness = clamp(arm.g - 0.08 * rubber, 0.35, 0.95);
        // tyre marks (braking zones / kerb exits): long dark streaks
        float marks = smoothstep(0.62, 0.85, fbm(vec2(lat * 1.6, s * 0.012), 4)) * (1.0 - smoothstep(1.0, 3.0, dl));
        albedo *= 1.0 - 0.35 * marks;
        // braking zones: pairs of black lines from locked/ABS-limited tyres either side of the line
        if (brakeZone > 0.01) {
            float lanes = fbm(vec2((lat - vColor.x) * 5.5, s * 0.015), 3);
            float pair = 1.0 - smoothstep(0.08, 0.22, abs(abs(lat - vColor.x + 0.15 * sin(s * 0.03)) - 0.80));
            float streak = max(pair * 0.8, smoothstep(0.55, 0.8, lanes)) * (1.0 - smoothstep(1.2, 2.4, dl));
            albedo *= 1.0 - 0.55 * streak * brakeZone * (0.6 + 0.4 * valueNoise(vec2(s * 0.2, lat * 3.0)));
        }
        // painted white edge lines
        float hw = lat < 0.0 ? vColor.y : vColor.z;
        float edge = abs(abs(lat) - (hw - 0.22));
        float paint = 1.0 - smoothstep(0.055, 0.075, edge);
        paint *= 0.75 + 0.25 * valueNoise(vec2(s * 0.8, lat * 9.0));
        // start / finish line
        float sw = min(s, uTrackLength - s);
        paint = max(paint, (1.0 - smoothstep(0.30, 0.34, abs(sw - 0.3))) * step(abs(lat), hw - 0.2));
        albedo = mix(albedo, vec3(0.72), paint);
        // graffiti, worn away where the cars run
        vec3 gcol;
        float g = graffiti(s, lat, hwSide, gcol);
        if (sw > 60.0 && g > 0.0) {
            float wear = 0.45 + 0.55 * smoothstep(0.35, 0.75, fbm(vec2(lat * 4.0, s * 0.9), 3));
            wear *= 1.0 - 0.6 * rubber;
            float cov = smoothstep(0.3, 0.6, g) * wear;
            albedo = mix(albedo, gcol, cov);
            roughness = mix(roughness, 0.6, cov);
            n = normalize(mix(n, vec3(0, 0, 1), cov * 0.6));
        }
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
