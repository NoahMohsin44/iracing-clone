// GT3 car materials. The painted shell is a single closed mesh; windows, lights, vents and the
// livery are resolved here from body-space position/normal, wheel arches are cut out.
#include "common.glsl"
#include "lighting.glsl"

in vec3 vRel;
in vec3 vNormal;
in vec4 vTangent;
in vec2 vUV;
in vec4 vColor;
in vec3 vLocal;

uniform int uMode;          // 0 opaque, 1 glass (transparent pass)
uniform int uIsBody;
uniform mat3 uWorldToLocal; // body rotation inverse (normals to body space)
uniform vec3 uPaint;
uniform vec3 uPaint2;
uniform vec3 uAccent;
uniform vec4 uArch[2];      // z, y, radius, inner |x|
uniform float uBrakeLight;
uniform float uBrakeGlow;
uniform float uRpmFrac;
uniform float uCockpit;     // camera inside the car

out vec4 fragColor;

const int M_BODY = 0, M_CARBON = 1, M_BLACK = 2, M_GLASS = 3, M_HEAD = 4, M_TAIL = 5, M_CHROME = 6, M_RUBBER = 7,
          M_RIM = 8, M_DISC = 9, M_CALIPER = 10, M_INTERIOR = 11, M_SEAT = 12, M_DISPLAY = 13, M_LINER = 14,
          M_LEDS = 15, M_CAGE = 16, M_HELMET = 17, M_VISOR = 18, M_SUIT = 19, M_TRIM = 20;

float sdSeg(vec2 p, vec2 a, vec2 b) {
    vec2 pa = p - a, ba = b - a;
    float h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
    return length(pa - ba * h);
}

// signed edge function: > 0 on the left of a->b
float edgeF(vec2 p, vec2 a, vec2 b) {
    vec2 d = normalize(b - a);
    return (p.x - a.x) * d.y - (p.y - a.y) * d.x;
}

float sideWindow(vec2 zy) {
    // polygon in (z, y), returns distance inside (negative = outside)
    float e0 = zy.y - 0.36;                                          // bottom
    float e1 = 0.71 - zy.y;                                          // top
    float e2 = edgeF(zy, vec2(-0.51, 0.36), vec2(-0.11, 0.705));     // A-pillar (front)
    float e3 = -edgeF(zy, vec2(0.93, 0.37), vec2(0.46, 0.705));      // C-pillar (rear)
    float bp = abs(zy.x - 0.245) - 0.045;                            // B-pillar
    return min(min(min(e0, e1), min(e2, e3)), bp);
}

int classifyBody(vec3 p, vec3 n, out float trim) {
    trim = 1.0;
    float ax = abs(p.x);
    // windscreen
    if (p.z > -0.80 && p.z < -0.11 && p.y > 0.235 && n.y > 0.25) {
        float edge = min(min(p.z + 0.80, -0.11 - p.z), (0.50 - max(0.0, p.y - 0.55) * 0.3) - ax);
        if (edge > 0.0) {
            trim = edge;
            return edge < 0.025 ? M_TRIM : M_GLASS;
        }
    }
    // side windows
    if (abs(n.x) > 0.4) {
        float d = sideWindow(p.zy);
        if (d > 0.0) {
            trim = d;
            return d < 0.018 ? M_TRIM : M_GLASS;
        }
    }
    // rear window
    if (p.z > 0.66 && p.z < 1.30 && n.y > 0.3 && ax < 0.40 && p.y > 0.37) {
        float edge = min(min(p.z - 0.66, 1.30 - p.z), 0.40 - ax);
        trim = edge;
        return edge < 0.02 ? M_TRIM : M_GLASS;
    }
    if (p.z < -1.90 && ax > 0.38 && ax < 0.90 && p.y > -0.10 && p.y < 0.07 && n.z < -0.1) return M_HEAD;
    if (p.z > 2.06 && ax > 0.30 && ax < 0.93 && p.y > 0.07 && p.y < 0.21) return M_TAIL;
    if (p.z < -2.12 && p.y < -0.12 && n.z < -0.15) return M_BLACK;
    if (p.z > -1.75 && p.z < -1.15 && ax > 0.17 && ax < 0.50 && n.y > 0.6) return M_BLACK;
    if (p.z > -1.00 && p.z < -0.74 && p.y > -0.19 && p.y < 0.06 && abs(n.x) > 0.6) return M_BLACK;
    if (p.y < -0.27 || (p.z > 2.0 && p.y < -0.05)) return M_CARBON;
    return M_BODY;
}

vec3 livery(vec3 p, vec3 n, inout float metal) {
    vec3 c = uPaint;
    float ax = abs(p.x);
    // twin stripes over the top
    if (n.y > 0.15 && ax > 0.07 && ax < 0.21) c = uPaint2;
    if (n.y > 0.15 && abs(ax - 0.07) < 0.008) c = uAccent;
    // side sweep
    float sweep = -0.11 + (p.z + 2.4) * 0.035;
    if (abs(n.x) > 0.3 && p.y < sweep) c = uPaint2;
    if (abs(n.x) > 0.3 && abs(p.y - sweep) < 0.012) c = uAccent;
    // door roundel with the race number
    if (abs(n.x) > 0.55) {
        vec2 q = (p.zy - vec2(0.05, 0.06)) / 0.19;
        if (p.x > 0.0) q.x = -q.x;  // read the number front-to-back on both sides
        float r = length(q);
        if (r < 1.0) {
            c = vec3(0.85);
            if (r > 0.9) c = uAccent;
            float d = min(sdSeg(q, vec2(-0.42, 0.52), vec2(0.42, 0.52)), sdSeg(q, vec2(0.42, 0.52), vec2(-0.08, -0.62)));
            if (d < 0.11) c = vec3(0.02);
        }
    }
    return c;
}

void main() {
    vec3 N = normalize(vNormal);
    vec3 world = vRel + uCamPos.xyz;
    int mat = int(vColor.w + 0.5);
    float trim = 1.0;
    vec3 nl = normalize(uWorldToLocal * N);
    vec3 p = vLocal;

    if (uIsBody == 1) {
        for (int i = 0; i < 2; ++i) {
            vec4 a = uArch[i];
            if (abs(p.x) > a.w && length(vec2(p.z - a.x, p.y - a.y)) < a.z) discard;
        }
        mat = classifyBody(p, nl, trim);
    }
    bool glass = mat == M_GLASS;
    if (uMode == 1 && !glass) discard;
    if (uMode == 0 && mat == M_GLASS) discard;

    bool back = !gl_FrontFacing && uIsBody == 1;
    if (back) N = -N;

    Surface s;
    s.albedo = vec3(0.05);
    s.normal = N;
    s.roughness = 0.5;
    s.metallic = 0.0;
    s.ao = 1.0;
    s.translucency = 0.0;
    s.clearcoat = 0.0;
    s.specular = 0.5;
    vec3 emissive = vec3(0.0);

    if (back) {
        // inside of the shell: bare carbon / painted interior
        s.albedo = vec3(0.035);
        s.roughness = 0.6;
    } else if (mat == M_BODY) {
        float metal = 0.0;
        s.albedo = livery(p, nl, metal);
        s.roughness = 0.32;
        s.clearcoat = 1.0;
        // a little grime towards the sills and the rear
        float grime = smoothstep(0.0, -0.3, p.y) * 0.35 + smoothstep(1.6, 2.3, p.z) * 0.2;
        grime *= 0.6 + 0.4 * fbm(p.xz * 9.0 + p.y * 4.0, 3);
        s.albedo = mix(s.albedo, vec3(0.12, 0.11, 0.10), grime * 0.5);
        s.roughness = mix(s.roughness, 0.7, grime * 0.6);
        s.clearcoat *= 1.0 - grime * 0.5;
    } else if (mat == M_CARBON) {
        vec3 an = abs(nl);
        vec2 uv = an.x > an.y && an.x > an.z ? p.zy : (an.y > an.z ? p.xz : p.xy);
        vec2 g = uv * 85.0;
        float twill = step(0.5, fract((floor(g.x) + floor(g.y)) * 0.25 + fract(g.x) * 0.25));
        s.albedo = mix(vec3(0.018), vec3(0.045), twill);
        s.roughness = mix(0.28, 0.4, twill);
        s.clearcoat = 0.9;
    } else if (mat == M_BLACK || mat == M_TRIM) {
        s.albedo = vec3(0.012);
        s.roughness = mat == M_TRIM ? 0.4 : 0.75;
        if (mat == M_BLACK) {
            // honeycomb-ish mesh / louvres
            float m = step(0.55, fract(p.z * 34.0)) * step(0.5, fract(p.x * 34.0 + p.y * 34.0));
            s.albedo *= 0.6 + 0.8 * m;
            s.ao = 0.6 + 0.4 * m;
        }
    } else if (mat == M_GLASS || mat == M_VISOR) {
        s.albedo = vec3(0.0);
        s.roughness = 0.03;
        s.specular = 0.5;
    } else if (mat == M_HEAD) {
        s.albedo = vec3(0.75);
        s.metallic = 1.0;
        s.roughness = 0.12;
        float drl = 1.0 - smoothstep(0.008, 0.014, abs(p.y - 0.045));
        emissive = vec3(1.0, 0.97, 0.92) * (drl * 9.0 + 0.4);
    } else if (mat == M_TAIL) {
        s.albedo = vec3(0.25, 0.01, 0.01);
        s.roughness = 0.08;
        float strip = 1.0 - smoothstep(0.01, 0.02, abs(p.y - 0.14));
        emissive = vec3(1.0, 0.02, 0.01) * (0.6 + strip * 2.5 + uBrakeLight * (5.0 + strip * 12.0));
    } else if (mat == M_CHROME) {
        float heat = smoothstep(0.96, 1.03, abs(p.x));
        s.albedo = mix(vec3(0.75, 0.72, 0.68), vec3(0.45, 0.42, 0.62), heat);
        s.metallic = 1.0;
        s.roughness = 0.22;
    } else if (mat == M_RUBBER) {
        float tread = step(0.25, vUV.y) * step(vUV.y, 0.75);
        s.albedo = vec3(0.028);
        s.roughness = mix(0.72, 0.9, tread);
        s.specular = 0.3;
    } else if (mat == M_RIM) {
        s.albedo = vec3(0.10, 0.10, 0.11);
        s.metallic = 1.0;
        s.roughness = 0.28;
    } else if (mat == M_DISC) {
        float r = length(vLocal.yz);
        float slots = step(0.85, fract(atan(vLocal.z, vLocal.y) * 6.0 + r * 18.0));
        s.albedo = vec3(0.32, 0.31, 0.30) * (1.0 - slots * 0.5);
        s.metallic = 1.0;
        s.roughness = 0.45;
        float glowT = clamp(uBrakeGlow, 0.0, 1.0);
        emissive = vec3(1.0, 0.28, 0.04) * glowT * glowT * 9.0 * smoothstep(0.11, 0.17, r);
    } else if (mat == M_CALIPER) {
        s.albedo = vec3(0.55, 0.025, 0.02);
        s.roughness = 0.3;
        s.clearcoat = 0.6;
    } else if (mat == M_INTERIOR) {
        s.albedo = vec3(0.035);
        s.roughness = 0.65;
    } else if (mat == M_SEAT) {
        s.albedo = vec3(0.045, 0.04, 0.042);
        s.roughness = 0.95;
    } else if (mat == M_DISPLAY) {
        s.albedo = vec3(0.01);
        s.roughness = 0.1;
        float bar = step(fract(vUV.x * 1.0), uRpmFrac);
        emissive = mix(vec3(0.02, 0.05, 0.09), vec3(0.15, 0.55, 0.25), bar * step(0.6, vUV.y)) * 1.5;
    } else if (mat == M_LINER) {
        s.albedo = vec3(0.015);
        s.roughness = 0.9;
        s.ao = 0.5;
    } else if (mat == M_LEDS) {
        float idx = vColor.z;
        float thr = 0.66 + idx * 0.033;
        vec3 col = idx < 0.45 ? vec3(0.1, 1.0, 0.15) : (idx < 0.8 ? vec3(1.0, 0.12, 0.05) : vec3(0.15, 0.3, 1.0));
        bool flash = uRpmFrac > 0.985 && fract(uCamPos.w * 8.0) < 0.5;
        float on = (uRpmFrac > thr || flash) ? 1.0 : 0.0;
        if (flash) col = vec3(0.15, 0.3, 1.0);
        s.albedo = col * 0.1;
        s.roughness = 0.2;
        emissive = col * on * 6.0;
    } else if (mat == M_CAGE) {
        s.albedo = vec3(0.30, 0.31, 0.32);
        s.roughness = 0.45;
        s.metallic = 0.3;
    } else if (mat == M_HELMET) {
        s.albedo = uPaint2;
        s.roughness = 0.25;
        s.clearcoat = 1.0;
    } else if (mat == M_SUIT) {
        s.albedo = uPaint * 0.6 + 0.03;
        s.roughness = 0.9;
    }

    float sh = sampleShadow(vRel, N);
    vec3 col = shadeSurface(s, vRel, sh) + emissive;
    col = applyFog(col, vRel);

    if (uMode == 1) {
        // tinted glass: reflections on top, see-through underneath (premultiplied alpha)
        vec3 V = normalize(-vRel);
        float NoV = abs(dot(N, V));
        float fres = 0.04 + 0.96 * pow(1.0 - NoV, 5.0);
        float alpha = uCockpit > 0.5 ? (0.10 + 0.5 * fres) : (0.55 + 0.4 * fres);
        if (mat == M_VISOR) alpha = 0.85;
        vec3 tint = vec3(0.01, 0.012, 0.014);
        fragColor = vec4(col + tint * alpha, alpha);
        return;
    }
    fragColor = vec4(col, 1.0);
}
