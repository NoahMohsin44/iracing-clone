// Renders the sky (single-scattering Rayleigh/Mie atmosphere + a procedural cumulus layer and
// a lit ground below the horizon) into one face of a cubemap.
#include "common.glsl"
#include "cubeface.glsl"

in vec2 vUV;
out vec4 fragColor;

uniform int uFace;
uniform int uSize;
uniform vec3 uSun;
uniform float uSunIntensity;
uniform vec3 uSunLight;    // sun irradiance after atmospheric transmittance
uniform float uCloudCover;
uniform float uCloudSeed;

const float Re = 6360e3;
const float Ra = 6420e3;
const vec3 bR = vec3(5.802e-6, 13.558e-6, 33.1e-6);
const vec3 bOz = vec3(0.650e-6, 1.881e-6, 0.085e-6);
const float bMs = 3.996e-6;
const float bMe = 4.44e-6;
const float HR = 8000.0;
const float HM = 1200.0;
const float g = 0.78;

vec2 raySphere(vec3 ro, vec3 rd, float r) {
    float b = dot(ro, rd);
    float c = dot(ro, ro) - r * r;
    float d = b * b - c;
    if (d < 0.0) return vec2(-1.0);
    float s = sqrt(d);
    return vec2(-b - s, -b + s);
}

vec3 opticalToSun(vec3 p) {
    vec2 tl = raySphere(p, uSun, Ra);
    float dl = tl.y / 8.0;
    float odR = 0.0, odM = 0.0;
    for (int j = 0; j < 8; ++j) {
        vec3 pl = p + uSun * (float(j) + 0.5) * dl;
        float hl = length(pl) - Re;
        if (hl < 0.0) return vec3(1e9);
        odR += exp(-hl / HR) * dl;
        odM += exp(-hl / HM) * dl;
    }
    return vec3(odR, odM, 0.0);
}

vec3 atmosphere(vec3 rd, out vec3 transmittance, out float groundT) {
    vec3 ro = vec3(0.0, Re + 550.0, 0.0);
    vec2 ta = raySphere(ro, rd, Ra);
    float tmax = ta.y;
    vec2 tg = raySphere(ro, rd, Re);
    groundT = -1.0;
    if (tg.x > 0.0) {
        tmax = tg.x;
        groundT = tg.x;
    }
    const int N = 24;
    float dt = tmax / float(N);
    float mu = dot(rd, uSun);
    float phaseR = 3.0 / (16.0 * PI) * (1.0 + mu * mu);
    float phaseM = 3.0 / (8.0 * PI) * ((1.0 - g * g) * (1.0 + mu * mu)) /
                   ((2.0 + g * g) * pow(1.0 + g * g - 2.0 * g * mu, 1.5));
    vec3 sumR = vec3(0.0), sumM = vec3(0.0);
    float odR = 0.0, odM = 0.0;
    for (int i = 0; i < N; ++i) {
        vec3 p = ro + rd * (float(i) + 0.5) * dt;
        float h = length(p) - Re;
        float hr = exp(-h / HR) * dt;
        float hm = exp(-h / HM) * dt;
        odR += hr;
        odM += hm;
        vec3 l = opticalToSun(p);
        if (l.x > 1e8) continue;
        vec3 tau = bR * (odR + l.x) + bOz * (odR + l.x) * 0.6 + bMe * (odM + l.y);
        vec3 att = exp(-tau);
        sumR += att * hr;
        sumM += att * hm;
    }
    transmittance = exp(-(bR * odR + bMe * odM));
    return uSunIntensity * (sumR * bR * phaseR + sumM * bMs * phaseM);
}

float cloudNoise(vec2 p) {
    float n = fbm(p, 6);
    float detail = fbm(p * 3.7 + 11.0, 4);
    return n * 0.8 + detail * 0.25;
}

void main() {
    vec2 uv = gl_FragCoord.xy / float(uSize);
    vec3 rd = cubeDir(uFace, uv);
    vec3 trans;
    float groundT;
    vec3 col = atmosphere(rd, trans, groundT);

    if (groundT > 0.0) {
        // ground below the horizon: dark, sun and sky lit forest/fields, seen through the haze
        vec3 albedo = vec3(0.045, 0.06, 0.035);
        float sunUp = max(uSun.y, 0.0);
        vec3 lit = albedo / PI * (uSunLight * sunUp + uSunIntensity * vec3(0.10, 0.13, 0.18) * 0.6);
        col += lit * trans;
    } else if (rd.y > 0.0) {
        // cumulus layer at ~1.8 km
        float t = 1250.0 / max(rd.y, 0.02);
        vec2 p = rd.xz * t / 2600.0 + vec2(uCloudSeed, uCloudSeed * 0.7);
        float n = cloudNoise(p);
        float cover = uCloudCover;
        float dens = smoothstep(1.0 - cover, 1.0 - cover + 0.32, n);
        if (dens > 0.0) {
            // fake self shadowing: sample towards the sun
            vec2 ps = p + normalize(uSun.xz + 1e-4) * 0.06;
            float dsun = smoothstep(1.0 - cover, 1.0 - cover + 0.32, cloudNoise(ps));
            float shade = mix(1.0, 0.45, dsun);
            float silver = pow(saturate(dot(rd, uSun)), 12.0) * 2.5;
            vec3 skyAmb = uSunIntensity * vec3(0.05, 0.065, 0.09);
            vec3 cloudCol = uSunLight * (0.16 * shade + 0.08 * silver) + skyAmb * (0.8 - 0.3 * dens);
            float horizonFade = smoothstep(0.015, 0.14, rd.y);
            float alpha = dens * 0.92 * horizonFade;
            float aerial = 1.0 - exp(-t / 52000.0);
            cloudCol = mix(cloudCol, col, aerial);
            col = mix(col, cloudCol, alpha);
        }
    }
    fragColor = vec4(max(col, vec3(0.0)), 1.0);
}
