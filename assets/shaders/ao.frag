// Screen-space ambient occlusion (Alchemy-style) at half resolution from linear view distance.
#include "common.glsl"
in vec2 vUV;
out float fragAO;
layout(binding = 0) uniform sampler2D uDist;
uniform vec2 uTexel;      // 1 / half-res size
uniform float uRadius;    // metres

vec3 viewPos(vec2 uv) {
    float d = textureLod(uDist, uv, 0.0).r;
    vec2 ndc = uv * 2.0 - 1.0;
    return vec3(ndc.x * d / uProj[0][0], ndc.y * d / uProj[1][1], -d);
}

void main() {
    float dist = textureLod(uDist, vUV, 0.0).r;
    if (dist > 5000.0) {
        fragAO = 1.0;
        return;
    }
    vec3 P = viewPos(vUV);
    // normal from the neighbour with the smaller depth step on each axis (avoids silhouette smear)
    vec3 px = viewPos(vUV + vec2(uTexel.x, 0.0)) - P, nx = P - viewPos(vUV - vec2(uTexel.x, 0.0));
    vec3 py = viewPos(vUV + vec2(0.0, uTexel.y)) - P, ny = P - viewPos(vUV - vec2(0.0, uTexel.y));
    vec3 dx = abs(px.z) < abs(nx.z) ? px : nx;
    vec3 dy = abs(py.z) < abs(ny.z) ? py : ny;
    vec3 N = normalize(cross(dx, dy));

    const int K = 12;
    float screenR = uRadius * uProj[1][1] * 0.5 / dist;          // in uv units
    screenR = min(screenR, 0.08);
    float noise = fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));
    float ang = noise * 6.2831853;
    float sum = 0.0;
    for (int i = 0; i < K; ++i) {
        float t = (float(i) + 0.5) / float(K);
        float a = ang + t * 6.2831853 * 3.0;        // spiral
        vec2 off = vec2(cos(a), sin(a)) * screenR * t;
        off.x *= uViewport.y / uViewport.x;        // aspect
        vec3 S = viewPos(vUV + off);
        vec3 v = S - P;
        float vv = dot(v, v);
        float range = 1.0 - smoothstep(uRadius * 0.5, uRadius, sqrt(vv));
        sum += max(0.0, dot(v, N) - 0.01 * dist) / (vv + 0.0005 * dist * dist + 1e-4) * range;
    }
    float ao = max(0.0, 1.0 - 1.6 * sum / float(K));
    // fade out with distance (sub-pixel detail beyond ~150 m)
    fragAO = mix(ao, 1.0, smoothstep(80.0, 220.0, dist));
}
