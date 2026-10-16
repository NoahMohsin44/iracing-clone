// Background sky: samples the generated sky cubemap and adds the sun disc.
#include "common.glsl"

in vec2 vUV;
out vec4 fragColor;
layout(binding = 0) uniform samplerCube uSky;

void main() {
    vec4 p = uInvViewProj * vec4(vUV * 2.0 - 1.0, 1.0, 1.0);
    vec3 dir = normalize(p.xyz / p.w);
    vec3 col = textureLod(uSky, dir, 0.0).rgb;
    float cosA = dot(dir, uSunDir.xyz);
    float disc = smoothstep(0.99996, 0.99999, cosA);
    float glow = pow(saturate(cosA), 900.0) * 0.6;
    col += uSunColor.rgb * (disc * uSunDir.w + glow * 4.0) * step(0.0, dir.y + 0.02);
    fragColor = vec4(col, 1.0);
}
