// MSAA resolve weighted by 1/(1+luma) so bright edges (sun glints, sky) don't alias.
in vec2 vUV;
out vec4 fragColor;
layout(binding = 0) uniform sampler2DMS uColor;
uniform int uSamples;

void main() {
    ivec2 p = ivec2(gl_FragCoord.xy);
    vec3 sum = vec3(0.0);
    float wsum = 0.0;
    for (int i = 0; i < uSamples; ++i) {
        vec3 c = texelFetch(uColor, p, i).rgb;
        c = max(c, vec3(0.0));
        float w = 1.0 / (1.0 + dot(c, vec3(0.2126, 0.7152, 0.0722)));
        sum += c * w;
        wsum += w;
    }
    vec3 c = sum / max(wsum, 1e-5);
    if (any(isnan(c))) c = vec3(0.0);
    fragColor = vec4(c, 1.0);
}
