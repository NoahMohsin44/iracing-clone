// Linear view distance at half resolution from the (multisampled) reversed-Z depth buffer.
in vec2 vUV;
out float fragDist;
layout(binding = 0) uniform sampler2DMS uDepthMS;
layout(binding = 1) uniform sampler2D uDepth;
uniform int uMultisample;
uniform float uNear;

void main() {
    ivec2 p = ivec2(gl_FragCoord.xy) * 2;
    // farthest of the 2x2 footprint: keeps thin foreground edges from bleeding AO onto the background
    float d = 1.0;
    for (int i = 0; i < 4; ++i) {
        ivec2 q = p + ivec2(i & 1, i >> 1);
        float z = uMultisample == 1 ? texelFetch(uDepthMS, q, 0).r : texelFetch(uDepth, q, 0).r;
        d = min(d, z);
    }
    fragDist = d <= 0.0 ? 1e6 : uNear / d;   // reversed-Z infinite projection: depth = near / distance
}
