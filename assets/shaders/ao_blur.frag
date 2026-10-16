// Depth-aware 9-tap blur of the AO term along one axis.
in vec2 vUV;
out float fragAO;
layout(binding = 0) uniform sampler2D uAO;
layout(binding = 1) uniform sampler2D uDist;
uniform vec2 uDir;   // texel step

void main() {
    float d0 = texture(uDist, vUV).r;
    float sum = 0.0, wsum = 0.0;
    for (int i = -4; i <= 4; ++i) {
        vec2 uv = vUV + uDir * float(i);
        float d = texture(uDist, uv).r;
        float w = exp(-float(i * i) / 8.0) * exp(-abs(d - d0) / (0.02 * d0 + 0.05) );
        sum += texture(uAO, uv).r * w;
        wsum += w;
    }
    fragAO = sum / max(wsum, 1e-4);
}
