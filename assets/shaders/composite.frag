// Final image: exposure, bloom, ACES filmic tone mapping, sRGB encoding, vignette, dithering.
in vec2 vUV;
out vec4 fragColor;
layout(binding = 0) uniform sampler2D uHdr;
layout(binding = 1) uniform sampler2D uBloomTex;
uniform float uExposure;
uniform float uBloom;
uniform float uVignette;

const mat3 ACESIn = mat3(0.59719, 0.07600, 0.02840, 0.35458, 0.90834, 0.13383, 0.04823, 0.01566, 0.83777);
const mat3 ACESOut = mat3(1.60475, -0.10208, -0.00327, -0.53108, 1.10813, -0.07276, -0.07367, -0.00605, 1.07602);

vec3 rrtOdt(vec3 v) {
    vec3 a = v * (v + 0.0245786) - 0.000090537;
    vec3 b = v * (0.983729 * v + 0.4329510) + 0.238081;
    return a / b;
}

vec3 linearToSrgb(vec3 c) {
    return mix(c * 12.92, 1.055 * pow(c, vec3(1.0 / 2.4)) - 0.055, step(vec3(0.0031308), c));
}

float hash(vec2 p) { return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453); }

void main() {
    vec3 hdr = texture(uHdr, vUV).rgb;
    vec3 bloom = texture(uBloomTex, vUV).rgb;
    vec3 c = mix(hdr, bloom, uBloom) * uExposure;
    c = ACESOut * rrtOdt(ACESIn * c);
    c = clamp(c, 0.0, 1.0);
    vec2 q = vUV - 0.5;
    c *= 1.0 - uVignette * dot(q, q) * 1.6;
    c = linearToSrgb(c);
    c += (hash(gl_FragCoord.xy) - 0.5) / 255.0;
    fragColor = vec4(c, 1.0);
}
