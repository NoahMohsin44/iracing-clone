// Depth-only pass. Foliage variants alpha test against the leaf atlas.
#ifdef ALPHA_TEST
in vec2 vUV;
in vec4 vColor;
layout(binding = 0) uniform sampler2D uFoliage;
#endif
void main() {
#ifdef ALPHA_TEST
    if (vColor.r > 0.5 && texture(uFoliage, vUV).a < 0.5) discard;
#endif
}
