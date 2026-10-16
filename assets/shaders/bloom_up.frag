// 3x3 tent upsample, blended additively onto the next larger level.
in vec2 vUV;
out vec4 fragColor;
layout(binding = 0) uniform sampler2D uSrc;
uniform vec2 uTexel;

void main() {
    vec2 t = uTexel;
    vec3 c = texture(uSrc, vUV).rgb * 4.0;
    c += (texture(uSrc, vUV + vec2(-t.x, 0)).rgb + texture(uSrc, vUV + vec2(t.x, 0)).rgb +
          texture(uSrc, vUV + vec2(0, -t.y)).rgb + texture(uSrc, vUV + vec2(0, t.y)).rgb) * 2.0;
    c += texture(uSrc, vUV + vec2(-t.x, -t.y)).rgb + texture(uSrc, vUV + vec2(t.x, -t.y)).rgb +
         texture(uSrc, vUV + vec2(-t.x, t.y)).rgb + texture(uSrc, vUV + vec2(t.x, t.y)).rgb;
    fragColor = vec4(c / 16.0, 1.0);
}
