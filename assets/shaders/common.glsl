// Shared frame constants. All geometry is rendered camera-relative: positions passed between
// stages are world positions minus the camera position (uCamPos), which keeps float precision
// high across the 20 km circuit.
layout(std140, binding = 0) uniform Frame {
    mat4 uViewProj;      // rotation-only view * projection
    mat4 uView;
    mat4 uProj;
    mat4 uInvViewProj;
    vec4 uCamPos;        // xyz world camera position, w = time (s)
    vec4 uSunDir;        // xyz direction towards the sun, w = sun disc intensity
    vec4 uSunColor;      // rgb sun irradiance, w = exposure
    vec4 uSH[9];         // diffuse irradiance spherical harmonics (already divided by pi)
    mat4 uShadowMat[4];  // camera-relative position -> shadow map uv/depth
    vec4 uCascadeSplits; // view-space distances
    vec4 uShadowParams;  // x texel size, y cascade count, z shadow distance, w enabled
    vec4 uFogParams;     // x density, y height falloff, z base height, w sun inscatter
    vec4 uFogColor;      // rgb horizon colour
    vec4 uViewport;      // w, h, 1/w, 1/h
    vec4 uMisc;          // x wind time, y specular max mip, z wet, w quality
};

const float PI = 3.14159265359;

float saturate(float x) { return clamp(x, 0.0, 1.0); }
vec3 saturate(vec3 x) { return clamp(x, 0.0, 1.0); }

float hash12(vec2 p) {
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

float hash13(vec3 p3) {
    p3 = fract(p3 * 0.1031);
    p3 += dot(p3, p3.zyx + 31.32);
    return fract((p3.x + p3.y) * p3.z);
}

float valueNoise(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    float a = hash12(i), b = hash12(i + vec2(1, 0)), c = hash12(i + vec2(0, 1)), d = hash12(i + vec2(1, 1));
    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

float fbm(vec2 p, int oct) {
    float s = 0.0, a = 0.5;
    for (int i = 0; i < oct; ++i) {
        s += a * valueNoise(p);
        p = p * 2.03 + vec2(17.1, 9.2);
        a *= 0.5;
    }
    return s;
}

vec3 luminanceWeights() { return vec3(0.2126, 0.7152, 0.0722); }
