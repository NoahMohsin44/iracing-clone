// Split-sum BRDF integration lookup (Karis 2013).
in vec2 vUV;
out vec2 fragColor;
const float PI = 3.14159265359;

float radicalInverse(uint bits) {
    bits = (bits << 16u) | (bits >> 16u);
    bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
    bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
    bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
    bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
    return float(bits) * 2.3283064365386963e-10;
}

void main() {
    float NoV = max(vUV.x, 1e-3);
    float rough = vUV.y;
    float a = rough * rough;
    vec3 V = vec3(sqrt(1.0 - NoV * NoV), 0.0, NoV);
    float A = 0.0, B = 0.0;
    const uint COUNT = 256u;
    for (uint i = 0u; i < COUNT; ++i) {
        vec2 xi = vec2(float(i) / float(COUNT), radicalInverse(i));
        float phi = 2.0 * PI * xi.x;
        float cosT = sqrt((1.0 - xi.y) / (1.0 + (a * a - 1.0) * xi.y));
        float sinT = sqrt(1.0 - cosT * cosT);
        vec3 H = vec3(cos(phi) * sinT, sin(phi) * sinT, cosT);
        vec3 L = normalize(2.0 * dot(V, H) * H - V);
        float NoL = max(L.z, 0.0);
        float NoH = max(H.z, 0.0);
        float VoH = max(dot(V, H), 0.0);
        if (NoL > 0.0) {
            float k = a * 0.5;
            float gv = NoV / (NoV * (1.0 - k) + k);
            float gl = NoL / (NoL * (1.0 - k) + k);
            float G = gv * gl;
            float Gv = G * VoH / (NoH * NoV);
            float Fc = pow(1.0 - VoH, 5.0);
            A += (1.0 - Fc) * Gv;
            B += Fc * Gv;
        }
    }
    fragColor = vec2(A, B) / float(COUNT);
}
