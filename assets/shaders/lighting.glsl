// Physically based shading: GGX specular, Lambert diffuse, image based lighting (prefiltered sky
// cubemap + SH irradiance), cascaded shadow maps and height fog / aerial perspective.
layout(binding = 10) uniform samplerCube uEnvSpecular;
layout(binding = 11) uniform sampler2D uBrdfLut;
layout(binding = 12) uniform sampler2DArrayShadow uShadowMap;

struct Surface {
    vec3 albedo;
    vec3 normal;     // world space
    float roughness;
    float metallic;
    float ao;
    float translucency;  // foliage back-lighting
    float clearcoat;     // car paint
    float specular;      // dielectric F0 scale (0.5 = 4%)
};

vec3 irradianceSH(vec3 n) {
    return max(vec3(0.0),
        uSH[0].rgb * 0.282095 +
        uSH[1].rgb * 0.488603 * n.y + uSH[2].rgb * 0.488603 * n.z + uSH[3].rgb * 0.488603 * n.x +
        uSH[4].rgb * 1.092548 * n.x * n.y + uSH[5].rgb * 1.092548 * n.y * n.z +
        uSH[6].rgb * 0.315392 * (3.0 * n.z * n.z - 1.0) +
        uSH[7].rgb * 1.092548 * n.x * n.z + uSH[8].rgb * 0.546274 * (n.x * n.x - n.y * n.y));
}

float D_GGX(float NoH, float a) {
    float a2 = a * a;
    float d = NoH * NoH * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}

float V_SmithGGXCorrelated(float NoV, float NoL, float a) {
    float a2 = a * a;
    float gv = NoL * sqrt(NoV * NoV * (1.0 - a2) + a2);
    float gl = NoV * sqrt(NoL * NoL * (1.0 - a2) + a2);
    return 0.5 / max(gv + gl, 1e-5);
}

vec3 F_Schlick(vec3 f0, float VoH) {
    float f = pow(1.0 - VoH, 5.0);
    return f0 + (1.0 - f0) * f;
}

vec3 F_SchlickRoughness(vec3 f0, float NoV, float r) {
    return f0 + (max(vec3(1.0 - r), f0) - f0) * pow(1.0 - NoV, 5.0);
}

float viewDepth(vec3 rel) { return -(uView * vec4(rel, 1.0)).z; }

float sampleShadow(vec3 rel, vec3 n) {
    if (uShadowParams.w < 0.5) return 1.0;
    float d = viewDepth(rel);
    int cascades = int(uShadowParams.y);
    int c = 0;
    for (int i = 0; i < 4; ++i) {
        if (i < cascades - 1 && d > uCascadeSplits[i]) c = i + 1;
    }
    if (d > uCascadeSplits[cascades - 1]) return 1.0;
    // normal offset scaled by the cascade's texel footprint
    float texelWorld = uShadowParams.x * (c == 0 ? uCascadeSplits[0] : uCascadeSplits[c]) * 2.2;
    vec3 p = rel + n * texelWorld * 1.6;
    vec4 sc = uShadowMat[c] * vec4(p, 1.0);
    vec3 uvz = sc.xyz / sc.w;
    if (any(lessThan(uvz.xy, vec2(0.0))) || any(greaterThan(uvz.xy, vec2(1.0)))) return 1.0;
    float bias = 0.00015 * float(c + 1);
    float texel = 1.0 / textureSize(uShadowMap, 0).x;
    float s = 0.0;
    for (int y = -1; y <= 1; ++y)
        for (int x = -1; x <= 1; ++x)
            s += texture(uShadowMap, vec4(uvz.xy + vec2(x, y) * texel * 1.25, float(c), uvz.z - bias));
    s /= 9.0;
    // fade out at the end of the shadow range
    float fade = saturate((uCascadeSplits[cascades - 1] - d) / (uCascadeSplits[cascades - 1] * 0.1));
    return mix(1.0, s, fade);
}

vec3 envSpecular(vec3 r, float roughness) {
    float mip = roughness * uMisc.y;
    vec3 c = textureLod(uEnvSpecular, r, mip).rgb;
    // the lower hemisphere of the sky cube is ground; never reflect a bright horizon from below
    return c;
}

vec3 shadeSurface(Surface s, vec3 rel, float shadow) {
    vec3 V = normalize(-rel);
    vec3 N = s.normal;
    vec3 L = uSunDir.xyz;
    vec3 H = normalize(V + L);
    float NoV = max(dot(N, V), 1e-4);
    float NoL = saturate(dot(N, L));
    float NoH = saturate(dot(N, H));
    float VoH = saturate(dot(V, H));
    float a = max(s.roughness * s.roughness, 0.002);
    vec3 f0 = mix(vec3(0.08 * s.specular), s.albedo, s.metallic);
    vec3 diffuseColor = s.albedo * (1.0 - s.metallic);

    // direct sun
    vec3 F = F_Schlick(f0, VoH);
    vec3 spec = D_GGX(NoH, a) * V_SmithGGXCorrelated(NoV, NoL, a) * F;
    vec3 diff = diffuseColor / PI * (1.0 - F);
    vec3 direct = (diff + spec) * NoL * uSunColor.rgb * shadow;

    // foliage: light transmitted through leaves
    if (s.translucency > 0.0) {
        float back = saturate(dot(-N, L)) * 0.6 + 0.4 * pow(saturate(dot(-V, L)), 4.0);
        direct += diffuseColor * s.translucency * back * uSunColor.rgb * (0.25 + 0.75 * shadow) / PI;
    }

    // clear coat (car paint): second specular lobe
    if (s.clearcoat > 0.0) {
        float ac = 0.03 * 0.03;
        float Fc = 0.04 + 0.96 * pow(1.0 - VoH, 5.0);
        direct += D_GGX(NoH, ac + 0.0004) * V_SmithGGXCorrelated(NoV, NoL, 0.03) * Fc * s.clearcoat * NoL *
                  uSunColor.rgb * shadow;
    }

    // image based lighting
    vec3 Fr = F_SchlickRoughness(f0, NoV, s.roughness);
    vec2 brdf = texture(uBrdfLut, vec2(NoV, s.roughness)).rg;
    vec3 R = reflect(-V, N);
    float horizon = saturate(1.0 + 1.2 * dot(R, N));
    vec3 ambientSpec = envSpecular(R, s.roughness) * (Fr * brdf.x + brdf.y) * horizon * horizon;
    vec3 ambientDiff = irradianceSH(N) * diffuseColor * (1.0 - Fr);
    // sky occlusion: surfaces in shadow still receive skylight, slightly reduced
    float skyOcc = mix(0.55, 1.0, shadow);
    vec3 ambient = (ambientDiff + ambientSpec * mix(0.5, 1.0, shadow)) * s.ao * skyOcc;
    if (s.clearcoat > 0.0) {
        float Fc = 0.04 + 0.96 * pow(1.0 - NoV, 5.0);
        ambient += envSpecular(R, 0.03) * Fc * s.clearcoat * mix(0.6, 1.0, shadow) * horizon;
    }
    return direct + ambient;
}

// Exponential height fog with sun in-scattering (aerial perspective).
vec3 applyFog(vec3 color, vec3 rel) {
    float dist = length(rel);
    vec3 dir = rel / max(dist, 1e-3);
    float camY = uCamPos.y;
    float k = uFogParams.y;
    float h0 = camY - uFogParams.z;
    float dy = rel.y;
    float heightTerm = abs(dy) > 0.01 ? (1.0 - exp(-k * dy)) / (k * dy) : 1.0;
    float optical = uFogParams.x * exp(-k * h0) * dist * heightTerm;
    float fog = 1.0 - exp(-optical);
    float sunAmt = pow(saturate(dot(dir, uSunDir.xyz)), 6.0) * uFogParams.w;
    vec3 fogCol = uFogColor.rgb + uSunColor.rgb * sunAmt * 0.12;
    return mix(color, fogCol, fog);
}
