#pragma once
#include <functional>
#include <memory>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "car/CarModel.h"
#include "gfx/Camera.h"
#include "gfx/Framebuffer.h"
#include "gfx/PostFX.h"
#include "gfx/Shader.h"
#include "gfx/Shadows.h"
#include "gfx/Sky.h"
#include "gfx/Texture.h"
#include "world/Terrain.h"
#include "world/TrackData.h"
#include "world/TrackMesh.h"
#include "world/Vegetation.h"

namespace apex {

struct RenderSettings {
    int msaa = 4;
    float renderScale = 1.0f;
    int shadowQuality = 2;
    int vegetationQuality = 2;
    bool bloom = true;
    float exposureEv = 0.0f;
    float drawDistance = 4000.0f;
    float anisotropy = 16.0f;
};

// Everything needed to draw one car this frame (copied out of the physics state).
struct CarVisual {
    glm::dvec3 position{0.0};
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
    glm::dvec3 wheelCentre[4];
    double wheelSpin[4] = {};
    double wheelSteer[4] = {};
    float brakeGlow[4] = {};
    float steeringWheelAngle = 0.0f;  // radians, + = clockwise (right)
    float brakeLight = 0.0f;
    float rpmFrac = 0.0f;
    glm::vec3 paint{0.92f, 0.92f, 0.90f};
    glm::vec3 paint2{0.05f, 0.18f, 0.55f};
    glm::vec3 accent{0.95f, 0.35f, 0.02f};
    bool hideDriver = false;
};

class Renderer {
public:
    struct Stats {
        float gpuMs = 0.0f;
        int trees = 0;
        int treeDraws = 0;
        int trackChunks = 0;
    };

    bool init(const TrackData& track, const RenderSettings& rs, const std::function<void(float, const char*)>& progress);
    void applySettings(const RenderSettings& rs);
    void setTimeOfDay(float hours);
    void render(const Camera& cam, const std::vector<CarVisual>& cars, int windowW, int windowH, float time, bool cockpit);
    const Stats& stats() const { return stats_; }
    const RenderSettings& settings() const { return rs_; }

private:
    struct FrameUBO {
        glm::mat4 viewProj, view, proj, invViewProj;
        glm::vec4 camPos, sunDir, sunColor;
        glm::vec4 sh[9];
        glm::mat4 shadowMat[4];
        glm::vec4 cascadeSplits, shadowParams, fogParams, fogColor, viewport, misc;
    };
    void ensureTargets(int w, int h);
    void updateUBO(const Camera& cam, int w, int h, float time);
    void shadowPass(const Camera& cam, const std::vector<CarVisual>& cars);
    void drawWorld(const Camera& cam, const Frustum& f);
    void drawCars(const Camera& cam, const std::vector<CarVisual>& cars, const Shader& sh, bool shadow, int mode);
    void bindLighting() const;

    const TrackData* track_ = nullptr;
    RenderSettings rs_;
    Sky sky_;
    ShadowMaps shadows_;
    PostFX post_;
    Terrain terrain_;
    TrackMesh trackMesh_;
    Vegetation vegetation_;
    CarModel carModel_;
    Framebuffer scene_;
    GLuint ubo_ = 0;
    GLuint timer_[2] = {0, 0};
    int timerIdx_ = 0;
    bool timerPending_[2] = {false, false};
    float timeOfDay_ = 14.5f;

    Shader terrainSh_, canopySh_, vergeSh_, roadSh_, objectSh_, treeSh_, carSh_, skySh_;
    Shader terrainShadowSh_, meshShadowSh_, treeShadowSh_;
    Texture tGrass_[3], tForest_[3], tGravel_[3], tAsphalt_[3], tBark_, tFoliage_;
    Stats stats_;
    std::vector<int> visibleChunks_;
};

}  // namespace apex
