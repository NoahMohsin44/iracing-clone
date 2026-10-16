#include "gfx/Renderer.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/matrix_transform.hpp>

#include "core/Log.h"
#include "core/Paths.h"
#include "gfx/ProcTextures.h"

namespace apex {

namespace {
bool loadSet(Texture* t, const std::string& name, float aniso) {
    const std::string dir = paths::asset("textures/" + name + "/");
    return t[0].loadFile(dir + "albedo.jpg", true, aniso) && t[1].loadFile(dir + "normal.jpg", false, aniso) &&
           t[2].loadFile(dir + "arm.jpg", false, aniso);
}

glm::mat4 relModel(const glm::dvec3& pos, const glm::dquat& rot, const glm::dvec3& camPos) {
    glm::dmat4 m = glm::translate(glm::dmat4(1.0), pos - camPos) * glm::mat4_cast(rot);
    return glm::mat4(m);
}
}  // namespace

bool Renderer::init(const TrackData& track, const RenderSettings& rs, const std::function<void(float, const char*)>& progress) {
    track_ = &track;
    rs_ = rs;
    progress(0.05f, "Compiling shaders");
    bool ok = true;
    ok &= terrainSh_.load("terrain.vert", "terrain.frag");
    ok &= canopySh_.load("terrain.vert", "terrain.frag", {"CANOPY"});
    ok &= vergeSh_.load("mesh.vert", "terrain.frag", {"VERGE"});
    ok &= roadSh_.load("mesh.vert", "road.frag");
    ok &= objectSh_.load("mesh.vert", "object.frag");
    ok &= treeSh_.load("tree.vert", "tree.frag");
    ok &= carSh_.load("mesh.vert", "car.frag");
    ok &= skySh_.load("fullscreen.vert", "sky.frag");
    ok &= terrainShadowSh_.load("terrain.vert", "shadow.frag", {"SHADOW"});
    ok &= meshShadowSh_.load("mesh.vert", "shadow.frag", {"SHADOW"});
    ok &= treeShadowSh_.load("tree.vert", "shadow.frag", {"SHADOW", "ALPHA_TEST"});
    ok &= post_.init();
    if (!ok) {
        LOGE("Shader compilation failed");
        return false;
    }
    progress(0.12f, "Building the sky");
    if (!sky_.init()) return false;
    setTimeOfDay(timeOfDay_);

    progress(0.2f, "Loading textures");
    const float an = rs.anisotropy;
    ok &= loadSet(tGrass_, "grass", an) && loadSet(tForest_, "forest_floor", an) && loadSet(tGravel_, "gravel", an) &&
          loadSet(tAsphalt_, "asphalt", an);
    ok &= tBark_.loadFile(paths::asset("textures/bark/albedo.jpg"), true, an);
    if (!ok) LOGW("Some textures failed to load");
    progress(0.3f, "Painting foliage");
    tFoliage_ = proctex::foliageAtlas();
    tGraffiti_ = proctex::graffitiAtlas(paths::asset("fonts/TitilliumWeb-Bold.ttf"));

    progress(0.4f, "Building terrain");
    terrain_.build(track);
    progress(0.55f, "Building the circuit");
    trackMesh_.build(track);
    scenery_.build(track);
    progress(0.7f, "Planting the Green Hell");
    vegetation_.build(track, rs.vegetationQuality);
    progress(0.9f, "Assembling the car");
    if (!carModel_.load(paths::asset("cars/gt3/gt3.glb"), CarModel::Geometry{})) carModel_.build(CarModel::Geometry{});

    glCreateBuffers(1, &ubo_);
    glNamedBufferStorage(ubo_, sizeof(FrameUBO), nullptr, GL_DYNAMIC_STORAGE_BIT);
    glCreateQueries(GL_TIME_ELAPSED, 2, timer_);
    shadows_.init(rs.shadowQuality);
    progress(1.0f, "Ready");
    return true;
}

void Renderer::applySettings(const RenderSettings& rs) {
    bool vegChanged = rs.vegetationQuality != rs_.vegetationQuality;
    rs_ = rs;
    shadows_.init(rs.shadowQuality);
    if (vegChanged && track_) vegetation_.build(*track_, rs.vegetationQuality);
}

void Renderer::setTimeOfDay(float hours) {
    timeOfDay_ = hours;
    sky_.update(hours);
}

void Renderer::ensureTargets(int w, int h) {
    int samples = std::max(1, rs_.msaa);
    if (scene_.width() != w || scene_.height() != h || scene_.samples() != samples) {
        scene_.create(w, h, {GL_RGBA16F}, GL_DEPTH_COMPONENT32F, samples);
        post_.resize(w, h);
        LOGI("Render targets %dx%d, %dx MSAA", w, h, samples);
    }
}

void Renderer::updateUBO(const Camera& cam, int w, int h, float time) {
    FrameUBO u{};
    u.view = cam.view();
    u.proj = cam.proj();
    u.viewProj = u.proj * u.view;
    u.invViewProj = glm::inverse(u.viewProj);
    u.camPos = glm::vec4(glm::vec3(cam.position), time);
    u.sunDir = glm::vec4(sky_.sunDirection(), 1800.0f);
    u.sunColor = glm::vec4(sky_.sunIrradiance(), std::pow(2.0f, rs_.exposureEv));
    for (int i = 0; i < 9; ++i) u.sh[i] = sky_.sh()[i];
    for (int i = 0; i < 4; ++i) u.shadowMat[i] = shadows_.shadowMatrix(std::min(i, shadows_.cascades() - 1));
    u.cascadeSplits = glm::vec4(shadows_.split(0), shadows_.split(1), shadows_.split(2), shadows_.split(3));
    u.shadowParams = glm::vec4(shadows_.texelSize(), static_cast<float>(shadows_.cascades()), shadows_.split(3), 1.0f);
    u.fogParams = glm::vec4(0.000085f, 0.0011f, 0.0f, 1.0f);
    u.fogColor = glm::vec4(sky_.fogColor(), 1.0f);
    u.viewport = glm::vec4(w, h, 1.0f / w, 1.0f / h);
    u.misc = glm::vec4(time, static_cast<float>(sky_.specularMips() - 1), 0.0f, 0.0f);
    glNamedBufferSubData(ubo_, 0, sizeof(FrameUBO), &u);
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, ubo_);
}

void Renderer::bindLighting() const {
    sky_.specular().bind(10);
    sky_.brdfLut().bind(11);
    shadows_.depth().bind(12);
}

void Renderer::drawCars(const Camera& cam, const std::vector<CarVisual>& cars, const Shader& sh, bool shadow, int mode) {
    const glm::dvec3 camPos = cam.position;
    const CarModel& m = carModel_;
    for (const CarVisual& c : cars) {
        glm::mat4 body = relModel(c.position, c.orientation, camPos);
        if (!shadow) {
            sh.set("uWorldToLocal", glm::transpose(glm::mat3(glm::mat3_cast(glm::quat(c.orientation)))));
            sh.set("uPaint", c.paint);
            sh.set("uPaint2", c.paint2);
            sh.set("uAccent", c.accent);
            sh.set("uBrakeLight", c.brakeLight);
            sh.set("uRpmFrac", c.rpmFrac);
            sh.set("uBrakeGlow", 0.0f);
            glUniform4fv(sh.loc("uArch[0]"), 2, &m.arches[0].x);
            sh.set("uMode", mode);
        }
        sh.set("uModel", body);
        if (!shadow) sh.set("uIsBody", m.loaded() ? 2 : 1);
        m.body.draw();
        if (mode == 1) continue;
        if (!shadow) sh.set("uIsBody", 0);
        m.parts.draw();
        if (!c.hideDriver || shadow) m.driver.draw();
        if (!shadow) {
            glm::mat4 sw = body * glm::translate(glm::mat4(1.0f), m.geo.steeringWheelCentre) *
                           glm::rotate(glm::mat4(1.0f), -m.geo.steeringWheelTilt, glm::vec3(1, 0, 0)) *
                           glm::rotate(glm::mat4(1.0f), -c.steeringWheelAngle, glm::vec3(0, 0, 1));
            sh.set("uModel", sw);
            m.steeringWheel.draw();
        }
        const glm::dmat4 rot = glm::mat4_cast(c.orientation);
        for (int i = 0; i < 4; ++i) {
            const int axle = i / 2;
            const double side = (i % 2 == 0) ? -1.0 : 1.0;
            glm::dmat4 base = glm::translate(glm::dmat4(1.0), c.wheelCentre[i] - camPos) * rot *
                              glm::rotate(glm::dmat4(1.0), -c.wheelSteer[i], glm::dvec3(0, 1, 0));
            glm::dmat4 mirror = glm::scale(glm::dmat4(1.0), glm::dvec3(side, 1.0, 1.0));
            glm::dmat4 spin = glm::rotate(glm::dmat4(1.0), -c.wheelSpin[i], glm::dvec3(1, 0, 0));
            if (!shadow) sh.set("uBrakeGlow", c.brakeGlow[i]);
            sh.set("uModel", glm::mat4(base * spin * mirror));
            m.wheel[axle].draw();
            sh.set("uModel", glm::mat4(base * mirror));
            m.caliper[axle].draw();
        }
    }
}

void Renderer::shadowPass(const Camera& cam, const std::vector<CarVisual>& cars) {
    shadows_.update(cam, sky_.sunDirection());
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_CLAMP);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.6f, 3.0f);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    const glm::dvec3 camPos = cam.position;
    for (int c = 0; c < shadows_.cascades(); ++c) {
        shadows_.bindCascade(c);
        glClearDepth(1.0);
        glClear(GL_DEPTH_BUFFER_BIT);
        const glm::mat4& lvp = shadows_.lightViewProj(c);
        Frustum f = shadows_.cascadeFrustum(c);
        const float range = shadows_.split(c) * 2.0f + 400.0f;

        terrainShadowSh_.use();
        terrainShadowSh_.set("uLightVP", lvp);
        terrain_.bindTextures();
        terrain_.drawNear(terrainShadowSh_, camPos, f, range, c >= 2 ? 1 : 0);

        meshShadowSh_.use();
        meshShadowSh_.set("uLightVP", lvp);
        trackMesh_.visible(camPos, f, range, visibleChunks_);
        for (int idx : visibleChunks_) {
            const auto& ch = trackMesh_.chunks()[idx];
            meshShadowSh_.set("uModel", glm::mat4(glm::translate(glm::dmat4(1.0), ch.origin - camPos)));
            ch.rails.draw();
            ch.posts.draw();
        }
        scenery_.visible(camPos, f, range, visibleScenery_);
        for (int idx : visibleScenery_) {
            const auto& cell = scenery_.cells()[idx];
            meshShadowSh_.set("uModel", glm::mat4(glm::translate(glm::dmat4(1.0), cell.origin - camPos)));
            cell.mesh.draw();
        }
        drawCars(cam, cars, meshShadowSh_, true, 0);

        treeShadowSh_.use();
        treeShadowSh_.set("uLightVP", lvp);
        tFoliage_.bind(0);
        vegetation_.draw(camPos, f, std::min(range, shadows_.split(c) + 260.0f), c >= 2 ? 1 : 0);
    }
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glDisable(GL_DEPTH_CLAMP);
}

void Renderer::drawWorld(const Camera& cam, const Frustum& f) {
    const glm::dvec3 camPos = cam.position;
    const float maxDist = rs_.drawDistance;

    // ---- terrain (near, canopy shell, horizon)
    tGrass_[0].bind(1);
    tGrass_[1].bind(2);
    tGrass_[2].bind(3);
    tForest_[0].bind(4);
    tForest_[1].bind(5);
    tForest_[2].bind(6);
    tGravel_[0].bind(7);
    tGravel_[1].bind(8);
    tGravel_[2].bind(9);
    terrain_.bindTextures();
    terrainSh_.use();
    terrain_.drawNear(terrainSh_, camPos, f, maxDist);
    canopySh_.use();
    terrain_.drawCanopy(canopySh_, camPos, f, maxDist, vegetation_.bandWidth() + 6.0f);
    terrainSh_.use();
    terrain_.drawFar(terrainSh_, camPos, f, std::max(maxDist, 12000.0f));
    terrain_.setNearUniforms(terrainSh_);

    // ---- circuit
    trackMesh_.visible(camPos, f, maxDist, visibleChunks_);
    stats_.trackChunks = static_cast<int>(visibleChunks_.size());
    vergeSh_.use();
    terrain_.setNearUniforms(vergeSh_);
    for (int idx : visibleChunks_) {
        const auto& ch = trackMesh_.chunks()[idx];
        vergeSh_.set("uModel", glm::mat4(glm::translate(glm::dmat4(1.0), ch.origin - camPos)));
        ch.verge.draw();
    }
    tAsphalt_[0].bind(1);
    tAsphalt_[1].bind(2);
    tAsphalt_[2].bind(3);
    tGraffiti_.bind(4);
    roadSh_.use();
    roadSh_.set("uTrackLength", track_->length());
    for (int idx : visibleChunks_) {
        const auto& ch = trackMesh_.chunks()[idx];
        roadSh_.set("uModel", glm::mat4(glm::translate(glm::dmat4(1.0), ch.origin - camPos)));
        roadSh_.set("uChunkS", ch.s0);
        ch.road.draw();
    }
    objectSh_.use();
    for (int idx : visibleChunks_) {
        const auto& ch = trackMesh_.chunks()[idx];
        objectSh_.set("uModel", glm::mat4(glm::translate(glm::dmat4(1.0), ch.origin - camPos)));
        objectSh_.set("uMode", 1);
        objectSh_.set("uAlbedo", glm::vec3(0.62f, 0.63f, 0.64f));
        objectSh_.set("uRoughness", 0.38f);
        objectSh_.set("uMetallic", 0.9f);
        ch.rails.draw();
        objectSh_.set("uMode", 0);
        objectSh_.set("uAlbedo", glm::vec3(0.45f, 0.46f, 0.47f));
        objectSh_.set("uRoughness", 0.55f);
        objectSh_.set("uMetallic", 0.8f);
        ch.posts.draw();
    }
    // buildings, marshal posts, bridge parapets
    scenery_.visible(camPos, f, maxDist, visibleScenery_);
    objectSh_.set("uMode", 3);
    objectSh_.set("uMetallic", 0.0f);
    objectSh_.set("uRoughness", 0.8f);
    for (int idx : visibleScenery_) {
        const auto& cell = scenery_.cells()[idx];
        objectSh_.set("uModel", glm::mat4(glm::translate(glm::dmat4(1.0), cell.origin - camPos)));
        cell.mesh.draw();
    }

    // ---- forest
    treeSh_.use();
    tFoliage_.bind(0);
    tBark_.bind(1);
    const bool a2c = scene_.samples() > 1;
    treeSh_.set("uAlphaToCoverage", a2c ? 1 : 0);
    if (a2c) glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    vegetation_.draw(camPos, f, std::min(maxDist, 2500.0f));
    if (a2c) glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    stats_.trees = vegetation_.stats().instances;
    stats_.treeDraws = vegetation_.stats().drawCalls;
}

void Renderer::render(const Camera& camIn, const std::vector<CarVisual>& cars, int windowW, int windowH, float time,
                      bool cockpit) {
    if (windowW <= 0 || windowH <= 0) return;
    const int w = std::max(16, static_cast<int>(windowW * rs_.renderScale));
    const int h = std::max(16, static_cast<int>(windowH * rs_.renderScale));
    ensureTargets(w, h);
    Camera cam = camIn;
    cam.aspect = static_cast<float>(w) / h;

    // GPU timing (read the previous frame's query)
    int prev = 1 - timerIdx_;
    if (timerPending_[prev]) {
        GLint available = 0;
        glGetQueryObjectiv(timer_[prev], GL_QUERY_RESULT_AVAILABLE, &available);
        if (available) {
            GLuint64 ns = 0;
            glGetQueryObjectui64v(timer_[prev], GL_QUERY_RESULT, &ns);
            stats_.gpuMs = ns / 1e6f;
            timerPending_[prev] = false;
        }
    }
    if (!timerPending_[timerIdx_]) {
        glBeginQuery(GL_TIME_ELAPSED, timer_[timerIdx_]);
    }

    glClipControl(GL_LOWER_LEFT, GL_ZERO_TO_ONE);
    shadows_.update(cam, sky_.sunDirection());
    updateUBO(cam, w, h, time);
    bindLighting();
    shadowPass(cam, cars);
    updateUBO(cam, w, h, time);

    // ---- main pass
    scene_.bind();
    glDisable(GL_SCISSOR_TEST);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClearDepth(0.0);
    glDepthMask(GL_TRUE);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_GREATER);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    if (scene_.samples() > 1) glEnable(GL_MULTISAMPLE);
    bindLighting();

    const Frustum f = cam.frustum();
    drawWorld(cam, f);

    carSh_.use();
    carSh_.set("uCockpit", cockpit ? 1.0f : 0.0f);
    drawCars(cam, cars, carSh_, false, 0);

    // sky behind everything (reversed-Z: cleared depth 0 = infinitely far)
    glDepthFunc(GL_GEQUAL);
    glDepthMask(GL_FALSE);
    skySh_.use();
    sky_.skyCube().bind(0);
    gl::drawFullscreenTriangle();

    // transparent: car glass
    glDepthFunc(GL_GREATER);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    carSh_.use();
    bindLighting();
    drawCars(cam, cars, carSh_, false, 1);
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);

    // ---- post
    glClipControl(GL_LOWER_LEFT, GL_NEGATIVE_ONE_TO_ONE);
    post_.resolve(scene_);
    if (rs_.ambientOcclusion) post_.ambientOcclusion(scene_, cam.nearZ);
    else post_.disableAO();
    if (rs_.bloom) post_.bloom();
    post_.composite(windowW, windowH, std::pow(2.0f, rs_.exposureEv), rs_.bloom, cockpit ? 0.35f : 0.22f);

    if (!timerPending_[timerIdx_]) {
        glEndQuery(GL_TIME_ELAPSED);
        timerPending_[timerIdx_] = true;
    }
    timerIdx_ = 1 - timerIdx_;
}

}  // namespace apex
