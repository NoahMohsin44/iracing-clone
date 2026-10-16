#pragma once
#include <vector>

#include <glm/glm.hpp>

#include "core/Settings.h"
#include "gfx/Renderer.h"
#include "game/Session.h"
#include "world/TrackData.h"

namespace apex {

class Hud {
public:
    void init(const TrackData& track);
    void draw(const Session& s, const TrackData& track, const Settings& settings, float fps, const Renderer::Stats& rs,
              const DriverControls& in);
    bool visible = true;

private:
    std::vector<glm::vec2> map_;  // normalised track outline
    float mapAspect_ = 1.0f;
    glm::vec2 mapMin_{0.0f}, mapMax_{1.0f};
};

}  // namespace apex
