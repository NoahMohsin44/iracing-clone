#pragma once
// Track definition shared by physics, AI, audio and rendering: the centreline ribbon with its
// cross-section (road, kerbs, verge, armco), the heightmapped terrain around it and land cover.
#include <cstdint>
#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace apex {

enum class Surface : uint8_t { Asphalt = 0, Kerb, Concrete, Grass, Gravel, Dirt, Count };

struct SurfaceInfo {
    const char* name;
    float grip;               // friction multiplier relative to dry asphalt
    float rollingResistance;  // extra rolling resistance coefficient
    float bumpAmplitude;      // metres of random vertical noise
    float bumpWavelength;     // metres
    float drag;               // speed-proportional drag coefficient (N per N load per m/s), e.g. gravel
};
const SurfaceInfo& surfaceInfo(Surface s);

struct TrackSample {
    glm::vec3 pos{0.0f};        // centreline position (world, metres)
    glm::vec3 tangent{0.0f};    // unit tangent including slope
    glm::vec3 right{0.0f};      // unit horizontal vector to the right of travel
    float s = 0.0f;
    float halfLeft = 5.0f, halfRight = 5.0f;
    float bank = 0.0f;          // radians, positive = right edge lower
    float curvature = 0.0f;     // 1/m, positive = right-hand corner
    float barrierLeft = 8.0f, barrierRight = 8.0f;  // lateral distance of the armco from centreline
    float vergeOuterLeftY = 0.0f, vergeOuterRightY = 0.0f;  // terrain height where the verge meets the terrain
    float racingLine = 0.0f;    // lateral offset of the ideal line
    float lineCurvature = 0.0f;
    float speedHint = 50.0f;    // m/s, ideal speed along the racing line
};

struct KerbSpan {
    float s0 = 0, s1 = 0;  // along-track extent
    int side = 1;          // +1 right, -1 left
};

struct Section {
    std::string name;
    float start = 0.0f;
};

struct Heightmap {
    int w = 0, h = 0;
    float x0 = 0.0f, z0 = 0.0f, res = 1.0f;
    std::vector<float> data;

    bool load(const std::string& path, float hmin, float hmax);
    bool contains(float x, float z) const;
    float at(int i, int j) const;
    float sample(float x, float z) const;  // piecewise-linear, matches the rendered triangulation
    glm::vec3 normal(float x, float z) const;
    float width() const { return (w - 1) * res; }
    float depth() const { return (h - 1) * res; }
};

struct TrackLocation {
    int seg = -1;         // segment index (sample i -> i+1)
    float t = 0.0f;       // fraction along the segment
    float s = 0.0f;       // distance along the centreline
    float lateral = 0.0f; // signed lateral offset, positive = right
    float distance = 0.0f;  // horizontal distance from the (clamped) centreline foot point
};

struct GroundInfo {
    float height = 0.0f;
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    Surface surface = Surface::Grass;
    TrackLocation loc;
    bool onRoad = false;
};

class TrackData {
public:
    static constexpr float kVergeWidth = 3.5f;
    static constexpr float kKerbInner = 0.35f;   // kerb starts this far inside the white line
    static constexpr float kKerbOuter = 0.95f;   // and extends this far outside
    static constexpr float kKerbHeight = 0.035f;

    bool load(const std::string& directory);
    // Flat test track from a closed loop of points (x, z) spaced ~spacing apart (used by tests).
    bool createSynthetic(const std::vector<glm::vec2>& loop, float halfWidth, float spacing, float barrierOffset);

    const std::string& name() const { return name_; }
    float length() const { return length_; }
    float spacing() const { return spacing_; }
    int sampleCount() const { return static_cast<int>(samples_.size()); }
    const TrackSample& sample(int i) const { return samples_[wrapIndex(i)]; }
    const std::vector<TrackSample>& samples() const { return samples_; }
    const std::vector<KerbSpan>& kerbs() const { return kerbs_; }
    const std::vector<Section>& sections() const { return sections_; }
    const std::vector<std::string>& attribution() const { return attribution_; }
    const std::string& directory() const { return directory_; }       // empty for test tracks
    const std::vector<glm::vec2>& bridges() const { return bridges_; }  // (s0, s1) of bridge decks
    bool onBridge(float s, float margin = 0.0f) const;
    const Heightmap& nearTerrain() const { return near_; }
    const Heightmap& farTerrain() const { return far_; }
    const std::vector<uint8_t>& landcover() const { return landcover_; }  // RGB at nearTerrain resolution

    int wrapIndex(int i) const {
        int n = static_cast<int>(samples_.size());
        return ((i % n) + n) % n;
    }
    float wrapS(float s) const;

    // Spatial queries ---------------------------------------------------------------------------
    TrackLocation locate(const glm::vec3& p, int hint = -1) const;
    GroundInfo ground(const glm::vec3& p, int& hint) const;
    float terrainHeight(float x, float z) const;
    glm::vec3 terrainNormal(float x, float z) const;
    glm::vec3 landcoverAt(float x, float z) const;  // forest, farmland, built-up in [0,1]

    // Ribbon evaluation -------------------------------------------------------------------------
    glm::vec3 centreAt(float s) const;               // centreline (smooth)
    glm::vec3 tangentAt(float s) const;
    glm::vec3 rightAt(float s) const;
    float bankAt(float s) const;
    float halfWidthAt(float s, int side) const;
    float barrierAt(float s, int side) const;
    float racingLineAt(float s) const;
    float speedHintAt(float s) const;
    float roadHeight(float s, float lateral) const;  // laser-scanned surface (or banked plane), not kerbs
    bool hasScannedSurface() const { return surface_.rows > 0; }
    glm::vec3 surfacePoint(float s, float lateral) const;
    bool kerbAt(float s, int side) const;
    float kerbProfile(float s, float u) const;       // u: 0 inner edge .. 1 outer edge
    const char* sectionName(float s) const;
    int sectionIndex(float s) const;

private:
    void buildFrames();
    void buildBarriersAndVerge();
    void buildKerbs();
    void buildRacingLine();
    void buildSpeedProfile();
    void buildGrid();
    float interpScalar(float s, float TrackSample::*field) const;

    std::string name_;
    std::string directory_;
    std::vector<glm::vec2> bridges_;
    float length_ = 0.0f;
    float spacing_ = 2.0f;
    std::vector<TrackSample> samples_;
    std::vector<KerbSpan> kerbs_;
    std::vector<uint8_t> kerbMaskLeft_, kerbMaskRight_;  // per sample
    std::vector<Section> sections_;
    std::vector<std::string> attribution_;
    Heightmap near_, far_;
    std::vector<uint8_t> landcover_;

    // Laser-scanned road surface: heights relative to the centreline on a grid of rows along the
    // track (ds apart) and columns across it (dl apart, starting at lat0). Empty for test tracks.
    struct RoadSurface {
        int rows = 0, cols = 0;
        float ds = 1.0f, lat0 = -12.0f, dl = 0.5f;
        std::vector<float> rel;
        float at(int r, int c) const;
    } surface_;

    // uniform grid of sample indices for global nearest-sample search
    float gridX0_ = 0, gridZ0_ = 0, gridCell_ = 32.0f;
    int gridW_ = 0, gridH_ = 0;
    std::vector<std::vector<int>> grid_;
};

}  // namespace apex
