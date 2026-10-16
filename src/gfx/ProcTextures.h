#pragma once
#include <string>

#include "gfx/Texture.h"

namespace apex {

// Procedurally painted textures (no external assets needed).
namespace proctex {

// Atlas regions (uv = [x0, y0, x1, y1]) of the foliage atlas.
constexpr float kSpruceBranch[4] = {0.0f, 0.0f, 0.5f, 0.5f};
constexpr float kBeechCluster[4] = {0.5f, 0.0f, 1.0f, 0.5f};
constexpr float kSpruceImpostor[4] = {0.0f, 0.5f, 0.25f, 1.0f};
constexpr float kBeechImpostor[4] = {0.25f, 0.5f, 0.5f, 1.0f};

// 2048x1024 RGBA atlas: spruce branch spray, beech leaf cluster and two tree impostors.
// Mips are generated with alpha-coverage preservation so foliage doesn't thin out at distance.
Texture foliageAtlas();

// 2048x1024 R8 atlas of road graffiti (8x8 cells of 256x128): fans' names and slogans painted on
// the Nordschleife tarmac, rendered with the given TrueType font, plus a few painted symbols.
constexpr int kGraffitiCols = 8, kGraffitiRows = 8;
Texture graffitiAtlas(const std::string& fontPath);

}  // namespace proctex
}  // namespace apex
