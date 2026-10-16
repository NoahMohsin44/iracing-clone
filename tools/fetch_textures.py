#!/usr/bin/env python3
"""Downloads the CC0 PBR textures used by the game from Poly Haven (https://polyhaven.com, CC0).

Each material ends up in assets/textures/<name>/{albedo,normal,arm}.jpg
(arm = ambient occlusion / roughness / metalness packed in R/G/B, normal is OpenGL convention).
"""
import json
import os
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "assets", "textures")
UA = "ApexSim-texturefetch/0.1"

MATERIALS = {
    # game name      poly haven asset      resolution
    "asphalt":       ("asphalt_track",     "2k"),
    "grass":         ("leafy_grass",       "1k"),
    "forest_floor":  ("forest_leaves_02",  "1k"),
    "gravel":        ("gravel_floor",      "1k"),
    "concrete":      ("concrete_floor_02", "1k"),
    "bark":          ("pine_bark",         "1k"),
}
MAPS = {"albedo": "Diffuse", "normal": "nor_gl", "arm": "arm"}


def get(url):
    req = urllib.request.Request(url, headers={"User-Agent": UA})
    with urllib.request.urlopen(req, timeout=120) as r:
        return r.read()


def main():
    for name, (asset, res) in MATERIALS.items():
        files = json.loads(get(f"https://api.polyhaven.com/files/{asset}"))
        folder = os.path.join(OUT, name)
        os.makedirs(folder, exist_ok=True)
        for out_name, key in MAPS.items():
            dst = os.path.join(folder, out_name + ".jpg")
            if os.path.exists(dst):
                continue
            url = files[key][res]["jpg"]["url"]
            print(f"{name}/{out_name}.jpg <- {url}")
            with open(dst, "wb") as f:
                f.write(get(url))
        with open(os.path.join(folder, "SOURCE.txt"), "w") as f:
            f.write(f"Poly Haven asset '{asset}' ({res}), https://polyhaven.com/a/{asset}, license CC0 1.0\n")


if __name__ == "__main__":
    main()
