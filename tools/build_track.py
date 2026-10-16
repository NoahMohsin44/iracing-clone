#!/usr/bin/env python3
"""
Builds the Nurburgring Nordschleife track data used by the game.

Sources (fetched once, cached in tools/.cache):
  * Centreline + corner names: OpenStreetMap relation 38566 "Nurburgring Nordschleife"
    (c) OpenStreetMap contributors, ODbL 1.0.
  * Forest / farmland / built-up areas: OpenStreetMap (ODbL 1.0).
  * Elevation near the circuit: DGM1, the 1 m bare-earth terrain model from airborne laser scanning
    (c) GeoBasis-DE / LVermGeoRP (2025), dl-de/by-2-0, www.lvermgeo.rlp.de. This gives the road its
    real crests, compressions, cambers and bumps (the Karussell banking, Flugplatz, Pflanzgarten...).
  * Elevation elsewhere: AWS Terrain Tiles (Terrarium encoding; SRTM / EU-DEM derived, public domain/open).

Outputs (assets/tracks/nordschleife/):
  track.json        metadata (length, sections, heightmap extents, attribution)
  centerline.csv    s, x, y, z, half-width left/right, bank angle  (2 m spacing)
  road_surface.png  16-bit laser-scanned road surface: rows every 1 m along the track, columns every
                    0.5 m across (+-12 m), height relative to the centreline (0.2 mm units)
  height_near.png   16-bit heightmap around the circuit (4 m grid, lidar), road already carved in
  height_far.png    16-bit heightmap for the horizon (~48 m grid, 24 km square)
  landcover.png     RGB mask aligned with height_near: R forest, G farmland/meadow, B built-up
  buildings.json    OSM building footprints within 350 m of the circuit (x, z, height, roof type)

World coordinates: X = east, Y = up, Z = south (right-handed, metres), origin at track centre.

Requires: numpy, scipy, pillow, pyproj.   Usage:  python tools/build_track.py
"""
import io
import json
import math
import os
import sys
import time
import urllib.request
import xml.etree.ElementTree as ET

import re
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage
from scipy.spatial import cKDTree

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CACHE = os.path.join(ROOT, "tools", ".cache")
OUT = os.path.join(ROOT, "assets", "tracks", "nordschleife")
UA = "ApexSim-trackbuilder/0.1 (open-source sim racing project)"

RELATION_ID = 38566
SAMPLE_SPACING = 2.0          # metres between centreline samples
NEAR_RES = 4.0                # metres per heightmap texel near the track
NEAR_MARGIN = 700.0           # metres of terrain beyond the track bounding box
FAR_SIZE = 24000.0            # metres, square horizon heightmap
FAR_RES = 48.0
SURF_DS = 1.0                 # road surface grid: metres between rows (along the track)
SURF_DL = 0.5                 # metres between columns (across)
SURF_HALF = 12.0              # lateral extent either side of the centreline
SURF_SCALE = 0.0002           # metres per unit of the 16-bit relative height
BRIDGE_MARGIN = 8.0           # metres of interpolation either side of a bridge deck
START_SECTION = "T13"         # start/finish straight (Nordschleife-only layout)
START_OFFSET = 120.0          # metres into the T13 section


# ----------------------------------------------------------------------------- download helpers
def fetch(url, name, binary=True):
    os.makedirs(CACHE, exist_ok=True)
    path = os.path.join(CACHE, name)
    if not os.path.exists(path):
        for attempt in range(5):
            try:
                req = urllib.request.Request(url, headers={"User-Agent": UA})
                with urllib.request.urlopen(req, timeout=120) as r:
                    data = r.read()
                with open(path, "wb") as f:
                    f.write(data)
                time.sleep(0.3)
                break
            except Exception as e:  # noqa: BLE001
                print(f"  retry {attempt + 1} {url}: {e}")
                time.sleep(2 ** attempt)
        else:
            raise RuntimeError(f"failed to download {url}")
    with open(path, "rb") as f:
        return f.read()


# ----------------------------------------------------------------------------- projection
class Projection:
    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        phi = math.radians(lat0)
        self.m_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * phi) + 1.175 * math.cos(4 * phi)
        self.m_per_deg_lon = 111412.84 * math.cos(phi) - 93.5 * math.cos(3 * phi)

    def to_xz(self, lat, lon):
        x = (np.asarray(lon) - self.lon0) * self.m_per_deg_lon
        z = -(np.asarray(lat) - self.lat0) * self.m_per_deg_lat
        return x, z

    def to_latlon(self, x, z):
        lon = self.lon0 + np.asarray(x) / self.m_per_deg_lon
        lat = self.lat0 - np.asarray(z) / self.m_per_deg_lat
        return lat, lon


# ----------------------------------------------------------------------------- OSM centreline
def load_centreline():
    xml = fetch(f"https://api.openstreetmap.org/api/0.6/relation/{RELATION_ID}/full", f"relation_{RELATION_ID}.xml")
    root = ET.fromstring(xml)
    nodes = {n.get("id"): (float(n.get("lat")), float(n.get("lon"))) for n in root.findall("node")}
    ways = {w.get("id"): w for w in root.findall("way")}
    rel = [r for r in root.findall("relation") if r.get("id") == str(RELATION_ID)][0]
    pts, names = [], []
    last = None
    for m in rel.findall("member"):
        w = ways[m.get("ref")]
        nds = [n.get("ref") for n in w.findall("nd")]
        tags = {t.get("k"): t.get("v") for t in w.findall("tag")}
        name = tags.get("name")
        if name == "Nürburgring Nordschleife":   # bridge segment on the Döttinger Höhe straight
            name = "Döttinger Höhe"
        if last is not None and nds[0] != last:
            if nds[-1] == last:
                nds = nds[::-1]
            else:
                raise RuntimeError(f"relation members not connected at way {m.get('ref')}")
        for i, nd in enumerate(nds):
            if last is not None and i == 0:
                continue
            pts.append(nodes[nd])
            names.append(name)
        last = nds[-1]
    if pts[0] == pts[-1]:
        pts, names = pts[:-1], names[:-1]
    print(f"centreline: {len(pts)} OSM nodes")
    return np.array(pts), names


def load_bridges(proj):
    """(x, z) end points of the bridge decks the track runs over (OSM bridge=yes members)."""
    root = ET.fromstring(fetch(f"https://api.openstreetmap.org/api/0.6/relation/{RELATION_ID}/full",
                               f"relation_{RELATION_ID}.xml"))
    nodes = {n.get("id"): (float(n.get("lat")), float(n.get("lon"))) for n in root.findall("node")}
    spans = []
    for w in root.findall("way"):
        tags = {t.get("k"): t.get("v") for t in w.findall("tag")}
        if tags.get("bridge") in (None, "no"):
            continue
        ll = np.array([nodes[nd.get("ref")] for nd in w.findall("nd")])
        x, z = proj.to_xz(ll[:, 0], ll[:, 1])
        spans.append(np.stack([x, z], axis=1))
    return spans


def bridge_rows(spans, pts, spacing, rows, ds):
    """Boolean mask over surface rows (spacing ds) covering each bridge deck plus a margin."""
    tree = cKDTree(pts)
    n = len(pts)
    mask = np.zeros(rows, bool)
    for sp in spans:
        _, idx = tree.query(sp)
        s_a, s_b = idx.min() * spacing, idx.max() * spacing
        if s_b - s_a > n * spacing / 2:   # wraps around the start line
            s_a, s_b = s_b, s_a + n * spacing
        r0 = int(np.floor((s_a - BRIDGE_MARGIN) / ds))
        r1 = int(np.ceil((s_b + BRIDGE_MARGIN) / ds))
        mask[np.arange(r0, r1 + 1) % rows] = True
    return mask


def fill_rows(values, mask):
    """Linear interpolation (closed loop) of values over the masked rows; works on 1-D and 2-D."""
    rows = len(values)
    good = np.where(~mask)[0]
    if len(good) == len(values):
        return values
    ext = np.concatenate([good - rows, good, good + rows])
    out = values.copy()
    bad = np.where(mask)[0]
    if values.ndim == 1:
        out[bad] = np.interp(bad, ext, np.tile(values[good], 3))
    else:
        for c in range(values.shape[1]):
            out[bad, c] = np.interp(bad, ext, np.tile(values[good, c], 3))
    return out


def catmull_rom_closed(p, spacing):
    """Resample a closed polyline with a centripetal Catmull-Rom spline at ~uniform spacing."""
    n = len(p)
    dense = []
    for i in range(n):
        p0, p1, p2, p3 = p[(i - 1) % n], p[i], p[(i + 1) % n], p[(i + 2) % n]

        def tj(ti, a, b):
            return ti + max(np.linalg.norm(b - a), 1e-6) ** 0.5

        t0 = 0.0
        t1 = tj(t0, p0, p1)
        t2 = tj(t1, p1, p2)
        t3 = tj(t2, p2, p3)
        seg_len = np.linalg.norm(p2 - p1)
        steps = max(2, int(seg_len / (spacing * 0.25)))
        for k in range(steps):
            t = t1 + (t2 - t1) * k / steps
            a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
            a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
            a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
            b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
            b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3
            dense.append((t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2)
    dense = np.array(dense)
    seg = np.linalg.norm(np.diff(np.vstack([dense, dense[:1]]), axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    total = s[-1]
    count = int(round(total / spacing))
    target = np.linspace(0, total, count, endpoint=False)
    closed = np.vstack([dense, dense[:1]])
    out = np.stack([np.interp(target, s, closed[:, 0]), np.interp(target, s, closed[:, 1])], axis=1)
    return out, total


def smooth_closed(values, sigma_samples):
    return ndimage.gaussian_filter1d(values, sigma_samples, mode="wrap")


# ----------------------------------------------------------------------------- elevation tiles
def lonlat_to_tile(lon, lat, z):
    n = 2 ** z
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


class Terrarium:
    def __init__(self, zoom, lat_min, lat_max, lon_min, lon_max):
        self.z = zoom
        x0, y0 = lonlat_to_tile(lon_min, lat_max, zoom)
        x1, y1 = lonlat_to_tile(lon_max, lat_min, zoom)
        self.tx0, self.ty0 = int(x0), int(y0)
        self.tx1, self.ty1 = int(x1), int(y1)
        w = (self.tx1 - self.tx0 + 1) * 256
        h = (self.ty1 - self.ty0 + 1) * 256
        self.data = np.zeros((h, w), np.float32)
        count = (self.tx1 - self.tx0 + 1) * (self.ty1 - self.ty0 + 1)
        print(f"elevation z{zoom}: {count} tiles")
        for ty in range(self.ty0, self.ty1 + 1):
            for tx in range(self.tx0, self.tx1 + 1):
                png = fetch(f"https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{zoom}/{tx}/{ty}.png",
                            f"terrarium_{zoom}_{tx}_{ty}.png")
                img = np.asarray(Image.open(io.BytesIO(png)).convert("RGB")).astype(np.float32)
                elev = img[:, :, 0] * 256.0 + img[:, :, 1] + img[:, :, 2] / 256.0 - 32768.0
                oy, ox = (ty - self.ty0) * 256, (tx - self.tx0) * 256
                self.data[oy:oy + 256, ox:ox + 256] = elev

    def sample(self, lat, lon):
        lat = np.asarray(lat, np.float64)
        lon = np.asarray(lon, np.float64)
        n = 2 ** self.z
        px = ((lon + 180.0) / 360.0 * n - self.tx0) * 256.0 - 0.5
        py = ((1.0 - np.arcsinh(np.tan(np.radians(lat))) / np.pi) / 2.0 * n - self.ty0) * 256.0 - 0.5
        return ndimage.map_coordinates(self.data, [py.ravel(), px.ravel()], order=3, mode="nearest").reshape(lat.shape)


# ----------------------------------------------------------------------------- lidar (DGM1 Rheinland-Pfalz)
class Dgm1:
    """1 m bare-earth elevation (ETRS89 / UTM 32N) from the open-data 1 km GeoTIFF tiles."""
    BASE = "https://geobasis-rlp.de/data/dgm1/current"

    def __init__(self, proj):
        from pyproj import Transformer
        self.to_utm = Transformer.from_crs("EPSG:4326", "EPSG:25832", always_xy=True)
        self.proj = proj
        index = fetch(f"{self.BASE}/meta4/dgm1_tif_07.meta4", "dgm1_tif_07.meta4").decode("utf-8")
        self.names = {(int(m.group(2)), int(m.group(3))): m.group(1)
                      for m in re.finditer(r'<file name="(dgm1_32_(\d+)_(\d+)_1_rp_\d+\.tif)">', index)}
        self.tiles = {}

    def utm(self, x, z):
        lat, lon = self.proj.to_latlon(np.asarray(x, np.float64), np.asarray(z, np.float64))
        e, n = self.to_utm.transform(lon, lat)
        return np.asarray(e), np.asarray(n)

    def load(self, keys):
        keys = [k for k in set(keys) if k in self.names and k not in self.tiles]
        os.makedirs(os.path.join(CACHE, "dgm1"), exist_ok=True)

        def get(k):
            name = self.names[k]
            fetch(f"{self.BASE}/tif/{name}", os.path.join("dgm1", name))
        with ThreadPoolExecutor(6) as ex:
            list(ex.map(get, keys))
        for k in keys:
            with open(os.path.join(CACHE, "dgm1", self.names[k]), "rb") as f:
                self.tiles[k] = np.asarray(Image.open(io.BytesIO(f.read())), np.float32)
        print(f"lidar: {len(self.tiles)} tiles loaded")

    def sample(self, x, z):
        """Bilinear height at world (x, z); NaN where no tile is available."""
        e, n = self.utm(x, z)
        shape = e.shape
        e, n = e.ravel(), n.ravel()
        # pixel centres sit at (k + 0.5) m; rows count down from the tile's north edge
        col = e - 0.5
        row = -(n - 0.5)
        c0 = np.floor(col).astype(np.int64)
        r0 = np.floor(row).astype(np.int64)
        fc, fr = col - c0, row - r0
        out = np.zeros(e.shape, np.float64)
        for dr, dc, wgt in ((0, 0, (1 - fr) * (1 - fc)), (0, 1, (1 - fr) * fc), (1, 0, fr * (1 - fc)), (1, 1, fr * fc)):
            out += wgt * self._px(r0 + dr, c0 + dc)
        return out.reshape(shape)

    def _px(self, rows, cols):
        # global pixel (row counts down from N=0, col from E=0) -> tile lookup
        # pixel centre: E = col + 0.5, N = -row + 0.5; a tile's row 0 is its northern edge
        te = np.floor_divide(cols, 1000)
        tn = np.floor_divide(-rows, 1000)
        lc = cols - te * 1000
        lr = rows + tn * 1000 + 999
        vals = np.full(rows.shape, np.nan)
        for k in set(zip(te.tolist(), tn.tolist())):
            t = self.tiles.get(k)
            if t is None:
                continue
            m = (te == k[0]) & (tn == k[1])
            vals[m] = t[lr[m], lc[m]]
        return vals


# ----------------------------------------------------------------------------- landcover (OSM)
FOREST_TAGS = {("landuse", "forest"), ("natural", "wood")}
FARM_TAGS = {("landuse", "farmland"), ("landuse", "meadow"), ("landuse", "grass"), ("natural", "grassland"),
             ("landuse", "farmyard"), ("natural", "heath"), ("natural", "scrub")}
BUILT_TAGS = {("landuse", "residential"), ("landuse", "commercial"), ("landuse", "industrial"),
              ("landuse", "retail"), ("amenity", "parking"), ("building", "*")}


def classify(tags):
    for k, v in tags.items():
        if (k, v) in FOREST_TAGS:
            return 0
        if (k, v) in FARM_TAGS:
            return 1
        if (k, v) in BUILT_TAGS or (k == "building"):
            return 2
    return None


def join_rings(segments):
    """Join way node-lists into closed rings by matching endpoints."""
    segs = [list(s) for s in segments if len(s) >= 2]
    rings = []
    while segs:
        ring = segs.pop(0)
        changed = True
        while ring[0] != ring[-1] and changed:
            changed = False
            for i, s in enumerate(segs):
                if s[0] == ring[-1]:
                    ring += s[1:]
                elif s[-1] == ring[-1]:
                    ring += s[::-1][1:]
                elif s[-1] == ring[0]:
                    ring = s[:-1] + ring
                elif s[0] == ring[0]:
                    ring = s[::-1][:-1] + ring
                else:
                    continue
                segs.pop(i)
                changed = True
                break
        if len(ring) >= 4:
            rings.append(ring)
    return rings


def load_landcover(lat_min, lat_max, lon_min, lon_max):
    nodes, ways, rels = {}, {}, {}
    dlat, dlon = 0.02, 0.03
    lat = lat_min
    tiles = []
    while lat < lat_max:
        lon = lon_min
        while lon < lon_max:
            tiles.append((lon, lat, min(lon + dlon, lon_max), min(lat + dlat, lat_max)))
            lon += dlon
        lat += dlat
    print(f"landcover: {len(tiles)} OSM map tiles")

    def ingest(root):
        for n in root.findall("node"):
            nodes[n.get("id")] = (float(n.get("lat")), float(n.get("lon")))
        for w in root.findall("way"):
            ways[w.get("id")] = ([nd.get("ref") for nd in w.findall("nd")],
                                 {t.get("k"): t.get("v") for t in w.findall("tag")})
        for r in root.findall("relation"):
            tags = {t.get("k"): t.get("v") for t in r.findall("tag")}
            members = [(m.get("type"), m.get("ref"), m.get("role")) for m in r.findall("member")]
            rels[r.get("id")] = (members, tags)

    for (a, b, c, d) in tiles:
        xml = fetch(f"https://api.openstreetmap.org/api/0.6/map?bbox={a:.4f},{b:.4f},{c:.4f},{d:.4f}",
                    f"map_{a:.4f}_{b:.4f}_{c:.4f}_{d:.4f}.xml")
        ingest(ET.fromstring(xml))

    polys = []  # (class, outer_rings, inner_rings) with rings as lists of (lat, lon)
    for rid, (members, tags) in list(rels.items()):
        if tags.get("type") != "multipolygon":
            continue
        cls = classify(tags)
        if cls is None:
            continue
        missing = any(t == "way" and ref not in ways for t, ref, _ in members)
        if missing:
            xml = fetch(f"https://api.openstreetmap.org/api/0.6/relation/{rid}/full", f"relation_{rid}.xml")
            ingest(ET.fromstring(xml))
        outer = join_rings([ways[ref][0] for t, ref, role in members if t == "way" and role != "inner" and ref in ways])
        inner = join_rings([ways[ref][0] for t, ref, role in members if t == "way" and role == "inner" and ref in ways])
        to_ll = lambda ring: [nodes[n] for n in ring if n in nodes]
        polys.append((cls, [to_ll(r) for r in outer], [to_ll(r) for r in inner]))
    buildings = []
    for wid, (nds, tags) in ways.items():
        if "building" in tags and len(nds) >= 4 and nds[0] == nds[-1] and all(n in nodes for n in nds):
            buildings.append((tags, [nodes[n] for n in nds[:-1]]))
        cls = classify(tags)
        if cls is None or len(nds) < 4 or nds[0] != nds[-1]:
            continue
        polys.append((cls, [[nodes[n] for n in nds if n in nodes]], []))
    print(f"landcover: {len(polys)} polygons, {len(buildings)} buildings")
    return polys, buildings


def building_record(tags, xz):
    """Footprint (counter-clockwise seen from above), height and roof style for the game."""
    x, z = xz[:, 0], xz[:, 1]
    area2 = np.sum(x * np.roll(z, -1) - np.roll(x, -1) * z)
    if area2 > 0:              # +z is south, so CCW seen from above has negative shoelace sum
        xz = xz[::-1]
    area = abs(area2) / 2
    kind = tags.get("building", "yes")
    industrial = kind in ("industrial", "commercial", "retail", "warehouse", "hangar", "service") or area > 650
    small = kind in ("garage", "garages", "shed", "hut", "carport", "roof", "kiosk") or area < 35
    height = None
    for key in ("height", "building:height"):
        try:
            height = float(tags.get(key, "").replace("m", "").strip())
        except ValueError:
            pass
    if height is None and tags.get("building:levels", "").isdigit():
        height = 2.9 * int(tags["building:levels"]) + (0.5 if industrial else 0.0)
    if height is None:
        height = 8.5 if industrial else (2.8 if small else 5.8)
    roof = 0 if (industrial or small or len(xz) > 6) else 1     # 0 flat, 1 gable
    return {"pts": [[round(float(a), 2), round(float(b), 2)] for a, b in xz], "h": round(height, 2),
            "roof": roof, "kind": 1 if industrial else (2 if small else 0)}


# ----------------------------------------------------------------------------- main
def main():
    os.makedirs(OUT, exist_ok=True)
    latlon, names = load_centreline()
    lat0 = (latlon[:, 0].min() + latlon[:, 0].max()) / 2
    lon0 = (latlon[:, 1].min() + latlon[:, 1].max()) / 2
    proj = Projection(lat0, lon0)
    x, z = proj.to_xz(latlon[:, 0], latlon[:, 1])
    raw = np.stack([x, z], axis=1)

    # Light smoothing of OSM kinks before spline fitting (keeps corners, removes 1-node zigzags).
    pts, length = catmull_rom_closed(raw, SAMPLE_SPACING)
    for _ in range(2):
        pts = np.stack([smooth_closed(pts[:, 0], 2.0), smooth_closed(pts[:, 1], 2.0)], axis=1)
    seg = np.linalg.norm(np.diff(np.vstack([pts, pts[:1]]), axis=0), axis=1)
    length = seg.sum()
    # Re-sample once more at exact spacing after smoothing.
    s_acc = np.concatenate([[0], np.cumsum(seg)])
    closed = np.vstack([pts, pts[:1]])
    n = int(round(length / SAMPLE_SPACING))
    t = np.linspace(0, length, n, endpoint=False)
    pts = np.stack([np.interp(t, s_acc, closed[:, 0]), np.interp(t, s_acc, closed[:, 1])], axis=1)
    print(f"track length {length:.1f} m, {n} samples")

    # Section name for every sample: map arc-length fraction onto the raw OSM polyline (monotonic,
    # so hairpins such as the Karussell cannot pick up the name of the adjacent section).
    raw_seg = np.linalg.norm(np.diff(np.vstack([raw, raw[:1]]), axis=0), axis=1)
    raw_s = np.concatenate([[0], np.cumsum(raw_seg)])[:-1] / raw_seg.sum()
    frac = np.arange(n) / n
    idx = np.clip(np.searchsorted(raw_s, frac, side="right") - 1, 0, len(raw) - 1)
    sample_names = [names[i] for i in idx]
    # Fill unnamed connector ways with the previous name.
    last = None
    for i in range(len(sample_names)):
        if sample_names[i] is None:
            sample_names[i] = last
        else:
            last = sample_names[i]
    for i in range(len(sample_names)):
        if sample_names[i] is None:
            sample_names[i] = sample_names[i - 1] or "Nordschleife"

    # Rotate so sample 0 is the start/finish line.
    first = next(i for i, nm in enumerate(sample_names) if nm == START_SECTION)
    start = (first + int(START_OFFSET / SAMPLE_SPACING)) % n
    pts = np.roll(pts, -start, axis=0)
    sample_names = sample_names[start:] + sample_names[:start]

    # Heading & curvature (positive = right-hand corner; +Z is south so +heading turns clockwise).
    d = np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)
    hd = np.arctan2(d[:, 1], d[:, 0])
    dh = np.angle(np.exp(1j * (np.roll(hd, -1) - hd)))
    curvature = smooth_closed(dh / SAMPLE_SPACING, 3.0)

    # Tarmac half width per sample.
    wide = {"T13": 6.0, "Döttinger Höhe": 6.0, "Antoniusbuche": 5.5, "Tiergarten": 5.5, "Hohenrain": 5.5,
            "Sabine-Schmitz-Kurve": 5.0, "Galgenkopf": 5.0}
    half = np.array([wide.get(nm, 4.75) for nm in sample_names], np.float64)
    half = smooth_closed(half, 15.0)

    # ------------------------------------------------------------------ elevation
    lat_s, lon_s = proj.to_latlon(pts[:, 0], pts[:, 1])
    pad = 0.02
    dem_near = Terrarium(14, lat_s.min() - pad, lat_s.max() + pad, lon_s.min() - pad * 1.6, lon_s.max() + pad * 1.6)

    # Lidar tiles for the whole near-heightmap area (the road needs only the corridor, the scenery
    # around it looks right only with bare-earth heights - SRTM measures the tree tops).
    xmin, xmax = pts[:, 0].min() - NEAR_MARGIN, pts[:, 0].max() + NEAR_MARGIN
    zmin, zmax = pts[:, 1].min() - NEAR_MARGIN, pts[:, 1].max() + NEAR_MARGIN
    lidar = Dgm1(proj)
    ce, cn = lidar.utm(np.array([xmin, xmax, xmin, xmax]), np.array([zmin, zmin, zmax, zmax]))
    lidar.load([(te, tn) for te in range(int(ce.min() // 1000), int(ce.max() // 1000) + 1)
                for tn in range(int(cn.min() // 1000), int(cn.max() // 1000) + 1)])

    # Road surface grid: rows every SURF_DS along the centreline, columns across it.
    tang = np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    nrm = np.stack([-tang[:, 1], tang[:, 0]], axis=1)  # points to the right of travel
    rows = int(round(length / SURF_DS))
    s_rows = np.arange(rows) * SURF_DS
    s_smp = np.arange(n + 1) * SAMPLE_SPACING
    def along(v):  # linear interpolation of per-sample values at the surface rows (closed loop)
        v = np.vstack([v, v[:1]]) if v.ndim == 2 else np.concatenate([v, v[:1]])
        if v.ndim == 1:
            return np.interp(s_rows, s_smp, v)
        return np.stack([np.interp(s_rows, s_smp, v[:, k]) for k in range(v.shape[1])], axis=1)
    p_rows = along(pts)
    n_rows = along(nrm)
    n_rows /= np.linalg.norm(n_rows, axis=1, keepdims=True)
    lat_cols = np.arange(-SURF_HALF, SURF_HALF + 1e-6, SURF_DL)
    SX = p_rows[:, 0:1] + n_rows[:, 0:1] * lat_cols[None, :]
    SZ = p_rows[:, 1:2] + n_rows[:, 1:2] * lat_cols[None, :]
    surf = lidar.sample(SX, SZ)
    if np.isnan(surf).any():
        print(f"  warning: {np.isnan(surf).mean() * 100:.2f}% of the road surface has no lidar; using SRTM")
        sl, so = proj.to_latlon(SX, SZ)
        surf = np.where(np.isnan(surf), dem_near.sample(sl, so), surf)
    # Bridge decks are not in a bare-earth model: bridge them over.
    bmask = bridge_rows(load_bridges(proj), pts, SAMPLE_SPACING, rows, SURF_DS)
    surf = fill_rows(surf, bmask)
    # The lidar points carry centimetre noise. Raw, it becomes a carpet of short bumps that unloads
    # tyres at speed (and spins the car on a light brake). Treat it like a scanned-track pipeline:
    #  - across the tarmac, fit every cross-section with a smooth camber + crown (quadratic),
    #  - along the track, smooth the fit over ~3 m: real crests, dips and compressions longer than
    #    10-15 m stay, sensor noise goes; the game adds fine road texture procedurally.
    surf = ndimage.gaussian_filter(surf, sigma=(0.8, 0.6), mode=("wrap", "nearest"))
    half_rows = along(half)
    L = lat_cols[None, :]
    wfit = np.clip((half_rows[:, None] + 0.4 - np.abs(L)) / 0.8, 0.0, 1.0)      # tarmac (+ a little)
    X = np.stack([np.ones_like(L), L, L * L], axis=-1)                            # (1, cols, 3)
    A = np.einsum("rc,rci,rcj->rij", wfit, np.broadcast_to(X, (rows,) + X.shape[1:]),
                  np.broadcast_to(X, (rows,) + X.shape[1:]))
    bvec = np.einsum("rc,rci,rc->ri", wfit, np.broadcast_to(X, (rows,) + X.shape[1:]), surf)
    coef = np.linalg.solve(A + np.eye(3)[None] * 1e-6, bvec[..., None])[..., 0]  # (rows, 3)
    for k, sig in enumerate((3.0, 4.0, 8.0)):                                      # height, crossfall, crown
        coef[:, k] = ndimage.gaussian_filter1d(coef[:, k], sig / SURF_DS, mode="wrap")
    fit = coef[:, 0:1] + coef[:, 1:2] * L + coef[:, 2:3] * L * L
    # beyond the tarmac (kerb run-off, verge) keep the scanned shape, smoothed, blended in from the fit
    outside = ndimage.gaussian_filter(surf, sigma=(2.0 / SURF_DS, 1.0 / SURF_DS), mode=("wrap", "nearest"))
    blend = np.clip((np.abs(L) - half_rows[:, None]) / 1.5, 0.0, 1.0)
    blend = blend * blend * (3 - 2 * blend)
    on = wfit >= 1.0
    edge = np.abs(np.abs(L) - half_rows[:, None]) < SURF_DL * 0.75
    print(f"road fit: scan - fit {np.sqrt(np.mean((surf - fit)[on] ** 2)) * 1000:.1f} mm rms on the tarmac, "
          f"verge - fit at the tarmac edge p95 {np.percentile(np.abs(outside - fit)[edge], 95) * 100:.1f} cm")
    surf = fit * (1 - blend) + outside * blend
    print(f"road surface: {rows} x {len(lat_cols)} samples, {bmask.sum()} rows bridged")

    road = surf[::int(round(SAMPLE_SPACING / SURF_DS)), len(lat_cols) // 2][:n]
    road_old = ndimage.median_filter(dem_near.sample(lat_s, lon_s), size=21, mode="wrap")
    dd = road - smooth_closed(road_old, 9.0)
    print(f"road elevation {road.min():.1f} .. {road.max():.1f} m (lidar - SRTM: mean {dd.mean():.1f}, "
          f"max |diff| {np.abs(dd).max():.1f} m)")

    # ------------------------------------------------------------------ banking
    # Cross slope of the scanned surface over the tarmac width (least squares), as a bank angle
    # (positive = right edge lower). Used for orienting cars and cameras; the physics and the road
    # mesh use the full surface grid.
    srows = surf[::int(round(SAMPLE_SPACING / SURF_DS))][:n]
    bank = np.zeros(n)
    for i in range(n):
        m = np.abs(lat_cols) <= half[i]
        L = lat_cols[m]
        slope = np.polyfit(L, srows[i, m], 1)[0]
        bank[i] = math.atan(-slope)
    bank = smooth_closed(bank, 2.0)
    print(f"bank from lidar: {np.degrees(bank).min():.1f} .. {np.degrees(bank).max():.1f} deg")

    # ------------------------------------------------------------------ write centreline
    origin_y = float(np.floor(road.min() / 10.0) * 10.0 - 20.0)   # keep world Y small-ish
    with open(os.path.join(OUT, "centerline.csv"), "w") as f:
        f.write("# s,x,y,z,half_width_left,half_width_right,bank\n")
        for i in range(n):
            f.write(f"{i * SAMPLE_SPACING:.2f},{pts[i, 0]:.3f},{road[i] - origin_y:.3f},{pts[i, 1]:.3f},"
                    f"{half[i]:.3f},{half[i]:.3f},{bank[i]:.5f}\n")

    sections = []
    for i, nm in enumerate(sample_names):
        if not sections or sections[-1]["name"] != nm:
            sections.append({"name": nm, "start": i * SAMPLE_SPACING})
    if len(sections) > 1 and sections[-1]["name"] == sections[0]["name"]:
        sections.pop()  # the wrap-around chunk belongs to the first section

    # ------------------------------------------------------------------ road surface grid
    # Heights are stored relative to the centreline as the game interpolates it (uniform Catmull-Rom
    # through the 2 m samples), so centre + relative reproduces the scan exactly at every row.
    yc = road - origin_y
    step = int(round(SAMPLE_SPACING / SURF_DS))
    yc_rows = np.empty(rows)
    for k in range(step):
        t = k / step
        y0, y1, y2, y3 = np.roll(yc, 1), yc, np.roll(yc, -1), np.roll(yc, -2)
        yc_rows[k::step] = 0.5 * ((2 * y1) + (-y0 + y2) * t + (2 * y0 - 5 * y1 + 4 * y2 - y3) * t * t +
                                  (-y0 + 3 * y1 - 3 * y2 + y3) * t ** 3)[:len(yc_rows[k::step])]
    rel = (surf - origin_y) - yc_rows[:, None]
    q = np.clip(np.round(rel / SURF_SCALE) + 32768, 0, 65535).astype(np.uint16)
    Image.fromarray(q).save(os.path.join(OUT, "road_surface.png"))
    clipped = (np.abs(rel) > 32767 * SURF_SCALE).mean()
    print(f"road_surface.png: relative heights {rel.min():.2f} .. {rel.max():.2f} m, clipped {clipped * 100:.2f}%")

    # ------------------------------------------------------------------ near heightmap
    nx = int(math.ceil((xmax - xmin) / NEAR_RES)) + 1
    nz = int(math.ceil((zmax - zmin) / NEAR_RES)) + 1
    gx = xmin + np.arange(nx) * NEAR_RES
    gz = zmin + np.arange(nz) * NEAR_RES
    GX, GZ = np.meshgrid(gx, gz)
    glat, glon = proj.to_latlon(GX, GZ)
    srtm = dem_near.sample(glat, glon) - origin_y
    lid = lidar.sample(GX, GZ) - origin_y
    valid = ~np.isnan(lid)
    # Fade from lidar to SRTM towards missing tiles and the outer border (where the far terrain,
    # also SRTM, takes over), so there is no step in the landscape.
    fade = ndimage.distance_transform_edt(valid) * NEAR_RES
    edge = np.minimum.reduce([GX - xmin, xmax - GX, GZ - zmin, zmax - GZ])
    wl = np.clip(np.minimum(fade, edge) / 300.0, 0.0, 1.0)
    wl = wl * wl * (3 - 2 * wl)
    height = np.where(valid, lid, srtm) * wl + srtm * (1 - wl)
    height = ndimage.gaussian_filter(height, 0.6)
    print(f"near heightmap {nx}x{nz} @ {NEAR_RES} m, lidar covers {valid.mean() * 100:.0f}%")

    # Carve the road: nearest centreline sample per texel -> lateral offset -> blend to road profile.
    ctree = cKDTree(pts)
    dist, ci = ctree.query(np.stack([GX.ravel(), GZ.ravel()], axis=1), k=1, workers=-1)
    ci = ci.reshape(GX.shape)
    off = np.stack([GX - pts[ci, 0], GZ - pts[ci, 1]], axis=-1)
    lateral = off[..., 0] * nrm[ci, 0] + off[..., 1] * nrm[ci, 1]
    # scanned height at the texel's lateral position, clamped to the verge's outer edge (the verge
    # mesh runs from the road edge down/up to the terrain there)
    col = (np.clip(lateral, -half[ci] - 3.5, half[ci] + 3.5) + SURF_HALF) / SURF_DL
    c0 = np.clip(np.floor(col).astype(int), 0, len(lat_cols) - 2)
    fcol = col - c0
    row = (ci * int(round(SAMPLE_SPACING / SURF_DS))) % rows
    road_y = (surf[row, c0] * (1 - fcol) + surf[row, c0 + 1] * fcol) - origin_y
    w = half[ci]
    flat = w + 3.5
    # the scanned terrain is right next to the road, so only a short blend is needed
    blend = np.clip((np.abs(lateral) - flat) / 8.0, 0.0, 1.0)
    blend = blend * blend * (3 - 2 * blend)
    # Under the road surface the terrain sits just below so it never pokes through.
    under = np.clip((np.abs(lateral) - w) / 3.5, 0.0, 1.0)  # 0 under the road .. 1 at the verge's outer edge
    road_level = road_y - 0.35 + 0.29 * under
    carved = road_level * (1 - blend) + height * blend
    height = np.where(np.abs(lateral) < flat + 8.0, carved, height)

    hmin, hmax = float(height.min()), float(height.max())
    q = np.clip((height - hmin) / (hmax - hmin) * 65535.0, 0, 65535).astype(np.uint16)
    Image.fromarray(q).save(os.path.join(OUT, "height_near.png"))

    # ------------------------------------------------------------------ far heightmap (horizon)
    half_far = FAR_SIZE / 2
    lat_a, lon_a = proj.to_latlon(np.array([-half_far, half_far]), np.array([half_far, -half_far]))
    dem_far = Terrarium(11, lat_a.min() - 0.02, lat_a.max() + 0.02, lon_a.min() - 0.03, lon_a.max() + 0.03)
    nf = int(FAR_SIZE / FAR_RES) + 1
    fx = -half_far + np.arange(nf) * FAR_RES
    FX, FZ = np.meshgrid(fx, fx)
    flat_, flon_ = proj.to_latlon(FX, FZ)
    far = dem_far.sample(flat_, flon_) - origin_y
    fmin, fmax = float(far.min()), float(far.max())
    qf = np.clip((far - fmin) / (fmax - fmin) * 65535.0, 0, 65535).astype(np.uint16)
    Image.fromarray(qf).save(os.path.join(OUT, "height_far.png"))
    print(f"far heightmap {nf}x{nf} @ {FAR_RES} m")

    # ------------------------------------------------------------------ landcover
    lat_b, lon_b = proj.to_latlon(np.array([xmin, xmax]), np.array([zmax, zmin]))
    polys, osm_buildings = load_landcover(lat_b.min(), lat_b.max(), lon_b.min(), lon_b.max())
    layers = [Image.new("L", (nx, nz), 0) for _ in range(3)]
    draws = [ImageDraw.Draw(l) for l in layers]

    def to_px(ring):
        ll = np.array(ring)
        px, pz = proj.to_xz(ll[:, 0], ll[:, 1])
        return list(zip(((px - xmin) / NEAR_RES).tolist(), ((pz - zmin) / NEAR_RES).tolist()))

    for cls, outers, inners in polys:
        for r in outers:
            if len(r) >= 3:
                draws[cls].polygon(to_px(r), fill=255)
        for r in inners:
            if len(r) >= 3:
                draws[cls].polygon(to_px(r), fill=0)
    lc = np.stack([np.asarray(l) for l in layers], axis=-1).astype(np.float32)
    # Keep the road corridor clear of trees and soften edges.
    corridor = np.abs(lateral) < (w + 9.0)
    lc[..., 0][corridor] = 0
    lc = ndimage.gaussian_filter(lc, sigma=(1.2, 1.2, 0))
    Image.fromarray(np.clip(lc, 0, 255).astype(np.uint8)).save(os.path.join(OUT, "landcover.png"))
    forest_frac = (lc[..., 0] > 127).mean()
    print(f"landcover: forest covers {forest_frac * 100:.1f}% of the near area")

    # ------------------------------------------------------------------ buildings near the circuit
    records = []
    for tags, ring in osm_buildings:
        ll = np.array(ring)
        bx, bz = proj.to_xz(ll[:, 0], ll[:, 1])
        d, _ = ctree.query(np.stack([bx, bz], axis=1))
        if d.min() > 350.0 or d.min() < 9.0:   # nothing on the track itself
            continue
        records.append(building_record(tags, np.stack([bx, bz], axis=1)))
    with open(os.path.join(OUT, "buildings.json"), "w", encoding="utf-8") as f:
        json.dump({"buildings": records}, f, separators=(",", ":"))
    print(f"buildings: {len(records)} within 350 m of the circuit")

    # bridge decks (without the interpolation margin), as s ranges
    bridges = []
    r = 0
    while r < rows:
        if bmask[r]:
            r0 = r
            while r < rows and bmask[r]:
                r += 1
            bridges.append([round((r0 * SURF_DS + BRIDGE_MARGIN) % length, 1), round(((r - 1) * SURF_DS - BRIDGE_MARGIN) % length, 1)])
        r += 1

    meta = {
        "name": "Nürburgring Nordschleife",
        "length": float(n * SAMPLE_SPACING),
        "sampleSpacing": SAMPLE_SPACING,
        "origin": {"lat": lat0, "lon": lon0, "elevation": origin_y},
        "startOffset": 0.0,
        "sections": sections,
        "heightNear": {"file": "height_near.png", "x0": float(xmin), "z0": float(zmin), "res": NEAR_RES,
                        "w": nx, "h": nz, "min": hmin, "max": hmax},
        "heightFar": {"file": "height_far.png", "x0": float(-half_far), "z0": float(-half_far), "res": FAR_RES,
                       "w": nf, "h": nf, "min": fmin, "max": fmax},
        "landcover": {"file": "landcover.png"},
        "buildings": {"file": "buildings.json"},
        "bridges": bridges,
        "roadSurface": {"file": "road_surface.png", "ds": SURF_DS, "lat0": -SURF_HALF, "dl": SURF_DL,
                        "rows": rows, "cols": len(lat_cols), "scale": SURF_SCALE},
        "attribution": [
            "Track layout, corner names and land cover (c) OpenStreetMap contributors, ODbL 1.0",
            "Road surface and terrain: DGM1 laser scan (c) GeoBasis-DE / LVermGeoRP (2025), dl-de/by-2-0",
            "Distant elevation: AWS Terrain Tiles (Mapzen terrarium), derived from SRTM/EU-DEM/GMTED",
        ],
    }
    with open(os.path.join(OUT, "track.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)
    print(f"sections: {len(sections)}; wrote {OUT}")


if __name__ == "__main__":
    sys.exit(main())
