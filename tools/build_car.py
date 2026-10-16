"""
Builds the GT3 car model (assets/cars/gt3/gt3.glb) with Blender's Python API.

    blender -b --factory-startup --python tools/build_car.py -- [--preview out_prefix]

Everything is modelled from code: the body is a loft through designed cross-sections (like a
designer's section drawings), mirrored and smoothed with Catmull-Clark subdivision, the wheel
arches are boolean cuts that also form the arch liners, and the aero parts, lights, wheels,
interior and driver are separate parts. Materials are named after the game's car material ids.

Coordinates: the game's car body frame (x right, y up, z back, origin at the centre of gravity,
ground at y = -0.43). Blender is z-up, so a point (x, y, z) is placed at (x, -z, y); the glTF
exporter's +Y-up conversion turns that back into the game frame exactly.
"""
import math
import os
import sys

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "assets", "cars", "gt3", "gt3.glb")

GROUND = -0.43
FRONT_AXLE, REAR_AXLE = -1.4445, 1.2555
FRONT_TRACK, REAR_TRACK = 1.66, 1.62
FRONT_R, REAR_R = 0.340, 0.355
FRONT_W, REAR_W = 0.30, 0.31
WHEEL_Y = {0: GROUND + 0.335, 1: GROUND + 0.350}   # wheel centre heights at static ride height


def B(x, y, z):
    """game body frame -> Blender"""
    return Vector((x, -z, y))


# ----------------------------------------------------------------------------- materials
MATERIALS = {
    # name: (base colour, metallic, roughness)  - the game shades by name, these are for previews
    "Body": ((0.80, 0.80, 0.78), 0.0, 0.3),
    "Carbon": ((0.03, 0.03, 0.035), 0.0, 0.35),
    "BlackPlastic": ((0.02, 0.02, 0.02), 0.0, 0.7),
    "Glass": ((0.05, 0.06, 0.07), 0.0, 0.05),
    "Headlight": ((0.9, 0.9, 0.95), 0.5, 0.1),
    "Taillight": ((0.5, 0.02, 0.02), 0.0, 0.2),
    "Chrome": ((0.9, 0.9, 0.9), 1.0, 0.1),
    "Rubber": ((0.02, 0.02, 0.02), 0.0, 0.9),
    "Rim": ((0.08, 0.08, 0.09), 1.0, 0.35),
    "BrakeDisc": ((0.3, 0.3, 0.3), 1.0, 0.5),
    "Caliper": ((0.8, 0.1, 0.05), 0.0, 0.4),
    "Interior": ((0.03, 0.03, 0.03), 0.0, 0.6),
    "Seat": ((0.05, 0.05, 0.06), 0.0, 0.8),
    "Display": ((0.02, 0.05, 0.08), 0.0, 0.2),
    "ArchLiner": ((0.015, 0.015, 0.015), 0.0, 0.8),
    "ShiftLeds": ((0.1, 0.5, 0.1), 0.0, 0.3),
    "Cage": ((0.25, 0.25, 0.27), 1.0, 0.4),
    "Helmet": ((0.9, 0.9, 0.9), 0.0, 0.3),
    "Visor": ((0.02, 0.02, 0.03), 0.0, 0.05),
    "Suit": ((0.1, 0.1, 0.4), 0.0, 0.8),
    "Trim": ((0.01, 0.01, 0.01), 0.0, 0.4),
}


def make_materials():
    mats = {}
    for name, (col, metal, rough) in MATERIALS.items():
        m = bpy.data.materials.new(name)
        m.use_nodes = True
        bsdf = m.node_tree.nodes["Principled BSDF"]
        bsdf.inputs["Base Color"].default_value = (*col, 1.0)
        bsdf.inputs["Metallic"].default_value = metal
        bsdf.inputs["Roughness"].default_value = rough
        m.diffuse_color = (*col, 1.0)
        mats[name] = m
    return mats


# ----------------------------------------------------------------------------- helpers
def new_object(name, bm, mats, mat_names):
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(ob)
    for n in mat_names:
        ob.data.materials.append(mats[n])
    return ob


def apply_modifiers(ob):
    bpy.context.view_layer.objects.active = ob
    for o in bpy.context.selected_objects:
        o.select_set(False)
    ob.select_set(True)
    for m in list(ob.modifiers):
        bpy.ops.object.modifier_apply(modifier=m.name)


def smooth_shade(ob, angle=math.radians(40)):
    for p in ob.data.polygons:
        p.use_smooth = True
    if hasattr(ob.data, "use_auto_smooth"):
        ob.data.use_auto_smooth = True
        ob.data.auto_smooth_angle = angle


def hermite(zs, vals, zq):
    """Monotone cubic (Fritsch-Carlson) interpolation of vals(zs) at zq."""
    zs = np.asarray(zs, float)
    vals = np.asarray(vals, float)
    h = np.diff(zs)
    d = np.diff(vals) / h
    m = np.zeros_like(vals)
    m[1:-1] = (d[:-1] + d[1:]) / 2
    m[0], m[-1] = d[0], d[-1]
    for k in range(len(d)):
        if d[k] == 0:
            m[k] = m[k + 1] = 0
        else:
            a, b = m[k] / d[k], m[k + 1] / d[k]
            s = a * a + b * b
            if s > 9:
                t = 3 / math.sqrt(s)
                m[k], m[k + 1] = t * a * d[k], t * b * d[k]
    k = np.clip(np.searchsorted(zs, zq) - 1, 0, len(zs) - 2)
    t = (zq - zs[k]) / h[k]
    h00 = 2 * t ** 3 - 3 * t ** 2 + 1
    h10 = t ** 3 - 2 * t ** 2 + t
    h01 = -2 * t ** 3 + 3 * t ** 2
    h11 = t ** 3 - t ** 2
    return h00 * vals[k] + h10 * h[k] * m[k] + h01 * vals[k + 1] + h11 * h[k] * m[k + 1]


def bump(z, centre, half):
    """smooth 0..1..0 bump of half-width half"""
    u = np.clip(np.abs(z - centre) / half, 0, 1)
    return 0.5 + 0.5 * np.cos(np.pi * u)


# ----------------------------------------------------------------------------- body shell
# Half cross-sections (x >= 0) from the bottom centre round to the top centre. 13 points:
#  0 floor centre, 1-2 floor, 3 sill corner, 4 lower side, 5 widest line, 6 upper side,
#  7 shoulder, 8 fender peak / belt, 9 hood edge / window base, 10 hood / roof rail,
#  11 hood / roof, 12 top centre.  Heights are body-frame y (ground at -0.43).
STATIONS = [
    (-2.425, [(0, -0.30), (0.20, -0.30), (0.40, -0.30), (0.52, -0.27), (0.60, -0.20), (0.64, -0.10), (0.63, -0.02),
              (0.58, 0.02), (0.48, 0.035), (0.36, 0.04), (0.24, 0.042), (0.12, 0.043), (0, 0.043)]),
    (-2.40, [(0, -0.345), (0.30, -0.345), (0.60, -0.345), (0.71, -0.31), (0.78, -0.21), (0.81, -0.09), (0.80, 0.0),
             (0.75, 0.05), (0.64, 0.075), (0.47, 0.08), (0.31, 0.083), (0.15, 0.085), (0, 0.085)]),
    (-2.22, [(0, -0.355), (0.35, -0.355), (0.72, -0.355), (0.84, -0.32), (0.90, -0.21), (0.925, -0.06), (0.915, 0.07),
             (0.88, 0.16), (0.79, 0.215), (0.63, 0.19), (0.43, 0.175), (0.21, 0.175), (0, 0.178)]),
    (-1.90, [(0, -0.36), (0.35, -0.36), (0.76, -0.36), (0.88, -0.32), (0.945, -0.19), (0.965, -0.03), (0.955, 0.13),
             (0.92, 0.265), (0.84, 0.33), (0.69, 0.285), (0.45, 0.26), (0.22, 0.262), (0, 0.268)]),
    (FRONT_AXLE, [(0, -0.36), (0.35, -0.36), (0.78, -0.36), (0.90, -0.32), (0.955, -0.18), (0.968, -0.02), (0.96, 0.16),
                  (0.93, 0.30), (0.855, 0.375), (0.71, 0.33), (0.46, 0.305), (0.23, 0.31), (0, 0.318)]),
    (-1.00, [(0, -0.36), (0.35, -0.36), (0.79, -0.36), (0.91, -0.32), (0.958, -0.18), (0.968, 0.0), (0.96, 0.19),
             (0.935, 0.33), (0.865, 0.40), (0.73, 0.365), (0.47, 0.35), (0.235, 0.355), (0, 0.36)]),
    (-0.70, [(0, -0.36), (0.35, -0.36), (0.80, -0.36), (0.925, -0.32), (0.962, -0.18), (0.970, 0.0), (0.962, 0.20),
             (0.935, 0.36), (0.87, 0.44), (0.77, 0.43), (0.55, 0.415), (0.27, 0.42), (0, 0.425)]),
    (-0.36, [(0, -0.36), (0.35, -0.36), (0.80, -0.36), (0.93, -0.32), (0.965, -0.18), (0.972, 0.0), (0.964, 0.20),
             (0.935, 0.36), (0.87, 0.47), (0.765, 0.53), (0.665, 0.62), (0.33, 0.64), (0, 0.645)]),
    (-0.08, [(0, -0.36), (0.35, -0.36), (0.80, -0.36), (0.93, -0.32), (0.965, -0.18), (0.972, 0.0), (0.964, 0.20),
             (0.935, 0.36), (0.87, 0.47), (0.755, 0.535), (0.59, 0.78), (0.30, 0.805), (0, 0.81)]),
    (0.30, [(0, -0.36), (0.35, -0.36), (0.80, -0.36), (0.93, -0.32), (0.965, -0.18), (0.973, 0.0), (0.966, 0.20),
            (0.94, 0.355), (0.875, 0.47), (0.755, 0.535), (0.585, 0.775), (0.30, 0.80), (0, 0.805)]),
    (0.75, [(0, -0.36), (0.35, -0.36), (0.80, -0.36), (0.93, -0.32), (0.968, -0.18), (0.978, 0.0), (0.972, 0.20),
            (0.955, 0.35), (0.905, 0.47), (0.77, 0.545), (0.58, 0.70), (0.30, 0.722), (0, 0.726)]),
    (REAR_AXLE, [(0, -0.36), (0.35, -0.36), (0.79, -0.36), (0.92, -0.32), (0.965, -0.18), (0.975, 0.0), (0.97, 0.20),
                 (0.955, 0.36), (0.91, 0.475), (0.77, 0.545), (0.56, 0.60), (0.29, 0.612), (0, 0.615)]),
    (1.75, [(0, -0.35), (0.35, -0.35), (0.77, -0.35), (0.90, -0.31), (0.955, -0.17), (0.962, 0.0), (0.955, 0.19),
            (0.935, 0.345), (0.885, 0.46), (0.76, 0.535), (0.55, 0.552), (0.28, 0.556), (0, 0.558)]),
    (2.12, [(0, -0.30), (0.33, -0.30), (0.70, -0.30), (0.82, -0.27), (0.885, -0.15), (0.905, 0.01), (0.90, 0.19),
            (0.885, 0.36), (0.845, 0.48), (0.73, 0.565), (0.52, 0.575), (0.27, 0.577), (0, 0.578)]),
    (2.27, [(0, -0.26), (0.30, -0.26), (0.64, -0.26), (0.76, -0.23), (0.83, -0.13), (0.85, 0.02), (0.845, 0.19),
            (0.83, 0.36), (0.79, 0.48), (0.69, 0.578), (0.50, 0.594), (0.26, 0.597), (0, 0.598)]),
    (2.305, [(0, -0.24), (0.25, -0.24), (0.53, -0.24), (0.65, -0.21), (0.72, -0.12), (0.75, 0.02), (0.75, 0.18),
             (0.735, 0.34), (0.70, 0.46), (0.61, 0.555), (0.44, 0.575), (0.22, 0.578), (0, 0.58)]),
]
NPT = 13


def flare(z, idx, axle):
    """wheel-arch flares: extra half-width at ring point idx near an axle (GT3 wide-body)."""
    zc, amount, half = (FRONT_AXLE, 0.05, 0.62) if axle == 0 else (REAR_AXLE, 0.06, 0.70)
    weight = {3: 0.35, 4: 0.8, 5: 1.0, 6: 1.0, 7: 0.75, 8: 0.25}.get(idx, 0.0)
    return amount * weight * bump(z, zc, half) ** 0.7


def body_rings():
    zs = np.array([s[0] for s in STATIONS])
    pts = np.array([s[1] for s in STATIONS])  # (stations, NPT, 2)
    # ring positions: dense at the ends, ~6 cm in the middle
    zq = np.concatenate([np.linspace(zs[0], -2.20, 10, endpoint=False), np.linspace(-2.20, 2.12, 72, endpoint=False),
                         np.linspace(2.12, zs[-1], 6)])
    rings = []
    for z in zq:
        ring = []
        for k in range(NPT):
            x = float(hermite(zs, pts[:, k, 0], z))
            y = float(hermite(zs, pts[:, k, 1], z))
            if k not in (0, NPT - 1):
                x += flare(z, k, 0) + flare(z, k, 1)
            else:
                x = 0.0
            ring.append((x, y))
        rings.append((z, ring))
    return rings


def build_body(mats):
    bm = bmesh.new()
    rings = body_rings()
    verts = []
    for z, ring in rings:
        verts.append([bm.verts.new(B(x, y, z)) for x, y in ring])
    for a, b in zip(verts[:-1], verts[1:]):
        for k in range(NPT - 1):
            bm.faces.new((a[k], a[k + 1], b[k + 1], b[k]))
    bm.faces.new(list(reversed(verts[0])))
    bm.faces.new(verts[-1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    # creases keep the flat floor, the tail face and the nose crisp under subdivision
    crease = bm.edges.layers.float.get("crease_edge") or bm.edges.layers.float.new("crease_edge")
    for a, b in zip(verts[:-1], verts[1:]):
        e = bm.edges.get((a[2], b[2]))
        if e:
            e[crease] = 1.0
    for ring, value in ((verts[-1], 0.75), (verts[0], 0.5)):
        for k in range(NPT - 1):
            e = bm.edges.get((ring[k], ring[k + 1]))
            if e:
                e[crease] = value
    ob = new_object("body", bm, mats, list(MATERIALS.keys()))
    mir = ob.modifiers.new("mirror", "MIRROR")
    mir.use_axis = (True, False, False)
    mir.use_clip = True
    mir.use_mirror_merge = True
    sub = ob.modifiers.new("subsurf", "SUBSURF")
    sub.levels = sub.render_levels = 2
    apply_modifiers(ob)
    return ob


def ring_point(k, z):
    zs = np.array([s[0] for s in STATIONS])
    pts = np.array([s[1] for s in STATIONS])
    return float(hermite(zs, pts[:, k, 0], z)), float(hermite(zs, pts[:, k, 1], z))


def cut_arches(body, mats):
    """Boolean the wheel openings out of the closed shell; the cutter walls become the liners."""
    cutters = []
    for axle, (zc, track, r) in enumerate(((FRONT_AXLE, FRONT_TRACK, FRONT_R + 0.055), (REAR_AXLE, REAR_TRACK, REAR_R + 0.05))):
        for side in (-1, 1):
            bm = bmesh.new()
            # a cylinder along x from just inboard of the tyre to well outside the body
            x0, x1 = side * (track / 2 - 0.26), side * 1.4
            n = 64
            ring0, ring1 = [], []
            for k in range(n):
                a = 2 * math.pi * k / n
                yy = WHEEL_Y[axle] + r * math.cos(a)
                zz = zc + r * math.sin(a) * (1.06 if axle == 0 else 1.0)  # room to steer
                ring0.append(bm.verts.new(B(x0, yy, zz)))
                ring1.append(bm.verts.new(B(x1, yy, zz)))
            for k in range(n):
                bm.faces.new((ring0[k], ring0[(k + 1) % n], ring1[(k + 1) % n], ring1[k]))
            bm.faces.new(ring0)
            bm.faces.new(list(reversed(ring1)))
            bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
            c = new_object(f"arch_cutter_{axle}_{side}", bm, mats, ["ArchLiner"])
            cutters.append(c)
    for c in cutters:
        mod = body.modifiers.new("arch", "BOOLEAN")
        mod.operation = "DIFFERENCE"
        mod.solver = "EXACT"
        mod.object = c
        if hasattr(mod, "material_mode"):
            mod.material_mode = "TRANSFER"
        apply_modifiers(body)
    for c in cutters:
        bpy.data.objects.remove(c)


# ----------------------------------------------------------------------------- wheels
def revolve(bm, profile, segments, mat_index=0, smooth=True, x_scale=1.0):
    """Revolve a (x, r) profile about the x axis (wheel axle). Returns the vertex rings."""
    rings = []
    for k in range(segments):
        a = 2 * math.pi * k / segments
        ca, sa = math.cos(a), math.sin(a)
        rings.append([bm.verts.new(B(x * x_scale, r * ca, r * sa)) for x, r in profile])
    faces = []
    for k in range(segments):
        a, b = rings[k], rings[(k + 1) % segments]
        for i in range(len(profile) - 1):
            if profile[i][1] < 1e-6 and profile[i + 1][1] < 1e-6:
                continue
            f = bm.faces.new((a[i], a[i + 1], b[i + 1], b[i]))
            f.material_index = mat_index
            f.smooth = smooth
            faces.append(f)
    return rings, faces


def box(bm, centre, half, mat_index=0, rot=None):
    """axis-aligned (in the game frame) box, optionally rotated by a 3x3 game-frame matrix"""
    cx, cy, cz = centre
    hx, hy, hz = half
    corners = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                v = np.array([sx * hx, sy * hy, sz * hz])
                if rot is not None:
                    v = rot @ v
                corners.append(bm.verts.new(B(cx + v[0], cy + v[1], cz + v[2])))
    idx = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    out = []
    for f in idx:
        face = bm.faces.new([corners[i] for i in f])
        face.material_index = mat_index
        out.append(face)
    return out


def build_wheel(mats, axle):
    """One wheel in its own frame: axle along x, outward = +x, centre at the origin."""
    R = FRONT_R if axle == 0 else REAR_R
    W = FRONT_W if axle == 0 else REAR_W
    rim_r = 0.2286 + 0.012   # 18 inch wheel + flange
    names = ["Rubber", "Rim", "BrakeDisc", "Chrome"]
    bm = bmesh.new()
    h = W / 2
    # slick: flat tread, rounded shoulders, bulging sidewall down to the rim flange
    tyre = [(-h + 0.012, rim_r - 0.004), (-h - 0.004, rim_r + 0.02), (-h - 0.010, rim_r + 0.05),
            (-h - 0.006, R - 0.045), (-h + 0.010, R - 0.012), (-h + 0.035, R - 0.001), (0.0, R),
            (h - 0.035, R - 0.001), (h - 0.010, R - 0.012), (h + 0.006, R - 0.045), (h + 0.010, rim_r + 0.05),
            (h + 0.004, rim_r + 0.02), (h - 0.012, rim_r - 0.004)]
    revolve(bm, tyre, 72, 0)
    # rim barrel (inside of the tyre, seen through the spokes) and outer lip
    barrel = [(h - 0.010, rim_r - 0.002), (h - 0.025, rim_r - 0.012), (-h + 0.02, rim_r - 0.012), (-h + 0.01, rim_r - 0.002)]
    revolve(bm, barrel, 48, 1)
    lip = [(h - 0.012, rim_r - 0.004), (h + 0.002, rim_r - 0.010), (h - 0.004, rim_r - 0.022), (h - 0.03, rim_r - 0.024)]
    revolve(bm, lip, 48, 1)
    # centre-lock hub and nut
    hub_x = h - 0.055
    revolve(bm, [(hub_x - 0.03, 0.0), (hub_x - 0.03, 0.085), (hub_x + 0.005, 0.085), (hub_x + 0.012, 0.07),
                 (hub_x + 0.012, 0.0)], 32, 1, smooth=False)
    revolve(bm, [(hub_x + 0.012, 0.0), (hub_x + 0.012, 0.045), (hub_x + 0.045, 0.040), (hub_x + 0.048, 0.022),
                 (hub_x + 0.048, 0.0)], 12, 3, smooth=False)
    # ten spokes from the hub to the rim lip, slightly dished
    for k in range(10):
        a = 2 * math.pi * k / 10 + (0.08 if k % 2 else -0.08)
        ca, sa = math.cos(a), math.sin(a)
        for i in range(8):
            t0, t1 = i / 8, (i + 1) / 8
            r0, r1 = 0.075 + (rim_r - 0.095) * t0, 0.075 + (rim_r - 0.095) * t1
            x0 = hub_x + 0.005 - 0.035 * t0 ** 1.5
            x1 = hub_x + 0.005 - 0.035 * t1 ** 1.5
            wdt0, wdt1 = 0.016 - 0.005 * t0, 0.016 - 0.005 * t1
            th = 0.018
            quad = []
            for (r, x, wd) in ((r0, x0, wdt0), (r1, x1, wdt1)):
                for sw in (-1, 1):
                    for sx in (0, -1):
                        quad.append(bm.verts.new(B(x + sx * th, r * ca - sw * wd * sa, r * sa + sw * wd * ca)))
            # quad: [r0 -w front, r0 -w back, r0 +w front, r0 +w back, r1 ...]
            for f in [(0, 2, 6, 4), (1, 5, 7, 3), (0, 4, 5, 1), (2, 3, 7, 6)]:
                face = bm.faces.new([quad[j] for j in f])
                face.material_index = 1
    # brake disc with hat
    disc_r = 0.190 if axle == 0 else 0.178
    revolve(bm, [(-0.05, 0.10), (-0.05, disc_r), (-0.016, disc_r), (-0.016, 0.10)], 64, 2, smooth=False)
    revolve(bm, [(-0.016, 0.075), (0.03, 0.075), (0.03, 0.10), (-0.016, 0.10)], 32, 2, smooth=False)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return new_object(f"wheel_{'front' if axle == 0 else 'rear'}", bm, mats, names)


def build_caliper(mats, axle):
    """Brake caliper (does not spin), in the wheel frame, sitting at the back of the disc."""
    disc_r = 0.190 if axle == 0 else 0.178
    bm = bmesh.new()
    n = 10
    a0, a1 = math.radians(95), math.radians(150)  # behind and above the axle (z back, y up)
    verts = []
    for k in range(n + 1):
        a = a0 + (a1 - a0) * k / n
        ring = []
        for (x, r) in ((-0.075, disc_r - 0.055), (-0.075, disc_r + 0.018), (0.012, disc_r + 0.018), (0.012, disc_r - 0.055)):
            ring.append(bm.verts.new(B(x, r * math.cos(a), r * math.sin(a))))
        verts.append(ring)
    for a, b in zip(verts[:-1], verts[1:]):
        for i in range(4):
            bm.faces.new((a[i], a[(i + 1) % 4], b[(i + 1) % 4], b[i]))
    bm.faces.new(list(reversed(verts[0])))
    bm.faces.new(verts[-1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return new_object(f"caliper_{'front' if axle == 0 else 'rear'}", bm, mats, ["Caliper"])


# ----------------------------------------------------------------------------- exterior parts
def tube(bm, a, b, r, seg=10, mat_index=0, caps=True):
    """cylinder between game-frame points a and b"""
    a, b = np.array(a, float), np.array(b, float)
    d = b - a
    L = np.linalg.norm(d)
    d /= L
    u = np.cross(d, [0, 1, 0] if abs(d[1]) < 0.9 else [1, 0, 0])
    u /= np.linalg.norm(u)
    v = np.cross(d, u)
    ra, rb = [], []
    for k in range(seg):
        t = 2 * math.pi * k / seg
        o = (u * math.cos(t) + v * math.sin(t)) * r
        ra.append(bm.verts.new(B(*(a + o))))
        rb.append(bm.verts.new(B(*(b + o))))
    for k in range(seg):
        f = bm.faces.new((ra[k], ra[(k + 1) % seg], rb[(k + 1) % seg], rb[k]))
        f.material_index = mat_index
        f.smooth = True
    if caps:
        bm.faces.new(list(reversed(ra))).material_index = mat_index
        bm.faces.new(rb).material_index = mat_index


def extrude_profile(bm, profile, x0, x1, mat_index=0, smooth=True, closed=True):
    """Extrude a (z, y) profile along x from x0 to x1 (for wings, splitters...)."""
    ra = [bm.verts.new(B(x0, y, z)) for z, y in profile]
    rb = [bm.verts.new(B(x1, y, z)) for z, y in profile]
    n = len(profile)
    for k in range(n if closed else n - 1):
        f = bm.faces.new((ra[k], ra[(k + 1) % n], rb[(k + 1) % n], rb[k]))
        f.material_index = mat_index
        f.smooth = smooth
    if closed:
        f = bm.faces.new(list(reversed(ra)))
        f.material_index = mat_index
        f = bm.faces.new(rb)
        f.material_index = mat_index


def airfoil(chord, thick, z_le, y_le, angle, n=14, camber=0.04):
    """(z, y) outline of a cambered airfoil, leading edge at (z_le, y_le), angle of attack in rad
    (positive = trailing edge up, i.e. downforce for an inverted wing)."""
    pts_u, pts_l = [], []
    for i in range(n + 1):
        t = 0.5 - 0.5 * math.cos(math.pi * i / n)
        yt = 5 * thick * (0.2969 * math.sqrt(t) - 0.126 * t - 0.3516 * t ** 2 + 0.2843 * t ** 3 - 0.1036 * t ** 4)
        yc = -camber * 4 * t * (1 - t)            # inverted camber
        pts_u.append((t, yc + yt))
        pts_l.append((t, yc - yt))
    outline = pts_u + pts_l[::-1][1:-1]
    ca, sa = math.cos(angle), math.sin(angle)
    out = []
    for t, h in outline:
        zz, yy = t * chord, h * chord
        out.append((z_le + zz * ca - yy * sa, y_le + zz * sa + yy * ca))
    return out


def build_exterior_parts(mats):
    names = list(MATERIALS.keys())
    M = {n: i for i, n in enumerate(names)}
    bm = bmesh.new()

    # ---- front splitter: flat carbon plate following the nose, with end plates
    outline = []
    for k in range(17):
        t = k / 16
        x = -0.98 + 1.96 * t
        z = -2.36 - 0.17 * (1 - (x / 0.98) ** 2) ** 0.7
        outline.append((x, z))
    back = [(0.98, -1.88), (-0.98, -1.88)]
    for y0, y1 in ((-0.372, -0.352),):
        top = [bm.verts.new(B(x, y1, z)) for x, z in outline + back]
        bot = [bm.verts.new(B(x, y0, z)) for x, z in outline + back]
        nloop = len(top)
        bm.faces.new(top[::-1]).material_index = M["Carbon"]
        bm.faces.new(bot).material_index = M["Carbon"]
        for k in range(nloop):
            bm.faces.new((top[k], top[(k + 1) % nloop], bot[(k + 1) % nloop], bot[k])).material_index = M["Carbon"]
    for sd in (-1, 1):
        box(bm, (sd * 0.975, -0.31, -2.20), (0.008, 0.055, 0.17), M["Carbon"])
        # dive planes (canards) on the bumper corners
        for i, (y, z0) in enumerate(((-0.20, -2.30), (-0.08, -2.25))):
            rot = np.array([[1, 0, 0], [0, math.cos(0.25), -math.sin(0.25)], [0, math.sin(0.25), math.cos(0.25)]])
            box(bm, (sd * (0.86 + 0.02 * i), y, z0 + 0.06), (0.075, 0.004, 0.075), M["Carbon"], rot)
        # side skirts
        box(bm, (sd * 0.955, -0.335, -0.12), (0.035, 0.03, 0.80), M["Carbon"])
        # mirrors: carbon stalk + painted housing
        tube(bm, (sd * 0.86, 0.50, -0.40), (sd * 0.99, 0.53, -0.43), 0.012, 8, M["Carbon"])
        mir = bmesh.ops.create_uvsphere(bm, u_segments=16, v_segments=10, radius=1.0)
        for v in mir["verts"]:
            g = np.array([v.co.x * 0.085, v.co.z * 0.036, -v.co.y * 0.075])  # game-frame offsets
            if g[2] > 0:
                g[2] *= 0.35                              # flat mirror face at the back
            v.co = B(sd * 1.03 + g[0], 0.545 + g[1], -0.43 + g[2])
        for f in {f for v in mir["verts"] for f in v.link_faces}:
            f.material_index = M["Body"]
            f.smooth = True
        box(bm, (sd * 1.03, 0.545, -0.401), (0.072, 0.026, 0.002), M["Chrome"])
        # exhaust exits in the side, ahead of the rear wheels
        for z in (0.66, 0.78):
            tube(bm, (sd * 0.93, -0.27, z), (sd * 0.985, -0.27, z), 0.045, 14, M["Chrome"])
        # wing endplates + swan-neck mounts
        ep = [(1.98, 0.62), (2.48, 0.64), (2.50, 0.90), (2.02, 0.88)]
        extrude_profile(bm, [(z, y) for z, y in ep], sd * 0.905, sd * 0.915, M["Carbon"], smooth=False)
        neck = [(1.92, 0.52), (1.97, 0.70), (2.08, 0.80), (2.18, 0.835), (2.25, 0.835), (2.20, 0.80),
                (2.09, 0.75), (2.03, 0.62), (2.00, 0.52)]
        extrude_profile(bm, neck, sd * 0.33, sd * 0.345, M["Carbon"], smooth=False)

    # ---- rear wing: main plane + gurney-flapped upper element
    main = airfoil(0.30, 0.10, 2.06, 0.765, -0.10)
    extrude_profile(bm, main, -0.905, 0.905, M["Carbon"])
    flap = airfoil(0.14, 0.09, 2.33, 0.795, -0.55, camber=0.06)
    extrude_profile(bm, flap, -0.905, 0.905, M["Carbon"])
    box(bm, (0.0, 0.86, 2.462), (0.905, 0.012, 0.002), M["Carbon"])  # gurney

    # ---- rear diffuser: floor ramp with strakes
    diff = [(1.90, -0.355), (2.40, -0.20), (2.40, -0.188), (1.90, -0.343)]
    extrude_profile(bm, diff, -0.86, 0.86, M["Carbon"], smooth=False)
    for xs in (-0.62, -0.30, 0.0, 0.30, 0.62):
        fin = [(1.95, -0.345), (2.42, -0.19), (2.42, -0.36), (2.10, -0.36)]
        extrude_profile(bm, fin, xs - 0.004, xs + 0.004, M["Carbon"], smooth=False)

    # ---- windscreen wiper, towing strap, roof antenna, fuel filler
    tube(bm, (-0.05, 0.505, -0.535), (-0.62, 0.53, -0.50), 0.008, 6, M["BlackPlastic"])
    box(bm, (0.42, -0.33, -2.43), (0.02, 0.025, 0.012), M["Caliper"])
    tube(bm, (0.0, 0.80, 0.42), (0.0, 0.86, 0.48), 0.004, 5, M["BlackPlastic"])
    box(bm, (0.62, 0.62, 0.65), (0.07, 0.006, 0.07), M["Chrome"])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return new_object("exterior", bm, mats, names)


# ----------------------------------------------------------------------------- cockpit
STEERING_CENTRE = (-0.37, 0.33, -0.29)
STEERING_TILT = 0.42


def build_interior(mats):
    names = list(MATERIALS.keys())
    M = {n: i for i, n in enumerate(names)}
    bm = bmesh.new()
    # dashboard: wraps under the windscreen, carbon-look, with the driver's display
    dash = [(-0.80, 0.27), (-0.66, 0.35), (-0.50, 0.365), (-0.44, 0.345), (-0.45, 0.20), (-0.54, 0.08), (-0.80, 0.06)]
    extrude_profile(bm, dash, -0.84, 0.84, M["Interior"], smooth=False)
    box(bm, (-0.37, 0.315, -0.445), (0.09, 0.038, 0.006), M["Display"],
        np.array([[1, 0, 0], [0, math.cos(-0.35), -math.sin(-0.35)], [0, math.sin(-0.35), math.cos(-0.35)]]))
    # centre console with the switch panel
    box(bm, (0.0, -0.17, -0.18), (0.11, 0.15, 0.36), M["Interior"])
    box(bm, (0.0, 0.03, -0.42), (0.13, 0.17, 0.05), M["Interior"])
    box(bm, (0.0, 0.10, -0.368), (0.10, 0.07, 0.004), M["Display"])
    for i in range(8):
        box(bm, (-0.075 + (i % 4) * 0.05, -0.00 + (i // 4) * 0.035, -0.366), (0.012, 0.009, 0.006), M["Caliper"] if i == 0 else M["BlackPlastic"])
    # floor and footwell
    box(bm, (0.0, -0.33, 0.0), (0.88, 0.012, 1.0), M["Interior"])
    # steering column
    cx, cy, cz = STEERING_CENTRE
    col_dir = np.array([0.0, -math.sin(STEERING_TILT), -math.cos(STEERING_TILT)])
    tube(bm, (cx, cy, cz) + col_dir * 0.04, (cx, cy, cz) + col_dir * 0.36, 0.03, 10, M["Interior"])
    # bucket seat: shell, side bolsters and head restraint wings
    box(bm, (-0.37, -0.27, 0.33), (0.23, 0.05, 0.25), M["Seat"])
    seat_rot = np.array([[1, 0, 0], [0, math.cos(-0.25), -math.sin(-0.25)], [0, math.sin(-0.25), math.cos(-0.25)]])
    box(bm, (-0.37, 0.12, 0.57), (0.23, 0.36, 0.04), M["Seat"], seat_rot)
    for sd in (-1, 1):
        box(bm, (-0.37 + sd * 0.21, -0.12, 0.35), (0.03, 0.10, 0.24), M["Seat"])
        box(bm, (-0.37 + sd * 0.17, 0.48, 0.55), (0.04, 0.12, 0.10), M["Seat"], seat_rot)
    # harness belts
    for sd in (-1, 1):
        tube(bm, (-0.37 + sd * 0.09, 0.42, 0.56), (-0.37 + sd * 0.07, 0.10, 0.40), 0.018, 4, M["Suit"])
    # fire extinguisher in the passenger footwell
    tube(bm, (0.40, -0.25, -0.55), (0.40, -0.25, -0.20), 0.055, 12, M["Caliper"])
    # roll cage (welded-in, follows the roof)
    r = 0.021
    for sd in (-1, 1):
        x0 = sd * 0.80
        a = (x0, -0.31, -0.74)
        b = (sd * 0.70, 0.42, -0.50)
        c = (sd * 0.55, 0.715, -0.08)
        d = (sd * 0.53, 0.715, 0.62)
        e = (sd * 0.80, -0.31, 0.62)
        tube(bm, a, b, r, 10, M["Cage"])
        tube(bm, b, c, r, 10, M["Cage"])
        tube(bm, c, d, r, 10, M["Cage"])
        tube(bm, (sd * 0.53, 0.715, 0.62), (sd * 0.66, 0.50, 0.62), r, 10, M["Cage"])
        tube(bm, (sd * 0.66, 0.50, 0.62), e, r, 10, M["Cage"])
        # NASCAR-style door bars
        tube(bm, (sd * 0.86, -0.20, -0.68), (sd * 0.86, 0.22, 0.58), r, 10, M["Cage"])
        tube(bm, (sd * 0.86, 0.20, -0.66), (sd * 0.86, -0.22, 0.58), r, 10, M["Cage"])
        tube(bm, (sd * 0.86, -0.20, -0.68), (x0, -0.31, -0.74), r, 8, M["Cage"])
        tube(bm, (sd * 0.86, 0.22, 0.58), (sd * 0.66, 0.50, 0.62), r, 8, M["Cage"])
    tube(bm, (-0.55, 0.715, -0.08), (0.55, 0.715, -0.08), r, 10, M["Cage"])
    tube(bm, (-0.53, 0.715, 0.62), (0.53, 0.715, 0.62), r, 10, M["Cage"])
    tube(bm, (-0.66, 0.50, 0.62), (0.80, -0.31, 0.62), r, 10, M["Cage"])
    tube(bm, (-0.70, 0.42, -0.50), (0.70, 0.42, -0.50), r, 10, M["Cage"])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return new_object("interior", bm, mats, names)


def build_steering_wheel(mats):
    """Local frame: rim in the x-y plane facing +z (towards the driver), column along -z."""
    names = list(MATERIALS.keys())
    M = {n: i for i, n in enumerate(names)}
    bm = bmesh.new()
    # carbon body plate: GT3 'butterfly' shape
    outline = []
    for k in range(24):
        a = 2 * math.pi * k / 24
        x = 0.135 * math.copysign(abs(math.cos(a)) ** 0.6, math.cos(a))
        y = 0.075 * math.copysign(abs(math.sin(a)) ** 0.8, math.sin(a)) - (0.012 if math.sin(a) < 0 else 0.0)
        outline.append((x, y))
    front = [bm.verts.new(B(x, y, 0.012)) for x, y in outline]
    rear = [bm.verts.new(B(x, y, -0.008)) for x, y in outline]
    bm.faces.new(front).material_index = M["Carbon"]
    bm.faces.new(list(reversed(rear))).material_index = M["Carbon"]
    for k in range(24):
        bm.faces.new((front[k], rear[k], rear[(k + 1) % 24], front[(k + 1) % 24])).material_index = M["Carbon"]
    # rubber grips either side
    for sd in (-1, 1):
        pts = [(sd * (0.120 + 0.03 * math.sin(math.pi * t)), -0.075 + 0.15 * t, 0.006) for t in np.linspace(0, 1, 7)]
        for a, b in zip(pts[:-1], pts[1:]):
            tube(bm, a, b, 0.020, 10, M["Rubber"])
    # display, shift lights, buttons, rotaries
    box(bm, (0.0, 0.0, 0.0135), (0.048, 0.03, 0.0015), M["Display"])
    for i in range(10):
        box(bm, (-0.063 + i * 0.014, 0.052, 0.0135), (0.005, 0.004, 0.002), M["ShiftLeds"])
    cols = ["Caliper", "Headlight", "Suit", "Helmet"]
    for i in range(8):
        sd = -1 if i < 4 else 1
        box(bm, (sd * (0.068 + (i % 2) * 0.022), -0.012 + ((i % 4) // 2) * 0.026, 0.014), (0.007, 0.007, 0.003),
            M[cols[i % 4]])
    for sd in (-1, 1):
        tube(bm, (sd * 0.035, -0.052, 0.012), (sd * 0.035, -0.052, 0.022), 0.011, 10, M["Chrome"])
    # shift paddles behind the wheel
    for sd in (-1, 1):
        box(bm, (sd * 0.10, 0.02, -0.022), (0.05, 0.03, 0.003), M["Carbon"])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return new_object("steering_wheel", bm, mats, names)


def build_driver(mats):
    names = list(MATERIALS.keys())
    M = {n: i for i, n in enumerate(names)}
    bm = bmesh.new()

    def ellipsoid(centre, radii, mat, seg=(16, 10)):
        res = bmesh.ops.create_uvsphere(bm, u_segments=seg[0], v_segments=seg[1], radius=1.0)
        for v in res["verts"]:
            g = np.array([v.co.x * radii[0], v.co.z * radii[1], -v.co.y * radii[2]])
            v.co = B(centre[0] + g[0], centre[1] + g[1], centre[2] + g[2])
        for f in {f for v in res["verts"] for f in v.link_faces}:
            f.material_index = M[mat]
            f.smooth = True

    ellipsoid((-0.37, 0.61, 0.22), (0.135, 0.15, 0.155), "Helmet")
    ellipsoid((-0.37, 0.605, 0.128), (0.105, 0.055, 0.06), "Visor")
    ellipsoid((-0.37, 0.43, 0.30), (0.17, 0.07, 0.13), "Suit")                 # HANS / shoulders
    ellipsoid((-0.37, 0.18, 0.40), (0.19, 0.27, 0.12), "Suit")                 # torso
    cx, cy, cz = STEERING_CENTRE
    for sd in (-1, 1):
        shoulder = np.array([-0.37 + sd * 0.19, 0.40, 0.30])
        hand = np.array([cx + sd * 0.13, cy, cz + 0.03])
        elbow = (shoulder + hand) / 2 + np.array([sd * 0.08, -0.13, 0.03])
        tube(bm, shoulder, elbow, 0.047, 10, M["Suit"])
        tube(bm, elbow, hand, 0.040, 10, M["Suit"])
        ellipsoid(hand, (0.035, 0.045, 0.04), "Interior", (8, 6))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return new_object("driver", bm, mats, names)


def place_preview_wheels(wheels, calipers):
    """Instances at the four corners, for previews only."""
    inst = []
    for axle in (0, 1):
        zc = FRONT_AXLE if axle == 0 else REAR_AXLE
        tr = FRONT_TRACK if axle == 0 else REAR_TRACK
        for side in (-1, 1):
            for src in (wheels[axle], calipers[axle]):
                ob = src.copy()
                ob.hide_render = False
                bpy.context.collection.objects.link(ob)
                ob.scale = (side, 1, 1)
                ob.location = B(side * tr / 2, WHEEL_Y[axle], zc)
                inst.append(ob)
    return inst


# ----------------------------------------------------------------------------- previews
def preview(prefix, objects):
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.light = "STUDIO"
    scene.display.shading.color_type = "MATERIAL"
    scene.display.shading.show_cavity = True
    scene.render.resolution_x, scene.render.resolution_y = 960, 540
    scene.world = bpy.data.worlds.new("w") if scene.world is None else scene.world
    cam_data = bpy.data.cameras.new("cam")
    cam_data.lens = 35
    cam = bpy.data.objects.new("cam", cam_data)
    bpy.context.collection.objects.link(cam)
    scene.camera = cam
    shots = {"front34": (B(-4.6, 1.0, -5.2), B(0, 0.0, 0)), "side": (B(-7.2, 0.15, 0.0), B(0, 0.0, 0)),
             "rear34": (B(4.4, 1.3, 5.4), B(0, 0.05, 0.3)), "top": (B(0, 9.5, 0.01), B(0, 0, 0)),
             "front": (B(0, 0.25, -7.0), B(0, 0.05, 0)), "rear": (B(0, 0.35, 7.0), B(0, 0.05, 0))}
    for name, (eye, target) in shots.items():
        cam.location = eye
        d = target - eye
        cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
        scene.render.filepath = f"{prefix}_{name}.png"
        bpy.ops.render.render(write_still=True)


def export(objects):
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    for ob in bpy.data.objects:
        ob.select_set(ob in objects)
    bpy.ops.export_scene.gltf(filepath=OUT, export_format="GLB", use_selection=True, export_apply=True,
                              export_yup=True, export_normals=True, export_texcoords=False,
                              export_materials="EXPORT", export_cameras=False, export_lights=False)
    print(f"wrote {OUT} ({os.path.getsize(OUT) / 1e6:.2f} MB)")


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    bpy.ops.wm.read_factory_settings(use_empty=True)
    mats = make_materials()
    body = build_body(mats)
    cut_arches(body, mats)
    smooth_shade(body)
    exterior = build_exterior_parts(mats)
    interior = build_interior(mats)
    wheel_obj = build_steering_wheel(mats)
    driver = build_driver(mats)
    wheels = [build_wheel(mats, 0), build_wheel(mats, 1)]
    calipers = [build_caliper(mats, 0), build_caliper(mats, 1)]
    # the steering wheel is exported in its own frame; place a copy for previews
    if "--preview" in argv:
        sw = wheel_obj.copy()
        bpy.context.collection.objects.link(sw)
        sw.matrix_world = (Matrix.Translation(B(*STEERING_CENTRE)) @ Matrix.Rotation(-STEERING_TILT, 4, "X"))
        wheel_obj.hide_render = True
    export([body, exterior, interior, wheel_obj, driver] + wheels + calipers)
    if "--preview" in argv:
        for ob in wheels + calipers:
            ob.hide_render = True
        place_preview_wheels(wheels, calipers)
        preview(argv[argv.index("--preview") + 1], [body])


main()
