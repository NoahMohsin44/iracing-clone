// Direction for texel (u, v) in [0,1] of cubemap face (GL face order +X -X +Y -Y +Z -Z).
vec3 cubeDir(int face, vec2 uv) {
    vec2 c = uv * 2.0 - 1.0;
    vec3 d;
    if (face == 0) d = vec3(1.0, -c.y, -c.x);
    else if (face == 1) d = vec3(-1.0, -c.y, c.x);
    else if (face == 2) d = vec3(c.x, 1.0, c.y);
    else if (face == 3) d = vec3(c.x, -1.0, -c.y);
    else if (face == 4) d = vec3(c.x, -c.y, 1.0);
    else d = vec3(-c.x, -c.y, -1.0);
    return normalize(d);
}
