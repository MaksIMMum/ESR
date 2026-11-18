#!/usr/bin/env python3
"""
generate_lowpoly_car.py - Procedural low-poly sports car generator
Generates a sleek, low-poly sports car OBJ file with vertex colors and 100% outward-facing normals.
"""

import math
import os

def build_car():
    verts = []
    colors = []
    faces = []

    def add_quad(v0, v1, v2, v3, col, center=(0.0, 0.05, 0.0)):
        fc = [(v0[i] + v1[i] + v2[i] + v3[i]) / 4.0 for i in range(3)]
        to_face = [fc[i] - center[i] for i in range(3)]
        
        ax, ay, az = v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]
        bx, by, bz = v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]
        nx = ay * bz - az * by
        ny = az * bx - ax * bz
        nz = ax * by - ay * bx
        
        if nx * to_face[0] + ny * to_face[1] + nz * to_face[2] < 0:
            v1, v3 = v3, v1

        idx = len(verts)
        verts.extend([v0, v1, v2, v3])
        colors.extend([col, col, col, col])
        faces.append([idx, idx + 1, idx + 2])
        faces.append([idx, idx + 2, idx + 3])

    def add_tri(v0, v1, v2, col, center=(0.0, 0.05, 0.0)):
        fc = [(v0[i] + v1[i] + v2[i]) / 3.0 for i in range(3)]
        to_face = [fc[i] - center[i] for i in range(3)]
        
        ax, ay, az = v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]
        bx, by, bz = v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]
        nx = ay * bz - az * by
        ny = az * bx - ax * bz
        nz = ax * by - ay * bx
        
        if nx * to_face[0] + ny * to_face[1] + nz * to_face[2] < 0:
            v1, v2 = v2, v1

        idx = len(verts)
        verts.extend([v0, v1, v2])
        colors.extend([col, col, col])
        faces.append([idx, idx + 1, idx + 2])

    C_BODY    = (0.92, 0.14, 0.14)  # Vibrant Sports Red
    C_HOOD    = (0.95, 0.20, 0.20)  # Hood Red
    C_ROOF    = (0.80, 0.10, 0.10)  # Roof Darker Red
    C_GLASS   = (0.45, 0.78, 0.95)  # Ice Blue Glass
    C_LIGHT_F = (1.00, 0.95, 0.40)  # Yellow Xenon Headlight
    C_LIGHT_R = (0.95, 0.05, 0.05)  # Ruby Taillight
    C_TIRE    = (0.12, 0.12, 0.14)  # Charcoal Tire
    C_RIM     = (0.82, 0.82, 0.88)  # Silver Alloy Rim
    C_WING    = (0.15, 0.15, 0.18)  # Black Carbon Wing
    C_GRILL   = (0.08, 0.08, 0.08)  # Black Grill
    C_UNDER   = (0.15, 0.15, 0.15)  # Floor

    # 1. Hood (y = 0.08 slanting down to 0.02 at z=1.10)
    add_quad((-0.52, 0.08, 0.25), (0.52, 0.08, 0.25), (0.48, 0.02, 1.10), (-0.48, 0.02, 1.10), C_HOOD)

    # 2. Trunk (z in [-1.10, -0.55], y in [0.08, 0.08])
    add_quad((-0.48, 0.08, -1.10), (0.48, 0.08, -1.10), (0.52, 0.08, -0.55), (-0.52, 0.08, -0.55), C_BODY)

    # 3. Front Nose / Bumper (z = 1.10, y in [-0.15, 0.02])
    add_quad((-0.24, 0.02, 1.10), (0.24, 0.02, 1.10), (0.24, -0.15, 1.10), (-0.24, -0.15, 1.10), C_GRILL)
    add_quad((-0.48, 0.02, 1.10), (-0.24, 0.02, 1.10), (-0.24, -0.15, 1.10), (-0.48, -0.15, 1.10), C_LIGHT_F)
    add_quad((0.24, 0.02, 1.10), (0.48, 0.02, 1.10), (0.48, -0.15, 1.10), (0.24, -0.15, 1.10), C_LIGHT_F)

    # 4. Rear Bumper (z = -1.10, y in [-0.15, 0.08])
    add_quad((0.24, 0.08, -1.10), (-0.24, 0.08, -1.10), (-0.24, -0.15, -1.10), (0.24, -0.15, -1.10), C_BODY)
    add_quad((-0.24, 0.08, -1.10), (-0.48, 0.08, -1.10), (-0.48, -0.04, -1.10), (-0.24, -0.04, -1.10), C_LIGHT_R)
    add_quad((-0.24, -0.04, -1.10), (-0.48, -0.04, -1.10), (-0.48, -0.15, -1.10), (-0.24, -0.15, -1.10), C_BODY)
    add_quad((0.48, 0.08, -1.10), (0.24, 0.08, -1.10), (0.24, -0.04, -1.10), (0.48, -0.04, -1.10), C_LIGHT_R)
    add_quad((0.48, -0.04, -1.10), (0.24, -0.04, -1.10), (0.24, -0.15, -1.10), (0.48, -0.15, -1.10), C_BODY)

    # 5. Lower Chassis Sides (y in [-0.15, 0.08])
    # Left side (x = -0.52 to -0.48)
    add_quad((-0.52, 0.08, -0.55), (-0.52, 0.08, 0.25), (-0.52, -0.15, 0.25), (-0.52, -0.15, -0.55), C_BODY)
    add_quad((-0.52, 0.08, 0.25), (-0.48, 0.02, 1.10), (-0.48, -0.15, 1.10), (-0.52, -0.15, 0.25), C_BODY)
    add_quad((-0.48, 0.08, -1.10), (-0.52, 0.08, -0.55), (-0.52, -0.15, -0.55), (-0.48, -0.15, -1.10), C_BODY)

    # Right side (x = 0.52 to 0.48)
    add_quad((0.52, 0.08, 0.25), (0.52, 0.08, -0.55), (0.52, -0.15, -0.55), (0.52, -0.15, 0.25), C_BODY)
    add_quad((0.48, 0.02, 1.10), (0.52, 0.08, 0.25), (0.52, -0.15, 0.25), (0.48, -0.15, 1.10), C_BODY)
    add_quad((0.52, 0.08, -0.55), (0.48, 0.08, -1.10), (0.48, -0.15, -1.10), (0.52, -0.15, -0.55), C_BODY)

    # 6. Floor (y = -0.15)
    add_quad((0.48, -0.15, 1.10), (-0.48, -0.15, 1.10), (-0.48, -0.15, -1.10), (0.48, -0.15, -1.10), C_UNDER, center=(0, -0.10, 0))

    # 7. Cabin / Cockpit:
    # Windshield (z: 0.25 -> -0.05, y: 0.08 -> 0.30)
    add_quad((-0.42, 0.08, 0.25), (0.42, 0.08, 0.25), (0.36, 0.30, -0.05), (-0.36, 0.30, -0.05), C_GLASS)
    # Roof (z: -0.05 -> -0.42, y = 0.30)
    add_quad((-0.36, 0.30, -0.05), (0.36, 0.30, -0.05), (0.36, 0.30, -0.42), (-0.36, 0.30, -0.42), C_ROOF)
    # Rear Window (z: -0.42 -> -0.55, y: 0.30 -> 0.08)
    add_quad((-0.36, 0.30, -0.42), (0.36, 0.30, -0.42), (0.42, 0.08, -0.55), (-0.42, 0.08, -0.55), C_GLASS)
    # Left side window
    add_quad((-0.36, 0.30, -0.42), (-0.36, 0.30, -0.05), (-0.42, 0.08, 0.25), (-0.42, 0.08, -0.55), C_GLASS)
    # Right side window
    add_quad((0.36, 0.30, -0.05), (0.36, 0.30, -0.42), (0.42, 0.08, -0.55), (0.42, 0.08, 0.25), C_GLASS)

    # 8. Rear Spoiler / Wing
    # Blade at y = 0.20, z in [-1.10, -0.85], x in [-0.55, 0.55]
    add_quad((-0.55, 0.20, -0.85), (0.55, 0.20, -0.85), (0.55, 0.20, -1.10), (-0.55, 0.20, -1.10), C_WING, center=(0, 0.19, -0.97))
    add_quad((0.55, 0.18, -0.85), (-0.55, 0.18, -0.85), (-0.55, 0.18, -1.10), (0.55, 0.18, -1.10), C_WING, center=(0, 0.21, -0.97))
    # Wing struts
    add_quad((-0.28, 0.08, -0.98), (-0.24, 0.08, -0.98), (-0.24, 0.19, -0.98), (-0.28, 0.19, -0.98), C_WING, center=(-0.26, 0.14, -0.98))
    add_quad((0.24, 0.08, -0.98), (0.28, 0.08, -0.98), (0.28, 0.19, -0.98), (0.24, 0.19, -0.98), C_WING, center=(0.26, 0.14, -0.98))

    # 9. 4 Wheels (Octagonal Cylinders)
    wheel_centers = [
        (-0.52, -0.12,  0.58, True),
        ( 0.52, -0.12,  0.58, False),
        (-0.52, -0.11, -0.62, True),
        ( 0.52, -0.11, -0.62, False),
    ]

    R = 0.135
    W = 0.10

    for (cx, cy, cz, is_left) in wheel_centers:
        x_out = cx - (W/2 if is_left else -W/2)
        x_in  = cx + (W/2 if is_left else -W/2)
        angles = [i * 2 * math.pi / 8 for i in range(8)]
        pts_out = [(x_out, cy + R * math.sin(a), cz + R * math.cos(a)) for a in angles]
        pts_in  = [(x_in,  cy + R * math.sin(a), cz + R * math.cos(a)) for a in angles]

        center_pt = (x_out, cy, cz)
        for i in range(8):
            p1 = pts_out[i]
            p2 = pts_out[(i + 1) % 8]
            # Hubcap / rim faces outward along X
            add_tri(center_pt, p1, p2, C_RIM, center=(0.0, cy, cz))

        for i in range(8):
            o1, o2 = pts_out[i], pts_out[(i + 1) % 8]
            i1, i2 = pts_in[i], pts_in[(i + 1) % 8]
            add_quad(o1, o2, i2, i1, C_TIRE, center=(cx, cy, cz))

    return verts, colors, faces

def save_obj(filepath, verts, colors, faces):
    with open(filepath, 'w') as f:
        f.write("# Sleek Low-Poly Sports Car for ESR Engine\n")
        f.write(f"# Vertices: {len(verts)}, Triangles: {len(faces)}\n\n")
        for v, c in zip(verts, colors):
            f.write(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f} {c[0]:.3f} {c[1]:.3f} {c[2]:.3f}\n")
        f.write("\n")
        for tri in faces:
            f.write(f"f {tri[0]+1} {tri[1]+1} {tri[2]+1}\n")

if __name__ == '__main__':
    verts, colors, faces = build_car()
    os.makedirs('assets', exist_ok=True)
    out_obj = 'assets/car.obj'
    save_obj(out_obj, verts, colors, faces)
    print(f"Saved {out_obj} ({len(verts)} vertices, {len(faces)} triangles)")
