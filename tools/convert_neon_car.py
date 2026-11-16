#!/usr/bin/env python3
"""
convert_neon_car.py - Convert CubeCoders JetExamples esp32-neon-car model
into an ESR static Mesh3D header with full perspective-correct texture mapping.
"""

from PIL import Image
import os
import sys

def main():
    obj_path = 'assets/car.obj'
    inc_path = 'assets/livery_data.inc'
    out_path = 'include/models/car.h'

    if not os.path.exists(obj_path):
        print(f"Error: Missing {obj_path}", file=sys.stderr)
        sys.exit(1)

    raw_v = []
    raw_vt = []

    with open(obj_path, 'r') as f:
        lines = f.readlines()

    for line in lines:
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if parts[0] == 'v':
            raw_v.append([float(parts[1]), float(parts[2]), float(parts[3])])
        elif parts[0] == 'vt':
            raw_vt.append([float(parts[1]), float(parts[2])])

    # Centering & Normalization
    scale = 1.0 / 255.0
    y_center = 45.0

    unique_combos = {}
    mesh_vertices = []
    mesh_indices = []

    cur_mtl = 'body'

    for line in lines:
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if parts[0] == 'usemtl':
            cur_mtl = parts[1]
        elif parts[0] == 'f':
            face_toks = parts[1:]
            face_indices = []
            for tok in face_toks:
                p = tok.split('/')
                vi = int(p[0]) - 1
                vti = int(p[1]) - 1 if len(p) > 1 and p[1] else -1
                key = (vi, vti, cur_mtl)

                if key not in unique_combos:
                    idx = len(mesh_vertices)
                    unique_combos[key] = idx

                    # Transform position: negate X to fix mirrored model, center Y, scale
                    vx = -raw_v[vi][0] * scale
                    vy = (raw_v[vi][1] - y_center) * scale
                    vz = raw_v[vi][2] * scale

                    # Texture attributes stored in color: (u, v, is_window)
                    if 'Window' in cur_mtl:
                        u = 0.0
                        v = 0.0
                        is_window = 1.0
                    elif vti >= 0 and vti < len(raw_vt):
                        u, v = raw_vt[vti]
                        is_window = 0.0
                    else:
                        u = 0.0
                        v = 0.0
                        is_window = 0.0

                    mesh_vertices.append(((vx, vy, vz), (u, v, is_window)))

                face_indices.append(unique_combos[key])

            # Fan triangulation: keep original face order because negating X
            # already reversed the winding from CW (inward) to CCW (outward).
            for i in range(1, len(face_indices) - 1):
                mesh_indices.append(face_indices[0])
                mesh_indices.append(face_indices[i])
                mesh_indices.append(face_indices[i + 1])

    print(f"Generated Car Mesh:")
    print(f"  • Vertices: {len(mesh_vertices)}")
    print(f"  • Triangles: {len(mesh_indices) // 3} ({len(mesh_indices)} indices)")

    # Read livery texture data
    livery_inc_data = ""
    if os.path.exists(inc_path):
        with open(inc_path, 'r') as f:
            livery_inc_data = f.read()

    # Output C++ header
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'w') as f:
        f.write("#ifndef MODEL_CAR_H\n")
        f.write("#define MODEL_CAR_H\n\n")
        f.write('#include <stdint.h>\n')
        f.write('#include "esr/esr_pipeline.hpp"\n')
        f.write('#include "esr/esr_mesh.hpp"\n')
        f.write('#include "esr/esr_shader.hpp"\n\n')

        # Livery texture
        if livery_inc_data:
            f.write("// --- 256x256 8-bit Indexed Texture Atlas & Palette ---\n")
            f.write(livery_inc_data)
            f.write("\n")

        # VBO
        f.write("static const esr::Vertex3D CAR_VERTICES[] = {\n")
        for i, (pos, col) in enumerate(mesh_vertices):
            comma = "," if i < len(mesh_vertices) - 1 else ""
            f.write(f"    {{{{{pos[0]: .6f}f, {pos[1]: .6f}f, {pos[2]: .6f}f}}, "
                    f"{{{col[0]:.6f}f, {col[1]:.6f}f, {col[2]:.6f}f}}}}{comma}\n")
        f.write("};\n\n")

        # IBO
        f.write("static const uint16_t CAR_INDICES[] = {\n")
        for i in range(0, len(mesh_indices), 3):
            comma = "," if i + 3 < len(mesh_indices) else ""
            f.write(f"    {mesh_indices[i]}, {mesh_indices[i+1]}, {mesh_indices[i+2]}{comma}\n")
        f.write("};\n\n")

        # Mesh3D
        f.write("static const esr::Mesh3D CAR_MESH = {\n")
        f.write("    CAR_VERTICES,\n")
        f.write("    sizeof(CAR_VERTICES) / sizeof(CAR_VERTICES[0]),\n")
        f.write("    CAR_INDICES,\n")
        f.write("    sizeof(CAR_INDICES) / sizeof(CAR_INDICES[0])\n")
        f.write("};\n\n")

        # Shaders for perspective-correct texture mapping
        f.write("""// Shaders for perspective-correct texture mapping
inline void car_vertex_shader(const void* in_vertex, esr::esr_vertex_out_t* out_vertex, const void* uniforms) {
    if (!in_vertex || !out_vertex) return;
    const auto* v = static_cast<const esr::Vertex3D*>(in_vertex);
    if (uniforms) {
        const auto* mvp = static_cast<const esr::Mat4*>(uniforms);
        out_vertex->clip_pos = esr::transform_mvp(*v, *mvp).position;
    } else {
        out_vertex->clip_pos = esr::vec4_from_vec3(v->position, 1.0f);
    }
    out_vertex->varyings.data[0] = v->color.x; // u
    out_vertex->varyings.data[1] = v->color.y; // v
    out_vertex->varyings.data[2] = v->color.z; // is_window
    out_vertex->varyings.count = 3;
}

inline uint16_t car_pixel_shader(const float* varyings, const void* uniforms) {
    (void)uniforms;
    if (!varyings) return 0;
    if (varyings[2] > 0.5f) {
        return 0x5DF7; // Cyan ice reflection for front & rear windows
    }
    float u = varyings[0];
    float v = varyings[1];
    int x = static_cast<int>(u * 255.0f + 0.5f) & 255;
    int y = static_cast<int>(v * 255.0f + 0.5f) & 255;
    uint8_t pal_idx = CAR_LIVERY_INDICES[(y << 8) | x];
    return CAR_LIVERY_PALETTE[pal_idx];
}

#endif // MODEL_CAR_H
""")

    print(f"Successfully wrote {out_path}!")

if __name__ == '__main__':
    main()
