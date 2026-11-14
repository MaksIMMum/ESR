#!/usr/bin/env python3
"""
obj2header.py - Convert Wavefront .obj files to C/C++ static header files
for the ESR (Embedded Software Rasterizer) indexed mesh pipeline.

Usage:
    python3 tools/obj2header.py assets/cube.obj -o include/models/cube.h --name cube

Features:
    - Subtracts 1 from .obj indices (1-based -> 0-based)
    - Automatically triangulates quad faces (f 1 2 3 4 -> (1,2,3) and (1,3,4))
    - Generates vertex colors based on normalized vertex positions
    - Outputs a self-contained C++ header with Mesh3D, VBO, and IBO
"""

import argparse
import os
import sys
import math


def parse_obj(filepath):
    """
    Parse a Wavefront .obj file.

    Returns:
        vertices: list of (x, y, z) tuples
        faces: list of lists of 0-based vertex indices (triangulated)
    """
    vertices = []
    faces = []

    with open(filepath, 'r') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue

            parts = line.split()
            keyword = parts[0]

            if keyword == 'v':
                if len(parts) < 4:
                    print(f"WARNING: line {line_num}: vertex needs 3 coordinates, got {len(parts)-1}", file=sys.stderr)
                    continue
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                vertices.append((x, y, z))

            elif keyword == 'f':
                # Parse face indices. .obj supports formats like:
                #   f 1 2 3
                #   f 1/1 2/2 3/3
                #   f 1/1/1 2/2/2 3/3/3
                #   f 1//1 2//2 3//3
                face_indices = []
                for token in parts[1:]:
                    # Take only the vertex index (before first '/')
                    idx_str = token.split('/')[0]
                    idx = int(idx_str) - 1  # Convert 1-based to 0-based
                    if idx < 0:
                        print(f"WARNING: line {line_num}: negative index after conversion: {idx}", file=sys.stderr)
                    face_indices.append(idx)

                if len(face_indices) < 3:
                    print(f"WARNING: line {line_num}: face needs at least 3 vertices, got {len(face_indices)}", file=sys.stderr)
                    continue

                # Fan triangulation for polygons with 3+ vertices
                for i in range(1, len(face_indices) - 1):
                    faces.append([face_indices[0], face_indices[i], face_indices[i + 1]])

    return vertices, faces


def compute_vertex_colors(vertices):
    """
    Generate vertex colors based on normalized vertex positions.
    Maps position components to RGB in [0.0, 1.0] range.
    """
    if not vertices:
        return []

    # Find bounding box
    xs = [v[0] for v in vertices]
    ys = [v[1] for v in vertices]
    zs = [v[2] for v in vertices]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_z, max_z = min(zs), max(zs)

    range_x = max_x - min_x if max_x != min_x else 1.0
    range_y = max_y - min_y if max_y != min_y else 1.0
    range_z = max_z - min_z if max_z != min_z else 1.0

    colors = []
    for (x, y, z) in vertices:
        r = (x - min_x) / range_x
        g = (y - min_y) / range_y
        b = (z - min_z) / range_z
        # Clamp to ensure valid range
        r = max(0.0, min(1.0, r))
        g = max(0.0, min(1.0, g))
        b = max(0.0, min(1.0, b))
        # Ensure minimum brightness for visibility
        brightness = (r + g + b) / 3.0
        if brightness < 0.15:
            r = max(r, 0.2)
            g = max(g, 0.2)
            b = max(b, 0.2)
        colors.append((r, g, b))

    return colors


def generate_header(name, vertices, faces, colors):
    """
    Generate C/C++ header file content with VBO, IBO, and Mesh3D.
    """
    guard = f"MODEL_{name.upper()}_H"
    prefix = name.upper()

    lines = []
    lines.append(f"#ifndef {guard}")
    lines.append(f"#define {guard}")
    lines.append("")
    lines.append('#include "esr/esr_pipeline.hpp"')
    lines.append('#include "esr/esr_mesh.hpp"')
    lines.append("")

    # Vertex Buffer Object (VBO)
    lines.append(f"static const esr::Vertex3D {prefix}_VERTICES[] = {{")
    for i, ((x, y, z), (r, g, b)) in enumerate(zip(vertices, colors)):
        comma = "," if i < len(vertices) - 1 else ""
        lines.append(f"    {{{{{x: .6f}f, {y: .6f}f, {z: .6f}f}}, {{{r:.6f}f, {g:.6f}f, {b:.6f}f}}}}{comma}")
    lines.append("};")
    lines.append("")

    # Index Buffer Object (IBO)
    lines.append(f"static const uint16_t {prefix}_INDICES[] = {{")
    idx_strings = []
    for face in faces:
        idx_strings.append(f"    {face[0]}, {face[1]}, {face[2]}")
    lines.append(",\n".join(idx_strings))
    lines.append("};")
    lines.append("")

    # Mesh3D struct
    lines.append(f"static const esr::Mesh3D {prefix}_MESH = {{")
    lines.append(f"    {prefix}_VERTICES,")
    lines.append(f"    sizeof({prefix}_VERTICES) / sizeof({prefix}_VERTICES[0]),")
    lines.append(f"    {prefix}_INDICES,")
    lines.append(f"    sizeof({prefix}_INDICES) / sizeof({prefix}_INDICES[0])")
    lines.append("};")
    lines.append("")
    lines.append(f"#endif // {guard}")
    lines.append("")

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description="Convert Wavefront .obj files to C/C++ ESR mesh headers."
    )
    parser.add_argument("input", help="Path to .obj file")
    parser.add_argument("-o", "--output", required=True,
                        help="Output header file path")
    parser.add_argument("--name", required=True,
                        help="Model name (used for C identifiers, e.g. 'cube')")

    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f"ERROR: Input file not found: {args.input}", file=sys.stderr)
        sys.exit(1)

    print(f"Parsing: {args.input}")
    vertices, faces = parse_obj(args.input)

    if not vertices:
        print("ERROR: No vertices found in .obj file.", file=sys.stderr)
        sys.exit(1)
    if not faces:
        print("ERROR: No faces found in .obj file.", file=sys.stderr)
        sys.exit(1)

    # Validate indices
    max_idx = max(max(f) for f in faces)
    if max_idx >= len(vertices):
        print(f"ERROR: Index {max_idx} out of bounds (only {len(vertices)} vertices).", file=sys.stderr)
        sys.exit(1)

    # Check uint16_t range
    if len(vertices) > 65535:
        print(f"ERROR: Too many vertices ({len(vertices)}) for uint16_t indices.", file=sys.stderr)
        sys.exit(1)

    colors = compute_vertex_colors(vertices)
    header = generate_header(args.name, vertices, faces, colors)

    # Create output directory if needed
    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    with open(args.output, 'w', newline='\n') as f:
        f.write(header)

    num_triangles = len(faces)
    num_indices = num_triangles * 3
    print(f"Done: {len(vertices)} vertices, {num_triangles} triangles ({num_indices} indices)")
    print(f"Output: {args.output}")


if __name__ == "__main__":
    main()
