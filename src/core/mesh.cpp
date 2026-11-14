#include "esr/esr_mesh.hpp"

namespace esr {

int draw_mesh(const Mesh3D& mesh,
              const Mat4& mvp,
              const Viewport& vp,
              Framebuffer& fb,
              DepthBuffer* db,
              DepthFunc depth_func) {
    int total_pixels = 0;

    // Iterate through index buffer in triplets (each triplet = one triangle)
    for (uint32_t i = 0; i + 2 < mesh.index_count; i += 3) {
        const Vertex3D& v0 = mesh.vertices[mesh.indices[i + 0]];
        const Vertex3D& v1 = mesh.vertices[mesh.indices[i + 1]];
        const Vertex3D& v2 = mesh.vertices[mesh.indices[i + 2]];

        // Full geometry pipeline: MVP -> clip -> perspective divide -> viewport
        ScreenTriangle out_triangles[ESR_MAX_CLIPPED_TRIANGLES];
        int tri_count = process_triangle(v0, v1, v2, mvp, vp, out_triangles);

        // Rasterize each produced screen triangle
        for (int t = 0; t < tri_count; ++t) {
            total_pixels += rasterize_triangle(out_triangles[t], fb, db, depth_func);
        }
    }

    return total_pixels;
}

} // namespace esr
