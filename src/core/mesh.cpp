#include "esr/esr_mesh.hpp"
#include "esr/esr_profiler.hpp"

namespace esr {

int draw_mesh(const Mesh3D& mesh,
              const Mat4& mvp,
              const Viewport& vp,
              Framebuffer& fb,
              DepthBuffer* db,
              const PipelineState& state,
              Profiler* profiler) {
    int total_pixels = 0;
    const void* uniforms = state.uniforms ? state.uniforms : &mvp;

    // Iterate through index buffer in triplets (each triplet = one triangle)
    for (uint32_t i = 0; i + 2 < mesh.index_count; i += 3) {
        const Vertex3D& v0 = mesh.vertices[mesh.indices[i + 0]];
        const Vertex3D& v1 = mesh.vertices[mesh.indices[i + 1]];
        const Vertex3D& v2 = mesh.vertices[mesh.indices[i + 2]];

        // Full geometry pipeline: VertexShader/MVP -> clip -> perspective divide -> viewport -> culling
        ScreenTriangle out_triangles[ESR_MAX_CLIPPED_TRIANGLES];
        int tri_count = 0;

        if (profiler) profiler->begin_stage(ProfileStage::Geometry);
        if (state.vertex_shader) {
            tri_count = process_triangle(&v0, &v1, &v2, state.vertex_shader, uniforms,
                                         vp, out_triangles, state.cull_mode, state.front_face);
        } else {
            tri_count = process_triangle(v0, v1, v2, mvp, vp, out_triangles,
                                         state.cull_mode, state.front_face);
        }
        if (profiler) profiler->end_stage(ProfileStage::Geometry);

        // Rasterize each produced screen triangle (fill or wireframe, with optional pixel shader)
        if (profiler) profiler->begin_stage(ProfileStage::Raster);
        for (int t = 0; t < tri_count; ++t) {
            total_pixels += rasterize_triangle(out_triangles[t], fb, db,
                                               state.depth_func, state.polygon_mode,
                                               state.pixel_shader, uniforms);
        }
        if (profiler) profiler->end_stage(ProfileStage::Raster);
    }

    return total_pixels;
}

} // namespace esr
