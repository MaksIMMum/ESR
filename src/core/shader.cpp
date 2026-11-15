#include "esr/esr_shader.hpp"
#include "esr/esr_pipeline.hpp"

namespace esr {

void default_vertex_shader(const void* in_vertex,
                           esr_vertex_out_t* out_vertex,
                           const void* uniforms) {
    if (!in_vertex || !out_vertex) return;

    const auto* v = static_cast<const Vertex3D*>(in_vertex);

    if (uniforms) {
        const auto* mvp = static_cast<const Mat4*>(uniforms);
        out_vertex->clip_pos = transform_mvp(*v, *mvp).position;
    } else {
        out_vertex->clip_pos = vec4_from_vec3(v->position, 1.0f);
    }

    out_vertex->varyings.data[0] = v->color.x;
    out_vertex->varyings.data[1] = v->color.y;
    out_vertex->varyings.data[2] = v->color.z;
    out_vertex->varyings.count = 3;
}

uint16_t default_pixel_shader(const float* varyings,
                              const void* uniforms) {
    (void)uniforms;
    if (!varyings) return 0;
    return esr_pack_rgb565(varyings[0], varyings[1], varyings[2]);
}

} // namespace esr
