#include "esr/esr_pipeline.hpp"
#include <cmath>

namespace esr {

ClipVertex transform_mvp(const Vertex3D& vertex, const Mat4& mvp) {
    const Vec4 pos4 = vec4_from_vec3(vertex.position, 1.0f);
    ClipVertex out;
    out.position = mat4_mul_vec4(mvp, pos4);
    out.color = vertex.color;
    return out;
}

bool perspective_divide(const ClipVertex& clip, Vec3& out_ndc, float& out_inv_w) {
    constexpr float EPSILON = 1e-7f;
    if (std::fabs(clip.position.w) < EPSILON) {
        return false;
    }

    out_inv_w = 1.0f / clip.position.w;
    out_ndc.x = clip.position.x * out_inv_w;
    out_ndc.y = clip.position.y * out_inv_w;
    out_ndc.z = clip.position.z * out_inv_w;
    return true;
}

ScreenVertex viewport_transform(const Vec3& ndc, float inv_w, const Vec3& color, const Viewport& vp) {
    ScreenVertex out;

    // Map NDC X [-1, 1] -> [vp.x, vp.x + vp.width]
    out.x = vp.x + (ndc.x + 1.0f) * 0.5f * vp.width;

    // Map NDC Y [-1, 1] -> [vp.y, vp.y + vp.height]
    // Invert Y so that NDC +1 (top) maps to screen row vp.y (0)
    out.y = vp.y + (1.0f - ndc.y) * 0.5f * vp.height;

    // Map NDC Z [-1, 1] -> [vp.min_depth, vp.max_depth]
    out.z = vp.min_depth + (ndc.z + 1.0f) * 0.5f * (vp.max_depth - vp.min_depth);

    out.inv_w = inv_w;
    out.color = color;
    return out;
}

bool process_triangle(const Vertex3D& v0,
                      const Vertex3D& v1,
                      const Vertex3D& v2,
                      const Mat4& mvp,
                      const Viewport& vp,
                      ScreenTriangle& out_triangle) {
    const Vertex3D in_vertices[3] = {v0, v1, v2};

    for (int i = 0; i < 3; ++i) {
        // 1. MVP transformation (Object Space -> Clip Space)
        ClipVertex clip = transform_mvp(in_vertices[i], mvp);

        // 2. Perspective division (Clip Space -> NDC)
        Vec3 ndc;
        float inv_w = 1.0f;
        if (!perspective_divide(clip, ndc, inv_w)) {
            return false; // Degenerate vertex (w ~ 0)
        }

        // 3. Viewport transformation (NDC -> Framebuffer Pixels)
        out_triangle.v[i] = viewport_transform(ndc, inv_w, clip.color, vp);
    }

    return true;
}

} // namespace esr
