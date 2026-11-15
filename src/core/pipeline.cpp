#include "esr/esr_pipeline.hpp"
#include <cmath>

namespace esr {

ClipVertex transform_mvp(const Vertex3D& vertex, const Mat4& mvp) {
    const Vec4 pos4 = vec4_from_vec3(vertex.position, 1.0f);
    ClipVertex out;
    out.position = mat4_mul_vec4(mvp, pos4);
    out.color = vertex.color;
    out.varyings[0] = vertex.color.x;
    out.varyings[1] = vertex.color.y;
    out.varyings[2] = vertex.color.z;
    out.varying_count = 3;
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

/* ------------------------------------------------------------------ */
/* Near-plane clipping                                                  */
/* ------------------------------------------------------------------ */

/** Signed distance to the near plane in clip space: >= 0 means "in front" (visible side). */
static inline float near_plane_distance(const ClipVertex& v) {
    return v.position.z + v.position.w;
}

/** Linear interpolation of a clip vertex (position and every attribute). */
static inline ClipVertex lerp_clip_vertex(const ClipVertex& a, const ClipVertex& b, float t) {
    ClipVertex r;
    r.position.x = a.position.x + (b.position.x - a.position.x) * t;
    r.position.y = a.position.y + (b.position.y - a.position.y) * t;
    r.position.z = a.position.z + (b.position.z - a.position.z) * t;
    r.position.w = a.position.w + (b.position.w - a.position.w) * t;
    r.color.x = a.color.x + (b.color.x - a.color.x) * t;
    r.color.y = a.color.y + (b.color.y - a.color.y) * t;
    r.color.z = a.color.z + (b.color.z - a.color.z) * t;
    r.varying_count = a.varying_count;
    for (uint8_t i = 0; i < a.varying_count; ++i) {
        r.varyings[i] = a.varyings[i] + (b.varyings[i] - a.varyings[i]) * t;
    }
    return r;
}

int clip_triangle_near(const ClipTriangle& in, ClipTriangle out[ESR_MAX_CLIPPED_TRIANGLES]) {
    float d[3];
    int inside_count = 0;
    for (int i = 0; i < 3; ++i) {
        d[i] = near_plane_distance(in.v[i]);
        if (d[i] >= 0.0f) {
            ++inside_count;
        }
    }

    if (inside_count == 0) {
        return 0; // Fully behind the near plane: reject.
    }
    if (inside_count == 3) {
        out[0] = in; // Fully in front: trivial accept, no new vertices.
        return 1;
    }

    // Sutherland-Hodgman against a single plane. Walking the edges in order
    // keeps the original winding. One plane cuts a triangle into at most a quad.
    ClipVertex poly[4];
    int n = 0;
    for (int i = 0; i < 3; ++i) {
        const int j = (i + 1) % 3;
        const ClipVertex& a = in.v[i];
        const ClipVertex& b = in.v[j];
        const bool a_in = d[i] >= 0.0f;
        const bool b_in = d[j] >= 0.0f;

        if (a_in) {
            poly[n++] = a;
        }
        if (a_in != b_in) {
            // Edge crosses the plane: d(t) = d[i] + t * (d[j] - d[i]) = 0.
            // Signs differ, so the denominator is never zero.
            const float t = d[i] / (d[i] - d[j]);
            poly[n++] = lerp_clip_vertex(a, b, t);
        }
    }

    // Fan triangulation: 3 vertices -> 1 triangle, 4 vertices -> 2 triangles.
    int count = 0;
    for (int k = 1; k + 1 < n; ++k) {
        out[count].v[0] = poly[0];
        out[count].v[1] = poly[k];
        out[count].v[2] = poly[k + 1];
        ++count;
    }
    return count;
}

/* ------------------------------------------------------------------ */
/* Viewport                                                             */
/* ------------------------------------------------------------------ */

ScreenVertex viewport_transform(const Vec3& ndc, float inv_w, const Vec3& color, const Viewport& vp,
                                const float* varyings, uint8_t varying_count) {
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
    out.varying_count = varying_count;
    if (varyings && varying_count > 0) {
        for (uint8_t i = 0; i < varying_count; ++i) {
            out.varyings[i] = varyings[i];
        }
    }
    return out;
}

int process_triangle(const void* v0,
                     const void* v1,
                     const void* v2,
                     VertexShader vs,
                     const void* uniforms,
                     const Viewport& vp,
                     ScreenTriangle out_triangles[ESR_MAX_CLIPPED_TRIANGLES],
                     CullMode cull_mode,
                     WindingOrder front_face) {
    // 1. Programmable Vertex Shader transformation
    VertexOut vo[3];
    vs(v0, &vo[0], uniforms);
    vs(v1, &vo[1], uniforms);
    vs(v2, &vo[2], uniforms);

    ClipTriangle clip_tri;
    for (int i = 0; i < 3; ++i) {
        clip_tri.v[i].position = vo[i].clip_pos;
        clip_tri.v[i].varying_count = vo[i].varyings.count;
        for (uint8_t k = 0; k < vo[i].varyings.count; ++k) {
            clip_tri.v[i].varyings[k] = vo[i].varyings.data[k];
        }
        if (vo[i].varyings.count >= 3) {
            clip_tri.v[i].color = Vec3{vo[i].varyings.data[0], vo[i].varyings.data[1], vo[i].varyings.data[2]};
        } else {
            clip_tri.v[i].color = Vec3{1.0f, 1.0f, 1.0f};
        }
    }

    // 2. Near-plane clipping (Clip Space, before any division)
    ClipTriangle clipped[ESR_MAX_CLIPPED_TRIANGLES];
    const int clipped_count = clip_triangle_near(clip_tri, clipped);

    int out_count = 0;
    for (int t = 0; t < clipped_count; ++t) {
        ScreenTriangle screen_tri;
        bool ok = true;

        for (int i = 0; i < 3 && ok; ++i) {
            // 3. Perspective division (Clip Space -> NDC)
            Vec3 ndc;
            float inv_w = 1.0f;
            ok = perspective_divide(clipped[t].v[i], ndc, inv_w);

            // 4. Viewport transformation (NDC -> Framebuffer Pixels)
            if (ok) {
                screen_tri.v[i] = viewport_transform(ndc, inv_w, clipped[t].v[i].color, vp,
                                                     clipped[t].v[i].varyings, clipped[t].v[i].varying_count);
            }
        }

        if (ok) {
            // 5. Backface / Frontface culling (screen space 2D cross product)
            if (cull_mode != CullMode::None) {
                const float area = screen_triangle_signed_area(screen_tri.v[0], screen_tri.v[1], screen_tri.v[2]);
                if (is_triangle_culled(area, cull_mode, front_face)) {
                    continue; // Culled before rasterization!
                }
            }
            out_triangles[out_count++] = screen_tri;
        }
    }

    return out_count;
}

int process_triangle(const Vertex3D& v0,
                     const Vertex3D& v1,
                     const Vertex3D& v2,
                     const Mat4& mvp,
                     const Viewport& vp,
                     ScreenTriangle out_triangles[ESR_MAX_CLIPPED_TRIANGLES],
                     CullMode cull_mode,
                     WindingOrder front_face) {
    return process_triangle(&v0, &v1, &v2, default_vertex_shader, &mvp, vp,
                            out_triangles, cull_mode, front_face);
}

} // namespace esr
