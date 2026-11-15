#ifndef ESR_PIPELINE_HPP
#define ESR_PIPELINE_HPP

#include "esr/esr_math.h"
#include "esr/esr_depth.hpp"
#include "esr/esr_shader.hpp"

namespace esr {

/* ------------------------------------------------------------------ */
/* Pipeline Vertex & Triangle Types                                    */
/* ------------------------------------------------------------------ */

/**
 * Input 3D vertex in object/world space.
 */
struct Vertex3D {
    Vec3 position;
    Vec3 color; // RGB float [0.0f, 1.0f]
};

/**
 * Vertex in homogeneous Clip Space after MVP transformation.
 * Coordinates are [x, y, z, w].
 */
struct ClipVertex {
    Vec4 position;
    Vec3 color;
    float varyings[ESR_MAX_VARYINGS] = {0.0f};
    uint8_t varying_count = 0;
};

/**
 * Triangle in Clip Space consisting of 3 ClipVertices.
 */
struct ClipTriangle {
    ClipVertex v[3];
};

/**
 * Vertex transformed into screen / viewport coordinates.
 */
struct ScreenVertex {
    float x = 0.0f;     // Framebuffer pixel X coordinate [0.0, width]
    float y = 0.0f;     // Framebuffer pixel Y coordinate [0.0, height]
    float z = 0.0f;     // Normalized depth [0.0, 1.0] for Z-buffer
    float inv_w = 0.0f; // 1.0f / w, required for perspective-correct interpolation
    Vec3 color = {0.0f, 0.0f, 0.0f};  // Vertex color attribute
    float varyings[ESR_MAX_VARYINGS] = {0.0f};
    uint8_t varying_count = 0;
};

/**
 * Triangle in Screen Space ready for 2D rasterization.
 */
struct ScreenTriangle {
    ScreenVertex v[3];
};

/**
 * Viewport configuration mapping NDC [-1, 1] to framebuffer pixels.
 */
struct Viewport {
    float x = 0.0f;
    float y = 0.0f;
    float width = 0.0f;
    float height = 0.0f;
    float min_depth = 0.0f;
    float max_depth = 1.0f;
};

/* ------------------------------------------------------------------ */
/* Pipeline States & Modes (REQ-5)                                     */
/* ------------------------------------------------------------------ */

/**
 * Triangle face culling modes.
 */
enum class CullMode : uint8_t {
    None = 0,   // No triangles culled (render both front and back faces)
    Back = 1,   // Cull back-facing triangles (standard optimization)
    Front = 2,  // Cull front-facing triangles
    CW = 3,     // Cull clockwise screen triangles
    CCW = 4     // Cull counter-clockwise screen triangles
};

constexpr CullMode CULL_NONE  = CullMode::None;
constexpr CullMode CULL_BACK  = CullMode::Back;
constexpr CullMode CULL_FRONT = CullMode::Front;
constexpr CullMode CULL_CW    = CullMode::CW;
constexpr CullMode CULL_CCW   = CullMode::CCW;

/**
 * Vertex winding order convention for determining front-facing polygons.
 */
enum class WindingOrder : uint8_t {
    CCW = 0,    // Counter-Clockwise is front-facing (standard OpenGL default)
    CW = 1      // Clockwise is front-facing
};

constexpr WindingOrder WINDING_CCW = WindingOrder::CCW;
constexpr WindingOrder WINDING_CW  = WindingOrder::CW;

/**
 * Polygon rasterization mode.
 */
enum class PolygonMode : uint8_t {
    Fill = 0,       // Solid fill (AABB + Pineda)
    Wireframe = 1   // Edge outlines only (Bresenham line rasterizer)
};

constexpr PolygonMode POLYGON_MODE_FILL      = PolygonMode::Fill;
constexpr PolygonMode POLYGON_MODE_WIREFRAME = PolygonMode::Wireframe;

/**
 * Global pipeline configuration state.
 */
struct PipelineState {
    CullMode cull_mode = CullMode::None;
    WindingOrder front_face = WindingOrder::CCW;
    PolygonMode polygon_mode = PolygonMode::Fill;
    DepthFunc depth_func = DepthFunc::Less;

    // --- Shader Callback Pipeline (REQ-1) ---
    VertexShader vertex_shader = nullptr; // nullptr = default MVP transformation
    PixelShader  pixel_shader  = nullptr; // nullptr = default fast Gouraud fill
    const void*  uniforms      = nullptr; // Uniform buffer binding pointer
};

/**
 * Bind uniform buffer data to pipeline state.
 */
inline void esr_set_uniforms(PipelineState& state, const void* uniforms) {
    state.uniforms = uniforms;
}

/**
 * Compute the 2D screen-space signed area (cross product of edges v0->v1 and v0->v2).
 *
 * In screen coordinates (where Y points down):
 *   signed_area < 0 -> Counter-Clockwise (CCW)
 *   signed_area > 0 -> Clockwise (CW)
 *   signed_area == 0 -> Degenerate / Collinear (edge-on)
 */
inline float screen_triangle_signed_area(const ScreenVertex& v0,
                                         const ScreenVertex& v1,
                                         const ScreenVertex& v2) {
    return (v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x);
}

/**
 * Check whether a triangle should be culled according to culling mode and winding convention.
 */
inline bool is_triangle_culled(float signed_area,
                               CullMode cull_mode,
                               WindingOrder front_face = WindingOrder::CCW) {
    if (cull_mode == CullMode::None) {
        return false;
    }
    if (signed_area == 0.0f) {
        return true;
    }
    const bool is_ccw = (signed_area < 0.0f);
    if (cull_mode == CullMode::CCW) {
        return is_ccw;
    }
    if (cull_mode == CullMode::CW) {
        return !is_ccw;
    }
    const bool is_front = (front_face == WindingOrder::CCW) ? is_ccw : !is_ccw;
    if (cull_mode == CullMode::Back) {
        return !is_front;
    }
    if (cull_mode == CullMode::Front) {
        return is_front;
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* Pipeline Transformation Functions                                   */
/* ------------------------------------------------------------------ */

/**
 * Transform an object-space 3D vertex to 4D homogeneous Clip Space via MVP matrix.
 */
ClipVertex transform_mvp(const Vertex3D& vertex, const Mat4& mvp);

/**
 * Maximum number of triangles clip_triangle_near() can produce from one input triangle.
 */
constexpr int ESR_MAX_CLIPPED_TRIANGLES = 2;

/**
 * Clip a clip-space triangle against the near plane (z >= -w).
 *
 * Must run BEFORE perspective division. Vertices behind the camera have w <= 0,
 * and dividing by such w either explodes (w ~ 0) or mirrors the vertex to the
 * other side of the screen (w < 0). Clipping at z = -w guarantees w >= near > 0
 * for every output vertex when a mat4_perspective() projection is used.
 *
 * Cases (number of vertices in front of the near plane):
 *   3 -> 1 triangle (input copied unchanged)
 *   2 -> 2 triangles (quad, split as a fan)
 *   1 -> 1 smaller triangle
 *   0 -> 0 triangles (fully rejected)
 *
 * Positions and all vertex attributes (color) are linearly interpolated in
 * clip space at the intersection points, which is correct before the divide.
 * Winding order is preserved, so backface culling still works afterwards.
 *
 * @param in  Input triangle in clip space.
 * @param out Output array with room for ESR_MAX_CLIPPED_TRIANGLES triangles.
 * @return Number of triangles written to `out` (0, 1 or 2).
 */
int clip_triangle_near(const ClipTriangle& in, ClipTriangle out[ESR_MAX_CLIPPED_TRIANGLES]);

/**
 * Perform perspective division: Clip Space -> Normalized Device Coordinates (NDC).
 *
 * @param clip      Input clip vertex.
 * @param out_ndc   Output coordinates in NDC range [-1.0, 1.0].
 * @param out_inv_w Output reciprocal of w (1.0f / w).
 * @return false if |w| is close to 0 (degenerate / division by zero), true otherwise.
 */
bool perspective_divide(const ClipVertex& clip, Vec3& out_ndc, float& out_inv_w);

/**
 * Map Normalized Device Coordinates (NDC) to 2D screen/framebuffer pixel space.
 * Inverts the Y-axis so NDC +1 (top) maps to screen row 0.
 */
ScreenVertex viewport_transform(const Vec3& ndc, float inv_w, const Vec3& color, const Viewport& vp,
                                const float* varyings = nullptr, uint8_t varying_count = 0);

/**
 * Full geometry stage for one triangle:
 *   MVP -> Near-plane clipping -> Perspective Divide -> Viewport -> Culling.
 *
 * @param out_triangles Output array with room for ESR_MAX_CLIPPED_TRIANGLES triangles.
 * @return Number of screen triangles written (0 if the triangle is fully behind
 *         the near plane, degenerate, or culled, otherwise 1 or 2).
 */
int process_triangle(const Vertex3D& v0,
                     const Vertex3D& v1,
                     const Vertex3D& v2,
                     const Mat4& mvp,
                     const Viewport& vp,
                     ScreenTriangle out_triangles[ESR_MAX_CLIPPED_TRIANGLES],
                     CullMode cull_mode = CullMode::None,
                     WindingOrder front_face = WindingOrder::CCW);

/**
 * Programmable geometry stage for one triangle using a Vertex Shader callback:
 *   VertexShader -> Near-plane clipping -> Perspective Divide -> Viewport -> Culling.
 */
int process_triangle(const void* v0,
                     const void* v1,
                     const void* v2,
                     VertexShader vs,
                     const void* uniforms,
                     const Viewport& vp,
                     ScreenTriangle out_triangles[ESR_MAX_CLIPPED_TRIANGLES],
                     CullMode cull_mode = CullMode::None,
                     WindingOrder front_face = WindingOrder::CCW);

} // namespace esr

#endif // ESR_PIPELINE_HPP
