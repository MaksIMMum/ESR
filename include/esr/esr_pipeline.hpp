#ifndef ESR_PIPELINE_HPP
#define ESR_PIPELINE_HPP

#include "esr/esr_math.h"

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
    float x;      // Framebuffer pixel X coordinate [0.0, width]
    float y;      // Framebuffer pixel Y coordinate [0.0, height]
    float z;      // Normalized depth [0.0, 1.0] for Z-buffer
    float inv_w;  // 1.0f / w, required for perspective-correct interpolation
    Vec3 color;   // Vertex color attribute
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
/* Pipeline Transformation Functions                                   */
/* ------------------------------------------------------------------ */

/**
 * Transform an object-space 3D vertex to 4D homogeneous Clip Space via MVP matrix.
 */
ClipVertex transform_mvp(const Vertex3D& vertex, const Mat4& mvp);

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
ScreenVertex viewport_transform(const Vec3& ndc, float inv_w, const Vec3& color, const Viewport& vp);

/**
 * Convenience function: process a 3D triangle through MVP -> Perspective Divide -> Viewport.
 *
 * @return true if all 3 vertices successfully transformed, false if any vertex failed division.
 */
bool process_triangle(const Vertex3D& v0,
                      const Vertex3D& v1,
                      const Vertex3D& v2,
                      const Mat4& mvp,
                      const Viewport& vp,
                      ScreenTriangle& out_triangle);

} // namespace esr

#endif // ESR_PIPELINE_HPP
