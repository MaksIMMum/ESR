#ifndef ESR_RASTER_HPP
#define ESR_RASTER_HPP

#include <stdint.h>

#include "esr/esr_pipeline.hpp"
#include "esr/esr_depth.hpp"

namespace esr {

/* ------------------------------------------------------------------ */
/* Framebuffer                                                          */
/* ------------------------------------------------------------------ */

/**
 * Non-owning view of an RGB888 color buffer.
 * Layout: row-major, 3 bytes per pixel (R, G, B), row pitch = width * 3.
 * Matches the `uint8_t fb[H][W][3]` arrays used by the desktop and ESP32 targets.
 */
struct Framebuffer {
    uint8_t* color = nullptr;
    int width = 0;
    int height = 0;
};

/* ------------------------------------------------------------------ */
/* Triangle rasterizer                                                  */
/* ------------------------------------------------------------------ */

/**
 * Sub-pixel precision of the rasterizer: screen coordinates are snapped to
 * 1 / (1 << ESR_SUBPIXEL_BITS) of a pixel (28.4 fixed point) before edge setup.
 */
constexpr int ESR_SUBPIXEL_BITS = 4;

/**
 * Triangles with any vertex further than this many pixels from the origin
 * (or with NaN/Inf coordinates) are rejected. Keeps all edge-function math
 * inside 64-bit integer range. Ordinary geometry never comes close to it.
 */
constexpr float ESR_RASTER_GUARD_BAND = 4194304.0f; // 2^22 px

/**
 * Rasterize one screen-space triangle into the framebuffer with
 * linearly interpolated vertex colors and optional 16-bit depth testing.
 *
 * Algorithm:
 *  - Vertices are snapped to 28.4 fixed point.
 *  - Only pixels inside the triangle's Axis-Aligned Bounding Box, clamped to
 *    the framebuffer [0, width) x [0, height), are visited.
 *  - Coverage uses Pineda edge functions E(x, y) = A*x + B*y + C evaluated
 *    incrementally: one integer add per edge per pixel, no per-pixel division.
 *  - Top-Left fill rule: a pixel center that lies exactly on an edge is
 *    covered only if that edge is a top or left edge. Triangles sharing an
 *    edge therefore never leave gaps and never draw a pixel twice.
 *  - Colors and depth are stepped with per-triangle gradients (one reciprocal of the
 *    area per triangle).
 *  - Early-Z: if a DepthBuffer is provided, depth comparison is evaluated before
 *    pixel shading, immediately discarding hidden/occluded fragments.
 *
 * Both clockwise and counter-clockwise winding are accepted.
 * Degenerate (zero-area) triangles draw nothing.
 *
 * @param tri         Screen-space triangle.
 * @param fb          Target color framebuffer.
 * @param db          Optional target 16-bit depth buffer (nullptr to disable depth testing).
 * @param depth_func  Depth comparison function (defaults to DepthFunc::Less).
 * @return Number of pixels written.
 */
int rasterize_triangle(const ScreenTriangle& tri,
                       Framebuffer& fb,
                       DepthBuffer* db = nullptr,
                       DepthFunc depth_func = DepthFunc::Less);

} // namespace esr

#endif // ESR_RASTER_HPP
