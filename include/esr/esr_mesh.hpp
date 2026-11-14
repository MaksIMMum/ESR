#ifndef ESR_MESH_HPP
#define ESR_MESH_HPP

#include <stdint.h>
#include "esr/esr_pipeline.hpp"
#include "esr/esr_raster.hpp"

namespace esr {

/* ------------------------------------------------------------------ */
/* Indexed Mesh Data Structure                                         */
/* ------------------------------------------------------------------ */

/**
 * Static indexed 3D mesh stored entirely in Flash/RODATA.
 *
 * Points to const vertex and index arrays with zero dynamic heap allocation.
 * Index buffer stores triangle triplets (every 3 consecutive indices form
 * one triangle). Suitable for VBO/IBO-style rendering on ESP32-S3 and desktop.
 */
struct Mesh3D {
    const Vertex3D* vertices;   // Pointer to Flash (RODATA) vertex array
    uint32_t vertex_count;      // Total number of vertices
    const uint16_t* indices;    // Pointer to Flash (RODATA) index array (triplets)
    uint32_t index_count;       // Total number of indices (must be multiple of 3)
};

/* ------------------------------------------------------------------ */
/* Mesh Rendering Functions                                            */
/* ------------------------------------------------------------------ */

/**
 * Render all triangles of an indexed mesh through the full pipeline.
 *
 * Iterates through the index buffer in triplets, transforms each triangle
 * via the MVP matrix, clips against the near plane, and rasterizes into
 * the framebuffer.
 *
 * @param mesh   Indexed mesh (vertices + indices in Flash/RODATA).
 * @param mvp    Model-View-Projection matrix.
 * @param vp     Viewport configuration.
 * @param fb     Target framebuffer for rasterization.
 * @return Total number of pixels written across all triangles.
 */
int draw_mesh(const Mesh3D& mesh,
              const Mat4& mvp,
              const Viewport& vp,
              Framebuffer& fb);

} // namespace esr

#endif // ESR_MESH_HPP
