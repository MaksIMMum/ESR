#ifndef ESR_SHADER_HPP
#define ESR_SHADER_HPP

#include <stdint.h>
#include "esr/esr_math.h"

namespace esr {

/** Maximum number of floating-point varyings passed from vertex to pixel shader. */
constexpr uint8_t ESR_MAX_VARYINGS = 8;

/**
 * Varyings storage passed between vertex and pixel shaders.
 */
struct Varyings {
    float data[ESR_MAX_VARYINGS] = {0.0f};
    uint8_t count = 0;
};

using esr_varyings_t = Varyings;

/**
 * Output of the Vertex Shader.
 */
struct VertexOut {
    Vec4 clip_pos;          // Homogeneous clip-space coordinates [x, y, z, w]
    Varyings varyings;      // Varyings to be interpolated across the triangle
};

using esr_vertex_out_t = VertexOut;

/**
 * Vertex Shader callback signature.
 *
 * @param in_vertex   Pointer to input vertex (e.g. Vertex3D or custom user struct).
 * @param out_vertex  Pointer to output struct to receive clip_pos and varyings.
 * @param uniforms    Pointer to user-defined uniform data (matrices, lights, etc.).
 */
typedef void (*esr_vertex_shader_t)(const void* in_vertex,
                                    esr_vertex_out_t* out_vertex,
                                    const void* uniforms);

using VertexShader = esr_vertex_shader_t;

/**
 * Pixel (Fragment) Shader callback signature.
 *
 * @param varyings  Pointer to interpolated varyings for the current fragment/pixel.
 * @param uniforms  Pointer to user-defined uniform data.
 * @return 16-bit RGB565 packed color.
 */
typedef uint16_t (*esr_pixel_shader_t)(const float* varyings,
                                       const void* uniforms);

using PixelShader = esr_pixel_shader_t;

/* ------------------------------------------------------------------ */
/* RGB565 Packing & Unpacking Helpers                                 */
/* ------------------------------------------------------------------ */

/**
 * Pack normalized float RGB [0.0, 1.0] into 16-bit RGB565.
 */
inline uint16_t esr_pack_rgb565(float r, float g, float b) {
    int ir = static_cast<int>(r * 31.0f + 0.5f);
    int ig = static_cast<int>(g * 63.0f + 0.5f);
    int ib = static_cast<int>(b * 31.0f + 0.5f);
    if (ir < 0) ir = 0; else if (ir > 31) ir = 31;
    if (ig < 0) ig = 0; else if (ig > 63) ig = 63;
    if (ib < 0) ib = 0; else if (ib > 31) ib = 31;
    return static_cast<uint16_t>((ir << 11) | (ig << 5) | ib);
}

/**
 * Pack 8-bit integer RGB [0, 255] into 16-bit RGB565.
 */
inline uint16_t esr_pack_rgb565_u8(uint8_t r, uint8_t g, uint8_t b) {
    return static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

/** Extract 8-bit Red component from RGB565. */
inline uint8_t esr_rgb565_r(uint16_t c) {
    return static_cast<uint8_t>(((c >> 11) & 0x1F) * 255 / 31);
}

/** Extract 8-bit Green component from RGB565. */
inline uint8_t esr_rgb565_g(uint16_t c) {
    return static_cast<uint8_t>(((c >> 5) & 0x3F) * 255 / 63);
}

/** Extract 8-bit Blue component from RGB565. */
inline uint8_t esr_rgb565_b(uint16_t c) {
    return static_cast<uint8_t>((c & 0x1F) * 255 / 31);
}

/** Convert RGB565 to Vec3 with components in [0.0, 1.0]. */
inline Vec3 esr_rgb565_to_vec3(uint16_t c) {
    return Vec3{
        ((c >> 11) & 0x1F) / 31.0f,
        ((c >> 5) & 0x3F) / 63.0f,
        (c & 0x1F) / 31.0f
    };
}

/* ------------------------------------------------------------------ */
/* Default Built-in Shaders                                           */
/* ------------------------------------------------------------------ */

/**
 * Default Vertex Shader:
 * Transforms Vertex3D::pos by Mat4 MVP matrix (passed via uniforms).
 * Passes Vertex3D::color as 3 float varyings (R, G, B in [0.0, 1.0]).
 */
void default_vertex_shader(const void* in_vertex,
                           esr_vertex_out_t* out_vertex,
                           const void* uniforms);

/**
 * Default Pixel Shader:
 * Reads first 3 varyings as RGB and packs into RGB565.
 */
uint16_t default_pixel_shader(const float* varyings,
                              const void* uniforms);

} // namespace esr

#endif // ESR_SHADER_HPP
