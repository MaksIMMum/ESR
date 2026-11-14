#ifndef ESR_DEPTH_HPP
#define ESR_DEPTH_HPP

#include <stdint.h>
#include <string.h>

namespace esr {

/**
 * Depth comparison test functions.
 */
enum class DepthFunc : uint8_t {
    Never,
    Less,       // Fragment passes if frag_z < current_z (default OpenGL / 3D standard)
    LEqual,     // Fragment passes if frag_z <= current_z
    Greater,    // Fragment passes if frag_z > current_z
    GEqual,     // Fragment passes if frag_z >= current_z
    Equal,      // Fragment passes if frag_z == current_z
    NotEqual,   // Fragment passes if frag_z != current_z
    Always      // Fragment always passes
};

/**
 * Non-owning view of a 16-bit depth buffer.
 *
 * Layout: row-major, 1 uint16_t per pixel (0 to 65535), row pitch = width * sizeof(uint16_t).
 * Memory footprint for 160x120 is 38.4 KB, fitting effortlessly in ESP32-S3 internal SRAM.
 */
struct DepthBuffer {
    uint16_t* data = nullptr;
    int width = 0;
    int height = 0;
};

/**
 * Clear the depth buffer. Default value is 0xFFFF (65535), representing
 * maximum depth / infinity / far plane.
 * Uses memset for hardware-accelerated block filling when clear_val is 0xFFFF or 0x0000.
 */
void clear_depth(DepthBuffer& db, uint16_t clear_val = 0xFFFF);

/**
 * Convert normalized depth z in [0.0f, 1.0f] to 16-bit unsigned integer [0, 65535].
 * Clamps out-of-range values.
 */
inline uint16_t to_depth16(float z) {
    if (z <= 0.0f) return 0;
    if (z >= 1.0f) return 65535;
    return static_cast<uint16_t>(z * 65535.0f + 0.5f);
}

/**
 * Evaluate depth test comparison.
 * @param frag_z  Incoming fragment depth.
 * @param curr_z  Current depth value in the depth buffer.
 * @param func    Depth comparison function.
 * @return true if fragment passes depth test.
 */
inline bool eval_depth_test(uint16_t frag_z, uint16_t curr_z, DepthFunc func) {
    switch (func) {
        case DepthFunc::Never:    return false;
        case DepthFunc::Less:     return frag_z < curr_z;
        case DepthFunc::LEqual:   return frag_z <= curr_z;
        case DepthFunc::Greater:  return frag_z > curr_z;
        case DepthFunc::GEqual:   return frag_z >= curr_z;
        case DepthFunc::Equal:    return frag_z == curr_z;
        case DepthFunc::NotEqual: return frag_z != curr_z;
        case DepthFunc::Always:   return true;
    }
    return false;
}

} // namespace esr

#endif // ESR_DEPTH_HPP
