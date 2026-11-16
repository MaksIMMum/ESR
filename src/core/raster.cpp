#include "esr/esr_raster.hpp"
#include "esr/esr_depth.hpp"

#include <cmath>
#include <cstdint>

namespace esr {

namespace {

constexpr int SUB_BITS = ESR_SUBPIXEL_BITS;
constexpr int32_t SUB_ONE = 1 << SUB_BITS;   // 1 pixel in fixed point
constexpr int32_t SUB_HALF = SUB_ONE >> 1;   // pixel center offset

/** Snap a pixel coordinate to 28.4 fixed point (round to nearest). */
inline int32_t to_fixed(float v) {
    return static_cast<int32_t>(std::floor(v * static_cast<float>(SUB_ONE) + 0.5f));
}

/** floor(v / SUB_ONE) for signed v, without relying on arithmetic right shift. */
inline int32_t floor_div_sub(int32_t v) {
    return v >= 0 ? (v >> SUB_BITS) : -((-v + SUB_ONE - 1) >> SUB_BITS);
}

/** ceil(v / SUB_ONE) for signed v. */
inline int32_t ceil_div_sub(int32_t v) {
    return -floor_div_sub(-v);
}

inline int32_t min3(int32_t a, int32_t b, int32_t c) {
    const int32_t m = a < b ? a : b;
    return m < c ? m : c;
}

inline int32_t max3(int32_t a, int32_t b, int32_t c) {
    const int32_t m = a > b ? a : b;
    return m > c ? m : c;
}

/** Colors are stepped as 8.16 fixed point: 255 << 16 is full intensity. */
constexpr int COLOR_FRAC_BITS = 16;
constexpr int32_t COLOR_MAX = 255 << COLOR_FRAC_BITS;
constexpr float COLOR_SCALE = 255.0f * (1 << COLOR_FRAC_BITS);

inline int32_t to_color_fixed(float v) {
    // Clamp before converting so wild gradients can never overflow int32.
    const float s = v * COLOR_SCALE;
    if (s > 1.0e9f) return 1000000000;
    if (s < -1.0e9f) return -1000000000;
    return static_cast<int32_t>(std::floor(s + 0.5f));
}

inline uint8_t to_u8(int32_t v) {
    if (v <= 0) return 0;
    if (v >= COLOR_MAX) return 255;
    return static_cast<uint8_t>((v + (1 << (COLOR_FRAC_BITS - 1))) >> COLOR_FRAC_BITS);
}

/**
 * 16-bit depth buffer values are in [0, 65535].
 * Stepped in 16.15 fixed point so depth values and gradients fit in signed int32_t.
 * 65535 * 32768 = 2,147,450,880 <= INT32_MAX (2,147,483,647).
 */
constexpr float DEPTH_SCALE = 65535.0f * 32768.0f;

inline uint16_t to_u16_depth(int32_t v) {
    if (v <= 0) return 0;
    if (v >= 2147450880) return 65535;
    return static_cast<uint16_t>((v + (1 << 14)) >> 15);
}

inline int32_t to_depth_fixed(float v) {
    const float s = v * DEPTH_SCALE;
    if (s > 2147450880.0f) return 2147450880;
    if (s < 0.0f) return 0;
    return static_cast<int32_t>(std::floor(s + 0.5f));
}

inline int32_t to_depth_gradient_fixed(float v) {
    const float s = v * DEPTH_SCALE;
    if (s > 1.0e9f) return 1000000000;
    if (s < -1.0e9f) return -1000000000;
    return static_cast<int32_t>(std::floor(s + 0.5f));
}

/**
 * One Pineda edge function E(p) = (p.x - a.x) * (b.y - a.y) - (p.y - a.y) * (b.x - a.x),
 * stored as its value at the first pixel center of the AABB plus per-pixel steps.
 */
struct Edge {
    int64_t step_x;   // E(x + 1, y) - E(x, y)
    int64_t step_y;   // E(x, y + 1) - E(x, y)
    int64_t raw;      // Unbiased E at the AABB origin pixel center (for interpolation)
    int64_t biased;   // raw + fill-rule bias (for the coverage test)
};

/**
 * Set up edge a -> b for a triangle with positive area (interior has E > 0).
 *
 * Screen Y points down. With positive area the vertices run counter-clockwise
 * on screen, so:
 *   left edge : b.y > a.y (edge goes down)
 *   top edge  : b.y == a.y and b.x < a.x (horizontal, goes left)
 * Pixels exactly on a top/left edge (E == 0) are covered, on any other edge
 * they are not. With integer E that is "E + bias >= 0", bias in {0, -1}.
 */
inline Edge make_edge(int32_t ax, int32_t ay, int32_t bx, int32_t by, int64_t px, int64_t py) {
    const int64_t A = static_cast<int64_t>(by) - ay;
    const int64_t B = static_cast<int64_t>(ax) - bx;

    Edge e;
    e.step_x = A * SUB_ONE;
    e.step_y = B * SUB_ONE;
    e.raw = (px - ax) * A + (py - ay) * B;

    const bool top_left = (A > 0) || (A == 0 && B > 0);
    e.biased = e.raw + (top_left ? 0 : -1);
    return e;
}

/** True if every value the inner loops will produce for this edge fits in int32. */
inline bool fits_int32(const Edge& e, int span_x, int span_y) {
    const int64_t c[4] = {
        e.biased,
        e.biased + e.step_x * span_x,
        e.biased + e.step_y * span_y,
        e.biased + e.step_x * span_x + e.step_y * span_y,
    };
    for (int64_t v : c) {
        if (v < INT32_MIN || v > INT32_MAX) {
            return false;
        }
    }
    return true;
}

struct ColorSetup {
    int32_t r, g, b;          // 8.16 color at the AABB origin pixel center
    int32_t drdx, dgdx, dbdx; // Per-pixel step in x
    int32_t drdy, dgdy, dbdy; // Per-pixel step in y
};

struct DepthSetup {
    DepthBuffer* db = nullptr;
    DepthFunc func = DepthFunc::Less;
    int32_t z = 0;          // 16.15 depth at the AABB origin pixel center
    int32_t dzdx = 0;       // Per-pixel depth step in x
    int32_t dzdy = 0;       // Per-pixel depth step in y
};

/**
 * Float attribute stepping state. For perspective-correct interpolation the
 * stepped values are A * (1/w) and `q` = 1/w; otherwise they are A directly.
 */
struct VaryingSetup {
    float start[ESR_MAX_VARYINGS] = {0.0f};
    float dvdx[ESR_MAX_VARYINGS]  = {0.0f};
    float dvdy[ESR_MAX_VARYINGS]  = {0.0f};
    uint8_t count = 0;
    float q = 1.0f;     // 1/w at the AABB origin pixel center (Perspective only)
    float dqdx = 0.0f;  // Per-pixel 1/w step in x
    float dqdy = 0.0f;  // Per-pixel 1/w step in y
};

/** Clamp a normalized float color channel to [0, 255]. */
inline uint8_t unit_to_u8(float v) {
    if (!(v > 0.0f)) return 0;   // Also maps NaN to 0.
    if (v >= 1.0f) return 255;
    return static_cast<uint8_t>(v * 255.0f + 0.5f);
}

/**
 * Walk the AABB with float attribute interpolation.
 *
 * Perspective : step A/w and 1/w, recover A = (A/w) * w per shaded pixel.
 * UseShader   : true  -> call pixel_shader(varyings), write RGB565 result;
 *               false -> varyings[0..2] are RGB in [0, 1], written directly.
 */
template <typename T, bool HasDepth, bool Perspective, bool UseShader>
int walk_shader(Framebuffer& fb, int minx, int miny, int maxx, int maxy,
                const Edge e[3], const VaryingSetup& v_setup, const DepthSetup& d,
                PixelShader pixel_shader, const void* uniforms) {
    const T sx0 = static_cast<T>(e[0].step_x);
    const T sx1 = static_cast<T>(e[1].step_x);
    const T sx2 = static_cast<T>(e[2].step_x);
    const T sy0 = static_cast<T>(e[0].step_y);
    const T sy1 = static_cast<T>(e[1].step_y);
    const T sy2 = static_cast<T>(e[2].step_y);

    T row0 = static_cast<T>(e[0].biased);
    T row1 = static_cast<T>(e[1].biased);
    T row2 = static_cast<T>(e[2].biased);
    int32_t row_z = d.z;

    float row_v[ESR_MAX_VARYINGS];
    for (uint8_t k = 0; k < v_setup.count; ++k) {
        row_v[k] = v_setup.start[k];
    }
    float row_q = v_setup.q;

    const int pitch = fb.width * 3;
    uint8_t* row_ptr = fb.color + (static_cast<long>(miny) * fb.width + minx) * 3;

    uint16_t* depth_row_ptr = nullptr;
    int depth_pitch = 0;
    if constexpr (HasDepth) {
        depth_pitch = d.db->width;
        depth_row_ptr = d.db->data + (static_cast<long>(miny) * d.db->width + minx);
    }

    int written = 0;

    for (int y = miny; y <= maxy; ++y) {
        T w0 = row0, w1 = row1, w2 = row2;
        int32_t z = row_z;
        uint8_t* p = row_ptr;
        uint16_t* dp = depth_row_ptr;
        bool entered = false;

        float cur_v[ESR_MAX_VARYINGS];
        for (uint8_t k = 0; k < v_setup.count; ++k) {
            cur_v[k] = row_v[k];
        }
        float q = row_q;

        for (int x = minx; x <= maxx; ++x) {
            if ((w0 | w1 | w2) >= 0) {
                entered = true;

                bool pass = true;
                if constexpr (HasDepth) {
                    const uint16_t frag_z = to_u16_depth(z);
                    if (eval_depth_test(frag_z, *dp, d.func)) {
                        *dp = frag_z;
                    } else {
                        pass = false;
                    }
                }

                if (pass) {
                    const float* frag = cur_v;
                    float persp_v[ESR_MAX_VARYINGS];
                    if constexpr (Perspective) {
                        // Recover A = (A/w) / (1/w): one reciprocal per shaded pixel.
                        const float w = 1.0f / q;
                        for (uint8_t k = 0; k < v_setup.count; ++k) {
                            persp_v[k] = cur_v[k] * w;
                        }
                        frag = persp_v;
                    }
                    if constexpr (UseShader) {
                        uint16_t col565 = pixel_shader(frag, uniforms);
                        p[0] = esr_rgb565_r(col565);
                        p[1] = esr_rgb565_g(col565);
                        p[2] = esr_rgb565_b(col565);
                    } else {
                        p[0] = unit_to_u8(frag[0]);
                        p[1] = unit_to_u8(frag[1]);
                        p[2] = unit_to_u8(frag[2]);
                    }
                    ++written;
                }
            } else if (entered) {
                break;
            }

            w0 += sx0; w1 += sx1; w2 += sx2;
            for (uint8_t k = 0; k < v_setup.count; ++k) {
                cur_v[k] += v_setup.dvdx[k];
            }
            if constexpr (Perspective) {
                q += v_setup.dqdx;
            }
            if constexpr (HasDepth) {
                z += d.dzdx;
                ++dp;
            }
            p += 3;
        }

        row0 += sy0; row1 += sy1; row2 += sy2;
        for (uint8_t k = 0; k < v_setup.count; ++k) {
            row_v[k] += v_setup.dvdy[k];
        }
        if constexpr (Perspective) {
            row_q += v_setup.dqdy;
        }
        row_ptr += pitch;
        if constexpr (HasDepth) {
            row_z += d.dzdy;
            depth_row_ptr += depth_pitch;
        }
    }

    return written;
}

/**
 * Walk the AABB. T is int32_t (fast path) or int64_t (huge triangles).
 * HasDepth enables zero-overhead compile-time elimination of depth testing when false.
 * Per pixel: 3 integer adds + 1 sign test for coverage, 3 integer adds for color,
 * and 1 integer add + Early-Z test when depth testing is active.
 */
template <typename T, bool HasDepth>
int walk(Framebuffer& fb, int minx, int miny, int maxx, int maxy, const Edge e[3], const ColorSetup& c, const DepthSetup& d) {
    const T sx0 = static_cast<T>(e[0].step_x);
    const T sx1 = static_cast<T>(e[1].step_x);
    const T sx2 = static_cast<T>(e[2].step_x);
    const T sy0 = static_cast<T>(e[0].step_y);
    const T sy1 = static_cast<T>(e[1].step_y);
    const T sy2 = static_cast<T>(e[2].step_y);

    T row0 = static_cast<T>(e[0].biased);
    T row1 = static_cast<T>(e[1].biased);
    T row2 = static_cast<T>(e[2].biased);
    int32_t row_r = c.r, row_g = c.g, row_b = c.b;
    int32_t row_z = d.z;

    const int pitch = fb.width * 3;
    uint8_t* row_ptr = fb.color + (static_cast<long>(miny) * fb.width + minx) * 3;

    uint16_t* depth_row_ptr = nullptr;
    int depth_pitch = 0;
    if constexpr (HasDepth) {
        depth_pitch = d.db->width;
        depth_row_ptr = d.db->data + (static_cast<long>(miny) * d.db->width + minx);
    }

    int written = 0;

    for (int y = miny; y <= maxy; ++y) {
        T w0 = row0, w1 = row1, w2 = row2;
        int32_t r = row_r, g = row_g, b = row_b;
        int32_t z = row_z;
        uint8_t* p = row_ptr;
        uint16_t* dp = depth_row_ptr;
        bool entered = false;

        for (int x = minx; x <= maxx; ++x) {
            // Inside <=> all three biased edge values are >= 0 (sign bits clear).
            if ((w0 | w1 | w2) >= 0) {
                entered = true;

                // Early-Z test: compare incoming fragment depth against depth buffer.
                if constexpr (HasDepth) {
                    const uint16_t frag_z = to_u16_depth(z);
                    if (eval_depth_test(frag_z, *dp, d.func)) {
                        *dp = frag_z; // Pass: update depth buffer
                        p[0] = to_u8(r);
                        p[1] = to_u8(g);
                        p[2] = to_u8(b);
                        ++written;
                    }
                } else {
                    p[0] = to_u8(r);
                    p[1] = to_u8(g);
                    p[2] = to_u8(b);
                    ++written;
                }
            } else if (entered) {
                break; // Triangle is convex: the span on this row has ended.
            }

            w0 += sx0; w1 += sx1; w2 += sx2;
            r += c.drdx; g += c.dgdx; b += c.dbdx;
            if constexpr (HasDepth) {
                z += d.dzdx;
                ++dp;
            }
            p += 3;
        }

        row0 += sy0; row1 += sy1; row2 += sy2;
        row_r += c.drdy; row_g += c.dgdy; row_b += c.dbdy;
        row_ptr += pitch;
        if constexpr (HasDepth) {
            row_z += d.dzdy;
            depth_row_ptr += depth_pitch;
        }
    }
    return written;
}

/** Instantiate walk_shader for the runtime (fits_32, has_depth) combination. */
template <bool Perspective, bool UseShader>
int dispatch_walk_shader(bool fits_32, bool has_depth,
                         Framebuffer& fb, int minx, int miny, int maxx, int maxy,
                         const Edge e[3], const VaryingSetup& v, const DepthSetup& d,
                         PixelShader ps, const void* uniforms) {
    if (has_depth) {
        return fits_32
            ? walk_shader<int32_t, true, Perspective, UseShader>(fb, minx, miny, maxx, maxy, e, v, d, ps, uniforms)
            : walk_shader<int64_t, true, Perspective, UseShader>(fb, minx, miny, maxx, maxy, e, v, d, ps, uniforms);
    }
    return fits_32
        ? walk_shader<int32_t, false, Perspective, UseShader>(fb, minx, miny, maxx, maxy, e, v, d, ps, uniforms)
        : walk_shader<int64_t, false, Perspective, UseShader>(fb, minx, miny, maxx, maxy, e, v, d, ps, uniforms);
}

/** True if inv_w is usable for perspective-correct interpolation (finite, > 0). */
inline bool valid_inv_w(float q) {
    return q > 0.0f && std::isfinite(q);
}

} // namespace

int rasterize_line(const ScreenVertex& v0,
                   const ScreenVertex& v1,
                   Framebuffer& fb,
                   DepthBuffer* db,
                   DepthFunc depth_func,
                   PixelShader pixel_shader,
                   const void* uniforms,
                   InterpMode interp_mode) {
    if (fb.color == nullptr || fb.width <= 0 || fb.height <= 0) {
        return 0;
    }

    if (!std::isfinite(v0.x) || !std::isfinite(v0.y) ||
        !std::isfinite(v1.x) || !std::isfinite(v1.y)) {
        return 0;
    }

    int x0 = static_cast<int>(std::floor(v0.x + 0.5f));
    int y0 = static_cast<int>(std::floor(v0.y + 0.5f));
    int x1 = static_cast<int>(std::floor(v1.x + 0.5f));
    int y1 = static_cast<int>(std::floor(v1.y + 0.5f));

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int steps = (dx > dy) ? dx : dy;
    float inv_steps = (steps > 0) ? (1.0f / static_cast<float>(steps)) : 0.0f;

    const bool flat = (interp_mode == InterpMode::Flat);
    const bool persp = (interp_mode == InterpMode::Perspective) &&
                       valid_inv_w(v0.inv_w) && valid_inv_w(v1.inv_w);
    // Flat: zero gradients, so every pixel keeps v0 (provoking vertex) attributes.
    if (flat) {
        inv_steps = 0.0f;
    }

    // Fixed-point color and depth interpolation
    int32_t r = to_color_fixed(v0.color.x);
    int32_t g = to_color_fixed(v0.color.y);
    int32_t b = to_color_fixed(v0.color.z);
    int32_t dr = to_color_fixed((v1.color.x - v0.color.x) * inv_steps);
    int32_t dg = to_color_fixed((v1.color.y - v0.color.y) * inv_steps);
    int32_t db_col = to_color_fixed((v1.color.z - v0.color.z) * inv_steps);

    // Depth is screen-linear in every mode (flat lines still need a sloped z).
    const float z_steps = (steps > 0) ? (1.0f / static_cast<float>(steps)) : 0.0f;
    int32_t z = to_depth_fixed(v0.z);
    int32_t dz = to_depth_gradient_fixed((v1.z - v0.z) * z_steps);

    // Float attribute stepping: color in slots [0..2], varyings in [3..].
    // Perspective: step A/w and 1/w, recover A per pixel. Screen-space/Flat: step A.
    constexpr int MAX_ATTR = 3 + ESR_MAX_VARYINGS;
    const uint8_t var_count = v0.varying_count;
    const int attr_count = 3 + var_count;
    float a0[MAX_ATTR], a1[MAX_ATTR];
    a0[0] = v0.color.x; a0[1] = v0.color.y; a0[2] = v0.color.z;
    a1[0] = v1.color.x; a1[1] = v1.color.y; a1[2] = v1.color.z;
    for (uint8_t k = 0; k < var_count; ++k) {
        a0[3 + k] = v0.varyings[k];
        a1[3 + k] = v1.varyings[k];
    }
    const float q0 = persp ? v0.inv_w : 1.0f;
    const float q1 = persp ? v1.inv_w : 1.0f;
    float cur_a[MAX_ATTR], da[MAX_ATTR];
    for (int k = 0; k < attr_count; ++k) {
        cur_a[k] = a0[k] * q0;
        da[k] = (a1[k] * q1 - a0[k] * q0) * inv_steps;
    }
    float cur_q = q0;
    const float dq = (q1 - q0) * inv_steps;
    float cur_v[ESR_MAX_VARYINGS];

    int written = 0;
    const bool has_depth = (db != nullptr && db->data != nullptr &&
                            db->width == fb.width && db->height == fb.height);

    while (true) {
        if (x0 >= 0 && x0 < fb.width && y0 >= 0 && y0 < fb.height) {
            bool pass = true;
            if (has_depth) {
                uint16_t frag_z = to_u16_depth(z);
                uint16_t* dp = db->data + (y0 * db->width + x0);
                if (eval_depth_test(frag_z, *dp, depth_func)) {
                    *dp = frag_z;
                } else {
                    pass = false;
                }
            }

            if (pass) {
                uint8_t* p = fb.color + (y0 * fb.width + x0) * 3;
                const float w = persp ? 1.0f / cur_q : 1.0f;
                for (uint8_t k = 0; k < var_count; ++k) {
                    cur_v[k] = cur_a[3 + k] * w;
                }
                if (persp && pixel_shader == nullptr) {
                    p[0] = unit_to_u8(cur_a[0] * w);
                    p[1] = unit_to_u8(cur_a[1] * w);
                    p[2] = unit_to_u8(cur_a[2] * w);
                } else if (pixel_shader != nullptr) {
                    uint16_t col565 = pixel_shader(cur_v, uniforms);
                    p[0] = esr_rgb565_r(col565);
                    p[1] = esr_rgb565_g(col565);
                    p[2] = esr_rgb565_b(col565);
                } else {
                    p[0] = to_u8(r);
                    p[1] = to_u8(g);
                    p[2] = to_u8(b);
                }
                ++written;
            }
        }

        if (x0 == x1 && y0 == y1) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }

        r += dr;
        g += dg;
        b += db_col;
        z += dz;
        for (int k = 0; k < attr_count; ++k) {
            cur_a[k] += da[k];
        }
        cur_q += dq;
    }

    return written;
}

int rasterize_triangle(const ScreenTriangle& tri,
                       Framebuffer& fb,
                       DepthBuffer* db,
                       DepthFunc depth_func,
                       PolygonMode polygon_mode,
                       PixelShader pixel_shader,
                       const void* uniforms,
                       InterpMode interp_mode) {
    if (fb.color == nullptr || fb.width <= 0 || fb.height <= 0) {
        return 0;
    }

    // Wireframe polygon mode: rasterize 3 edges with Bresenham line algorithm
    if (polygon_mode == PolygonMode::Wireframe) {
        int w = 0;
        if (interp_mode == InterpMode::Flat) {
            // Every edge takes the provoking vertex (v0) attributes.
            ScreenVertex e1 = tri.v[1], e2 = tri.v[2];
            e1.color = tri.v[0].color;
            e2.color = tri.v[0].color;
            for (uint8_t k = 0; k < tri.v[0].varying_count; ++k) {
                e1.varyings[k] = tri.v[0].varyings[k];
                e2.varyings[k] = tri.v[0].varyings[k];
            }
            w += rasterize_line(tri.v[0], e1, fb, db, depth_func, pixel_shader, uniforms, interp_mode);
            w += rasterize_line(e1, e2, fb, db, depth_func, pixel_shader, uniforms, InterpMode::ScreenSpace);
            w += rasterize_line(e2, tri.v[0], fb, db, depth_func, pixel_shader, uniforms, InterpMode::ScreenSpace);
            return w;
        }
        w += rasterize_line(tri.v[0], tri.v[1], fb, db, depth_func, pixel_shader, uniforms, interp_mode);
        w += rasterize_line(tri.v[1], tri.v[2], fb, db, depth_func, pixel_shader, uniforms, interp_mode);
        w += rasterize_line(tri.v[2], tri.v[0], fb, db, depth_func, pixel_shader, uniforms, interp_mode);
        return w;
    }

    for (int i = 0; i < 3; ++i) {
        // Negated comparison also rejects NaN.
        if (!(std::fabs(tri.v[i].x) <= ESR_RASTER_GUARD_BAND) ||
            !(std::fabs(tri.v[i].y) <= ESR_RASTER_GUARD_BAND)) {
            return 0;
        }
    }

    // 1. Snap to 28.4 fixed point.
    int32_t X[3], Y[3];
    const Vec3* col[3];
    const float* var[3];
    float Z[3];
    float Q[3]; // 1/w per vertex (perspective-correct interpolation)
    for (int i = 0; i < 3; ++i) {
        X[i] = to_fixed(tri.v[i].x);
        Y[i] = to_fixed(tri.v[i].y);
        col[i] = &tri.v[i].color;
        var[i] = tri.v[i].varyings;
        Z[i] = tri.v[i].z;
        Q[i] = tri.v[i].inv_w;
    }

    // 2. Signed doubled area; normalize winding so the interior is E > 0.
    int64_t area = (static_cast<int64_t>(X[2]) - X[0]) * (static_cast<int64_t>(Y[1]) - Y[0]) -
                   (static_cast<int64_t>(Y[2]) - Y[0]) * (static_cast<int64_t>(X[1]) - X[0]);
    if (area == 0) {
        return 0;
    }
    if (area < 0) {
        int32_t t;
        t = X[1]; X[1] = X[2]; X[2] = t;
        t = Y[1]; Y[1] = Y[2]; Y[2] = t;
        const Vec3* tc = col[1]; col[1] = col[2]; col[2] = tc;
        const float* tv = var[1]; var[1] = var[2]; var[2] = tv;
        float tz = Z[1]; Z[1] = Z[2]; Z[2] = tz;
        float tq = Q[1]; Q[1] = Q[2]; Q[2] = tq;
        area = -area;
    }

    // 3. AABB of pixel centers (x + 0.5) inside the triangle's extent,
    //    clamped to the framebuffer.
    int minx = ceil_div_sub(min3(X[0], X[1], X[2]) - SUB_HALF);
    int miny = ceil_div_sub(min3(Y[0], Y[1], Y[2]) - SUB_HALF);
    int maxx = floor_div_sub(max3(X[0], X[1], X[2]) - SUB_HALF);
    int maxy = floor_div_sub(max3(Y[0], Y[1], Y[2]) - SUB_HALF);
    if (minx < 0) minx = 0;
    if (miny < 0) miny = 0;
    if (maxx > fb.width - 1) maxx = fb.width - 1;
    if (maxy > fb.height - 1) maxy = fb.height - 1;
    if (minx > maxx || miny > maxy) {
        return 0; // Off-screen or covers no pixel center.
    }

    // 4. Edge functions at the first pixel center. Edge i is opposite vertex i,
    //    so E_i / area is the barycentric weight of vertex i.
    const int64_t px = static_cast<int64_t>(minx) * SUB_ONE + SUB_HALF;
    const int64_t py = static_cast<int64_t>(miny) * SUB_ONE + SUB_HALF;
    const Edge e[3] = {
        make_edge(X[1], Y[1], X[2], Y[2], px, py),
        make_edge(X[2], Y[2], X[0], Y[0], px, py),
        make_edge(X[0], Y[0], X[1], Y[1], px, py),
    };

    // 5. Color & depth gradients: one reciprocal per triangle instead of per-pixel divisions.
    const float inv_area = 1.0f / static_cast<float>(area);
    const float w0 = static_cast<float>(e[0].raw) * inv_area;
    const float w1 = static_cast<float>(e[1].raw) * inv_area;
    const float w2 = static_cast<float>(e[2].raw) * inv_area;
    const float dx0 = static_cast<float>(e[0].step_x) * inv_area;
    const float dx1 = static_cast<float>(e[1].step_x) * inv_area;
    const float dx2 = static_cast<float>(e[2].step_x) * inv_area;
    const float dy0 = static_cast<float>(e[0].step_y) * inv_area;
    const float dy1 = static_cast<float>(e[1].step_y) * inv_area;
    const float dy2 = static_cast<float>(e[2].step_y) * inv_area;

    DepthSetup d;
    const bool has_depth = (db != nullptr && db->data != nullptr &&
                            db->width == fb.width && db->height == fb.height);
    if (has_depth) {
        d.db = db;
        d.func = depth_func;
        // Depth (z_ndc = z/w) is linear in screen space in every interpolation mode.
        const float z0 = w0 * Z[0] + w1 * Z[1] + w2 * Z[2];
        const float dz_dx = dx0 * Z[0] + dx1 * Z[1] + dx2 * Z[2];
        const float dz_dy = dy0 * Z[0] + dy1 * Z[1] + dy2 * Z[2];
        d.z = to_depth_fixed(z0);
        d.dzdx = to_depth_gradient_fixed(dz_dx);
        d.dzdy = to_depth_gradient_fixed(dz_dy);
    }

    // 6. Walk the box. 32-bit accumulators whenever they cannot overflow
    //    (always true for normal on-screen geometry), 64-bit otherwise.
    const int span_x = maxx - minx + 1;
    const int span_y = maxy - miny + 1;
    const bool fits_32 = fits_int32(e[0], span_x, span_y) &&
                         fits_int32(e[1], span_x, span_y) &&
                         fits_int32(e[2], span_x, span_y);

    const bool use_shader = (pixel_shader != nullptr);

    // Perspective-correct mode needs a usable 1/w on every vertex; otherwise
    // (e.g. hand-built screen triangles with inv_w == 0) fall back to affine.
    InterpMode mode = interp_mode;
    if (mode == InterpMode::Perspective &&
        !(valid_inv_w(Q[0]) && valid_inv_w(Q[1]) && valid_inv_w(Q[2]))) {
        mode = InterpMode::ScreenSpace;
    }

    // ---- 7a. Flat: provoking vertex tri.v[0], zero gradients. --------------
    // tri.v[0] is never swapped by the winding normalization above.
    if (mode == InterpMode::Flat) {
        if (use_shader) {
            VaryingSetup v_setup;
            v_setup.count = tri.v[0].varying_count;
            for (uint8_t k = 0; k < v_setup.count; ++k) {
                v_setup.start[k] = tri.v[0].varyings[k];
            }
            return dispatch_walk_shader<false, true>(fits_32, has_depth, fb, minx, miny, maxx, maxy,
                                                     e, v_setup, d, pixel_shader, uniforms);
        }
        ColorSetup c{};
        c.r = to_color_fixed(tri.v[0].color.x);
        c.g = to_color_fixed(tri.v[0].color.y);
        c.b = to_color_fixed(tri.v[0].color.z);
        if (has_depth) {
            return fits_32 ? walk<int32_t, true>(fb, minx, miny, maxx, maxy, e, c, d)
                           : walk<int64_t, true>(fb, minx, miny, maxx, maxy, e, c, d);
        }
        return fits_32 ? walk<int32_t, false>(fb, minx, miny, maxx, maxy, e, c, d)
                       : walk<int64_t, false>(fb, minx, miny, maxx, maxy, e, c, d);
    }

    // ---- 7b. Perspective-correct: step A/w and 1/w, divide per pixel. -----
    if (mode == InterpMode::Perspective) {
        VaryingSetup v_setup;
        const float* src[3];
        float color_attr[3][3];
        if (use_shader) {
            v_setup.count = tri.v[0].varying_count;
            for (int i = 0; i < 3; ++i) src[i] = var[i];
        } else {
            v_setup.count = 3; // RGB written directly by the walker
            for (int i = 0; i < 3; ++i) {
                color_attr[i][0] = col[i]->x;
                color_attr[i][1] = col[i]->y;
                color_attr[i][2] = col[i]->z;
                src[i] = color_attr[i];
            }
        }
        for (uint8_t k = 0; k < v_setup.count; ++k) {
            const float a0 = src[0][k] * Q[0];
            const float a1 = src[1][k] * Q[1];
            const float a2 = src[2][k] * Q[2];
            v_setup.start[k] = w0 * a0 + w1 * a1 + w2 * a2;
            v_setup.dvdx[k]  = dx0 * a0 + dx1 * a1 + dx2 * a2;
            v_setup.dvdy[k]  = dy0 * a0 + dy1 * a1 + dy2 * a2;
        }
        v_setup.q    = w0 * Q[0] + w1 * Q[1] + w2 * Q[2];
        v_setup.dqdx = dx0 * Q[0] + dx1 * Q[1] + dx2 * Q[2];
        v_setup.dqdy = dy0 * Q[0] + dy1 * Q[1] + dy2 * Q[2];

        if (use_shader) {
            return dispatch_walk_shader<true, true>(fits_32, has_depth, fb, minx, miny, maxx, maxy,
                                                    e, v_setup, d, pixel_shader, uniforms);
        }
        return dispatch_walk_shader<true, false>(fits_32, has_depth, fb, minx, miny, maxx, maxy,
                                                 e, v_setup, d, nullptr, nullptr);
    }

    // ---- 7c. Screen-space (affine) barycentric interpolation. -------------
    if (use_shader) {
        VaryingSetup v_setup;
        v_setup.count = tri.v[0].varying_count;
        for (uint8_t k = 0; k < v_setup.count; ++k) {
            float val0 = var[0][k];
            float val1 = var[1][k];
            float val2 = var[2][k];
            v_setup.start[k] = w0 * val0 + w1 * val1 + w2 * val2;
            v_setup.dvdx[k]  = dx0 * val0 + dx1 * val1 + dx2 * val2;
            v_setup.dvdy[k]  = dy0 * val0 + dy1 * val1 + dy2 * val2;
        }
        return dispatch_walk_shader<false, true>(fits_32, has_depth, fb, minx, miny, maxx, maxy,
                                                 e, v_setup, d, pixel_shader, uniforms);
    }

    // Fast-path: default fixed-point Gouraud rasterization
    ColorSetup c;
    c.r = to_color_fixed(w0 * col[0]->x + w1 * col[1]->x + w2 * col[2]->x);
    c.g = to_color_fixed(w0 * col[0]->y + w1 * col[1]->y + w2 * col[2]->y);
    c.b = to_color_fixed(w0 * col[0]->z + w1 * col[1]->z + w2 * col[2]->z);
    c.drdx = to_color_fixed(dx0 * col[0]->x + dx1 * col[1]->x + dx2 * col[2]->x);
    c.dgdx = to_color_fixed(dx0 * col[0]->y + dx1 * col[1]->y + dx2 * col[2]->y);
    c.dbdx = to_color_fixed(dx0 * col[0]->z + dx1 * col[1]->z + dx2 * col[2]->z);
    c.drdy = to_color_fixed(dy0 * col[0]->x + dy1 * col[1]->x + dy2 * col[2]->x);
    c.dgdy = to_color_fixed(dy0 * col[0]->y + dy1 * col[1]->y + dy2 * col[2]->y);
    c.dbdy = to_color_fixed(dy0 * col[0]->z + dy1 * col[1]->z + dy2 * col[2]->z);

    if (has_depth) {
        if (fits_32) {
            return walk<int32_t, true>(fb, minx, miny, maxx, maxy, e, c, d);
        }
        return walk<int64_t, true>(fb, minx, miny, maxx, maxy, e, c, d);
    }
    if (fits_32) {
        return walk<int32_t, false>(fb, minx, miny, maxx, maxy, e, c, d);
    }
    return walk<int64_t, false>(fb, minx, miny, maxx, maxy, e, c, d);
}

} // namespace esr
