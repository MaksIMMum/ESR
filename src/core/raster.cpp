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

} // namespace

int rasterize_triangle(const ScreenTriangle& tri,
                       Framebuffer& fb,
                       DepthBuffer* db,
                       DepthFunc depth_func) {
    if (fb.color == nullptr || fb.width <= 0 || fb.height <= 0) {
        return 0;
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
    float Z[3];
    for (int i = 0; i < 3; ++i) {
        X[i] = to_fixed(tri.v[i].x);
        Y[i] = to_fixed(tri.v[i].y);
        col[i] = &tri.v[i].color;
        Z[i] = tri.v[i].z;
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
        float tz = Z[1]; Z[1] = Z[2]; Z[2] = tz;
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

    DepthSetup d;
    const bool has_depth = (db != nullptr && db->data != nullptr &&
                            db->width == fb.width && db->height == fb.height);
    if (has_depth) {
        d.db = db;
        d.func = depth_func;
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

    if (has_depth) {
        if (fits_32) {
            return walk<int32_t, true>(fb, minx, miny, maxx, maxy, e, c, d);
        }
        return walk<int64_t, true>(fb, minx, miny, maxx, maxy, e, c, d);
    } else {
        if (fits_32) {
            return walk<int32_t, false>(fb, minx, miny, maxx, maxy, e, c, d);
        }
        return walk<int64_t, false>(fb, minx, miny, maxx, maxy, e, c, d);
    }
}

} // namespace esr
