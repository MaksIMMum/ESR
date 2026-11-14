/**
 * test_esr_depth.cpp - Unit tests for the 16-bit Depth Buffer and Early-Z rejection.
 *
 * Requirements tested (TASK-05 / REQ-4):
 *   1. esr_depth_buffer_t / DepthBuffer management (uint16_t[H][W]).
 *   2. Fast buffer clearing clear_depth() to 0xFFFF via memset.
 *   3. Depth quantization mapping [0.0, 1.0] to [0, 65535].
 *   4. Early-Z comparison functions (LESS, LEQUAL, GREATER, ALWAYS, etc.).
 *   5. Overlapping triangles depth arbitration (Painter's algorithm resolution).
 *   6. Complete rejection of occluded geometry.
 *   7. Linear depth gradient interpolation across slanted triangles.
 *   8. 3D indexed mesh rendering with depth buffer.
 */
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>

#include "esr/esr_depth.hpp"
#include "esr/esr_raster.hpp"
#include "esr/esr_mesh.hpp"
#include "models/cube.h"

using namespace esr;

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        ++g_checks;                                                          \
        if (!(cond)) {                                                       \
            ++g_failures;                                                    \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
        }                                                                    \
    } while (0)

static const int W = 160;
static const int H = 120;

struct TestContext {
    int w, h;
    std::vector<uint8_t> color_data;
    std::vector<uint16_t> depth_data;

    TestContext(int w_ = W, int h_ = H)
        : w(w_), h(h_),
          color_data(static_cast<size_t>(w_) * h_ * 3, 0),
          depth_data(static_cast<size_t>(w_) * h_, 0xFFFF) {}

    Framebuffer fb() { return Framebuffer{color_data.data(), w, h}; }
    DepthBuffer db() { return DepthBuffer{depth_data.data(), w, h}; }

    void clear_color() { memset(color_data.data(), 0, color_data.size()); }
    void clear_depth_buf(uint16_t val = 0xFFFF) {
        DepthBuffer d = db();
        clear_depth(d, val);
    }

    const uint8_t* color_at(int x, int y) const {
        return &color_data[(static_cast<size_t>(y) * w + x) * 3];
    }

    uint16_t depth_at(int x, int y) const {
        return depth_data[static_cast<size_t>(y) * w + x];
    }
};

static ScreenVertex sv(float x, float y, float z, Vec3 c) {
    ScreenVertex v;
    v.x = x;
    v.y = y;
    v.z = z;
    v.inv_w = 1.0f;
    v.color = c;
    return v;
}

static ScreenTriangle tri(ScreenVertex a, ScreenVertex b, ScreenVertex c) {
    ScreenTriangle t;
    t.v[0] = a;
    t.v[1] = b;
    t.v[2] = c;
    return t;
}

/* ------------------------------------------------------------------ */
/* 1. Fast Buffer Clearing Tests                                       */
/* ------------------------------------------------------------------ */

static void test_clear_depth_default_0xffff() {
    TestContext ctx(W, H);
    // Fill with zeroes first
    memset(ctx.depth_data.data(), 0, ctx.depth_data.size() * sizeof(uint16_t));

    DepthBuffer db = ctx.db();
    clear_depth(db, 0xFFFF);

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            CHECK(ctx.depth_at(x, y) == 0xFFFF);
        }
    }
}

static void test_clear_depth_custom_values() {
    TestContext ctx(W, H);
    DepthBuffer db = ctx.db();

    // Clear to zero
    clear_depth(db, 0x0000);
    for (int i = 0; i < W * H; ++i) {
        CHECK(ctx.depth_data[i] == 0x0000);
    }

    // Clear to arbitrary pattern
    clear_depth(db, 0x1234);
    for (int i = 0; i < W * H; ++i) {
        CHECK(ctx.depth_data[i] == 0x1234);
    }
}

/* ------------------------------------------------------------------ */
/* 2. Quantization & Depth Test Comparison Functions                   */
/* ------------------------------------------------------------------ */

static void test_quantization() {
    CHECK(to_depth16(0.0f) == 0);
    CHECK(to_depth16(1.0f) == 65535);
    CHECK(std::abs(static_cast<int>(to_depth16(0.5f)) - 32768) <= 1);
    CHECK(to_depth16(-0.5f) == 0);      // Clamped
    CHECK(to_depth16(2.0f) == 65535);   // Clamped
}

static void test_depth_eval_functions() {
    const uint16_t closer = 1000;
    const uint16_t same   = 2000;
    const uint16_t farther = 3000;

    // Never
    CHECK(!eval_depth_test(closer, same, DepthFunc::Never));
    CHECK(!eval_depth_test(farther, same, DepthFunc::Never));

    // Less
    CHECK(eval_depth_test(closer, same, DepthFunc::Less));
    CHECK(!eval_depth_test(same, same, DepthFunc::Less));
    CHECK(!eval_depth_test(farther, same, DepthFunc::Less));

    // LEqual
    CHECK(eval_depth_test(closer, same, DepthFunc::LEqual));
    CHECK(eval_depth_test(same, same, DepthFunc::LEqual));
    CHECK(!eval_depth_test(farther, same, DepthFunc::LEqual));

    // Greater
    CHECK(!eval_depth_test(closer, same, DepthFunc::Greater));
    CHECK(!eval_depth_test(same, same, DepthFunc::Greater));
    CHECK(eval_depth_test(farther, same, DepthFunc::Greater));

    // GEqual
    CHECK(!eval_depth_test(closer, same, DepthFunc::GEqual));
    CHECK(eval_depth_test(same, same, DepthFunc::GEqual));
    CHECK(eval_depth_test(farther, same, DepthFunc::GEqual));

    // Equal
    CHECK(!eval_depth_test(closer, same, DepthFunc::Equal));
    CHECK(eval_depth_test(same, same, DepthFunc::Equal));
    CHECK(!eval_depth_test(farther, same, DepthFunc::Equal));

    // NotEqual
    CHECK(eval_depth_test(closer, same, DepthFunc::NotEqual));
    CHECK(!eval_depth_test(same, same, DepthFunc::NotEqual));
    CHECK(eval_depth_test(farther, same, DepthFunc::NotEqual));

    // Always
    CHECK(eval_depth_test(closer, same, DepthFunc::Always));
    CHECK(eval_depth_test(same, same, DepthFunc::Always));
    CHECK(eval_depth_test(farther, same, DepthFunc::Always));
}

/* ------------------------------------------------------------------ */
/* 3. Overlapping Triangles (Painter's Algorithm Arbitration)           */
/* ------------------------------------------------------------------ */

static void test_overlapping_triangles_front_to_back() {
    // Red triangle at Z = 0.8 (Farther)
    // Green triangle at Z = 0.3 (Closer)
    const Vec3 RED   = {1.0f, 0.0f, 0.0f};
    const Vec3 GREEN = {0.0f, 1.0f, 0.0f};

    ScreenTriangle tri_far = tri(sv(20.0f, 20.0f, 0.8f, RED),
                                 sv(80.0f, 20.0f, 0.8f, RED),
                                 sv(50.0f, 80.0f, 0.8f, RED));

    ScreenTriangle tri_near = tri(sv(40.0f, 40.0f, 0.3f, GREEN),
                                  sv(100.0f, 40.0f, 0.3f, GREEN),
                                  sv(70.0f, 100.0f, 0.3f, GREEN));

    // Order 1: Render Far (Red) first, then Near (Green) second
    TestContext ctx1(W, H);
    Framebuffer fb1 = ctx1.fb();
    DepthBuffer db1 = ctx1.db();

    int px_far1 = rasterize_triangle(tri_far, fb1, &db1);
    int px_near1 = rasterize_triangle(tri_near, fb1, &db1);
    CHECK(px_far1 > 0);
    CHECK(px_near1 > 0);

    // Order 2: Render Near (Green) first, then Far (Red) second
    TestContext ctx2(W, H);
    Framebuffer fb2 = ctx2.fb();
    DepthBuffer db2 = ctx2.db();

    int px_near2 = rasterize_triangle(tri_near, fb2, &db2);
    int px_far2 = rasterize_triangle(tri_far, fb2, &db2);
    CHECK(px_near2 > 0);
    // In Order 2, some pixels of tri_far should be rejected by Early-Z where tri_near occludes it!
    CHECK(px_far2 < px_far1);

    // BOTH framebuffers must be pixel-identical!
    // The closer Green triangle MUST be in front in both cases!
    CHECK(ctx1.color_data == ctx2.color_data);
    CHECK(ctx1.depth_data == ctx2.depth_data);

    // Check center of the overlap region: must be pure green!
    // Near triangle center is roughly at (70, 60)
    const uint8_t* overlap_color = ctx2.color_at(60, 50);
    CHECK(overlap_color[1] == 255); // Green channel full
    CHECK(overlap_color[0] == 0);   // Red channel zero

    // Check depth value at overlap region: must correspond to Z=0.3 (~19660)
    uint16_t overlap_depth = ctx2.depth_at(60, 50);
    CHECK(std::abs(static_cast<int>(overlap_depth) - static_cast<int>(0.3f * 65535.0f)) <= 50);
}

/* ------------------------------------------------------------------ */
/* 4. Complete Early-Z Rejection of Occluded Triangle                   */
/* ------------------------------------------------------------------ */

static void test_complete_occlusion_early_z() {
    const Vec3 GREEN = {0.0f, 1.0f, 0.0f};
    const Vec3 RED   = {1.0f, 0.0f, 0.0f};

    // Front triangle (Green, Z = 0.2)
    ScreenTriangle front = tri(sv(30.0f, 30.0f, 0.2f, GREEN),
                               sv(90.0f, 30.0f, 0.2f, GREEN),
                               sv(60.0f, 90.0f, 0.2f, GREEN));

    // Identical position triangle behind it (Red, Z = 0.7)
    ScreenTriangle behind = tri(sv(30.0f, 30.0f, 0.7f, RED),
                                sv(90.0f, 30.0f, 0.7f, RED),
                                sv(60.0f, 90.0f, 0.7f, RED));

    TestContext ctx(W, H);
    Framebuffer fb = ctx.fb();
    DepthBuffer db = ctx.db();

    int written_front = rasterize_triangle(front, fb, &db);
    CHECK(written_front > 0);

    // Render the occluded triangle: Early-Z should reject EVERY single pixel!
    int written_behind = rasterize_triangle(behind, fb, &db);
    CHECK(written_behind == 0);

    // Color buffer should remain completely free of any red pixels
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const uint8_t* c = ctx.color_at(x, y);
            CHECK(c[0] == 0); // No red
        }
    }
}

/* ------------------------------------------------------------------ */
/* 5. Linear Depth Gradient Across Slanted Triangle                     */
/* ------------------------------------------------------------------ */

static void test_slanted_triangle_depth_gradient() {
    const Vec3 WHITE = {1.0f, 1.0f, 1.0f};

    // Triangle slanted along X: left vertex at z = 0.2, right vertex at z = 0.8
    ScreenTriangle slanted = tri(sv(20.0f, 20.0f, 0.2f, WHITE),
                                 sv(80.0f, 20.0f, 0.8f, WHITE),
                                 sv(20.0f, 80.0f, 0.2f, WHITE));

    TestContext ctx(W, H);
    Framebuffer fb = ctx.fb();
    DepthBuffer db = ctx.db();

    int written = rasterize_triangle(slanted, fb, &db);
    CHECK(written > 0);

    // Verify depth increases monotonically along scanline y = 25
    uint16_t prev_depth = 0;
    bool checked = false;
    for (int x = 25; x <= 70; ++x) {
        uint16_t d = ctx.depth_at(x, 25);
        if (d != 0xFFFF) {
            if (checked) {
                CHECK(d > prev_depth); // Monotonically increasing depth
            }
            prev_depth = d;
            checked = true;
        }
    }
    CHECK(checked);
}

/* ------------------------------------------------------------------ */
/* 6. Mesh Rendering with Depth Buffer                                 */
/* ------------------------------------------------------------------ */

static void test_mesh_with_depth_buffer() {
    TestContext ctx(W, H);
    Framebuffer fb = ctx.fb();
    DepthBuffer db = ctx.db();

    Mat4 proj = mat4_perspective(deg_to_rad(60.0f), (float)W / (float)H, 0.1f, 100.0f);
    Mat4 view = mat4_look_at(vec3(0.0f, 0.0f, 3.5f), vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f));
    Mat4 rot  = mat4_rotate_y(deg_to_rad(45.0f));
    Mat4 mvp  = mat4_mul_mat4(proj, mat4_mul_mat4(view, rot));
    Viewport vp = {0.0f, 0.0f, (float)W, (float)H, 0.0f, 1.0f};

    int pixels_with_depth = draw_mesh(CUBE_MESH, mvp, vp, fb, &db);
    CHECK(pixels_with_depth > 0);

    // Verify depth buffer has been updated (not all 0xFFFF)
    int modified_depth_pixels = 0;
    for (int i = 0; i < W * H; ++i) {
        if (ctx.depth_data[i] < 0xFFFF) {
            ++modified_depth_pixels;
        }
    }
    CHECK(modified_depth_pixels > 0);
    // Total pixels drawn across all triangles (including overdraw) >= unique pixels in depth buffer
    CHECK(pixels_with_depth >= modified_depth_pixels);
}

int main() {
    printf("Running ESR 16-Bit Depth Buffer & Early-Z unit tests...\n");

    test_clear_depth_default_0xffff();
    test_clear_depth_custom_values();
    test_quantization();
    test_depth_eval_functions();
    test_overlapping_triangles_front_to_back();
    test_complete_occlusion_early_z();
    test_slanted_triangle_depth_gradient();
    test_mesh_with_depth_buffer();

    printf("ESR Depth tests completed: %d checks, %d failed.\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
