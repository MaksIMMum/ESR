/**
 * test_esr_culling.cpp - Unit tests for Face Culling and Wireframe Polygon Mode.
 *
 * Requirements tested (TASK-08 / REQ-5):
 *   1. Screen-space 2D cross-product calculation for winding order detection.
 *   2. Culling modes: CULL_NONE, CULL_CW, CULL_CCW, CULL_BACK, CULL_FRONT.
 *   3. Vertex winding convention support (WindingOrder::CCW vs CW).
 *   4. Early triangle rejection in process_triangle() before rasterization.
 *   5. Integer Bresenham line rasterizer with color & depth interpolation.
 *   6. Polygon modes: POLYGON_MODE_FILL vs POLYGON_MODE_WIREFRAME.
 *   7. Full 3D mesh rendering with culling and wireframe pipeline states.
 */

#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>

#include "esr/esr_pipeline.hpp"
#include "esr/esr_raster.hpp"
#include "esr/esr_depth.hpp"
#include "esr/esr_mesh.hpp"
#include "models/cube.h"
#include "models/icosphere.h"

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
        if (x < 0 || x >= w || y < 0 || y >= h) return nullptr;
        return &color_data[(y * w + x) * 3];
    }

    uint16_t depth_at(int x, int y) const {
        if (x < 0 || x >= w || y < 0 || y >= h) return 0;
        return depth_data[y * w + x];
    }

    int count_non_black_pixels() const {
        int count = 0;
        for (size_t i = 0; i < color_data.size(); i += 3) {
            if (color_data[i] != 0 || color_data[i + 1] != 0 || color_data[i + 2] != 0) {
                ++count;
            }
        }
        return count;
    }
};

/* ------------------------------------------------------------------ */
/* 1. Screen-space Signed Area & Winding Detection                    */
/* ------------------------------------------------------------------ */

static void test_signed_area_and_winding() {
    printf("[1/7] Testing screen-space signed area and winding detection...\n");

    // In screen coordinates, Y points down (0 is top, H is bottom).
    // Counter-Clockwise (CCW) triangle on screen:
    // v0 = (10, 50) [bottom-left]
    // v1 = (50, 50) [bottom-right]
    // v2 = (10, 10) [top-left]
    // Going v0 -> v1 -> v2 is Counter-Clockwise.
    ScreenVertex ccw_v0{10.0f, 50.0f, 0.5f, 1.0f, {1, 1, 1}};
    ScreenVertex ccw_v1{50.0f, 50.0f, 0.5f, 1.0f, {1, 1, 1}};
    ScreenVertex ccw_v2{10.0f, 10.0f, 0.5f, 1.0f, {1, 1, 1}};

    float ccw_area = screen_triangle_signed_area(ccw_v0, ccw_v1, ccw_v2);
    // (50 - 10)*(10 - 50) - (50 - 50)*(10 - 10) = 40 * (-40) = -1600 < 0
    CHECK(ccw_area < 0.0f);
    CHECK(ccw_area == -1600.0f);

    // Clockwise (CW) triangle on screen:
    // v0 = (10, 10) [top-left]
    // v1 = (50, 10) [top-right]
    // v2 = (50, 50) [bottom-right]
    // Going v0 -> v1 -> v2 is Clockwise.
    ScreenVertex cw_v0{10.0f, 10.0f, 0.5f, 1.0f, {1, 1, 1}};
    ScreenVertex cw_v1{50.0f, 10.0f, 0.5f, 1.0f, {1, 1, 1}};
    ScreenVertex cw_v2{50.0f, 50.0f, 0.5f, 1.0f, {1, 1, 1}};

    float cw_area = screen_triangle_signed_area(cw_v0, cw_v1, cw_v2);
    // (50 - 10)*(50 - 10) - (10 - 10)*(50 - 10) = 40 * 40 = +1600 > 0
    CHECK(cw_area > 0.0f);
    CHECK(cw_area == 1600.0f);

    // Degenerate collinear points:
    ScreenVertex degen_v0{10.0f, 10.0f, 0.5f, 1.0f, {1, 1, 1}};
    ScreenVertex degen_v1{20.0f, 20.0f, 0.5f, 1.0f, {1, 1, 1}};
    ScreenVertex degen_v2{30.0f, 30.0f, 0.5f, 1.0f, {1, 1, 1}};

    float degen_area = screen_triangle_signed_area(degen_v0, degen_v1, degen_v2);
    CHECK(degen_area == 0.0f);
}

/* ------------------------------------------------------------------ */
/* 2. Culling Mode Logic (is_triangle_culled)                         */
/* ------------------------------------------------------------------ */

static void test_cull_mode_logic() {
    printf("[2/7] Testing culling mode decisions...\n");

    const float ccw_area = -100.0f;
    const float cw_area = 100.0f;
    const float zero_area = 0.0f;

    // CullMode::None / CULL_NONE: cull nothing
    CHECK(!is_triangle_culled(ccw_area, CullMode::None));
    CHECK(!is_triangle_culled(cw_area, CullMode::None));
    CHECK(!is_triangle_culled(zero_area, CullMode::None));
    CHECK(!is_triangle_culled(ccw_area, CULL_NONE));

    // CullMode::CW / CULL_CW: cull only CW
    CHECK(!is_triangle_culled(ccw_area, CullMode::CW));
    CHECK(is_triangle_culled(cw_area, CullMode::CW));
    CHECK(is_triangle_culled(zero_area, CullMode::CW));
    CHECK(is_triangle_culled(cw_area, CULL_CW));

    // CullMode::CCW / CULL_CCW: cull only CCW
    CHECK(is_triangle_culled(ccw_area, CullMode::CCW));
    CHECK(!is_triangle_culled(cw_area, CullMode::CCW));
    CHECK(is_triangle_culled(zero_area, CullMode::CCW));
    CHECK(is_triangle_culled(ccw_area, CULL_CCW));

    // CullMode::Back with standard front_face = CCW
    CHECK(!is_triangle_culled(ccw_area, CullMode::Back, WindingOrder::CCW));
    CHECK(is_triangle_culled(cw_area, CullMode::Back, WindingOrder::CCW));
    CHECK(is_triangle_culled(cw_area, CULL_BACK, WINDING_CCW));

    // CullMode::Front with standard front_face = CCW
    CHECK(is_triangle_culled(ccw_area, CullMode::Front, WindingOrder::CCW));
    CHECK(!is_triangle_culled(cw_area, CullMode::Front, WindingOrder::CCW));
    CHECK(is_triangle_culled(ccw_area, CULL_FRONT, WINDING_CCW));

    // CullMode::Back with reversed front_face = CW
    CHECK(is_triangle_culled(ccw_area, CullMode::Back, WindingOrder::CW));
    CHECK(!is_triangle_culled(cw_area, CullMode::Back, WindingOrder::CW));

    // CullMode::Front with reversed front_face = CW
    CHECK(!is_triangle_culled(ccw_area, CullMode::Front, WindingOrder::CW));
    CHECK(is_triangle_culled(cw_area, CullMode::Front, WindingOrder::CW));
}

/* ------------------------------------------------------------------ */
/* 3. process_triangle Culling Integration                            */
/* ------------------------------------------------------------------ */

static void test_process_triangle_culling() {
    printf("[3/7] Testing process_triangle early culling integration...\n");

    Viewport vp{0, 0, 160, 120, 0.0f, 1.0f};

    // Construct a front-facing CCW triangle in clip space [-1, 1]:
    // In NDC: v0 = (-0.5, -0.5), v1 = (0.5, -0.5), v2 = (0.0, 0.5)
    // When mapped to screen (Y down):
    // v0.y = (1 - (-0.5)) * 60 = 90
    // v1.y = 90
    // v2.y = (1 - 0.5) * 60 = 30
    // v0=(40, 90), v1=(120, 90), v2=(80, 30) -> Screen CCW!
    Vertex3D front_v0{{-0.5f, -0.5f, -2.0f}, {1, 0, 0}};
    Vertex3D front_v1{{ 0.5f, -0.5f, -2.0f}, {0, 1, 0}};
    Vertex3D front_v2{{ 0.0f,  0.5f, -2.0f}, {0, 0, 1}};

    // Back-facing triangle has reversed winding:
    Vertex3D back_v0 = front_v0;
    Vertex3D back_v1 = front_v2;
    Vertex3D back_v2 = front_v1;

    // Camera looking down -Z:
    Mat4 proj = mat4_perspective(deg_to_rad(60.0f), 160.0f / 120.0f, 0.1f, 100.0f);

    ScreenTriangle out_tris[ESR_MAX_CLIPPED_TRIANGLES];

    // With CullMode::None, both front and back triangles pass:
    int count_front_none = process_triangle(front_v0, front_v1, front_v2, proj, vp, out_tris, CullMode::None);
    CHECK(count_front_none == 1);

    int count_back_none = process_triangle(back_v0, back_v1, back_v2, proj, vp, out_tris, CullMode::None);
    CHECK(count_back_none == 1);

    // With CullMode::Back, front passes, back is culled:
    int count_front_backcull = process_triangle(front_v0, front_v1, front_v2, proj, vp, out_tris, CullMode::Back);
    CHECK(count_front_backcull == 1);

    int count_back_backcull = process_triangle(back_v0, back_v1, back_v2, proj, vp, out_tris, CullMode::Back);
    CHECK(count_back_backcull == 0); // Early culled before rasterization!

    // With CullMode::Front, front is culled, back passes:
    int count_front_frontcull = process_triangle(front_v0, front_v1, front_v2, proj, vp, out_tris, CullMode::Front);
    CHECK(count_front_frontcull == 0);

    int count_back_frontcull = process_triangle(back_v0, back_v1, back_v2, proj, vp, out_tris, CullMode::Front);
    CHECK(count_back_frontcull == 1);
}

/* ------------------------------------------------------------------ */
/* 4. Bresenham Line Rasterizer (rasterize_line)                       */
/* ------------------------------------------------------------------ */

static void test_rasterize_line() {
    printf("[4/7] Testing Bresenham line rasterizer...\n");

    TestContext ctx(100, 100);
    Framebuffer fb = ctx.fb();
    DepthBuffer db = ctx.db();

    // 1. Horizontal line: (10, 20) -> (30, 20)
    ctx.clear_color();
    ScreenVertex h0{10.0f, 20.0f, 0.5f, 1.0f, {1.0f, 0.0f, 0.0f}};
    ScreenVertex h1{30.0f, 20.0f, 0.5f, 1.0f, {1.0f, 0.0f, 0.0f}};
    int h_pixels = rasterize_line(h0, h1, fb);
    CHECK(h_pixels == 21); // 10..30 inclusive is 21 pixels
    for (int x = 10; x <= 30; ++x) {
        const uint8_t* p = ctx.color_at(x, 20);
        CHECK(p[0] == 255 && p[1] == 0 && p[2] == 0);
    }
    // Pixel above/below must be black:
    CHECK(ctx.color_at(10, 19)[0] == 0);
    CHECK(ctx.color_at(10, 21)[0] == 0);

    // 2. Vertical line: (50, 10) -> (50, 40)
    ctx.clear_color();
    ScreenVertex v0{50.0f, 10.0f, 0.5f, 1.0f, {0.0f, 1.0f, 0.0f}};
    ScreenVertex v1{50.0f, 40.0f, 0.5f, 1.0f, {0.0f, 1.0f, 0.0f}};
    int v_pixels = rasterize_line(v0, v1, fb);
    CHECK(v_pixels == 31); // 10..40 inclusive is 31 pixels
    for (int y = 10; y <= 40; ++y) {
        const uint8_t* p = ctx.color_at(50, y);
        CHECK(p[0] == 0 && p[1] == 255 && p[2] == 0);
    }

    // 3. Diagonal line: (0, 0) -> (20, 20)
    ctx.clear_color();
    ScreenVertex d0{0.0f, 0.0f, 0.5f, 1.0f, {0.0f, 0.0f, 1.0f}};
    ScreenVertex d1{20.0f, 20.0f, 0.5f, 1.0f, {0.0f, 0.0f, 1.0f}};
    int d_pixels = rasterize_line(d0, d1, fb);
    CHECK(d_pixels == 21);
    for (int i = 0; i <= 20; ++i) {
        const uint8_t* p = ctx.color_at(i, i);
        CHECK(p[2] == 255);
    }

    // 4. Color interpolation along line: (0, 0) Red -> (20, 0) Blue
    ctx.clear_color();
    ScreenVertex c0{0.0f, 5.0f, 0.5f, 1.0f, {1.0f, 0.0f, 0.0f}};
    ScreenVertex c1{20.0f, 5.0f, 0.5f, 1.0f, {0.0f, 0.0f, 1.0f}};
    rasterize_line(c0, c1, fb);
    const uint8_t* start_col = ctx.color_at(0, 5);
    const uint8_t* mid_col = ctx.color_at(10, 5);
    const uint8_t* end_col = ctx.color_at(20, 5);
    CHECK(start_col[0] == 255 && start_col[2] == 0);
    CHECK(mid_col[0] >= 120 && mid_col[0] <= 135);
    CHECK(mid_col[2] >= 120 && mid_col[2] <= 135);
    CHECK(end_col[0] == 0 && end_col[2] == 255);

    // 5. Line depth testing & occlusions:
    ctx.clear_color();
    ctx.clear_depth_buf();
    // Draw a green line at depth z = 0.5
    ScreenVertex l_far0{10.0f, 50.0f, 0.5f, 1.0f, {0.0f, 1.0f, 0.0f}};
    ScreenVertex l_far1{30.0f, 50.0f, 0.5f, 1.0f, {0.0f, 1.0f, 0.0f}};
    rasterize_line(l_far0, l_far1, fb, &db, DepthFunc::Less);

    // Draw a red line crossing at depth z = 0.2 (closer) -> should overwrite
    ScreenVertex l_near0{20.0f, 40.0f, 0.2f, 1.0f, {1.0f, 0.0f, 0.0f}};
    ScreenVertex l_near1{20.0f, 60.0f, 0.2f, 1.0f, {1.0f, 0.0f, 0.0f}};
    rasterize_line(l_near0, l_near1, fb, &db, DepthFunc::Less);
    const uint8_t* cross_px = ctx.color_at(20, 50);
    CHECK(cross_px[0] == 255 && cross_px[1] == 0); // Closer red won!

    // Draw a blue line crossing at depth z = 0.8 (further) -> should be rejected
    ScreenVertex l_behind0{20.0f, 40.0f, 0.8f, 1.0f, {0.0f, 0.0f, 1.0f}};
    ScreenVertex l_behind1{20.0f, 60.0f, 0.8f, 1.0f, {0.0f, 0.0f, 1.0f}};
    rasterize_line(l_behind0, l_behind1, fb, &db, DepthFunc::Less);
    cross_px = ctx.color_at(20, 50);
    CHECK(cross_px[0] == 255 && cross_px[2] == 0); // Still red! Occluded blue rejected.

    // 6. Clipping / boundary checks:
    // Out of bounds line should not crash or corrupt memory:
    ScreenVertex off0{-100.0f, -50.0f, 0.5f, 1.0f, {1.0f, 1.0f, 1.0f}};
    ScreenVertex off1{-10.0f, -5.0f, 0.5f, 1.0f, {1.0f, 1.0f, 1.0f}};
    int off_pixels = rasterize_line(off0, off1, fb);
    CHECK(off_pixels == 0);

    // Partially clipped line:
    ctx.clear_color();
    ScreenVertex part0{-10.0f, 50.0f, 0.5f, 1.0f, {1.0f, 1.0f, 1.0f}};
    ScreenVertex part1{10.0f, 50.0f, 0.5f, 1.0f, {1.0f, 1.0f, 1.0f}};
    int part_pixels = rasterize_line(part0, part1, fb);
    CHECK(part_pixels == 11); // 0..10 inclusive are within [0, 100)
}

/* ------------------------------------------------------------------ */
/* 5. PolygonMode::Wireframe vs PolygonMode::Fill                     */
/* ------------------------------------------------------------------ */

static void test_polygon_modes() {
    printf("[5/7] Testing PolygonMode::Fill vs PolygonMode::Wireframe...\n");

    TestContext ctx_fill(80, 80);
    TestContext ctx_wire(80, 80);

    // Large triangle: (10, 10), (70, 10), (40, 70)
    ScreenTriangle tri;
    tri.v[0] = ScreenVertex{10.0f, 10.0f, 0.5f, 1.0f, {1.0f, 0.0f, 0.0f}};
    tri.v[1] = ScreenVertex{70.0f, 10.0f, 0.5f, 1.0f, {0.0f, 1.0f, 0.0f}};
    tri.v[2] = ScreenVertex{40.0f, 70.0f, 0.5f, 1.0f, {0.0f, 0.0f, 1.0f}};

    Framebuffer fb_fill = ctx_fill.fb();
    Framebuffer fb_wire = ctx_wire.fb();

    int fill_pixels = rasterize_triangle(tri, fb_fill, nullptr, DepthFunc::Less, PolygonMode::Fill);
    int wire_pixels = rasterize_triangle(tri, fb_wire, nullptr, DepthFunc::Less, PolygonMode::Wireframe);

    CHECK(fill_pixels > 0);
    CHECK(wire_pixels > 0);
    // Wireframe must draw significantly fewer pixels than solid fill:
    CHECK(wire_pixels < fill_pixels);
    printf("       Solid fill pixels: %d, Wireframe pixels: %d (ratio: %.1fx)\n",
           fill_pixels, wire_pixels, (float)fill_pixels / wire_pixels);

    // Centroid of the triangle: (40, 30)
    // In Solid Fill, the centroid must be colored:
    const uint8_t* fill_center = ctx_fill.color_at(40, 30);
    CHECK(fill_center[0] != 0 || fill_center[1] != 0 || fill_center[2] != 0);

    // In Wireframe, the interior centroid MUST be blank (0, 0, 0):
    const uint8_t* wire_center = ctx_wire.color_at(40, 30);
    CHECK(wire_center[0] == 0 && wire_center[1] == 0 && wire_center[2] == 0);

    // On the edges (e.g. at (40, 10) on edge v0->v1), wireframe must have pixels:
    const uint8_t* wire_edge = ctx_wire.color_at(40, 10);
    CHECK(wire_edge[0] != 0 || wire_edge[1] != 0 || wire_edge[2] != 0);
}

/* ------------------------------------------------------------------ */
/* 6. draw_mesh with Culling Pipeline State                           */
/* ------------------------------------------------------------------ */

static void test_draw_mesh_culling() {
    printf("[6/7] Testing draw_mesh culling optimization...\n");

    TestContext ctx_none(160, 120);
    TestContext ctx_back(160, 120);
    Viewport vp{0, 0, 160, 120, 0.0f, 1.0f};

    Mat4 view = mat4_look_at({0.0f, 0.0f, 3.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
    Mat4 proj = mat4_perspective(deg_to_rad(60.0f), 160.0f / 120.0f, 0.1f, 100.0f);
    Mat4 mvp = mat4_mul_mat4(proj, view);

    // 1. Triangle count verification across mesh with process_triangle:
    int emitted_none = 0;
    int emitted_back = 0;
    ScreenTriangle out_tris[ESR_MAX_CLIPPED_TRIANGLES];

    for (uint32_t i = 0; i + 2 < ICOSPHERE_MESH.index_count; i += 3) {
        const Vertex3D& v0 = ICOSPHERE_MESH.vertices[ICOSPHERE_MESH.indices[i + 0]];
        const Vertex3D& v1 = ICOSPHERE_MESH.vertices[ICOSPHERE_MESH.indices[i + 1]];
        const Vertex3D& v2 = ICOSPHERE_MESH.vertices[ICOSPHERE_MESH.indices[i + 2]];

        emitted_none += process_triangle(v0, v1, v2, mvp, vp, out_tris, CullMode::None);
        emitted_back += process_triangle(v0, v1, v2, mvp, vp, out_tris, CullMode::Back, WindingOrder::CCW);
    }

    printf("       Icosphere (80 tris): CullMode::None = %d tris, CullMode::Back = %d tris\n",
           emitted_none, emitted_back);
    CHECK(emitted_none == 80);
    // Backface culling on a sphere facing camera perspective should cull back-facing triangles:
    CHECK(emitted_back > 0 && emitted_back < emitted_none);
    CHECK(emitted_back >= 25 && emitted_back <= 45);

    // 2. Pixel drawing verification without depth buffer (evaluating overdraw prevention):
    PipelineState state_none;
    state_none.cull_mode = CullMode::None;
    state_none.polygon_mode = PolygonMode::Fill;

    PipelineState state_back;
    state_back.cull_mode = CullMode::Back;
    state_back.front_face = WindingOrder::CCW;
    state_back.polygon_mode = PolygonMode::Fill;

    Framebuffer fb_none = ctx_none.fb();
    Framebuffer fb_back = ctx_back.fb();

    int pixels_none = draw_mesh(ICOSPHERE_MESH, mvp, vp, fb_none, nullptr, state_none);
    int pixels_back = draw_mesh(ICOSPHERE_MESH, mvp, vp, fb_back, nullptr, state_back);

    printf("       Pixels drawn: CullMode::None = %d, CullMode::Back = %d (saved %.1f%%)\n",
           pixels_none, pixels_back, (1.0f - (float)pixels_back / pixels_none) * 100.0f);
    CHECK(pixels_none > 0);
    CHECK(pixels_back > 0);
    CHECK(pixels_back < pixels_none);
}

/* ------------------------------------------------------------------ */
/* 7. draw_mesh Wireframe Rendering                                   */
/* ------------------------------------------------------------------ */

static void test_draw_mesh_wireframe() {
    printf("[7/7] Testing draw_mesh wireframe rendering...\n");

    TestContext ctx_solid(160, 120);
    TestContext ctx_wire(160, 120);
    Viewport vp{0, 0, 160, 120, 0.0f, 1.0f};

    Mat4 view = mat4_look_at({2.0f, 2.0f, 3.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
    Mat4 proj = mat4_perspective(deg_to_rad(60.0f), 160.0f / 120.0f, 0.1f, 100.0f);
    Mat4 mvp = mat4_mul_mat4(proj, view);

    PipelineState state_solid;
    state_solid.cull_mode = CullMode::Back;
    state_solid.polygon_mode = PolygonMode::Fill;

    Framebuffer fb_solid = ctx_solid.fb();
    DepthBuffer db_solid = ctx_solid.db();
    draw_mesh(CUBE_MESH, mvp, vp, fb_solid, &db_solid, state_solid);
    int solid_pixels = ctx_solid.count_non_black_pixels();

    PipelineState state_wire;
    state_wire.cull_mode = CullMode::Back;
    state_wire.polygon_mode = PolygonMode::Wireframe;

    Framebuffer fb_wire = ctx_wire.fb();
    DepthBuffer db_wire = ctx_wire.db();
    draw_mesh(CUBE_MESH, mvp, vp, fb_wire, &db_wire, state_wire);
    int wire_pixels = ctx_wire.count_non_black_pixels();

    printf("       Cube Solid drawn pixels: %d, Wireframe drawn pixels: %d\n",
           solid_pixels, wire_pixels);

    CHECK(solid_pixels > 0);
    CHECK(wire_pixels > 0);
    CHECK(wire_pixels < solid_pixels); // Wireframe outlines only
}

int main() {
    printf("=== Running ESR Face Culling & Wireframe Unit Tests ===\n");

    test_signed_area_and_winding();
    test_cull_mode_logic();
    test_process_triangle_culling();
    test_rasterize_line();
    test_polygon_modes();
    test_draw_mesh_culling();
    test_draw_mesh_wireframe();

    printf("====================================================\n");
    printf("Culling & Wireframe Tests: %d checks, %d failures\n", g_checks, g_failures);

    return (g_failures == 0) ? 0 : 1;
}
