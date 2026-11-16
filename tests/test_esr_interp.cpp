/**
 * test_esr_interp.cpp - Unit tests for the 3 attribute interpolation modes.
 *
 * Requirements tested (TASK-07 / REQ-3):
 *   1. Flat: every fragment receives the provoking vertex (v0) attributes,
 *      for both the fixed-point color path and the pixel-shader path, for
 *      either winding, in wireframe, and through near-plane clipping.
 *   2. Screen-space: varyings equal the affine 2D barycentric combination.
 *   3. Perspective-correct: varyings equal (sum b_i A_i / w_i) / (sum b_i / w_i)
 *      for triangles and lines; identical to screen-space when w is constant;
 *      graceful fallback when inv_w is missing.
 *   4. Test scene: steep-angled checkerboard floor rendered through the full
 *      pipeline. Perspective mode matches an analytic ray-cast of the texture,
 *      screen-space mode shows the classic affine distortion.
 */

#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>

#include "esr/esr_shader.hpp"
#include "esr/esr_pipeline.hpp"
#include "esr/esr_raster.hpp"
#include "esr/esr_depth.hpp"
#include "esr/esr_mesh.hpp"

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

struct Image {
    int w, h;
    std::vector<uint8_t> data;
    Image(int w_ = W, int h_ = H) : w(w_), h(h_), data(static_cast<size_t>(w_) * h_ * 3, 0) {}
    Framebuffer fb() { return Framebuffer{data.data(), w, h}; }
    const uint8_t* px(int x, int y) const { return &data[(static_cast<size_t>(y) * w + x) * 3]; }
    bool lit(int x, int y) const { const uint8_t* p = px(x, y); return p[0] | p[1] | p[2]; }
};

/* ------------------------------------------------------------------ */
/* Varying capture: the pixel shader records every fragment's varyings  */
/* in invocation order (row-major over covered pixels).                */
/* ------------------------------------------------------------------ */

struct Capture {
    std::vector<std::vector<float>> frags;
    uint8_t count = 0;
};

static uint16_t capture_ps(const float* varyings, const void* uniforms) {
    Capture* cap = const_cast<Capture*>(static_cast<const Capture*>(uniforms));
    cap->frags.emplace_back(varyings, varyings + cap->count);
    return 0xFFFF; // white, so coverage is visible in the framebuffer
}

static ScreenVertex sv(float x, float y, float inv_w, float r, float g, float b,
                       float a0 = 0.0f, float a1 = 0.0f) {
    ScreenVertex v;
    v.x = x; v.y = y; v.z = 0.5f; v.inv_w = inv_w;
    v.color = {r, g, b};
    v.varyings[0] = a0;
    v.varyings[1] = a1;
    v.varying_count = 2;
    return v;
}

static ScreenTriangle tri(const ScreenVertex& a, const ScreenVertex& b, const ScreenVertex& c) {
    ScreenTriangle t;
    t.v[0] = a; t.v[1] = b; t.v[2] = c;
    return t;
}

/** Barycentric weights of point p relative to the (float) triangle. */
static void barycentric(const ScreenTriangle& t, float px, float py, float b[3]) {
    const float x0 = t.v[0].x, y0 = t.v[0].y;
    const float x1 = t.v[1].x, y1 = t.v[1].y;
    const float x2 = t.v[2].x, y2 = t.v[2].y;
    const float area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
    b[0] = ((x1 - px) * (y2 - py) - (y1 - py) * (x2 - px)) / area;
    b[1] = ((x2 - px) * (y0 - py) - (y2 - py) * (x0 - px)) / area;
    b[2] = 1.0f - b[0] - b[1];
}

/** Render with capture_ps and pair each captured fragment with its pixel. */
struct Sample { int x, y; std::vector<float> v; };

static std::vector<Sample> capture_triangle(const ScreenTriangle& t, InterpMode mode) {
    Image img;
    Framebuffer fb = img.fb();
    Capture cap;
    cap.count = t.v[0].varying_count;
    rasterize_triangle(t, fb, nullptr, DepthFunc::Less, PolygonMode::Fill, capture_ps, &cap, mode);

    std::vector<Sample> out;
    size_t i = 0;
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            if (img.lit(x, y) && i < cap.frags.size()) {
                out.push_back({x, y, cap.frags[i++]});
            }
        }
    }
    CHECK(i == cap.frags.size());
    return out;
}

/* ------------------------------------------------------------------ */
/* 1. Flat                                                              */
/* ------------------------------------------------------------------ */

static void test_flat_fixed_point_path() {
    // Vertex positions on the 1/16 sub-pixel grid; v0 = red is the provoking vertex.
    const ScreenTriangle t = tri(sv(20, 10, 1, 1, 0, 0), sv(140, 30, 1, 0, 1, 0), sv(60, 110, 1, 0, 0, 1));
    for (int winding = 0; winding < 2; ++winding) {
        const ScreenTriangle tt = winding ? tri(t.v[0], t.v[2], t.v[1]) : t; // v0 stays v0
        Image img;
        Framebuffer fb = img.fb();
        const int n = rasterize_triangle(tt, fb, nullptr, DepthFunc::Less, PolygonMode::Fill,
                                         nullptr, nullptr, InterpMode::Flat);
        CHECK(n > 1000);
        int wrong = 0;
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x)
                if (img.lit(x, y)) {
                    const uint8_t* p = img.px(x, y);
                    if (!(p[0] == 255 && p[1] == 0 && p[2] == 0)) ++wrong;
                }
        CHECK(wrong == 0);
    }
}

static void test_flat_shader_path() {
    const ScreenTriangle t = tri(sv(20, 10, 1.0f, 1, 0, 0, 0.25f, 0.75f),
                                 sv(140, 30, 0.5f, 0, 1, 0, 1.0f, 0.0f),
                                 sv(60, 110, 0.1f, 0, 0, 1, 0.0f, 1.0f));
    const auto samples = capture_triangle(t, InterpMode::Flat);
    CHECK(samples.size() > 1000);
    int wrong = 0;
    for (const auto& s : samples) {
        if (s.v[0] != 0.25f || s.v[1] != 0.75f) ++wrong;
    }
    CHECK(wrong == 0);
}

static void test_flat_wireframe() {
    const ScreenTriangle t = tri(sv(20, 10, 1, 1, 0, 0), sv(140, 30, 1, 0, 1, 0), sv(60, 110, 1, 0, 0, 1));
    Image img;
    Framebuffer fb = img.fb();
    const int n = rasterize_triangle(t, fb, nullptr, DepthFunc::Less, PolygonMode::Wireframe,
                                     nullptr, nullptr, InterpMode::Flat);
    CHECK(n > 100);
    int wrong = 0;
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            if (img.lit(x, y)) {
                const uint8_t* p = img.px(x, y);
                if (!(p[0] == 255 && p[1] == 0 && p[2] == 0)) ++wrong;
            }
    CHECK(wrong == 0);
}

static void test_flat_survives_near_clipping() {
    // v0 (red) is behind the camera: the clipper creates new vertices, but every
    // output triangle must still carry the original provoking vertex attributes.
    const Mat4 proj = mat4_perspective(deg_to_rad(60.0f), (float)W / H, 0.1f, 100.0f);
    const Viewport vp = {0.0f, 0.0f, (float)W, (float)H, 0.0f, 1.0f};
    const Vertex3D v0 = {{0.0f, -0.5f, 1.0f}, {1.0f, 0.0f, 0.0f}};
    const Vertex3D v1 = {{-1.0f, -0.5f, -3.0f}, {0.0f, 1.0f, 0.0f}};
    const Vertex3D v2 = {{1.0f, -0.5f, -3.0f}, {0.0f, 0.0f, 1.0f}};

    ScreenTriangle out[ESR_MAX_CLIPPED_TRIANGLES];
    const int n = process_triangle(v0, v1, v2, proj, vp, out, CullMode::None, WindingOrder::CCW,
                                   InterpMode::Flat);
    CHECK(n == 2);
    for (int t = 0; t < n; ++t)
        for (int i = 0; i < 3; ++i) {
            CHECK(out[t].v[i].color.x == 1.0f && out[t].v[i].color.y == 0.0f && out[t].v[i].color.z == 0.0f);
            CHECK(out[t].v[i].varyings[0] == 1.0f && out[t].v[i].varyings[1] == 0.0f);
        }

    // Same input in screen-space mode keeps interpolated (non-red) clip vertices.
    const int n2 = process_triangle(v0, v1, v2, proj, vp, out, CullMode::None, WindingOrder::CCW,
                                    InterpMode::ScreenSpace);
    CHECK(n2 == 2);
    CHECK(out[0].v[0].color.x < 1.0f);
}

/* ------------------------------------------------------------------ */
/* 2. Screen-space & 3. Perspective-correct vs. analytic references    */
/* ------------------------------------------------------------------ */

// A triangle whose bottom edge is near (w = 1) and apex far (w = 10).
static ScreenTriangle steep_triangle() {
    return tri(sv(80, 8, 0.1f, 1, 0, 0, 0.0f, 0.0f),    // far apex
               sv(8, 112, 1.0f, 0, 1, 0, 1.0f, 0.0f),   // near left
               sv(152, 112, 1.0f, 0, 0, 1, 0.0f, 1.0f)); // near right
}

static void test_screen_space_matches_affine() {
    const ScreenTriangle t = steep_triangle();
    const auto samples = capture_triangle(t, InterpMode::ScreenSpace);
    CHECK(samples.size() > 3000);
    float max_err = 0.0f;
    for (const auto& s : samples) {
        float b[3];
        barycentric(t, s.x + 0.5f, s.y + 0.5f, b);
        for (int k = 0; k < 2; ++k) {
            const float ref = b[0] * t.v[0].varyings[k] + b[1] * t.v[1].varyings[k] + b[2] * t.v[2].varyings[k];
            max_err = std::fmax(max_err, std::fabs(ref - s.v[k]));
        }
    }
    printf("  screen-space max |err| vs affine reference      = %.2e\n", max_err);
    CHECK(max_err < 1e-3f);
}

static void test_perspective_matches_hyperbolic() {
    const ScreenTriangle t = steep_triangle();
    const auto persp = capture_triangle(t, InterpMode::Perspective);
    const auto affine = capture_triangle(t, InterpMode::ScreenSpace);
    CHECK(persp.size() == affine.size());

    float max_err = 0.0f;
    float max_affine_dev = 0.0f;
    for (size_t i = 0; i < persp.size(); ++i) {
        const auto& s = persp[i];
        float b[3];
        barycentric(t, s.x + 0.5f, s.y + 0.5f, b);
        float q = 0.0f;
        for (int j = 0; j < 3; ++j) q += b[j] * t.v[j].inv_w;
        for (int k = 0; k < 2; ++k) {
            float aq = 0.0f;
            for (int j = 0; j < 3; ++j) aq += b[j] * t.v[j].varyings[k] * t.v[j].inv_w;
            const float ref = aq / q;
            max_err = std::fmax(max_err, std::fabs(ref - s.v[k]));
            max_affine_dev = std::fmax(max_affine_dev, std::fabs(affine[i].v[k] - s.v[k]));
        }
    }
    printf("  perspective  max |err| vs (A/w)/(1/w) reference = %.2e\n", max_err);
    printf("  perspective vs screen-space max deviation       = %.3f\n", max_affine_dev);
    CHECK(max_err < 1e-3f);
    CHECK(max_affine_dev > 0.25f); // Depth range 1..10 -> large affine distortion.
}

static void test_perspective_equals_affine_when_w_constant() {
    ScreenTriangle t = steep_triangle();
    for (int i = 0; i < 3; ++i) t.v[i].inv_w = 0.4f;
    const auto persp = capture_triangle(t, InterpMode::Perspective);
    const auto affine = capture_triangle(t, InterpMode::ScreenSpace);
    CHECK(persp.size() == affine.size());
    float max_dev = 0.0f;
    for (size_t i = 0; i < persp.size(); ++i)
        for (int k = 0; k < 2; ++k)
            max_dev = std::fmax(max_dev, std::fabs(persp[i].v[k] - affine[i].v[k]));
    CHECK(max_dev < 1e-4f);
}

static void test_perspective_fallback_without_inv_w() {
    ScreenTriangle t = steep_triangle();
    for (int i = 0; i < 3; ++i) t.v[i].inv_w = 0.0f; // hand-built triangle, no 1/w
    const auto persp = capture_triangle(t, InterpMode::Perspective);
    const auto affine = capture_triangle(t, InterpMode::ScreenSpace);
    CHECK(persp.size() == affine.size());
    bool all_finite = true;
    float max_dev = 0.0f;
    for (size_t i = 0; i < persp.size(); ++i)
        for (int k = 0; k < 2; ++k) {
            all_finite = all_finite && std::isfinite(persp[i].v[k]);
            max_dev = std::fmax(max_dev, std::fabs(persp[i].v[k] - affine[i].v[k]));
        }
    CHECK(all_finite);
    CHECK(max_dev == 0.0f);
}

static void test_perspective_fixed_color_path() {
    // No pixel shader: color ramp red(far) -> green(near). Perspective-correct
    // color should stay red-dominant much further toward the near edge.
    const ScreenTriangle t = tri(sv(80, 8, 0.1f, 1, 0, 0), sv(8, 112, 1.0f, 0, 1, 0), sv(152, 112, 1.0f, 0, 1, 0));
    Image a, p;
    Framebuffer fa = a.fb(), fp = p.fb();
    rasterize_triangle(t, fa, nullptr, DepthFunc::Less, PolygonMode::Fill, nullptr, nullptr, InterpMode::ScreenSpace);
    rasterize_triangle(t, fp, nullptr, DepthFunc::Less, PolygonMode::Fill, nullptr, nullptr, InterpMode::Perspective);
    // Pixel at screen-space parameter s = 0.5 between apex and base (y ~ 60).
    const int x = 80, y = 60;
    float b[3];
    barycentric(t, x + 0.5f, y + 0.5f, b);
    const float q = b[0] * 0.1f + (b[1] + b[2]) * 1.0f;
    const float red_ref = b[0] * 0.1f / q;
    CHECK(std::abs(a.px(x, y)[0] - (int)(b[0] * 255.0f + 0.5f)) <= 1);
    CHECK(std::abs(p.px(x, y)[0] - (int)(red_ref * 255.0f + 0.5f)) <= 1);
    CHECK(a.px(x, y)[0] > p.px(x, y)[0] + 60);
}

static void test_perspective_line() {
    // Line from near (w = 1, a = 0) to far (w = 10, a = 1), horizontal 100 px.
    ScreenVertex v0 = sv(10, 50, 1.0f, 0, 0, 0, 0.0f, 0.0f);
    ScreenVertex v1 = sv(110, 50, 0.1f, 0, 0, 0, 1.0f, 0.0f);
    Image img;
    Framebuffer fb = img.fb();
    Capture cap;
    cap.count = 2;
    const int n = rasterize_line(v0, v1, fb, nullptr, DepthFunc::Less, capture_ps, &cap, InterpMode::Perspective);
    CHECK(n == 101);
    CHECK((int)cap.frags.size() == n);
    float max_err = 0.0f;
    for (int i = 0; i < n; ++i) {
        const float s = i / 100.0f;
        const float ref = (s * 0.1f) / ((1.0f - s) * 1.0f + s * 0.1f);
        max_err = std::fmax(max_err, std::fabs(cap.frags[i][0] - ref));
    }
    CHECK(max_err < 1e-4f);
    // Midpoint: screen-space would give 0.5, perspective gives 1/11.
    CHECK(std::fabs(cap.frags[50][0] - 1.0f / 11.0f) < 1e-4f);

    Capture flat;
    flat.count = 2;
    rasterize_line(v1, v0, fb, nullptr, DepthFunc::Less, capture_ps, &flat, InterpMode::Flat);
    bool all_v0 = true;
    for (const auto& f : flat.frags) all_v0 = all_v0 && f[0] == 1.0f;
    CHECK(all_v0);
}

/* ------------------------------------------------------------------ */
/* 4. Test scene: steep checkerboard floor through the full pipeline   */
/* ------------------------------------------------------------------ */

struct PlaneVertex { Vec3 pos; float u, v; };
struct PlaneUniforms { Mat4 mvp; };

static const int CHECKS = 8; // 8x8 checkerboard over uv [0,1]^2

static void plane_vs(const void* in, esr_vertex_out_t* out, const void* uniforms) {
    const auto* v = static_cast<const PlaneVertex*>(in);
    const auto* u = static_cast<const PlaneUniforms*>(uniforms);
    out->clip_pos = mat4_mul_vec4(u->mvp, vec4_from_vec3(v->pos, 1.0f));
    out->varyings.data[0] = v->u;
    out->varyings.data[1] = v->v;
    out->varyings.count = 2;
}

static int checker_cell(float u, float v) {
    return ((int)std::floor(u * CHECKS) + (int)std::floor(v * CHECKS)) & 1;
}

static uint16_t checker_ps(const float* varyings, const void*) {
    return checker_cell(varyings[0], varyings[1]) ? esr_pack_rgb565(1, 1, 1)
                                                 : esr_pack_rgb565(1, 0, 0);
}

// Floor y = -1, x in [-2, 2], z in [-1.5, -20]; camera at origin looking down -Z.
static const float PX0 = -2.0f, PX1 = 2.0f, PZ0 = -1.5f, PZ1 = -20.0f, PY = -1.0f;
static const float FOV = 60.0f;

/** Analytic ray-plane intersection -> uv at a screen point (false if it misses). */
static bool ground_truth_uv(float sx, float sy, float& u, float& v) {
    const float tan_half = std::tan(deg_to_rad(FOV) * 0.5f);
    const float nx = sx / W * 2.0f - 1.0f;
    const float ny = 1.0f - sy / H * 2.0f;
    const Vec3 d = {nx * tan_half * ((float)W / H), ny * tan_half, -1.0f};
    if (d.y >= 0.0f) return false;
    const float t = PY / d.y;
    const float X = d.x * t, Z = d.z * t;
    u = (X - PX0) / (PX1 - PX0);
    v = (Z - PZ0) / (PZ1 - PZ0);
    return u >= 0.0f && u <= 1.0f && v >= 0.0f && v <= 1.0f;
}

/** Fraction of unambiguous plane pixels whose checker cell disagrees with the ray cast. */
static float render_plane_mismatch(InterpMode mode, int* tested_out) {
    const PlaneVertex verts[4] = {
        {{PX0, PY, PZ0}, 0.0f, 0.0f}, {{PX1, PY, PZ0}, 1.0f, 0.0f},
        {{PX1, PY, PZ1}, 1.0f, 1.0f}, {{PX0, PY, PZ1}, 0.0f, 1.0f},
    };
    PlaneUniforms uni;
    uni.mvp = mat4_perspective(deg_to_rad(FOV), (float)W / H, 0.1f, 100.0f);
    const Viewport vp = {0.0f, 0.0f, (float)W, (float)H, 0.0f, 1.0f};

    Image img;
    Framebuffer fb = img.fb();
    const int idx[6] = {0, 1, 2, 0, 2, 3};
    for (int i = 0; i < 6; i += 3) {
        ScreenTriangle st[ESR_MAX_CLIPPED_TRIANGLES];
        const int n = process_triangle(&verts[idx[i]], &verts[idx[i + 1]], &verts[idx[i + 2]],
                                       plane_vs, &uni, vp, st, CullMode::None, WindingOrder::CCW, mode);
        for (int t = 0; t < n; ++t) {
            rasterize_triangle(st[t], fb, nullptr, DepthFunc::Less, PolygonMode::Fill, checker_ps, &uni, mode);
        }
    }

    int tested = 0, wrong = 0;
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            if (!img.lit(x, y)) continue;
            // Skip pixels whose footprint straddles a checker boundary (ambiguous).
            float u, v;
            int cells[5];
            bool ok = ground_truth_uv(x + 0.5f, y + 0.5f, u, v);
            cells[0] = ok ? checker_cell(u, v) : -1;
            const float off[4][2] = {{0, 0}, {1, 0}, {0, 1}, {1, 1}};
            for (int c = 0; c < 4 && ok; ++c) {
                ok = ground_truth_uv(x + off[c][0], y + off[c][1], u, v);
                cells[c + 1] = ok ? checker_cell(u, v) : -1;
                ok = ok && cells[c + 1] == cells[0];
            }
            if (!ok) continue;
            ++tested;
            const bool white = img.px(x, y)[1] > 128;
            if (white != (cells[0] == 1)) ++wrong;
        }
    }
    *tested_out = tested;
    return tested ? (float)wrong / tested : 1.0f;
}

static void test_scene_steep_checkerboard() {
    int tested_p = 0, tested_s = 0;
    const float persp = render_plane_mismatch(InterpMode::Perspective, &tested_p);
    const float screen = render_plane_mismatch(InterpMode::ScreenSpace, &tested_s);
    printf("  steep checkerboard: perspective mismatch = %.2f%% of %d px, "
           "screen-space mismatch = %.2f%% of %d px\n",
           persp * 100.0f, tested_p, screen * 100.0f, tested_s);
    CHECK(tested_p > 2000);
    CHECK(persp < 0.002f);   // distortion eliminated
    CHECK(screen > 0.10f);   // affine warping clearly visible
}

int main() {
    printf("=== ESR attribute interpolation tests (REQ-3) ===\n");
    test_flat_fixed_point_path();
    test_flat_shader_path();
    test_flat_wireframe();
    test_flat_survives_near_clipping();
    test_screen_space_matches_affine();
    test_perspective_matches_hyperbolic();
    test_perspective_equals_affine_when_w_constant();
    test_perspective_fallback_without_inv_w();
    test_perspective_fixed_color_path();
    test_perspective_line();
    test_scene_steep_checkerboard();

    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
