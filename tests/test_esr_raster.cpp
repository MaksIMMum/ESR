/**
 * test_esr_raster.cpp - Unit tests for the AABB + Pineda triangle rasterizer.
 */
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>

#include "esr/esr_raster.hpp"

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

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */

struct Image {
    int w, h;
    std::vector<uint8_t> px;
    Image(int w_, int h_) : w(w_), h(h_), px((size_t)w_ * h_ * 3, 0) {}
    Framebuffer fb() { return Framebuffer{px.data(), w, h}; }
    void clear() { memset(px.data(), 0, px.size()); }
    bool covered(int x, int y) const {
        const uint8_t* p = &px[((size_t)y * w + x) * 3];
        return (p[0] | p[1] | p[2]) != 0;
    }
    const uint8_t* at(int x, int y) const { return &px[((size_t)y * w + x) * 3]; }
};

static ScreenVertex sv(float x, float y, Vec3 c = vec3(1.0f, 1.0f, 1.0f)) {
    ScreenVertex v;
    v.x = x;
    v.y = y;
    v.z = 0.5f;
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

static int count_covered(const Image& img) {
    int n = 0;
    for (int y = 0; y < img.h; ++y)
        for (int x = 0; x < img.w; ++x)
            n += img.covered(x, y) ? 1 : 0;
    return n;
}

/** Deterministic LCG so the tests are reproducible on every platform. */
static uint32_t g_rng = 12345u;
static float rnd01() {
    g_rng = g_rng * 1664525u + 1013904223u;
    return (float)(g_rng >> 8) / 16777216.0f;
}

/** Snap to the 1/16 sub-pixel grid so the reference test sees exactly what the rasterizer sees. */
static float snap(float v) { return std::floor(v * 16.0f + 0.5f) / 16.0f; }

/** Old full-screen bringup rasterizer (float barycentrics, inclusive on every edge). */
/* Evaluated in double so ties on 1/16-snapped inputs are exact. */
static double edge_ref(double ax, double ay, double bx, double by, double px, double py) {
    return (px - ax) * (by - ay) - (py - ay) * (bx - ax);
}

/**
 * Tile the whole screen with a jittered grid of triangles (random diagonal,
 * random winding) and count how many triangles cover each pixel.
 * A correct fill rule gives exactly 1 everywhere: no gaps, no overdraw.
 */
static void check_tiling_exactly_once(int cols, int rows, float jitter, float offset) {
    std::vector<float> gx((cols + 1) * (rows + 1)), gy((cols + 1) * (rows + 1));
    for (int j = 0; j <= rows; ++j) {
        for (int i = 0; i <= cols; ++i) {
            float x = (float)W * i / cols;
            float y = (float)H * j / rows;
            const bool border_x = (i == 0 || i == cols);
            const bool border_y = (j == 0 || j == rows);
            if (!border_x) x = snap(x + offset + (rnd01() - 0.5f) * jitter);
            if (!border_y) y = snap(y + offset + (rnd01() - 0.5f) * jitter);
            gx[j * (cols + 1) + i] = x;
            gy[j * (cols + 1) + i] = y;
        }
    }

    std::vector<int> hits(W * H, 0);
    Image img(W, H);
    Framebuffer fb = img.fb();
    int total_written = 0;

    for (int j = 0; j < rows; ++j) {
        for (int i = 0; i < cols; ++i) {
            const int k00 = j * (cols + 1) + i, k10 = k00 + 1;
            const int k01 = k00 + (cols + 1), k11 = k01 + 1;
            ScreenVertex a = sv(gx[k00], gy[k00]), b = sv(gx[k10], gy[k10]);
            ScreenVertex c = sv(gx[k01], gy[k01]), d = sv(gx[k11], gy[k11]);

            ScreenTriangle t[2];
            if (rnd01() < 0.5f) {
                t[0] = tri(a, b, d);
                t[1] = tri(a, d, c);
            } else {
                t[0] = tri(a, b, c);
                t[1] = tri(b, d, c);
            }
            for (int n = 0; n < 2; ++n) {
                if (rnd01() < 0.5f) { // flip winding
                    ScreenVertex tmp = t[n].v[1];
                    t[n].v[1] = t[n].v[2];
                    t[n].v[2] = tmp;
                }
                img.clear();
                const int written = rasterize_triangle(t[n], fb);
                CHECK(written == count_covered(img));
                total_written += written;
                for (int y = 0; y < H; ++y)
                    for (int x = 0; x < W; ++x)
                        if (img.covered(x, y)) ++hits[y * W + x];
            }
        }
    }

    int gaps = 0, overdraw = 0;
    for (int h : hits) {
        if (h == 0) ++gaps;
        if (h > 1) ++overdraw;
    }
    if (gaps || overdraw) {
        printf("  tiling %dx%d jitter=%.2f offset=%.2f: gaps=%d overdraw=%d\n",
               cols, rows, jitter, offset, gaps, overdraw);
    }
    CHECK(gaps == 0);
    CHECK(overdraw == 0);
    CHECK(total_written == W * H);
}

/* ------------------------------------------------------------------ */
/* Test Cases                                                           */
/* ------------------------------------------------------------------ */

/** Grid lines pass exactly through pixel centers: every edge test hits E == 0. */
static void test_top_left_rule_axis_aligned_on_centers() {
    check_tiling_exactly_once(8, 6, 0.0f, 0.5f);
    check_tiling_exactly_once(16, 12, 0.0f, 0.0f);
}

/** Jittered sub-pixel vertices: arbitrary slopes and shared diagonal edges. */
static void test_top_left_rule_jittered_mesh() {
    for (int run = 0; run < 8; ++run) {
        check_tiling_exactly_once(10, 8, 10.0f, 0.0f);
    }
}

/** Fan of triangles around one shared vertex placed exactly on a pixel center. */
static void test_top_left_rule_fan() {
    const float cx = 80.5f, cy = 60.5f;
    const int N = 16;
    std::vector<int> hits(W * H, 0);
    Image img(W, H);
    Framebuffer fb = img.fb();
    for (int k = 0; k < N; ++k) {
        const float a0 = 2.0f * ESR_PI * k / N, a1 = 2.0f * ESR_PI * (k + 1) / N;
        ScreenTriangle t = tri(sv(cx, cy),
                               sv(snap(cx + 50.0f * std::cos(a0)), snap(cy + 50.0f * std::sin(a0))),
                               sv(snap(cx + 50.0f * std::cos(a1)), snap(cy + 50.0f * std::sin(a1))));
        img.clear();
        rasterize_triangle(t, fb);
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x)
                if (img.covered(x, y)) ++hits[y * W + x];
    }
    int overdraw = 0;
    for (int h : hits) overdraw += h > 1 ? 1 : 0;
    CHECK(overdraw == 0);
    CHECK(hits[60 * W + 80] == 1);       // The shared center pixel is drawn exactly once.
    CHECK(hits[60 * W + 100] == 1);      // Interior of the fan has no holes.
    CHECK(hits[45 * W + 70] == 1);
}

/** New rasterizer must agree with the old full-screen scan except on exact-edge ties. */
static void test_matches_baseline_coverage() {
    Image img(W, H);
    Framebuffer fb = img.fb();
    for (int n = 0; n < 200; ++n) {
        float vx[3], vy[3];
        for (int i = 0; i < 3; ++i) {
            vx[i] = snap(rnd01() * 200.0f - 20.0f);
            vy[i] = snap(rnd01() * 160.0f - 20.0f);
        }
        img.clear();
        rasterize_triangle(tri(sv(vx[0], vy[0]), sv(vx[1], vy[1]), sv(vx[2], vy[2])), fb);

        const double area = edge_ref(vx[0], vy[0], vx[1], vy[1], vx[2], vy[2]);
        int mismatches = 0;
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                const double px = x + 0.5, py = y + 0.5;
                const double e0 = edge_ref(vx[1], vy[1], vx[2], vy[2], px, py);
                const double e1 = edge_ref(vx[2], vy[2], vx[0], vy[0], px, py);
                const double e2 = edge_ref(vx[0], vy[0], vx[1], vy[1], px, py);
                if (area == 0.0) {
                    mismatches += img.covered(x, y) ? 1 : 0;
                    continue;
                }
                const double s = area > 0 ? 1.0 : -1.0;
                const bool strictly_in = e0 * s > 0 && e1 * s > 0 && e2 * s > 0;
                const bool in_or_on = e0 * s >= 0 && e1 * s >= 0 && e2 * s >= 0;
                const bool got = img.covered(x, y);
                if (strictly_in && !got) ++mismatches;  // gap
                if (!in_or_on && got) ++mismatches;     // drew outside
            }
        }
        CHECK(mismatches == 0);
    }
}

static void test_winding_independent() {
    Image a(W, H), b(W, H);
    Framebuffer fa = a.fb(), fbb = b.fb();
    const ScreenVertex v0 = sv(80.3f, 7.9f, vec3(1, 0, 0));
    const ScreenVertex v1 = sv(10.2f, 110.0f, vec3(0, 1, 0));
    const ScreenVertex v2 = sv(150.6f, 109.4f, vec3(0, 0, 1));
    const int na = rasterize_triangle(tri(v0, v1, v2), fa);
    const int nb = rasterize_triangle(tri(v0, v2, v1), fbb);
    CHECK(na > 0);
    CHECK(na == nb);
    CHECK(a.px == b.px);
}

static void test_color_interpolation() {
    Image img(W, H);
    Framebuffer fb = img.fb();
    const ScreenVertex v0 = sv(80.0f, 8.0f, vec3(1, 0, 0));
    const ScreenVertex v1 = sv(10.0f, 110.0f, vec3(0, 1, 0));
    const ScreenVertex v2 = sv(150.0f, 110.0f, vec3(0, 0, 1));
    rasterize_triangle(tri(v0, v1, v2), fb);

    // Compare every pixel against exact barycentric interpolation.
    const double area = edge_ref(v0.x, v0.y, v1.x, v1.y, v2.x, v2.y);
    int max_err = 0;
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            if (!img.covered(x, y)) continue;
            const double px = x + 0.5, py = y + 0.5;
            const double w[3] = {edge_ref(v1.x, v1.y, v2.x, v2.y, px, py) / area,
                                edge_ref(v2.x, v2.y, v0.x, v0.y, px, py) / area,
                                edge_ref(v0.x, v0.y, v1.x, v1.y, px, py) / area};
            for (int ch = 0; ch < 3; ++ch) {
                const int expect = (int)std::lround(std::fmin(1.0, std::fmax(0.0, w[ch])) * 255.0);
                const int err = std::abs(expect - (int)img.at(x, y)[ch]);
                if (err > max_err) max_err = err;
            }
        }
    }
    CHECK(max_err <= 1);

    // Near the red apex the pixel is (almost) pure red.
    const uint8_t* top = img.at(80, 10);
    CHECK(top[0] > 230 && top[1] < 25 && top[2] < 25);
}

static void test_aabb_clamped_to_framebuffer() {
    // Same triangle, partly off-screen, rendered into the real FB and into a
    // larger FB with a 100 px margin. The visible window must match exactly.
    const float ax = -40.25f, ay = -30.0f, bx = 210.5f, by = 50.0f, cx = 60.0f, cy = 170.75f;
    Image small(W, H), big(W + 200, H + 200);
    Framebuffer fs = small.fb(), fbig = big.fb();
    const int ns = rasterize_triangle(tri(sv(ax, ay), sv(bx, by), sv(cx, cy)), fs);
    rasterize_triangle(tri(sv(ax + 100, ay + 100), sv(bx + 100, by + 100), sv(cx + 100, cy + 100)), fbig);

    int diff = 0;
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            diff += small.covered(x, y) != big.covered(x + 100, y + 100) ? 1 : 0;
    CHECK(diff == 0);
    CHECK(ns == count_covered(small));
}

static void test_rejects() {
    Image img(W, H);
    Framebuffer fb = img.fb();
    // Fully off-screen on every side.
    CHECK(rasterize_triangle(tri(sv(-50, 10), sv(-10, 10), sv(-30, 50)), fb) == 0);
    CHECK(rasterize_triangle(tri(sv(170, 10), sv(200, 10), sv(180, 50)), fb) == 0);
    CHECK(rasterize_triangle(tri(sv(10, -50), sv(50, -50), sv(30, -10)), fb) == 0);
    CHECK(rasterize_triangle(tri(sv(10, 130), sv(50, 130), sv(30, 170)), fb) == 0);
    // Degenerate: collinear and repeated vertices.
    CHECK(rasterize_triangle(tri(sv(10, 10), sv(50, 50), sv(90, 90)), fb) == 0);
    CHECK(rasterize_triangle(tri(sv(10, 10), sv(10, 10), sv(90, 20)), fb) == 0);
    // Tiny triangle that contains no pixel center.
    CHECK(rasterize_triangle(tri(sv(10.6f, 10.6f), sv(10.9f, 10.6f), sv(10.6f, 10.9f)), fb) == 0);
    // NaN / Inf.
    CHECK(rasterize_triangle(tri(sv(NAN, 10), sv(50, 10), sv(30, 50)), fb) == 0);
    CHECK(rasterize_triangle(tri(sv(INFINITY, 10), sv(50, 10), sv(30, 50)), fb) == 0);
    // Empty framebuffer.
    Framebuffer empty;
    CHECK(rasterize_triangle(tri(sv(0, 0), sv(100, 0), sv(0, 100)), empty) == 0);
    CHECK(count_covered(img) == 0);
}

/** Vertices far outside the screen force the 64-bit edge path; must still cover everything. */
static void test_huge_triangle_64bit_path() {
    Image img(W, H);
    Framebuffer fb = img.fb();
    const int n = rasterize_triangle(tri(sv(-1.0e6f, -1.0e6f), sv(3.0e6f, -1.0e6f), sv(-1.0e6f, 3.0e6f)), fb);
    CHECK(n == W * H);
    CHECK(count_covered(img) == W * H);

    // Beyond the guard band: rejected instead of overflowing.
    img.clear();
    CHECK(rasterize_triangle(tri(sv(-1.0e8f, -1.0e8f), sv(1.0e8f, 0), sv(0, 1.0e8f)), fb) == 0);
}

/**
 * Pins the convention itself: a square whose edges run exactly through pixel
 * centers keeps its top row and left column and drops the bottom row and right
 * column. (A consistent "bottom-right" rule would also be gap-free, so the tiling
 * tests alone cannot tell the two apart.)
 */
static void test_top_left_convention() {
    Image img(W, H);
    Framebuffer fb = img.fb();
    const ScreenVertex tl = sv(10.5f, 10.5f), tr = sv(20.5f, 10.5f);
    const ScreenVertex bl = sv(10.5f, 20.5f), br = sv(20.5f, 20.5f);
    rasterize_triangle(tri(tl, tr, br), fb);
    rasterize_triangle(tri(tl, br, bl), fb);
    int wrong = 0;
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            wrong += img.covered(x, y) != (x >= 10 && x < 20 && y >= 10 && y < 20) ? 1 : 0;
    CHECK(wrong == 0);
    CHECK(count_covered(img) == 100);
}

/** Single pixel: a small triangle around one pixel center covers exactly that pixel. */
static void test_single_pixel() {
    Image img(W, H);
    Framebuffer fb = img.fb();
    CHECK(rasterize_triangle(tri(sv(20.5f, 29.8f), sv(21.3f, 31.0f), sv(19.7f, 31.0f)), fb) == 1);
    CHECK(img.covered(20, 30));
}

int main() {
    printf("Running esr_raster tests...\n");

    test_top_left_rule_axis_aligned_on_centers();
    test_top_left_rule_jittered_mesh();
    test_top_left_rule_fan();
    test_matches_baseline_coverage();
    test_winding_independent();
    test_color_interpolation();
    test_aabb_clamped_to_framebuffer();
    test_rejects();
    test_huge_triangle_64bit_path();
    test_top_left_convention();
    test_single_pixel();

    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
