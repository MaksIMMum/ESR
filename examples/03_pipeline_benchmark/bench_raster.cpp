/**
 * bench_raster.cpp - Benchmark: full-screen scan (bringup baseline) vs AABB + Pineda.
 *
 * Baseline = the original bringup rasterizer from main/main.c: visits all W*H
 * pixels and does 3 float divisions per pixel. On the ESP32-S3 it measured
 * ~32 ms per triangle at 160x120.
 *
 * Desktop timings are NOT ESP32 timings; use the speedup ratio as the signal,
 * and the "Render time" lines printed by the ESP32 firmware for absolute numbers.
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <chrono>
#include <vector>

#include "esr/esr_raster.hpp"

using namespace esr;

static const int W = 160;
static const int H = 120;
static uint8_t g_fb[H][W][3];

/* ---------------- Baseline: copy of the bringup rasterizer ---------------- */

static float edge(float ax, float ay, float bx, float by, float px, float py) {
    return (px - ax) * (by - ay) - (py - ay) * (bx - ax);
}

static int baseline_rasterize(const ScreenTriangle& tri) {
    const float vx[3] = {tri.v[0].x, tri.v[1].x, tri.v[2].x};
    const float vy[3] = {tri.v[0].y, tri.v[1].y, tri.v[2].y};
    const float col[3][3] = {
        {tri.v[0].color.x, tri.v[0].color.y, tri.v[0].color.z},
        {tri.v[1].color.x, tri.v[1].color.y, tri.v[1].color.z},
        {tri.v[2].color.x, tri.v[2].color.y, tri.v[2].color.z},
    };
    const float area = edge(vx[0], vy[0], vx[1], vy[1], vx[2], vy[2]);
    if (area == 0.0f) return 0;

    int written = 0;
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const float px = x + 0.5f, py = y + 0.5f;
            const float w0 = edge(vx[1], vy[1], vx[2], vy[2], px, py) / area;
            const float w1 = edge(vx[2], vy[2], vx[0], vy[0], px, py) / area;
            const float w2 = edge(vx[0], vy[0], vx[1], vy[1], px, py) / area;
            if (w0 >= 0.0f && w1 >= 0.0f && w2 >= 0.0f) {
                for (int c = 0; c < 3; ++c) {
                    const float v = w0 * col[0][c] + w1 * col[1][c] + w2 * col[2][c];
                    g_fb[y][x][c] = (uint8_t)(v * 255.0f);
                }
                ++written;
            }
        }
    }
    return written;
}

/* ---------------- Harness ---------------- */

static ScreenVertex sv(float x, float y, Vec3 c) {
    ScreenVertex v;
    v.x = x; v.y = y; v.z = 0.5f; v.inv_w = 1.0f; v.color = c;
    return v;
}

struct Result {
    double ms_per_frame;
    long pixels;
};

template <typename F>
static Result run(const std::vector<ScreenTriangle>& tris, F raster, double min_seconds) {
    using clock = std::chrono::steady_clock;
    long pixels = 0;
    long frames = 0;
    const auto t0 = clock::now();
    double elapsed = 0.0;
    do {
        memset(g_fb, 0, sizeof(g_fb));
        pixels = 0;
        for (const ScreenTriangle& t : tris) pixels += raster(t);
        ++frames;
        elapsed = std::chrono::duration<double>(clock::now() - t0).count();
    } while (elapsed < min_seconds);
    return Result{elapsed * 1000.0 / (double)frames, pixels};
}

static void bench(const char* name, const std::vector<ScreenTriangle>& tris) {
    Framebuffer fb{&g_fb[0][0][0], W, H};
    const Result base = run(tris, [](const ScreenTriangle& t) { return baseline_rasterize(t); }, 0.3);
    const Result fast = run(tris, [&fb](const ScreenTriangle& t) { return rasterize_triangle(t, fb); }, 0.3);

    printf("%-34s %5zu tri | baseline %9.4f ms (%6ld px) | AABB+Pineda %9.4f ms (%6ld px) | speedup %6.1fx\n",
           name, tris.size(), base.ms_per_frame, base.pixels, fast.ms_per_frame, fast.pixels,
           base.ms_per_frame / fast.ms_per_frame);
}

int main() {
    printf("ESR rasterizer benchmark, %dx%d RGB888, desktop host\n", W, H);
    printf("(baseline = full-screen scan from the ESP32 bringup, ~32 ms/tri on ESP32-S3)\n\n");

    const Vec3 R = vec3(1, 0, 0), G = vec3(0, 1, 0), B = vec3(0, 0, 1);

    // 1. The exact bringup triangle (main/main.c).
    bench("bringup triangle (large)", {ScreenTriangle{{sv(80, 8, R), sv(10, 110, G), sv(150, 110, B)}}});

    // 2. A small 16x16 px triangle: the case where AABB culling pays off most.
    bench("small triangle (16 px)", {ScreenTriangle{{sv(70, 50, R), sv(70, 66, G), sv(86, 58, B)}}});

    // 3. Mesh-like workload: screen tiled by 16x12 cells = 384 small triangles.
    std::vector<ScreenTriangle> mesh;
    const int cols = 16, rows = 12;
    for (int j = 0; j < rows; ++j) {
        for (int i = 0; i < cols; ++i) {
            const float x0 = (float)W * i / cols, x1 = (float)W * (i + 1) / cols;
            const float y0 = (float)H * j / rows, y1 = (float)H * (j + 1) / rows;
            mesh.push_back(ScreenTriangle{{sv(x0, y0, R), sv(x1, y0, G), sv(x1, y1, B)}});
            mesh.push_back(ScreenTriangle{{sv(x0, y0, R), sv(x1, y1, B), sv(x0, y1, G)}});
        }
    }
    bench("tiled mesh 16x12 (full screen)", mesh);

    printf("\nNote: pixel counts differ slightly on shared edges: the baseline draws pixels on\n"
           "every edge (overdraw), the new rasterizer applies the top-left fill rule.\n");
    return 0;
}
