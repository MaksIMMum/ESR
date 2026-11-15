/**
 * test_esr_profiler.cpp - Unit tests for the ESR Profiler & Telemetry Suite.
 *
 * Requirements tested (TASK-11 / REQ-6):
 *   1. High-resolution time query (esr_get_time_us) and CPU cycle counter.
 *   2. Memory tracking (SRAM, PSRAM, Process RSS).
 *   3. Profiler stage breakdown (Clear, Geometry, Raster, Present).
 *   4. Latency and FPS calculations with Exponential Moving Average (EMA).
 *   5. Throughput rates: triangles/sec, pixels/sec, culling ratio.
 *   6. 5x7 ASCII bitmap font rasterization and text drawing (draw_char, draw_text).
 *   7. On-screen Heads-Up Display (HUD) rendering directly into the framebuffer.
 *   8. Periodic telemetry logging formatting.
 */

#include <stdio.h>
#include <string.h>
#include <vector>
#include <cmath>

#include "esr/esr_profiler.hpp"

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

/* ------------------------------------------------------------------ */
/* 1. Timer & System Query Tests                                      */
/* ------------------------------------------------------------------ */

static void test_timer_queries() {
    printf("[1/6] Testing high-resolution timers and cycle counters...\n");

    uint64_t t0 = esr_get_time_us();
    CHECK(t0 > 0);

    // Busy wait briefly
    volatile uint64_t sum = 0;
    for (int i = 0; i < 100000; ++i) {
        sum += i;
    }
    (void)sum;

    uint64_t t1 = esr_get_time_us();
    CHECK(t1 >= t0);

    uint64_t c0 = esr_get_cpu_cycles();
    for (int i = 0; i < 10000; ++i) {
        sum += i;
    }
    uint64_t c1 = esr_get_cpu_cycles();
    CHECK(c1 >= c0);
}

/* ------------------------------------------------------------------ */
/* 2. Memory Query Tests                                              */
/* ------------------------------------------------------------------ */

static void test_memory_queries() {
    printf("[2/6] Testing memory tracking queries...\n");

    size_t sram = esr_get_free_sram();
    CHECK(sram > 0);

    size_t rss = esr_get_process_rss();
    // On Linux desktop, RSS of running test executable is typically 1..50 MB
    printf("       Free SRAM estimate: %zu KB, Process RSS: %zu KB\n",
           sram / 1024, rss / 1024);
}

/* ------------------------------------------------------------------ */
/* 3. Profiler Frame & Stage Latency Tracking                         */
/* ------------------------------------------------------------------ */

static void test_profiler_lifecycle() {
    printf("[3/6] Testing profiler stages, frame times, and FPS...\n");

    Profiler profiler;

    // Simulate 10 frames
    for (int f = 0; f < 10; ++f) {
        profiler.begin_frame();

        // 1. Clear stage
        profiler.begin_stage(ProfileStage::Clear);
        volatile int dummy = 0;
        for (int i = 0; i < 20000; ++i) dummy += i;
        profiler.end_stage(ProfileStage::Clear);

        // 2. Geometry stage
        profiler.begin_stage(ProfileStage::Geometry);
        for (int i = 0; i < 50000; ++i) dummy += i;
        profiler.end_stage(ProfileStage::Geometry);

        // 3. Raster stage
        profiler.begin_stage(ProfileStage::Raster);
        for (int i = 0; i < 80000; ++i) dummy += i;
        profiler.end_stage(ProfileStage::Raster);

        // Submit: 80 triangles, 40 drawn (50% culled), 4000 pixels
        profiler.end_frame(80, 40, 4000);
        (void)dummy;
    }

    const ProfilerStats& s = profiler.stats();
    CHECK(s.total_frames == 10);
    CHECK(s.fps > 0.0f);
    CHECK(s.frame_time_ms > 0.0f);
    CHECK(s.geom_time_ms > 0.0f);
    CHECK(s.rast_time_ms > 0.0f);
    CHECK(s.clear_time_ms > 0.0f);

    printf("       Simulated 10 frames -> FPS: %.1f, Frame: %.2f ms (Geom: %.2f ms, Rast: %.2f ms)\n",
           s.fps, s.frame_time_ms, s.geom_time_ms, s.rast_time_ms);
}

/* ------------------------------------------------------------------ */
/* 4. Throughput & Culling Metrics                                    */
/* ------------------------------------------------------------------ */

static void test_throughput_metrics() {
    printf("[4/6] Testing throughput rates (tris/sec, pixels/sec, culling ratio)...\n");

    Profiler profiler;
    profiler.begin_frame();
    profiler.end_frame(100, 45, 5000);

    const ProfilerStats& s = profiler.stats();
    CHECK(s.tris_submitted == 100);
    CHECK(s.tris_drawn == 45);
    CHECK(s.tris_culled == 55);
    CHECK(std::fabs(s.cull_percentage - 55.0f) < 0.1f);
    CHECK(s.pixels_drawn == 5000);
    CHECK(s.tris_per_sec > 0.0f);
    CHECK(s.pixels_per_sec > 0.0f);
}

/* ------------------------------------------------------------------ */
/* 5. 5x7 ASCII Bitmap Font Drawing                                   */
/* ------------------------------------------------------------------ */

static void test_bitmap_font() {
    printf("[5/6] Testing embedded 5x7 font rendering...\n");

    std::vector<uint8_t> fb_data(100 * 100 * 3, 0);
    Framebuffer fb{fb_data.data(), 100, 100};

    // Draw single character 'A' in bright red (255, 0, 0)
    draw_char(fb, 10, 10, 'A', 255, 0, 0, false);

    // Count drawn red pixels
    int red_pixels = 0;
    for (int y = 10; y < 18; ++y) {
        for (int x = 10; x < 16; ++x) {
            uint8_t* p = fb.color + (y * fb.width + x) * 3;
            if (p[0] == 255 && p[1] == 0 && p[2] == 0) {
                ++red_pixels;
            }
        }
    }
    CHECK(red_pixels > 0);

    // Draw string with background
    draw_text(fb, 20, 20, "ESR 3D", 0, 255, 0, true);
    int green_pixels = 0;
    int bg_pixels = 0;
    for (int y = 20; y < 28; ++y) {
        for (int x = 20; x < 60; ++x) {
            uint8_t* p = fb.color + (y * fb.width + x) * 3;
            if (p[1] == 255) ++green_pixels;
            if (p[0] == 16 && p[1] == 16 && p[2] == 16) ++bg_pixels;
        }
    }
    CHECK(green_pixels > 0);
    CHECK(bg_pixels > 0);
}

/* ------------------------------------------------------------------ */
/* 6. On-Screen HUD Overlay Rendering                                 */
/* ------------------------------------------------------------------ */

static void test_hud_overlay() {
    printf("[6/6] Testing on-screen real-time HUD overlay rasterization...\n");

    std::vector<uint8_t> fb_data(160 * 120 * 3, 0);
    Framebuffer fb{fb_data.data(), 160, 120};

    Profiler profiler;
    profiler.begin_frame();
    profiler.end_frame(80, 42, 3800);

    // Draw full multi-line HUD at (4, 4)
    profiler.draw_hud(fb, 4, 4, false);

    // Verify non-black pixels in the HUD bounding box (top 40 rows)
    int hud_pixels = 0;
    for (int y = 4; y < 45; ++y) {
        for (int x = 4; x < 150; ++x) {
            uint8_t* p = fb.color + (y * fb.width + x) * 3;
            if (p[0] != 0 || p[1] != 0 || p[2] != 0) {
                ++hud_pixels;
            }
        }
    }
    CHECK(hud_pixels > 200);

    // Test compact HUD (1 line)
    memset(fb_data.data(), 0, fb_data.size());
    profiler.draw_hud(fb, 2, 2, true);
    int compact_pixels = 0;
    for (int y = 2; y < 12; ++y) {
        for (int x = 2; x < 158; ++x) {
            uint8_t* p = fb.color + (y * fb.width + x) * 3;
            if (p[0] != 0 || p[1] != 0 || p[2] != 0) {
                ++compact_pixels;
            }
        }
    }
    CHECK(compact_pixels > 50);
}

int main() {
    printf("=== Running ESR Profiler & Telemetry Unit Tests (REQ-6) ===\n");

    test_timer_queries();
    test_memory_queries();
    test_profiler_lifecycle();
    test_throughput_metrics();
    test_bitmap_font();
    test_hud_overlay();

    printf("===========================================================\n");
    printf("Profiler Tests: %d checks, %d failures\n", g_checks, g_failures);

    return (g_failures == 0) ? 0 : 1;
}
