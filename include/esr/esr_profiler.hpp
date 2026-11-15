#ifndef ESR_PROFILER_HPP
#define ESR_PROFILER_HPP

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include "esr/esr_raster.hpp"
#include "esr/esr_font.hpp"

namespace esr {

/* ------------------------------------------------------------------ */
/* High-Resolution Time & Memory Query Functions                       */
/* ------------------------------------------------------------------ */

/**
 * Returns current monotonic time in microseconds (us).
 * On ESP32, backed by esp_timer_get_time() hardware timer.
 * On Desktop, backed by std::chrono::high_resolution_clock.
 */
uint64_t esr_get_time_us();

/**
 * Returns current CPU core cycles (nanosecond-grade precision).
 */
uint64_t esr_get_cpu_cycles();

/**
 * Returns free internal SRAM in bytes (crucial for ESP32).
 */
size_t esr_get_free_sram();

/**
 * Returns free external PSRAM in bytes (if equipped on ESP32, 0 otherwise).
 */
size_t esr_get_free_psram();

/**
 * Returns process Resident Set Size (RSS) physical RAM in bytes.
 */
size_t esr_get_process_rss();

/* ------------------------------------------------------------------ */
/* Profiler Breakdown Stages & Data                                   */
/* ------------------------------------------------------------------ */

enum class ProfileStage : uint8_t {
    Clear = 0,      // Buffer clearing (memset fb, clear_depth)
    Geometry = 1,   // Vertex transformations, near-plane clipping, perspective divide, culling
    Raster = 2,     // Pineda traversal, Early-Z, pixel shading, wireframe lines
    Present = 3,    // Display push (SPI DMA on ESP32, SDL2 blit on PC)
    Count = 4
};

struct ProfilerStats {
    // Smoothed timing metrics
    float fps = 0.0f;
    float frame_time_ms = 0.0f;
    float clear_time_ms = 0.0f;
    float geom_time_ms = 0.0f;
    float rast_time_ms = 0.0f;
    float present_time_ms = 0.0f;

    // Raw last frame timings (microseconds)
    uint32_t last_frame_time_us = 0;
    uint32_t last_clear_us = 0;
    uint32_t last_geom_us = 0;
    uint32_t last_rast_us = 0;
    uint32_t last_present_us = 0;

    // Throughput counters
    uint32_t tris_submitted = 0;
    uint32_t tris_drawn = 0;
    uint32_t tris_culled = 0;
    uint32_t pixels_drawn = 0;

    // Throughput rates
    float tris_per_sec = 0.0f;
    float pixels_per_sec = 0.0f;
    float cull_percentage = 0.0f;

    // Memory footprint
    size_t free_sram_bytes = 0;
    size_t free_psram_bytes = 0;
    size_t process_rss_bytes = 0;

    // Total frames profiled
    uint64_t total_frames = 0;
};

/* ------------------------------------------------------------------ */
/* Profiler Engine                                                     */
/* ------------------------------------------------------------------ */

class Profiler {
public:
    Profiler();

    /** Start profiling a new frame. */
    void begin_frame();

    /** Begin measuring a specific graphics pipeline stage. */
    void begin_stage(ProfileStage stage);

    /** End measuring a specific graphics pipeline stage. */
    void end_stage(ProfileStage stage);

    /**
     * Finish profiling current frame and calculate smoothed statistics.
     *
     * @param tris_submitted Total mesh triangles submitted before clipping/culling.
     * @param tris_drawn     Triangles actually passed to rasterizer.
     * @param pixels_drawn   Total pixels written to framebuffer.
     */
    void end_frame(uint32_t tris_submitted, uint32_t tris_drawn, uint32_t pixels_drawn);

    /** Retrieve latest computed stats. */
    const ProfilerStats& stats() const { return m_stats; }

    /**
     * Draw on-screen real-time Heads-Up Display (HUD) directly onto the framebuffer.
     *
     * @param fb       Target color framebuffer.
     * @param x        Top-left X pixel coordinate.
     * @param y        Top-left Y pixel coordinate.
     * @param compact  If true, shows a compact 1-line HUD (e.g. for small displays).
     */
    void draw_hud(Framebuffer& fb, int x = 4, int y = 4, bool compact = false) const;

    /**
     * Print formatted telemetry string to stdout or custom stream.
     */
    void print_telemetry(FILE* stream = stdout) const;

    /**
     * Print periodic telemetry (e.g. once every N frames).
     */
    void log_periodic(uint32_t frame_interval = 60, FILE* stream = stdout);

private:
    ProfilerStats m_stats;

    uint64_t m_frame_start_us = 0;
    uint64_t m_stage_start_us[static_cast<size_t>(ProfileStage::Count)] = {0};
    uint32_t m_stage_elapsed_us[static_cast<size_t>(ProfileStage::Count)] = {0};

    // History for exponential moving average (smoothing)
    float m_ema_frame_ms = 0.0f;
    float m_ema_clear_ms = 0.0f;
    float m_ema_geom_ms = 0.0f;
    float m_ema_rast_ms = 0.0f;
    float m_ema_present_ms = 0.0f;
};

} // namespace esr

#endif // ESR_PROFILER_HPP
