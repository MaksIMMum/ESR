#include "esr/esr_profiler.hpp"

#include <stdio.h>
#include <string.h>
#include <cmath>

#ifdef ESP_PLATFORM
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <xtensa/hal.h>
#else
#include <chrono>
#include <unistd.h>
#endif

namespace esr {

/* ------------------------------------------------------------------ */
/* System Query Functions                                             */
/* ------------------------------------------------------------------ */

uint64_t esr_get_time_us() {
#ifdef ESP_PLATFORM
    return static_cast<uint64_t>(esp_timer_get_time());
#else
    using namespace std::chrono;
    return duration_cast<microseconds>(high_resolution_clock::now().time_since_epoch()).count();
#endif
}

uint64_t esr_get_cpu_cycles() {
#ifdef ESP_PLATFORM
    return xthal_get_ccount();
#elif defined(__x86_64__) || defined(_M_X64)
    return __builtin_ia32_rdtsc();
#else
    return esr_get_time_us() * 240; // 240MHz approximation
#endif
}

size_t esr_get_free_sram() {
#ifdef ESP_PLATFORM
    return heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
#else
    // On Linux desktop, simulate free memory or return remaining of 512KB embedded envelope
    size_t rss = esr_get_process_rss();
    constexpr size_t EMBEDDED_SRAM_TOTAL = 512 * 1024;
    return (rss < EMBEDDED_SRAM_TOTAL) ? (EMBEDDED_SRAM_TOTAL - rss) : (256 * 1024);
#endif
}

size_t esr_get_free_psram() {
#ifdef ESP_PLATFORM
    return heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
#else
    return 0;
#endif
}

size_t esr_get_process_rss() {
#ifdef ESP_PLATFORM
    return 0;
#else
    FILE* f = fopen("/proc/self/statm", "r");
    if (!f) return 0;
    long pages = 0;
    long resident = 0;
    if (fscanf(f, "%ld %ld", &pages, &resident) == 2) {
        fclose(f);
        long page_size = sysconf(_SC_PAGESIZE);
        return static_cast<size_t>(resident) * (page_size > 0 ? page_size : 4096);
    }
    fclose(f);
    return 0;
#endif
}

/* ------------------------------------------------------------------ */
/* Profiler Implementation                                            */
/* ------------------------------------------------------------------ */

Profiler::Profiler() : m_stats{} {
}

void Profiler::begin_frame() {
    m_frame_start_us = esr_get_time_us();
    for (size_t i = 0; i < static_cast<size_t>(ProfileStage::Count); ++i) {
        m_stage_elapsed_us[i] = 0;
    }
}

void Profiler::begin_stage(ProfileStage stage) {
    size_t idx = static_cast<size_t>(stage);
    if (idx < static_cast<size_t>(ProfileStage::Count)) {
        m_stage_start_us[idx] = esr_get_time_us();
    }
}

void Profiler::end_stage(ProfileStage stage) {
    size_t idx = static_cast<size_t>(stage);
    if (idx < static_cast<size_t>(ProfileStage::Count)) {
        if (m_stage_start_us[idx] > 0) {
            uint64_t now = esr_get_time_us();
            if (now >= m_stage_start_us[idx]) {
                m_stage_elapsed_us[idx] += static_cast<uint32_t>(now - m_stage_start_us[idx]);
            }
            m_stage_start_us[idx] = 0;
        }
    }
}

void Profiler::end_frame(uint32_t tris_submitted, uint32_t tris_drawn, uint32_t pixels_drawn) {
    uint64_t now = esr_get_time_us();
    uint32_t frame_us = (now >= m_frame_start_us) ? static_cast<uint32_t>(now - m_frame_start_us) : 1;
    if (frame_us == 0) frame_us = 1;

    m_stats.last_frame_time_us = frame_us;
    m_stats.last_clear_us = m_stage_elapsed_us[static_cast<size_t>(ProfileStage::Clear)];
    m_stats.last_geom_us  = m_stage_elapsed_us[static_cast<size_t>(ProfileStage::Geometry)];
    m_stats.last_rast_us  = m_stage_elapsed_us[static_cast<size_t>(ProfileStage::Raster)];
    m_stats.last_present_us = m_stage_elapsed_us[static_cast<size_t>(ProfileStage::Present)];

    float cur_frame_ms = frame_us / 1000.0f;
    float cur_clear_ms = m_stats.last_clear_us / 1000.0f;
    float cur_geom_ms  = m_stats.last_geom_us / 1000.0f;
    float cur_rast_ms  = m_stats.last_rast_us / 1000.0f;
    float cur_present_ms = m_stats.last_present_us / 1000.0f;

    // Exponential Moving Average (alpha = 0.15 for smooth HUD digits without lag)
    constexpr float ALPHA = 0.15f;
    if (m_stats.total_frames == 0) {
        m_ema_frame_ms = cur_frame_ms;
        m_ema_clear_ms = cur_clear_ms;
        m_ema_geom_ms  = cur_geom_ms;
        m_ema_rast_ms  = cur_rast_ms;
        m_ema_present_ms = cur_present_ms;
    } else {
        m_ema_frame_ms = (1.0f - ALPHA) * m_ema_frame_ms + ALPHA * cur_frame_ms;
        m_ema_clear_ms = (1.0f - ALPHA) * m_ema_clear_ms + ALPHA * cur_clear_ms;
        m_ema_geom_ms  = (1.0f - ALPHA) * m_ema_geom_ms  + ALPHA * cur_geom_ms;
        m_ema_rast_ms  = (1.0f - ALPHA) * m_ema_rast_ms  + ALPHA * cur_rast_ms;
        m_ema_present_ms = (1.0f - ALPHA) * m_ema_present_ms + ALPHA * cur_present_ms;
    }

    m_stats.frame_time_ms = m_ema_frame_ms;
    m_stats.clear_time_ms = m_ema_clear_ms;
    m_stats.geom_time_ms  = m_ema_geom_ms;
    m_stats.rast_time_ms  = m_ema_rast_ms;
    m_stats.present_time_ms = m_ema_present_ms;

    m_stats.fps = (m_ema_frame_ms > 0.0001f) ? (1000.0f / m_ema_frame_ms) : 0.0f;

    // Throughput
    m_stats.tris_submitted = tris_submitted;
    m_stats.tris_drawn     = tris_drawn;
    m_stats.tris_culled    = (tris_submitted >= tris_drawn) ? (tris_submitted - tris_drawn) : 0;
    m_stats.pixels_drawn   = pixels_drawn;

    m_stats.cull_percentage = (tris_submitted > 0)
        ? (100.0f * static_cast<float>(m_stats.tris_culled) / static_cast<float>(tris_submitted))
        : 0.0f;

    m_stats.tris_per_sec   = m_stats.tris_drawn * m_stats.fps;
    m_stats.pixels_per_sec = m_stats.pixels_drawn * m_stats.fps;

    // Memory footprint
    m_stats.free_sram_bytes   = esr_get_free_sram();
    m_stats.free_psram_bytes  = esr_get_free_psram();
    m_stats.process_rss_bytes = esr_get_process_rss();

    m_stats.total_frames++;
}

void Profiler::draw_hud(Framebuffer& fb, int x, int y, bool compact) const {
    if (fb.color == nullptr || fb.width <= 0 || fb.height <= 0) return;

    char line[64];
    int cur_y = y;

    if (compact) {
        snprintf(line, sizeof(line), "FPS:%4.1f %4.1fms SRAM:%dK",
                 m_stats.fps, m_stats.frame_time_ms,
                 static_cast<int>(m_stats.free_sram_bytes / 1024));
        draw_text(fb, x, cur_y, line, 50, 255, 50, true);
        return;
    }

    // Line 1: FPS and Frame Latency (Bright Lime Green)
    snprintf(line, sizeof(line), "FPS:%4.1f (%4.1fms)",
             m_stats.fps, m_stats.frame_time_ms);
    draw_text(fb, x, cur_y, line, 80, 255, 80, true);
    cur_y += 9;

    // Line 2: Geometry & Raster Breakdown (Cyan)
    snprintf(line, sizeof(line), "G:%3.1fms R:%3.1fms",
             m_stats.geom_time_ms, m_stats.rast_time_ms);
    draw_text(fb, x, cur_y, line, 80, 220, 255, true);
    cur_y += 9;

    // Line 3: Triangle stats and Culling % (Warm Yellow)
    snprintf(line, sizeof(line), "Tri:%d/%d (%2.0f%%c)",
             m_stats.tris_drawn, m_stats.tris_submitted, m_stats.cull_percentage);
    draw_text(fb, x, cur_y, line, 255, 230, 70, true);
    cur_y += 9;

    // Line 4: Memory footprint (Soft Magenta)
    snprintf(line, sizeof(line), "SRAM:%3dK Px:%d",
             static_cast<int>(m_stats.free_sram_bytes / 1024), m_stats.pixels_drawn);
    draw_text(fb, x, cur_y, line, 255, 160, 255, true);
}

void Profiler::print_telemetry(FILE* stream) const {
    if (!stream) stream = stdout;
    fprintf(stream,
            "[ESR Profiler] Frame %4llu | FPS: %5.1f | Frame: %5.2f ms | "
            "Geom: %4.2f ms | Rast: %4.2f ms | Tris: %3d/%3d (culled %2.0f%%) | "
            "Fill: %5d px (%5.1f kpx/s) | Free SRAM: %3zu KB\n",
            static_cast<unsigned long long>(m_stats.total_frames),
            m_stats.fps,
            m_stats.frame_time_ms,
            m_stats.geom_time_ms,
            m_stats.rast_time_ms,
            m_stats.tris_drawn,
            m_stats.tris_submitted,
            m_stats.cull_percentage,
            m_stats.pixels_drawn,
            m_stats.pixels_per_sec / 1000.0f,
            m_stats.free_sram_bytes / 1024);
}

void Profiler::log_periodic(uint32_t frame_interval, FILE* stream) {
    if (frame_interval > 0 && (m_stats.total_frames % frame_interval == 0)) {
        print_telemetry(stream);
    }
}

} // namespace esr
