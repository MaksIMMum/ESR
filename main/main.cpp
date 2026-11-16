#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "esr/esr_math.h"
#include "esr/esr_pipeline.hpp"
#include "esr/esr_raster.hpp"
#include "esr/esr_depth.hpp"
#include "esr/esr_mesh.hpp"
#include "esr/esr_shader.hpp"
#include "esr/esr_profiler.hpp"
#include "esr/esr_font.hpp"

#include "models/car.h"

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "st7789.h"
#include "test_photo.h"
#else
#include <stdlib.h>
#include <time.h>
#endif

// ============================================================================
// Render Resolution Configuration
// ============================================================================
// Change this to test any resolution:
//   - 160x120 : ~94 KB  -> Fits automatically in Fast Internal SRAM (50-60 FPS)
//   - 240x240 : ~281 KB -> Automatically routes to 8MB Octal PSRAM (~22 FPS)
//   - 240x320 : ~375 KB -> Automatically routes to 8MB Octal PSRAM (~16 FPS)
// ============================================================================
#define RENDER_W 240
#define RENDER_H 320

static void clear_framebuffer(uint8_t* fb, int w, int h, uint8_t r, uint8_t g, uint8_t b)
{
    if (r == g && g == b) {
        memset(fb, r, (size_t)w * h * 3);
        return;
    }
    const size_t total_pixels = (size_t)w * h;
    for (size_t i = 0; i < total_pixels; ++i) {
        fb[i * 3 + 0] = r;
        fb[i * 3 + 1] = g;
        fb[i * 3 + 2] = b;
    }
}

extern "C" void app_main(void)
{
    printf("\n=======================================================\n");
    printf("       ESR 3D Real-Time Engine on ESP32-S3             \n");
    printf("       Display: ST7789 IPS 240x320                     \n");
    printf("       Configured Render Resolution: %dx%d             \n", RENDER_W, RENDER_H);
    printf("=======================================================\n\n");

#ifdef ESP_PLATFORM
    printf("Initializing ST7789 display controller (240x320)...\n");
    esp_err_t err = st7789_init();
    if (err == ESP_OK) {
        printf("Display initialized! Showing splash screen...\n");
        st7789_draw_bitmap(0, 0, PHOTO_WIDTH, PHOTO_HEIGHT, photo_data);
        vTaskDelay(pdMS_TO_TICKS(1500));

        // Clear display backdrop with dark slate (RGB565: 0x0842)
        st7789_fill_color(0x0842);
        printf("Starting real-time 3D rendering loop...\n");
    } else {
        printf("Warning: ST7789 display init failed: %s\n", esp_err_to_name(err));
    }
#endif

    const size_t needed_fb = (size_t)RENDER_W * RENDER_H * 3;
    const size_t needed_depth = (size_t)RENDER_W * RENDER_H * sizeof(uint16_t);
    const size_t total_needed = needed_fb + needed_depth;

    uint8_t* fb_data = nullptr;
    uint16_t* depth_data = nullptr;
    const char* ram_location = "Host RAM";

#ifdef ESP_PLATFORM
    size_t free_sram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t max_sram_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    printf("[ESR Memory Policy] System RAM Audit:\n");
    printf("  • Internal Fast SRAM: %zu KB free (Largest contiguous block: %zu KB)\n",
           free_sram / 1024, max_sram_block / 1024);
    printf("  • External Octal PSRAM: %zu KB free\n", free_psram / 1024);
    printf("  • Required for %dx%d: %zu KB total (Color: %zu KB, Depth: %zu KB)\n\n",
           RENDER_W, RENDER_H, total_needed / 1024, needed_fb / 1024, needed_depth / 1024);

    // Keep a 48 KB safety margin in Internal SRAM for FreeRTOS tasks and SPI DMA
    constexpr size_t SRAM_SAFETY_MARGIN = 48 * 1024;

    // Check 1: Does the resolution fit inside Fast Internal SRAM?
    if (needed_fb <= max_sram_block && (free_sram >= total_needed + SRAM_SAFETY_MARGIN)) {
        printf(">>> [POLICY] Resolution fits in FAST INTERNAL SRAM! Allocating on-chip...\n");
        fb_data = (uint8_t*)heap_caps_malloc(needed_fb, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        depth_data = (uint16_t*)heap_caps_malloc(needed_depth, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (fb_data && depth_data) {
            ram_location = "Internal SRAM";
            printf(">>> [POLICY SUCCESS] Buffers placed in ZERO-WAIT-STATE Internal SRAM (High FPS mode)!\n\n");
        } else {
            if (fb_data) { free(fb_data); fb_data = nullptr; }
            if (depth_data) { free(depth_data); depth_data = nullptr; }
        }
    }

    // Check 2: If too large for Internal SRAM, route to 8MB Octal PSRAM
    if (!fb_data || !depth_data) {
        if (free_psram >= total_needed + 64 * 1024) {
            printf(">>> [POLICY] Buffer exceeds Fast SRAM envelope. Routing to 8MB Octal PSRAM...\n");
            fb_data = (uint8_t*)heap_caps_malloc(needed_fb, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            depth_data = (uint16_t*)heap_caps_malloc(needed_depth, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (fb_data && depth_data) {
                ram_location = "Octal PSRAM";
                printf(">>> [POLICY SUCCESS] Buffers placed in 8MB Octal PSRAM (High Capacity mode)!\n\n");
            }
        }
    }
#else
    fb_data = (uint8_t*)malloc(needed_fb);
    depth_data = (uint16_t*)malloc(needed_depth);
    ram_location = "Host RAM";
#endif

    if (!fb_data || !depth_data) {
        printf("FATAL: Out of memory for rendering buffers!\n");
        return;
    }

    // Centering offsets (0, 0 when RENDER_W == 240 && RENDER_H == 320)
    const int dst_x = (240 - RENDER_W) / 2;
    const int dst_y = (320 - RENDER_H) / 2;

    using namespace esr;

    Framebuffer fb_target{fb_data, RENDER_W, RENDER_H};
    DepthBuffer depth_buf{depth_data, RENDER_W, RENDER_H};
    Profiler profiler;

    // Projection & View setup
    const float aspect = (float)RENDER_W / (float)RENDER_H;
    Mat4 proj = mat4_perspective(deg_to_rad(60.0f), aspect, 0.1f, 100.0f);
    Viewport vp = {0.0f, 0.0f, (float)RENDER_W, (float)RENDER_H, 0.0f, 1.0f};

    // Showcase: Neon Race Car only
    const float car_dist = (aspect < 1.0f) ? 3.0f : 2.5f;
    uint32_t frame_count = 0;
    float angle_x = deg_to_rad(18.0f); // Showroom elevation angle
    float angle_y = 0.0f;
    PolygonMode poly_mode = PolygonMode::Fill;

    while (1) {
        profiler.begin_frame();

        // Stage 1: Clear
        profiler.begin_stage(ProfileStage::Clear);
        clear_framebuffer(fb_data, RENDER_W, RENDER_H, 16, 16, 16); // Fast charcoal background
        clear_depth(depth_buf, 0xFFFF);
        profiler.end_stage(ProfileStage::Clear);

        // Smooth horizontal turntable orbit
        angle_y += deg_to_rad(2.0f);
        if (angle_y > 2.0f * 3.14159265f) angle_y -= 2.0f * 3.14159265f;

        // Camera View & Model matrix with rotation
        Mat4 view = mat4_look_at(vec3(0.0f, 0.0f, car_dist), vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f));
        Mat4 rot_y = mat4_rotate_y(angle_y);
        Mat4 rot_x = mat4_rotate_x(angle_x);
        Mat4 model = mat4_mul_mat4(rot_x, rot_y);
        Mat4 mvp   = mat4_mul_mat4(proj, mat4_mul_mat4(view, model));

        // Pipeline configuration: textured car with perspective-correct interpolation
        PipelineState state;
        state.cull_mode = (poly_mode == PolygonMode::Fill) ? CullMode::Back : CullMode::None;
        state.front_face = WindingOrder::CCW;
        state.polygon_mode = poly_mode;
        state.interp_mode = InterpMode::Perspective;
        state.vertex_shader = car_vertex_shader;
        state.pixel_shader = car_pixel_shader;
        esr_set_uniforms(state, &mvp);

        // Stage 2 & 3: Geometry & Raster (automatically measured by draw_mesh)
        int pixels_drawn = draw_mesh(CAR_MESH, mvp, vp, fb_target, &depth_buf, state, &profiler);

        uint32_t total_mesh_tris = CAR_MESH.index_count / 3;
        uint32_t tris_drawn = (state.cull_mode != CullMode::None) ? (total_mesh_tris / 2) : total_mesh_tris;

        // Stage 4: Real-time on-screen HUD overlay
        profiler.draw_hud(fb_target, 4, 4, false);

        // Resolution badge & RAM location at screen bottom
        char res_badge[48];
        snprintf(res_badge, sizeof(res_badge), "%dx%d [%s]",
                 RENDER_W, RENDER_H,
                 (strcmp(ram_location, "Internal SRAM") == 0) ? "SRAM" : "PSRAM");
        draw_text(fb_target, 4, RENDER_H - 10, res_badge, 180, 200, 230, true);

        // Stage 5: Present (Push to ST7789 display via high-speed SPI DMA)
        profiler.begin_stage(ProfileStage::Present);
#ifdef ESP_PLATFORM
        st7789_draw_rgb888_buffer(dst_x, dst_y, RENDER_W, RENDER_H, fb_data);
#endif
        profiler.end_stage(ProfileStage::Present);

        // Conclude frame metrics
        profiler.end_frame(total_mesh_tris, tris_drawn, static_cast<uint32_t>(pixels_drawn));

        // Print telemetry to serial monitor periodically every 30 frames (~1 sec)
        profiler.log_periodic(30, stdout);

        frame_count++;

#ifdef ESP_PLATFORM
        // Yield 1 tick to FreeRTOS idle task to feed watchdog timer
        vTaskDelay(pdMS_TO_TICKS(1));
#endif
    }

    free(fb_data);
    free(depth_data);
}
