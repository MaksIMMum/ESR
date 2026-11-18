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

// 3D Model Assets
#include "models/car.h"
#include "models/cube.h"
#include "models/icosphere.h"
#include "models/pyramid.h"

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "st7789.h"
#else
#include <stdlib.h>
#include <time.h>
#endif



// Model Selection: Choose which model to render on screen:
//   0 : Low-Poly Sports Car (148 tris, vertex-colored)
//   1 : Unit Cube (12 tris)
//   2 : Icosphere (80 tris)
//   3 : Pyramid (6 tris)
//  -1 : Auto-cycle through all models every few seconds
#define ACTIVE_MODEL_ID        0

// Render Resolution:
//   160x120 : ~94 KB  -> Fits inside Fast Internal SRAM (50-60 FPS)
//   240x240 : ~281 KB -> Auto-routes to 8MB Octal PSRAM (~22 FPS)
//   240x320 : ~375 KB -> Auto-routes to 8MB Octal PSRAM (~16 FPS)
#define RENDER_W               160
#define RENDER_H               120



//   1 : 16-bit Depth Buffer active (solid occlusion, Early-Z active)
//   0 : Disabled (back faces overdraw front faces)
#define ENABLE_DEPTH_TEST      1


//   1 : Culls back-facing triangles via 2D signed area (saves ~50% raster cost)
//   0 : Disabled (draws all triangles front and back)
#define ENABLE_BACKFACE_CULL   1


//   0 : Solid Fill (Subpixel 28.4 fixed-point Pineda integer rasterization)
//   1 : Wireframe Mode (Bresenham edges only)
#define WIREFRAME_MODE         0


//   0 : Flat (Provoking vertex color, faceted low-poly retro aesthetic)
//   1 : ScreenSpace (Fast 2D linear affine interpolation, no per-pixel divide)
//   2 : Perspective (Perspective-correct A/w interpolation, exact math)
#define INTERPOLATION_MODE     2


//   0 : Auto policy (SRAM for <=160x120, PSRAM for native 240x320)
//   1 : Force Internal Fast SRAM (Zero wait-state, highest FPS)
//   2 : Force 8MB Octal PSRAM (Benchmark SPI bus latency vs SRAM)
#define FORCE_RAM_MODE         0


#define SHOW_HUD               1


#define PAUSE_ROTATION         0


#define ROT_SPEED_Y            0.05f
#define ROT_SPEED_X            0.0f



struct ModelConfig {
    const esr::Mesh3D* mesh;
    const char*        name;
    float              camera_dist;
    esr::WindingOrder  winding;
};

static const ModelConfig MODELS[] = {
    {&CAR_MESH,       "Low-Poly Sports Car (148 tris)", 3.0f, esr::WindingOrder::CCW},
    {&CUBE_MESH,      "Unit Cube (12 tris)",            3.4f, esr::WindingOrder::CW},
    {&ICOSPHERE_MESH, "Icosphere (80 tris)",            2.8f, esr::WindingOrder::CCW},
    {&PYRAMID_MESH,   "Pyramid (6 tris)",               3.2f, esr::WindingOrder::CW},
};

static constexpr int NUM_MODELS = sizeof(MODELS) / sizeof(MODELS[0]);



static void clear_framebuffer(uint8_t* fb, int w, int h, uint8_t r, uint8_t g, uint8_t b) {
    if (r == g && g == b) {
        memset(fb, r, (size_t)w * h * 3);
        return;
    }
    const size_t total = (size_t)w * h;
    for (size_t i = 0; i < total; ++i) {
        fb[i * 3 + 0] = r;
        fb[i * 3 + 1] = g;
        fb[i * 3 + 2] = b;
    }
}

static bool allocate_render_buffers(int w, int h,
                                    uint8_t** out_fb,
                                    uint16_t** out_depth,
                                    const char** out_ram_loc) {
    const size_t fb_size = (size_t)w * h * 3;
    const size_t depth_size = (size_t)w * h * sizeof(uint16_t);
    const size_t total_size = fb_size + depth_size;

    *out_fb = nullptr;
    *out_depth = nullptr;
    *out_ram_loc = "Host RAM";

#ifdef ESP_PLATFORM
    size_t free_sram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t max_sram_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    printf("[ESR Memory Policy] System RAM Audit:\n");
    printf("  • Internal Fast SRAM: %zu KB free (Largest contiguous block: %zu KB)\n",
           free_sram / 1024, max_sram_block / 1024);
    printf("  • External Octal PSRAM: %zu KB free\n", free_psram / 1024);
    printf("  • Required for %dx%d: %zu KB total (Color: %zu KB, Depth: %zu KB)\n\n",
           w, h, total_size / 1024, fb_size / 1024, depth_size / 1024);

    // Keep 48 KB safety margin in Internal SRAM for FreeRTOS tasks & SPI DMA
    constexpr size_t SRAM_SAFETY_MARGIN = 48 * 1024;

    const bool allow_sram  = (FORCE_RAM_MODE != 2);
    const bool allow_psram = (FORCE_RAM_MODE != 1);

    // Check 1: Fast Internal SRAM
    if (allow_sram && (fb_size <= max_sram_block) && (free_sram >= total_size + SRAM_SAFETY_MARGIN)) {
        printf(">>> [POLICY] Allocating inside Fast On-Chip SRAM (Zero Wait-State)...\n");
        *out_fb = (uint8_t*)heap_caps_malloc(fb_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        *out_depth = (uint16_t*)heap_caps_malloc(depth_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (*out_fb && *out_depth) {
            *out_ram_loc = "Internal SRAM";
            printf(">>> [POLICY SUCCESS] Buffers placed in Fast Internal SRAM (High FPS mode)!\n\n");
            return true;
        }
        if (*out_fb) { free(*out_fb); *out_fb = nullptr; }
        if (*out_depth) { free(*out_depth); *out_depth = nullptr; }
    }

    // Check 2: External 8MB Octal PSRAM
    if (allow_psram && (free_psram >= total_size + 64 * 1024)) {
        printf(">>> [POLICY] Routing buffers to 8MB Octal PSRAM...\n");
        *out_fb = (uint8_t*)heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        *out_depth = (uint16_t*)heap_caps_malloc(depth_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (*out_fb && *out_depth) {
            *out_ram_loc = "Octal PSRAM";
            printf(">>> [POLICY SUCCESS] Buffers placed in 8MB Octal PSRAM!\n\n");
            return true;
        }
    }
    return false;
#else
    *out_fb = (uint8_t*)malloc(fb_size);
    *out_depth = (uint16_t*)malloc(depth_size);
    return (*out_fb && *out_depth);
#endif
}


extern "C" void app_main(void) {
    printf("\n=======================================================\n");
    printf("       Embedded Software Rasterizer (ESR) on ESP32-S3   \n");
    printf("       Display: ST7789 IPS (240x320)                   \n");
    printf("       Configured Render Resolution: %dx%d             \n", RENDER_W, RENDER_H);
    printf("=======================================================\n\n");

#ifdef ESP_PLATFORM
    printf("Initializing ST7789 display controller (240x320)...\n");
    esp_err_t err = st7789_init();
    if (err == ESP_OK) {
        // Clear physical display backdrop with dark slate (RGB565: 0x0842)
        st7789_fill_color(0x0842);
        printf("Display initialized! Starting real-time 3D pipeline...\n");
    } else {
        printf("Warning: ST7789 display init failed: %s\n", esp_err_to_name(err));
    }
#endif

    // Allocate frame and depth buffers according to memory policy
    uint8_t* fb_data = nullptr;
    uint16_t* depth_data = nullptr;
    const char* ram_location = "Host RAM";

    if (!allocate_render_buffers(RENDER_W, RENDER_H, &fb_data, &depth_data, &ram_location)) {
        printf("FATAL: Out of memory for rendering buffers!\n");
        return;
    }

    // Centering offsets on physical 240x320 panel
    const int dst_x = (240 - RENDER_W) / 2;
    const int dst_y = (320 - RENDER_H) / 2;

    using namespace esr;

    Framebuffer fb_target{fb_data, RENDER_W, RENDER_H};
    DepthBuffer depth_buf{depth_data, RENDER_W, RENDER_H};
    Profiler profiler;

    // Viewport & Projection Setup
    const float aspect = (float)RENDER_W / (float)RENDER_H;
    Mat4 proj = mat4_perspective(deg_to_rad(60.0f), aspect, 0.1f, 100.0f);
    Viewport vp = {0.0f, 0.0f, (float)RENDER_W, (float)RENDER_H, 0.0f, 1.0f};

    // Active model configuration
    int current_model_id = (ACTIVE_MODEL_ID >= 0 && ACTIVE_MODEL_ID < NUM_MODELS) ? ACTIVE_MODEL_ID : 0;
    const bool auto_cycle = (ACTIVE_MODEL_ID < 0);

    PolygonMode poly_mode = (WIREFRAME_MODE != 0) ? PolygonMode::Wireframe : PolygonMode::Fill;

    InterpMode interp_mode = InterpMode::Perspective;
#if INTERPOLATION_MODE == 0
    interp_mode = InterpMode::Flat;
#elif INTERPOLATION_MODE == 1
    interp_mode = InterpMode::ScreenSpace;
#endif

    float angle_x = deg_to_rad(20.0f);
    float angle_y = 0.0f;
    uint32_t frame_count = 0;

    printf("[ESR Config] Active Model: #%d (%s)\n", current_model_id, MODELS[current_model_id].name);
    printf("             Resolution: %dx%d [%s]\n", RENDER_W, RENDER_H, ram_location);
    printf("             DepthTest: %s | BackfaceCull: %s\n",
           ENABLE_DEPTH_TEST ? "ON" : "OFF (Painter's Glitch)",
           ENABLE_BACKFACE_CULL ? "ON" : "OFF");
    printf("             Mode: %s | Interp: %s\n\n",
           WIREFRAME_MODE ? "Wireframe" : "Solid Fill",
           (INTERPOLATION_MODE == 0) ? "Flat" : (INTERPOLATION_MODE == 1 ? "ScreenSpace (Affine)" : "Perspective-Correct"));

    if (auto_cycle) {
        printf("[ESR] Auto-cycle mode ENABLED: cycles model every 180 frames (~3-5s)\n");
    }

    while (1) {
        profiler.begin_frame();
        const ModelConfig& active_model = MODELS[current_model_id];


        profiler.begin_stage(ProfileStage::Clear);
        clear_framebuffer(fb_data, RENDER_W, RENDER_H, 16, 16, 16); // Fast charcoal background
        if (ENABLE_DEPTH_TEST) {
            clear_depth(depth_buf, 0xFFFF); // Far plane in 16-bit depth
        }
        profiler.end_stage(ProfileStage::Clear);

#if !PAUSE_ROTATION
        angle_y += ROT_SPEED_Y;
        angle_x += ROT_SPEED_X;
        if (angle_y > 2.0f * 3.14159265f) angle_y -= 2.0f * 3.14159265f;
        if (angle_x > 2.0f * 3.14159265f) angle_x -= 2.0f * 3.14159265f;
#endif

        // Dynamic camera distance adjusted for portrait vs landscape aspect ratio
        float cam_dist = active_model.camera_dist * (aspect < 1.0f ? 1.25f : 1.0f);
        Mat4 view  = mat4_look_at(vec3(0.0f, 0.0f, cam_dist), vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f));
        Mat4 model = mat4_mul_mat4(mat4_rotate_x(angle_x), mat4_rotate_y(angle_y));
        Mat4 mvp   = mat4_mul_mat4(proj, mat4_mul_mat4(view, model));

        // Pipeline state: Backface culling + Winding convention + Interpolation
        PipelineState state;
        state.cull_mode = (ENABLE_BACKFACE_CULL != 0) ? CullMode::Back : CullMode::None;
        state.front_face = active_model.winding;
        state.polygon_mode = poly_mode;
        state.interp_mode = interp_mode;


        DepthBuffer* active_db = (ENABLE_DEPTH_TEST != 0) ? &depth_buf : nullptr;
        int pixels_drawn = draw_mesh(*active_model.mesh, mvp, vp, fb_target, active_db, state, &profiler);

        uint32_t total_tris = active_model.mesh->index_count / 3;
        uint32_t tris_drawn = (state.cull_mode != CullMode::None) ? (total_tris / 2) : total_tris;

#if SHOW_HUD
        profiler.draw_hud(fb_target, 4, 4, false);

        // Status badge: Resolution, RAM domain, and Model ID
        char badge[64];
        snprintf(badge, sizeof(badge), "%dx%d [%s] #%d",
                 RENDER_W, RENDER_H,
                 (strcmp(ram_location, "Internal SRAM") == 0) ? "SRAM" : "PSRAM",
                 current_model_id);
        draw_text(fb_target, 4, RENDER_H - 10, badge, 180, 200, 230, true);
#endif

        profiler.begin_stage(ProfileStage::Present);
#ifdef ESP_PLATFORM
        st7789_draw_rgb888_buffer(dst_x, dst_y, RENDER_W, RENDER_H, fb_data);
#endif
        profiler.end_stage(ProfileStage::Present);

        // Finalize frame timings
        profiler.end_frame(total_tris, tris_drawn, static_cast<uint32_t>(pixels_drawn));

        // Print telemetry to serial monitor periodically every 30 frames (~1 sec)
        profiler.log_periodic(30, stdout);

        frame_count++;

        // Handle auto-cycle if enabled
        if (auto_cycle && (frame_count % 180 == 0)) {
            current_model_id = (current_model_id + 1) % NUM_MODELS;
            printf("\n[ESR] Auto-switched to Model #%d: %s\n",
                   current_model_id, MODELS[current_model_id].name);
        }

#ifdef ESP_PLATFORM
        // Yield 1 tick to FreeRTOS idle task to feed watchdog timer
        vTaskDelay(1);
#endif
    }

    free(fb_data);
    free(depth_data);
}
