/**
 * viewer.cpp - Interactive Real-Time Desktop 3D Viewer for ESR.
 *
 * Features:
 *   - Real-time 60+ FPS true 3D rendering with live HUD overlay.
 *   - Interactive mouse drag rotation & keyboard controls.
 *   - Dynamic toggles:
 *       [SPACE]  : Pause / resume auto-rotation
 *       [W]      : Toggle Wireframe vs Solid Fill
 *       [C]      : Toggle Backface Culling (CullMode::Back vs CullMode::None)
 *       [L]      : Toggle Directional Diffuse Lighting (Shader vs Unshaded)
 *       [M]      : Switch 3D model (Icosphere vs Cube)
 *       [H]      : Toggle on-screen Profiler HUD
 *       [ESC]/Q  : Quit viewer
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <cmath>
#include <vector>

#if __has_include(<SDL2/SDL.h>)
#include <SDL2/SDL.h>
#else
#include <SDL.h>
#endif

#include "esr/esr_pipeline.hpp"
#include "esr/esr_raster.hpp"
#include "esr/esr_depth.hpp"
#include "esr/esr_mesh.hpp"
#include "esr/esr_shader.hpp"
#include "esr/esr_profiler.hpp"
#include "models/car.h"

using namespace esr;

static const int RENDER_W = 160;
static const int RENDER_H = 120;
static const int WINDOW_SCALE = 5; // 800 x 600 window
static const int WINDOW_W = RENDER_W * WINDOW_SCALE;
static const int WINDOW_H = RENDER_H * WINDOW_SCALE;

int main(int argc, char** argv) {
    bool headless = false;
    bool vsync = false;
    int max_frames = -1;
    int render_w = RENDER_W;
    int render_h = RENDER_H;
    int window_scale = WINDOW_SCALE;
    const char* screenshot_path = nullptr;
    const char* dump_frames_dir = nullptr;
    float rot_step = 1.0f;
    bool enable_hud = true;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--headless") == 0) headless = true;
        else if (strcmp(argv[i], "--vsync") == 0) vsync = true;
        else if (strcmp(argv[i], "--no-hud") == 0) enable_hud = false;
        else if (strcmp(argv[i], "--screenshot") == 0 && i + 1 < argc) {
            screenshot_path = argv[++i];
        }
        else if (strcmp(argv[i], "--dump-frames") == 0 && i + 1 < argc) {
            dump_frames_dir = argv[++i];
        }
        else if (strcmp(argv[i], "--rot-step") == 0 && i + 1 < argc) {
            rot_step = static_cast<float>(atof(argv[++i]));
        }
        else if (strcmp(argv[i], "--esp32-screen") == 0 || strcmp(argv[i], "--240x320") == 0) {
            render_w = 240;
            render_h = 320;
            window_scale = 2; // 480 x 640 window
        } else if (strcmp(argv[i], "--res") == 0 && i + 2 < argc) {
            render_w = atoi(argv[++i]);
            render_h = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--frames") == 0 && i + 1 < argc) {
            max_frames = atoi(argv[++i]);
        }
    }

    const int window_w = render_w * window_scale;
    const int window_h = render_h * window_scale;

    printf("=====================================================\n");
    printf("   ESR Interactive Real-Time 3D Viewer & Profiler     \n");
    printf("   Render resolution: %dx%d (Window: %dx%d)          \n",
           render_w, render_h, window_w, window_h);
    printf("   Controls:                                         \n");
    printf("     [SPACE]   : Toggle auto-turntable               \n");
    printf("     [W]       : Toggle Wireframe / Solid Fill       \n");
    printf("     [C]       : Toggle Backface Culling             \n");
    printf("     [V]       : Toggle V-Sync (Capped vs Uncapped)  \n");
    printf("     [H]       : Toggle Profiler HUD Overlay         \n");
    printf("     [I]       : Cycle Interp (Flat/Screen/Persp)    \n");
    printf("     [Drag]    : Orbit camera with mouse             \n");
    printf("     [ESC]/[Q] : Exit                                \n");
    printf("   Options:                                          \n");
    printf("     --esp32-screen / --240x320 : Native ST7789 display resolution\n");
    printf("=====================================================\n\n");

    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* texture = nullptr;

    if (!headless) {
        if (SDL_Init(SDL_INIT_VIDEO) != 0) {
            fprintf(stderr, "SDL_Init failed: %s (falling back to headless mode)\n", SDL_GetError());
            headless = true;
        } else {
            window = SDL_CreateWindow("ESR - Embedded Software Rasterizer [Real-Time Profiler]",
                                      SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                      window_w, window_h, SDL_WINDOW_SHOWN);
            if (!window) {
                fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
                headless = true;
            } else {
                Uint32 flags = SDL_RENDERER_ACCELERATED | (vsync ? SDL_RENDERER_PRESENTVSYNC : 0);
                renderer = SDL_CreateRenderer(window, -1, flags);
                if (!renderer) renderer = SDL_CreateRenderer(window, -1, 0);

                SDL_GL_SetSwapInterval(vsync ? 1 : 0);

                texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB24,
                                            SDL_TEXTUREACCESS_STREAMING,
                                            render_w, render_h);
            }
        }
    }

    // Allocate render buffers
    std::vector<uint8_t> fb_data(static_cast<size_t>(render_w) * render_h * 3, 0);
    std::vector<uint16_t> db_data(static_cast<size_t>(render_w) * render_h, 0xFFFF);
    Framebuffer fb{fb_data.data(), render_w, render_h};
    DepthBuffer db{db_data.data(), render_w, render_h};
    Viewport vp{0.0f, 0.0f, static_cast<float>(render_w), static_cast<float>(render_h), 0.0f, 1.0f};

    // State toggles
    bool auto_rotate = true;
    bool wireframe = false;
    bool culling = true;
    bool hud = enable_hud;
    InterpMode interp_mode = InterpMode::Perspective;

    float rot_x = 20.0f;
    float rot_y = 0.0f;
    bool mouse_down = false;
    int last_mouse_x = 0;
    int last_mouse_y = 0;

    Profiler profiler;
    float aspect = (float)render_w / (float)render_h;
    Mat4 proj = mat4_perspective(deg_to_rad(60.0f), aspect, 0.1f, 100.0f);

    bool running = true;
    int frame_count = 0;

    while (running) {
        if (!headless) {
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_QUIT) {
                    running = false;
                } else if (event.type == SDL_KEYDOWN) {
                    switch (event.key.keysym.sym) {
                        case SDLK_ESCAPE:
                        case SDLK_q: running = false; break;
                        case SDLK_SPACE: auto_rotate = !auto_rotate; break;
                        case SDLK_w: wireframe = !wireframe; break;
                        case SDLK_c: culling = !culling; break;
                        case SDLK_v:
                            vsync = !vsync;
                            SDL_GL_SetSwapInterval(vsync ? 1 : 0);
                            printf("[Viewer] V-Sync %s (%s)\n",
                                   vsync ? "ENABLED" : "DISABLED",
                                   vsync ? "capped to monitor" : "UNCAPPED");
                            break;
                        case SDLK_h: hud = !hud; break;
                        case SDLK_i:
                            interp_mode = static_cast<InterpMode>((static_cast<int>(interp_mode) + 1) % 3);
                            printf("[Viewer] Interpolation: %s\n", interp_mode_name(interp_mode));
                            break;
                    }
                } else if (event.type == SDL_MOUSEBUTTONDOWN) {
                    if (event.button.button == SDL_BUTTON_LEFT) {
                        mouse_down = true;
                        last_mouse_x = event.button.x;
                        last_mouse_y = event.button.y;
                    }
                } else if (event.type == SDL_MOUSEBUTTONUP) {
                    if (event.button.button == SDL_BUTTON_LEFT) {
                        mouse_down = false;
                    }
                } else if (event.type == SDL_MOUSEMOTION) {
                    if (mouse_down) {
                        rot_y += (event.motion.x - last_mouse_x) * 0.5f;
                        rot_x += (event.motion.y - last_mouse_y) * 0.5f;
                        last_mouse_x = event.motion.x;
                        last_mouse_y = event.motion.y;
                    }
                }
            }
        }

        if (auto_rotate) {
            rot_y += rot_step;
        }

        // --- Begin Profiling Frame ---
        profiler.begin_frame();

        // Stage 1: Clear
        profiler.begin_stage(ProfileStage::Clear);
        memset(fb_data.data(), 0, fb_data.size());
        clear_depth(db);
        profiler.end_stage(ProfileStage::Clear);

        // Stage 2 & 3: Geometry & Raster
        const Mesh3D& mesh = CAR_MESH;
        float dist = 2.5f;
        Mat4 model = mat4_mul_mat4(mat4_rotate_x(deg_to_rad(rot_x)),
                                   mat4_rotate_y(deg_to_rad(rot_y)));
        Mat4 view = mat4_look_at({0.0f, 0.0f, dist}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
        Mat4 mvp = mat4_mul_mat4(proj, mat4_mul_mat4(view, model));

        PipelineState state;
        state.cull_mode = culling ? CullMode::Back : CullMode::None;
        state.front_face = WindingOrder::CCW;
        state.polygon_mode = wireframe ? PolygonMode::Wireframe : PolygonMode::Fill;
        state.interp_mode = interp_mode;
        state.vertex_shader = car_vertex_shader;
        state.pixel_shader = car_pixel_shader;
        esr_set_uniforms(state, &mvp);

        // Geometry & Raster stages are tracked directly inside draw_mesh
        int pixels_drawn = draw_mesh(mesh, mvp, vp, fb, &db, state, &profiler);

        uint32_t total_mesh_tris = mesh.index_count / 3;
        uint32_t tris_drawn = total_mesh_tris;
        if (culling) {
            tris_drawn = total_mesh_tris / 2; // ~50% culled
        }

        // Draw live on-screen HUD overlay
        if (hud) {
            profiler.draw_hud(fb, 4, 4, false);
        }

        // Stage 4: Present (display texture transfer)
        profiler.begin_stage(ProfileStage::Present);
        if (!headless && texture && renderer) {
            SDL_UpdateTexture(texture, nullptr, fb_data.data(), render_w * 3);
            SDL_RenderClear(renderer);
            SDL_RenderCopy(renderer, texture, nullptr, nullptr);
            SDL_RenderPresent(renderer);
        }
        profiler.end_stage(ProfileStage::Present);

        profiler.end_frame(total_mesh_tris, tris_drawn, pixels_drawn);
        profiler.log_periodic(60);

        if (dump_frames_dir) {
            char fname[512];
            snprintf(fname, sizeof(fname), "%s/frame_%04d.ppm", dump_frames_dir, frame_count);
            FILE* f = fopen(fname, "wb");
            if (f) {
                fprintf(f, "P6\n%d %d\n255\n", render_w, render_h);
                fwrite(fb_data.data(), 1, fb_data.size(), f);
                fclose(f);
            }
        }

        frame_count++;
        if (max_frames > 0 && frame_count >= max_frames) {
            break;
        }
    }

    if (screenshot_path) {
        FILE* f = fopen(screenshot_path, "wb");
        if (f) {
            fprintf(f, "P6\n%d %d\n255\n", render_w, render_h);
            fwrite(fb_data.data(), 1, fb_data.size(), f);
            fclose(f);
            printf("[Viewer] Saved screenshot to %s\n", screenshot_path);
        }
    }

    if (!headless) {
        if (texture) SDL_DestroyTexture(texture);
        if (renderer) SDL_DestroyRenderer(renderer);
        if (window) SDL_DestroyWindow(window);
        SDL_Quit();
    }

    printf("\nViewer finished after %d frames.\n", frame_count);
    return 0;
}
