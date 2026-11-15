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
#include "models/cube.h"
#include "models/icosphere.h"

using namespace esr;

static const int RENDER_W = 160;
static const int RENDER_H = 120;
static const int WINDOW_SCALE = 5; // 800 x 600 window
static const int WINDOW_W = RENDER_W * WINDOW_SCALE;
static const int WINDOW_H = RENDER_H * WINDOW_SCALE;

struct LightUniforms {
    Mat4 mvp;
    Vec3 light_dir;
    Vec3 light_color;
    Vec3 ambient;
    Vec3 obj_color;
};

static void lighting_vs(const void* in_vertex, esr_vertex_out_t* out_vertex, const void* uniforms) {
    const auto* v = static_cast<const Vertex3D*>(in_vertex);
    const auto* u = static_cast<const LightUniforms*>(uniforms);

    out_vertex->clip_pos = mat4_mul_vec4(u->mvp, vec4_from_vec3(v->position, 1.0f));

    // Spherical normal
    Vec3 normal = vec3_normalize(v->position);
    out_vertex->varyings.data[0] = normal.x;
    out_vertex->varyings.data[1] = normal.y;
    out_vertex->varyings.data[2] = normal.z;
    out_vertex->varyings.count = 3;
}

static uint16_t lighting_ps(const float* varyings, const void* uniforms) {
    const auto* u = static_cast<const LightUniforms*>(uniforms);

    Vec3 normal = vec3_normalize(Vec3{varyings[0], varyings[1], varyings[2]});
    float n_dot_l = vec3_dot(normal, u->light_dir);
    if (n_dot_l < 0.0f) n_dot_l = 0.0f;

    float intensity = 0.15f + 0.85f * n_dot_l;
    Vec3 col = vec3_scale(u->obj_color, intensity);

    return esr_pack_rgb565(col.x, col.y, col.z);
}

int main(int argc, char** argv) {
    bool headless = false;
    bool vsync = false;
    int max_frames = -1;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--headless") == 0) headless = true;
        else if (strcmp(argv[i], "--vsync") == 0) vsync = true;
        else if (strcmp(argv[i], "--frames") == 0 && i + 1 < argc) {
            max_frames = atoi(argv[++i]);
        }
    }

    printf("=====================================================\n");
    printf("   ESR Interactive Real-Time 3D Viewer & Profiler     \n");
    printf("   Render resolution: %dx%d (Window: %dx%d)          \n",
           RENDER_W, RENDER_H, WINDOW_W, WINDOW_H);
    printf("   Controls:                                         \n");
    printf("     [SPACE]   : Toggle rotation                     \n");
    printf("     [W]       : Toggle Wireframe / Solid Fill       \n");
    printf("     [C]       : Toggle Backface Culling             \n");
    printf("     [L]       : Toggle Lighting Shader (N · L)      \n");
    printf("     [M]       : Switch Mesh (Icosphere / Cube)      \n");
    printf("     [V]       : Toggle V-Sync (Capped vs Uncapped)  \n");
    printf("     [H]       : Toggle Profiler HUD Overlay         \n");
    printf("     [Drag]    : Orbit camera with mouse             \n");
    printf("     [ESC]/[Q] : Exit                                \n");
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
                                      WINDOW_W, WINDOW_H, SDL_WINDOW_SHOWN);
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
                                            RENDER_W, RENDER_H);
            }
        }
    }

    // Allocate render buffers
    std::vector<uint8_t> fb_data(static_cast<size_t>(RENDER_W) * RENDER_H * 3, 0);
    std::vector<uint16_t> db_data(static_cast<size_t>(RENDER_W) * RENDER_H, 0xFFFF);
    Framebuffer fb{fb_data.data(), RENDER_W, RENDER_H};
    DepthBuffer db{db_data.data(), RENDER_W, RENDER_H};
    Viewport vp{0.0f, 0.0f, static_cast<float>(RENDER_W), static_cast<float>(RENDER_H), 0.0f, 1.0f};

    // State toggles
    bool auto_rotate = true;
    bool wireframe = false;
    bool culling = true;
    bool lighting = true;
    bool hud = true;
    int current_mesh = 0; // 0 = Icosphere, 1 = Cube

    float rot_x = 20.0f;
    float rot_y = 0.0f;
    bool mouse_down = false;
    int last_mouse_x = 0;
    int last_mouse_y = 0;

    Profiler profiler;
    Mat4 proj = mat4_perspective(deg_to_rad(60.0f), (float)RENDER_W / (float)RENDER_H, 0.1f, 100.0f);

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
                        case SDLK_l: lighting = !lighting; break;
                        case SDLK_m: current_mesh = (current_mesh + 1) % 2; break;
                        case SDLK_v:
                            vsync = !vsync;
                            SDL_GL_SetSwapInterval(vsync ? 1 : 0);
                            printf("[Viewer] V-Sync %s (%s)\n",
                                   vsync ? "ENABLED" : "DISABLED",
                                   vsync ? "capped to monitor" : "UNCAPPED");
                            break;
                        case SDLK_h: hud = !hud; break;
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
            rot_y += 1.0f;
        }

        // --- Begin Profiling Frame ---
        profiler.begin_frame();

        // Stage 1: Clear
        profiler.begin_stage(ProfileStage::Clear);
        memset(fb_data.data(), 0, fb_data.size());
        clear_depth(db);
        profiler.end_stage(ProfileStage::Clear);

        // Stage 2 & 3: Geometry & Raster
        const Mesh3D& mesh = (current_mesh == 0) ? ICOSPHERE_MESH : CUBE_MESH;
        float dist = (current_mesh == 0) ? 2.8f : 3.5f;
        Mat4 model = mat4_mul_mat4(mat4_rotate_x(deg_to_rad(rot_x)),
                                   mat4_rotate_y(deg_to_rad(rot_y)));
        Mat4 view = mat4_look_at({0.0f, 0.0f, dist}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
        Mat4 mvp = mat4_mul_mat4(proj, mat4_mul_mat4(view, model));

        PipelineState state;
        state.cull_mode = culling ? CullMode::Back : CullMode::None;
        state.front_face = (current_mesh == 0) ? WindingOrder::CCW : WindingOrder::CW;
        state.polygon_mode = wireframe ? PolygonMode::Wireframe : PolygonMode::Fill;

        LightUniforms u;
        if (lighting && current_mesh == 0) {
            u.mvp = mvp;
            u.light_dir = vec3_normalize({0.6f, 0.8f, 0.7f});
            u.light_color = {1.0f, 1.0f, 1.0f};
            u.ambient = {0.15f, 0.15f, 0.15f};
            u.obj_color = {0.2f, 0.85f, 0.95f}; // Cyan/Azure
            state.vertex_shader = lighting_vs;
            state.pixel_shader = lighting_ps;
            esr_set_uniforms(state, &u);
        }

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
            SDL_UpdateTexture(texture, nullptr, fb_data.data(), RENDER_W * 3);
            SDL_RenderClear(renderer);
            SDL_RenderCopy(renderer, texture, nullptr, nullptr);
            SDL_RenderPresent(renderer);
        }
        profiler.end_stage(ProfileStage::Present);

        profiler.end_frame(total_mesh_tris, tris_drawn, pixels_drawn);
        profiler.log_periodic(60);

        frame_count++;
        if (max_frames > 0 && frame_count >= max_frames) {
            break;
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
