#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstdlib>

#include "esr/esr_pipeline.hpp"

#define W 160
#define H 120

static uint8_t fb[H][W][3];

static float edge(float ax, float ay, float bx, float by, float px, float py)
{
    return (px - ax) * (by - ay) - (py - ay) * (bx - ax);
}

static void render_screen_triangle(const esr::ScreenTriangle& tri)
{
    const float vx[3] = {tri.v[0].x, tri.v[1].x, tri.v[2].x};
    const float vy[3] = {tri.v[0].y, tri.v[1].y, tri.v[2].y};

    const float col[3][3] = {
        {tri.v[0].color.x, tri.v[0].color.y, tri.v[0].color.z},
        {tri.v[1].color.x, tri.v[1].color.y, tri.v[1].color.z},
        {tri.v[2].color.x, tri.v[2].color.y, tri.v[2].color.z},
    };

    float area = edge(vx[0], vy[0], vx[1], vy[1], vx[2], vy[2]);
    if (area == 0.0f) {
        return; // Degenerate edge
    }

    // Support both clockwise and counter-clockwise winding
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            float px = x + 0.5f;
            float py = y + 0.5f;

            float w0 = edge(vx[1], vy[1], vx[2], vy[2], px, py) / area;
            float w1 = edge(vx[2], vy[2], vx[0], vy[0], px, py) / area;
            float w2 = edge(vx[0], vy[0], vx[1], vy[1], px, py) / area;

            if (w0 >= 0.0f && w1 >= 0.0f && w2 >= 0.0f) {
                for (int c = 0; c < 3; ++c) {
                    float v = w0 * col[0][c] + w1 * col[1][c] + w2 * col[2][c];
                    fb[y][x][c] = (uint8_t)(v * 255.0f);
                }
            }
        }
    }
}

static int ensure_dir(const char *dir)
{
#if defined(_WIN32)
    return mkdir(dir);
#else
    return mkdir(dir, 0755);
#endif
}

static int save_ppm(const char *filename)
{
    ensure_dir("output");
    ensure_dir("output/desktop");

    FILE *f = fopen(filename, "wb");
    if (!f) {
        perror("Failed to open output PPM file");
        return -1;
    }

    fprintf(f, "P6\n%d %d\n255\n", W, H);
    size_t written = fwrite(fb, 1, sizeof(fb), f);
    fclose(f);

    return (written == sizeof(fb)) ? 0 : -1;
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("=========================================\n");
    printf("   ESR 3D Pipeline Desktop Runner        \n");
    printf("   True 3D Perspective Rotation Demo     \n");
    printf("   Resolution: %dx%d                     \n", W, H);
    printf("=========================================\n\n");

    using namespace esr;

    // 1. Define a 3D triangle in object space
    const Vertex3D tri[3] = {
        {{ 0.0f,  0.8f, 0.0f}, {1.0f, 0.0f, 0.0f}}, // Top: Red
        {{-0.8f, -0.6f, 0.0f}, {0.0f, 1.0f, 0.0f}}, // Bottom-Left: Green
        {{ 0.8f, -0.6f, 0.0f}, {0.0f, 0.0f, 1.0f}}, // Bottom-Right: Blue
    };

    // 2. Camera View and Perspective Projection matrices
    Mat4 view = mat4_look_at(vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 1.0f, 0.0f));
    Mat4 proj = mat4_perspective(deg_to_rad(60.0f), (float)W / (float)H, 0.1f, 100.0f);
    Viewport vp = {0.0f, 0.0f, (float)W, (float)H, 0.0f, 1.0f};

    // 3. Render 36 animation frames rotating in 3D around the Y-axis
    ensure_dir("output/desktop/frames");
    printf("Generating 36-frame 3D rotation sequence...\n");

    for (int frame = 0; frame < 36; ++frame) {
        memset(fb, 0, sizeof(fb));

        float angle_y = deg_to_rad((float)frame * 10.0f);
        float angle_x = deg_to_rad(20.0f); // Tilted down towards camera

        Mat4 rot_y = mat4_rotate_y(angle_y);
        Mat4 rot_x = mat4_rotate_x(angle_x);
        Mat4 trans = mat4_translate(vec3(0.0f, 0.0f, -2.5f)); // Placed 2.5 units in front of camera

        Mat4 model = mat4_mul_mat4(trans, mat4_mul_mat4(rot_x, rot_y));
        Mat4 mvp   = mat4_mul_mat4(proj, mat4_mul_mat4(view, model));

        ScreenTriangle st[ESR_MAX_CLIPPED_TRIANGLES];
        const int st_count = process_triangle(tri[0], tri[1], tri[2], mvp, vp, st);
        for (int t = 0; t < st_count; ++t) {
            render_screen_triangle(st[t]);
        }

        char frame_path[128];
        snprintf(frame_path, sizeof(frame_path), "output/desktop/frames/f_%02d.ppm", frame);
        save_ppm(frame_path);

        // Frame 5 (tilted at 50 degrees) will be our primary still frame
        if (frame == 5) {
            save_ppm("output/desktop/frame.ppm");
            printf("Still frame saved to 'output/desktop/frame.ppm' (Tilted 3D view at 50 deg):\n");
            for (int i = 0; i < 3; ++i) {
                printf("  V%d: Screen(%.1f, %.1f) Depth=%.3f InvW=%.3f\n",
                       i, st[0].v[i].x, st[0].v[i].y, st[0].v[i].z, st[0].v[i].inv_w);
            }
        }
    }

    // 4. Near-plane clipping check: fly the camera towards and through the triangle.
    //    Without clipping, vertices behind the camera (w <= 0) would be mirrored
    //    across the screen or blow up to inf/NaN as the camera passes the triangle.
    ensure_dir("output/desktop/flythrough");
    printf("\nGenerating 40-frame camera fly-through sequence (near-plane clipping)...\n");

    const Mat4 tri_model = mat4_translate(vec3(0.0f, 0.0f, -2.5f));
    for (int frame = 0; frame < 40; ++frame) {
        memset(fb, 0, sizeof(fb));

        // Camera moves from z = 0 to z = -3.0 (the triangle sits at z = -2.5),
        // looking slightly up and to the right so the near plane cuts it unevenly.
        const float cam_z = -(float)frame * 0.075f;
        Mat4 fly_view = mat4_look_at(vec3(0.0f, 0.0f, cam_z),
                                     vec3(0.3f, 0.2f, cam_z - 1.0f),
                                     vec3(0.0f, 1.0f, 0.0f));
        Mat4 mvp = mat4_mul_mat4(proj, mat4_mul_mat4(fly_view, tri_model));

        ScreenTriangle st[ESR_MAX_CLIPPED_TRIANGLES];
        const int st_count = process_triangle(tri[0], tri[1], tri[2], mvp, vp, st);
        for (int t = 0; t < st_count; ++t) {
            render_screen_triangle(st[t]);
        }
        printf("  frame %02d: camera z = %+.3f -> %d triangle(s)\n", frame, cam_z, st_count);

        char frame_path[128];
        snprintf(frame_path, sizeof(frame_path), "output/desktop/flythrough/f_%02d.ppm", frame);
        save_ppm(frame_path);
    }

    // 5. Assemble the rotation frames into a smooth animated GIF
    printf("\nAssembling 'output/desktop/spinning_3d.gif' using ImageMagick...\n");
    int ret = system("magick -delay 6 -loop 0 output/desktop/frames/f_*.ppm -scale 300% output/desktop/spinning_3d.gif");
    if (ret == 0) {
        printf("Successfully generated animated 3D preview: output/desktop/spinning_3d.gif!\n");
    }

    if (system("magick -delay 8 -loop 0 output/desktop/flythrough/f_*.ppm -scale 300% output/desktop/flythrough.gif") == 0) {
        printf("Successfully generated fly-through preview: output/desktop/flythrough.gif!\n");
    }

    // Scale still frame to PNG
    (void)!system("magick output/desktop/frame.ppm -scale 400% output/desktop/frame_4x.png");

    return 0;
}
