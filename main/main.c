#include <stdio.h>
#include <stdint.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "st7789.h"
#include "test_photo.h"
#else
#include <time.h>
#endif

#define W 160
#define H 120

#ifndef DUMP_FRAME
#define DUMP_FRAME 1
#endif

static uint8_t fb[H][W][3];

static float edge(float ax, float ay, float bx, float by, float px, float py)
{
    return (px - ax) * (by - ay) - (py - ay) * (bx - ax);
}

static void render_triangle(void)
{
    const float vx[3] = {80.0f, 10.0f, 150.0f};
    const float vy[3] = {8.0f, 110.0f, 110.0f};

    const float col[3][3] = {
        {1.0f, 0.0f, 0.0f}, // Red
        {0.0f, 1.0f, 0.0f}, // Green
        {0.0f, 0.0f, 1.0f}, // Blue
    };

    float area = edge(vx[0], vy[0], vx[1], vy[1], vx[2], vy[2]);
    memset(fb, 0, sizeof(fb));

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

static void print_ascii(void)
{
    for (int y = 0; y < H; y += 4) {
        for (int x = 0; x < W; x += 2) {
            int r = fb[y][x][0], g = fb[y][x][1], b = fb[y][x][2];
            char ch = ' ';
            if (r | g | b) ch = (r >= g && r >= b) ? 'R' : (g >= b ? 'G' : 'B');
            putchar(ch);
        }
        putchar('\n');
    }
}

#if DUMP_FRAME
static void dump_frame(void)
{
    const uint8_t *p = &fb[0][0][0];
    printf("PPM_BEGIN %d %d\n", W, H);
    for (int i = 0; i < W * H * 3; i++) {
        printf("%02x", p[i]);
        if ((i % 32) == 31) {
            putchar('\n');
            if (((i / 32) % 16) == 15) vTaskDelay(1);
        }
    }
    printf("\nPPM_END\n");
}
#endif

void app_main(void)
{
    printf("\n=========================================\n");
    printf("       ESR Bringup on ESP32-S3          \n");
    printf("=========================================\n");

#ifdef ESP_PLATFORM
    printf("Initializing ST7789 display (240x320)...\n");
    esp_err_t err = st7789_init();
    if (err == ESP_OK) {
        // Display full-screen photo (240x320)
        printf("Display initialized! Rendering full-screen photo (%dx%d)...\n", PHOTO_WIDTH, PHOTO_HEIGHT);
        int64_t tp0 = esp_timer_get_time();
        st7789_draw_bitmap(0, 0, PHOTO_WIDTH, PHOTO_HEIGHT, photo_data);
        int64_t tp1 = esp_timer_get_time();
        printf("Photo rendered and sent in: %.2f ms!\n", (double)(tp1 - tp0) / 1000.0);

        // Show photo for 4 seconds before starting triangle loop
        vTaskDelay(pdMS_TO_TICKS(4000));

        // Fill background with dark slate blue (RGB565: 0x10A2)
        printf("Switching to 3D triangle rasterizer...\n");
        st7789_fill_color(0x10A2);
    } else {
        printf("Warning: ST7789 initialization failed: %s\n", esp_err_to_name(err));
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Center 160x120 rendered triangle on the 240x320 display
    const int dst_x = (240 - W) / 2; // 40
    const int dst_y = (320 - H) / 2; // 100

    while (1) {
        printf("\n=== Rendering Triangle ===\n");
        int64_t t0 = esp_timer_get_time();
        render_triangle();
        int64_t t1 = esp_timer_get_time();

        double render_ms = (double)(t1 - t0) / 1000.0;
        printf("Render time: %.2f ms\n", render_ms);

#ifdef ESP_PLATFORM
        int64_t tb0 = esp_timer_get_time();
        st7789_draw_rgb888_buffer(dst_x, dst_y, W, H, &fb[0][0][0]);
        int64_t tb1 = esp_timer_get_time();
        double blit_ms = (double)(tb1 - tb0) / 1000.0;
        printf("Screen Blit time: %.2f ms (Total: %.2f ms)\n", blit_ms, render_ms + blit_ms);
#endif

        print_ascii();

#if DUMP_FRAME
        dump_frame();
#endif
        printf("Frame sent to ST7789 screen! Next refresh in 3 s.\n");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
