#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>

#define W 160
#define H 120

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
            if (r | g | b) {
                ch = (r >= g && r >= b) ? 'R' : (g >= b ? 'G' : 'B');
            }
            putchar(ch);
        }
        putchar('\n');
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
    printf("   ESR Native Desktop Runner (Offscreen) \n");
    printf("   Resolution: %dx%d\n", W, H);
    printf("=========================================\n\n");

    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    render_triangle();

    clock_gettime(CLOCK_MONOTONIC, &end);

    double elapsed_ms = (double)(end.tv_sec - start.tv_sec) * 1000.0 +
                        (double)(end.tv_nsec - start.tv_nsec) / 1000000.0;

    printf("Render completed in: %.3f ms\n\n", elapsed_ms);

    printf("ASCII Preview:\n");
    print_ascii();

    const char *out_file = "output/desktop/frame.ppm";
    if (save_ppm(out_file) == 0) {
        printf("\nSuccessfully saved frame to '%s' (%zu bytes)\n", out_file, sizeof(fb));
    } else {
        fprintf(stderr, "\nError saving '%s'\n", out_file);
        return 1;
    }

    return 0;
}
