/**
 * test_esr_pipeline.cpp - Unit tests for the 3D geometry pipeline harness.
 */
#include <stdio.h>
#include <cmath>
#include "esr/esr_pipeline.hpp"

using namespace esr;

static int g_checks = 0;
static int g_failures = 0;

static const float EPS = 1e-5f;

#define CHECK(cond)                                                          \
    do {                                                                     \
        ++g_checks;                                                          \
        if (!(cond)) {                                                       \
            ++g_failures;                                                    \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
        }                                                                    \
    } while (0)

static bool feq(float a, float b, float eps = EPS) {
    return std::fabs(a - b) <= eps;
}

static bool vec3_eq(const Vec3& a, const Vec3& b) {
    return feq(a.x, b.x) && feq(a.y, b.y) && feq(a.z, b.z);
}

static bool vec4_eq(const Vec4& a, const Vec4& b) {
    return feq(a.x, b.x) && feq(a.y, b.y) && feq(a.z, b.z) && feq(a.w, b.w);
}

/* ------------------------------------------------------------------ */
/* Test Cases                                                         */
/* ------------------------------------------------------------------ */

static void test_transform_mvp_identity() {
    Vertex3D v = {{1.0f, 2.0f, 3.0f}, {1.0f, 0.0f, 0.0f}};
    Mat4 identity = mat4_identity();

    ClipVertex clip = transform_mvp(v, identity);
    CHECK(vec4_eq(clip.position, vec4(1.0f, 2.0f, 3.0f, 1.0f)));
    CHECK(vec3_eq(clip.color, vec3(1.0f, 0.0f, 0.0f)));
}

static void test_transform_mvp_translation() {
    Vertex3D v = {{1.0f, 2.0f, 3.0f}, {0.0f, 1.0f, 0.0f}};
    Mat4 trans = mat4_translate(vec3(10.0f, -5.0f, 2.0f));

    ClipVertex clip = transform_mvp(v, trans);
    CHECK(vec4_eq(clip.position, vec4(11.0f, -3.0f, 5.0f, 1.0f)));
}

static void test_perspective_divide_normal() {
    ClipVertex clip = {vec4(2.0f, -4.0f, 6.0f, 2.0f), vec3(1.0f, 1.0f, 1.0f)};
    Vec3 ndc;
    float inv_w = 0.0f;

    bool ok = perspective_divide(clip, ndc, inv_w);
    CHECK(ok == true);
    CHECK(vec3_eq(ndc, vec3(1.0f, -2.0f, 3.0f)));
    CHECK(feq(inv_w, 0.5f));
}

static void test_perspective_divide_zero_w() {
    ClipVertex clip = {vec4(1.0f, 2.0f, 3.0f, 0.0f), vec3(0.0f, 0.0f, 0.0f)};
    Vec3 ndc;
    float inv_w = 0.0f;

    bool ok = perspective_divide(clip, ndc, inv_w);
    CHECK(ok == false);
}

static void test_viewport_transform() {
    Viewport vp = {0.0f, 0.0f, 160.0f, 120.0f, 0.0f, 1.0f};

    // 1. Center of NDC (0, 0, 0) -> Center of screen (80, 60, 0.5)
    ScreenVertex center = viewport_transform(vec3(0.0f, 0.0f, 0.0f), 1.0f, vec3(1, 1, 1), vp);
    CHECK(feq(center.x, 80.0f));
    CHECK(feq(center.y, 60.0f));
    CHECK(feq(center.z, 0.5f));

    // 2. Top-left of NDC (-1, 1, -1) -> Top-left pixel (0, 0, 0)
    ScreenVertex top_left = viewport_transform(vec3(-1.0f, 1.0f, -1.0f), 1.0f, vec3(1, 0, 0), vp);
    CHECK(feq(top_left.x, 0.0f));
    CHECK(feq(top_left.y, 0.0f));
    CHECK(feq(top_left.z, 0.0f));

    // 3. Bottom-right of NDC (1, -1, 1) -> Bottom-right pixel (160, 120, 1)
    ScreenVertex bottom_right = viewport_transform(vec3(1.0f, -1.0f, 1.0f), 1.0f, vec3(0, 0, 1), vp);
    CHECK(feq(bottom_right.x, 160.0f));
    CHECK(feq(bottom_right.y, 120.0f));
    CHECK(feq(bottom_right.z, 1.0f));
}

static void test_process_triangle_deterministic_3d() {
    // Symmetrical triangle standing vertically in front of camera at Z = -4.0
    Vertex3D v0 = {{ 0.0f,  1.0f, -4.0f}, {1.0f, 0.0f, 0.0f}}; // Top
    Vertex3D v1 = {{-1.0f, -1.0f, -4.0f}, {0.0f, 1.0f, 0.0f}}; // Bottom-Left
    Vertex3D v2 = {{ 1.0f, -1.0f, -4.0f}, {0.0f, 0.0f, 1.0f}}; // Bottom-Right

    Mat4 model = mat4_identity();
    Mat4 view = mat4_look_at(vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 1.0f, 0.0f));
    Mat4 proj = mat4_perspective(deg_to_rad(60.0f), 160.0f / 120.0f, 0.1f, 100.0f);

    Mat4 mv = mat4_mul_mat4(view, model);
    Mat4 mvp = mat4_mul_mat4(proj, mv);

    Viewport vp = {0.0f, 0.0f, 160.0f, 120.0f, 0.0f, 1.0f};
    ScreenTriangle out_tri;

    bool ok = process_triangle(v0, v1, v2, mvp, vp, out_tri);
    CHECK(ok == true);

    // Vertex 0 (top vertex) should be horizontally centered on screen (x == 80)
    CHECK(feq(out_tri.v[0].x, 80.0f));
    // Vertices 1 and 2 should be symmetric around the vertical center line
    CHECK(out_tri.v[1].x < 80.0f);
    CHECK(out_tri.v[2].x > 80.0f);
    CHECK(feq(80.0f - out_tri.v[1].x, out_tri.v[2].x - 80.0f));

    // Base vertices 1 and 2 must have identical Y coordinates
    CHECK(feq(out_tri.v[1].y, out_tri.v[2].y));
    // Top vertex must be higher on screen than base (smaller Y in screen coords)
    CHECK(out_tri.v[0].y < out_tri.v[1].y);

    // All depth values must be within valid range [0, 1]
    for (int i = 0; i < 3; ++i) {
        CHECK(out_tri.v[i].z >= 0.0f && out_tri.v[i].z <= 1.0f);
        CHECK(out_tri.v[i].inv_w > 0.0f);
    }
}

int main() {
    printf("Running ESR 3D Pipeline unit tests...\n");

    test_transform_mvp_identity();
    test_transform_mvp_translation();
    test_perspective_divide_normal();
    test_perspective_divide_zero_w();
    test_viewport_transform();
    test_process_triangle_deterministic_3d();

    printf("ESR Pipeline tests completed: %d checks, %d failed.\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
