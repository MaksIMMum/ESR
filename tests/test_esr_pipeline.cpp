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
    ScreenTriangle out_tris[ESR_MAX_CLIPPED_TRIANGLES];

    int count = process_triangle(v0, v1, v2, mvp, vp, out_tris);
    CHECK(count == 1); // Fully in front of the camera: nothing clipped
    const ScreenTriangle& out_tri = out_tris[0];

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

/* ------------------------------------------------------------------ */
/* Near-plane clipping                                                  */
/* ------------------------------------------------------------------ */

static ClipVertex cv(float x, float y, float z, float w, Vec3 color) {
    ClipVertex v;
    v.position = Vec4{x, y, z, w};
    v.color = color;
    return v;
}

static float near_dist(const ClipVertex& v) { return v.position.z + v.position.w; }

/** Signed 2D area of the triangle after perspective division (sign = winding). */
static float ndc_signed_area(const ClipTriangle& t) {
    float x[3], y[3];
    for (int i = 0; i < 3; ++i) {
        x[i] = t.v[i].position.x / t.v[i].position.w;
        y[i] = t.v[i].position.y / t.v[i].position.w;
    }
    return (x[1] - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (y[1] - y[0]);
}

static const Vec3 RED   = {1.0f, 0.0f, 0.0f};
static const Vec3 GREEN = {0.0f, 1.0f, 0.0f};
static const Vec3 BLUE  = {0.0f, 0.0f, 1.0f};

static void test_clip_all_inside() {
    ClipTriangle in = {{cv(0, 1, 0, 1, RED), cv(-1, -1, 0, 1, GREEN), cv(1, -1, 0, 1, BLUE)}};
    ClipTriangle out[ESR_MAX_CLIPPED_TRIANGLES];

    CHECK(clip_triangle_near(in, out) == 1);
    for (int i = 0; i < 3; ++i) {
        CHECK(vec4_eq(out[0].v[i].position, in.v[i].position));
        CHECK(vec3_eq(out[0].v[i].color, in.v[i].color));
    }
}

static void test_clip_all_behind() {
    // z + w < 0 for every vertex (includes w <= 0, i.e. behind the camera)
    ClipTriangle in = {{cv(0, 1, -3, 1, RED), cv(-1, -1, 1, -2, GREEN), cv(1, -1, -0.5f, 0, BLUE)}};
    ClipTriangle out[ESR_MAX_CLIPPED_TRIANGLES];

    CHECK(clip_triangle_near(in, out) == 0);
}

static void test_clip_one_vertex_behind() {
    // v0 behind (d = -1), v1/v2 in front (d = 1) -> quad -> 2 triangles
    ClipTriangle in = {{cv(0, 1, -2, 1, RED), cv(-1, -1, 0, 1, GREEN), cv(1, -1, 0, 1, BLUE)}};
    ClipTriangle out[ESR_MAX_CLIPPED_TRIANGLES];

    int n = clip_triangle_near(in, out);
    CHECK(n == 2);

    int on_plane = 0;
    for (int t = 0; t < n; ++t) {
        for (int i = 0; i < 3; ++i) {
            const ClipVertex& v = out[t].v[i];
            CHECK(near_dist(v) >= -EPS);
            if (feq(near_dist(v), 0.0f)) {
                ++on_plane;
                // Edges v0->v1 and v2->v0 are cut exactly halfway: color is a 50/50 mix
                CHECK(feq(v.color.x, 0.5f));
                CHECK(feq(v.position.y, 0.0f));
            }
        }
        CHECK(ndc_signed_area(out[t]) * ndc_signed_area(in) > 0.0f); // winding kept
    }
    CHECK(on_plane >= 2); // both new vertices are on the near plane
}

static void test_clip_two_vertices_behind() {
    // v0 in front (d = 1), v1/v2 behind (d = -1) -> 1 smaller triangle
    ClipTriangle in = {{cv(0, 1, 0, 1, RED), cv(-1, -1, -2, 1, GREEN), cv(1, -1, -2, 1, BLUE)}};
    ClipTriangle out[ESR_MAX_CLIPPED_TRIANGLES];

    CHECK(clip_triangle_near(in, out) == 1);
    CHECK(vec4_eq(out[0].v[0].position, in.v[0].position));
    for (int i = 1; i < 3; ++i) {
        CHECK(feq(near_dist(out[0].v[i]), 0.0f));
        CHECK(feq(out[0].v[i].color.x, 0.5f)); // halfway from red to green/blue
        CHECK(feq(out[0].v[i].position.y, 0.0f));
    }
    CHECK(ndc_signed_area(out[0]) * ndc_signed_area(in) > 0.0f);
}

/**
 * Fly the camera towards and through a triangle. Every produced vertex must be
 * finite, in front of the camera (inv_w > 0) and inside the depth range.
 */
static void test_camera_fly_through() {
    Vertex3D v0 = {{ 0.0f,  1.0f, 0.0f}, RED};
    Vertex3D v1 = {{-1.0f, -1.0f, 0.0f}, GREEN};
    Vertex3D v2 = {{ 1.0f, -1.0f, 0.0f}, BLUE};
    Mat4 proj = mat4_perspective(deg_to_rad(60.0f), 160.0f / 120.0f, 0.1f, 100.0f);
    Viewport vp = {0.0f, 0.0f, 160.0f, 120.0f, 0.0f, 1.0f};

    bool saw_rejected = false, saw_split = false;
    for (int step = 0; step <= 80; ++step) {
        // Camera at z from 2.0 down to -2.0, tilted (up and right) so the near plane crosses the triangle
        float cam_z = 2.0f - step * 0.05f;
        Mat4 view = mat4_look_at(vec3(0.0f, 0.0f, cam_z), vec3(0.4f, 0.5f, cam_z - 1.0f), vec3(0.0f, 1.0f, 0.0f));
        Mat4 mvp = mat4_mul_mat4(proj, view);

        ScreenTriangle out[ESR_MAX_CLIPPED_TRIANGLES];
        int n = process_triangle(v0, v1, v2, mvp, vp, out);
        CHECK(n >= 0 && n <= ESR_MAX_CLIPPED_TRIANGLES);
        if (n == 0) saw_rejected = true;
        if (n == 2) saw_split = true;

        for (int t = 0; t < n; ++t) {
            for (int i = 0; i < 3; ++i) {
                const ScreenVertex& s = out[t].v[i];
                CHECK(std::isfinite(s.x) && std::isfinite(s.y));
                CHECK(s.inv_w > 0.0f);
                CHECK(s.z >= -EPS && s.z <= 1.0f + EPS);
            }
        }
    }
    CHECK(saw_rejected); // camera ended up behind the triangle
    CHECK(saw_split);    // at least one frame had the near plane cutting the triangle
}

int main() {
    printf("Running ESR 3D Pipeline unit tests...\n");

    test_transform_mvp_identity();
    test_transform_mvp_translation();
    test_perspective_divide_normal();
    test_perspective_divide_zero_w();
    test_viewport_transform();
    test_process_triangle_deterministic_3d();
    test_clip_all_inside();
    test_clip_all_behind();
    test_clip_one_vertex_behind();
    test_clip_two_vertices_behind();
    test_camera_fly_through();

    printf("ESR Pipeline tests completed: %d checks, %d failed.\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
