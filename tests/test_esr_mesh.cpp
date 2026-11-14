/**
 * test_esr_mesh.cpp - Unit tests for the Indexed Mesh Pipeline (VBO/IBO).
 *
 * Tests:
 *   1. Mesh3D structure validity (cube.h index bounds)
 *   2. Synthetic 2-triangle quad indexing
 *   3. Pyramid mesh index validity
 *   4. Icosphere mesh index validity and triangle count
 *   5. draw_mesh() renders pixels with a visible cube
 *   6. draw_mesh() produces zero pixels for fully clipped mesh
 *   7. Index count is always a multiple of 3
 *   8. Zero-allocation: Mesh3D points to const static data
 */
#include <stdio.h>
#include <cmath>
#include <cstring>
#include "esr/esr_mesh.hpp"
#include "esr/esr_raster.hpp"

// Generated model headers
#include "models/cube.h"
#include "models/pyramid.h"
#include "models/icosphere.h"

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

/* ------------------------------------------------------------------ */
/* Test: Cube index bounds                                             */
/* ------------------------------------------------------------------ */

static void test_cube_index_bounds() {
    // All indices in cube.h must be < vertex_count
    CHECK(CUBE_MESH.vertex_count == 8);
    CHECK(CUBE_MESH.index_count == 36);
    CHECK(CUBE_MESH.index_count % 3 == 0);

    for (uint32_t i = 0; i < CUBE_MESH.index_count; ++i) {
        CHECK(CUBE_MESH.indices[i] < CUBE_MESH.vertex_count);
    }
}

/* ------------------------------------------------------------------ */
/* Test: Pyramid index bounds                                          */
/* ------------------------------------------------------------------ */

static void test_pyramid_index_bounds() {
    CHECK(PYRAMID_MESH.vertex_count == 5);
    CHECK(PYRAMID_MESH.index_count == 18);
    CHECK(PYRAMID_MESH.index_count % 3 == 0);

    for (uint32_t i = 0; i < PYRAMID_MESH.index_count; ++i) {
        CHECK(PYRAMID_MESH.indices[i] < PYRAMID_MESH.vertex_count);
    }
}

/* ------------------------------------------------------------------ */
/* Test: Icosphere index bounds and triangle count                     */
/* ------------------------------------------------------------------ */

static void test_icosphere_index_bounds() {
    CHECK(ICOSPHERE_MESH.vertex_count == 42);
    CHECK(ICOSPHERE_MESH.index_count == 240);
    CHECK(ICOSPHERE_MESH.index_count % 3 == 0);

    // 80 triangles * 3 = 240 indices
    CHECK(ICOSPHERE_MESH.index_count / 3 == 80);

    for (uint32_t i = 0; i < ICOSPHERE_MESH.index_count; ++i) {
        CHECK(ICOSPHERE_MESH.indices[i] < ICOSPHERE_MESH.vertex_count);
    }
}

/* ------------------------------------------------------------------ */
/* Test: Synthetic 2-triangle quad (manual VBO/IBO)                    */
/* ------------------------------------------------------------------ */

static void test_synthetic_quad_indexing() {
    // A quad made of 4 vertices and 2 triangles, sharing an edge (indices 0,2)
    //
    //  v0 (0,1)---v1 (1,1)
    //   |  \         |
    //   |   tri0     |
    //   |      \     |
    //   |  tri1  \   |
    //  v3 (0,0)---v2 (1,0)
    //
    // tri0 = (0,1,2), tri1 = (0,2,3)

    static const Vertex3D quad_verts[] = {
        {{ 0.0f,  1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}},  // v0: top-left
        {{ 1.0f,  1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}},  // v1: top-right
        {{ 1.0f,  0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},  // v2: bottom-right
        {{ 0.0f,  0.0f, 0.0f}, {1.0f, 1.0f, 0.0f}},  // v3: bottom-left
    };
    static const uint16_t quad_indices[] = {
        0, 1, 2,   // tri0
        0, 2, 3    // tri1
    };
    static const Mesh3D quad_mesh = {
        quad_verts,
        sizeof(quad_verts) / sizeof(quad_verts[0]),
        quad_indices,
        sizeof(quad_indices) / sizeof(quad_indices[0])
    };

    CHECK(quad_mesh.vertex_count == 4);
    CHECK(quad_mesh.index_count == 6);
    CHECK(quad_mesh.index_count % 3 == 0);

    // Verify each index is within bounds
    for (uint32_t i = 0; i < quad_mesh.index_count; ++i) {
        CHECK(quad_mesh.indices[i] < quad_mesh.vertex_count);
    }

    // Verify the vertex data accessed through indices is correct
    // Triangle 0: vertices 0, 1, 2
    const Vertex3D& t0_v0 = quad_mesh.vertices[quad_mesh.indices[0]];
    const Vertex3D& t0_v1 = quad_mesh.vertices[quad_mesh.indices[1]];
    const Vertex3D& t0_v2 = quad_mesh.vertices[quad_mesh.indices[2]];

    CHECK(feq(t0_v0.position.x, 0.0f) && feq(t0_v0.position.y, 1.0f));
    CHECK(feq(t0_v1.position.x, 1.0f) && feq(t0_v1.position.y, 1.0f));
    CHECK(feq(t0_v2.position.x, 1.0f) && feq(t0_v2.position.y, 0.0f));

    // Triangle 1: vertices 0, 2, 3
    const Vertex3D& t1_v0 = quad_mesh.vertices[quad_mesh.indices[3]];
    const Vertex3D& t1_v1 = quad_mesh.vertices[quad_mesh.indices[4]];
    const Vertex3D& t1_v2 = quad_mesh.vertices[quad_mesh.indices[5]];

    CHECK(feq(t1_v0.position.x, 0.0f) && feq(t1_v0.position.y, 1.0f));
    CHECK(feq(t1_v1.position.x, 1.0f) && feq(t1_v1.position.y, 0.0f));
    CHECK(feq(t1_v2.position.x, 0.0f) && feq(t1_v2.position.y, 0.0f));

    // Shared edge check: tri0[0] and tri1[0] should be the same vertex (index 0)
    CHECK(quad_mesh.indices[0] == quad_mesh.indices[3]); // both index 0
    CHECK(quad_mesh.indices[2] == quad_mesh.indices[4]); // both index 2
}

/* ------------------------------------------------------------------ */
/* Test: draw_mesh() renders visible pixels                            */
/* ------------------------------------------------------------------ */

static void test_draw_mesh_renders_cube() {
    // Place the cube in front of the camera so it's visible
    const int W = 160, H = 120;
    uint8_t fb_data[W * H * 3];
    std::memset(fb_data, 0, sizeof(fb_data));

    Framebuffer fb;
    fb.color = fb_data;
    fb.width = W;
    fb.height = H;

    Viewport vp = {0.0f, 0.0f, (float)W, (float)H, 0.0f, 1.0f};

    // Camera looking at origin, cube centered at origin
    Mat4 model = mat4_identity();
    Mat4 view = mat4_look_at(
        vec3(3.0f, 3.0f, 3.0f),    // eye
        vec3(0.0f, 0.0f, 0.0f),    // center
        vec3(0.0f, 1.0f, 0.0f)     // up
    );
    Mat4 proj = mat4_perspective(deg_to_rad(60.0f), (float)W / (float)H, 0.1f, 100.0f);
    Mat4 mv = mat4_mul_mat4(view, model);
    Mat4 mvp = mat4_mul_mat4(proj, mv);

    int pixels = draw_mesh(CUBE_MESH, mvp, vp, fb);

    // The cube should be visible and produce some pixels
    CHECK(pixels > 0);

    // With a 160x120 framebuffer, the cube should cover a reasonable area
    CHECK(pixels > 100);

    // Verify that some pixels in the framebuffer are non-zero (were actually written)
    bool found_nonzero = false;
    for (int i = 0; i < W * H * 3; ++i) {
        if (fb_data[i] != 0) {
            found_nonzero = true;
            break;
        }
    }
    CHECK(found_nonzero);
}

/* ------------------------------------------------------------------ */
/* Test: draw_mesh() with mesh behind camera produces zero pixels      */
/* ------------------------------------------------------------------ */

static void test_draw_mesh_clipped() {
    const int W = 160, H = 120;
    uint8_t fb_data[W * H * 3];
    std::memset(fb_data, 0, sizeof(fb_data));

    Framebuffer fb;
    fb.color = fb_data;
    fb.width = W;
    fb.height = H;

    Viewport vp = {0.0f, 0.0f, (float)W, (float)H, 0.0f, 1.0f};

    // Camera looking away from the cube (cube is behind camera)
    Mat4 model = mat4_translate(vec3(0.0f, 0.0f, 10.0f)); // Push cube behind
    Mat4 view = mat4_look_at(
        vec3(0.0f, 0.0f, 0.0f),
        vec3(0.0f, 0.0f, -1.0f),   // Looking in -Z
        vec3(0.0f, 1.0f, 0.0f)
    );
    Mat4 proj = mat4_perspective(deg_to_rad(60.0f), (float)W / (float)H, 0.1f, 100.0f);
    Mat4 mv = mat4_mul_mat4(view, model);
    Mat4 mvp = mat4_mul_mat4(proj, mv);

    int pixels = draw_mesh(CUBE_MESH, mvp, vp, fb);

    // Cube is behind the camera, should produce zero pixels
    CHECK(pixels == 0);
}

/* ------------------------------------------------------------------ */
/* Test: draw_mesh() with synthetic quad produces pixels                */
/* ------------------------------------------------------------------ */

static void test_draw_mesh_synthetic_quad() {
    static const Vertex3D quad_verts[] = {
        {{-1.0f,  1.0f, -3.0f}, {1.0f, 0.0f, 0.0f}},
        {{ 1.0f,  1.0f, -3.0f}, {0.0f, 1.0f, 0.0f}},
        {{ 1.0f, -1.0f, -3.0f}, {0.0f, 0.0f, 1.0f}},
        {{-1.0f, -1.0f, -3.0f}, {1.0f, 1.0f, 0.0f}},
    };
    static const uint16_t quad_indices[] = { 0, 1, 2, 0, 2, 3 };
    static const Mesh3D quad_mesh = {
        quad_verts, 4, quad_indices, 6
    };

    const int W = 160, H = 120;
    uint8_t fb_data[W * H * 3];
    std::memset(fb_data, 0, sizeof(fb_data));

    Framebuffer fb = { fb_data, W, H };
    Viewport vp = {0.0f, 0.0f, (float)W, (float)H, 0.0f, 1.0f};

    Mat4 view = mat4_look_at(
        vec3(0.0f, 0.0f, 0.0f),
        vec3(0.0f, 0.0f, -1.0f),
        vec3(0.0f, 1.0f, 0.0f)
    );
    Mat4 proj = mat4_perspective(deg_to_rad(60.0f), (float)W / (float)H, 0.1f, 100.0f);
    Mat4 mvp = mat4_mul_mat4(proj, view);

    int pixels = draw_mesh(quad_mesh, mvp, vp, fb);
    CHECK(pixels > 0);
}

/* ------------------------------------------------------------------ */
/* Test: Index count is always a multiple of 3                         */
/* ------------------------------------------------------------------ */

static void test_index_count_multiple_of_3() {
    CHECK(CUBE_MESH.index_count % 3 == 0);
    CHECK(PYRAMID_MESH.index_count % 3 == 0);
    CHECK(ICOSPHERE_MESH.index_count % 3 == 0);
}

/* ------------------------------------------------------------------ */
/* Test: Mesh3D pointers reference const static data (zero-allocation) */
/* ------------------------------------------------------------------ */

static void test_mesh_const_data() {
    // Verify that the mesh struct points to the static arrays
    CHECK(CUBE_MESH.vertices == CUBE_VERTICES);
    CHECK(CUBE_MESH.indices == CUBE_INDICES);
    CHECK(PYRAMID_MESH.vertices == PYRAMID_VERTICES);
    CHECK(PYRAMID_MESH.indices == PYRAMID_INDICES);
    CHECK(ICOSPHERE_MESH.vertices == ICOSPHERE_VERTICES);
    CHECK(ICOSPHERE_MESH.indices == ICOSPHERE_INDICES);
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main() {
    printf("Running ESR Mesh Pipeline unit tests...\n");

    test_cube_index_bounds();
    test_pyramid_index_bounds();
    test_icosphere_index_bounds();
    test_synthetic_quad_indexing();
    test_draw_mesh_renders_cube();
    test_draw_mesh_clipped();
    test_draw_mesh_synthetic_quad();
    test_index_count_multiple_of_3();
    test_mesh_const_data();

    printf("ESR Mesh tests completed: %d checks, %d failed.\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
