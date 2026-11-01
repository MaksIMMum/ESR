/**
 * Unit tests for include/esr/esr_math.h
 */
#include <stdio.h>
#include <math.h>

#include "esr/esr_math.h"

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

static bool feq(float a, float b, float eps = EPS) { return fabsf(a - b) <= eps; }

static bool vec3_eq(Vec3 a, Vec3 b) { return feq(a.x, b.x) && feq(a.y, b.y) && feq(a.z, b.z); }

static bool vec4_eq(Vec4 a, Vec4 b) {
    return feq(a.x, b.x) && feq(a.y, b.y) && feq(a.z, b.z) && feq(a.w, b.w);
}

static bool mat4_eq(const Mat4& a, const Mat4& b) {
    for (int i = 0; i < 16; ++i) {
        if (!feq(a.m[i], b.m[i])) return false;
    }
    return true;
}

/* ---------------------------- Vec3 ---------------------------- */

static void test_vec3_add_sub() {
    const Vec3 a = vec3(1, 2, 3), b = vec3(4, -5, 6);
    CHECK(vec3_eq(vec3_add(a, b), vec3(5, -3, 9)));
    CHECK(vec3_eq(vec3_sub(a, b), vec3(-3, 7, -3)));
    CHECK(vec3_eq(vec3_sub(a, a), vec3(0, 0, 0)));
}

static void test_vec3_dot() {
    CHECK(feq(vec3_dot(vec3(1, 2, 3), vec3(4, -5, 6)), 12.0f));
    CHECK(feq(vec3_dot(vec3(1, 0, 0), vec3(0, 1, 0)), 0.0f)); /* perpendicular */
    CHECK(feq(vec3_dot(vec3(2, 0, 0), vec3(3, 0, 0)), 6.0f)); /* parallel */
}

static void test_vec3_cross() {
    const Vec3 x = vec3(1, 0, 0), y = vec3(0, 1, 0), z = vec3(0, 0, 1);
    /* right-handed basis */
    CHECK(vec3_eq(vec3_cross(x, y), z));
    CHECK(vec3_eq(vec3_cross(y, z), x));
    CHECK(vec3_eq(vec3_cross(z, x), y));
    /* anti-commutative */
    CHECK(vec3_eq(vec3_cross(y, x), vec3(0, 0, -1)));
    /* parallel vectors -> zero */
    CHECK(vec3_eq(vec3_cross(x, vec3(5, 0, 0)), vec3(0, 0, 0)));
    /* result is perpendicular to both inputs */
    const Vec3 a = vec3(1, 2, 3), b = vec3(-2, 0.5f, 4);
    const Vec3 c = vec3_cross(a, b);
    CHECK(feq(vec3_dot(c, a), 0.0f));
    CHECK(feq(vec3_dot(c, b), 0.0f));
}

static void test_vec3_normalize() {
    const Vec3 n = vec3_normalize(vec3(3, 0, 4));
    CHECK(vec3_eq(n, vec3(0.6f, 0.0f, 0.8f)));
    CHECK(feq(vec3_length(n), 1.0f));
    /* zero vector must not produce NaN */
    const Vec3 z = vec3_normalize(vec3(0, 0, 0));
    CHECK(vec3_eq(z, vec3(0, 0, 0)));
    CHECK(!isnan(z.x) && !isnan(z.y) && !isnan(z.z));
}

/* ---------------------------- Mat4 ---------------------------- */

static void test_mat4_identity_mul() {
    const Mat4 i = mat4_identity();
    const Vec4 v = vec4(1, 2, 3, 1);
    CHECK(vec4_eq(mat4_mul_vec4(i, v), v));

    const Mat4 t = mat4_translate(vec3(1, 2, 3));
    CHECK(mat4_eq(mat4_mul_mat4(i, t), t));
    CHECK(mat4_eq(mat4_mul_mat4(t, i), t));
}

static void test_mat4_translate() {
    const Mat4 t = mat4_translate(vec3(1, 2, 3));
    /* a point (w = 1) is moved */
    CHECK(vec4_eq(mat4_mul_vec4(t, vec4(1, 1, 1, 1)), vec4(2, 3, 4, 1)));
    /* a direction (w = 0) is NOT moved */
    CHECK(vec4_eq(mat4_mul_vec4(t, vec4(1, 1, 1, 0)), vec4(1, 1, 1, 0)));
}

static void test_mat4_scale() {
    const Mat4 s = mat4_scale(vec3(2, 3, 4));
    CHECK(vec4_eq(mat4_mul_vec4(s, vec4(1, 1, 1, 1)), vec4(2, 3, 4, 1)));
}

static void test_mat4_rotations() {
    const float quarter = ESR_PI * 0.5f;
    /* Counter-clockwise by 90 degrees, right-handed. */
    CHECK(vec4_eq(mat4_mul_vec4(mat4_rotate_z(quarter), vec4(1, 0, 0, 1)), vec4(0, 1, 0, 1)));
    CHECK(vec4_eq(mat4_mul_vec4(mat4_rotate_x(quarter), vec4(0, 1, 0, 1)), vec4(0, 0, 1, 1)));
    CHECK(vec4_eq(mat4_mul_vec4(mat4_rotate_y(quarter), vec4(0, 0, 1, 1)), vec4(1, 0, 0, 1)));

    /* The rotation axis itself stays fixed. */
    CHECK(vec4_eq(mat4_mul_vec4(mat4_rotate_x(1.0f), vec4(1, 0, 0, 1)), vec4(1, 0, 0, 1)));
    CHECK(vec4_eq(mat4_mul_vec4(mat4_rotate_y(1.0f), vec4(0, 1, 0, 1)), vec4(0, 1, 0, 1)));
    CHECK(vec4_eq(mat4_mul_vec4(mat4_rotate_z(1.0f), vec4(0, 0, 1, 1)), vec4(0, 0, 1, 1)));

    /* Rotation preserves length. */
    const Vec4 r = mat4_mul_vec4(mat4_rotate_y(0.7f), vec4(1, 2, 3, 1));
    CHECK(feq(vec3_length(vec3(r.x, r.y, r.z)), vec3_length(vec3(1, 2, 3)), 1e-4f));

    /* Rotating by +a and then -a gives identity. */
    CHECK(mat4_eq(mat4_mul_mat4(mat4_rotate_x(0.9f), mat4_rotate_x(-0.9f)), mat4_identity()));
    CHECK(mat4_eq(mat4_mul_mat4(mat4_rotate_y(0.9f), mat4_rotate_y(-0.9f)), mat4_identity()));
    CHECK(mat4_eq(mat4_mul_mat4(mat4_rotate_z(0.9f), mat4_rotate_z(-0.9f)), mat4_identity()));
}

static void test_mat4_mul_order() {
    const Mat4 t = mat4_translate(vec3(1, 0, 0));
    const Mat4 s = mat4_scale(vec3(2, 2, 2));
    const Vec4 p = vec4(1, 0, 0, 1);

    /* T * S: scale first -> (2,0,0), then translate -> (3,0,0) */
    CHECK(vec4_eq(mat4_mul_vec4(mat4_mul_mat4(t, s), p), vec4(3, 0, 0, 1)));
    /* S * T: translate first -> (2,0,0), then scale -> (4,0,0) */
    CHECK(vec4_eq(mat4_mul_vec4(mat4_mul_mat4(s, t), p), vec4(4, 0, 0, 1)));

    /* (A*B)*v == A*(B*v) */
    const Mat4 r = mat4_rotate_z(0.5f);
    const Vec4 lhs = mat4_mul_vec4(mat4_mul_mat4(r, t), p);
    const Vec4 rhs = mat4_mul_vec4(r, mat4_mul_vec4(t, p));
    CHECK(vec4_eq(lhs, rhs));
}

/* ------------------------ Camera / projection ------------------------ */

static void test_mat4_look_at() {
    /* Camera at (0,0,5) looking at the origin: the world just shifts by -5 in z. */
    const Mat4 v = mat4_look_at(vec3(0, 0, 5), vec3(0, 0, 0), vec3(0, 1, 0));
    CHECK(mat4_eq(v, mat4_translate(vec3(0, 0, -5))));
    CHECK(vec4_eq(mat4_mul_vec4(v, vec4(0, 0, 0, 1)), vec4(0, 0, -5, 1)));

    /* The eye itself always maps to the view-space origin. */
    const Vec3 eye = vec3(3, 4, 5);
    const Mat4 v2 = mat4_look_at(eye, vec3(-1, 2, 0), vec3(0, 1, 0));
    CHECK(vec4_eq(mat4_mul_vec4(v2, vec4_from_vec3(eye, 1.0f)), vec4(0, 0, 0, 1)));

    /* The target lies on the -Z axis of view space, at distance |target - eye|. */
    const Vec3 target = vec3(-1, 2, 0);
    const Vec4 t = mat4_mul_vec4(v2, vec4_from_vec3(target, 1.0f));
    CHECK(feq(t.x, 0.0f, 1e-4f));
    CHECK(feq(t.y, 0.0f, 1e-4f));
    CHECK(feq(t.z, -vec3_length(vec3_sub(target, eye)), 1e-4f));

    /* A point to the right of the camera ends up at +X in view space. */
    const Vec4 right = mat4_mul_vec4(v, vec4(1, 0, 0, 1));
    CHECK(right.x > 0.0f);
}

static void test_mat4_perspective() {
    const float n = 1.0f, f = 10.0f;
    const Mat4 p = mat4_perspective(ESR_PI * 0.5f, 1.0f, n, f); /* 90 deg, square */

    /* w_clip = -z_view */
    const Vec4 c = mat4_mul_vec4(p, vec4(0, 0, -4, 1));
    CHECK(feq(c.w, 4.0f));

    /* Point on the near plane -> NDC z = -1, far plane -> NDC z = +1. */
    const Vec4 cn = mat4_mul_vec4(p, vec4(0, 0, -n, 1));
    const Vec4 cf = mat4_mul_vec4(p, vec4(0, 0, -f, 1));
    CHECK(feq(cn.z / cn.w, -1.0f));
    CHECK(feq(cf.z / cf.w, 1.0f, 1e-4f));

    /* With 90 deg fov and aspect 1, the point (1,1,-1) lies on the frustum edge: NDC (1,1). */
    const Vec4 e = mat4_mul_vec4(p, vec4(1, 1, -1, 1));
    CHECK(feq(e.x / e.w, 1.0f));
    CHECK(feq(e.y / e.w, 1.0f));

    /* Farther objects look smaller (perspective division). */
    const Vec4 near_pt = mat4_mul_vec4(p, vec4(1, 0, -2, 1));
    const Vec4 far_pt = mat4_mul_vec4(p, vec4(1, 0, -8, 1));
    CHECK(fabsf(near_pt.x / near_pt.w) > fabsf(far_pt.x / far_pt.w));

    /* Aspect ratio 2: x is squeezed by 2 compared with y. */
    const Mat4 p2 = mat4_perspective(ESR_PI * 0.5f, 2.0f, n, f);
    const Vec4 a = mat4_mul_vec4(p2, vec4(1, 1, -1, 1));
    CHECK(feq(a.x / a.w, 0.5f));
    CHECK(feq(a.y / a.w, 1.0f));
}

static void test_full_pipeline() {
    /* P * V * M * v for a vertex at the model origin, moved to (0,0,0) world,
       camera 3 units back: it must land in the middle of the screen. */
    const Mat4 model = mat4_translate(vec3(0, 0, 0));
    const Mat4 view = mat4_look_at(vec3(0, 0, 3), vec3(0, 0, 0), vec3(0, 1, 0));
    const Mat4 proj = mat4_perspective(deg_to_rad(45.0f), 800.0f / 600.0f, 0.1f, 100.0f);
    const Mat4 mvp = mat4_mul_mat4(proj, mat4_mul_mat4(view, model));
    const Vec4 clip = mat4_mul_vec4(mvp, vec4(0, 0, 0, 1));
    CHECK(feq(clip.x / clip.w, 0.0f));
    CHECK(feq(clip.y / clip.w, 0.0f));
    CHECK(feq(clip.w, 3.0f));
    CHECK(fabsf(clip.z / clip.w) <= 1.0f); /* inside the near/far range */
}

int main() {
    test_vec3_add_sub();
    test_vec3_dot();
    test_vec3_cross();
    test_vec3_normalize();
    test_mat4_identity_mul();
    test_mat4_translate();
    test_mat4_scale();
    test_mat4_rotations();
    test_mat4_mul_order();
    test_mat4_look_at();
    test_mat4_perspective();
    test_full_pipeline();

    printf("%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
