/**
 * esr_math.h - Embedded Software Rasterizer: 3D vector & matrix math.
 */
#ifndef ESR_MATH_H
#define ESR_MATH_H

#include <math.h>

namespace esr {

/* ------------------------------------------------------------------ */
/* Constants and scalar helpers                                         */
/* ------------------------------------------------------------------ */

constexpr float ESR_PI = 3.14159265358979323846f;

inline float deg_to_rad(float degrees) { return degrees * (ESR_PI / 180.0f); }
inline float rad_to_deg(float radians) { return radians * (180.0f / ESR_PI); }

/* ------------------------------------------------------------------ */
/* Types                                                                */
/* ------------------------------------------------------------------ */

struct Vec3 {
    float x, y, z;
};

struct Vec4 {
    float x, y, z, w;
};

/** 4x4 matrix, column-major: element (row, col) = m[col * 4 + row]. */
struct Mat4 {
    float m[16];
};

/* ------------------------------------------------------------------ */
/* Constructors                                                         */
/* ------------------------------------------------------------------ */

inline Vec3 vec3(float x, float y, float z) { return Vec3{x, y, z}; }
inline Vec4 vec4(float x, float y, float z, float w) { return Vec4{x, y, z, w}; }
/** Build a Vec4 from a Vec3. w = 1 for points, w = 0 for directions. */
inline Vec4 vec4_from_vec3(Vec3 v, float w) { return Vec4{v.x, v.y, v.z, w}; }

/* ------------------------------------------------------------------ */
/* Vec3 operations                                                      */
/* ------------------------------------------------------------------ */

inline Vec3 vec3_add(Vec3 a, Vec3 b) { return Vec3{a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 vec3_sub(Vec3 a, Vec3 b) { return Vec3{a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 vec3_scale(Vec3 v, float s) { return Vec3{v.x * s, v.y * s, v.z * s}; }

inline float vec3_dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vec3 vec3_cross(Vec3 a, Vec3 b) {
    return Vec3{a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
}

inline float vec3_length(Vec3 v) { return sqrtf(vec3_dot(v, v)); }

/** Returns a unit-length copy of v. A (near-)zero vector gives (0,0,0), never NaN. */
inline Vec3 vec3_normalize(Vec3 v) {
    const float len = vec3_length(v);
    if (len < 1e-8f) {
        return Vec3{0.0f, 0.0f, 0.0f};
    }
    const float inv = 1.0f / len;
    return Vec3{v.x * inv, v.y * inv, v.z * inv};
}

/* ------------------------------------------------------------------ */
/* Mat4 basics                                                          */
/* ------------------------------------------------------------------ */

inline Mat4 mat4_identity() {
    Mat4 r = {{1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1}};
    return r;
}

/** r = m * v (column vector). */
inline Vec4 mat4_mul_vec4(const Mat4& m, Vec4 v) {
    return Vec4{
        m.m[0] * v.x + m.m[4] * v.y + m.m[8]  * v.z + m.m[12] * v.w,
        m.m[1] * v.x + m.m[5] * v.y + m.m[9]  * v.z + m.m[13] * v.w,
        m.m[2] * v.x + m.m[6] * v.y + m.m[10] * v.z + m.m[14] * v.w,
        m.m[3] * v.x + m.m[7] * v.y + m.m[11] * v.z + m.m[15] * v.w};
}

/**
 * r = a * b. The transform of b is applied FIRST, then the transform of a:
 * (a * b) * v == a * (b * v).
 */
inline Mat4 mat4_mul_mat4(const Mat4& a, const Mat4& b) {
    Mat4 r;
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k) {
                sum += a.m[k * 4 + row] * b.m[col * 4 + k];
            }
            r.m[col * 4 + row] = sum;
        }
    }
    return r;
}

/* ------------------------------------------------------------------ */
/* Affine transforms                                                    */
/* ------------------------------------------------------------------ */

inline Mat4 mat4_translate(Vec3 t) {
    Mat4 r = mat4_identity();
    r.m[12] = t.x;
    r.m[13] = t.y;
    r.m[14] = t.z;
    return r;
}

inline Mat4 mat4_scale(Vec3 s) {
    Mat4 r = mat4_identity();
    r.m[0]  = s.x;
    r.m[5]  = s.y;
    r.m[10] = s.z;
    return r;
}

/** Rotation around the X axis by `angle` radians (counter-clockwise looking down +X). */
inline Mat4 mat4_rotate_x(float angle) {
    const float c = cosf(angle), s = sinf(angle);
    Mat4 r = mat4_identity();
    r.m[5]  = c;
    r.m[6]  = s;
    r.m[9]  = -s;
    r.m[10] = c;
    return r;
}

/** Rotation around the Y axis by `angle` radians. */
inline Mat4 mat4_rotate_y(float angle) {
    const float c = cosf(angle), s = sinf(angle);
    Mat4 r = mat4_identity();
    r.m[0]  = c;
    r.m[2]  = -s;
    r.m[8]  = s;
    r.m[10] = c;
    return r;
}

/** Rotation around the Z axis by `angle` radians. */
inline Mat4 mat4_rotate_z(float angle) {
    const float c = cosf(angle), s = sinf(angle);
    Mat4 r = mat4_identity();
    r.m[0] = c;
    r.m[1] = s;
    r.m[4] = -s;
    r.m[5] = c;
    return r;
}

/* ------------------------------------------------------------------ */
/* Camera and projection                                                */
/* ------------------------------------------------------------------ */

/**
 * View matrix
 */
inline Mat4 mat4_look_at(Vec3 eye, Vec3 target, Vec3 up) {
    const Vec3 f = vec3_normalize(vec3_sub(target, eye)); /* forward */
    const Vec3 s = vec3_normalize(vec3_cross(f, up));     /* right   */
    const Vec3 u = vec3_cross(s, f);                      /* true up */

    Mat4 r = mat4_identity();
    r.m[0] = s.x;  r.m[4] = s.y;  r.m[8]  = s.z;
    r.m[1] = u.x;  r.m[5] = u.y;  r.m[9]  = u.z;
    r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z;
    r.m[12] = -vec3_dot(s, eye);
    r.m[13] = -vec3_dot(u, eye);
    r.m[14] = vec3_dot(f, eye);
    return r;
}

/**
 * Perspective projection (view space -> clip space), right-handed, NDC z in [-1, 1].
 *   fov_y  - vertical field of view in RADIANS
 *   aspect - viewport width / height
 *   near_z, far_z - distances to the near/far planes (both > 0, near_z < far_z)
 * After the multiplication, divide x, y, z by w to get NDC.
 */
inline Mat4 mat4_perspective(float fov_y, float aspect, float near_z, float far_z) {
    const float tan_half = tanf(fov_y * 0.5f);

    Mat4 r = {{0, 0, 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0}};
    r.m[0]  = 1.0f / (aspect * tan_half);
    r.m[5]  = 1.0f / tan_half;
    r.m[10] = -(far_z + near_z) / (far_z - near_z);
    r.m[11] = -1.0f;
    r.m[14] = -(2.0f * far_z * near_z) / (far_z - near_z);
    return r;
}

}

#endif
