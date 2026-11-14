#ifndef MODEL_CUBE_H
#define MODEL_CUBE_H

#include "esr/esr_pipeline.hpp"
#include "esr/esr_mesh.hpp"

static const esr::Vertex3D CUBE_VERTICES[] = {
    {{-1.000000f,  1.000000f,  1.000000f}, {0.000000f, 1.000000f, 1.000000f}},
    {{ 1.000000f,  1.000000f,  1.000000f}, {1.000000f, 1.000000f, 1.000000f}},
    {{ 1.000000f, -1.000000f,  1.000000f}, {1.000000f, 0.000000f, 1.000000f}},
    {{-1.000000f, -1.000000f,  1.000000f}, {0.000000f, 0.000000f, 1.000000f}},
    {{-1.000000f,  1.000000f, -1.000000f}, {0.000000f, 1.000000f, 0.000000f}},
    {{ 1.000000f,  1.000000f, -1.000000f}, {1.000000f, 1.000000f, 0.000000f}},
    {{ 1.000000f, -1.000000f, -1.000000f}, {1.000000f, 0.000000f, 0.000000f}},
    {{-1.000000f, -1.000000f, -1.000000f}, {0.200000f, 0.200000f, 0.200000f}}
};

static const uint16_t CUBE_INDICES[] = {
    0, 1, 2,
    0, 2, 3,
    5, 4, 7,
    5, 7, 6,
    4, 5, 1,
    4, 1, 0,
    3, 2, 6,
    3, 6, 7,
    1, 5, 6,
    1, 6, 2,
    4, 0, 3,
    4, 3, 7
};

static const esr::Mesh3D CUBE_MESH = {
    CUBE_VERTICES,
    sizeof(CUBE_VERTICES) / sizeof(CUBE_VERTICES[0]),
    CUBE_INDICES,
    sizeof(CUBE_INDICES) / sizeof(CUBE_INDICES[0])
};

#endif // MODEL_CUBE_H
