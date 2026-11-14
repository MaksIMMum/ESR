#ifndef MODEL_PYRAMID_H
#define MODEL_PYRAMID_H

#include "esr/esr_pipeline.hpp"
#include "esr/esr_mesh.hpp"

static const esr::Vertex3D PYRAMID_VERTICES[] = {
    {{ 0.000000f,  1.500000f,  0.000000f}, {0.500000f, 1.000000f, 0.500000f}},
    {{-1.000000f,  0.000000f,  1.000000f}, {0.000000f, 0.000000f, 1.000000f}},
    {{ 1.000000f,  0.000000f,  1.000000f}, {1.000000f, 0.000000f, 1.000000f}},
    {{ 1.000000f,  0.000000f, -1.000000f}, {1.000000f, 0.000000f, 0.000000f}},
    {{-1.000000f,  0.000000f, -1.000000f}, {0.200000f, 0.200000f, 0.200000f}}
};

static const uint16_t PYRAMID_INDICES[] = {
    0, 1, 2,
    0, 2, 3,
    0, 3, 4,
    0, 4, 1,
    1, 4, 3,
    1, 3, 2
};

static const esr::Mesh3D PYRAMID_MESH = {
    PYRAMID_VERTICES,
    sizeof(PYRAMID_VERTICES) / sizeof(PYRAMID_VERTICES[0]),
    PYRAMID_INDICES,
    sizeof(PYRAMID_INDICES) / sizeof(PYRAMID_INDICES[0])
};

#endif // MODEL_PYRAMID_H
