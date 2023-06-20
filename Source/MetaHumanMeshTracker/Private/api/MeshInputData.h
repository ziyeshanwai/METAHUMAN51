// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Defs.h"

namespace titan {
namespace api {

struct TITAN_API MeshInputData {
    const int m_numTriangles;
    const int* m_triangles;
    const int m_numVertices;
    const float* m_vertices;
};

}
}
