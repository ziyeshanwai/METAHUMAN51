// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/utils/Profiler.h>
#include <nls/DiffDataMatrix.h>
#include <nls/geometry/VertexConstraints.h>
#include <carbon/utils/Timer.h>

namespace epic::nls {

/**
 * Applies the vertex constraints @p vertexConstraints to @p vertices. VertexConstraints store the
 * constraints as constraint = residual + Jacobian * dx. Hence we need to pass @p baseVertices
 * so that the correct residual for the vertex constraints can be calculated.
 */
template <class T, int ResidualSize, int NumConstraintVertices>
DiffData<T> ApplyVertexConstraints(const DiffDataMatrix<T, 3, -1>& vertices,
                                   const Eigen::Matrix<T, 3, -1>& baseVertices,
                                   const VertexConstraints<T, ResidualSize, NumConstraintVertices>& vertexConstraints)
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    VectorPtr<T> output = std::make_shared<Vector<T>>(vertexConstraints.EvaluateResidual(vertices.Matrix(), baseVertices));

    JacobianConstPtr<T> jacobian;
    if (vertices.HasJacobian()) {
        PROFILING_BLOCK("jacobian");
        jacobian = vertices.Jacobian().Premultiply(vertexConstraints.SparseJacobian(vertices.Cols()));
        PROFILING_END_BLOCK;
    }
    return DiffData<T>(output, jacobian);
}

}  // namespace epic::nls
