// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/DiffDataMatrix.h>
#include <nls/math/Math.h>

#include <vector>

namespace epic {
namespace nls {

/**
 * Triangle strain using projections as described in
 * Bouaziz et al, "Projective Dynamics: Fusing Constraint Projections for Fast Simulation"
 * see also https://www.shapeop.org.
 *
 * The deformation gradient from the rest triangle to the current triangle should only have a rotation component
 * but no stretching i.e. you can rotate the triangle but not stretch it.
 * Define a Frame (local coordinate system) as [v1 - v0, v2 - v0]
 * Then we get the following deformation gradient: F = CurrFrame * inv(RestFrame)
 * G should only be a rotation, so the closest valid deformation gradient F' = svd(G).U * svd(F).V.t()
 * The stretch energy is therefore || F - F' ||_2^2
 *
 * For a triangle the deformation gradient has rank 2, and therefore we can simply project the 3D coordinates to 2D:
 * RestFrame = (Proj2D * RestFrame3D)
 * CurrFrame = CurrFrame3D
 * Then F' is a 3x2 matrix.
 * The stretch energy is then || CurrFrame3D * inv(Proj2D * RestFrame3D) - F' ||_2^2
 *
 * The stretch energy is scale invariant, so typically you would weight the stretch energy scaled by the triangle area:
 * Stretch Energy = triangleArea  * || CurrFrame3D * inv(Proj2D * RestFrame3D) - F' ||_2^2
 */
template <class T>
class TriangleStrain
{
public:
  	TriangleStrain() = default;
	TriangleStrain(const TriangleStrain& o) = delete;
	TriangleStrain& operator=(const TriangleStrain& o) = delete;

    void SetTopology(const Eigen::Matrix<int, 3, -1>& triangles);

    void SetRestPose(const Eigen::Matrix<T, 3, -1>& vertices);

    DiffData<T> EvaluateProjectiveStrain(const DiffDataMatrix<T, 3, -1>& vertices, const T strainWeight) const;

    DiffData<T> EvaluateGreenStrain(const DiffDataMatrix<T, 3, -1>& vertices, const T strainWeight) const;

    const Eigen::Matrix<int, 3, -1>& Triangles() const { return m_triangles; }

    //! Evaluate the projective strain per triangle, and then assign for each vertex the maximum strain value
    Eigen::VectorX<T> EvaluateProjectiveStrainPerVertex(const DiffDataMatrix<T, 3, -1>& vertices) const;

    //! Evaluate the green strain per triangle, and then assign for each vertex the maximum strain value
    Eigen::VectorX<T> EvaluateGreenStrainPerVertex(const DiffDataMatrix<T, 3, -1>& vertices) const;

private:
    int m_numVertices;

    Eigen::Matrix<int, 3, -1> m_triangles;

    //! inv(Proj2D * RestFrame3D) per triangle
    std::vector<Eigen::Matrix<T, 2, 2>> m_invRestFrame2D;
};


} // namespace nls
} //namespace epic
