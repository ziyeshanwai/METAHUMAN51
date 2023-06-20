// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/DiffDataMatrix.h>
#include <nls/math/Math.h>

#include <vector>

namespace epic {
namespace nls {

/**
 * Triangle deformation gradient for deformation transfer as described in
 * "Deformation Transfer for Detail-Preserving Surface Editing" by Botsch et al.
 * (however it currently does not include weighting based on the triangle area)
 *
 * The class can be used for deformation transfer in the following way:
 * Input: undeformedA, undeformedB, deformedA
 * Output: deformedB
 *
 * SetTopology(undeformedA.Triangles())
 * SetRestPose(undeformedA.Vertices())
 * G = DeformationGradient(deformedA.Vertices())
 * SetRestPose(undeformedB.Vertices())
 * minimize(G - DeformationGradient(deformedB)) // using DiffDataMatrix version
 */
template <class T>
class TriangleDeformationGradient
{
public:
  	TriangleDeformationGradient() = default;
	TriangleDeformationGradient(const TriangleDeformationGradient& o) = delete;
	TriangleDeformationGradient& operator=(const TriangleDeformationGradient& o) = delete;

    //! Set the mesh topology
    void SetTopology(const Eigen::Matrix<int, 3, -1>& triangles);

    //! Set the rest pose
    void SetRestPose(const Eigen::Matrix<T, 3, -1>& vertices);

    //! Returns the number of triangles
    int NumTriangles() const { return int(m_triangles.cols()); }

    /**
     * Calculate the 3D deformation gradient from the rest pose (@see SetRestPose()) to @p vertices
     */
    Eigen::Matrix<T, 9, -1> DeformationGradient3D(const Eigen::Matrix<T, 3, -1>& vertices) const;

    /**
     * Calculate the 3D deformation gradient from the rest pose (@see SetRestPose()) to @p vertices.
     * As described in "Deformation Transfer for Detail-Preserving Surface Editing", by Botsch et al
     * the Gradient Matrix G does not need the normal, but instead is set to be zero. The Jacobian
     * output accordingly does not need the normal.
     */
    DiffDataMatrix<T, 9, -1> DeformationGradient3D(const DiffDataMatrix<T, 3, -1>& vertices) const;

private:
    Eigen::Matrix<int, 3, -1> m_triangles;

    //! inv(RestFrame3D) per triangle - (extended by normal)
    std::vector<Eigen::Matrix<T, 3, 3>> m_invRestFrame3D;
};


} // namespace nls
} //namespace epic
