// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/DiffDataMatrix.h>
#include <nls/math/Math.h>

namespace epic {
namespace nls {


/**
 * Triangle bending model. The triangle bending model creates bending constraints for each pair of triangles that share an
 * edge. The goal is to keep the dihedral angle for the rest and deformed position consistent. The class implements
 * two different models based on a quadratic bending energy as well as measuring the dihedral error directly. See
 * EvaluateQuadratic() and EvaluateDihedral().
 *
 * The bending model is rotation and translation invariant. EvaluateQuadratic() is scale invariant in the sense that the energy is the same no matter how you scale the
 * rest and deformed vertices, but the scale needs to be consistent for both meshes. EvaluateDihedral() is scale invariant
 * in the sense that both rest vertices and deformed vertices can have completely different scales.
 *
 * Note regarding scale invariance: you need to be careful when integrating the bending constraint with other constraints
 * that may not be scale invariant (e.g. point to point constraint) or otherwise an energy minimization will result in
 * different results depending on the global scale of the data. You can anchieve this by globally modify the bending energy
 * by passing in a global bending weight.
 *
 * Note that testing on spherical triangle meshes has shown that the bending energy is not equal to the integrated
 * squared mean curvature. TODO: figure out why this is not the case.
 *
 * Implemented for T=float and T=double.
 */
template <class T>
class TriangleBending
{
public:
  	TriangleBending() = default;
	TriangleBending(const TriangleBending& o) = delete;
	TriangleBending& operator=(const TriangleBending& o) = delete;

    /**
     * Sets the triangles/topology for which to calculate the bending terms.
     */
    void SetTopology(const Eigen::Matrix<int, 3, -1>& triangles);


    //! @returns the number of bending constraints i.e. the number of neighbouring triangles that share an edge.
    int NumBendingTerms() const { return int(m_trianglePairs.cols()); }

    /**
     * Initializes the rest position for the bending constraints. Sets up the base data for both Quadratic and Dihedral
     * bending constraints.
     */
    void SetRestPose(const Eigen::Matrix<T, 3, -1>& vertices);

    /**
     * Quadratic bending energy (energy = ||K x||_2^2) as described in
     * Bergou et al, "A Quadratic Bending Model for Inextensible Surfaces", see Implementation Details
     * as well as  Wardetzky et al, "Discrete Quadratic Curvature Energies", where Equation (7) is equivalent to the cotan formulation.
     * The latter shows the relationship to the Dihedral error.
     *
     * The main issue of the quadratic bending model is that it is only accurate for inextensible surface i.e. surfaces that cannot
     * strain which is why the model is not scale invariant for different rest and deformed scales.
     * Also the model cannot differentiate between convex and concave curvature.
     * The advantage of the implementation is that the Jacobian is constant, though the Jacobian is also not fully
     * correct as it does not take the normalization of the target vector into account. (see implementation for details)
     *
     * @param vertices       The current deformed vertex configuration.
     * @param bendingWeight  A global weighting factor: energy = bendingWeight ||K x||_2^2
     */
    DiffData<T> EvaluateQuadratic(const DiffDataMatrix<T, 3, -1>& vertices, const T bendingWeight) const;

    /**
     * Dihedral bending energy (energy = (theta - theta_rest)^2 Le^2/Ae) as described in
     * Fröhlich and Botsch, "Example-Driven Deformations Based on Discrete Shells" and
     * Grinspun et al, "Discrete Shells", in each case Equation (2).
     * The advantage over the quadratic model is that it optimizes the dihedral angle of neighbouring triangles directly
     * and it is not influenced by any other in-plane deformation of the triangles. However, the cost is that
     *
     * @param vertices       The current deformed vertex configuration.
     * @param bendingWeight  A global weighting factor.
     */
    DiffData<T> EvaluateDihedral(const DiffDataMatrix<T, 3, -1>& vertices, const T bendingWeight) const;

    //! Calculates the dihedral angle for all neighbouring triangle pairs. Output size is NumBendingTerms().
    Eigen::VectorX<T> EvaluateAngles(const Eigen::Matrix<T, 3, -1>& vertices) const;

    //! @returns The rest bending energy as calculated using th quadratic bending model
    const Eigen::VectorX<T>& RestBendingEnergy() const { return m_restBendingEnergy; }

    const Eigen::VectorX<T>& RestEdgeAreaSqrt() const { return m_restEdgeAreaSqrt; }
    const Eigen::VectorX<T>& RestAngle() const { return m_restAngle; }
    const Eigen::VectorX<T>& RestEdgeLength() const { return m_restEdgeLength; }

    const Eigen::Matrix<int, 4, -1> TrianglePairs() const { return m_trianglePairs; }

    //! Evaluate the quadratic bending per edgepair, and then assign for each vertex the maximum bending value
    Eigen::VectorX<T> EvaluateQuadraticPerVertex(const DiffDataMatrix<T, 3, -1>& vertices) const;

    //! Evaluate the diahedral bending per edgepair, and then assign for each vertex the maximum bending value
    Eigen::VectorX<T> EvaluateDihedralPerVertex(const DiffDataMatrix<T, 3, -1>& vertices) const;

private:
    int m_numVertices;

    Eigen::Matrix<int, 3, -1> m_triangles;
    Eigen::Matrix<int, 4, -1> m_trianglePairs;

    Eigen::Matrix<T, 4, -1> m_K;
    Eigen::VectorX<T> m_restBendingEnergy;

    Eigen::VectorX<T> m_restAngle;
    Eigen::VectorX<T> m_restEdgeAreaSqrt;
    Eigen::VectorX<T> m_restOneOverEdgeAreaSqrt;
    Eigen::VectorX<T> m_restEdgeLength;
};


} // namespace nls
} //namespace epic
