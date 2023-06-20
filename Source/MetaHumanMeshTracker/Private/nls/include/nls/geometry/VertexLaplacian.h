// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/DiffDataMatrix.h>
#include <nls/math/Math.h>
#include <nls/geometry/Mesh.h>


namespace epic {
namespace nls {


/**
 * Class to evaluate the classic cotan Laplacian on the vertices of a triangle mesh i.e. E = || Lx - Lx0 ||_2^2.
 *
 * Implemented for T=float and T=double.
 */
template <class T>
class VertexLaplacian
{
public:
    enum class Type {
        UNIFORM,  // uniform laplacian
        COTAN,    // laplacian using cotangent weights
        MEANVALUE
    };

public:
  	VertexLaplacian() = default;
	VertexLaplacian(const VertexLaplacian& o) = delete;
	VertexLaplacian& operator=(const VertexLaplacian& o) = delete;

    /**
     * Sets the rest pose i.e. defines the Laplacian matrix with weights depending on @p type
     * For all types, the resulting Laplacian weights are normalized and divided by the squared root of the vertex area.
     * This means that the resulting vertex laplacian is scale invariant.
     */
    void SetRestPose(const Mesh<T>& mesh, Type type = Type::COTAN);

    /**
     * Evaluate Ldx (instead of Lx - Lx0)
     * @param vertexOffset   The current deformed vertex configuration.
     * @param laplacianWeight  A global weighting factor: energy = laplacianWeight ||L dx||_2^2
     */
    DiffDataMatrix<T, 3, -1> EvaluateLaplacianOnOffsets(const DiffDataMatrix<T, 3, -1>& vertexOffsets, const T laplacianWeight) const;


    /**
     * @brief Calculates the laplacian matrix
     * @param dim Set the output dimension i.e. 3 if we use a laplacian per vertex
     */
    static SparseMatrix<T> LaplacianMatrix(const Mesh<T>& mesh, Type type = Type::COTAN, int dim = 1);

private:
    SparseMatrix<T> m_laplacianMatrix;
};


} // namespace nls
} //namespace epic
