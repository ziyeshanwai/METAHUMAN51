// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/DiffDataMatrix.h>
#include <nls/math/Math.h>
#include <carbon/utils/Profiler.h>

namespace epic {
namespace nls {

/**
 * Function to calculate the point surface constraint: residual(x) = sqrt(wNormal) * w * normal.dot(v(x) - target)
 */
template <class T, int C>
class PointSurfaceConstraintFunction
{
public:
    static DiffData<T> Evaluate(const DiffDataMatrix<T, C, -1>& v,
                                const Eigen::Matrix<T, C, -1>& targets,
                                const Eigen::Matrix<T, C, -1>& normals,
                                const Vector<T>& weights,
                                const T wNormal)
	{
        PROFILING_FUNCTION(PROFILING_COLOR_CYAN);

        if (v.Cols() != int(targets.cols())) {
            throw std::runtime_error("point surface constraint: number of vertices and targets not matching");
        }
        if (v.Cols() != int(normals.cols())) {
            throw std::runtime_error("point surface constraint: number of vertices and target normals not matching");
        }
        if (v.Cols() != int(weights.size())) {
            throw std::runtime_error("point surface constraint: number of vertices and weights not matching");
        }

        const T sqrtwNormal = std::sqrt(wNormal);

        const int numConstraints = v.Cols();
        VectorPtr<T> residual = std::make_shared<Vector<T>>(v.Cols());
        for (int i = 0; i < numConstraints; i++) {
            (*residual)[i] = sqrtwNormal * weights[i] * normals.col(i).dot(v.Matrix().col(i) - targets.col(i));
        }

		JacobianConstPtr<T> Jacobian;
		if (v.HasJacobian()) {
            PROFILING_BLOCK("jacobian (1)");
            std::vector<Eigen::Triplet<T>> triplets;
            triplets.reserve(C * numConstraints);
            for (int i = 0; i < numConstraints; i++) {
                for (int k = 0; k < C; k++) {
                    triplets.push_back(Eigen::Triplet<T>(i, C * i + k, sqrtwNormal * weights[i] * normals(k,i)));
                }
            }
			SparseMatrix<T> J(numConstraints, C * numConstraints);
            J.setFromTriplets(triplets.begin(), triplets.end());
            PROFILING_END_BLOCK;

            PROFILING_BLOCK("jacobian (2)");
            Jacobian = v.Jacobian().Premultiply(J);
            PROFILING_END_BLOCK;
		}

		return DiffData<T>(residual, Jacobian);
	}

    static DiffData<T> Evaluate(const DiffDataMatrix<T, C, -1>& v,
                                const Eigen::VectorXi& indices,
                                const Eigen::Matrix<T, C, -1>& targets,
                                const Eigen::Matrix<T, C, -1>& normals,
                                const Vector<T>& weights,
                                const T wNormal)
	{
        PROFILING_FUNCTION(PROFILING_COLOR_CYAN);

        if (int(indices.size()) != int(targets.cols())) {
            throw std::runtime_error("point surface constraint: number of vertices and targets not matching");
        }
        if (int(indices.size()) != int(normals.cols())) {
            throw std::runtime_error("point surface constraint: number of vertices and target normals not matching");
        }
        if (int(indices.size()) != int(weights.size())) {
            throw std::runtime_error("point surface constraint: number of vertices and weights not matching");
        }

        const T sqrtwNormal = std::sqrt(wNormal);

        const int numConstraints = int(indices.size());
        VectorPtr<T> residual = std::make_shared<Vector<T>>(numConstraints);
        for (int i = 0; i < numConstraints; i++) {
            (*residual)[i] = sqrtwNormal * weights[i] * normals.col(i).dot(v.Matrix().col(indices[i]) - targets.col(i));
        }

		JacobianConstPtr<T> Jacobian;
		if (v.HasJacobian()) {
            PROFILING_BLOCK("jacobian (1)");
            std::vector<Eigen::Triplet<T>> triplets;
            triplets.reserve(C * numConstraints);
            for (int i = 0; i < numConstraints; i++) {
                for (int k = 0; k < C; k++) {
                    triplets.push_back(Eigen::Triplet<T>(i, C * indices[i] + k, sqrtwNormal * weights[i] * normals(k,i)));
                }
            }
			SparseMatrix<T> J(numConstraints, v.Size());
            J.setFromTriplets(triplets.begin(), triplets.end());
            PROFILING_END_BLOCK;

            PROFILING_BLOCK("jacobian (2)");
            Jacobian = v.Jacobian().Premultiply(J);
            PROFILING_END_BLOCK;
		}

		return DiffData<T>(residual, Jacobian);
	}
};


} // namespace nls
} //namespace epic
