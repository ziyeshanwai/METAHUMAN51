// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/DiffDataMatrix.h>
#include <nls/math/Math.h>
#include <carbon/utils/Profiler.h>

namespace epic {
namespace nls {

template <class T, int C, typename DISCARD = typename std::enable_if<(C == 2 || C ==3), void>::type>
class PointPointConstraintFunction
{
public:
    /**
     * Function to calculate the an point-point constraint:
     * residual(x) = sqrt(wPoint2Point) * w * (v(x) - target)
     */
	static DiffData<T> Evaluate(const DiffDataMatrix<T, C, -1>& v,
                                const Eigen::Matrix<T, C, -1>& targets,
                                const Vector<T>& weights,
                                const T wPoint2Point)
	{
        PROFILING_FUNCTION(PROFILING_COLOR_CYAN);

        if (v.Cols() != int(targets.cols())) {
            throw std::runtime_error("point point constraint: number of vertices and targets not matching");
        }
        if (v.Cols() != int(weights.size())) {
            throw std::runtime_error("point point constraint: number of vertices and weights not matching");
        }

        const T sqrtwPoint2Point = std::sqrt(wPoint2Point);

        const int numConstraints = v.Cols();
        VectorPtr<T> residual = std::make_shared<Vector<T>>(C * numConstraints);
        for (int i = 0; i < numConstraints; i++) {
            for (int k = 0; k < C; k++) {
                (*residual)[C * i + k] = sqrtwPoint2Point * weights[i] * (v.Matrix()(k, i) - targets(k, i));
            }
        }

		JacobianConstPtr<T> Jacobian;
		if (v.HasJacobian()) {
            PROFILING_BLOCK("jacobian (1)");
            std::vector<Eigen::Triplet<T>> triplets;
            triplets.reserve(C * numConstraints);
            for (int i = 0; i < numConstraints; i++) {
                for (int k = 0; k < C; k++) {
                    triplets.push_back(Eigen::Triplet<T>(C * i + k, C * i + k, sqrtwPoint2Point * weights[i]));
                }
            }

			SparseMatrix<T> J(C * numConstraints, C * numConstraints);
            J.setFromTriplets(triplets.begin(), triplets.end());
            PROFILING_END_BLOCK;

            PROFILING_BLOCK("jacobian (2)");
            Jacobian = v.Jacobian().Premultiply(J);
            PROFILING_END_BLOCK;
		}

		return DiffData<T>(residual, Jacobian);
	}


    /**
     * Function to calculate the an point-point constraint:
     * residual(x) = sqrt(wPoint2Point) * w * (v(x) - target)
     * Additional indices vector specifying on which vertices to evaluate the constraints. In this case the targets and weights need to have the samme number of points as the size of the indices vector.
     */
	static DiffData<T> Evaluate(const DiffDataMatrix<T, C, -1>& v,
                                const Eigen::VectorXi& indices,
                                const Eigen::Matrix<T, C, -1>& targets,
                                const Vector<T>& weights,
                                const T wPoint2Point)
	{
        PROFILING_FUNCTION(PROFILING_COLOR_CYAN);

        const int numConstraints = int(indices.size());

        if (numConstraints != int(targets.cols())) {
            throw std::runtime_error("point point constraint: number of vertices and targets not matching");
        }
        if (numConstraints != int(weights.size())) {
            throw std::runtime_error("point point constraint: number of vertices and weights not matching");
        }

        const T sqrtwPoint2Point = std::sqrt(wPoint2Point);

        VectorPtr<T> residual = std::make_shared<Vector<T>>(C * numConstraints);
        for (int i = 0; i < numConstraints; i++) {
            for (int k = 0; k < C; k++) {
                (*residual)[C * i + k] = sqrtwPoint2Point * weights[i] * (v.Matrix()(k, indices[i]) - targets(k, i));
            }
        }

		JacobianConstPtr<T> Jacobian;
		if (v.HasJacobian()) {
            PROFILING_BLOCK("jacobian (1)");
            std::vector<Eigen::Triplet<T>> triplets;
            triplets.reserve(C * numConstraints);
            for (int i = 0; i < numConstraints; i++) {
                for (int k = 0; k < C; k++) {
                    triplets.push_back(Eigen::Triplet<T>(C * i + k, C * indices[i] + k, sqrtwPoint2Point * weights[i]));
                }
            }

			SparseMatrix<T> J(C * numConstraints, v.Size());
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
