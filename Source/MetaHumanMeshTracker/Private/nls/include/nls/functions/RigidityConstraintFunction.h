// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/DiffDataMatrix.h>
#include <nls/math/Math.h>

namespace epic {
namespace nls {

/**
 * Implements the rigidity constraint as described in "Correspondence optimization for non-rigid registration of depth scans." by Hao Li et al.
 */
template <class T, int R, int C>
DiffData<T> RigidityConstraintFunction(const DiffDataMatrix<T, R, C>& mat, T weight)
{
    const Eigen::Matrix<T,R,C> linearTransform = mat.Matrix();
    constexpr int numConstraintsOrthogonal = C * (C-1) / 2;
    constexpr int numConstraintsUnitLength = C;
    const int numConstraintsTotal = numConstraintsOrthogonal + numConstraintsUnitLength;
    VectorPtr<T> result = std::make_shared<Vector<T>>(numConstraintsTotal);

    int index = 0;
    for (int i = 0; i < C; i++) {
        for (int j = i + 1; j < C; j++) {
            // the columns should be orthogonal
            (*result)[index++] = weight * linearTransform.col(i).dot(linearTransform.col(j));
        }
    }
    for (int i = 0; i < C; i++) {
        // the columns should have unit length
        (*result)[index++] = weight * (1 - linearTransform.col(i).squaredNorm());
    }
    CARBON_ASSERT(index == numConstraintsTotal, "total index increments do not match total number of constraints");

    JacobianConstPtr<T> Jacobian;
    if (mat.HasJacobian() && mat.Jacobian().NonZeros() > 0) {
        // dRigidity/dM * dMdx

        // create dQ/dM (extended in N)
        index = 0;
        std::vector<Eigen::Triplet<T>> triplets;
        triplets.reserve(numConstraintsOrthogonal * R * 2 + numConstraintsUnitLength * R);

        for (int i = 0; i < C; i++) {
            for (int j = i + 1; j < C; j++) {
                for (int k = 0; k < R; k++) {
                    triplets.push_back(Eigen::Triplet<T>(index, R * i + k, weight * linearTransform.coeff(k, j)));
                }
                for (int k = 0; k < R; k++) {
                    triplets.push_back(Eigen::Triplet<T>(index, R * j + k, weight * linearTransform.coeff(k, i)));
                }
                index++;
            }
        }
        for (int i = 0; i < C; i++) {
            for (int k = 0; k < R; k++) {
                triplets.push_back(Eigen::Triplet<T>(index, R * i + k, - weight * 2 * linearTransform.coeff(k, i)));
            }
            index++;
        }
        CARBON_ASSERT(index == numConstraintsTotal, "total index increments do not match total number of constraints");

        SparseMatrix<T> dRigiditydM(numConstraintsTotal, R * C);
        // TODO: potential optimization: don't use triplet but fill the values directly in the right order
        dRigiditydM.setFromTriplets(triplets.begin(), triplets.end());
        Jacobian = mat.Jacobian().Premultiply(dRigiditydM);
    }

    return DiffData<T>(result, Jacobian);
}

} // namespace nls
} //namespace epic
