// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>

namespace epic {
namespace nls {

/**
 * Calculates the Jacobian of the product A = M X
 * A: R x C
 * M: R x K
 * X: K x C
 * @returns dA/dX
 */
template <class T, int R, int K, int C>
SparseMatrix<T> JacobianOfPremultipliedMatrix(const Eigen::Matrix<T, R, K>& M)
{
    std::vector<Eigen::Triplet<T>> triplets;
    triplets.reserve(R * C * K);
    for (int c = 0; c < C; c++) {
        for (int r = 0; r < R; r++) {
            for (int k = 0; k < K; k++) {
                // A(r,c) += M(r,k) * X(k,c)
                triplets.push_back(Eigen::Triplet<T>(R * c + r, K * c + k, M.coeff(r, k)));
            }
        }
    }
    SparseMatrix<T> dAdX(R * C, K * C);
    // TODO: potential optimization: don't use triplet but fill the values directly in the right order
    dAdX.setFromTriplets(triplets.begin(), triplets.end());
    return dAdX;
}

/**
 * Calculates the Jacobian of the product A = X M
 * A: R x C
 * X: R x K
 * M: K x C
 * @returns dA/dX
 */
template <class T, int R, int K, int C>
SparseMatrix<T> JacobianOfPostmultipliedMatrix(const Eigen::Matrix<T, K, C>& M)
{
    std::vector<Eigen::Triplet<T>> triplets;
    triplets.reserve(R * C * K);
    for (int c = 0; c < C; c++) {
        for (int r = 0; r < R; r++) {
            for (int k = 0; k < K; k++) {
                // A(r,c) += X(r,k) * M(k,c)
                triplets.push_back(Eigen::Triplet<T>(R * c + r, R * k + r, M.coeff(k, c)));
            }
        }
    }
    SparseMatrix<T> dAdX(R * C, R * K);
    // TODO: potential optimization: don't use triplet but fill the values directly in the right order
    dAdX.setFromTriplets(triplets.begin(), triplets.end());
    return dAdX;
}


/**
 * Calculates the Jacobian of the cross product matrix L with an optional scaling:
 * SL = s * |  0  -z   y  |
 *          |  z   0  -x  |
 *          | -y   x   0  |
 * and X = [x y z]^T
 *
 * @returns dSL/dX
 *
 * dSL/dx = | 0  0  0 |
 *          | 0  0 -s |
 *          | 0  s  0 |
 *
 * dSL/dy = | 0  0  s |
 * 		    | 0  0  0 |
 * 		    |-s  0  0 |
 *
 * dSL/dz = | 0 -s  0 |
 * 		    | s  0  0 |
 * 		    | 0  0  0 |
 */
template <class T>
SparseMatrix<T> JacobianOfCrossProductMatrix(const T SCALE = T(1.0))
{
	SparseMatrix<T> J(3 * 3, 3);
	J.resizeNonZeros(6);

	// J.insert(1, 2) = SCALE;
	// J.insert(2, 1) = -SCALE;
	// J.insert(3, 2) = -SCALE;
	// J.insert(5, 0) = SCALE;
	// J.insert(6, 1) = SCALE;
	// J.insert(7, 0) = -SCALE;
	// J.makeCompressed();

	// efficient method
	int* outputRowIndices = J.outerIndexPtr();
	int* outputColIndices = J.innerIndexPtr();
	T* outputValues = J.valuePtr();
	int outputCurrIndex = 0;

	outputRowIndices[0] = outputCurrIndex;

	outputRowIndices[1] = outputCurrIndex;
	outputColIndices[outputCurrIndex] = 2;
	outputValues[outputCurrIndex] = SCALE;
	outputCurrIndex++;

	outputRowIndices[2] = outputCurrIndex;
	outputColIndices[outputCurrIndex] = 1;
	outputValues[outputCurrIndex] = -SCALE;
	outputCurrIndex++;

	outputRowIndices[3] = outputCurrIndex;
	outputColIndices[outputCurrIndex] = 2;
	outputValues[outputCurrIndex] = -SCALE;
	outputCurrIndex++;

	outputRowIndices[4] = outputCurrIndex;

	outputRowIndices[5] = outputCurrIndex;
	outputColIndices[outputCurrIndex] = 0;
	outputValues[outputCurrIndex] = SCALE;
	outputCurrIndex++;

	outputRowIndices[6] = outputCurrIndex;
	outputColIndices[outputCurrIndex] = 1;
	outputValues[outputCurrIndex] = SCALE;
	outputCurrIndex++;

	outputRowIndices[7] = outputCurrIndex;
	outputColIndices[outputCurrIndex] = 0;
	outputValues[outputCurrIndex] = -SCALE;
	outputCurrIndex++;

	outputRowIndices[8] = outputCurrIndex;

	outputRowIndices[9] = outputCurrIndex;

	return J;
}


/**
 * Calculates the Jacobian of the product A = L M with L being the cross product matrix and optional scaling:
 * L = |  0  -z   y  |
 *     |  z   0  -x  |
 *     | -y   x   0  |
 * and X = [x y z]^T
 *
 * @returns dA/dX
 *
 * dL/dx = | 0  0  0 |
 *         | 0  0 -1 |
 *         | 0  1  0 |
 *
 * dL/dy = | 0  0  1 |
 * 		   | 0  0  0 |
 * 		   |-1  0  0 |
 *
 * dL/dz = | 0 -1  0 |
 * 		   | 1  0  0 |
 * 		   | 0  0  0 |
 *
 * M =  | row(0) |
 *      | row(1) |
 * 		| row(2) |
 *
 * dA/dx =  |  0		|
 * 		    | -2 row(2)	|
 *          |  2 row(1)	|
 *
 * dA/dy =  |  2 row(2) |
 *          |  0        |
 * 			| -2 row(0) |
 *
 * dA/dz =  | -2 row(1) |
 * 			|  2 row(0) |
 * 			|  0        |
 */
template <class T, int C>
SparseMatrix<T> JacobianOfCrossProductMatrixPostMultipliedByMatrix(Eigen::Map<const Eigen::Matrix<T, 3, C>> M, const T SCALE = T(1.0))
{
	SparseMatrix<T> J(3 * C, 3);
	J.resizeNonZeros(6 * C);
	//
	// for (int c = 0; c < C; c++) {
	// 	J.insert(3 * c + 0, 1) = SCALE * M.coeff(2, c);
	// 	J.insert(3 * c + 0, 2) = -SCALE * M.coeff(1, c);
	// 	J.insert(3 * c + 1, 2) = SCALE * M.coeff(0, c);
	// 	J.insert(3 * c + 1, 0) = -SCALE * M.coeff(2, c);
	// 	J.insert(3 * c + 2, 0) = SCALE * M.coeff(1, c);
	// 	J.insert(3 * c + 2, 1) = -SCALE * M.coeff(0, c);
	// }
	// J.makeCompressed();

	// efficient method
	int* outputRowIndices = J.outerIndexPtr();
	int* outputColIndices = J.innerIndexPtr();
	T* outputValues = J.valuePtr();
	int outputCurrIndex = 0;

	for (int c = 0; c < C; c++) {
		outputRowIndices[3 * c + 0] = outputCurrIndex;
		//J.insert(3 * c + 0, 1) = SCALE * M.coeff(2, c);
		outputColIndices[outputCurrIndex] = 1;
		outputValues[outputCurrIndex] = SCALE * M.coeff(2, c);
		outputCurrIndex++;
		//J.insert(3 * c + 0, 2) = -SCALE * M.coeff(1, c);
		outputColIndices[outputCurrIndex] = 2;
		outputValues[outputCurrIndex] = -SCALE * M.coeff(1, c);
		outputCurrIndex++;

		outputRowIndices[3 * c + 1] = outputCurrIndex;
		//J.insert(3 * c + 1, 2) = SCALE * M.coeff(0, c);
		outputColIndices[outputCurrIndex] = 2;
		outputValues[outputCurrIndex] = SCALE * M.coeff(0, c);
		outputCurrIndex++;
		//J.insert(3 * c + 1, 0) = -SCALE * M.coeff(2, c);
		outputColIndices[outputCurrIndex] = 0;
		outputValues[outputCurrIndex] = -SCALE * M.coeff(2, c);
		outputCurrIndex++;

		outputRowIndices[3 * c + 2] = outputCurrIndex;
		//J.insert(3 * c + 2, 0) = SCALE * M.coeff(1, c);
		outputColIndices[outputCurrIndex] = 0;
		outputValues[outputCurrIndex] = SCALE * M.coeff(1, c);
		outputCurrIndex++;
		//J.insert(3 * c + 2, 1) = -SCALE * M.coeff(0, c);
		outputColIndices[outputCurrIndex] = 1;
		outputValues[outputCurrIndex] = -SCALE * M.coeff(0, c);
		outputCurrIndex++;
	}
	outputRowIndices[3 * C] = outputCurrIndex;
	CARBON_ASSERT(outputCurrIndex == 6 * C, "total index increments do not match number of non-zeros");

	return J;
}


/**
 * Calculates the Jacobian of the product A = M L with L being the cross product matrix and optional scaling:
 * L = |  0  -z   y  |
 *     |  z   0  -x  |
 *     | -y   x   0  |
 * and X = [x y z]^T
 *
 * @returns dA/dX
 *
 * dL/dx = | 0  0  0 |
 *         | 0  0 -1 |
 *         | 0  1  0 |
 *
 * dL/dy = | 0  0  1 |
 * 		   | 0  0  0 |
 * 		   |-1  0  0 |
 *
 * dL/dz = | 0 -1  0 |
 * 		   | 1  0  0 |
 * 		   | 0  0  0 |
 *
 *
 * M =  	|    col(0)   	col(1)      col(2) |
 *
 * dA/dx =  |  0          2 col(2)   -2 col(1) |
 *
 * dA/dy =  | -2 col(2)   0           2 col(0) |
 *
 * dA/dz =  |  2 col(1)  -2col(0)     0        |
 */
template <class T, int R>
SparseMatrix<T> JacobianOfCrossProductMatrixPreMultipliedByMatrix(Eigen::Map<const Eigen::Matrix<T, R, 3>> M, const T SCALE = T(1.0))
{
	// const T x = 0;
	// const T y = 0;
	// const T z = 0;
	// const T w = 1;

	SparseMatrix<T> J(R * 3,3);
	J.resizeNonZeros(R * 6);
	// for (int r = 0; r < R; r++) {
	// 	J.insert(3 * 0 + r, 1) = - SCALE * M.coeff(r, 2);
	// 	J.insert(3 * 0 + r, 2) =   SCALE * M.coeff(r, 1);
	// }
	// for (int r = 0; r < R; r++) {
	// 	J.insert(3 * 1 + r, 0) =   SCALE * M.coeff(r, 2);
	// 	J.insert(3 * 1 + r, 2) = - SCALE * M.coeff(r, 0);
	// }
	// for (int r = 0; r < R; r++) {
	// 	J.insert(3 * 2 + r, 0) = - SCALE * M.coeff(r, 1);
	// 	J.insert(3 * 2 + r, 1) =   SCALE * M.coeff(r, 0);
	// }
	// J.makeCompressed();

	// efficient method
	int* outputRowIndices = J.outerIndexPtr();
	int* outputColIndices = J.innerIndexPtr();
	T* outputValues = J.valuePtr();
	int outputCurrIndex = 0;
	for (int r = 0; r < R; r++) {
		outputRowIndices[R * 0 + r] = outputCurrIndex;
		//J.insert(3 * 0 + r, 1) = - SCALE * M.coeff(r, 2);
		outputColIndices[outputCurrIndex] = 1;
		outputValues[outputCurrIndex] = - SCALE * M.coeff(r, 2);
		outputCurrIndex++;
		//J.insert(3 * 0 + r, 2) =   SCALE * M.coeff(r, 1);
		outputColIndices[outputCurrIndex] = 2;
		outputValues[outputCurrIndex] = SCALE * M.coeff(r, 1);
		outputCurrIndex++;
	}
	for (int r = 0; r < R; r++) {
		outputRowIndices[R * 1 + r] = outputCurrIndex;
		//J.insert(3 * 1 + r, 0) = SCALE * M.coeff(r, 2);
		outputColIndices[outputCurrIndex] = 0;
		outputValues[outputCurrIndex] = SCALE * M.coeff(r, 2);
		outputCurrIndex++;
		//J.insert(3 * 1 + r, 2) = - SCALE * M.coeff(r, 0);
		outputColIndices[outputCurrIndex] = 2;
		outputValues[outputCurrIndex] = - SCALE * M.coeff(r, 0);
		outputCurrIndex++;
	}

	for (int r = 0; r < R; r++) {
		outputRowIndices[R * 2 + r] = outputCurrIndex;
		//J.insert(3 * 2 + r, 0) = - SCALE * M.coeff(r, 1);
		outputColIndices[outputCurrIndex] = 0;
		outputValues[outputCurrIndex] = - SCALE * M.coeff(r, 1);
		outputCurrIndex++;
		//J.insert(3 * 2 + r, 1) =   SCALE * M.coeff(r, 0);
		outputColIndices[outputCurrIndex] = 1;
		outputValues[outputCurrIndex] = SCALE * M.coeff(r, 0);
		outputCurrIndex++;
	}
	outputRowIndices[R * 3] = outputCurrIndex;
	CARBON_ASSERT(outputCurrIndex == R * 6, "total index increments do not match number of non-zeros");

	return J;
}

} // namespace nls
} //namespace epic
