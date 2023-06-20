// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/DiffDataMatrix.h>
#include <nls/math/Math.h>
#include <carbon/utils/Profiler.h>

namespace epic {
namespace nls {

/**
 * Function to add two value vectors: f(x) = a(x) + b(x)
 */
template <class T>
class MatrixMultiplyFunction
{
public:
	MatrixMultiplyFunction() {}

	template <int R1, int C1, int R2, int C2>
	static DiffDataMatrix<T, R1, C2> DenseMatrixMatrixMultiply(const DiffDataMatrix<T, R1, C1>& matA, const DiffDataMatrix<T, R2, C2>& matB)
	{
		PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

        CARBON_PRECONDITION(matA.Cols() == matB.Rows(), "for matrix multiplication the number of columns of A needs to match the number of rows of B");

        const int rows = matA.Rows();
        const int innerDim = matA.Cols();
        const int cols = matB.Cols();
        VectorPtr<T> output = std::make_shared<Vector<T>>(rows * cols);

        // C = A * B
        Eigen::Map<Eigen::MatrixX<T>>(output->data(), rows, cols) = matA.Matrix() * matB.Matrix();

		JacobianConstPtr<T> mergedJacobian;

        // dC/dx = dC/dA dA/dx + dC/dB dB/dx
		if (matB.HasJacobian() && matB.Jacobian().NonZeros() > 0) {
			PROFILING_BLOCK("jacobian b1");
            // dC/dB dB/dx
			// create dC/dB
			SparseMatrix<T> dCdB(rows * cols, matB.Rows() * matB.Cols());
			dCdB.reserve(rows * cols * innerDim);
			for (int j = 0; j < cols; j++) {
				for (int i = 0; i < rows; i++) {
					dCdB.startVec(rows * j + i);
					for (int k = 0; k < innerDim; k++) {
                        // c(i,j) += a(i,k) * b(k,j)
						dCdB.insertBackByOuterInnerUnordered(rows * j + i, innerDim * j + k) = matA.Matrix().coeff(i,k);
					}
				}
			}
			dCdB.finalize();
			PROFILING_END_BLOCK;

			PROFILING_BLOCK("jacobian b2");
			JacobianConstPtr<T> dCdx = matB.Jacobian().Premultiply(dCdB);
			PROFILING_END_BLOCK;

			PROFILING_BLOCK("jacobian b3");
			mergedJacobian = dCdx;
			PROFILING_END_BLOCK;
		}

        if (matA.HasJacobian() && matA.Jacobian().NonZeros() > 0) {
			PROFILING_BLOCK("jacobian a1");
            // dC/dA dA/dx
			// create dC/dA
			SparseMatrix<T> dCdA(rows * cols, matA.Rows() * matA.Cols());
			dCdA.reserve(rows * cols * innerDim);
            for (int j = 0; j < cols; j++) {
				for (int i = 0; i < rows; i++) {
					dCdA.startVec(rows * j + i);
					for (int k = 0; k < innerDim; k++) {
                        // c(i,j) += a(i,k) * b(k,j)
						dCdA.insertBackByOuterInnerUnordered(rows * j + i, rows * k + i) = matB.Matrix().coeff(k, j);
					}
				}
			}
			dCdA.finalize();
			PROFILING_END_BLOCK;

			PROFILING_BLOCK("jacobian a2");
			JacobianConstPtr<T> dCdx = matA.Jacobian().Premultiply(dCdA);
			PROFILING_END_BLOCK;

			PROFILING_BLOCK("jacobian a3");
            // add dCdx to the Jacobian matrix
            if (!mergedJacobian) {
                mergedJacobian = dCdx;
            } else {
                mergedJacobian = mergedJacobian->Add(dCdx);
            }
			PROFILING_END_BLOCK;
		}

		return DiffDataMatrix<T, R1, C2>(rows, cols, DiffData<T>(output, mergedJacobian));
	}
};


} // namespace nls
} //namespace epic
