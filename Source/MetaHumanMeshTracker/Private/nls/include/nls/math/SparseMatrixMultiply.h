// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/math/Math.h>
#include <carbon/utils/Profiler.h>
#include <carbon/utils/Timer.h>

#ifdef EIGEN_USE_MKL_ALL
#include <nls/math/MKLWrapper.h>
#endif

namespace epic {
namespace nls {

template <class T, int Options>
void SparseMatrixMultiply(const Eigen::SparseMatrix<T, Options>& A, bool transposeA,
                          const Eigen::SparseMatrix<T, Options>& B,  bool transposeB,
                          Eigen::SparseMatrix<T, Options>& C)
{
    PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);

#ifdef EIGEN_USE_MKL_ALL
    static constexpr int minimumNonZerosForMKL = 30; // random setting for now which is suitable for 4x4 matrices, not measured
    if (A.nonZeros() > minimumNonZerosForMKL && B.nonZeros() > minimumNonZerosForMKL) {
        // mkl has an overhead that we want to skip for small matrices
        mkl::SparseMatrixMultiply(A, transposeA, B, transposeB, C);
        return;
    }
#endif
    // eigen implementation
    if (transposeA && transposeB) {
        C = A.transpose() * B.transpose();
    } else if (transposeA) {
        C = A.transpose() * B;
    } else if (transposeB) {
        C = A * B.transpose();
    } else {
        C = A * B;
    }
}


} // namespace nls
} //namespace epic
