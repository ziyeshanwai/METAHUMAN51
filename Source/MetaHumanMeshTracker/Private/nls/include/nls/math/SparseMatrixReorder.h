// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/math/Math.h>

#ifdef EIGEN_USE_MKL_ALL
#include <nls/math/MKLWrapper.h>
#endif

namespace epic {
namespace nls {


template <class T, int Options>
void SparseMatrixReorder(Eigen::SparseMatrix<T, Options>& A)
{
    PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);

#ifdef EIGEN_USE_MKL_ALL
    mkl::SparseMatrixReorder(A);
#else
    // use eigen to reorder by tranposing the matrix twice
    SparseMatrix<T> other = A.transpose();
    A = other.transpose();
#endif
}


} // namespace nls
} //namespace epic
