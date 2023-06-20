// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>
#include <carbon/utils/Profiler.h>

#ifdef EIGEN_USE_MKL_ALL

#include "mkl_spblas.h"


namespace epic {
namespace nls {
namespace mkl {

//! Creates a row or column-major MKL matrix depending on the Eigen type.
template <class T, int Options>
sparse_matrix_t CreateMKLSparseMatrix(const Eigen::SparseMatrix<T, Options>& A);


//! Creates an Eigen Sparse matrix from an MKL sparse matrix.
template <class T, int Options>
void CreateEigenSparseMatrix(sparse_matrix_t mklA, Eigen::SparseMatrix<T, Options>& A);


//! Calculates AAt = A * A.transpose().
template <class T, int Options>
void ComputeAAt(const Eigen::SparseMatrix<T, Options>& A, Eigen::SparseMatrix<T, Options>& AAt);


/**
 * Calculates AAt for the upper triangular matrix only. Note that this is currently
 * slower than ComputeAAt above, so for now this should be used.
 */
template <class T>
void ComputeAAt_UpperOnly(const Eigen::SparseMatrix<T, Eigen::RowMajor>& A, Eigen::SparseMatrix<T, Eigen::RowMajor>& AAt);

//! Calculates the C = op(A) * op(B) where op() is transpose or none
template <class T, int Options>
void SparseMatrixMultiply(const Eigen::SparseMatrix<T, Options>& A, bool transposeA,
                          const Eigen::SparseMatrix<T, Options>& B, bool transposeB,
                          Eigen::SparseMatrix<T, Options>& C)
{
    PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);

    if (A.nonZeros() == 0 || B.nonZeros() == 0) {
        // special case handling empty matrices as MKL cannot load empty matrices
        const size_t C_rows = transposeA ? A.cols() : A.rows();
        const size_t C_cols = transposeB ? B.rows() : B.cols();
        C.resize(C_rows, C_cols);
        C.resizeNonZeros(0);
        return;
    }

    sparse_matrix_t mklA = CreateMKLSparseMatrix(A);
    sparse_matrix_t mklB = CreateMKLSparseMatrix(B);

    sparse_matrix_t mklC = nullptr;
    matrix_descr descr { SPARSE_MATRIX_TYPE_GENERAL, {}, {} };
    sparse_status_t status = mkl_sparse_sp2m (transposeA ? SPARSE_OPERATION_TRANSPOSE : SPARSE_OPERATION_NON_TRANSPOSE,
                                              descr, mklA,
                                              transposeB ? SPARSE_OPERATION_TRANSPOSE : SPARSE_OPERATION_NON_TRANSPOSE,
                                              descr, mklB,
                                              SPARSE_STAGE_FULL_MULT,
                                              &mklC);

    if (status != SPARSE_STATUS_SUCCESS) {
        mkl_sparse_destroy(mklA);
        mkl_sparse_destroy(mklB);
        throw std::runtime_error(std::string("failure to perform sparse matrix product ") + std::to_string(status));
    }

    // the output of sparse matrix-matrix multiply is not ordered, so we need to order it before assigning it to Eigen
    mkl_sparse_order(mklC);

    CreateEigenSparseMatrix(mklC, C);

    mkl_sparse_destroy(mklA);
    mkl_sparse_destroy(mklB);
    mkl_sparse_destroy(mklC);
}


//! Reorders the inner dimension of the matrix so that the inner indices are in order
template <class T, int Options>
void SparseMatrixReorder(Eigen::SparseMatrix<T, Options>& A)
{
    PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);

    if (A.nonZeros() == 0) {
        return;
    }

    sparse_matrix_t mklA = CreateMKLSparseMatrix(A);
    sparse_status_t status = mkl_sparse_order(mklA);
    if (status != SPARSE_STATUS_SUCCESS) {
        mkl_sparse_destroy(mklA);
        throw std::runtime_error(std::string("failure to reorder the matrix ") + std::to_string(status));
    }
    mkl_sparse_destroy(mklA);
}


}
}
}

#endif // EIGEN_USE_MKL_ALL
