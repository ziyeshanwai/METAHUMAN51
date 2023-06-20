// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/math/MKLWrapper.h>

#include <carbon/utils/Profiler.h>

#ifdef EIGEN_USE_MKL_ALL

namespace epic {
namespace nls {
namespace mkl {

namespace impl {

template <class T, int Options>
sparse_status_t CreateMKLSparseMatrix(const Eigen::SparseMatrix<T, Options>& A, sparse_matrix_t& mklA);

template <>
sparse_status_t CreateMKLSparseMatrix(const Eigen::SparseMatrix<float, Eigen::ColMajor>& A, sparse_matrix_t& mklA)
{
    return mkl_sparse_s_create_csc(&mklA, SPARSE_INDEX_BASE_ZERO, int(A.rows()), int(A.cols()), (int*) A.outerIndexPtr(),
                                   (int*) A.outerIndexPtr() + 1, (int*) A.innerIndexPtr(), (float*) A.valuePtr());
}

template <>
sparse_status_t CreateMKLSparseMatrix(const Eigen::SparseMatrix<float, Eigen::RowMajor>& A, sparse_matrix_t& mklA)
{
    return mkl_sparse_s_create_csr(&mklA, SPARSE_INDEX_BASE_ZERO, int(A.rows()), int(A.cols()), (int*) A.outerIndexPtr(),
                                   (int*) A.outerIndexPtr() + 1, (int*) A.innerIndexPtr(), (float*) A.valuePtr());
}

template <>
sparse_status_t CreateMKLSparseMatrix(const Eigen::SparseMatrix<double, Eigen::ColMajor>& A, sparse_matrix_t& mklA)
{
    return mkl_sparse_d_create_csc(&mklA, SPARSE_INDEX_BASE_ZERO, int(A.rows()), int(A.cols()), (int*) A.outerIndexPtr(),
                                   (int*) A.outerIndexPtr() + 1, (int*) A.innerIndexPtr(), (double*) A.valuePtr());
}

template <>
sparse_status_t CreateMKLSparseMatrix(const Eigen::SparseMatrix<double, Eigen::RowMajor>& A, sparse_matrix_t& mklA)
{
    return mkl_sparse_d_create_csr(&mklA, SPARSE_INDEX_BASE_ZERO, int(A.rows()), int(A.cols()), (int*) A.outerIndexPtr(),
                                   (int*) A.outerIndexPtr() + 1, (int*) A.innerIndexPtr(), (double*) A.valuePtr());
}


template <class T, int Options>
sparse_status_t ExportMKLSparseMatrix(sparse_matrix_t A,
                                      sparse_index_base_t* indexing,
                                      MKL_INT* rows,
                                      MKL_INT* cols,
                                      MKL_INT** inner_start,
                                      MKL_INT** inner_end,
                                      MKL_INT** inner_index,
                                      T** values);

template <>
sparse_status_t ExportMKLSparseMatrix<float, Eigen::ColMajor>(sparse_matrix_t A, sparse_index_base_t* indexing,
                                                              MKL_INT* rows, MKL_INT* cols,
                                                              MKL_INT** inner_start, MKL_INT** inner_end,
                                                              MKL_INT** inner_index,
                                                              float** values)
{
    return mkl_sparse_s_export_csc(A, indexing, rows, cols, inner_start, inner_end, inner_index, values);
}

template <>
sparse_status_t ExportMKLSparseMatrix<float, Eigen::RowMajor>(sparse_matrix_t A, sparse_index_base_t* indexing,
                                                              MKL_INT* rows, MKL_INT* cols,
                                                              MKL_INT** inner_start, MKL_INT** inner_end,
                                                              MKL_INT** inner_index,
                                                              float** values)
{
    return mkl_sparse_s_export_csr(A, indexing, rows, cols, inner_start, inner_end, inner_index, values);
}

template <>
sparse_status_t ExportMKLSparseMatrix<double, Eigen::ColMajor>(sparse_matrix_t A, sparse_index_base_t* indexing,
                                                              MKL_INT* rows, MKL_INT* cols,
                                                              MKL_INT** inner_start, MKL_INT** inner_end,
                                                              MKL_INT** inner_index,
                                                              double** values)
{
    return mkl_sparse_d_export_csc(A, indexing, rows, cols, inner_start, inner_end, inner_index, values);
}

template <>
sparse_status_t ExportMKLSparseMatrix<double, Eigen::RowMajor>(sparse_matrix_t A, sparse_index_base_t* indexing,
                                                              MKL_INT* rows, MKL_INT* cols,
                                                              MKL_INT** inner_start, MKL_INT** inner_end,
                                                              MKL_INT** inner_index,
                                                              double** values)
{
    return mkl_sparse_d_export_csr(A, indexing, rows, cols, inner_start, inner_end, inner_index, values);
}


template <class T, int Options>
void CreateEigenSparseMatrix(
    int rows,
    int cols,
    int* inner_start,
    int* inner_end,
    int* inner_index,
    T* values,
    Eigen::SparseMatrix<T, Options>& A)
{
    A.resize(rows, cols);
    int totalNonZeros = 0;
    for (int i = 0; i < int(A.outerSize()); i++) {
        totalNonZeros += inner_end[i] - inner_start[i];
    }
    A.resizeNonZeros(totalNonZeros);
    int counter = 0;
    for (int i = 0; i < int(A.outerSize()); i++) {
        A.outerIndexPtr()[i] = counter;
        const int nonZeros = inner_end[i] - inner_start[i];
        memcpy(A.innerIndexPtr() + counter, inner_index + inner_start[i], sizeof(int) * nonZeros);
        memcpy(A.valuePtr() + counter, values + inner_start[i], sizeof(T) * nonZeros);
        counter += nonZeros;
    }
    A.outerIndexPtr()[A.outerSize()] = totalNonZeros;
}

} // namespace impl


template <class T, int Options>
sparse_matrix_t CreateMKLSparseMatrix(const Eigen::SparseMatrix<T, Options>& A)
{
    PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);

    sparse_matrix_t mklA = nullptr;
    if (!A.isCompressed()) {
        throw std::runtime_error("can only convert compressed matrice to mkl");
    }

    sparse_status_t status = impl::CreateMKLSparseMatrix(A, mklA);

    if (status != SPARSE_STATUS_SUCCESS) {
        throw std::runtime_error(std::string("failure to allocate mkl matrix ") + std::to_string(status));
    }
    return mklA;
}

template sparse_matrix_t CreateMKLSparseMatrix(const Eigen::SparseMatrix<float, Eigen::ColMajor>& A);
template sparse_matrix_t CreateMKLSparseMatrix(const Eigen::SparseMatrix<float, Eigen::RowMajor>& A);
template sparse_matrix_t CreateMKLSparseMatrix(const Eigen::SparseMatrix<double, Eigen::ColMajor>& A);
template sparse_matrix_t CreateMKLSparseMatrix(const Eigen::SparseMatrix<double, Eigen::RowMajor>& A);


template <class T, int Options>
void CreateEigenSparseMatrix(sparse_matrix_t mklA, Eigen::SparseMatrix<T, Options>& A)
{
    PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);

    sparse_index_base_t indexing;
    MKL_INT rows, cols;
    MKL_INT* inner_start;
    MKL_INT* inner_end;
    MKL_INT* inner_index;
    T* values;

    sparse_status_t status = impl::ExportMKLSparseMatrix<T, Options>(mklA, &indexing, &rows, &cols, &inner_start, &inner_end, &inner_index, &values);
    if (status != SPARSE_STATUS_SUCCESS || indexing == SPARSE_INDEX_BASE_ONE) {
        throw std::runtime_error(std::string("failure to export mkl matrix ") + std::to_string(status));
    }
    impl::CreateEigenSparseMatrix<T>(rows, cols, inner_start, inner_end, inner_index, values, A);
}

template void CreateEigenSparseMatrix(sparse_matrix_t mklA, Eigen::SparseMatrix<float, Eigen::ColMajor>& A);
template void CreateEigenSparseMatrix(sparse_matrix_t mklA, Eigen::SparseMatrix<float, Eigen::RowMajor>& A);
template void CreateEigenSparseMatrix(sparse_matrix_t mklA, Eigen::SparseMatrix<double, Eigen::ColMajor>& A);
template void CreateEigenSparseMatrix(sparse_matrix_t mklA, Eigen::SparseMatrix<double, Eigen::RowMajor>& A);


template <class T, int Options>
void ComputeAAt(const Eigen::SparseMatrix<T, Options>& A, Eigen::SparseMatrix<T, Options>& AAt)
{
    PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);

    sparse_matrix_t mklA = CreateMKLSparseMatrix(A);

    sparse_matrix_t mklAAt = nullptr;
    matrix_descr descrA { SPARSE_MATRIX_TYPE_GENERAL, {}, {} };
    sparse_status_t status = mkl_sparse_sp2m (SPARSE_OPERATION_NON_TRANSPOSE, descrA, mklA, SPARSE_OPERATION_TRANSPOSE,
        descrA, mklA,  SPARSE_STAGE_FULL_MULT, &mklAAt);


    if (status != SPARSE_STATUS_SUCCESS) {
        mkl_sparse_destroy(mklA);
        throw std::runtime_error(std::string("failure to perform sparse AtA ") + std::to_string(status));
    }

    // the output of sparse matrix-matrix multiply is not ordered, so we need to order it before assigning it to Eigen
    mkl_sparse_order(mklAAt);

    CreateEigenSparseMatrix(mklAAt, AAt);

    mkl_sparse_destroy(mklA);
    mkl_sparse_destroy(mklAAt);
}

template void ComputeAAt(const Eigen::SparseMatrix<float, Eigen::ColMajor>& A, Eigen::SparseMatrix<float, Eigen::ColMajor>& AAt);
template void ComputeAAt(const Eigen::SparseMatrix<float, Eigen::RowMajor>& A, Eigen::SparseMatrix<float, Eigen::RowMajor>& AAt);
template void ComputeAAt(const Eigen::SparseMatrix<double, Eigen::ColMajor>& A, Eigen::SparseMatrix<double, Eigen::ColMajor>& AAt);
template void ComputeAAt(const Eigen::SparseMatrix<double, Eigen::RowMajor>& A, Eigen::SparseMatrix<double, Eigen::RowMajor>& AAt);


template <class T>
void ComputeAAt_UpperOnly(const Eigen::SparseMatrix<T, Eigen::RowMajor>& A, Eigen::SparseMatrix<T, Eigen::RowMajor>& AAt)
{
    PROFILING_FUNCTION(PROFILING_COLOR_MAGENTA);

    sparse_matrix_t mklA = CreateMKLSparseMatrix(A);

    sparse_matrix_t mklAAt = nullptr;
    sparse_status_t status = mkl_sparse_syrk(SPARSE_OPERATION_NON_TRANSPOSE, mklA, &mklAAt);

    if (status != SPARSE_STATUS_SUCCESS) {
        mkl_sparse_destroy(mklA);
        throw std::runtime_error(std::string("failure to perform sparse AtA ") + std::to_string(status));
    }

    // the output of sparse matrix-matrix multiply is not ordered, so we need to order it before assigning it to Eigen
    mkl_sparse_order(mklAAt);

    CreateEigenSparseMatrix(mklAAt, AAt);

    mkl_sparse_destroy(mklA);
    mkl_sparse_destroy(mklAAt);
}

template void ComputeAAt_UpperOnly(const Eigen::SparseMatrix<float, Eigen::RowMajor>& A, Eigen::SparseMatrix<float, Eigen::RowMajor>& AtA);
template void ComputeAAt_UpperOnly(const Eigen::SparseMatrix<double, Eigen::RowMajor>& A, Eigen::SparseMatrix<double, Eigen::RowMajor>& AtA);

} // namespace mkl
} // namespace nls
} // namespace epic

#endif // EIGEN_USE_MKL_ALL
