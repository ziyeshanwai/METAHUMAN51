// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/utils/TaskThreadPool.h>
#include <nls/math/Math.h>

namespace epic::nls {

/**
 * Method to calculate out = A * x + b with explicit multi-threading. It is only used in case MKL is not available, as MKL will
 * perform multi-threading for matrix multiplication.
 */
template <class T>
void ParallelNoAliasGEMV(Eigen::Ref<Eigen::VectorX<T>> out,
                         const Eigen::Ref<const Eigen::VectorX<T>>& b,
                         const Eigen::Ref<const Eigen::Matrix<T, -1, -1, Eigen::RowMajor>>& A,
                         const Eigen::Ref<const Eigen::VectorX<T>>& x)
{
    CARBON_PRECONDITION(out.size() == b.size(), "Size of output ({}) does not match size of b ({})", out.size(), b.size());
    CARBON_PRECONDITION(A.rows() == b.size(), "Number of rows of A ({}) does not match size of b ({})", A.rows(), b.size());
    CARBON_PRECONDITION(A.cols() == x.size(), "Number of columns of A ({}) does not match size of x ({}).", A.cols(), x.size());

#if defined(EIGEN_USE_BLAS)
    // use BLAS if available
    out.noalias() = A * x + b;
#else
    std::shared_ptr<epic::carbon::TaskThreadPool> globalThreadPool = epic::carbon::TaskThreadPool::GlobalInstance(/*createIfNotAvailable=*/ false);
    if (globalThreadPool && A.rows() > 1000) {

        const int numAvailableThreads = std::max(int(globalThreadPool->NumThreads()), 1);
        // to calculate the lower triangular matrix in blocks we require n * (n + 1) / 2 threads, where n is the split in row/col of AtA
        const int blockSize = int(A.rows()) / numAvailableThreads;
        const int lastBlockSize = int(A.rows()) - (numAvailableThreads - 1) * blockSize;

        epic::carbon::TaskFutures taskFutures;
        taskFutures.Reserve(numAvailableThreads);
        for (int ri = 0; ri < numAvailableThreads; ++ri) {
            taskFutures.Add(globalThreadPool->AddTask(std::bind([&](int r){
                const int rb = (r == numAvailableThreads - 1) ? lastBlockSize : blockSize;
                out.segment(r * blockSize, rb).noalias() = b.segment(r * blockSize, rb) + A.block(r * blockSize, 0, rb, A.cols()) * x;
            }, ri)));
        }
        taskFutures.Wait();
    } else {
        out.noalias() = A * x + b;
    }
#endif
}

/**
 * Method to calculate the lower triangular matrix of AtA with multi-threading. It is only used in case MKL is not available, as MKL will
 * perform multi-threading for matrix multiplication.
 */
template <class T>
void ParallelAtALowerAdd(Eigen::Matrix<T, -1, -1>& AtA, const Eigen::Ref<const Eigen::Matrix<T, -1, -1, Eigen::RowMajor>>& A)
{
    CARBON_PRECONDITION(AtA.rows() == AtA.cols(), "AtA is not square: {}x{}", AtA.rows(), AtA.cols());
    CARBON_PRECONDITION(AtA.rows() == A.cols(), "Number of columns of A ({}) does not match AtA ({})", A.cols(), AtA.cols());

#if defined(EIGEN_USE_BLAS)
    // use BLAS if available
    AtA.template triangularView<Eigen::Lower>() += A.transpose() * A;
#else
    std::shared_ptr<epic::carbon::TaskThreadPool> globalThreadPool = epic::carbon::TaskThreadPool::GlobalInstance(/*createIfNotAvailable=*/ false);
    if (globalThreadPool && A.rows() > 1000) {

        const int numAvailableThreads = std::max(int(globalThreadPool->NumThreads()), 1);
        // to calculate the lower triangular matrix in blocks we require n * (n + 1) / 2 threads, where n is the split in row/col of AtA
        const int numSplits = std::max(int((-1.0f + sqrt(1.0f + 8.0f * float(numAvailableThreads)))/2.0f), 1);
        const int blockSize = int(AtA.rows()) / numSplits;
        const int lastBlockSize = int(AtA.rows()) - (numSplits - 1) * blockSize;

        epic::carbon::TaskFutures taskFutures;
        taskFutures.Reserve(numSplits * (numSplits + 1) / 2);
        for (int ri = 0; ri < numSplits; ++ri) {
            for (int ci = 0; ci <= ri; ++ci) {
                taskFutures.Add(globalThreadPool->AddTask(std::bind([&](int r, int c){
                    const int rb = (r == numSplits - 1) ? lastBlockSize : blockSize;
                    const int cb = (c == numSplits - 1) ? lastBlockSize : blockSize;
                    if (r == c) {
                        AtA.block(r * blockSize, c * blockSize, rb, cb).template triangularView<Eigen::Lower>() += A.block(0, r * blockSize, A.rows(), rb).transpose() * A.block(0, c * blockSize, A.rows(), cb);
                    } else {
                        AtA.block(r * blockSize, c * blockSize, rb, cb).noalias() += A.block(0, r * blockSize, A.rows(), rb).transpose() * A.block(0, c * blockSize, A.rows(), cb);
                    }
                }, ri, ci)));
            }
        }
        taskFutures.Wait();
    } else {
        AtA.template triangularView<Eigen::Lower>() += A.transpose() * A;
    }
#endif
}

} // namespace epic::nls
