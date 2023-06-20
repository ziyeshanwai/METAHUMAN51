// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/Jacobian.h>
#include <nls/math/SparseMatrixMultiply.h>


namespace epic {
namespace nls {

template <class T>
JacobianConstPtr<T> SparseJacobian<T>::Premultiply(const SparseMatrix<T>& sparseMat) const
{
    SparseMatrixPtr<T> sparseMatrix = std::make_shared<SparseMatrix<T>>();
    SparseMatrixMultiply(sparseMat, false, *m_sparseMatrix, false, *sparseMatrix);
    return std::make_shared<SparseJacobian<T>>(sparseMatrix);
}

template <class T>
JacobianConstPtr<T> SparseJacobian<T>::Add(JacobianConstPtr<T> other) const
{
    SparseMatrixPtr<T> J = std::make_shared<SparseMatrix<T>>();
    *J = AddSparseMatricesAndPadColumns<T>(*m_sparseMatrix, *(other->AsSparseMatrix()), /*squeeze=*/false);
    return std::make_shared<SparseJacobian<T>>(J);
}


template <class T>
JacobianConstPtr<T> SparseJacobian<T>::Subtract(JacobianConstPtr<T> other) const
{
    SparseMatrixPtr<T> J = std::make_shared<SparseMatrix<T>>();
    *J = AddSparseMatricesAndPadColumns<T>(*m_sparseMatrix, - *(other->AsSparseMatrix()), /*squeeze=*/false);
    return std::make_shared<SparseJacobian<T>>(J);
}


template <class T>
JacobianConstPtr<T> SparseJacobian<T>::Scale(T scale) const
{
    return std::make_shared<SparseJacobian<T>>(std::make_shared<SparseMatrix<T>>(scale * *m_sparseMatrix));
}

template <class T>
JacobianConstPtr<T> SparseJacobian<T>::RowGather(const Eigen::VectorX<int>& blockIndices, int blockSize) const
{
    SparseMatrixPtr<T> jacobian = std::make_shared<SparseMatrix<T>>();
    *jacobian = epic::nls::RowGather(*m_sparseMatrix, blockIndices, blockSize);
    return std::make_shared<SparseJacobian<T>>(jacobian);
}

template <class T>
JacobianConstPtr<T> SparseJacobian<T>::RowScatter(int outputSize, const Eigen::VectorX<int>& blockIndices, int blockSize) const
{
     SparseMatrixPtr<T> jacobian = std::make_shared<SparseMatrix<T>>();
    *jacobian = epic::nls::RowScatter(*m_sparseMatrix, outputSize, blockIndices, blockSize);
    return std::make_shared<SparseJacobian<T>>(jacobian);
}

template <class T>
JacobianConstPtr<T> SparseJacobian<T>::Repeat(int N) const
{
    SparseMatrixPtr<T> repeatedJacobian = std::make_shared<SparseMatrix<T>>();
    RepeatRowsOfSparseMatrix<T>(*m_sparseMatrix, *repeatedJacobian, N);
    return std::make_shared<SparseJacobian<T>>(repeatedJacobian);
}

template <class T>
Eigen::SparseVector<T> SparseJacobian<T>::Row(int row) const
{
    return m_sparseMatrix->row(row);
}

template class SparseJacobian<float>;
template class SparseJacobian<double>;

} // namespace nls
} //namespace epic
