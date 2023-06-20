// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>

#include <memory>

namespace epic {
namespace nls {

template <class T>
class Jacobian;

template <class T>
using JacobianPtr = std::shared_ptr<Jacobian<T>>;

template <class T>
using JacobianConstPtr = std::shared_ptr<const Jacobian<T>>;


/**
 * Abstract Jacobian class
 */
template <class T>
class Jacobian
{
public:
	Jacobian() : m_rows(0), m_cols(0) {}
    Jacobian(int rows, int cols) : m_rows(rows), m_cols(cols) {}
    virtual ~Jacobian() = default;

    //! @returns the number of rows of the Jacobian matrix
    int Rows() const { return m_rows; }

    //! @returns the number of columns of the Jacobian matrix
    int Cols() const { return m_cols; }

    //! @returns True if the Jacobian matrix is sparse
    virtual bool IsSparse() const = 0;

    //! @returns the number of non zeros in the Jacobian matrix
    virtual int NonZeros() const = 0;

    //! @returns the Jacobian as a Sparse Matrix
    virtual SparseMatrixConstPtr<T> AsSparseMatrix() const = 0;

    //! @returns a new Jacobian by premultiplying the current Jacobian with sparseMat
    virtual JacobianConstPtr<T> Premultiply(const SparseMatrix<T>& sparseMat) const = 0;

    /**
     * Adds another Jacobian to this Jacobian and returns it as a new Jacobian.
     * @precondition: other.Rows() == Rows() and other.Cols() == Cols()
     */
    virtual JacobianConstPtr<T> Add(JacobianConstPtr<T> other) const = 0;

    /**
     * Subtracts another Jacobian from this Jacobian and returns it as a new Jacobian.
     * @precondition: other.Rows() == Rows() and other.Cols() == Cols()
     */
    virtual JacobianConstPtr<T> Subtract(JacobianConstPtr<T> other) const = 0;

    //! Scales the Jacobian by @p scale and returns it as a new Jacobian
    virtual JacobianConstPtr<T> Scale(T scale) const = 0;

    /**
     * Selects blockSize rows from the Jacobian and returns it as a new Jacobian
     *
     * newJacobian(blockSize  * i : blockSize * (i + 1), :) = thisJacobian(blockSize * blockIndices[i], blockSize * (blockIndices[i] + 1), :)
     */
    virtual JacobianConstPtr<T> RowGather(const Eigen::VectorX<int>& blockIndices, int blockSize = 1) const = 0;

    /**
     * Scatters the blockSize rows from this Jacobian to outputsize based on the blockIndices and returns it as a new Jacobian
     *
     * newJacobian(blockSize * blockIndices[i] : blockSize * (blockIndices[i] + 1), :) = thisJacobian(blockSize * i : blockSize * (i + 1), :)
     */
    virtual JacobianConstPtr<T> RowScatter(int outputSize, const Eigen::VectorX<int>& blockIndices, int blockSize = 1) const = 0;

    //! Repeats the rows of the Jacobian @p N times.
    virtual JacobianConstPtr<T> Repeat(int N) const = 0;

    //! Extract a single row of the Jacobian
    virtual Eigen::SparseVector<T> Row(int row) const = 0;

private:
    Jacobian(const Jacobian&) = delete;
    Jacobian& operator=(const Jacobian&) = delete;

private:
    int m_rows;
    int m_cols;
};


/**
 * Default Sparse Jacobian class using a row-major Eigen sparse matrix
 */
template <class T>
class SparseJacobian : public Jacobian<T>
{
public:
	SparseJacobian(SparseMatrixConstPtr<T> sparseMatrix) : Jacobian<T>(int(sparseMatrix->rows()), int(sparseMatrix->cols())), m_sparseMatrix(sparseMatrix) {}
	virtual ~SparseJacobian() {}

    virtual bool IsSparse() const final override { return true; }

    virtual int NonZeros() const final override { return int(m_sparseMatrix->nonZeros()); }

    virtual SparseMatrixConstPtr<T> AsSparseMatrix() const final override { return m_sparseMatrix; }

    virtual JacobianConstPtr<T> Premultiply(const SparseMatrix<T>& sparseMat) const final override;

    virtual JacobianConstPtr<T> Add(JacobianConstPtr<T> other) const final override;

    virtual JacobianConstPtr<T> Subtract(JacobianConstPtr<T> other) const final override;

    virtual JacobianConstPtr<T> Scale(T scale) const final override;

    virtual JacobianConstPtr<T> RowGather(const Eigen::VectorX<int>& blockIndices, int blockSize) const final override;

    virtual JacobianConstPtr<T> RowScatter(int outputSize, const Eigen::VectorX<int>& blockIndices, int blockSize) const final override;

    virtual JacobianConstPtr<T> Repeat(int N) const final override;

    virtual Eigen::SparseVector<T> Row(int row) const final override;

private:
    SparseMatrixConstPtr<T> m_sparseMatrix;
};

template <class T>
using SparseJacobianPtr = std::shared_ptr<SparseJacobian<T>>;

template <class T>
using SparseJacobianConstPtr = std::shared_ptr<const SparseJacobian<T>>;


} // namespace nls
} //namespace epic
