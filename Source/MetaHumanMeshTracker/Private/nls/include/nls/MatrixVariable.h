// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/DiffDataMatrix.h>
#include <nls/VectorVariable.h>

namespace epic {
namespace nls {

namespace internal {
//! helper class to have fixed and dynamic size dimensions (implementation further below)
template <int R, int C>
class DimInfo;
}


template <class T, int R = -1, int C = -1, typename DISCARD = typename std::enable_if<(R >= -1 && C >= -1), void>::type>
class MatrixVariable : public VectorVariable<T>
{
public:
    static constexpr int ROWS = R;
	static constexpr int COLS = C;

public:
    //! constructor for fixed size matrices (calls the protected version further below with the correct updateDimension)
    template <typename U = T>
	MatrixVariable(typename std::enable_if<std::is_same<U,T>::value && ((R >= 0) && (C >= 0))>::type* = 0)
        : MatrixVariable(/*updateDimension=*/R * C) {}

    //! constructor for dynamic size matrices (calls the protected version further below with the correct updateDimension)
    template <typename U = T>
	MatrixVariable(int rows, int cols, typename std::enable_if<std::is_same<U,T>::value && !((R >= 0) && (C >= 0))>::type* = 0)
        :  MatrixVariable(rows, cols, /*updateDimension=*/rows * cols) {}

    int Rows() const { return dimInfo.Rows(); }
    int Cols() const { return dimInfo.Cols(); }

	//! Sets the matrix and projects it to the manifold.
	void SetMatrix(const Eigen::Matrix<T,R,C>& M)
	{
		SetMatrix(M, /*projectToManifold*/true);
	}

    //! Convenience function returning the variable data as a matrix.
	Eigen::Map<const Eigen::Matrix<T,R,C>> Matrix() const
	{
		return Eigen::Map<const Eigen::Matrix<T,R,C>>(this->Value().data(), Rows(), Cols());
	}

	//! Convenience function returning a matrix with Jacobian (DiffDataMatrix) instead of just a vector
	DiffDataMatrix<T, R, C> EvaluateMatrix(Context<T>* context = nullptr)
	{
		return DiffDataMatrix<T, R, C>(Rows(), Cols(), this->Evaluate(context));
	}

    //! convenience function to set the identity matrix
    void SetIdentity()
	{
		SetMatrix(Eigen::Matrix<T,R,C>::Identity(Rows(), Cols()));
	}

    //! convenience function to set a random matrix
    void SetRandom()
	{
		SetMatrix(Eigen::Matrix<T, R, C>::Random(Rows(), Cols()));
	}

protected:
	//! Only subclasses can call this constructor to set up a matrix variable with an underlying manifold representation e.g. quaternion.
    template <typename U = T>
	MatrixVariable(int updateDimension, typename std::enable_if<std::is_same<U,T>::value && ((R >= 0) && (C >= 0))>::type* = 0)
        : VectorVariable<T>(R * C, updateDimension), dimInfo(R, C) {}

    //! Only subclasses can call this constructor to set up a matrix variable with an underlying manifold representation e.g. quaternion.
    template <typename U = T>
	MatrixVariable(int rows, int cols, int updateDimension, typename std::enable_if<std::is_same<U,T>::value && !((R >= 0) && (C >= 0))>::type* = 0)
        : VectorVariable<T>(rows * cols, updateDimension), dimInfo(rows, cols) {}

	/**
	 * Sets the matrix and optionally projects it to the manifold. Note that the caller needs to ensure that the matrix is already on the
	 * manifold if projectToManifold=false.
	 */
	void SetMatrix(const Eigen::Matrix<T,R,C>& M, bool projectToManifold)
	{
		this->SetValue(M.data(), M.size(), projectToManifold);
	}

	//! @see SetMatrix above
	void SetMatrix(const Eigen::Map<const Eigen::Matrix<T,R,C>>& M, bool projectToManifold)
	{
		this->SetValue(M.data(), M.size(), projectToManifold);
	}

private:

    internal::DimInfo<R,C> dimInfo;
};


namespace internal {

//! helper class to have fixed and dynamic size dimensions
template <int R, int C>
class DimInfo
{
public:
    DimInfo(int, int) {}
    static constexpr int Rows() { return R; }
    static constexpr int Cols() { return C; }
};

template <int R>
class DimInfo<R, -1>
{
public:
    DimInfo(int rows, int cols) : cols(cols)
    {
        CARBON_PRECONDITION(rows == R, "invalid row size {}", rows);
        CARBON_PRECONDITION(cols >= 0, "invalid column size {}", cols);
    }
    int Rows() const { return R; }
    int Cols() const { return cols; }

private:
    int cols;
};

template <int C>
class DimInfo<-1, C>
{
public:
    DimInfo(int rows, int cols) : rows(rows)
    {
        CARBON_PRECONDITION(rows >= 0, "invalid row size {}", rows);
        CARBON_PRECONDITION(cols == C, "invalid column size {}", cols);
    }
    int Rows() const { return rows; }
    int Cols() const { return C; }

private:
    int rows;
};

template <>
class DimInfo<-1, -1>
{
public:
    DimInfo(int rows, int cols) : rows(rows), cols(cols)
    {
        CARBON_PRECONDITION(rows >= 0 && cols >= 0, "invalid row and column sizes {}x{}", rows, cols);
    }
    int Rows() const { return rows; }
    int Cols() const { return cols; }

private:
    int rows;
    int cols;
};

} // namespace internal
} // namespace nls
} // namespace epic
