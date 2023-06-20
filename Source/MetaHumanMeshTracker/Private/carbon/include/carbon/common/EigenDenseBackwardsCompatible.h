// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/common/Common.h>

CARBON_DISABLE_EIGEN_WARNINGS
#include <Eigen/Dense>
CARBON_RENABLE_WARNINGS

#if !EIGEN_VERSION_AT_LEAST(3,3,9)
namespace Eigen {

	template <typename T>
	using Matrix2 = Matrix<T, 2, 2>;

	template <typename T>
	using Matrix2X = Matrix<T, 2, Dynamic>;

	template <typename T>
	using Matrix3 = Matrix<T, 3, 3>;

	template <typename T>
	using Matrix3X = Matrix<T, 3, Dynamic>;

	template <typename T>
	using Matrix4 = Matrix<T, 4, 4>;

	template <typename T>
	using Matrix4X = Matrix<T, 4, Dynamic>;

	template <typename T>
	using MatrixX = Matrix<T, Dynamic, Dynamic>;

	template <typename T>
	using MatrixX2 = Matrix<T, Dynamic, 2>;

	template <typename T>
	using MatrixX3 = Matrix<T, Dynamic, 3>;

	template <typename T>
	using MatrixX4 = Matrix<T, Dynamic, 4>;

	template <typename T, int Size>
	using RowVector = Matrix<T, 1, Size>;

	template <typename T>
	using RowVector2 = RowVector<T, 2>;

	template <typename T>
	using RowVector3 = RowVector<T, 3>;

	template <typename T>
	using RowVector4 = RowVector<T, 4>;

	template <typename T>
	using RowVectorX = RowVector<T, Dynamic>;

	template <typename T, int Size>
	using Vector = Matrix<T, Size, 1>;

	template <typename T>
	using Vector2 = Vector<T, 2>;

	template <typename T>
	using Vector3 = Vector<T, 3>;

	template <typename T>
	using Vector4 = Vector<T, 4>;

	template <typename T>
	using VectorX = Vector<T, Dynamic>;
}
#endif
