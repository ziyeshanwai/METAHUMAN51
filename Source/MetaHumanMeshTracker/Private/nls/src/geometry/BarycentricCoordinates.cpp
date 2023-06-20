// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/geometry/BarycentricCoordinates.h>

namespace epic {
namespace nls {

template<class T, int C>
Eigen::Vector<T, C> BarycentricCoordinates<T, C>::ComputeBarycentricCoordinates(const Eigen::Vector<T,3>& p, const Eigen::Vector<int,C>& indices, const Eigen::Matrix<T,3,-1>& vertices)
{
	static_assert(C == 3 || C == 4);
	if constexpr (C == 3)
	{	// orthogonal projection
		Eigen::Matrix<T, 3, 2> A;
		A.col(0) = vertices.col(indices[1]) - vertices.col(indices[0]);
		A.col(1) = vertices.col(indices[2]) - vertices.col(indices[0]);
		if (A.col(0).cross(A.col(1)).norm() < T(1e-10))
			throw std::runtime_error("Attempting to compute barycentric coordinates w.r.t. a degenerate triangle");
		Eigen::Vector<T, 2> ab = (A.transpose() * A).inverse()* A.transpose()* (p - vertices.col(indices[0]));
		Eigen::Vector<T, 3> weights;
		weights[0] = T(1.0) - ab[0] - ab[1];
		weights[1] = ab[0];
		weights[2] = ab[1];
		return weights;
	}
	else
	{	// direct inversion
		Eigen::Matrix<T, 3, 3> A;
		A.col(0) = vertices.col(indices[1]) - vertices.col(indices[0]);
		A.col(1) = vertices.col(indices[2]) - vertices.col(indices[0]);
		A.col(2) = vertices.col(indices[3]) - vertices.col(indices[0]);
		if (A.determinant() < T(1e-10))
			throw std::runtime_error("Attempting to compute barycentric coordinates w.r.t. a degenerate tet");
		Eigen::Vector<T, 3> abc = A.inverse() * (p - vertices.col(indices[0]));
		Eigen::Vector<T, 4> weights;
		weights[0] = T(1.0) - abc[0] - abc[1] - abc[2];
		weights[1] = abc[0];
		weights[2] = abc[1];
		weights[3] = abc[2];
		return weights;
	}
}

template class BarycentricCoordinates<float, 3>;
template class BarycentricCoordinates<double, 3>;
template class BarycentricCoordinates<float, 4>;
template class BarycentricCoordinates<double, 4>;

} // namespace nls
} //namespace epic
