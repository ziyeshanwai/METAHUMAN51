// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/geometry/RotationManifold.h>
#include <nls/math/Math.h>

namespace epic {
namespace nls {

/**
 * Projects the matrix M to the closest Rotation Matrix using SVD.
 * M = UDV'
 * R = UV' (or U [1 ... -1] V' if M is a reflection with a negative determinant)
 */
template <class T, int SIZE>
Eigen::Matrix<T,SIZE,SIZE> ProjectToClosestRotationMatrix(const Eigen::Matrix<T,SIZE,SIZE>& M)
{
	const Eigen::JacobiSVD<Eigen::Matrix<T,SIZE,SIZE>> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
	if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0) {
		// M is a reflection, so we need to invert the matrix
		Eigen::DiagonalMatrix<T,SIZE> diag;
		diag.setIdentity();
		diag.diagonal().coeffRef(SIZE-1) = T(-1);
		return svd.matrixU() * diag * svd.matrixV().transpose();
	} else {
		return svd.matrixU() * svd.matrixV().transpose();
	}
}

template Eigen::Matrix<float,2,2> ProjectToClosestRotationMatrix<float,2>(const Eigen::Matrix<float,2,2>& M);
template Eigen::Matrix<float,3,3> ProjectToClosestRotationMatrix<float,3>(const Eigen::Matrix<float,3,3>& M);
template Eigen::Matrix<double,2,2> ProjectToClosestRotationMatrix<double,2>(const Eigen::Matrix<double,2,2>& M);
template Eigen::Matrix<double,3,3> ProjectToClosestRotationMatrix<double,3>(const Eigen::Matrix<double,3,3>& M);

} // namespace nls
} //namespace epic
