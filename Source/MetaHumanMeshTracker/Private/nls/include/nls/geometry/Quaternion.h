// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>


namespace epic {
namespace nls {

/**
 * Quaternion to Rotation Matrix conversion that works for normalized and non-normalized quaternions.
 * If NORMALIZE is set to true, then the output rotation matrix will be an orthonormal rotation matrix.
 * norm(R) = sqrt(3)
 * If NORMALIZE is set to false, then the output rotation matrix will be scaled by the squared quaternion norm.
 * norm(R) = sqrt(3) * norm(q)^2
 */
template <class T, bool NORMALIZE>
Eigen::Matrix<T, 3, 3> QuaternionToRotationMatrix(const Vector<T>& quaternion)
{
	const T x = quaternion[0];
	const T y = quaternion[1];
	const T z = quaternion[2];
	const T w = quaternion[3];
	const T xx = x * x;
	const T yy = y * y;
	const T zz = z * z;
	const T ww = w * w;

	Eigen::Matrix<T, 3, 3> R;

	R.coeffRef(0,0) = (ww + xx - yy - zz);
	R.coeffRef(1,0) = T(2) * (x * y + z * w);
	R.coeffRef(2,0) = T(2) * (x * z - y * w);

	R.coeffRef(0,1) = T(2) * (x * y - z * w);
	R.coeffRef(1,1) = (ww - xx + yy - zz);
	R.coeffRef(2,1) = T(2) * (y * z + x * w);

	R.coeffRef(0,2) = T(2) * (x * z + y * w);
	R.coeffRef(1,2) = T(2) * (y * z - x * w);
	R.coeffRef(2,2) = (ww - xx - yy + zz);

	if (NORMALIZE) {
		const T normSquared = xx + yy + zz + ww;
		const T oneOverNormSquared = T(1.0) / normSquared;
		return R * oneOverNormSquared;
	}  else {
		return R;
	}
}


//! Calculates the inverse of the quaternion.
template <class T, bool NORMALIZE>
Eigen::Matrix<T, 4, 1> QuaternionInverse(const Eigen::Matrix<T, 4, 1>& q)
{
	if (NORMALIZE) {
		return Eigen::Matrix<T, 4, 1>(-q[0], -q[1], -q[2], q[3]).normalized();
	} else {
		return Eigen::Matrix<T, 4, 1>(-q[0], -q[1], -q[2], q[3]);
	}
}


template <class T, bool NORMALIZE>
Eigen::Matrix<T, 4, 1> QuaternionMultiplication(const Eigen::Matrix<T, 4, 1>& q1, const Eigen::Matrix<T, 4, 1>& q2)
{
	Eigen::Matrix<T, 4, 1> q;
	// q1 * q2 = [s1 u2 + s2 u1 + u1.cross(u2); s1 * s2 - u1.dot(u2)]
	q[0] = q1[3] * q2[0] + q2[3] * q1[0] + (q1[1] * q2[2] - q1[2] * q2[1]);
	q[1] = q1[3] * q2[1] + q2[3] * q1[1] + (q1[2] * q2[0] - q1[0] * q2[2]);
	q[2] = q1[3] * q2[2] + q2[3] * q1[2] + (q1[0] * q2[1] - q1[1] * q2[0]);
	q[3] = q1[3] * q2[3] - (q1[0] * q2[0] +  q1[1] * q2[1] + q1[2] * q2[2]);

	if (NORMALIZE) {
		return q.normalized();
	} else {
		return q;
	}
}


/**
 * Rotates vector \p p using the quaterion \p q. If NORMALIZE is false then it assumes that the quaternion is normalized,
 * otherwise the return vector will be scaled by the squaredNorm of q.
 */
template <class T, bool NORMALIZE>
Eigen::Matrix<T, 3, 1> QuaternionVectorMultiplication(const Eigen::Matrix<T, 4, 1>& q, const Eigen::Matrix<T, 3, 1>& p)
{
	const Eigen::Matrix<T, 4, 1> pext(p[0], p[1], p[2], 0);
	const Eigen::Matrix<T, 4, 1> qinv = QuaternionInverse<T, false>(q);
	const Eigen::Matrix<T, 4, 1> result = QuaternionMultiplication<T, false>(q, QuaternionMultiplication<T, false>(pext, qinv));
	CARBON_PRECONDITION(fabs(result[3]) < T(1e-6), "vector multiplication should keep w equal 0");
	if (NORMALIZE) {
		const T invNorm = T(1.0) / q.squaredNorm();
		return Eigen::Matrix<T, 3, 1>(result[0], result[1], result[2]) * invNorm;
	} else {
		return Eigen::Matrix<T, 3, 1>(result[0], result[1], result[2]);
	}
}


/**
 * Calculates the Jacobian dR/dq i.e of the rotation matrix R with respect to a Quaternion q. To get consistent Jacobians
 * the normalization flag for JacobianOfQuaternionToRotationMatrix and QuaternionToRotationMatrix need to match.
 */
template <class T, bool NORMALIZE>
Eigen::Matrix<T, 9, 4> JacobianOfQuaternionToRotationMatrix(const Eigen::Vector4<T>& quaternion)
{
	const T x = quaternion[0];
	const T y = quaternion[1];
	const T z = quaternion[2];
	const T w = quaternion[3];

	// CONVERSION from quaternion to rotation matrix
	// R.coeffRef(0,0) = (ww + xx - yy - zz);
	// R.coeffRef(1,0) = T(2) * (x * y + z * w);
	// R.coeffRef(2,0) = T(2) * (x * z - y * w);

	// R.coeffRef(0,1) = T(2) * (x * y - z * w);
	// R.coeffRef(1,1) = (ww - xx + yy - zz);
	// R.coeffRef(2,1) = T(2) * (y * z + x * w);

	// R.coeffRef(0,2) = T(2) * (x * z + y * w);
	// R.coeffRef(1,2) = T(2) * (y * z - x * w);
	// R.coeffRef(2,2) = (ww - xx - yy + zz);

	Eigen::Matrix<T, 9, 4> J;
	J.setZero();
	// (0,0)
	J.coeffRef(0, 0) = T( 2) * x;
	J.coeffRef(0, 1) = T(-2) * y;
	J.coeffRef(0, 2) = T(-2) * z;
	J.coeffRef(0, 3) = T( 2) * w;
	// (1,0)
	J.coeffRef(1, 0) = T(2) * y;
	J.coeffRef(1, 1) = T(2) * x;
	J.coeffRef(1, 2) = T(2) * w;
	J.coeffRef(1, 3) = T(2) * z;
	// (2,0)
	J.coeffRef(2, 0) = T(2) * z;
	J.coeffRef(2, 1) = T(-2) * w;
	J.coeffRef(2, 2) = T(2) * x;
	J.coeffRef(2, 3) = T(-2) * y;
	// (0,1)
	J.coeffRef(3, 0) = T(2) * y;
	J.coeffRef(3, 1) = T(2) * x;
	J.coeffRef(3, 2) = T(-2) * w;
	J.coeffRef(3, 3) = T(-2) * z;
	// (1,1)
	J.coeffRef(4, 0) = T(-2) * x;
	J.coeffRef(4, 1) = T( 2) * y;
	J.coeffRef(4, 2) = T(-2) * z;
	J.coeffRef(4, 3) = T( 2) * w;
	// (2,1)
	J.coeffRef(5, 0) = T(2) * w;
	J.coeffRef(5, 1) = T(2) * z;
	J.coeffRef(5, 2) = T(2) * y;
	J.coeffRef(5, 3) = T(2) * x;
	// (0,2)
	J.coeffRef(6, 0) = T(2) * z;
	J.coeffRef(6, 1) = T(2) * w;
	J.coeffRef(6, 2) = T(2) * x;
	J.coeffRef(6, 3) = T(2) * y;
	// (1,2)
	J.coeffRef(7, 0) = T(-2) * w;
	J.coeffRef(7, 1) = T(2) * z;
	J.coeffRef(7, 2) = T(2) * y;
	J.coeffRef(7, 3) = T(-2) * x;
	// (2,2)
	J.coeffRef(8, 0) = T(-2) * x;
	J.coeffRef(8, 1) = T(-2) * y;
	J.coeffRef(8, 2) = T( 2) * z;
	J.coeffRef(8, 3) = T( 2) * w;

	if (NORMALIZE) {
		// for the normalized Quaternion q to R conversion we would also need to take into account the normalization
		// e.g.
		// dR/dx = dR/dn dn/dx + dR/dx * oneOverNormSquared
		//       =  R dn/dx + dR/dx * oneOverNormSquared
		//       = -2x oneOverNormSquared^2 R + dR/dx * oneOverNormSquared
		//
		// or alternatively we multiply the jacobian with the jacobian of the prior normalization (JacobianOfVectorNormalization)
		const T normSquared = x * x + y * y + z * z + w * w;
		const T oneOverNormSquared = T(1.0) / normSquared;
		const Eigen::Matrix<T, 3, 3> R = QuaternionToRotationMatrix<T,true>(quaternion); // already contains one oneOverNormSquared

		for (int k = 0; k < 4; k++) {
			for (int j = 0; j < 9; j++) {
				J.coeffRef(j, k) = - T(2) * quaternion[k] * oneOverNormSquared * R.data()[j] + J.coeffRef(j, k) * oneOverNormSquared;
			}
		}
	}

	return J;
}


/**
 * Implements the special case of the Jacobian dR/dq of the rotation matrix R with respect to the identity quaternion q i.e.
 * q = [0,0,0,1]. (i.e. apply these numbers to JacobianOfQuaternionToRotationMatrix())
 *
 * In case of the calculating Jacobians for normalized quaternions the values for the w coordinate cancel completely out,
 * and therefore that dimension can be dropped in an optimization.
 * (see Iterative Estimation of Rotation and Translation using the Quaternion, by Mark Wheeler, 1995)
 */
template <class T, bool NORMALIZE>
Eigen::Matrix<T, 9, 4> JacobianOfIdentityQuaternionToRotationMatrix()
{
	// const T x = 0;
	// const T y = 0;
	// const T z = 0;
	// const T w = 1;

	Eigen::Matrix<T, 9, 4> J;
	J.setZero();
	// (0,0)
	if (NORMALIZE) {
		//J.coeffRef(0, 3) = T(2) - T(2); // as we also assume that the quaternion is normalized, the derivative of the normalization cancels out
	} else {
		J.coeffRef(0, 3) = T(2);
	}
	// (1,0)
	J.coeffRef(1, 2) = T(2);
	// (2,0)
	J.coeffRef(2, 1) = T(-2);
	// (0,1)
	J.coeffRef(3, 2) = T(-2);
	// (1,1)
	if (NORMALIZE) {
		// J.coeffRef(4, 3) =  T(2) - T(2); // as we also assume that the quaternion is normalized, the derivative of the normalization cancels out
	} else {
		J.coeffRef(4, 3) =  T(2);
	}
	// (2,1)
	J.coeffRef(5, 0) = T(2);
	// (0,2)
	J.coeffRef(6, 1) = T(2);
	// (1,2)
	J.coeffRef(7, 0) = T(-2);
	// (2,2)
	if (NORMALIZE) {
		// J.coeffRef(8, 3) = T(2) - T(2); // as we also assume that the quaternion is normalized, the derivative of the normalization cancels out
	} else {
		J.coeffRef(8, 3) = T(2);
	}

	return J;
}


} // namespace nls
} //namespace epic
