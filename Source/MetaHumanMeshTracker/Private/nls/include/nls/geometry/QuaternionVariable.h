// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>
#include <nls/MatrixVariable.h>
#include <nls/geometry/Jacobians.h>
#include <nls/geometry/Quaternion.h>
#include <nls/geometry/RotationManifold.h>

NLS_DISABLE_EIGEN_WARNINGS
#include <Eigen/Geometry>
NLS_RENABLE_WARNINGS

namespace epic {
namespace nls {


/**
 * A Rotation variable using a quaternion as the underlying representation. Quaternion updates are handled "incrementally" as
 * described in "Iterative Estimation of Rotation and Translation using the Quaternion", by Mark Wheeler, 1995
 * https://www.ri.cmu.edu/publications/iterative-estimation-of-rotation-and-translation-using-the-quaternion/.
 * The advantage is that there are only 3 unknowns instead of the general case of 4 variables per quaternion.
 *
 * There are two options:
 * PREMULTIPLY=true: R' = R * dR
 * PREMULTIPLY=false: R' = dR * R
 */
template <class T, bool PREMULTIPLY=true>
class QuaternionVariable : public MatrixVariable<T, 3, 3>
{
public:
	QuaternionVariable() : MatrixVariable<T, 3, 3>(/*updateDimension=*/3) {}

	virtual void Update(const Vector<T>& dx) override
	{
		CARBON_PRECONDITION(dx.size() == this->UpdateDimension(), "dx needs to match update dimension");

		Eigen::Matrix<T,4,1> dQ(dx[0], dx[1], dx[2], T(1.0));
		if (PREMULTIPLY) {
			m_quaternion.coeffs() = QuaternionMultiplication<T,/*NORMALIZE=*/true>(m_quaternion.coeffs(), dQ);
		} else {
			m_quaternion.coeffs() = QuaternionMultiplication<T,/*NORMALIZE=*/true>(dQ, m_quaternion.coeffs());
		}
		this->SetMatrix(QuaternionToRotationMatrix<T,/*NORMALIZE=*/false>(m_quaternion.coeffs()), /*projectToManifold=*/false);
	}

	virtual bool RealJacobian() override
	{
		// the Jacobian is the full Jacobian as it includes the normalization of the quaternion and
		// therefore the rotation is always in SO3.
		return true;
	}

private:
	//! @return dR/dq i.e. Jacobian of 3x3 rotation matrix with respect to a normalized quaternion
	virtual SparseMatrixConstPtr<T> CalculateLocalJacobianMatrix() override
	{
		if (this->ConstantIndices().size() > 0) {
			throw std::runtime_error("QuaternionVariable does not support partial contant indices");
		}

		const int Rows = this->OutputDimension();
		const int Cols = this->UpdateDimension();
		CARBON_ASSERT(Rows == 9, "invalid jacobian row size");
		CARBON_ASSERT(Cols == 3, "invalid jacobian column size");
		SparseMatrixConstPtr<T> J;
		// As we are using the delta to the identity quaternion to represent a delta rotation, we need
		// to evaluate the Jacobian at the identity quaternion which results
		// in the simplified version of the Jacobian as described in JacobianOfIdentityQuaternionToRotationMatrix().
		// Given that we are working in normalized quaternions the w component is completely gone, and the
		// rest ot the Jacobian has the structure of a CrossProductMatrix.
		if (PREMULTIPLY) {
			J = std::make_shared<SparseMatrix<T>>(JacobianOfCrossProductMatrixPreMultipliedByMatrix<T,3>(this->Matrix(), T(2.0)));

		} else {
			J = std::make_shared<SparseMatrix<T>>(JacobianOfCrossProductMatrixPostMultipliedByMatrix<T,3>(this->Matrix(), T(2.0)));
		}
		CARBON_POSTCONDITION(int(J->rows()) == Rows, "invalid jacobian row size");
		CARBON_POSTCONDITION(int(J->cols()) == Cols, "invalid jacobian column size");

		return J;
	}

	virtual void ProjectToManifold(Vector<T>& value) override
	{
		// First convert the 3x3 matrix to the closest rotation using SVD projection. Then convert the matrix to the quaternion.
		CARBON_PRECONDITION(value.size() == 9, "size of value needs to match the output dimension");
		Eigen::Matrix3<T> rot = Eigen::Map<const Eigen::Matrix3<T>>(value.data());
		rot = ProjectToClosestRotationMatrix<T,3>(rot);
		m_quaternion = Eigen::Quaternion<T>(rot);
		m_quaternion.normalize();
		Eigen::Map<Eigen::Matrix<T,3,3>>(value.data()) = QuaternionToRotationMatrix<T,/*NORMALIZE=*/false>(m_quaternion.coeffs());
	}

private:
	Eigen::Quaternion<T> m_quaternion;
};


} // namespace nls
} //namespace epic
