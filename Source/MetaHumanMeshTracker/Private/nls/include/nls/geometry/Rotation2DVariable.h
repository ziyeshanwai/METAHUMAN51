// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/Context.h>
#include <nls/DiffDataMatrix.h>
#include <nls/math/Math.h>
#include <nls/MatrixVariable.h>
#include <nls/geometry/RotationManifold.h>

NLS_DISABLE_EIGEN_WARNINGS
#include <Eigen/Geometry>
NLS_RENABLE_WARNINGS

namespace epic {
namespace nls {

/**
 * A 2D Rotation variable using a an angle as underlying representation.
 * https://en.wikipedia.org/wiki/Rotation_matrix
 */
template <class T>
class Rotation2DVariable : public MatrixVariable<T, 2, 2>
{
public:
	Rotation2DVariable() : MatrixVariable<T, 2, 2>(/*updateDimension=*/1)
	{
	}

	void SetAngle(const T& angle)
	{
		m_rotation = Eigen::Rotation2D<T>(angle);
		this->SetMatrix(m_rotation.toRotationMatrix(), /*projectToManifold=*/false);
	}

	T Angle() const
	{
		return m_rotation.angle();
	}

	virtual void Update(const Vector<T>& dx) override
	{
		CARBON_PRECONDITION(dx.size() == this->UpdateDimension(), "dx needs to match update dimension");
		m_rotation *= Eigen::Rotation2D<T>(dx[0]);
		this->SetMatrix(m_rotation.toRotationMatrix(), /*projectToManifold=*/false);

	}

	virtual bool RealJacobian() override
	{
		return true;
	}

private:
	Eigen::Matrix<T,4,1> CalculateDenseJacobianMatrix()
	{
		const T dSindAngle = cos(m_rotation.angle());
		const T dCosdAngle = - sin(m_rotation.angle());
		//Scalar sinA = sin(m_angle);
		//Scalar cosA = cos(m_angle);
		//return (Matrix2() << cosA, -sinA, sinA, cosA).finished();
		return Eigen::Matrix<T,4,1>(dCosdAngle, dSindAngle, -dSindAngle, dCosdAngle);
	}

	//! @return dR/dq i.e. Jacobian of 2D rotation matrix with respect to the rotation angle
	virtual SparseMatrixConstPtr<T> CalculateLocalJacobianMatrix() override
	{
		if (this->ConstantIndices().size() > 0) {
			throw std::runtime_error("Rotation2DVariable does not support partial contant indices");
		}

		// calculate Jacobian for 2d rotation
		const int Rows = this->OutputDimension();
		const int Cols = this->UpdateDimension();
		CARBON_ASSERT(Rows == 4, "invalid jacobian row size");
		CARBON_ASSERT(Cols == 1, "invalid jacobian column size");

		Eigen::Matrix<T,4,1> denseJacobianMatrix = CalculateDenseJacobianMatrix();
		SparseMatrixPtr<T> jacobianMatrix = std::make_shared<SparseMatrix<T>>(Rows, Cols);
		*jacobianMatrix = denseJacobianMatrix.sparseView();
		return jacobianMatrix;
	}

	virtual void ProjectToManifold(Vector<T>& value) override
	{
		// project the matrix to a rotation using SVD, then extract the angle
		CARBON_PRECONDITION(value.size() == 4, "size of value needs to match the output dimension");
		Eigen::Matrix2<T> rot = Eigen::Map<const Eigen::Matrix2<T>>(value.data());
		m_rotation.fromRotationMatrix(ProjectToClosestRotationMatrix<T,2>(rot));
		Eigen::Map<Eigen::Matrix<T,2,2>>(value.data()) = m_rotation.toRotationMatrix();
	}
private:
	Eigen::Rotation2D<T> m_rotation;
};


} // namespace nls
} //namespace epic
