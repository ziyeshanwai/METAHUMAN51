// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/Variable.h>

#include <vector>

namespace epic {
namespace nls {


template <class T>
class VectorVariable : public Variable<T>
{
public:
	VectorVariable(int size) : Variable<T>(size) {}
	VectorVariable(const Vector<T>& vector) : Variable<T>(vector) {}

	void SetZero()
	{
		this->Set(Vector<T>::Zero(this->OutputDimension()));
	}

	virtual void Set(const Vector<T>& value) override
	{
		CARBON_PRECONDITION(value.size() == this->OutputDimension(), "size of x needs to match the output dimension of the VectorVariable");
		// project to manifold as there may be a subclass of VectorVariable that requires the projection
		this->SetValue(value.data(), value.size(), /*projectToManifold*/true);
	}

	virtual void Update(const Vector<T>& dx) override
	{
		CARBON_PRECONDITION(dx.size() == this->UpdateDimension(), "size of dx needs to match the update dimension of the VectorVariable");
		Set(this->Value() + dx);
	}

	virtual bool RealJacobian() override
	{
		return true;
	}

	//! Specialization of MakeMutable() to also clear any constant indices
	virtual void MakeMutable() override
	{
		Variable<T>::MakeMutable();
		m_constantIndices.clear();
	}

	//! Make invidual values of the variable constant
	void MakeIndividualIndicesConstant(const std::vector<int>& constantIndices)
	{
		for (int index : constantIndices) {
			if (index < 0 || index >= this->UpdateDimension()) {
				CARBON_CRITICAL("constant index out of bounds");
			}
		}
		m_constantIndices = constantIndices;
		this->InvalidateCachedJacobian();
	}

	//! @return which indices are constant
	const std::vector<int>& ConstantIndices() const { return m_constantIndices; }

protected:
	//! Only subclasses can call this constructor to set up a vector variable with a different update dimension
	VectorVariable(int size, int updateDimension) : Variable<T>(size, updateDimension) {}

private:
	virtual SparseMatrixConstPtr<T> CalculateLocalJacobianMatrix() override
	{
		const int Rows = this->OutputDimension();
		const int Cols = this->UpdateDimension();
		CARBON_PRECONDITION(Rows == Cols, "local jacobian calculation inside VectorVariable needs to have the same update and output dimension, otherwise the subclass needs to implement the method");
		SparseMatrixPtr<T> jacobianMatrix = std::make_shared<SparseMatrix<T>>(Rows, Cols);
		if (m_constantIndices.size() > 0) {
			std::vector<bool> mutableVector(Rows, true);
			for (int index : m_constantIndices) {
				mutableVector[index] = false;
			}
			for (int i = 0; i < Cols; i++) {
				if (mutableVector[i]) {
					jacobianMatrix->insert(i, i) = T(1);
				}
			}
			jacobianMatrix->makeCompressed();
		} else {
			jacobianMatrix->setIdentity();
		}
		return jacobianMatrix;
	}

	//! Nothing to do for a generic vector variable.
	virtual void ProjectToManifold(Vector<T>& /*value*/) override {}

private:
	std::vector<int> m_constantIndices;
};


} // namespace nls
} //namespace epic
