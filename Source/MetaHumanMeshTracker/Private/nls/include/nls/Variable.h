// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/Context.h>
#include <nls/DiffData.h>
#include <nls/math/Math.h>

namespace epic {
namespace nls {

template <class T>
class Variable
{
public:
	using value_type = T;

public:
	Variable(const Vector<T>& vector)
		: m_outputDimension(int(vector.size()))
		, m_updateDimension(int(vector.size()))
		, m_constant(false)
		, m_value(std::make_shared<Vector<T>>(vector))
		, m_cachedLocalJacobian()
	{}

	Variable(int outputDimension, int updateDimension)
		: m_outputDimension(outputDimension)
		, m_updateDimension(updateDimension)
		, m_constant(false)
		, m_value()
		, m_cachedLocalJacobian()
	{
	}

	Variable(int outputDimension)
		: Variable(outputDimension, outputDimension)
	{}

	virtual ~Variable() {}

	//! explicitly delete the copy and constructor and assignment operator to prevent accidental errors where a variable may be copied and the mechanism for registering the Jacobian with a Context would fail.
	Variable(const Variable&) = delete;
	Variable& operator=(const Variable&) = delete;

	//! accept copy and move constructor as we are not duplicating the objects this way
	Variable(Variable&&) = default;
	Variable& operator=(Variable&&) = default;

	int OutputDimension() const { return m_outputDimension; }
	int UpdateDimension() const { return m_updateDimension; }
	int Size() const { return OutputDimension(); }

	void MakeConstant() { m_constant = true; }

	virtual void MakeMutable() { m_constant = false; }

	bool IsConstant() const { return m_constant; }

	const Vector<T>& Value() const
	{
		CARBON_PRECONDITION(m_value && m_value->size() == m_outputDimension, "variable needs to be initialized before use");
		return *m_value;
	}

	DiffData<T> Evaluate(Context<T>* context = nullptr)
	{
		CARBON_PRECONDITION(m_value && m_value->size() == m_outputDimension, "variable needs to be initialized before use");
		if (context && !IsConstant())
		{
			return DiffData<T>(m_value, GlobalJacobianMatrixPtr(context));
		}
		else
		{
			return DiffData<T>(m_value);
		}
	}

	/**
	 * Sets the variable \p value. The dimension of \p value needs to be OutputDimension(). A variable may have its internal valid manifold, and the
	 * function will project the value to the corresponding manifold. This means that calling Value() will return a value that may be different from the one passed to
	 * Set().
	 */
	virtual void Set(const Vector<T>& value) = 0;

	//! Updates the variable values by an offset dx. The dimension of dx needs to be UpdateDimension()
	virtual void Update(const Vector<T>& dx) = 0;

	//! Method indicating with the Jacobian is the real Jacobian or some simplification (e.g. the Jacobian may not include the projection to the manifold).
	virtual bool RealJacobian() = 0;

protected:
	void SetValue(const VectorConstPtr<T>& valuePtr, bool projectToManifold)
	{
		CARBON_PRECONDITION(valuePtr, "ptr needs to be valid");
		CARBON_PRECONDITION(valuePtr->size() == OutputDimension(), "size of vector needs to match the output dimension of the variable");
		if (projectToManifold) {
			SetValue(valuePtr->data(), valuePtr->size(), projectToManifold);
		} else {
			SetValue(valuePtr);
		}
	}

	void SetValue(const T* data, int64_t size, bool projectToManifold)
	{
		CARBON_PRECONDITION(data || size == 0, "ptr needs to be valid or size of variable needs to be zero");
		CARBON_PRECONDITION(int(size) == OutputDimension(), "size of vector needs to match the output dimension of the variable");
		VectorPtr<T> newValuePtr = std::make_shared<Vector<T>>(Eigen::Map<const Vector<T>>(data, size));
		if (projectToManifold) {
			ProjectToManifold(*newValuePtr);
		}
		SetValue(newValuePtr);
	}

	void InvalidateCachedJacobian()
	{
		m_cachedLocalJacobian.reset();
	}


private:
	JacobianConstPtr<T> GlobalJacobianMatrixPtr(Context<T>* context)
	{
		CARBON_PRECONDITION(context, "context needs to be valid");

		// Get Local Variable Jacobian
		SparseMatrixConstPtr<T> localJacobian = LocalJacobianMatrixPtr();
		CARBON_PRECONDITION(localJacobian, "local jacobian needs to be valid");
		CARBON_PRECONDITION(localJacobian->rows() == OutputDimension(), "number of rows of the local jacobian needs to match the output dimensions");
		CARBON_PRECONDITION(localJacobian->cols() == UpdateDimension(), "number of columns of the local jacobian needs to match the update dimensions");

		// Call context and get output Jacobian (column position for each variable is shifted according to the context)
		JacobianConstPtr<T> globalJacobian = context->Map(this, localJacobian);
		CARBON_POSTCONDITION(globalJacobian, "global jacobian needs to be valid");
		CARBON_POSTCONDITION(globalJacobian->Rows() == OutputDimension(), "number of rows of the global jacobian needs to match the output dimensions");
		CARBON_POSTCONDITION(globalJacobian->Cols() >= UpdateDimension(), "number of columns of the global jacobian needs to be largger or equal the update dimensions");

		return globalJacobian;
	}

	void SetValue(const VectorConstPtr<T>& valuePtr)
	{
		CARBON_PRECONDITION(valuePtr, "ptr needs to be valid");
		CARBON_PRECONDITION(valuePtr->size() == OutputDimension(), "size of vector needs to match the output dimension of the variable");
		m_value = valuePtr;
		m_cachedLocalJacobian.reset();
	}

	SparseMatrixConstPtr<T> LocalJacobianMatrixPtr()
	{
		CARBON_PRECONDITION(!IsConstant(), "variable should not be constant when querying the jacobian");
		if (!m_cachedLocalJacobian) {
			m_cachedLocalJacobian = CalculateLocalJacobianMatrix();
		}
		return m_cachedLocalJacobian;
	}

	virtual SparseMatrixConstPtr<T> CalculateLocalJacobianMatrix() = 0;

	//! Projects the variable to the valid manifold of the underlying representation.
	virtual void ProjectToManifold(Vector<T>& value) = 0;

private:
	int m_outputDimension;
	int m_updateDimension;
	bool m_constant = false;
	VectorConstPtr<T> m_value;
	SparseMatrixConstPtr<T> m_cachedLocalJacobian;
};


} // namespace nls
} //namespace epic
