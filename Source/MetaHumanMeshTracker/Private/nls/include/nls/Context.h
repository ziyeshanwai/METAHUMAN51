// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/Jacobian.h>
#include <nls/math/Math.h>

#include <map>
#include <vector>

namespace epic {
namespace nls {

template <class T>
class Variable;

/**
 * The Context class provides the variable context when creating nonlinear optimization problems. In particular
 * a Variable registers its local Jacobian with the Context and retrieves a modified Jacobian where the variable
 * placement (the column indices) has been modified so that multiple Variables can coexist. The functions
 * Value(), Set(), and Update() work similarly to the Variable implementation, but in this case these methods update
 * all Variables that have been registered by the Context.
 *
 * Once a variable has registered a local jacobian it is expeced to remain the same until Next() is being called on the
 * Context. This indicates that the variables have changed and therefore the Jacobians may also change. Note
 * that Update() and Set() wil automatically call Next().
 */
template <class T>
class Context
{
public:
	Context() : m_valueSize(0), m_updateSize(0), m_variableMap() {}

	void Clear()
	{
		m_valueSize = 0;
		m_updateSize = 0;
		m_variableMap.clear();
	}

	void Next()
	{
		for (auto& keyValue: m_variableMap)
		{
			keyValue.second.queries = 0;
		}
	}

	JacobianConstPtr<T> Map(Variable<T>* var, const SparseMatrixConstPtr<T>& localJacobian)
	{
		CARBON_PRECONDITION(localJacobian, "jacobian should be valid");
		CARBON_PRECONDITION(localJacobian->isCompressed(), "jacobian needs to be compressed");

		if (!localJacobian) {
			return JacobianConstPtr<T>();
		}

		auto varIter = m_variableMap.find(var);
		if (varIter == m_variableMap.end())
		{
			// variable not yet mapped
			const int cols = int(localJacobian->cols());
			const int rows = int(localJacobian->rows());
			const int colOffset = m_updateSize;
			const int rowOffset = m_valueSize;
			m_updateSize += cols;
			m_valueSize += rows;

			const JacobianConstPtr<T> globalJacobian = LocalToGlobalJacobian(localJacobian, colOffset);
			m_variableMap[var] = VariableInfo{localJacobian, globalJacobian, rows, cols, rowOffset, colOffset, /*queries=*/1};

			return globalJacobian;
		}
		else
		{
			VariableInfo& info = varIter->second;
			if (localJacobian != info.localJacobian) {
				// The jacobian should not change for multiple queries otherwise this would indicate an incorrect
				// implementation of the variable jacobian. When calling var.Evaluate() twice it should return
				// the exact same local jacobian with the same pointer i.e. the class needs to cache the Jacobian.
				CARBON_ASSERT(info.queries == 0, "the jacobian should not change for multiple queries");
				CARBON_ASSERT(localJacobian->cols() == info.localJacobian->cols(), "size of variable should not change");
				CARBON_ASSERT(localJacobian->rows() == info.localJacobian->rows(), "size of variable should not change");
				info.localJacobian = localJacobian;
				info.globalJacobian = LocalToGlobalJacobian(localJacobian, info.colOffset);
			}
			info.queries++;
			return info.globalJacobian;
		}
	}

	void Set(const Vector<T>& x)
	{
		if (x.size() != m_valueSize) {
			throw std::runtime_error(std::string("x size and value size do not match: " + std::to_string(x.size()) + " vs " + std::to_string(m_valueSize)));
		}

		for (auto& keyValue: m_variableMap)
		{
			Variable<T>* var = keyValue.first;
			const VariableInfo& info = keyValue.second;
			CARBON_ASSERT(info.rowOffset + info.rows <= x.size(), "vector size does not match variable placement in context");
			var->Set(x.segment(info.rowOffset, info.rows));
		}

		Next(); // variables have changed so we set all queries to nill
	}

	//! @returns the current variable vector defined by the mapping in this context
	Vector<T> Value()
	{
		Vector<T> value(m_valueSize);
		value.setZero();

		for (auto& keyValue: m_variableMap)
		{
			Variable<T>* var = keyValue.first;
			const VariableInfo& info = keyValue.second;
			CARBON_ASSERT(info.rowOffset + info.rows <= value.size(), "vector size does not match variable placement in context");
			value.segment(info.rowOffset, info.rows) = var->Value();
		}
		return value;
	}

	void Update(const Vector<T>& dx)
	{
		if (dx.size() != UpdateSize()) {
			throw std::runtime_error(std::string("dx size and update size do not match: " + std::to_string(dx.size()) + " vs " + std::to_string(UpdateSize())));
		}

		for (auto& keyValue: m_variableMap)
		{
			Variable<T>* var = keyValue.first;
			const VariableInfo& info = keyValue.second;
			CARBON_ASSERT(info.colOffset + info.cols <= dx.size(), "update vector size does not match variable placement in context");
			var->Update(dx.segment(info.colOffset, info.cols));
		}

		Next(); // variables have changed so we set all queries to null
	}

	std::vector<Variable<T>*> Variables()
	{
		std::vector<Variable<T>*> vars;
		vars.reserve(m_variableMap.size());
		for (auto& keyValue: m_variableMap)
		{
			vars.push_back(keyValue.first);
		}
		return vars;
	}

	int ValueSize() const { return m_valueSize; }
	int UpdateSize() const { return m_updateSize; }

	//! @returns the column index and size of the variable in the jacobian
	std::pair<int, int> MappedVariableIndexAndSize(const Variable<T>* var) const
	{
		auto it = m_variableMap.find(const_cast<Variable<T>*>(var));
		if (it != m_variableMap.end()) {
			return std::pair<int, int>(it->second.colOffset, it->second.cols);
		} else {
			throw std::runtime_error("Variable is not mapped by context");
		}
	}

private:
	JacobianConstPtr<T> LocalToGlobalJacobian(const SparseMatrixConstPtr<T>& localJacobian, int columnIndex)
	{
		static_assert(SparseMatrix<T>::IsRowMajor, "context only works with row major sparse matrices");
		CARBON_PRECONDITION(localJacobian, "jacobian needs to be valid");
		CARBON_PRECONDITION(localJacobian->isCompressed(), "jacobian needs to be compressed");
		SparseMatrixPtr<T> globalJacobian = std::make_shared<SparseMatrix<T>>(*localJacobian);
		CARBON_ASSERT(globalJacobian->isCompressed(), "global jacobian needs to be compressed");
		CARBON_PRECONDITION(columnIndex + localJacobian->cols() <= m_updateSize, "column index exceeds global jacobian size");

		// resize jacobian to full size (note that this is for free as m_size > cols)
		globalJacobian->conservativeResize(localJacobian->rows(), m_updateSize);
		int* columnIndices = globalJacobian->innerIndexPtr();
		for (int i = 0; i < globalJacobian->nonZeros(); i++) {
			columnIndices[i] += columnIndex;
		}
		return std::make_shared<SparseJacobian<T>>(globalJacobian);
	}


private:
	int m_valueSize;
	int m_updateSize;

	struct VariableInfo
	{
		//! local jacobian of size rows x cols
		SparseMatrixConstPtr<T> localJacobian;

		//! global jacobian which is the jacobian shifted by colOffset, total size rows x m_updateSize (when the global Jacobian was last constructed)
		JacobianConstPtr<T> globalJacobian;
		int rows; // variable output dimension
		int cols; // variable update dimension
		int rowOffset; // offset into global value dimension
		int colOffset; // offset into global update dimension
		int queries; // how often the variable has been queries since the last call to Next()
	};

	std::map<Variable<T>*, VariableInfo> m_variableMap;
};

} // namespace nls
} //namespace epic
