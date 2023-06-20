// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/Jacobian.h>
#include <nls/math/Math.h>

namespace epic {
namespace nls {

/**
 * Base data representation required for differentiable data with a vector of values and a Jacobian
 * values: f(x)
 * jacobian: df/dx
 */
template <class T>
class DiffData
{
public:
	DiffData(const VectorConstPtr<T>& values, const JacobianConstPtr<T>& jacobian) : m_value(values), m_jacobian(jacobian)
	{
		SanityCheck();
	}

	DiffData(const VectorConstPtr<T>& values) : DiffData(values, JacobianConstPtr<T>()) {}

	//! constructor that explicitly copies the data
	DiffData(const T* values, int size) : DiffData(std::make_shared<Vector<T>>(Eigen::Map<const Vector<T>>(values, size))) {}

	//! constructor that explicitly copies the data
	template <class S, int R, int C, typename DISCARD = typename std::enable_if<std::is_same<S,T>::value, void>::type>
	DiffData(const Eigen::Matrix<S,R,C>& mat) : DiffData(std::make_shared<Vector<T>>(Eigen::Map<const Vector<T>>(mat.data(), mat.size()))) {}

	//! constructor that explicitly copies the data
	template <class S, int R, int C, typename DISCARD = typename std::enable_if<std::is_same<S,T>::value, void>::type>
	DiffData(const Eigen::Matrix<S,R,C>& mat, const JacobianConstPtr<T>& jacobian) : DiffData(std::make_shared<Vector<T>>(Eigen::Map<const Vector<T>>(mat.data(), mat.size())), jacobian) {}

	void SanityCheck() const
	{
		CARBON_PRECONDITION(m_value, "value vector needs to be valid");
		if (m_jacobian) {
			CARBON_PRECONDITION(m_jacobian->Rows() == int(m_value->size()), "jacobian needs to match value vector size");
		}
	}

	int Size() const { return int(Value().size()); }
	const VectorConstPtr<T>& ValuePtr() const { return m_value; }
	const Vector<T>& Value() const { return *m_value; }

	bool HasJacobian() const { return bool(m_jacobian.get()); }
	const JacobianConstPtr<T>& JacobianPtr() const { return m_jacobian; }
	const epic::nls::Jacobian<T>& Jacobian() const { return *m_jacobian; }

	void SetJacobianPtr(const JacobianConstPtr<T>& jacobian) { m_jacobian = jacobian; }

	/**
     * Convenience function returning a reference to the value.
     * @warning Use with care as any copy of DiffData will have its value modified as well.
     */
	Vector<T>& MutableValue() { return const_cast<Vector<T>&>(*m_value); }

	/**
	 * Convenience function returning a reference to the jacobian ptr.
	 */
	JacobianConstPtr<T>& MutableJacobianPtr() { return m_jacobian; }

private:
	VectorConstPtr<T> m_value;
	JacobianConstPtr<T> m_jacobian;
};

} // namespace nls
} //namespace epic
