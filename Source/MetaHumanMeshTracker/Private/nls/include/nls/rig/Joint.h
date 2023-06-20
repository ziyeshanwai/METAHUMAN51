// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/DiffDataMatrix.h>
#include <nls/functions/MatrixMultiplyFunction.h>
#include <nls/geometry/Affine.h>
#include <nls/geometry/DiffDataAffine.h>

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

namespace epic {
namespace nls {


template <class T> struct JointHelper {};

template <class T, int R, int C>
struct JointHelper<Eigen::Matrix<T,R,C>> {  using SCALAR = T; };

template <class T, int R, int C> struct JointHelper<DiffDataAffine<T, R, C>> {  using SCALAR = T; };
template <class T, int R, int C> struct JointHelper<Affine<T, R, C>> {  using SCALAR = T; };

/**
 * A Joint represents a local transformation
*/
template <class AffineType>
class Joint
{
	using T = typename JointHelper<AffineType>::SCALAR;

public:
	Joint(const std::string& name = "") : m_name(name), m_parent(nullptr) {}

	const std::string& Name() const { return m_name; }

	bool HasParent() const { return bool(m_parent); }

	bool IsRoot() const { return !HasParent(); }

	const Joint* Parent() const { return m_parent; }

	bool SetParent(Joint* parent) {
		if (parent == this) return false;
		if (HasChild(parent, /*recursive=*/true)) {
			return false;
		}

		if (m_parent != parent) {
			if (m_parent) {
				m_parent->RemoveChild(this, /*updateChild=*/false);
			}
			m_parent = parent;
			if (m_parent) {
				m_parent->AddChild(this);
			}
		}
		return true;
	}

	void AddChild(Joint* child) {
		if (std::find(m_children.begin(), m_children.end(), child) == m_children.end()) {
			m_children.push_back(child);
			child->SetParent(this);
		}
	}

	void RemoveChild(Joint* child) {
		RemoveChild(child, /*updateChild=*/true);
	}

	int NumChildren() const { return int(m_children.size()); }

	//! @returns whether joint @p j is a child of this joint. Use @p recursive to check recursively.
	bool HasChild(Joint* j, bool recursive) const
	{
		auto it = std::find(m_children.begin(), m_children.end(), j);
		if (it != m_children.end()) {
			return true;
		}
		if (recursive) {
			for (const Joint* child : m_children) {
				if (child->HasChild(j, recursive)) {
					return true;
				}
			}
		}
		return false;
	}

	const AffineType& LocalMatrix()
	{
		return m_localMatrix;
	}

	void SetLocalMatrix(const AffineType& localMatrix)
	{
		m_localMatrix = localMatrix;
		InvalidateWorldMatrix();
	}

	const Affine<T,3,3>& BindMatrix() const
	{
		return m_bindMatrix;
	}

	void SetBindMatrix(const Affine<T,3,3>& bindMatrix)
	{
		m_bindMatrix = bindMatrix;
		m_bindMatrixInverse = bindMatrix.Matrix().inverse();
		InvalidateWorldMatrix();
	}

	const AffineType& WorldMatrix() {
		if (!m_worldMatrix.has_value()) {
			if (m_parent) {
				m_worldMatrix = m_parent->WorldMatrix() * m_localMatrix;
			} else {
				m_worldMatrix = m_localMatrix;
			}
		}
		return m_worldMatrix.value();
	}

	const AffineType& SkinningMatrix() {
		if (!m_skinningMatrix.has_value()) {
			AffineType worldMatrix = WorldMatrix();
			m_skinningMatrix = worldMatrix * m_bindMatrixInverse;
		}
		return m_skinningMatrix.value();
	}

private:
	void RemoveChild(Joint* child, bool updateChild) {
		auto it = std::find(m_children.begin(), m_children.end(), child);
		if (it != m_children.end()) {
			m_children.erase(it);
			if (updateChild) {
				child->SetParent(nullptr);

			}
		}
	}

	//! Set the world matrix as invalid, and therefore all children are also invalid
	void InvalidateWorldMatrix()
	{
		if (m_worldMatrix.has_value()) {
			// only call the children if the world matrix was valid before, otherwise the children will be invalid anyway
			m_worldMatrix.reset();
			m_skinningMatrix.reset();
			for (Joint<AffineType>* child : m_children) {
				child->InvalidateWorldMatrix();
			}
		}
	}

private:
	//! name of this joint
	std::string m_name;

	//! parent
	Joint<AffineType>* m_parent;

	//! children
	std::vector<Joint<AffineType>*> m_children;

	//! local transformation matrix
	AffineType m_localMatrix;

	//! bind matrix
	Affine<T,3,3> m_bindMatrix;

	//! inverse of the bind matrix
	Affine<T,3,3> m_bindMatrixInverse;

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4324)
#endif

	//! world transformation matrix
	std::optional<AffineType> m_worldMatrix;

	//! skinning matrix (m_worldMatrix * m_bindMatrixInverse)
	std::optional<AffineType> m_skinningMatrix;

#ifdef _MSC_VER
#pragma warning(pop)
#endif

};


} // namespace nls
} //namespace epic
