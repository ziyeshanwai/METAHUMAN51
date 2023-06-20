// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/math/Math.h>

namespace epic {
namespace nls {

/**
 * An affine transformation maps y = Mx + b where M is the linear transformation and b a translation
 * https://en.wikipedia.org/wiki/Affine_transformation
 *
 * The matrix is stored as a homogenous matrix i.e.
 * A = 	|M b|
 * 		|0 1|
 *
 * Hence,
 * |y| = A * |x|
 * |1|		 |1|
 *
 * Dimensions:
 * R: row dimension of M and b
 * C: column dimension of M
 *
 * Dimensions of the internal matrix: (R+1),(C+1). This is consitent with Eigen::Transform.
 */
template <class T, int R, int C, typename DISCARD = typename std::enable_if<(R > 0 && C > 0), void>::type>
class Affine
{
public:
  	Affine()
	{
        m_affineMatrix.setIdentity();
	}

	Affine(const Eigen::Matrix<T,R+1,C+1>& mat) : m_affineMatrix(mat)
	{
	}

	Affine& operator=(const Eigen::Matrix<T,R+1,C+1>& mat)
	{
		m_affineMatrix = mat;
		return *this;
	}

	void SetLinear(const Eigen::Matrix<T, R, C>& linear) { m_affineMatrix.template topLeftCorner<R, C>() = linear; }
	Eigen::Matrix<T, R, C> Linear() const { return m_affineMatrix.template topLeftCorner<R, C>(); }

	void SetTranslation(const Eigen::Vector<T, R>& translation) { m_affineMatrix.template topRightCorner<R, 1>() = translation; }
	Eigen::Vector<T, R> Translation() const { return m_affineMatrix.template topRightCorner<R, 1>(); }

	const Eigen::Matrix<T, R+1, C+1>& Matrix() const { return m_affineMatrix; }

	void SetMatrix(const Eigen::Matrix<T, R+1, C+1>& mat)
	{
		m_affineMatrix = mat;
	}

	static Affine FromTranslation(const Eigen::Vector<T, R>& translation)
	{
		Affine aff;
		aff.SetTranslation(translation);
		return aff;
	}

	Eigen::Matrix<T, R, -1> Transform(const Eigen::Matrix<T, C, -1>& vertices) const
	{
		const Eigen::Matrix<T, R, C> r = Linear();
		const Eigen::Vector<T, R> t = Translation();
		Eigen::Matrix<T, R, -1> output(R, vertices.cols());
		for (int i = 0; i < vertices.cols(); i++) {
			output.col(i) = r * vertices.col(i) + t;
		}
		return output;
	}

	Affine operator*(const Affine& aff) const
	{
		return Affine(Matrix() * aff.Matrix());
	}

	void SetIdentity()
	{
		m_affineMatrix.setIdentity();
	}

	Affine Inverse() const
	{
		const Eigen::Matrix<T, R, C> rInv = Linear().inverse();
		Affine affInverse;
		affInverse.SetLinear(rInv);
		affInverse.SetTranslation(rInv * (-Translation()));
		return affInverse;
	}

	//! @returns True if the linear component of the affine transformation contains scale
	bool HasScaling(const T eps = T(1e-4)) const
	{
		return (fabs(Linear().norm() - std::sqrt(T(3))) > eps);
	}

private:
	Eigen::Matrix<T, R+1, C+1> m_affineMatrix;
};


} // namespace nls
} //namespace epic
