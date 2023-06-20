// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/DiffDataMatrix.h>
#include <nls/math/Math.h>

#include <vector>

namespace epic {
namespace nls {

//! Estimate the closest rotation @p R based on input transformation @p F. If dRdF is not-zero then it will contain the derivative of R with respect to F.
template <class T>
void FtoR(const Eigen::Matrix<T, 3, 3>& F, Eigen::Matrix<T, 3, 3>& R, Eigen::Matrix<T, 9, 9>* dRdF);

/**
 * Various tetrahedral contraints including strain hyperelasticity, volume preservation, and the deformation gradient.
 */
template <class T>
class TetConstraints
{
public:
	enum ElasticityModel {Linear, Corotated};

  	TetConstraints() = default;
	TetConstraints(const TetConstraints& o) = delete;
	TetConstraints& operator=(const TetConstraints& o) = delete;

	//! Sets the topology of the tetrahedral mesh.
	void SetTopology(const Eigen::Matrix<int, 4, -1>& tets) { m_tets = tets; }

	//! Set the rest pose of the tetrahedral mesh.
	void SetRestPose(const Eigen::Matrix<T, 3, -1>& vertices, bool allowInvertedTets = false);

	//! @returns the number of tetraheda.
	int NumTets() const { return int(m_tets.cols()); }

	const std::vector<Eigen::Matrix<T, 3, 3>>& InvRestFrame() const { return m_invRestFrame; }
	const std::vector<T>& SqrtRestVolume() const { return m_sqrtRestVolume; }

	/**
	 * Tetrahedral strain hyperelasticity.
	 * Evaluates residual function r(x) and its Jacobian, e.g.
	 * - corotated: r(x) = F(x) - R(F(x))
	 */
	DiffData<T> EvaluateStrain(const DiffDataMatrix<T, 3, -1>& vertices, const T strainWeight) const;

	/**
	 * Tetrahedral strain hyperelasticity.
	 * Evaluates residual function r(x) and its Jacobian, e.g.
	 * - linear elasticity: r(x) = F(x) - I
	 * - corotated: r(x) = F(x) - R (where R is fixed to the rotation closest to F).
	 */
	DiffData<T> EvaluateStrainLinearProjective(const DiffDataMatrix<T, 3, -1>& vertices, const T strainWeight, ElasticityModel elModel) const;

	/**
	 * Tetrahedral volume loss.
	 * Evaluates residual function r(x) and its Jacobian, e.g. r(x) = det(F(x)) - 1
	 */
	DiffData<T> EvaluateVolumeLoss(const DiffDataMatrix<T, 3, -1>& vertices, T volumeWeight) const;

	/**
	 * Tetrahedral volume loss.
	 * Evaluates residual function r(x) and its Jacobian, e.g. r(x) = F(x) - F' (where F' is fixed to the volume preserving deformation closest to F)
	 */
	DiffData<T> EvaluateVolumeLossProjective(const DiffDataMatrix<T, 3, -1>& vertices, T volumeWeight) const;

	//! @returns F(x)
	DiffDataMatrix<T, 9, -1> EvaluateDeformationGradient(const DiffDataMatrix<T, 3, -1>& vertices, bool volumeWeighted = true, const std::vector<T>& perTetWeight = std::vector<T>()) const;

private:
	int m_numVertices;

	Eigen::Matrix<int, 4, -1> m_tets;
	std::vector<Eigen::Matrix<T, 3, 3>> m_invRestFrame;
	std::vector<T> m_sqrtRestVolume;
};


} // namespace nls
} //namespace epic
