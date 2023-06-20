// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/Pimpl.h>
#include <nls/math/Math.h>
#include <nls/geometry/BarycentricCoordinates.h>
#include <nls/geometry/Mesh.h>

namespace epic {
namespace nls {


template <class T>
class MeshCorrespondenceSearch
{
public:
	struct Result {
		Eigen::VectorXi srcIndices;
		Eigen::Matrix<T, 3, -1> targetVertices;
		Eigen::Matrix<T, 3, -1> targetNormals;
		Eigen::Vector<T, -1> weights;
	};

public:
  	MeshCorrespondenceSearch();
	~MeshCorrespondenceSearch();
	MeshCorrespondenceSearch(MeshCorrespondenceSearch&& o);
	MeshCorrespondenceSearch(const MeshCorrespondenceSearch& o) = delete;
	MeshCorrespondenceSearch& operator=(MeshCorrespondenceSearch&& o);
	MeshCorrespondenceSearch& operator=(const MeshCorrespondenceSearch& o) = delete;

	void Init(const Mesh<T>& targetMesh);

	//! @returns the target vertex weights, by default border vertices are zero, all others are one.
	const Eigen::Vector<T, -1>& TargetWeights() const;

	/**
	 * Set custom target vertex weights.
	 * @pre @p targetWeights need to have the size of the number of vertices of the target mesh
	 */
	void SetTargetWeights(const Eigen::Vector<T, -1>& targetWeights);

	void Search(const Mesh<T>& srcMesh, Result& result, const Eigen::VectorX<T>* weights = nullptr, T normalIncompatibilityThreshold = T(0.5)) const;
	void Search(const Eigen::Matrix<T, 3, -1>& srcVertices, const Eigen::Matrix<T, 3, -1>& srcNormals, Result& result, const Eigen::VectorX<T>* weights = nullptr, T normalIncompatibilityThreshold = T(0.5)) const;

	/**
	 * Search for the closest point on the target mesh and return them as barycentric coordinates.
	 */
	BarycentricCoordinates<T, 3> Search(const Eigen::Vector3<T>& pt) const;

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};


} // namespace nls
} //namespace epic
