// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nls/DiffDataMatrix.h>
#include <nls/geometry/BarycentricCoordinates.h>
#include <nls/geometry/Mesh.h>

#include <vector>

namespace epic::nls {

template <class T>
struct CollisionConstraintsData
{
    std::vector<int> srcCollisionIDs;
    std::vector<BarycentricCoordinates<T, 3>> targetCollisionBCs;
    std::vector<Eigen::Vector3<T>> targetNormals;

    void Clear()
    {
        srcCollisionIDs.clear();
        targetCollisionBCs.clear();
        targetNormals.clear();
    }

    int NumCollisions() const { return static_cast<int>(srcCollisionIDs.size()); }
};

/**
 * Class to evaluate collision constraints
 */
template <class T>
class CollisionConstraints
{
public:
    CollisionConstraints();
    ~CollisionConstraints();
    CollisionConstraints(CollisionConstraints&&);
    CollisionConstraints(CollisionConstraints&) = delete;
    CollisionConstraints& operator=(CollisionConstraints&&);
    CollisionConstraints& operator=(const CollisionConstraints&) = delete;

    //! Set the mask for the source mesh
    void SetSourceTopology(const Mesh<T>& mesh, const std::vector<int>& srcMask);

    //! Set the mask for the target mesh
    void SetTargetTopology(const Mesh<T>& mesh, const std::vector<int>& targetMask);

    //! Calculate collisions
    void CalculateCollisions(const Mesh<T>& srcMesh, const Mesh<T>& targetMesh);

	//! Evaluate collision constraints
    DiffData<T> EvaluateCollisions(const DiffDataMatrix<T, 3, -1>& srcVertices, const DiffDataMatrix<T, 3, -1>& targetVertices) const;

    //! Retrieve current collisions
    void GetCollisions(CollisionConstraintsData<T>& collisionConstraintsData) const;

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};

} // namespace epic::nls
