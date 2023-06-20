// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/CollisionConstraints.h>

#include <carbon/geometry/AABBTree.h>
#include <carbon/utils/Profiler.h>
#include <nls/functions/BarycentricCoordinatesFunction.h>
#include <nls/functions/GatherFunction.h>
#include <nls/functions/SubtractFunction.h>

namespace epic::nls {


template <class T>
struct CollisionConstraints<T>::Private
{
    std::vector<int> srcMask;
    Eigen::Matrix<int, 3, -1> srcTriangles;
    std::vector<int> targetMask;
    Eigen::Matrix<int, 3, -1> targetTriangles;

    CollisionConstraintsData<T> collisionConstraintsData;
};

template <class T>
CollisionConstraints<T>::CollisionConstraints() : m(std::make_unique<Private>())
{

}

template <class T> CollisionConstraints<T>::~CollisionConstraints() = default;
template <class T> CollisionConstraints<T>::CollisionConstraints(CollisionConstraints&&) = default;
template <class T> CollisionConstraints<T>& CollisionConstraints<T>::operator=(CollisionConstraints&&) = default;

template <class T>
Mesh<T> KeepTrianglesInMask(const Mesh<T>& mesh, const std::vector<int>& mask)
{
    Mesh<T> newMesh = mesh;
    newMesh.Triangulate();

    std::vector<bool> used(newMesh.NumVertices(), false);
    for (int vID : mask) {
        used[vID] = true;
    }
    std::vector<Eigen::Vector3i> triangles;
    for (int tID = 0; tID < newMesh.NumTriangles(); ++tID) {
        if (used[newMesh.Triangles()(0, tID)] &&
            used[newMesh.Triangles()(1, tID)] &&
            used[newMesh.Triangles()(2, tID)]) {
            triangles.push_back(newMesh.Triangles().col(tID));
        }
    }

    newMesh.SetTriangles(Eigen::Map<const Eigen::Matrix<int, 3, -1>>((const int*)triangles.data(), 3, triangles.size()));

    return newMesh;
}

template <class T>
void CollisionConstraints<T>::SetSourceTopology(const Mesh<T>& mesh, const std::vector<int>& srcMask)
{
    m->srcMask = srcMask;
    m->srcTriangles = KeepTrianglesInMask(mesh, srcMask).Triangles();
}

template <class T>
void CollisionConstraints<T>::SetTargetTopology(const Mesh<T>& mesh, const std::vector<int>& targetMask)
{
    m->targetMask = targetMask;
    m->targetTriangles = KeepTrianglesInMask(mesh, targetMask).Triangles();
}

template <class T>
void CollisionConstraints<T>::CalculateCollisions(const Mesh<T>& srcMesh, const Mesh<T>& targetMesh)
{
    epic::carbon::AABBTree<T> srcAabbTree(srcMesh.Vertices().transpose(), m->srcTriangles.transpose());
    epic::carbon::AABBTree<T> targetAabbTree(targetMesh.Vertices().transpose(), m->targetTriangles.transpose());

    m->collisionConstraintsData.Clear();

    // using approach from "Ray-traced collision detection for deformable bodies", Hermann et al
    for (int vID : m->srcMask) {
        // search for each vertex along the negative direction where it intersects the target mesh
        const Eigen::Vector3<T> srcNormal = srcMesh.VertexNormals().col(vID);
        auto [tID, bcWeights, dist] = targetAabbTree.intersectRay(srcMesh.Vertices().col(vID).transpose(), -srcNormal.transpose());
        if (tID >= 0) {
            // a collision is only possible if the ray is hitting the inner surface of the target mesh
            BarycentricCoordinates<T> bc(m->targetTriangles.col(tID), bcWeights.transpose());
            const Eigen::Vector3<T> targetNormal = bc.template Evaluate<3>(targetMesh.VertexNormals());
            if (targetNormal.dot(srcNormal) < 0) {
                // check that the collision from target to src is the same
                const Eigen::Vector3<T> targetVertex = bc.template Evaluate<3>(targetMesh.Vertices());
                auto [tID2, bcWeights2, dist2] = srcAabbTree.intersectRay(targetVertex.transpose(), srcNormal.transpose());
                if (fabs(dist - dist2) < 1e-3) {
                    m->collisionConstraintsData.srcCollisionIDs.push_back(vID);
                    m->collisionConstraintsData.targetCollisionBCs.push_back(bc);
                    const Eigen::Vector3<T> collisionNormal = (srcNormal - targetNormal).normalized();
                    m->collisionConstraintsData.targetNormals.push_back(collisionNormal);
                }
            }
        }
    }
}

template <class T>
DiffData<T> CollisionConstraints<T>::EvaluateCollisions(const DiffDataMatrix<T, 3, -1>& srcVertices, const DiffDataMatrix<T, 3, -1>& targetVertices) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_GREEN);

    DiffDataMatrix<T, 3, -1> srcCollisionVertices = GatherFunction<T>::template GatherColumns<3, -1, -1>(srcVertices, m->collisionConstraintsData.srcCollisionIDs);
    DiffDataMatrix<T, 3, -1> targetCollisionPoints = BarycentricCoordinatesFunction<T,3>::Evaluate(targetVertices, m->collisionConstraintsData.targetCollisionBCs);
    DiffDataMatrix<T, 3, -1> offsetMatrix = srcCollisionVertices - targetCollisionPoints;

    const int numConstraints = srcCollisionVertices.Cols();
    int validConstraints = 0;

    VectorPtr<T> result = std::make_shared<Vector<T> >(numConstraints);
    for (int i = 0; i < numConstraints; ++i) {
        validConstraints++;
        const T cost = m->collisionConstraintsData.targetNormals[i].dot(offsetMatrix.Matrix().col(i));
        if (cost > 0) {
            (*result)[i] = cost;
            validConstraints++;
        } else {
            (*result)[i] = 0;
        }
    }

    JacobianConstPtr<T> Jacobian;
    if (offsetMatrix.HasJacobian() && validConstraints > 0) {
        PROFILING_BLOCK("jacobian (1)");
        SparseMatrix<T> localJacobian(numConstraints, offsetMatrix.Size());
        localJacobian.reserve(validConstraints * 3);
        for (int i = 0; i < numConstraints; ++i) {
            localJacobian.startVec(i);
            if ((*result)[i] > 0) {
                for (int k = 0; k < 3; ++k) {
                    localJacobian.insertBackByOuterInnerUnordered(i, 3 * i + k) = m->collisionConstraintsData.targetNormals[i][k];
                }
            }
        }
        localJacobian.finalize();
        PROFILING_END_BLOCK;
        PROFILING_BLOCK("jacobian (2)");
        Jacobian = offsetMatrix.Jacobian().Premultiply(localJacobian);
        PROFILING_END_BLOCK;
    }

    return DiffData<T>(result, Jacobian);
}

template <class T>
void CollisionConstraints<T>::GetCollisions(CollisionConstraintsData<T>& collisionConstraintsData) const
{
    collisionConstraintsData = m->collisionConstraintsData;
}

// explicitly instantiate the CollisionConstraints classes
template class CollisionConstraints<float>;
template class CollisionConstraints<double>;

} // namespace epic::nls
