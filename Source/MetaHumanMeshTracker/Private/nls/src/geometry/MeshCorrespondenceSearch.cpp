// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/geometry/MeshCorrespondenceSearch.h>

#include <carbon/geometry/KdTree.h>
#include <carbon/utils/TaskThreadPool.h>
#include <nls/geometry/Triangle.h>
#include <carbon/utils/Profiler.h>
#include <carbon/utils/Timer.h>

#include <memory>
#include <mutex>
#include <thread>

namespace epic::nls {

template <class T>
struct MeshCorrespondenceSearch<T>::Private
{
    std::unique_ptr<epic::carbon::KdTree<T>> kdTree;

    Mesh<T> mesh;
    Eigen::Vector<T, -1> targetWeights;

    std::shared_ptr<epic::carbon::TaskThreadPool> taskThreadPool = epic::carbon::TaskThreadPool::GlobalInstance(/*createIfNotAvailable=*/true);

    mutable std::mutex mutex;
    mutable std::vector<std::vector<int>> perVertexTriangles;
};

template <class T>
MeshCorrespondenceSearch<T>::MeshCorrespondenceSearch()
    : m(std::make_unique<Private>())
{
}

template <class T> MeshCorrespondenceSearch<T>::~MeshCorrespondenceSearch() = default;
template <class T> MeshCorrespondenceSearch<T>::MeshCorrespondenceSearch(MeshCorrespondenceSearch&& o) = default;
template <class T> MeshCorrespondenceSearch<T>& MeshCorrespondenceSearch<T>::operator=(MeshCorrespondenceSearch&& o) = default;

template <class T>
void MeshCorrespondenceSearch<T>::Init(const Mesh<T>& mesh)
{
    PROFILING_FUNCTION(PROFILING_COLOR_CYAN);

    std::unique_lock<std::mutex> lock(m->mutex);

    m->mesh = mesh;
    m->mesh.CalculateVertexNormals();

    m->targetWeights.resize(m->mesh.NumVertices());
    for (int i = 0; i < m->mesh.NumVertices(); i++) {
        const Eigen::Vector3<T>& normal = m->mesh.VertexNormals().col(i);
        m->targetWeights[i] = (normal.squaredNorm() > T(0.05)) ? T(1) : T(0);
    }

    epic::carbon::TaskFutures taskFutures;
    taskFutures.Add(m->taskThreadPool->AddTask([&](){
        // set border vertices to zero
        const std::vector<int> borderVertices = m->mesh.CalculateBorderVertices();
        for (int vID : borderVertices) {
            m->targetWeights[vID] = T(0);
        }
    }));

    taskFutures.Add(m->taskThreadPool->AddTask([&](){
        // calculate kd tree
        m->kdTree = std::make_unique<epic::carbon::KdTree<T>>(m->mesh.Vertices().transpose());
    }));

    taskFutures.Wait();

    m->perVertexTriangles.clear();
}

template <class T>
const Eigen::Vector<T, -1>& MeshCorrespondenceSearch<T>::TargetWeights() const
{
    std::unique_lock<std::mutex> lock(m->mutex);
    return m->targetWeights;
}

template <class T>
void MeshCorrespondenceSearch<T>::SetTargetWeights(const Eigen::Vector<T, -1>& targetWeights)
{
    std::unique_lock<std::mutex> lock(m->mutex);
    if (targetWeights.size() != m->targetWeights.size()) {
        CARBON_CRITICAL("size of weights vector does not match the number of target vertices");
    }
    m->targetWeights = targetWeights;
}

template <class T>
void MeshCorrespondenceSearch<T>::Search(const Mesh<T>& srcMesh,
                                         Result& result,
                                         const Eigen::VectorX<T>* weights,
                                         T normalIncompatibilityThreshold) const
{
    if (!srcMesh.HasVertexNormals()) {
        CARBON_CRITICAL("mesh correspondence search: src mesh is missing vertex normals");
    }

    Search(srcMesh.Vertices(), srcMesh.VertexNormals(), result, weights, normalIncompatibilityThreshold);
}

template <class T>
void MeshCorrespondenceSearch<T>::Search(const Eigen::Matrix<T, 3, -1>& srcVertices,
                                         const Eigen::Matrix<T, 3, -1>& srcNormals,
                                         Result& result,
                                         const Eigen::VectorX<T>* weights,
                                         T normalIncompatibilityThreshold) const
{
    std::unique_lock<std::mutex> lock(m->mutex);

    if (srcVertices.cols() != srcNormals.cols()) {
        CARBON_CRITICAL("mesh correspondence search: sizes of vertices and normals do not match");
    }
    if (!m->kdTree) {
        CARBON_CRITICAL("search structure is not set up");
    }
    PROFILING_FUNCTION(PROFILING_COLOR_CYAN);

    // single-threaded queries to the kd tree
    // const Eigen::VectorXi indices = m->kdTree->Search(srcVertices.data(), int(srcVertices.cols()));

    // multi-threaded queries to the kd tree
    const int numQueries = int(srcVertices.cols());
    Eigen::VectorXi indices(numQueries);
    m->taskThreadPool->AddTaskRangeAndWait(numQueries, [&](int start, int end){
        m->kdTree->Search(srcVertices.data() + 3 * start, end - start, indices.data() + start);
    });

    const T normalIncompatibilityMultiplier = T(1.0) / std::max<T>(T(1e-6), (T(1) - normalIncompatibilityThreshold));

    result.srcIndices.resize(indices.size());
    result.targetVertices.resize(3, indices.size());
    result.targetNormals.resize(3, indices.size());
    result.weights.resize(indices.size());
    for (int i = 0; i < int(indices.size()); i++) {
        const Eigen::Vector3<T>& targetNormal = m->mesh.VertexNormals().col(indices[i]);
        const T normalCompatibilityWeight = std::max<T>(0, (srcNormals.col(i).dot(targetNormal) - normalIncompatibilityThreshold) * normalIncompatibilityMultiplier);

        result.srcIndices[i] = i;
        result.targetVertices.col(i) = m->mesh.Vertices().col(indices[i]);
        result.targetNormals.col(i) = m->mesh.VertexNormals().col(indices[i]);
        result.weights[i] = normalCompatibilityWeight * normalCompatibilityWeight * m->targetWeights[indices[i]];
        if (weights) {
            result.weights[i] *= (*weights)[i];
        }
    }
}


template <class T>
BarycentricCoordinates<T, 3> MeshCorrespondenceSearch<T>::Search(const Eigen::Vector3<T>& pt) const
{
    std::unique_lock<std::mutex> lock(m->mutex);

    if (!m->kdTree) {
        CARBON_CRITICAL("search structure is not set up");
    }

    if (m->perVertexTriangles.empty()) {
        m->perVertexTriangles.resize(m->mesh.NumVertices());
        for (int i = 0; i < m->mesh.NumTriangles(); ++i) {
            for (int k = 0; k < 3; ++k) {
                const int vID = m->mesh.Triangles()(k, i);
                m->perVertexTriangles[vID].push_back(i);
            }
        }
    }

    auto [vID, bestSquaredDistance] = m->kdTree->getClosestPoint(pt.transpose(), std::numeric_limits<T>::max());

    // check neighboring triangles
    BarycentricCoordinates<T, 3> bcOut = BarycentricCoordinates<T, 3>::SingleVertex(int(vID));
    for (int tID : m->perVertexTriangles[vID]) {
        const Eigen::Vector3i triangle = m->mesh.Triangles().col(tID);
        const Eigen::Vector3<T> v0 = m->mesh.Vertices().col(triangle[0]);
        const Eigen::Vector3<T> v1 = m->mesh.Vertices().col(triangle[1]);
        const Eigen::Vector3<T> v2 = m->mesh.Vertices().col(triangle[2]);
        const Eigen::Vector3<T> bc = ClosestPtPointTriangle(pt, v0, v1, v2);
        const Eigen::Vector3<T> pt_projected = bc[0] * v0 + bc[1] * v1 + bc[2] * v2;
        const T squaredDistance = (pt - pt_projected).squaredNorm();
        if (squaredDistance < bestSquaredDistance) {
            bcOut = BarycentricCoordinates<T, 3>(triangle, bc);
            bestSquaredDistance = squaredDistance;
        }
    }
    return bcOut;
}

template class MeshCorrespondenceSearch<float>;
template class MeshCorrespondenceSearch<double>;

} // namespace epic::nls
