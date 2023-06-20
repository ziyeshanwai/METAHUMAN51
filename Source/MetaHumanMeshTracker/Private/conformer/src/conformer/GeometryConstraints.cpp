// Copyright Epic Games, Inc. All Rights Reserved.

#include <conformer/GeometryConstraints.h>

#include <carbon/geometry/AABBTree.h>
#include <carbon/geometry/KdTree.h>
#include <carbon/utils/Profiler.h>
#include <carbon/utils/TaskThreadPool.h>
#include <nls/Cost.h>
#include <nls/functions/BarycentricCoordinatesFunction.h>
#include <nls/functions/PointPointConstraintFunction.h>
#include <nls/functions/PointSurfaceConstraintFunction.h>
#include <nls/geometry/MeshCorrespondenceSearch.h>
#include <nls/utils/ConfigurationParameter.h>

#include <algorithm>

namespace epic::nls {

// helper function to compare two matrices (Eigen operator== seems to crash if the matrices have not been initialized)
template <class T, int R1, int C1, int R2, int C2>
bool IsEqual(const Eigen::Matrix<T, R1, C1>& m1, const Eigen::Matrix<T, R2, C2>& m2) {
    if (m1.rows() != m2.rows()) return false;
    if (m1.cols() != m2.cols()) return false;
    return std::equal(m1.data(), m1.data() + m1.size(), m2.data());
}

template <class T>
struct GeometryConstraints<T>::Private
{
    //! The target mesh
    std::shared_ptr<const Mesh<T>> targetMesh;

    //! Target depthmap
    std::vector<std::shared_ptr<const DepthmapData<T>>> depthData;

    //! Weighting mask defining the weight per search point
    VertexWeights<T> sourceWeights;
    std::vector<std::pair<int, T>> nonzeroSourceWeights;

    Eigen::Vector<T, -1> targetWeights;

    std::shared_ptr<epic::carbon::AABBTree<T>> aabbTree;
    std::shared_ptr<epic::carbon::KdTree<T>> kdTree;

    //! The correspondences from the last search
    typename MeshCorrespondenceSearch<T>::Result source2TargetCorrespondences;

    struct Target2SourceCorrespondences {
		std::vector<BarycentricCoordinates<T>> srcBcs;
		Eigen::Matrix<T, 3, -1> targetVertices;
		Eigen::Matrix<T, 3, -1> targetNormals;
		Eigen::Vector<T, -1> weights;
	} target2SourceCorrespondences;

    Configuration config = {std::string("ICP Constraints Configuration"), {
        //!< how much weight to use on geometry constraint
        { "geometryWeight", ConfigurationParameter(T(1), T(0), T(1)) },
        //!< adapt between point2surface constraint (point2point = 0) to point2point constraint (point2point = 1)
        { "point2point", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< normal incompatibility threhold. remove correspondences where the dot product of source and target normal is smaller than normalIncompatibilityThreshold
        { "normalIncompatibilityThreshold", ConfigurationParameter(T(0.5), T(0), T(1)) },
        //!< flag whether to use the distance threshold
        { "useDistanceThreshold", ConfigurationParameter(true) },
        //!< minimum distance threshold for robust weighting of correspondences.
        { "minimumDistanceThreshold", ConfigurationParameter(T(0.5), T(0), T(10)) }
    }};

    std::shared_ptr<epic::carbon::TaskThreadPool> threadPool = epic::carbon::TaskThreadPool::GlobalInstance(/*createIfNotAvailable=*/true);
};


template <class T>
GeometryConstraints<T>::GeometryConstraints() : m(std::make_unique<Private>())
{
}

template <class T> GeometryConstraints<T>::~GeometryConstraints() = default;
template <class T> GeometryConstraints<T>::GeometryConstraints(GeometryConstraints&& other) = default;
template <class T> GeometryConstraints<T>& GeometryConstraints<T>::operator=(GeometryConstraints&& other) = default;

template <class T>
const Configuration& GeometryConstraints<T>::GetConfiguration() const
{
    return m->config;
}

template <class T>
void GeometryConstraints<T>::SetConfiguration(const Configuration& config, bool ignoreUnknownKeys)
{
    m->config.Set(config, ignoreUnknownKeys);
}

template <class T>
void GeometryConstraints<T>::SetSourceWeights(const VertexWeights<T>& vertexWeights)
{
    if (!IsEqual(m->sourceWeights.Weights(), vertexWeights.Weights())) {
        m->sourceWeights = vertexWeights;
        m->nonzeroSourceWeights = vertexWeights.NonzeroVerticesAndWeights();
        ClearPreviousCorrespondences();
    }
}

template <class T>
void GeometryConstraints<T>::SetTargetMesh(const std::shared_ptr<const Mesh<T>>& targetMesh)
{
    if (targetMesh->NumQuads() > 0) {
        CARBON_CRITICAL("target mesh needs to be triangulated");
    }
    if (!targetMesh->HasVertexNormals()) {
        CARBON_CRITICAL("target mesh needs to contain vertex normals");
    }
    if (m->targetMesh != targetMesh) {
        m->targetMesh = targetMesh;
        ClearPreviousCorrespondences();
        m->aabbTree = std::make_shared<epic::carbon::AABBTree<T>>(m->targetMesh->Vertices().transpose(), m->targetMesh->Triangles().transpose());
        m->kdTree = std::make_shared<epic::carbon::KdTree<T>>(m->targetMesh->Vertices().transpose());

        // verify the target vertices are valid
        const T* vertexPtr = m->targetMesh->Vertices().data();
        for (int i = 0; i < m->targetMesh->NumVertices() * 3; ++i) {
            if (!std::isfinite(vertexPtr[i])) {
                CARBON_CRITICAL("mesh contains vertices that are not finite numbers");
            }
        }
        const T* normalPtr = m->targetMesh->VertexNormals().data();
        for (int i = 0; i < m->targetMesh->NumVertices() * 3; ++i) {
            if (!std::isfinite(normalPtr[i])) {
                CARBON_CRITICAL("mesh contains normals that are not finite numbers");
            }
            if (std::abs(normalPtr[i]) > T(1.1)) {
                CARBON_CRITICAL("mesh contains normals that are not finite numbers");
            }
        }
    }
}

template <class T>
void GeometryConstraints<T>::AddTargetDepthAndNormals(const std::shared_ptr<const DepthmapData<T>>& depthData)
{
    m->depthData.push_back(depthData);
}

template <class T>
void GeometryConstraints<T>::SetTargetWeights(const Eigen::Vector<T, -1>& targetWeights)
{
    m->targetWeights = targetWeights;
}

template <class T>
void GeometryConstraints<T>::ClearPreviousCorrespondences()
{
    m->source2TargetCorrespondences = typename MeshCorrespondenceSearch<T>::Result();
    m->target2SourceCorrespondences = typename GeometryConstraints<T>::Private::Target2SourceCorrespondences();
}

template <class T>
bool GeometryConstraints<T>::HasCorrespondences() const
{
    return (m->source2TargetCorrespondences.srcIndices.size() > 0) || (m->target2SourceCorrespondences.srcBcs.size() > 0);
}

template <class T>
Cost<T> GeometryConstraints<T>::EvaluateICP(const DiffDataMatrix<T, 3, -1>& vertices)
{
    PROFILING_FUNCTION(PROFILING_COLOR_PINK);

    const T geometryWeight = m->config["geometryWeight"].template Value<T>();
    const T point2pointWeight = m->config["point2point"].template Value<T>();

    Cost<T> cost;

    if (geometryWeight > 0 && m->source2TargetCorrespondences.srcIndices.size() > 0) {
        const T point2surfaceWeight = std::max(T(0), T(1) - point2pointWeight);

        if (point2pointWeight > T(0)) {
            // point2point only (no tangential motion)
            const DiffData<T> residual = PointPointConstraintFunction<T, 3>::Evaluate(vertices, m->source2TargetCorrespondences.srcIndices, m->source2TargetCorrespondences.targetVertices, m->source2TargetCorrespondences.weights, geometryWeight * point2pointWeight);
            cost.Add(residual, T(1));
        }
        if (point2surfaceWeight > T(0)) {
            // point2surface (tangential motion)
            const DiffData<T> residual = PointSurfaceConstraintFunction<T, 3>::Evaluate(vertices, m->source2TargetCorrespondences.srcIndices, m->source2TargetCorrespondences.targetVertices, m->source2TargetCorrespondences.targetNormals, m->source2TargetCorrespondences.weights, geometryWeight * point2surfaceWeight);
            cost.Add(residual, T(1));
        }
    }

    if (geometryWeight > 0 && m->target2SourceCorrespondences.srcBcs.size() > 0) {
        const T point2surfaceWeight = std::max(T(0), T(1) - point2pointWeight);

        const DiffDataMatrix<T, 3, -1> srcVertices = BarycentricCoordinatesFunction<T, 3>::Evaluate(vertices, m->target2SourceCorrespondences.srcBcs);
        if (point2pointWeight > T(0)) {
            // point2point only (no tangential motion)

            const DiffData<T> residual = PointPointConstraintFunction<T, 3>::Evaluate(srcVertices, m->target2SourceCorrespondences.targetVertices, m->target2SourceCorrespondences.weights, geometryWeight * point2pointWeight);
            cost.Add(residual, T(1));
        }
        if (point2surfaceWeight > T(0)) {
            // point2surface (tangential motion)
            const DiffData<T> residual = PointSurfaceConstraintFunction<T, 3>::Evaluate(srcVertices, m->target2SourceCorrespondences.targetVertices, m->target2SourceCorrespondences.targetNormals, m->target2SourceCorrespondences.weights, geometryWeight * point2surfaceWeight);
            cost.Add(residual, T(1));
        }
    }

    return cost;
}

template <class T>
void GeometryConstraints<T>::SetupCorrespondences(const Mesh<T>& sourceMesh, bool useTarget2Source)
{
    ClearPreviousCorrespondences();
    if (useTarget2Source) {
        FindTarget2SourceCorrespondences(sourceMesh);
    } else {
        FindCorrespondences(sourceMesh.Vertices(), sourceMesh.VertexNormals(), m->source2TargetCorrespondences);
    }
}

template <class T>
void GeometryConstraints<T>::FindCorrespondences(const Eigen::Matrix<T, 3, -1>& vertices, const Eigen::Matrix<T, 3, -1>& normals, typename MeshCorrespondenceSearch<T>::Result& correspondences) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_PINK);

    if (m->sourceWeights.NumVertices() > 0 && m->sourceWeights.NumVertices() != int(vertices.cols())) {
        CARBON_CRITICAL("correspondence search weights do not have the right size");
    }

    correspondences = typename MeshCorrespondenceSearch<T>::Result();

    const T normalIncompatibilityThreshold = m->config["normalIncompatibilityThreshold"].template Value<T>();
    auto makeSquared = [](T val) { return val * val; };
    const T squaredMinimumDistanceThreshold = makeSquared(m->config["minimumDistanceThreshold"].template Value<T>());
    const T minimumDistanceThreshold = m->config["minimumDistanceThreshold"].template Value<T>();
    const bool useDistanceTreshold = m->config["useDistanceThreshold"].template Value<bool>();

    if (m->targetMesh) {
        // search mesh correspondences if available
        const bool useSourceSearchWeights = (m->sourceWeights.NumVertices() > 0);
        // multi-threaded queries to the kd tree
        const int numQueries = int(vertices.cols());
        // Eigen::VectorXi indices(numQueries);
        std::vector<BarycentricCoordinates<T>> bcs(numQueries);
        m->threadPool->AddTaskRangeAndWait(numQueries, [&](int start, int end) {
            //m->kdTree->Search(vertices.data() + 3 * start, end - start, indices.data() + start);
            for (int k = start; k < end; ++k) {
                auto [tID, bc, dist] = m->aabbTree->getClosestPoint(vertices.col(k).transpose(), T(1e9));
                bcs[k] = BarycentricCoordinates<T>(m->targetMesh->Triangles().col(tID), bc);
            }
            });

        const T normalIncompatibilityMultiplier = T(1.0) / std::max<T>(T(1e-6), (T(1) - normalIncompatibilityThreshold));

        correspondences.srcIndices.resize(numQueries);
        correspondences.targetVertices.resize(3, numQueries);
        correspondences.targetNormals.resize(3, numQueries);
        correspondences.weights.resize(numQueries);
        T sum = 0;
        for (int i = 0; i < numQueries; i++) {
            // we do not normalize the normal - if the normal of the vertex is 0 then it will be a bad correspondence
            // anyway and it is ok if it is being ignored
            const Eigen::Vector3<T>& targetNormal = bcs[i].template Evaluate<3>(m->targetMesh->VertexNormals());
            const Eigen::Vector3<T>& targetVertex = bcs[i].template Evaluate<3>(m->targetMesh->Vertices());
            const T normalCompatibilityWeight = std::max<T>(0, (normals.col(i).dot(targetNormal) - normalIncompatibilityThreshold) * normalIncompatibilityMultiplier);
            T weight = normalCompatibilityWeight * normalCompatibilityWeight;
            if (m->targetWeights.size() > 0) weight *= bcs[i].template Evaluate<1>(m->targetWeights.transpose())[0];
            sum += targetNormal.norm();
            correspondences.srcIndices[i] = i;
            correspondences.targetVertices.col(i) = targetVertex;
            correspondences.targetNormals.col(i) = targetNormal;
            correspondences.weights[i] = weight;
            if (useSourceSearchWeights) {
                correspondences.weights[i] *= m->sourceWeights.Weights()[i];
            }
        }

        if (useDistanceTreshold) {
            // robust reweighting based on distances
            std::vector<T> squaredDistances;
            squaredDistances.reserve(correspondences.srcIndices.size());
            for (int i = 0; i < int(correspondences.srcIndices.size()); ++i) {
                if (correspondences.weights[i] > 0) {
                    squaredDistances.push_back((vertices.col(i) - correspondences.targetVertices.col(i)).squaredNorm());
                }
            }
            if (squaredDistances.size() > 0) {
                std::nth_element(squaredDistances.begin(), squaredDistances.begin() + squaredDistances.size() / 2, squaredDistances.end());
                const T squaredDistanceTreshold = std::max<T>(squaredMinimumDistanceThreshold, T(2) * squaredDistances[squaredDistances.size() / 2]);
                for (int i = 0; i < int(correspondences.srcIndices.size()); ++i) {
                    const T squaredDistance = (vertices.col(i) - correspondences.targetVertices.col(i)).squaredNorm();
                    const T distanceWeight = std::max<T>(T(0), (squaredDistanceTreshold - squaredDistance) / squaredDistanceTreshold);
                    correspondences.weights[i] *= distanceWeight;
                }
            }
        }
    }

    // find depthmap correspondences if available
    if (m->depthData.size() > 0) {
        struct Corr {
            int srcIndex;
            Eigen::Vector3<T> targetVertex;
            Eigen::Vector3<T> targetNormal;
            T targetWeight;
            T distance;
        };
        std::vector<Corr> depthmapCorrespondences;
        depthmapCorrespondences.reserve(m->nonzeroSourceWeights.size() * m->depthData.size());
        std::vector<T> distances;
        distances.reserve(m->nonzeroSourceWeights.size() * m->depthData.size());

        for (int i = 0; i < int(m->depthData.size()); ++i) {
            const Camera<T>& camera = m->depthData[i]->camera;
            for (auto&& [srcIndex, weight] : m->nonzeroSourceWeights) {
                const Eigen::Vector3<T> vertex = vertices.col(srcIndex);
                const Eigen::Vector3<T> normal = normals.col(srcIndex);
                const Eigen::Vector2<T> pix = camera.Project(vertex, /*withExtrinsics=*/true);
                // nearest neighbor lookup
                const int x = int(pix[0]);
                const int y = int(pix[1]);
                Eigen::Vector3<T> targetVertex = vertex;
                Eigen::Vector3<T> targetNormal = normal;
                T targetWeight = 0;
                T distance = std::numeric_limits<T>::max();
                if (x >= 0 && x < camera.Width() && y >= 0 && y < camera.Height()) {
                    const int depthAndNormalIndex = y * camera.Width() + x;
                    const Eigen::Vector4<T> depthAndNormal = m->depthData[i]->depthAndNormals.col(depthAndNormalIndex);
                    if (depthAndNormal[0] > 0) {
                        targetVertex = camera.Unproject(Eigen::Vector2<T>(x, y) + Eigen::Vector2<T>(T(0.5), T(0.5)), depthAndNormal[0], /*withExtrinsics=*/true);
                        targetNormal = camera.Extrinsics().Linear().transpose() * depthAndNormal.tail(3);
                        const bool normalCompatible = (normal.dot(targetNormal) > normalIncompatibilityThreshold);
                        if (normalCompatible) {
                            targetWeight = weight;
                            distance = (targetVertex - vertex).squaredNorm();
                            distances.push_back(distance);
                        }
                    }
                }
                depthmapCorrespondences.push_back(Corr{ srcIndex, targetVertex, targetNormal, targetWeight, distance });
            }
        }

        T distanceThreshold = 0;
        if (useDistanceTreshold && distances.size() > 0) {
            std::nth_element(distances.begin(), distances.begin() + distances.size() / 2, distances.end());
            distanceThreshold = std::max<T>(minimumDistanceThreshold, T(2) * distances[distances.size() / 2]);
        }

        // combine the results
        const int numPrevCorrespondences = int(correspondences.srcIndices.size());
        const int numNewCorrespondences = int(depthmapCorrespondences.size());
        typename MeshCorrespondenceSearch<T>::Result extendedResult;
        extendedResult.srcIndices.resize(numPrevCorrespondences + numNewCorrespondences);
        extendedResult.targetVertices.resize(3, numPrevCorrespondences + numNewCorrespondences);
        extendedResult.targetNormals.resize(3, numPrevCorrespondences + numNewCorrespondences);
        extendedResult.weights.resize(numPrevCorrespondences + numNewCorrespondences);
        if (numPrevCorrespondences > 0) {
            extendedResult.srcIndices.head(numPrevCorrespondences) = correspondences.srcIndices;
            extendedResult.targetVertices.leftCols(numPrevCorrespondences) = correspondences.targetVertices;
            extendedResult.targetNormals.leftCols(numPrevCorrespondences) = correspondences.targetNormals;
            extendedResult.weights.head(numPrevCorrespondences) = correspondences.weights;
        }

        for (int k = 0; k < int(depthmapCorrespondences.size()); ++k) {
            extendedResult.srcIndices[numPrevCorrespondences + k] = depthmapCorrespondences[k].srcIndex;
            extendedResult.targetVertices.col(numPrevCorrespondences + k) = depthmapCorrespondences[k].targetVertex;
            extendedResult.targetNormals.col(numPrevCorrespondences + k) = depthmapCorrespondences[k].targetNormal;
            const T distanceWeight = useDistanceTreshold ? std::max<T>(T(0), (distanceThreshold - depthmapCorrespondences[k].distance) / distanceThreshold) : T(1);
            const T weight = distanceWeight * distanceWeight * depthmapCorrespondences[k].targetWeight;
            extendedResult.weights[numPrevCorrespondences + k] = weight;
        }

        correspondences = extendedResult;
    }

}

template <class T>
void GeometryConstraints<T>::FindTarget2SourceCorrespondences(const Mesh<T>& sourceMesh)
{
    PROFILING_FUNCTION(PROFILING_COLOR_PINK);

    if (!m->targetMesh) return;

    const T normalIncompatibilityThreshold = m->config["normalIncompatibilityThreshold"].template Value<T>();
    auto makeSquared = [](T val) { return val * val; };
    const T squaredMinimumDistanceThreshold = makeSquared(m->config["minimumDistanceThreshold"].template Value<T>());
    const bool useDistanceTreshold = m->config["useDistanceThreshold"].template Value<bool>();

    epic::carbon::AABBTree<T> srcAabbTree(sourceMesh.Vertices().transpose(), sourceMesh.Triangles().transpose());

    // multi-threaded queries to the kd tree
    const int numQueries = m->targetMesh->NumVertices();
    std::vector<BarycentricCoordinates<T>> bcs(numQueries);
    m->threadPool->AddTaskRangeAndWait(numQueries, [&](int start, int end){
        for (int k = start; k < end; ++k) {
            auto [tID, bc, dist] = srcAabbTree.getClosestPoint(m->targetMesh->Vertices().col(k).transpose(), T(1e9));
            bcs[k] = BarycentricCoordinates<T>(sourceMesh.Triangles().col(tID), bc);
        }
    });

    const T normalIncompatibilityMultiplier = T(1.0) / std::max<T>(T(1e-6), (T(1) - normalIncompatibilityThreshold));

    m->target2SourceCorrespondences.srcBcs.resize(numQueries);
    m->target2SourceCorrespondences.targetVertices.resize(3, numQueries);
    m->target2SourceCorrespondences.targetNormals.resize(3, numQueries);
    m->target2SourceCorrespondences.weights.resize(numQueries);
    for (int i = 0; i < numQueries; i++) {
        const Eigen::Vector3<T>& sourceNormal = bcs[i].template Evaluate<3>(sourceMesh.VertexNormals()).normalized();
        // const Eigen::Vector3<T>& sourceVertex = bcs[i].template Evaluate<3>(sourceMesh.Vertices());
        const Eigen::Vector3<T>& targetNormal = m->targetMesh->VertexNormals().col(i);
        const Eigen::Vector3<T>& targetVertex = m->targetMesh->Vertices().col(i);
        const T normalCompatibilityWeight = std::max<T>(0, (targetNormal.dot(sourceNormal) - normalIncompatibilityThreshold) * normalIncompatibilityMultiplier);
        T weight = normalCompatibilityWeight * normalCompatibilityWeight;
        if (m->targetWeights.size() > 0) weight *= m->targetWeights[i];
        if (m->sourceWeights.Weights().size() > 0) weight *= bcs[i].template Evaluate<1>(m->sourceWeights.Weights().transpose())[0];

        m->target2SourceCorrespondences.srcBcs[i] = bcs[i];
        m->target2SourceCorrespondences.targetVertices.col(i) = targetVertex;
        m->target2SourceCorrespondences.targetNormals.col(i) = targetNormal;
        m->target2SourceCorrespondences.weights[i] = weight;
    }

    if (useDistanceTreshold) {
        // robust reweighting based on distances
        std::vector<T> squaredDistances;
        squaredDistances.reserve(m->target2SourceCorrespondences.srcBcs.size());
        for (int i = 0; i < int(m->target2SourceCorrespondences.srcBcs.size()); ++i) {
            if (m->target2SourceCorrespondences.weights[i] > 0) {
                const Eigen::Vector3<T>& sourceVertex = m->target2SourceCorrespondences.srcBcs[i].template Evaluate<3>(sourceMesh.Vertices());
                squaredDistances.push_back((sourceVertex - m->target2SourceCorrespondences.targetVertices.col(i)).squaredNorm());
            }
        }
        std::nth_element(squaredDistances.begin(), squaredDistances.begin() + squaredDistances.size() / 2, squaredDistances.end());
        const T squaredMinimiumDistanceTreshold = std::max<T>(squaredMinimumDistanceThreshold, T(2) * squaredDistances[squaredDistances.size() / 2]);
        for (int i = 0; i < int(m->target2SourceCorrespondences.srcBcs.size()); ++i) {
            const Eigen::Vector3<T>& sourceVertex = m->target2SourceCorrespondences.srcBcs[i].template Evaluate<3>(sourceMesh.Vertices());
            const T squaredDistance = (sourceVertex - m->target2SourceCorrespondences.targetVertices.col(i)).squaredNorm();
            const T distanceWeight = std::max<T>(T(0), (squaredMinimiumDistanceTreshold - squaredDistance) / squaredMinimiumDistanceTreshold);
            m->target2SourceCorrespondences.weights[i] *= distanceWeight;
        }
    }
}

template <class T>
void GeometryConstraints<T>::SetupConstraints(const Eigen::Transform<T, 3, Eigen::Affine>& rigidTransform,
                                              const Eigen::Matrix<T, 3, -1>& vertices,
                                              const Eigen::Matrix<T, 3, -1>& normals,
                                              VertexConstraints<T, 1, 1>& point2SurfaceVertexConstraints,
                                              VertexConstraints<T, 3, 1>& point2PointVertexConstraints)
{
    PROFILING_FUNCTION(PROFILING_COLOR_PINK);

    const T geometryWeight = m->config["geometryWeight"];
    const T point2pointWeight = m->config["point2point"];
    const T point2surfaceWeight = std::max(T(0), T(1) - point2pointWeight);

    if (geometryWeight > 0) {
        const Eigen::Matrix<T, 3, -1> transformedVertices = rigidTransform * vertices;
        const Eigen::Matrix<T, 3, -1> transformedNormals = rigidTransform.linear() * normals;
        ClearPreviousCorrespondences();
        FindCorrespondences(transformedVertices, transformedNormals, m->source2TargetCorrespondences);
        const typename MeshCorrespondenceSearch<T>::Result& correspondences = m->source2TargetCorrespondences;

        if (point2pointWeight > T(0)) {
            const T combinedWeight = std::sqrt(geometryWeight * point2pointWeight);
            point2PointVertexConstraints.ResizeToFitAdditionalConstraints(correspondences.srcIndices.size());
            for (int i = 0; i < correspondences.srcIndices.size(); ++i) {
                const int vID = correspondences.srcIndices[i];
                const Eigen::Vector3<T>& targetVertex = correspondences.targetVertices.col(i);
                const T weight = correspondences.weights[i] * combinedWeight;
                if (weight > 0) {
                    const Eigen::Matrix<T, 3, 3> drdV = weight * rigidTransform.linear();
                    const Eigen::Vector3<T> residual = weight * (transformedVertices.col(vID) - targetVertex);
                    point2PointVertexConstraints.AddConstraint(Eigen::Vector<int, 1>(vID), residual, drdV);
                }
            }
        }
        if (point2surfaceWeight > T(0)) {
            const T combinedWeight = std::sqrt(geometryWeight * point2surfaceWeight);
            point2SurfaceVertexConstraints.ResizeToFitAdditionalConstraints(correspondences.srcIndices.size());
            const Eigen::Transform<T, 3, Eigen::Affine> rigidTransformInv = rigidTransform.inverse();

            for (int i = 0; i < correspondences.srcIndices.size(); ++i) {
                const int vID = correspondences.srcIndices[i];
                const Eigen::Vector3<T>& targetVertex = correspondences.targetVertices.col(i);
                const Eigen::Vector3<T>& targetNormal = correspondences.targetNormals.col(i);
                const T weight = correspondences.weights[i] * combinedWeight;
                if (weight > 0) {
                    const Eigen::Matrix<T, 1, 3> drdV = weight * rigidTransformInv.linear() * targetNormal;
                    const T residual = weight * targetNormal.dot(transformedVertices.col(vID) - targetVertex);
                    point2SurfaceVertexConstraints.AddConstraint(vID, residual, drdV);
                }
            }
        }
    }
}

// explicitly instantiate the GeometryConstraints classes
template class GeometryConstraints<float>;
template class GeometryConstraints<double>;

} // namespace epic::nls
