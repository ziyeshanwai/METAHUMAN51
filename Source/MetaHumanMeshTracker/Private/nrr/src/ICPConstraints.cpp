// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/ICPConstraints.h>

#include <carbon/utils/Profiler.h>
#include <nls/functions/PointPointConstraintFunction.h>
#include <nls/functions/PointSurfaceConstraintFunction.h>
#include <nls/geometry/MeshCorrespondenceSearch.h>

#include <algorithm>

namespace epic::nls {

template <class T>
struct ICPConstraints<T>::Private
{
    //! The target mesh
    Mesh<T> targetMesh;

    //! Correspondence search on the target Mesh
    MeshCorrespondenceSearch<T> correspondenceSearch;

    //! Target depthmaps
    std::vector<std::shared_ptr<const DepthmapData<T>>> depthmapData;

    //! Weighting mask defining the weight per search point
    VertexWeights<T> correspondenceSearchWeights;
    std::vector<std::pair<int, T>> nonzeroCorrespondenceSearchWeights;

    //! The correspondences from the last search
    typename MeshCorrespondenceSearch<T>::Result correspondences;

    Configuration config = {ICPConstraints<T>::ConfigName(), {
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
};


template <class T>
ICPConstraints<T>::ICPConstraints() : m(std::make_unique<Private>())
{
}

template <class T> ICPConstraints<T>::~ICPConstraints() = default;
template <class T> ICPConstraints<T>::ICPConstraints(ICPConstraints&&) = default;
template <class T> ICPConstraints<T>& ICPConstraints<T>::operator=(ICPConstraints&&) = default;

template <class T>
const Configuration& ICPConstraints<T>::GetConfiguration() const
{
    return m->config;
}

template <class T>
void ICPConstraints<T>::SetConfiguration(const Configuration& config, bool ignoreUnknownKeys)
{
    m->config.Set(config, ignoreUnknownKeys);
}

template <class T>
void ICPConstraints<T>::SetCorrespondenceSearchVertexWeights(const VertexWeights<T>& vertexWeights)
{
    m->correspondenceSearchWeights = vertexWeights;
    m->nonzeroCorrespondenceSearchWeights = vertexWeights.NonzeroVerticesAndWeights();
    ClearPreviousCorrespondences();
}

template <class T>
void ICPConstraints<T>::SetTargetMesh(const Mesh<T>& targetMesh)
{
    m->targetMesh = targetMesh;
    m->targetMesh.Triangulate();
    m->targetMesh.CalculateVertexNormals();
    m->correspondenceSearch.Init(m->targetMesh);
    ClearPreviousCorrespondences();
}

template <class T>
const Eigen::Vector<T, -1>& ICPConstraints<T>::TargetWeights() const
{
    return m->correspondenceSearch.TargetWeights();
}

template <class T>
void ICPConstraints<T>::SetTargetWeights(const Eigen::Vector<T, -1>& targetWeights)
{
    m->correspondenceSearch.SetTargetWeights(targetWeights);
}

template <class T>
void ICPConstraints<T>::AddTargetDepthAndNormals(const std::shared_ptr<const DepthmapData<T>>& depthmapData)
{
    m->depthmapData.push_back(depthmapData);
}

template <class T>
void ICPConstraints<T>::ClearPreviousCorrespondences()
{
    m->correspondences = typename MeshCorrespondenceSearch<T>::Result();
}

template <class T>
bool ICPConstraints<T>::HasCorrespondences() const
{
    return (m->correspondences.srcIndices.size() > 0);
}

template <class T>
Cost<T> ICPConstraints<T>::EvaluateICP(const DiffDataMatrix<T, 3, -1>& vertices)
{
    PROFILING_FUNCTION(PROFILING_COLOR_PINK);

    const T geometryWeight = m->config["geometryWeight"];
    const T point2pointWeight = m->config["point2point"];

    Cost<T> cost;

    if (geometryWeight > 0 && m->correspondences.srcIndices.size() > 0) {
        const T point2surfaceWeight = std::max(T(0), T(1) - point2pointWeight);

        if (point2pointWeight > T(0)) {
            // point2point only (no tangential motion)
            const DiffData<T> residual = PointPointConstraintFunction<T, 3>::Evaluate(vertices, m->correspondences.srcIndices, m->correspondences.targetVertices, m->correspondences.weights, geometryWeight * point2pointWeight);
            cost.Add(residual, T(1), "icp_p2p");
        }
        if (point2surfaceWeight > T(0)) {
            // point2surface (tangential motion)
            const DiffData<T> residual = PointSurfaceConstraintFunction<T, 3>::Evaluate(vertices, m->correspondences.srcIndices, m->correspondences.targetVertices, m->correspondences.targetNormals, m->correspondences.weights, geometryWeight * point2surfaceWeight);
            cost.Add(residual, T(1), "icp_p2s");
        }
    }

    return cost;
}

template <class T>
Cost<T> ICPConstraints<T>::EvaluateICP(const DiffDataMatrix<T, 3, -1>& vertices, const Eigen::Matrix<T, 3, -1>& normals, bool searchCorrespondences)
{
    PROFILING_FUNCTION(PROFILING_COLOR_PINK);

    const T geometryWeight = m->config["geometryWeight"];
    if (geometryWeight > 0) {
        if (searchCorrespondences || m->correspondences.srcIndices.size() == 0) {
            SetupCorrespondences(vertices.Matrix(), normals);
        }
        return EvaluateICP(vertices);
    } else {
        return Cost<T>();
    }
}

template <class T>
void ICPConstraints<T>::SetupCorrespondences(const Eigen::Matrix<T, 3, -1>& vertices, const Eigen::Matrix<T, 3, -1>& normals)
{
    FindCorrespondences(vertices, normals, m->correspondences);
}

template <class T>
void ICPConstraints<T>::FindCorrespondences(const Eigen::Matrix<T, 3, -1>& vertices, const Eigen::Matrix<T, 3, -1>& normals, typename MeshCorrespondenceSearch<T>::Result& correspondences) const
{
    PROFILING_FUNCTION(PROFILING_COLOR_PINK);

    if (m->correspondenceSearchWeights.NumVertices() > 0 && m->correspondenceSearchWeights.NumVertices() != int(vertices.cols())) {
        throw std::runtime_error("correspondence search weights do not have the right size");
    }

    const T normalIncompatibilityThreshold = m->config["normalIncompatibilityThreshold"];
    const T minimumDistanceThreshold = m->config["minimumDistanceThreshold"];
    const bool useDistanceTreshold = m->config["useDistanceThreshold"];

    correspondences = typename MeshCorrespondenceSearch<T>::Result();

    // search mesh correspondences if available
    if (m->targetMesh.NumVertices() > 0) {
        m->correspondenceSearch.Search(vertices, normals, correspondences, (m->correspondenceSearchWeights.NumVertices() > 0 ? &m->correspondenceSearchWeights.Weights() : nullptr), normalIncompatibilityThreshold);

        if (useDistanceTreshold) {
            // robust reweighting based on distances
            std::vector<T> distances;
            distances.reserve(correspondences.srcIndices.size());
            for (int i = 0; i < int(correspondences.srcIndices.size()); ++i) {
                if (correspondences.weights[i] > 0) {
                    const T distance = (vertices.col(i) - correspondences.targetVertices.col(i)).squaredNorm();
                    distances.push_back(distance);
                }
            }
            std::nth_element(distances.begin(), distances.begin() + distances.size() / 2, distances.end());
            const T distanceTreshold = std::max<T>(minimumDistanceThreshold, T(2) * distances[distances.size() / 2]);
            for (int i = 0; i < int(correspondences.srcIndices.size()); ++i) {
                const T distance = (vertices.col(i) - correspondences.targetVertices.col(i)).squaredNorm();
                const T distanceWeight = std::max<T>(T(0), (distanceTreshold - distance) / distanceTreshold);
                correspondences.weights[i] *= distanceWeight * distanceWeight;
            }
        }
    }

    // find depthmap correspondences if available
    if (m->depthmapData.size() > 0) {
        struct Corr {
            int srcIndex;
            Eigen::Vector3<T> targetVertex;
            Eigen::Vector3<T> targetNormal;
            T targetWeight;
            T distance;
        };
        std::vector<Corr> depthmapCorrespondences;
        depthmapCorrespondences.reserve(m->nonzeroCorrespondenceSearchWeights.size() * m->depthmapData.size());
        std::vector<T> distances;
        distances.reserve(m->nonzeroCorrespondenceSearchWeights.size() * m->depthmapData.size());

        for (int i = 0; i < int(m->depthmapData.size()); ++i) {
            const Camera<T>& camera = m->depthmapData[i]->camera;
            const Eigen::Matrix<T, 4, -1>& depthAndNormals = m->depthmapData[i]->depthAndNormals;
            for (auto && [srcIndex, weight] : m->nonzeroCorrespondenceSearchWeights) {
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
                    const Eigen::Vector4<T> depthAndNormal = depthAndNormals.col(depthAndNormalIndex);
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
                depthmapCorrespondences.push_back(Corr{srcIndex, targetVertex, targetNormal, targetWeight, distance});
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

        for(int k = 0; k < int(depthmapCorrespondences.size()); ++k) {
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
void ICPConstraints<T>::SetupICPConstraints(const Eigen::Transform<T, 3, Eigen::Affine>& rigidTransform,
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
        SetupCorrespondences(transformedVertices, transformedNormals);

        if (point2pointWeight > T(0)) {
            const T combinedWeight = std::sqrt(geometryWeight * point2pointWeight);
            point2PointVertexConstraints.ResizeToFitAdditionalConstraints(m->correspondences.srcIndices.size());
            for (int i = 0; i < m->correspondences.srcIndices.size(); ++i) {
                const int vID = m->correspondences.srcIndices[i];
                const Eigen::Vector3<T>& targetVertex = m->correspondences.targetVertices.col(i);
                const T weight = m->correspondences.weights[i] * combinedWeight;
                if (weight > 0) {
                    const Eigen::Matrix<T, 3, 3> drdV = weight * rigidTransform.linear();
                    const Eigen::Vector3<T> residual = weight * (transformedVertices.col(vID) - targetVertex);
                    point2PointVertexConstraints.AddConstraint(Eigen::Vector<int, 1>(vID), residual, drdV);
                }
            }
        }
        if (point2surfaceWeight > T(0)) {
            const T combinedWeight = std::sqrt(geometryWeight * point2surfaceWeight);
            point2SurfaceVertexConstraints.ResizeToFitAdditionalConstraints(m->correspondences.srcIndices.size());
            const Eigen::Transform<T, 3, Eigen::Affine> rigidTransformInv = rigidTransform.inverse();

            for (int i = 0; i < m->correspondences.srcIndices.size(); ++i) {
                const int vID = m->correspondences.srcIndices[i];
                const Eigen::Vector3<T>& targetVertex = m->correspondences.targetVertices.col(i);
                const Eigen::Vector3<T>& targetNormal = m->correspondences.targetNormals.col(i);
                const T weight = m->correspondences.weights[i] * combinedWeight;
                if (weight > 0) {
                    const Eigen::Matrix<T, 1, 3> drdV = weight * rigidTransformInv.linear() * targetNormal;
                    const T residual = weight * targetNormal.dot(transformedVertices.col(vID) - targetVertex);
                    point2SurfaceVertexConstraints.AddConstraint(vID, residual, drdV);
                }
            }
        }
    }
}

// explicitly instantiate the ICPConstraints classes
template class ICPConstraints<float>;
template class ICPConstraints<double>;

} // namespace epic::nls
