// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/EyeballConstraints.h>

#include <carbon/geometry/AABBTree.h>
#include <nls/functions/PointPointConstraintFunction.h>
#include <nls/functions/PointSurfaceConstraintFunction.h>
#include <nls/geometry/BarycentricCoordinates.h>

namespace epic::nls {

template <class T>
constexpr T SqrtOfMaxValue() { return std::sqrt(std::numeric_limits<T>::max()); }

template <class T>
struct EyeballConstraints<T>::Private
{
    //! The eyeball mesh
    Mesh<T> eyeballMesh;

    //! The center of the eyeball which is used as center for raycasting to calculate intersection with the eyeball
    Eigen::Vector3<T> eyeballCenter;

    //! The AABB tree for ray-eyeball intersection
    std::unique_ptr<carbon::AABBTree<T>> eyeballAABBTree;

    //! Vertices that should lie directly on the eyeball
    VertexWeights<T> interfaceVertexWeights;

    //! Vertices that are affected by the eyeball and are set to have a distance based on a rest pose
    VertexWeights<T> influenceVertexWeights;


    //! Distance of each target point to the eyeballMesh
    std::vector<T> eyeballDistances;

    Configuration config = {std::string("Eyeball Constraints Configuration"), {
        //!< how much weight to use on eyeball constraint
        { "eyeball", ConfigurationParameter(T(1), T(0), T(1)) },
        //!< minimum distance to the eyeball
        { "eyeballMinimumDistance", ConfigurationParameter(T(0), T(0), T(1)) },
    }};
};

template <class T>
EyeballConstraints<T>::EyeballConstraints()
    : m(std::make_unique<Private>())
{
}

template <class T> EyeballConstraints<T>::~EyeballConstraints() = default;
template <class T> EyeballConstraints<T>::EyeballConstraints(EyeballConstraints&&) = default;
template <class T> EyeballConstraints<T>& EyeballConstraints<T>::operator=(EyeballConstraints&&) = default;


template <class T>
const Configuration& EyeballConstraints<T>::GetConfiguration() const
{
    return m->config;
}

template <class T>
void EyeballConstraints<T>::SetConfiguration(const Configuration& config)
{
    m->config.Set(config);
}

template <class T>
void EyeballConstraints<T>::SetInfluenceVertices(const VertexWeights<T>& influenceVertexWeights)
{
    m->influenceVertexWeights = influenceVertexWeights;
}

template <class T>
void EyeballConstraints<T>::SetInterfaceVertices(const VertexWeights<T>& interfaceVertexWeights)
{
    m->interfaceVertexWeights = interfaceVertexWeights;
}

template <class T>
void EyeballConstraints<T>::SetEyeballMesh(const Mesh<T>& eyeballMesh)
{
    m->eyeballMesh = eyeballMesh;
    m->eyeballMesh.Triangulate();
    m->eyeballMesh.CalculateVertexNormals();
    m->eyeballCenter = m->eyeballMesh.Vertices().rowwise().mean();
}

template <class T>
std::vector<T> EyeballConstraints<T>::GetEyeballDistances(const Eigen::Matrix<T, 3, -1>& vertices) const
{
    std::vector<T> eyeballDistances;
    if (!m->eyeballAABBTree) {
        CARBON_CRITICAL("rest pose of EyeballConstraints has not been set");
    }
    eyeballDistances.clear();
    eyeballDistances.reserve(vertices.cols());

    for (int i = 0; i < static_cast<int>(vertices.cols()); ++i) {
        const Eigen::Vector3<T> vertexPos = vertices.col(i);
        const Eigen::Vector3<T> delta = (vertexPos - m->eyeballCenter);
        const auto[tID, bcWeights, dist] = m->eyeballAABBTree->intersectRay(m->eyeballCenter.transpose(), delta.normalized().transpose());
        eyeballDistances.push_back(delta.norm() - dist);
    }

    return eyeballDistances;
}

template <class T>
void EyeballConstraints<T>::SetRestPose(const Eigen::Matrix<T, 3, -1>& eyeballVertices, const Eigen::Matrix<T, 3, -1>& targetVertices)
{
    SetEyeballPose(eyeballVertices);

    const int numNonzeros = int(m->influenceVertexWeights.NonzeroVertices().size());
    Eigen::Matrix<T, 3, -1> vertices(3, numNonzeros);
    for (int i = 0; i < numNonzeros; ++i) {
        vertices.col(i) = targetVertices.col(m->influenceVertexWeights.NonzeroVertices()[i]);
    }
    m->eyeballDistances = GetEyeballDistances(vertices);
}

template <class T>
void EyeballConstraints<T>::SetEyeballPose(const Eigen::Matrix<T, 3, -1>& eyeballVertices)
{
    m->eyeballMesh.SetVertices(eyeballVertices);
    m->eyeballMesh.CalculateVertexNormals();
    m->eyeballAABBTree = std::make_unique<carbon::AABBTree<T>>(m->eyeballMesh.Vertices().transpose(), m->eyeballMesh.Triangles().transpose());
    m->eyeballCenter = m->eyeballMesh.Vertices().rowwise().mean();
}

template <class T>
DiffData<T> EyeballConstraints<T>::EvaluateEyeballConstraints(const DiffDataMatrix<T, 3, -1>& targetVertices)
{
    const T eyeballWeight = m->config["eyeball"].template Value<T>();
    const T eyeballMinimumDistance = m->config["eyeballMinimumDistance"].template Value<T>();
    const bool useEyeballMinimumDistance = (eyeballMinimumDistance > 0);

    if (eyeballWeight > 0) {
        const int numInterfacePts = int(m->interfaceVertexWeights.NonzeroVertices().size());
        const int numInfluencePts = int(m->influenceVertexWeights.NonzeroVertices().size());
        const int numPts = numInterfacePts + numInfluencePts;
        Eigen::Matrix<T, 3, -1> positions(3, numPts);
        Eigen::Matrix<T, 3, -1> normals(3, numPts);
        Eigen::VectorXi vertexIDs(numPts);
        Eigen::VectorX<T> weights(numPts);

        // interface points are located directly on the eyeball
        for (int i = 0; i < numInterfacePts; ++i) {
            const auto [vID, weight] = m->interfaceVertexWeights.NonzeroVerticesAndWeights()[i];
            const Eigen::Vector3<T> vertexPos = targetVertices.Matrix().col(vID);
            const Eigen::Vector3<T> delta = (vertexPos - m->eyeballCenter);
            const auto[tID, bcWeights, dist] = m->eyeballAABBTree->intersectRay(m->eyeballCenter.transpose(), delta.normalized().transpose());
            const BarycentricCoordinates<T> bc(m->eyeballMesh.Triangles().col(tID), bcWeights);
            const Eigen::Vector3<T> eyeballNormal = bc.template Evaluate<3>(m->eyeballMesh.VertexNormals()).normalized();
            positions.col(i) = m->eyeballCenter + dist * delta.normalized();;
            normals.col(i) = eyeballNormal;
            vertexIDs[i] = vID;
            weights[i] = weight;
        }

        // influence points are vertices that are affected by the eyeball
        for (int i = 0; i < numInfluencePts; ++i) {
            const auto [vID, weight] = m->influenceVertexWeights.NonzeroVerticesAndWeights()[i];
            const Eigen::Vector3<T> vertexPos = targetVertices.Matrix().col(vID);
            const Eigen::Vector3<T> delta = (vertexPos - m->eyeballCenter);
            const auto[tID, bcWeights, dist] = m->eyeballAABBTree->intersectRay(m->eyeballCenter.transpose(), delta.normalized().transpose());
            const BarycentricCoordinates<T> bc(m->eyeballMesh.Triangles().col(tID), bcWeights);
            const Eigen::Vector3<T> eyeballNormal = bc.template Evaluate<3>(m->eyeballMesh.VertexNormals()).normalized();
            const T eyeballDistance = useEyeballMinimumDistance ? std::max<T>(m->eyeballDistances[i], eyeballMinimumDistance) : m->eyeballDistances[i];
            const Eigen::Vector3<T> eyeballPos = m->eyeballCenter + (dist + eyeballDistance) * delta.normalized();
            positions.col(numInterfacePts + i) = eyeballPos;
            normals.col(numInterfacePts + i) = eyeballNormal;
            vertexIDs[numInterfacePts + i] = vID;
            weights[numInterfacePts + i] = weight;
        }

        return PointSurfaceConstraintFunction<T, 3>::Evaluate(targetVertices, vertexIDs, positions, normals, weights, eyeballWeight);
    } else {
        return DiffData<T>(Vector<T>());
    }
}

template <class T>
Eigen::Matrix<T, 3, -1> EyeballConstraints<T>::Project(const Eigen::Matrix<T, 3, -1>& vertices, const T distanceRadiusEffect)
{
    Eigen::Matrix<T, 3, -1> projectedVertices = vertices;
    for (int i = 0; i < static_cast<int>(m->influenceVertexWeights.NonzeroVertices().size()); ++i) {
        const int vID = m->influenceVertexWeights.NonzeroVertices()[i];
        const Eigen::Vector3<T> vertexPos = vertices.col(vID);
        const Eigen::Vector3<T> delta = (vertexPos - m->eyeballCenter);
        const auto[tID, bcWeights, dist] = m->eyeballAABBTree->intersectRay(m->eyeballCenter.transpose(), delta.normalized().transpose());
        if (tID < 0) {
            LOG_ERROR("this should not happen");
        }
        const T distance = delta.norm() - dist;
        const T referenceEyeballDistance = m->eyeballDistances[i];
        if (referenceEyeballDistance >= 0) {
            if (dist >= 0) {
                const T weight = (distanceRadiusEffect - referenceEyeballDistance) / distanceRadiusEffect;
                if (weight > 0) {
                    const T newDistance = (weight * referenceEyeballDistance + (T(1) - weight) * distance);
                    projectedVertices.col(vID) = m->eyeballCenter + (dist + newDistance) * delta.normalized();
                }
            } else {
                // this is a vertex that is inside the eye in the deformed state. we simply move it according to the reference.
                projectedVertices.col(vID) = m->eyeballCenter + (dist + referenceEyeballDistance) * delta.normalized();
            }
        } else {
            // this is a vertex that is inside the eye in the reference. we simply move it according to the reference.
            projectedVertices.col(vID) = m->eyeballCenter + (dist + referenceEyeballDistance) * delta.normalized();
        }
    }

    // project interface vertices onto the eyeball
    for (int vID : m->interfaceVertexWeights.NonzeroVertices()) {
        const Eigen::Vector3<T> vertexPos = vertices.col(vID);
        const Eigen::Vector3<T> delta = (vertexPos - m->eyeballCenter);
        const auto[tID, bcWeights, dist] = m->eyeballAABBTree->intersectRay(m->eyeballCenter.transpose(), delta.normalized().transpose());
        projectedVertices.col(vID) = m->eyeballCenter + dist * delta.normalized();
    }

    return projectedVertices;
}

// explicitly instantiate the EyeballConstraints classes
template class EyeballConstraints<float>;
template class EyeballConstraints<double>;

} // namespace epic::nls
