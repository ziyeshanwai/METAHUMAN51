// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/MarkerConstraints.h>

#include <nls/functions/BarycentricCoordinatesFunction.h>
#include <nls/functions/PointPointConstraintFunction.h>
#include <nls/functions/PointSurfaceConstraintFunction.h>

namespace epic::nls {

template <class T>
struct MarkerConstraints<T>::Private
{
    // Barycentric coordinates of source markers.
    std::vector<BarycentricCoordinates<T>> sourceMarkers;
    // Positions of target markers. Number of columns corresponds to number of markers.
    Eigen::Matrix<T, 3, -1> targetMarkers;
    // Normals of target markers. Number of columns corresponds to number of markers.
    Eigen::Matrix<T, 3, -1> targetMarkersNormals;
    // Individual marker weights.
    Eigen::Vector<T, -1> markerWeights;
    // Weight used for all markers.
    T globalMarkerWeight{0.0};
    // Number of vertices in the source mesh.
    int numVertices{0};

    Configuration config = { std::string("Marker Configuration") , {
        //!< adapt between point2surface constraint (point2point = 0) to point2point constraint (point2point = 1)
        { "point2point", ConfigurationParameter(T(0), T(0), T(1)) },
        //!< how much to increase the marker influence on the optimization (0 - same as vertices, 1 - x10, 2 - x100, etc.)
        { "globalWeight", ConfigurationParameter(int(2), int(0), int(10)) },
        //!< a scalar less than 1 to use as a base for non-linear decrease of the marker influence with each iteration. w_i = w * (powBase ^ (i/maxIterations))
        { "powBase", ConfigurationParameter(T(0.001), T(0), T(1)) }
    } };

    //! Updates weight for all markers based on configuration and the ratio between number of markers and vertices.
    void UpdateGlobalMarkerWeight()
    {
        if (sourceMarkers.size() == 0) {
            return;
        }
        const int globalWeight = config["globalWeight"].template Value<int>();
        const int verticesMarkersRatio = numVertices / static_cast<int>(sourceMarkers.size());
        // marker weight is dependent on the ratio between number of vertices and markers
        // it's calculated as:
        // w = ratio * (10^globalWeight)
        // so globalWeight influences the order of magnitude of the difference between marker and vertex weights
        globalMarkerWeight = static_cast<T>(verticesMarkersRatio) * static_cast<T>(std::pow(10.0, globalWeight));
    }
};


template <class T>
MarkerConstraints<T>::MarkerConstraints() : m(std::make_unique<Private>())
{
}

template <class T> MarkerConstraints<T>::~MarkerConstraints() = default;
template <class T> MarkerConstraints<T>::MarkerConstraints(MarkerConstraints&& other) = default;
template <class T> MarkerConstraints<T>& MarkerConstraints<T>::operator=(MarkerConstraints&& other) = default;

template <class T>
const Configuration& MarkerConstraints<T>::GetConfiguration() const
{
    return m->config;
}

template <class T>
void MarkerConstraints<T>::SetConfiguration(const Configuration& config, int numVertices)
{
    m->config.Set(config);
    m->numVertices = numVertices;
    m->UpdateGlobalMarkerWeight();
}

template <class T>
void MarkerConstraints<T>::UpdateSourceMarker(int markerID, const BarycentricCoordinates<T>& coordinates)
{
    if (markerID < 0 || markerID >= static_cast<int>(m->sourceMarkers.size())) {
        throw std::runtime_error("marker index out of bounds, " + std::to_string(markerID) + ", [0, " + std::to_string(m->sourceMarkers.size()) + ")");
    }
    m->sourceMarkers[markerID] = coordinates;
}

template <class T>
void MarkerConstraints<T>::UpdateTargetMarker(int markerID, const Eigen::Vector3<T>& position, const Eigen::Vector3<T>& normal)
{
    if (markerID < 0 || markerID >= m->targetMarkers.cols()) {
        throw std::runtime_error("marker index out of bounds, " + std::to_string(markerID) + ", [0, " + std::to_string(m->targetMarkers.cols()) + ")");
    }
    m->targetMarkers.col(markerID) = position;
    m->targetMarkersNormals.col(markerID) = normal;
}

template <class T>
void MarkerConstraints<T>::SetMarkers(const std::vector<BarycentricCoordinates<T>>& sourceMarkers,
                                        const Eigen::Matrix<T, 3, -1>& targetMarkers,
                                        const Eigen::Matrix<T, 3, -1>& targetNormals,
                                        const Eigen::Vector<T, -1>& markerWeights)
{
    if ((static_cast<int>(sourceMarkers.size()) != targetMarkers.cols()) || (static_cast<int>(sourceMarkers.size()) != targetNormals.cols())) {
        throw std::runtime_error("number of provided source markers, target markers and their normals do not match!");
    }
    m->sourceMarkers = sourceMarkers;
    m->targetMarkers = targetMarkers;
    m->targetMarkersNormals = targetNormals;
    m->markerWeights = (markerWeights.size() > 0) ? markerWeights : Eigen::VectorX<T>::Ones(sourceMarkers.size());
}

template <class T>
void MarkerConstraints<T>::RemoveMarkers()
{
    m->sourceMarkers.clear();
    m->targetMarkers.resize(3, 0);
    m->targetMarkersNormals.resize(3, 0);
    m->markerWeights.resize(0);
}

template <class T>
void MarkerConstraints<T>::UpdateIndividualMarkerWeight(int markerID, T markerWeight)
{
    if (markerID < 0 || markerID >= m->markerWeights.size()) {
        throw std::runtime_error("marker index out of bounds, " + std::to_string(markerID) + ", [0, " + std::to_string(m->markerWeights.size()) + ")");
    }
    m->markerWeights[markerID] = markerWeight;
}

template <class T>
Cost<T> MarkerConstraints<T>::Evaluate(const DiffDataMatrix<T, 3, -1>& vertices, int numIterations, int currentIteration)
{
    const T markerPoint2PointWeight = m->config["point2point"].template Value<T>();
    const T markerPowBase = m->config["powBase"].template Value<T>();

    Cost<T> cost;

    if (m->sourceMarkers.size() > 0) {
        const T markerPoint2SurfaceWeight = std::max(T(0), T(1) - markerPoint2PointWeight);
        const T currentGlobalMarkerWeight = m->globalMarkerWeight * (std::pow(markerPowBase, static_cast<T>(currentIteration) / static_cast<T>(numIterations)) - markerPowBase);
        const DiffDataMatrix<T, 3, -1> sourceMarkersVertices = BarycentricCoordinatesFunction<T, 3>::Evaluate(vertices, m->sourceMarkers);
        if (markerPoint2PointWeight > T(0)) {
            const DiffData<T> markerResidual = PointPointConstraintFunction<T, 3>::Evaluate(sourceMarkersVertices, m->targetMarkers, m->markerWeights, markerPoint2PointWeight);
            cost.Add(markerResidual, currentGlobalMarkerWeight, "marker_p2p");
        }
        if (markerPoint2SurfaceWeight > T(0)) {
            const DiffData<T> markerResidual = PointSurfaceConstraintFunction<T, 3>::Evaluate(sourceMarkersVertices, m->targetMarkers, m->targetMarkersNormals, m->markerWeights, markerPoint2SurfaceWeight);
            cost.Add(markerResidual, currentGlobalMarkerWeight, "marker_p2s");
        }
    }

    return cost;
}

// explicitly instantiate the MarkerConstraints classes
template class MarkerConstraints<float>;
template class MarkerConstraints<double>;

} // namespace epic::nls
