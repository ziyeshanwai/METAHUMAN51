// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nls/Cost.h>
#include <nls/DiffDataMatrix.h>
#include <nls/geometry/BarycentricCoordinates.h>
#include <nls/utils/ConfigurationParameter.h>

namespace epic::nls {

/**
 * Class to evaluate marker constraints (point2point and/or point2surface)
 */
template <class T>
class MarkerConstraints
{
public:
    MarkerConstraints();
    ~MarkerConstraints();
    MarkerConstraints(MarkerConstraints&& other);
    MarkerConstraints(const MarkerConstraints& other) = delete;
    MarkerConstraints& operator=(MarkerConstraints&& other);
    MarkerConstraints& operator=(const MarkerConstraints& other) = delete;

    const Configuration& GetConfiguration() const;
    void SetConfiguration(const Configuration& config, int numVertices);

    //! Updates barycentric coordinates for marker with specified index in the source markers vector.
    void UpdateSourceMarker(int markerID, const BarycentricCoordinates<T>& coordinates);

    //! Updates position and normal for marker with specified index in the target markers vector.
    void UpdateTargetMarker(int markerID, const Eigen::Vector3<T>& position, const Eigen::Vector3<T>& normal);

    /** Sets source and target markers.
    * @param sourceMarkers   Barycentric coordinates of source markers.
    * @param targetMarkers   Positions of target markers.
    * @param targetNormals   Normals of target markers.
    * @param markerWeights   Individual weights for each marker. If not set, uses a vector of ones.
    */
    void SetMarkers(const std::vector<BarycentricCoordinates<T>>& sourceMarkers,
                    const Eigen::Matrix<T, 3, -1>& targetMarkers,
                    const Eigen::Matrix<T, 3, -1>& targetNormals,
                    const Eigen::Vector<T, -1>& markerWeights = Eigen::Vector<T, -1>());

    //! Removes all source and target markers.
    void RemoveMarkers();

    //! Updates marker weight for marker with specified index in the markers vector.
    void UpdateIndividualMarkerWeight(int markerID, T markerWeight);

    /* Evaluates marker constraints. The weight of these constraints decreases with each iteration.
    * @param vertices           Vertices based on which positions of source markers are calculated.
    * @param numIterations      Maximum number of iterations.
    * @param currentIteration   Current iteration.
    */
    Cost<T> Evaluate(const DiffDataMatrix<T, 3, -1>& vertices, int numIterations = 1, int currentIteration = 0);

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};

} // namespace epic::nls
