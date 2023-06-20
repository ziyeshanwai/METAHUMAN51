// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nls/DiffDataMatrix.h>
#include <nls/geometry/Mesh.h>
#include <nls/utils/ConfigurationParameter.h>
#include <nrr/VertexWeights.h>


namespace epic::nls {

/**
 * Class to evaluate constraints relative to the eyeballs
 */
template <class T>
class EyeballConstraints
{
public:
    EyeballConstraints();
    ~EyeballConstraints();
    EyeballConstraints(EyeballConstraints&&);
    EyeballConstraints(EyeballConstraints&) = delete;
    EyeballConstraints& operator=(EyeballConstraints&&);
    EyeballConstraints& operator=(const EyeballConstraints&) = delete;

    const Configuration& GetConfiguration() const;

    void SetConfiguration(const Configuration& config);

    //! Set the eyeball mesh.
    void SetEyeballMesh(const Mesh<T>& eyeballMesh);

    [[deprecated("deprecated, use SetInterfaceVertices() instead")]]
    void SetTargetVertexWeights(const VertexWeights<T>& interfaceVertexWeights) { SetInterfaceVertices(interfaceVertexWeights); }

    //! Set the vertices that are influenced by the eyeball.
    void SetInfluenceVertices(const VertexWeights<T>& influenceVertexWeights);

    //! Set the vertices that lie on the eyeball.
    void SetInterfaceVertices(const VertexWeights<T>& interfaceVertexWeights);

    //! Calculate for each vertex the distance to the eyeball
    std::vector<T> GetEyeballDistances(const Eigen::Matrix<T, 3, -1>& vertices) const;

    //! Set the eyeball rest pose. This is only required for influence vertices.
    void SetRestPose(const Eigen::Matrix<T, 3, -1>& eyeballVertices, const Eigen::Matrix<T, 3, -1>& targetVertices);

    //! Set the current eyeball pose.
    void SetEyeballPose(const Eigen::Matrix<T, 3, -1>& eyeballVertices);

    //! Evaluate the eyeball constraints.
    DiffData<T> EvaluateEyeballConstraints(const DiffDataMatrix<T, 3, -1>& eyeballVertices);

    /**
     * Project the vertices that are interface vertices onto the eyeball, the influence vertices are projected
     * depending on the distance in the reference as set up by SetRestPose() and @p distanceRadiusEffect.
     */
    Eigen::Matrix<T, 3, -1> Project(const Eigen::Matrix<T, 3, -1>& vertices, const T distanceRadiusEffect);

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};

} // namespace epic::nls
