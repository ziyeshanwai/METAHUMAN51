// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nls/Cost.h>
#include <nls/DiffDataMatrix.h>
#include <nls/geometry/Camera.h>
#include <nls/geometry/Mesh.h>
#include <nls/geometry/MeshCorrespondenceSearch.h>
#include <nls/utils/ConfigurationParameter.h>
#include <nrr/VertexWeights.h>


namespace epic::nls {

/**
 * Class to evaluate ICP correspondences (point2point and/or point2surface)
 */
template <class T>
class PTPConstraints
{
public:
    PTPConstraints();
    ~PTPConstraints();
    PTPConstraints(PTPConstraints&& other);
    PTPConstraints(const PTPConstraints& other) = delete;
    PTPConstraints& operator=(PTPConstraints&& other);
    PTPConstraints& operator=(const PTPConstraints& other) = delete;

    //! Sets the correspondence search vertex weights
    void SetVertexWeights(const VertexWeights<T>& vertexWeights);

    //! Set the target vertices
    void SetTarget(const Eigen::Matrix<T, 3, -1>& targetVertices);

    const Configuration& GetConfiguration() const;

    void SetConfiguration(const Configuration& config);

    /**
     * Evaluate the ICP constraint.
     * @param[in] vertices  The differentiable input vertices.
     */
    Cost<T> Evaluate(const DiffDataMatrix<T, 3, -1>& vertices);

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};

} // namespace epi::nls
