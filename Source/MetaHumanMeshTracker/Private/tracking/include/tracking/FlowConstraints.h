// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nls/Cost.h>
#include <nls/DiffDataMatrix.h>
#include <nls/geometry/Camera.h>
#include <nls/geometry/Mesh.h>
#include <nls/geometry/VertexConstraints.h>
#include <nls/utils/ConfigurationParameter.h>
#include <nrr/VertexWeights.h>

#include <map>
#include <vector>

namespace epic::nls {

template <class T>
class FlowConstraintsData {
public:
    Camera<T> camera;
    Eigen::Matrix<T, 2, -1> targetPositions;
    Eigen::VectorX<T> weights;
    Eigen::VectorXi vertexIndices;
};


/**
 * Class to evaluate flow constraints (2D point2point)
 */
template <class T>
class FlowConstraints
{
public:
    FlowConstraints();
    ~FlowConstraints();
    FlowConstraints(FlowConstraints&& other);
    FlowConstraints(const FlowConstraints& other) = delete;
    FlowConstraints& operator=(FlowConstraints&& other);
    FlowConstraints& operator=(const FlowConstraints& other) = delete;

    //! Sets the vertex weights
    void SetVertexWeights(const VertexWeights<T>& vertexWeights);

    //! Set FlowData
    void SetFlowData(const std::map<std::string, FlowConstraintsData<T>>& flowConstraintsData);

    //! Set the flow weight
    void SetFlowWeight(T weight);

    Cost<T> Evaluate(const DiffDataMatrix<T, 3, -1>& vertices, std::map<std::string, FlowConstraintsData<T>>* debugFlowConstraints = nullptr);

    void SetupFlowConstraints(const Eigen::Transform<T, 3, Eigen::Affine>& rigidTransform,
                              const Eigen::Matrix<T, 3, -1>& vertices,
                              VertexConstraints<T, 2, 1>& flowConstraints) const;

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};

} // namespace epic::nls
