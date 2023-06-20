// Copyright Epic Games, Inc. All Rights Reserved.

#include <tracking/FlowConstraints.h>

#include <carbon/utils/Profiler.h>
#include <nls/functions/PointPointConstraintFunction.h>

namespace epic {
namespace nls {

template <class T>
struct FlowConstraints<T>::Private
{
    //! Weighting mask defining the weight per search point.
    VertexWeights<T> vertexWeights;

    //! Flow constraints data per camera.
    std::map<std::string, FlowConstraintsData<T>> flowConstraintsData;

    //! Weight for flow. By default flow is disabled.
    T flowWeight = T(0.0);
};


template <class T>
FlowConstraints<T>::FlowConstraints() : m(std::make_unique<Private>())
{
}

template <class T> FlowConstraints<T>::~FlowConstraints() = default;
template <class T> FlowConstraints<T>::FlowConstraints(FlowConstraints&& other) = default;
template <class T> FlowConstraints<T>& FlowConstraints<T>::operator=(FlowConstraints&& other) = default;

template <class T>
void FlowConstraints<T>::SetFlowWeight(T weight)
{
    m->flowWeight = weight;
}

template <class T>
void FlowConstraints<T>::SetVertexWeights(const VertexWeights<T>& vertexWeights)
{
    m->vertexWeights = vertexWeights;
}

template <class T>
void FlowConstraints<T>::SetFlowData(const std::map<std::string, FlowConstraintsData<T>>& flowConstraintsData)
{
    m->flowConstraintsData = flowConstraintsData;
}

template <class T>
Cost<T> FlowConstraints<T>::Evaluate(const DiffDataMatrix<T, 3, -1>& vertices, std::map<std::string, FlowConstraintsData<T>>* debugFlowConstraints)
{
    PROFILING_FUNCTION(PROFILING_COLOR_PINK);

    Cost<T> cost;

    if (m->flowWeight > 0) {
        for (auto && [cameraName, flowConstraintsData] : m->flowConstraintsData) {
            DiffDataMatrix<T, 2, -1> projectedVertices = flowConstraintsData.camera.Project(vertices, /*withExtrinsics=*/true);
            Eigen::VectorX<T> vertexWeights = flowConstraintsData.weights;
            for (int32_t i = 0; i < flowConstraintsData.vertexIndices.size(); ++i) {
                const int vID = flowConstraintsData.vertexIndices[i];
                vertexWeights[i] *= m->vertexWeights.Weights()[vID];
            }
            DiffData<T> residual = PointPointConstraintFunction<T, 2>::Evaluate(projectedVertices, flowConstraintsData.vertexIndices, flowConstraintsData.targetPositions, vertexWeights, m->flowWeight);
            cost.Add(residual, T(1), cameraName + "_flowConstraint");
            if (debugFlowConstraints) {
                FlowConstraintsData<T> copiedData = flowConstraintsData;
                copiedData.weights = vertexWeights;
                (*debugFlowConstraints)[cameraName] = copiedData;
            }
        }
    }

    return cost;
}

template <class T>
void FlowConstraints<T>::SetupFlowConstraints(const Eigen::Transform<T, 3, Eigen::Affine>& rigidTransform,
                                              const Eigen::Matrix<T, 3, -1>& vertices,
                                              VertexConstraints<T, 2, 1>& flowConstraints) const
{
    if (m->flowWeight > 0) {
        int numTotalConstraints = 0;
        for (auto && [cameraName, flowConstraintsData] : m->flowConstraintsData) {
            numTotalConstraints += int(flowConstraintsData.vertexIndices.size());
        }

        flowConstraints.ResizeToFitAdditionalConstraints(numTotalConstraints);

        for (auto && [cameraName, flowConstraintsData] : m->flowConstraintsData) {
            const Eigen::Matrix<T, 3, 3> K = flowConstraintsData.camera.Intrinsics();
            const Eigen::Matrix<T, 4, 4> totalTransform = flowConstraintsData.camera.Extrinsics().Matrix() * rigidTransform.matrix();
            const Eigen::Matrix<T, 3, 3> KR = K * totalTransform.block(0, 0, 3, 3);
            const Eigen::Vector<T, 3> Kt = K * totalTransform.block(0, 3, 3, 1);

            for (int i = 0; i < int(flowConstraintsData.vertexIndices.size()); ++i) {
                const T weight = flowConstraintsData.weights[i] * sqrt(m->flowWeight);
                if (weight > 0) {
                    const int vID = flowConstraintsData.vertexIndices[i];
                    const Eigen::Vector2<T> targetPixelPosition = flowConstraintsData.targetPositions.col(i);
                    const Eigen::Vector3<T> pix = KR * vertices.col(vID) + Kt;
                    const T x = pix[0];
                    const T y = pix[1];
                    const T z = pix[2];
                    const T invZ = T(1) / z;
                    const Eigen::Vector2<T> residual = weight * (invZ * pix.template head<2>() - targetPixelPosition);
                    // dpix[0] / d(x, y, z) = [1/z,   0, -x/z^2]
                    // dpix[1] / d(x, y, z) = [  0, 1/z, -y/z^2]
                    // dpix[0] / d(vx, vy, vz) = dpix[0] / d(x, y, z) * d(x, y, z) / d(vx, vy, vz)
                    // d(x, y, z) / d(vx, vy, vz) = KR
                    Eigen::Matrix<T, 2, 3> drdV;
                    drdV(0, 0) = weight * invZ * (KR(0, 0) - (x * invZ) * KR(2, 0));
                    drdV(0, 1) = weight * invZ * (KR(0, 1) - (x * invZ) * KR(2, 1));
                    drdV(0, 2) = weight * invZ * (KR(0, 2) - (x * invZ) * KR(2, 2));
                    drdV(1, 0) = weight * invZ * (KR(1, 0) - (y * invZ) * KR(2, 0));
                    drdV(1, 1) = weight * invZ * (KR(1, 1) - (y * invZ) * KR(2, 1));
                    drdV(1, 2) = weight * invZ * (KR(1, 2) - (y * invZ) * KR(2, 2));

                    flowConstraints.AddConstraint(Eigen::Vector<int, 1>(vID), residual, drdV);
                }
            }
        }
    }
}



// explicitly instantiate the FlowConstraints classes
template class FlowConstraints<float>;
template class FlowConstraints<double>;

} // namespace nls
} //namespace epic
