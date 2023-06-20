// Copyright Epic Games, Inc. All Rights Reserved.

#include <nrr/PTPConstraints.h>

#include <carbon/utils/Profiler.h>
#include <nls/functions/PointPointConstraintFunction.h>
#include <nls/functions/PointSurfaceConstraintFunction.h>
#include <nls/geometry/MeshCorrespondenceSearch.h>

#include <algorithm>

namespace epic::nls {

template <class T>
struct PTPConstraints<T>::Private
{
    //! The target vertices
    Eigen::Matrix<T, 3, -1> targetVertices;

    //! Weighting mask defining the weight per search point
    VertexWeights<T> perVertexWeights;

    Configuration config = {std::string("PTP Constraints Configuration"), {
        //!< adapt between point2surface constraint (point2point = 0) to point2point constraint (point2point = 1)
        { "point2point", ConfigurationParameter(T(1), T(0), T(1)) }
    }};
};


template <class T>
PTPConstraints<T>::PTPConstraints() : m(std::make_unique<Private>())
{
}


template <class T> PTPConstraints<T>::~PTPConstraints() = default;
template <class T> PTPConstraints<T>::PTPConstraints(PTPConstraints&& other) = default;
template <class T> PTPConstraints<T>& PTPConstraints<T>::operator=(PTPConstraints&& other) = default;


template <class T>
const Configuration& PTPConstraints<T>::GetConfiguration() const
{
    return m->config;
}

template <class T>
void PTPConstraints<T>::SetConfiguration(const Configuration& config)
{
    m->config.Set(config);
}

template <class T>
void PTPConstraints<T>::SetVertexWeights(const VertexWeights<T>& vertexWeights)
{
    m->perVertexWeights = vertexWeights;
}

template <class T>
void PTPConstraints<T>::SetTarget(const Eigen::Matrix<T, 3, -1>& targetVertices)
{
    m->targetVertices = targetVertices;
}

template <class T>
Cost<T> PTPConstraints<T>::Evaluate(const DiffDataMatrix<T, 3, -1>& vertices)
{
    PROFILING_FUNCTION(PROFILING_COLOR_PINK);

    const T point2pointWeight = m->config["point2point"].template Value<T>();

    Cost<T> cost;

    if (point2pointWeight > T(0)) {
        const DiffData<T> residual = PointPointConstraintFunction<T, 3>::Evaluate(vertices, m->targetVertices, m->perVertexWeights.Weights(), point2pointWeight);
        cost.Add(residual, T(1), "ptp");
    }

    return cost;
}

// explicitly instantiate the PTPConstraints classes
template class PTPConstraints<float>;
template class PTPConstraints<double>;

} // namespace epic::nls
