// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <nls/Context.h>
#include <nls/Cost.h>
#include <nls/DiffDataMatrix.h>
#include <nls/utils/ConfigurationParameter.h>

namespace epic::nls {

template <class T>
class DeformationModel {
public:
    DeformationModel() = default;
    virtual ~DeformationModel() {}

    //! Evaluates the vertices of the mesh
    virtual DiffDataMatrix<T, 3, -1> EvaluateVertices(Context<T>* context) = 0;

    //! Evaluates the model constraints such as per-vertex offsets, strain, identity regularization. Depends on the deformation model.
    virtual Cost<T> EvaluateModelConstraints(Context<T>* context) = 0;

    //! Returns the configuration parameters of this deformation model
    virtual const Configuration& GetConfiguration() const = 0;

    //! Sets the configuration of the deformation model
    virtual void SetConfiguration(const Configuration& config) = 0;
};


} // namespace epic::nls
