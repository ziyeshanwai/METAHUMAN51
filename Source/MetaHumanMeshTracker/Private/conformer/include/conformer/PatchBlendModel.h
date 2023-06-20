// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nrr/deformation_models/DeformationModel.h>
#include <nls/DiffDataMatrix.h>
#include <nls/geometry/Affine.h>
#include <nls/geometry/Mesh.h>

namespace epic::nls {

template <class T>
class PatchBlendModel {
public:
    PatchBlendModel();
    ~PatchBlendModel();
    PatchBlendModel(PatchBlendModel&& other);
    PatchBlendModel(const PatchBlendModel& other) = delete;
    PatchBlendModel& operator=(PatchBlendModel&& other);
    PatchBlendModel& operator=(const PatchBlendModel& other) = delete;

    std::pair<DiffDataMatrix<T, 3, -1>, Cost<T>> EvaluateVerticesAndConstraints(Context<T>* context);

    const Configuration& GetConfiguration() const;
    void SetConfiguration(const Configuration& config);

    //! Load the identity model
    void LoadModel(const std::string& identityModelFile);

    //! @returns the number of parameters
    int NumParameters() const;

    //! @returns the number of patches
    int NumPatches() const;

    //! Resets the model parameters to default.
    void ResetParameters();

    //! @returns the deformed vertex positions without any rigid transformation
    Eigen::Matrix<T, 3, -1> DeformedVertices();

    //! Transform all patches
    void TransformPatches(const Affine<T, 3, 3>& aff);

    //! Bake the rotation linearization
    void BakeRotationLinearization();

private:
    void UpdateRegionModels(bool withModes);

    DiffDataMatrix<T, 3, -1> EvaluateVertices(Context<T>* context);
    DiffData<T> EvaluateRegularization(Context<T>* context);
    DiffData<T> EvaluatePatchSmoothness(Context<T>* context);

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};


} // namespace epic::nls
