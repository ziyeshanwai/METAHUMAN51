// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nrr/deformation_models/DeformationModel.h>
#include <nls/geometry/Affine.h>
#include <nls/geometry/Mesh.h>

namespace epic::nls {

template <class T>
class DeformationModelPatchBlend final : public DeformationModel<T> {
public:
    DeformationModelPatchBlend();
    virtual ~DeformationModelPatchBlend() override;
    DeformationModelPatchBlend(DeformationModelPatchBlend&& other);
    DeformationModelPatchBlend(const DeformationModelPatchBlend& other) = delete;
    DeformationModelPatchBlend& operator=(DeformationModelPatchBlend&& other);
    DeformationModelPatchBlend& operator=(const DeformationModelPatchBlend& other) = delete;

    //! inherited from DeformationModel
    virtual DiffDataMatrix<T, 3, -1> EvaluateVertices(Context<T>* context) override;
    virtual Cost<T> EvaluateModelConstraints(Context<T>* context) override;
    virtual const Configuration& GetConfiguration() const override;
    virtual void SetConfiguration(const Configuration& config) override;

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

    //! @returns the deformed vertex positions with or without rigid transformation
    DiffDataMatrix<T, 3, -1> EvaluateVertices(Context<T>* context, bool withPose);

    //! Set the current rigid transformation
    void SetRigidTransformation(const Affine<T, 3, 3>& affine);

    //! Returns the current rigid transformation
    Affine<T, 3, 3> RigidTransformation() const;

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};


} // namespace epic::nls
