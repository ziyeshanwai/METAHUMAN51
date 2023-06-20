// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nrr/deformation_models/DeformationModel.h>
#include <nls/geometry/Affine.h>
#include <nls/geometry/DiffDataAffine.h>

namespace epic::nls {

template <class T>
class DeformationModelRigid final : public DeformationModel<T> {
public:
    static constexpr const char* ConfigName() { return "Rigid Deformation Model Configuration"; }

    DeformationModelRigid();
    virtual ~DeformationModelRigid() override;
    DeformationModelRigid(DeformationModelRigid&& other);
    DeformationModelRigid(const DeformationModelRigid& other) = delete;
    DeformationModelRigid& operator=(DeformationModelRigid&& other);
    DeformationModelRigid& operator=(const DeformationModelRigid& other) = delete;

    //! inherited from DeformationModel
    virtual DiffDataMatrix<T, 3, -1> EvaluateVertices(Context<T>* context) override;
    virtual Cost<T> EvaluateModelConstraints(Context<T>* context) override;
    virtual const Configuration& GetConfiguration() const override;
    virtual void SetConfiguration(const Configuration& config) override;

    //! Evaluate the affine transformation
    DiffDataAffine<T, 3, 3> EvaluateAffine(Context<T>* context);

    //! Set the current mesh (required)
    void SetVertices(const Eigen::Matrix<T, 3, -1>& vertices);

    //! Set the current rigid transformation
    void SetRigidTransformation(const Affine<T, 3, 3>& affine);

    //! Returns the current rigid transformation
    Affine<T, 3, 3> RigidTransformation() const;

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};


} // namespace epic::nls
