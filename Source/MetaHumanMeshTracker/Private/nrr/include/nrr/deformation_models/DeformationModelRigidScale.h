// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nrr/deformation_models/DeformationModel.h>
#include <nls/geometry/Affine.h>
#include <nls/geometry/DiffDataAffine.h>

namespace epic::nls {

template <class T>
class DeformationModelRigidScale final : public DeformationModel<T> {
public:
    DeformationModelRigidScale();
    virtual ~DeformationModelRigidScale() override;
    DeformationModelRigidScale(DeformationModelRigidScale&& other);
    DeformationModelRigidScale(const DeformationModelRigidScale& other) = delete;
    DeformationModelRigidScale& operator=(DeformationModelRigidScale&& other);
    DeformationModelRigidScale& operator=(const DeformationModelRigidScale& other) = delete;

    //! inherited from DeformationModel
    virtual DiffDataMatrix<T, 3, -1> EvaluateVertices(Context<T>* context) override;
    virtual Cost<T> EvaluateModelConstraints(Context<T>* context) override;
    virtual const Configuration& GetConfiguration() const override;
    virtual void SetConfiguration(const Configuration& config) override;

    //! Set the current mesh (required)
    void SetVertices(const Eigen::Matrix<T, 3, -1>& vertices);

    /**
     * @returns the current rigid transformation either assuming the vertices are first scaled and then transformed, or alternatively first transformed and then scaled.
     * Note that the deformation model first moves the vertices to the center of gravity before scaling and rotating the vertices. However, when retrieving the
     * transformation and scale, the customer needs to determine whether first to transform and then scale (preScale=false) or scale and then transform (preScale=true)
     */
    Affine<T, 3, 3> RigidTransformation(bool preScale) const;

    /**
     * Set the current rigid transformation either assuming the vertices are first scaled and then transformed, or alternatively first transformed and then scaled.
     */
    void SetRigidTransformation(const Affine<T, 3, 3>& affine, bool preScale);

    //! @returns the estimate scale
    T Scale() const;

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};


} // namespace epic::nls
