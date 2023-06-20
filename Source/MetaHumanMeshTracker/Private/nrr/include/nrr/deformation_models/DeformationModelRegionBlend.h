// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nrr/deformation_models/DeformationModel.h>
#include <nls/geometry/Affine.h>
#include <nls/geometry/Mesh.h>

#include <nls/BoundedVectorVariable.h>

namespace epic::nls {

template <class T>
class DeformationModelRegionBlend final : public DeformationModel<T> {
public:
    DeformationModelRegionBlend();
    virtual ~DeformationModelRegionBlend() override;
    DeformationModelRegionBlend(DeformationModelRegionBlend&& other);
    DeformationModelRegionBlend(const DeformationModelRegionBlend& other) = delete;
    DeformationModelRegionBlend& operator=(DeformationModelRegionBlend&& other);
    DeformationModelRegionBlend& operator=(const DeformationModelRegionBlend& other) = delete;

    //! inherited from DeformationModel
    virtual DiffDataMatrix<T, 3, -1> EvaluateVertices(Context<T>* context) override;
    virtual Cost<T> EvaluateModelConstraints(Context<T>* context) override;
    virtual const Configuration& GetConfiguration() const override;
    virtual void SetConfiguration(const Configuration& config) override;

    //! Load the region blend model
    void LoadModel(const std::string& regionBlendModelFile);

    //! @returns the number of parameters
    int NumParameters() const;

    //! @returns number of vertices.
    int NumVertices() const;

    //! Resets the model parameters to default.
    void ResetParameters();

    //! @returns the current model parameters
    const Vector<T>& ModelParameters() const;

    //! @returns sets the current model parameters
    void SetModelParameters(const Vector<T>& params);

    //! @returns the deformed vertex positions without any rigid transformation
    Eigen::Matrix<T, 3, -1> DeformedVertices() const;

    //! Set the current rigid transformation
    void SetRigidTransformation(const Affine<T, 3, 3>& affine);

    //! @returns the current rigid transformation
    Affine<T, 3, 3> RigidTransformation() const;

    //! @returns the variable representing the parameters
    BoundedVectorVariable<T>* Variable();

    //! @returns the scale of the model
    T Scale() const;

    //! @returns the pivot which is used for scaling i.e. scaled_model = scale (model - scaling_pivot) + scaling_pivot
    Eigen::Vector3<T> ScalingPivot() const;

    const std::vector<std::string>& RegionNames() const;
    void SetRegionNames(const std::vector<std::string>& regionNames);
    void SetRegion(const std::string& regionName, const Vector<T>& regionData);
    void SetRegions(const std::map<std::string, Vector<T>>& regions);

    const std::vector<std::string>& CharacterNames() const;
    void SetCharacterNames(const std::vector<std::string>& charNames);
    void SetCharacter(const std::string& charName, const Eigen::Matrix<T, 3, -1>& charData);
    void SetCharacters(const std::map<std::string, Eigen::Matrix<T, 3, -1>>& characters);

    void SetArchetype(const Eigen::Matrix<T, 3, -1>& archetype);

    void SetSymmetricRegions(const std::vector<std::pair<std::string, std::string>>& symmetricRegions);

    void GenerateModel();
private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};


} // namespace epic::nls
