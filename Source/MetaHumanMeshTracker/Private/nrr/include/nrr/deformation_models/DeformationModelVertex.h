// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nrr/deformation_models/DeformationModel.h>
#include <nls/geometry/Affine.h>
#include <nls/geometry/Mesh.h>

namespace epic::nls {

template <class T>
class DeformationModelVertex final : public DeformationModel<T> {
public:
    static constexpr const char* ConfigName() { return "Per-vertex Deformation Model Configuration"; }

    DeformationModelVertex();
    virtual ~DeformationModelVertex() override;
    DeformationModelVertex(DeformationModelVertex&& other);
    DeformationModelVertex(const DeformationModelVertex& other) = delete;
    DeformationModelVertex& operator=(DeformationModelVertex&& other);
    DeformationModelVertex& operator=(const DeformationModelVertex& other) = delete;

    //! inherited from DeformationModel
    virtual DiffDataMatrix<T, 3, -1> EvaluateVertices(Context<T>* context) override;
    virtual Cost<T> EvaluateModelConstraints(Context<T>* context) override;
    virtual const Configuration& GetConfiguration() const override;
    virtual void SetConfiguration(const Configuration& config) override;

    std::pair<DiffDataMatrix<T, 3, -1>, DiffDataMatrix<T, 3, -1>> EvaluateBothStabilizedAndTransformedVertices(Context<T>* context);

    //! Set the mesh topolgy (required for model constraints)
    void SetMeshTopology(const Mesh<T>& mesh);

    //! Set the rest vertex positions (required)
    void SetRestVertices(const Eigen::Matrix<T, 3, -1>& vertices);

    //! Sets the vertex offsets (optional)
    void SetVertexOffsets(const Eigen::Matrix<T, 3, -1>& offsets);

    //! Returns the vertex offsets
    Eigen::Matrix<T, 3, -1> VertexOffsets() const;

    //! Returns the deformed vertex positions (RestVertices() + VertexOffsets())
    Eigen::Matrix<T, 3, -1> DeformedVertices() const;

    //! Set the current rigid transformation
    void SetRigidTransformation(const Affine<T, 3, 3>& affine);

    //! Returns the current rigid transformation
    Affine<T, 3, 3> RigidTransformation() const;

    //! Make vertices with specified IDs constant.
    void MakeVerticesConstant(const std::vector<int>& constantIndices);

    //! Make all vertices mutable.
    void MakeVerticesMutable();

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};


} // namespace epic::nls
