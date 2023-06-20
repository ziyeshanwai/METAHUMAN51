// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nls/BoundedVectorVariable.h>
#include <nrr/deformation_models/DeformationModel.h>
#include <nls/geometry/Affine.h>

#include <nls/rig/Rig.h>
#include <nls/rig/RigLogicSolveControls.h>

#include <string>
#include <vector>

namespace epic::nls {

template <class T>
class DeformationModelRigLogic final : public DeformationModel<T> {
public:
    static constexpr const char* ConfigName() { return "RigLogic Deformation Model Configuration"; }

    DeformationModelRigLogic();
    virtual ~DeformationModelRigLogic() override;
    DeformationModelRigLogic(DeformationModelRigLogic&& other);
    DeformationModelRigLogic(const DeformationModelRigLogic& other) = delete;
    DeformationModelRigLogic& operator=(DeformationModelRigLogic&& other);
    DeformationModelRigLogic& operator=(const DeformationModelRigLogic& other) = delete;

    //! inherited from DeformationModel
    virtual DiffDataMatrix<T, 3, -1> EvaluateVertices(Context<T>* context) override;
    virtual Cost<T> EvaluateModelConstraints(Context<T>* context) override;
    virtual const Configuration& GetConfiguration() const override;
    virtual void SetConfiguration(const Configuration& config) override;

    //! Evaluates the vertices of the mesh at index @p meshIndex
    DiffDataMatrix<T, 3, -1> EvaluateVertices(Context<T>* context, int lod, int meshIndex, bool withRigid);

    //! Evaluates the vertices of multiple meshes
    std::vector<DiffDataMatrix<T, 3, -1>> EvaluateVertices(Context<T>* context, int lod, const std::vector<int>& meshIndices, bool withRigid);

    //! Evaluates the vertices of multiple meshes
    void EvaluateVertices(Context<T>* context, int lod, const std::vector<int>& meshIndices, bool withRigid, typename RigGeometry<T>::State& state);

    //! Evaluates the vertices of the mesh with name @p meshName
    DiffDataMatrix<T, 3, -1> EvaluateVertices(Context<T>* context, int lod, const char* meshName, bool withRigid);

    //! Evaluates the gui controls
    DiffData<T> EvaluateGuiControls(Context<T>* context) const;

    //! Set the RigLogic and geometry (required)
    void SetRig(std::shared_ptr<const Rig<T>> rig);

    //! Set the RigLogicSolveControls (optional - set to nullptr to use full riglogic)
    void SetRigLogicSolveControls(std::shared_ptr<const RigLogicSolveControls<T>> rigLogicSolveControls);

    //! Set the current rigid transformation
    void SetRigidTransformation(const Affine<T, 3, 3>& affine);

    //! Returns the current rigid transformation
    Affine<T, 3, 3> RigidTransformation() const;

    //! Returns the deformed vertex positions for mesh @p meshIndex
    Eigen::Matrix<T, 3, -1> DeformedVertices(int meshIndex);

    //! @returns the current Gui controls
    Eigen::VectorX<T> GuiControls() const;

    //! Sets the Gui controls
    void SetGuiControls(const Eigen::VectorX<T>& guiControls);

    //! @returns the current solve control values
    const Eigen::VectorX<T>& SolveControls() const;

    //! Set the solve control values
    void SetSolveControls(const Eigen::VectorX<T>& controls);

    //! @returns the names of the solve controls
    const std::vector<std::string>& SolveControlNames() const;

    //! @returns the ranges for eadch solve control
    const Eigen::Matrix<T, 2, -1>& SolveControlRanges() const;

    //! @returns which controls are optimized
    const std::vector<bool>& SolveControlsToOptimize() const;

    //! Sets which controls are optimized
    void SetSolveControlsToOptimize(const std::vector<bool>& controlsToOptimize);

    //! @returns a pointer to the underlying solve control variable
    BoundedVectorVariable<T>* SolveControlVariable();

    //! @returns the mesh index for the mesh with name @p name
    int MeshIndex(const char* meshName) const;

    //! @returns the mesh index of the left eye mesh
    int LeftEyeMeshIndex() const;

    //! @returns the mesh index of the right eye mesh
    int RightEyeMeshIndex() const;

    //! @return the mesh index of the teeth mesh
    int TeethMeshIndex() const;

    /**
     * @returns the symmetric controls that are be enforced as model constraints e.g. CTRL_L_mouth_funnelU.ty and CTRL_R_mouth_funnelU.ty should be activated in the same way.
     *          This represents a L2 constraints of the form w * || rigControl[id1] - rigControl[id2] ||_2^2
     */
    const std::vector<std::tuple<std::string, std::string, T>>& SymmetricControls() const;

    //! Sets symmetric controls. @see SymmetricControls().
    void SetSymmetricControls(const std::vector<std::tuple<std::string, std::string, T>>& symmetricControls);

private:
    struct Private;
    epic::carbon::Pimpl<Private> m;
};


} // namespace epic::nls
