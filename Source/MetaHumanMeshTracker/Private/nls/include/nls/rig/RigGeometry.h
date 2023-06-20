// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Pimpl.h>
#include <nls/Context.h>
#include <nls/DiffDataMatrix.h>
#include <nls/geometry/Mesh.h>
#include <nls/math/Math.h>
#include <nls/rig/JointRig2.h>
#include <nls/rig/RigLogic.h>

#include <map>
namespace dna {
class StreamReader;
}


namespace epic::nls {

//! RigGeometry implements the rig geometry evaluation based on rig logic inputs.
template <class T>
class RigGeometry
{
public:
    class State;

public:
	RigGeometry();
    ~RigGeometry();
    RigGeometry(RigGeometry&&);
    RigGeometry(RigGeometry&) = delete;
    RigGeometry& operator=(RigGeometry&&);
    RigGeometry& operator=(const RigGeometry&) = delete;

    std::shared_ptr<RigGeometry> Clone() const;

    //! Initializes RigGeometry with the data from the dna::StreamReader
    bool Init(dna::StreamReader* reader, bool withJointScaling = false);

    //! Sets the joints and evaluates the mesh vertices for LOD @p lod and mesh indices @p meshIndices.
    State EvaluateRigGeometry(const DiffDataAffine<T, 3, 3>& diffRigid, const DiffData<T>& diffJoints, const DiffData<T>& diffPsd, int lod, const std::vector<int>& meshIndices) const;

    //! Sets the joints and evaluates the mesh vertices for LOD @p lod and mesh indices @p meshIndices.
    State& EvaluateRigGeometry(const DiffDataAffine<T, 3, 3>& diffRigid, const DiffData<T>& diffJoints, const DiffData<T>& diffPsd, int lod, const std::vector<int>& meshIndices, State& state) const;

    //! @return the @p meshIndex 'th mesh
    const Mesh<T>& GetMesh(int meshIndex) const;

    //! @return the name of the @p meshIndex 'th mesh
    const std::string& GetMeshName(int meshIndex) const;

    //! @return the index of the mesh with name @p meshName
    int GetMeshIndex(const std::string& meshName) const;

    //! @return the number of meshes
    int NumMeshes() const;

    //! @return all mesh indices that are part of @p lod
    const std::vector<int>& GetMeshIndicesForLOD(int lod) const;

    //! @return the joint indices that are used on LOD @p lod
    const std::vector<int>& JointIndicesForLOD(int lod) const;

    //! @return the underlying joint rig
    const JointRig2<T>& GetJointRig() const;

    //! @return the current bind matrix of the joint rig.
    Eigen::Matrix<T, 4, 4> GetBindMatrix(int jointIndex) const;

    /**
     * @brief Makes a blendshape only rig. However, it keeps the eyes as joints.
     */
    void MakeBlendshapeOnly(const RigLogic<T>& rigLogic);

    //! Removes all unused joints from both RigGeometry and RigLogic.
    void RemoveUnusedJoints(RigLogic<T>& rigLogic);

    //! Remove all LODs besides the highest.
    void ReduceToLOD0Only();

    //! Update the @p meshIndex 'th mesh vertices
    void SetMesh(int meshIndex, const Eigen::Matrix<T, 3, -1>& vertices);

    //! Assembles the mesh at index @p meshIndex stored within dna::StreamReader
    static Mesh<T> ReadMesh(dna::StreamReader* reader, int meshIndex);

    //! Resample the rig geometry such that new vertex i will correspond to previous vertex newToOldMap[i].
    void Resample(int meshIndex, const std::vector<int>& newToOldMap);

    //! @return the index of the head mesh
    int HeadMeshIndex(int lod) const;

    //! @return the index of the teeth mesh
    int TeethMeshIndex(int lod) const;

    //! @return the index of the left eye mesh
    int EyeLeftMeshIndex(int lod) const;

    //! @return the index of the right eye mesh
    int EyeRightMeshIndex(int lod) const;

private:
    //! Evaluates the joint deltas and stores it in @p state.
    void EvaluateJointDeltas(const DiffDataAffine<T, 3, 3>& diffRigid, const DiffData<T>& diffJoints, State& state) const;

    //! Evaluates the joint deltas and stores it in @p state.
    void EvaluateJointDeltasWithoutJacobians(const DiffDataAffine<T, 3, 3>& diffRigid, const DiffData<T>& diffJoints, State& state) const;

    /**
     * Evaluates the mesh vertices of for LOD @p lod and mesh index @p meshIndex.
     * @param diffPsd          The psd coefficients. Typically the output from RigLogic::EvaluatePsd()
     * @param lod              The lod for which to evalute the vertices.
     * @param meshIndex        Which mesh to evaluate.
     * @return the evaluated deformed mesh vertices. The output contains Jacobians if either the internal rig has the jacobians set, or if @diffBlendshapes contains the Jacobian.
     */
    void EvaluateBlendshapes(const DiffData<T>& diffPsd, int lod, int meshIndex, State& state) const;

    //! evaluates the skinning for geometry @p geometryName with state as input (evaluated blendshape vertices) and output (final vertices).
    void EvaluateSkinningWithoutJacobians(int meshIndex, State& state) const;

    void EvaluateSkinningWithJacobians(int meshIndex, State& state) const;

private:
    struct Private;
    //epic::carbon::Pimpl<Private> m;
    std::unique_ptr<Private> m;
};


//! Rig geometry state containing results of a specific evaluation
template <class T>
class RigGeometry<T>::State
{
public:
    State();
    ~State();
    State(State&&);
    State& operator=(State&&);
    State(const State&) = delete;
    State& operator=(const State&) = delete;

    //! @return the evaluated vertices. See RigGeometry<T>::EvaluateRigGeometry
    const std::vector<DiffDataMatrix<T, 3, -1>>& Vertices() const;

    //! @return the mesh indices that were passed to RigGeometry<T>::EvaluateRigGeometry for evalution
    const std::vector<int>& MeshIndices() const;

    //! @return the current world matrix of the joint rig.
    Eigen::Matrix<T, 4, 4> GetWorldMatrix(int jointIndex) const;

private:
    struct Private;
    std::unique_ptr<Private> m;

    friend class RigGeometry<T>;
};

} // namespace epic::nls
