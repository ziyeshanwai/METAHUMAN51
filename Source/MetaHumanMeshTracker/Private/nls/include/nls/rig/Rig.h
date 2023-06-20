// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/rig/RigLogic.h>
#include <nls/rig/RigGeometry.h>

#include <string>
#include <vector>

namespace epic::nls {

/**
 * Class combining RigLogic, RigGeometry, and the default values.
 */
template <class T>
class Rig
{
public:
    Rig();
    ~Rig();

    /**
     * @brief Loads the rig from @p DNAFilename.
     *
     * @param DNAFilename The name of the DNA file.
     * @param withJointScaling If False, then any joint scaling that is part of the DNA is discarded. Currently only pupils have scaling information.
     * @return true   If DNA file could be read.
     * @return false  If any error occurred during reading.
     */
    bool LoadRig(const std::string& DNAFilename, bool withJointScaling = false);

    /**
     * @brief Loads the rig from @p DNAStream.
     *
     * @param DNAStream The DNA stream.
     * @param withJointScaling If False, then any joint scaling that is part of the DNA is discarded. Currently only pupils have scaling information.
     * @return true   If DNA file could be read.
     * @return false  If any error occurred during reading.
     */
    bool LoadRig(dna::StreamReader* DNAStream, bool withJointScaling = false);

    //! Tests whether the rig adheres to \p topology
    bool VerifyTopology(const Mesh<T>& topology) const;

    const std::shared_ptr<const RigLogic<T>>& GetRigLogic() const { return m_rigLogic; }
    const std::shared_ptr<const RigGeometry<T>>& GetRigGeometry() const { return m_rigGeometry; }
    const std::vector<std::string>& GetGuiControlNames() const;
    const std::vector<std::string>& GetRawControlNames() const;

    const Mesh<T>& GetBaseMesh() const { return m_baseMesh; }
    const Mesh<T>& GetBaseMeshTriangulated() const { return m_baseMeshTriangulated; }

    //! Evaluates the vertices of multiple meshes
    std::vector<Eigen::Matrix<T, 3, -1>> EvaluateVertices(const Eigen::VectorX<T>& guiControls, int lod, const std::vector<int>& meshIndices) const;

    //! Evaluates the vertices of multiple meshes
    void EvaluateVertices(const Eigen::VectorX<T>& guiControls, int lod, const std::vector<int>& meshIndices, typename RigGeometry<T>::State& state) const;

    //! Convert joint-based shapes into blendshape-only shapes. Does not create blendshapes for the eye geometry.
    void MakeBlendshapeOnly();

    //! Only keep the highest resolution mesh
    void ReduceToLOD0Only();

    //! Resample the rig such that a new vertex i will correspond to previous vertex newToOldMap[i].
    void Resample(const int meshIndex, const std::vector<int>& newToOldMap);

private:
    std::shared_ptr<const RigLogic<T>> m_rigLogic;
    std::shared_ptr<const RigGeometry<T>> m_rigGeometry;

    //! The base mesh (quads)
    Mesh<T> m_baseMesh;

    //! the base mesh (triangulated)
    Mesh<T> m_baseMeshTriangulated;
};

} // namespace epic::nls
