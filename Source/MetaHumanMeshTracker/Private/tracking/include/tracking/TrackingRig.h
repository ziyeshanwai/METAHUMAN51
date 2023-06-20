// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/rig/Rig.h>
#include <nls/rig/RigLogicSolveControls.h>

#include <string>
#include <vector>

namespace pma
{
    class MemoryResource;
}
namespace epic {
namespace nls {

/**
 * Class combining RigLogic, RigGeometry, and the default values.
 */
template <class T>
class TrackingRig
{
public:
    TrackingRig();
    ~TrackingRig();

    bool LoadRig(const std::string& DNAFilename, const std::string& SolveControlDefinitionFilename);
    bool LoadRig(dna::StreamReader* DNAStream, const std::string& SolveControlDefinitionFilename);

    //! Tests whether the rig adheres to \p topology
    bool VerifyTopology(const Mesh<T>& topology) const;

    const std::shared_ptr<const Rig<T>> GetRig() const { return m_rig; }
    const std::shared_ptr<const RigLogic<T>> GetRigLogic() const { return m_rig->GetRigLogic(); }
    const std::shared_ptr<const RigGeometry<T>> GetRigGeometry() const { return m_rig->GetRigGeometry(); }
    const Eigen::VectorX<T>& GetDefaultGuiControlValues() const { return m_defaultGuiControlValues; }
    const std::vector<std::string>& GetGuiControlNames() const;
    const std::vector<std::string>& GetRawControlNames() const;

    const Mesh<T>& GetBaseMesh() const { return m_rig->GetBaseMesh(); }
    const Mesh<T>& GetBaseMeshTriangulated() const { return m_rig->GetBaseMeshTriangulated(); }

    //! @returns the number of solve control sets (at least one, the last one is all gui controls)
    size_t GetNumSolveControlSets() const { return m_solveControlSets.size(); }

    size_t GetNumSolveControls(size_t solveControlSetIndex) const;
    const std::shared_ptr<const RigLogicSolveControls<T>>& GetRigLogicSolveControls(size_t solveControlSetIndex) const;
    const Eigen::VectorX<T>& GetDefaultSolveControlValues(size_t solveControlSetIndex) const;
    const std::vector<std::vector<int>>& GetAffectedGuiControls(size_t solveControlSetIndex) const;

    //! @returns the names of the control sets
    std::vector<std::string> GetSolveControlSetNames() const;

    //! @returns the names of the riglogic controls for set @p solveControlSetIndex
    const std::vector<std::string>& GetRigSolveControlNames(size_t solveControlSetIndex) const;

    //! @returns the range of the riglogic solve control values
    const Eigen::Matrix<T, 2, -1>& GetRigSolveControlRanges(size_t solveControlSetIndex) const;

    //! The last control set is the Gui controls
    bool IsGuiControls(size_t solveControlSetIndex) const { return (solveControlSetIndex == m_solveControlSets.size() - 1); }

    //! Convenience function to evaluate the rig.
    Eigen::Matrix<T, 3, -1> EvaluateVertices(const Eigen::VectorX<T>& guiControls, int lod, int meshIndex) const;

private:
    bool LoadSolveControlDefinition(const std::string& SolveControlDefinitionFilename);

private:
    std::shared_ptr<const Rig<T>> m_rig;
    Eigen::VectorX<T> m_defaultGuiControlValues;

    struct SolveControlSet {
        //! if empty then it refers to the full rig
        std::shared_ptr<const RigLogicSolveControls<T>> rigLogicSolveControls;

        //! the default values for the solve controls
        Eigen::VectorX<T> defaultSolveControlValues;

        //! per solve control a vector which gui controls are affected
        std::vector<std::vector<int>> affectedGuiControls;
    };
    //! rig logic solve control sets
    std::vector<SolveControlSet> m_solveControlSets;
};

} // namespace nls
} //namespace epic
