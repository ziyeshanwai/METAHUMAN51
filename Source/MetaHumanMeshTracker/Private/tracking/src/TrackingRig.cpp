// Copyright Epic Games, Inc. All Rights Reserved.

#include <tracking/TrackingRig.h>
#include <carbon/io/JsonIO.h>
#include <nls/utils/FileIO.h>
#include <pma/PolyAllocator.h>
#include <carbon/common/External.h>

#define FULL_SOLVESET_NAME "All"

namespace epic::nls {

    using namespace pma;

template <class T>
TrackingRig<T>::TrackingRig()
{
}


template <class T>
TrackingRig<T>::~TrackingRig()
{
}

template <class T>
bool TrackingRig<T>::VerifyTopology(const Mesh<T>& topology) const
{
    if (GetBaseMesh().NumVertices() != topology.NumVertices()) {
        LOG_ERROR("tracking rig mesh does not have same number of vertices as the requested topology: {} vs {}", GetBaseMesh().NumVertices(), topology.NumVertices());
        return false;
    }
    if ((GetBaseMesh().NumQuads() != topology.NumQuads()) || (GetBaseMesh().Quads() != topology.Quads())) {
        LOG_ERROR("tracking rig has different quad topology compared to the requested topology: {} vs {}", GetBaseMesh().NumQuads(), topology.NumQuads());
        return false;
    }
    if ((GetBaseMesh().NumTriangles() != topology.NumTriangles()) || (GetBaseMesh().Triangles() != topology.Triangles())) {
        LOG_ERROR("tracking rig has different triangle topology compared to the requested topology: {} vs {}", GetBaseMesh().NumTriangles(), topology.NumTriangles());
        return false;
    }
    if (GetBaseMesh().NumTexQuads() != topology.NumTexQuads()) {
        LOG_ERROR("tracking rig has different texture quad topology compared to the requested topology: {} vs {}", GetBaseMesh().NumTexQuads(), topology.NumTexQuads());
        return false;
    }
    if (GetBaseMesh().NumTexTriangles() != topology.NumTexTriangles()) {
        LOG_ERROR("tracking rig has different texture triangle topology compared to the requested topology: {} vs {}", GetBaseMesh().NumTexTriangles(), topology.NumTexTriangles());
        return false;
    }
    if (GetBaseMesh().NumTexcoords() != topology.NumTexcoords()) {
        LOG_ERROR("tracking rig has different texcoordinates compared to the requested topology: {} vs {}", GetBaseMesh().NumTexcoords(), topology.NumTexcoords());
        return false;
    }
    // texture coordinates may have a different order, so we need to compare the coordinates per texture quad/triangle
    T maxUVDiff = 0;
    for (int i = 0; i < GetBaseMesh().NumTexQuads(); ++i) {
        Eigen::Matrix<T, 2, 4> uvs1, uvs2;
        for (int k = 0; k < 4; ++k) uvs1.col(k) = GetBaseMesh().Texcoords().col(GetBaseMesh().TexQuads()(k, i));
        for (int k = 0; k < 4; ++k) uvs2.col(k) = topology.Texcoords().col(topology.TexQuads()(k, i));
        maxUVDiff = std::max<T>(maxUVDiff, (uvs1 - uvs2).cwiseAbs().maxCoeff());
    }
    for (int i = 0; i < GetBaseMesh().NumTexTriangles(); ++i) {
        Eigen::Matrix<T, 2, 3> uvs1, uvs2;
        for (int k = 0; k < 3; ++k) uvs1.col(k) = GetBaseMesh().Texcoords().col(GetBaseMesh().TexTriangles()(k, i));
        for (int k = 0; k < 3; ++k) uvs2.col(k) = topology.Texcoords().col(topology.TexTriangles()(k, i));
        maxUVDiff = std::max<T>(maxUVDiff, (uvs1 - uvs2).cwiseAbs().maxCoeff());
    }
    if (maxUVDiff > T(1e-6)) {
        LOG_WARNING("tracking rig has different texcoordinate values compared to the requested topology: difference {}", maxUVDiff);
    }
    return true;
}

template <class T>
bool TrackingRig<T>::LoadRig(const std::string& DNAFilename, const std::string& SolveControlDefinitionFilename)
{
    auto memoryResource = MEM_RESOURCE;
    PolyAllocator<RigLogic<T>> rPolyAlloc{ memoryResource };
    std::shared_ptr<Rig<T>> rig = std::allocate_shared<Rig<T>>(rPolyAlloc);
    if (!rig->LoadRig(DNAFilename)) {
        LOG_ERROR("failed to load rig from dna file {}", DNAFilename);
        return false;
    }
    m_rig = rig;
    m_defaultGuiControlValues = Eigen::VectorX<T>::Zero(m_rig->GetRigLogic()->NumGUIControls());
    return LoadSolveControlDefinition(SolveControlDefinitionFilename);
}

template <class T>
bool TrackingRig<T>::LoadRig(dna::StreamReader* DNAStream, const std::string& SolveControlDefinitionFilename)
{
    auto memoryResource = MEM_RESOURCE;
    PolyAllocator<RigLogic<T>> rPolyAlloc{ memoryResource };
    std::shared_ptr<Rig<T>> rig = std::allocate_shared<Rig<T>>(rPolyAlloc);
    if (!rig->LoadRig(DNAStream)) {
        LOG_ERROR("failed to load rig from dnastream");
        return false;
    }
    m_rig = rig;
    m_defaultGuiControlValues = Eigen::VectorX<T>::Zero(m_rig->GetRigLogic()->NumGUIControls());
    return LoadSolveControlDefinition(SolveControlDefinitionFilename);
}

template <class T>
bool TrackingRig<T>::LoadSolveControlDefinition(const std::string& SolveControlDefinitionFilename)
{
    // load rig solver definition
    if (!SolveControlDefinitionFilename.empty()) {
        const carbon::JsonElement rigSolverDefinition = carbon::ReadJson(ReadFile(SolveControlDefinitionFilename));
        if (rigSolverDefinition.Contains("Defaults")) {
            for (const auto& [guiControlName, value] : rigSolverDefinition["Defaults"].Map()) {
                auto it = std::find(m_rig->GetRigLogic()->GuiControlNames().begin(), m_rig->GetRigLogic()->GuiControlNames().end(), guiControlName);
                if (it != m_rig->GetRigLogic()->GuiControlNames().end()) {
                    const size_t guiControlIndex = std::distance(m_rig->GetRigLogic()->GuiControlNames().begin(), it);
                    m_defaultGuiControlValues[guiControlIndex] = value.template Get<T>();
                } else {
                    CARBON_CRITICAL("rig does not contain a gui control of name {}", guiControlName);
                }
            }
        }

        if (rigSolverDefinition.Contains("Solver Sets")) {
            const carbon::JsonElement& rigSolverSetsJson = rigSolverDefinition["Solver Sets"];
            auto memoryResource = MEM_RESOURCE;
            PolyAllocator<RigLogicSolveControls<T>> polyAlloc{ memoryResource };
            for (const auto& rigSolverSetJson : rigSolverSetsJson.Array()) {
                auto rigLogicSolveControls = std::allocate_shared<RigLogicSolveControls<T>>(polyAlloc);
                rigLogicSolveControls->Init(*m_rig->GetRigLogic(), rigSolverSetJson);
                m_solveControlSets.push_back(SolveControlSet{rigLogicSolveControls, Eigen::VectorX<T>(), {}});
            }
        }
    }

    m_solveControlSets.push_back(SolveControlSet()); // the empty set is all controls

    for (auto& solveControlSet : m_solveControlSets) {
        if (solveControlSet.rigLogicSolveControls) {
            std::vector<int> inconsistentSolveControls;
            solveControlSet.defaultSolveControlValues = solveControlSet.rigLogicSolveControls->SolveControlsFromGuiControls(m_defaultGuiControlValues, inconsistentSolveControls);
            solveControlSet.affectedGuiControls = solveControlSet.rigLogicSolveControls->UsedGuiControlsPerSolveControl();
        } else {
            solveControlSet.defaultSolveControlValues = m_defaultGuiControlValues;
            solveControlSet.affectedGuiControls.clear();
            for (int i = 0; i < static_cast<int>(m_defaultGuiControlValues.size()); ++i) {
                solveControlSet.affectedGuiControls.push_back({i}); // solve controls and gui controls are the same
            }
        }
    }

    for (const auto& solveControlSet : m_solveControlSets) {
        const size_t numSolveControls = size_t(solveControlSet.defaultSolveControlValues.size());
        if (solveControlSet.rigLogicSolveControls) {
            CARBON_ASSERT(size_t(solveControlSet.rigLogicSolveControls->NumSolveControls()) == numSolveControls, "number of solve controls is incorrect");
        }
        CARBON_ASSERT(numSolveControls == solveControlSet.affectedGuiControls.size(), "number of solve controls does not match the number of affected gui controls");
    }

    return true;
}

template <class T>
const std::vector<std::string>& TrackingRig<T>::GetGuiControlNames() const
{
    if (!m_rig->GetRigLogic()) {
        CARBON_CRITICAL("rig is not valid");
    }
    return m_rig->GetRigLogic()->GuiControlNames();
}


template <class T>
const std::vector<std::string>& TrackingRig<T>::GetRawControlNames() const
{
    if (!m_rig->GetRigLogic()) {
        CARBON_CRITICAL("rig is not valid");
    }
    return m_rig->GetRigLogic()->RawControlNames();
}


template <class T>
size_t TrackingRig<T>::GetNumSolveControls(size_t solveControlSetIndex) const
{
    if (solveControlSetIndex >= m_solveControlSets.size()) {
        CARBON_CRITICAL("invalid solve control set index");
    }
    return size_t(m_solveControlSets[solveControlSetIndex].defaultSolveControlValues.size());
}

template <class T>
const std::shared_ptr<const RigLogicSolveControls<T>>& TrackingRig<T>::GetRigLogicSolveControls(size_t solveControlSetIndex) const
{
    if (solveControlSetIndex >= m_solveControlSets.size()) {
        CARBON_CRITICAL("invalid solve control set index");
    }
    return m_solveControlSets[solveControlSetIndex].rigLogicSolveControls;
}

template <class T>
const Eigen::VectorX<T>& TrackingRig<T>::GetDefaultSolveControlValues(size_t solveControlSetIndex) const
{
    if (solveControlSetIndex >= m_solveControlSets.size()) {
        CARBON_CRITICAL("invalid solve control set index");
    }
    return m_solveControlSets[solveControlSetIndex].defaultSolveControlValues;
}

template <class T>
const std::vector<std::vector<int>>& TrackingRig<T>::GetAffectedGuiControls(size_t solveControlSetIndex) const
{
    if (solveControlSetIndex >= m_solveControlSets.size()) {
        CARBON_CRITICAL("invalid solve control set index");
    }
    return m_solveControlSets[solveControlSetIndex].affectedGuiControls;
}

template <class T>
std::vector<std::string> TrackingRig<T>::GetSolveControlSetNames() const
{
    std::vector<std::string> solveControlSets;
    for (const auto& solveControlSet : m_solveControlSets) {
        if (solveControlSet.rigLogicSolveControls) {
            solveControlSets.push_back(solveControlSet.rigLogicSolveControls->Name());
        } else {
            solveControlSets.push_back(FULL_SOLVESET_NAME);
        }
    }

    return solveControlSets;
}

template <class T>
const std::vector<std::string>& TrackingRig<T>::GetRigSolveControlNames(size_t solveControlSetIndex) const
{
    if (solveControlSetIndex < m_solveControlSets.size()) {
        if (m_solveControlSets[solveControlSetIndex].rigLogicSolveControls) {
            return m_solveControlSets[solveControlSetIndex].rigLogicSolveControls->SolveControlNames();
        } else {
            return m_rig->GetRigLogic()->GuiControlNames();
        }
    } else {
        CARBON_CRITICAL("invalid solve control set index");
    }
}

template <class T>
const Eigen::Matrix<T, 2, -1>& TrackingRig<T>::GetRigSolveControlRanges(size_t solveControlSetIndex) const
{
    if (solveControlSetIndex < m_solveControlSets.size()) {
        if (m_solveControlSets[solveControlSetIndex].rigLogicSolveControls) {
            return m_solveControlSets[solveControlSetIndex].rigLogicSolveControls->SolveControlRanges();
        } else {
            return m_rig->GetRigLogic()->GuiControlRanges();
        }
    } else {
        CARBON_CRITICAL("invalid solve control set index");
    }
}

template <class T>
Eigen::Matrix<T, 3, -1> TrackingRig<T>::EvaluateVertices(const Eigen::VectorX<T>& guiControls, int lod, int meshIndex) const
{
    DiffData<T> rawControls = m_rig->GetRigLogic()->EvaluateRawControls(guiControls);
    DiffData<T> psd = m_rig->GetRigLogic()->EvaluatePSD(rawControls);
    DiffData<T> joints = m_rig->GetRigLogic()->EvaluateJoints(psd, lod);
    DiffDataAffine<T, 3, 3> rigid;
    return m_rig->GetRigGeometry()->EvaluateRigGeometry(rigid, joints, psd, lod, {meshIndex}).Vertices().front().Matrix();
}

// explicitly instantiate the TrackingRig classes
template class TrackingRig<float>;
template class TrackingRig<double>;

} // namespace epic::nls
