// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/rig/Rig.h>
#include <nls/rig/RigLogicDNAResource.h>
#include <nls/utils/FileIO.h>
#include <pma/PolyAllocator.h>
#include <carbon/common/External.h>

namespace epic::nls {

using namespace pma;

template <class T>
Rig<T>::Rig()
{
}


template <class T>
Rig<T>::~Rig()
{
}

template <class T>
bool Rig<T>::VerifyTopology(const Mesh<T>& topology) const
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
        LOG_ERROR("tracking rig has different texcoordinate values compared to the requested topology: difference {}", maxUVDiff);
        return false;
    }
    return true;
}

template <class T>
bool Rig<T>::LoadRig(const std::string& DNAFilename, bool withJointScaling)
{
    std::shared_ptr<const RigLogicDNAResource> dnaResource = RigLogicDNAResource::LoadDNA(DNAFilename, /*retain=*/false);
    if (!dnaResource) {
        LOG_ERROR("failed to open dnafile {}", DNAFilename);
        return false;
    }

    return LoadRig(dnaResource->Stream(), withJointScaling);
}

template <class T>
bool Rig<T>::LoadRig(dna::StreamReader* DNAStream, bool withJointScaling)
{
    auto memoryResource = MEM_RESOURCE;
    PolyAllocator<RigLogic<T>> rlPolyAlloc{ memoryResource };
    PolyAllocator<RigGeometry<T>> rgPolyAlloc{ memoryResource };
    std::shared_ptr<RigLogic<T>> rigLogic = std::allocate_shared<RigLogic<T>>(rlPolyAlloc);
    std::shared_ptr<RigGeometry<T>> rigGeometry = std::allocate_shared<RigGeometry<T>>(rgPolyAlloc);
    if (!rigLogic->Init(DNAStream, withJointScaling)) {
        LOG_ERROR("failed to load riglogic from dnastream");
        return false;
    }
    if (!rigGeometry->Init(DNAStream, rigLogic->WithJointScaling())) {
        LOG_ERROR("failed to load riggeometry from dnastream");
        return false;
    }
    m_rigLogic = rigLogic;
    m_rigGeometry = rigGeometry;

    // get the neutral mesh geometry
    m_baseMesh = m_rigGeometry->GetMesh(0);
    m_baseMeshTriangulated = m_baseMesh;
    m_baseMeshTriangulated.Triangulate();
    m_baseMeshTriangulated.CalculateVertexNormals();

    return true;
}

template <class T>
const std::vector<std::string>& Rig<T>::GetGuiControlNames() const
{
    if (!m_rigLogic) {
        CARBON_CRITICAL("rig is not valid");
    }
    return m_rigLogic->GuiControlNames();
}


template <class T>
const std::vector<std::string>& Rig<T>::GetRawControlNames() const
{
    if (!m_rigLogic) {
        CARBON_CRITICAL("rig is not valid");
    }
    return m_rigLogic->RawControlNames();
}

template <class T>
void Rig<T>::EvaluateVertices(const Eigen::VectorX<T>& guiControls, int lod, const std::vector<int>& meshIndices, typename RigGeometry<T>::State& state) const
{
    const DiffData<T> rawControls = m_rigLogic->EvaluateRawControls(guiControls);
    const DiffData<T> psd = m_rigLogic->EvaluatePSD(rawControls);
    const DiffData<T> joints = m_rigLogic->EvaluateJoints(psd, lod);
    const DiffDataAffine<T, 3, 3> rigid;
    m_rigGeometry->EvaluateRigGeometry(rigid, joints, psd, lod, meshIndices, state);
}

template <class T>
std::vector<Eigen::Matrix<T, 3, -1>> Rig<T>::EvaluateVertices(const Eigen::VectorX<T>& guiControls, int lod, const std::vector<int>& meshIndices) const
{
    typename RigGeometry<T>::State state;
    EvaluateVertices(guiControls, lod, meshIndices, state);

    std::vector<Eigen::Matrix<T, 3, -1>> vectorOfVertices;
    for (const auto result : state.Vertices()) {
        vectorOfVertices.emplace_back(result.Matrix());
    }
    return vectorOfVertices;
}

template <class T>
void Rig<T>::ReduceToLOD0Only()
{
    auto newRigLogic = m_rigLogic->Clone();
    newRigLogic->ReduceToLOD0Only();
    m_rigLogic = newRigLogic;

    auto newRigGeometry = m_rigGeometry->Clone();
    newRigGeometry->ReduceToLOD0Only();
    m_rigGeometry = newRigGeometry;
}

template <class T>
void Rig<T>::MakeBlendshapeOnly()
{
    auto newRigGeometry = m_rigGeometry->Clone();
    auto newRigLogic = m_rigLogic->Clone();
    newRigGeometry->MakeBlendshapeOnly(*newRigLogic);
    newRigGeometry->RemoveUnusedJoints(*newRigLogic);
    m_rigGeometry = newRigGeometry;
    m_rigLogic = newRigLogic;
}

template <class T>
void Rig<T>::Resample(const int meshIndex, const std::vector<int>& newToOldMap)
{
    auto newRigGeometry = m_rigGeometry->Clone();
    newRigGeometry->Resample(meshIndex, newToOldMap);
    m_rigGeometry = newRigGeometry;
    m_baseMesh.Resample(newToOldMap);
    m_baseMeshTriangulated.Resample(newToOldMap);
}

// explicitly instantiate the Rig classes
template class Rig<float>;
template class Rig<double>;

} // namespace epic::nls
