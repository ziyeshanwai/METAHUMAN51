// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/Common.h>
#include <carbon/io/JsonIO.h>
#include <carbon/io/Utils.h>
#include <nls/math/Math.h>
#include <nls/serialization/ObjFileFormat.h>
#include <nrr/volumetric/VolumetricFaceModel.h>

#include <filesystem>

namespace epic::nls {

template<class T>
bool VolumetricFaceModel<T>::Load(const std::string& directory) {

    bool success = true;

    const std::string tetsFilename = directory + "/flesh";
    const std::string skinFilename = directory + "/skin.obj";
    const std::string fleshFilename = directory + "/flesh.obj";
    const std::string craniumFilename = directory + "/cranium_with_teeth.obj";
    const std::string mandibleFilename = directory + "/mandible_with_teeth.obj";
    const std::string teethFilename = directory + "/teeth.obj";
    const std::string embeddingFilename = directory + "/flesh_surface_embedding.json";

    const std::string skinFleshMappingFilename = directory + "/skin_flesh_mapping.json";
    const std::string craniumFleshMappingFilename = directory + "/cranium_with_teeth_flesh_mapping.json";
    const std::string mandibleFleshMappingFilename = directory + "/mandible_with_teeth_flesh_mapping.json";

    const std::string teethRigidityMaskFilename = directory + "/teeth_rigidity_mask.json";

    auto readMesh = [](const std::string& filename, Mesh<T>& mesh) -> bool {
            if (ObjFileReader<T>().readObj(filename, mesh)) {
                return true;
            } else {
                LOG_ERROR("failed to load obj {}", filename);
                return false;
            }
        };

    // load meshes
    success &= readMesh(skinFilename, m_skinMesh);
    success &= readMesh(fleshFilename, m_fleshMesh);
    success &= readMesh(craniumFilename, m_craniumMesh);
    success &= readMesh(mandibleFilename, m_mandibleMesh);
    success &= readMesh(teethFilename, m_teethMesh);

    // load tet mesh
    m_tetMesh.LoadFromNPY(tetsFilename + "_verts.npy", tetsFilename + "_tets.npy");

    // load flesh embedding in tet mesh
    m_embedding.Deserialize(embeddingFilename);

    // load mesh-flesh pairs
    m_skinFleshMapping = epic::carbon::ReadJson(ReadFile(skinFleshMappingFilename)).Get<std::vector<std::pair<int,int>>>();
    m_craniumFleshMapping = epic::carbon::ReadJson(ReadFile(craniumFleshMappingFilename)).Get<std::vector<std::pair<int,int>>>();
    m_mandibleFleshMapping = epic::carbon::ReadJson(ReadFile(mandibleFleshMappingFilename)).Get<std::vector<std::pair<int,int>>>();

    epic::carbon::JsonElement json = epic::carbon::ReadJson(ReadFile(teethRigidityMaskFilename));
    m_lowerTeethRigidityMask.Load(json, "lower_teeth_rigidity_mask", m_teethMesh.NumVertices());
    m_upperTeethRigidityMask.Load(json, "upper_teeth_rigidity_mask", m_teethMesh.NumVertices());

    return success;
}

template<class T>
bool VolumetricFaceModel<T>::Save(const std::string& directory) const {

    if (std::filesystem::exists(directory)) {
        LOG_ERROR("directory \"{}\" already exists, cannot save flesh model to an existing directory", directory);
        return false;
    }

    const auto parentDirectory = std::filesystem::path(directory).parent_path();
    if (!std::filesystem::exists(parentDirectory)) {
        LOG_ERROR("Parent directory \"{}\" of output directory needs to exists.", parentDirectory.string());
        return false;
    }
    std::filesystem::create_directories(directory);

    ObjFileWriter<T>().writeObj(GetSkinMesh(), directory + "/skin.obj");
    ObjFileWriter<T>().writeObj(GetFleshMesh(), directory + "/flesh.obj");
    ObjFileWriter<T>().writeObj(GetCraniumMesh(), directory + "/cranium_with_teeth.obj");
    ObjFileWriter<T>().writeObj(GetMandibleMesh(), directory + "/mandible_with_teeth.obj");
    ObjFileWriter<T>().writeObj(GetTeethMesh(), directory + "/teeth.obj");

    GetTetMesh().SaveToNPY(directory + "/flesh_verts.npy", directory + "/flesh_tets.npy");

    m_embedding.Serialize(directory + "/flesh_surface_embedding.json");

    epic::carbon::WriteFile(directory + "/skin_flesh_mapping.json", epic::carbon::WriteJson(epic::carbon::JsonElement(m_skinFleshMapping)));
    epic::carbon::WriteFile(directory + "/cranium_with_teeth_flesh_mapping.json", epic::carbon::WriteJson(epic::carbon::JsonElement(m_craniumFleshMapping)));
    epic::carbon::WriteFile(directory + "/mandible_with_teeth_flesh_mapping.json", epic::carbon::WriteJson(epic::carbon::JsonElement(m_mandibleFleshMapping)));

    epic::carbon::JsonElement json(epic::carbon::JsonElement::JsonType::Object);
    m_lowerTeethRigidityMask.Save(json, "lower_teeth_rigidity_mask");
    m_upperTeethRigidityMask.Save(json, "upper_teeth_rigidity_mask");
    WriteFile(directory + "/teeth_rigidity_mask.json", epic::carbon::WriteJson(json));

    return true;
}

template<class T>
void VolumetricFaceModel<T>::SetSkinMeshVertices(const Eigen::Matrix<T, 3, -1>& skinVertices) {
    CARBON_ASSERT(static_cast<int>(skinVertices.cols()) == m_skinMesh.NumVertices(),
                    "number of skin vertices is incorrect");
    m_skinMesh.SetVertices(skinVertices);
}

template<class T>
void VolumetricFaceModel<T>::SetFleshMeshVertices(const Eigen::Matrix<T, 3, -1>& fleshVertices) {
    CARBON_ASSERT(static_cast<int>(fleshVertices.cols()) == m_fleshMesh.NumVertices(),
                    "number of flesh vertices is incorrect");
    m_fleshMesh.SetVertices(fleshVertices);
}

template<class T>
void VolumetricFaceModel<T>::SetCraniumMeshVertices(const Eigen::Matrix<T, 3, -1>& craniumVertices) {
    CARBON_ASSERT(static_cast<int>(craniumVertices.cols()) == m_craniumMesh.NumVertices(),
                    "number of cranium vertices is incorrect");
    m_craniumMesh.SetVertices(craniumVertices);
}

template<class T>
void VolumetricFaceModel<T>::SetMandibleMeshVertices(const Eigen::Matrix<T, 3, -1>& mandibleVertices) {
    CARBON_ASSERT(static_cast<int>(mandibleVertices.cols()) == m_mandibleMesh.NumVertices(),
                    "number of mandible vertices is incorrect");
    m_mandibleMesh.SetVertices(mandibleVertices);
}

template<class T>
void VolumetricFaceModel<T>::SetTeethMeshVertices(const Eigen::Matrix<T, 3, -1>& teethVertices) {
    CARBON_ASSERT(static_cast<int>(teethVertices.cols()) == m_teethMesh.NumVertices(),
                    "number of teeth vertices is incorrect");
    m_teethMesh.SetVertices(teethVertices);
}

template<class T>
void VolumetricFaceModel<T>::SetTetMeshVertices(const Eigen::Matrix<T, 3, -1>& tetVertices) {
    CARBON_ASSERT(static_cast<int>(tetVertices.cols()) == m_tetMesh.NumVertices(),
                    "number of tet vertices is incorrect");
    m_tetMesh.SetVertices(tetVertices);
}

template<class T>
void VolumetricFaceModel<T>::UpdateFleshMeshVerticesFromSkinCraniumAndMandible() {
    // copy the skin, cranium, and mandible vertices
    Eigen::Matrix<T, 3, -1> fleshVertices = GetFleshMesh().Vertices();
    for (const auto& [skinIndex, fleshIndex] : SkinFleshMapping()) {
        fleshVertices.col(fleshIndex) = GetSkinMesh().Vertices().col(skinIndex);
    }
    for (const auto& [craniumIndex, fleshIndex] : CraniumFleshMapping()) {
        fleshVertices.col(fleshIndex) = GetCraniumMesh().Vertices().col(craniumIndex);
    }
    for (const auto& [mandibleIndex, fleshIndex] : MandibleFleshMapping()) {
        fleshVertices.col(fleshIndex) = GetMandibleMesh().Vertices().col(mandibleIndex);
    }
    SetFleshMeshVertices(fleshVertices);
}

// explicitly instantiate VolumetricFaceModel for float and double
template class VolumetricFaceModel<float>;
template class VolumetricFaceModel<double>;

}  // namespace epic::nls
