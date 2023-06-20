// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/io/JsonIO.h>
#include <nls/geometry/Mesh.h>
#include <nls/serialization/EigenSerialization.h>
#include <nls/serialization/ObjFileFormat.h>
#include <nls/utils/FileIO.h>
#include <nrr/MeshLandmarks.h>
#include <nrr/SymmetryMapping.h>
#include <nrr/VertexWeights.h>

#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace epic {
namespace nls {

/**
 * Class containing all template face specific data such as mesh symmetry information, landmark positions on the mesh, weight masks etc.
 * The class does not contain any geometry/texture information for an individual.
 */
class TemplateDescription
{
public:
    TemplateDescription() = default;

    bool Load(const std::string& filename)
    {
        try {
            const std::string filedata = ReadFile(filename);
            const carbon::JsonElement j = carbon::ReadJson(filedata);

            const std::string descriptionDirectory = std::filesystem::absolute(std::filesystem::path(filename)).parent_path().string();

            auto makeAbsolute = [&](const std::string& filename) {
                if (std::filesystem::path(filename).is_relative()) {
                    return descriptionDirectory + "/" + filename;
                } else {
                    return filename;
                }
            };

            // parse topology
            if (j.Contains("topology")) {
                ObjFileReader<float> objReader;
                const std::string topologyFilename = makeAbsolute(j["topology"].String());
                if (!objReader.readObj(topologyFilename, m_topology)) {
                    CARBON_CRITICAL("failed to load topology from {}", topologyFilename);
                }
            } else {
                CARBON_CRITICAL("topology missing from template description");
            }

            // parse symmetry information
            if (j.Contains("symmetry")) {
                const std::string symmetryFile = makeAbsolute(j["symmetry"].String());
                if (!m_symmetryMapping.Load(symmetryFile)) {
                    CARBON_CRITICAL("failure to load symmetry information from {}", symmetryFile);
                }

                if (m_symmetryMapping.NumSymmetries() != m_topology.NumVertices()) {
                    LOG_INFO("symmetry size: {}", m_symmetryMapping.NumSymmetries());
                    LOG_INFO("topology size: {}", m_topology.NumVertices());
                    CARBON_CRITICAL("symmetry mapping does not have the same vertex count as the topology");
                }
            }

            // parse mask information
            if (j.Contains("masks")) {
                const std::string masksFile = makeAbsolute(j["masks"].String());
                const std::string masksFileData = ReadFile(masksFile);
                const carbon::JsonElement jMasks = carbon::ReadJson(masksFileData);

                for (const auto& [maskName, _] : jMasks.Map()) {
                    m_vertexWeights[maskName] = std::make_shared<VertexWeights<float>>(jMasks, maskName, m_topology.NumVertices());
                }
            }

            // parse mesh landmarks
            if (j.Contains("mesh_landmarks")) {
                const std::string meshLandmarksFile = makeAbsolute(j["mesh_landmarks"].String());
                if (!m_meshLandmarks.Load(meshLandmarksFile, m_topology)) {
                    LOG_ERROR("failure to load mesh landmarks from {}", meshLandmarksFile);
                    return false;
                }

                if (!m_eyeLeftMeshLandmarks.Load(meshLandmarksFile, Mesh<float>(), "eyeLeft_lod0_mesh")) {
                    LOG_ERROR("failure to load mesh landmarks for the left eye from {}", meshLandmarksFile);
                    return false;
                }

                if (!m_eyeRightMeshLandmarks.Load(meshLandmarksFile, Mesh<float>(), "eyeRight_lod0_mesh")) {
                    LOG_ERROR("failure to load mesh landmarks for the right eye from {}", meshLandmarksFile);
                    return false;
                }

                if (!m_teethMeshLandmarks.Load(meshLandmarksFile, Mesh<float>(), "teeth_lod0_mesh")) {
                    LOG_ERROR("failure to load mesh landmarks for the teeth from {}", meshLandmarksFile);
                    return false;
                }
            }

            if (j.Contains("identity_model")) {
                m_identityModelFilename = makeAbsolute(j["identity_model"].String());
            }

            if (j.Contains("assets")) {
                for (auto&& [key, elem] : j["assets"].Map()) {
                    const std::string topologyPath = makeAbsolute(j["assets"][key]["topology"].Get<std::string>());
                    const std::string maskPath = makeAbsolute(j["assets"][key]["masks"].Get<std::string>());

                    ObjFileReader<float> objReader;
                    if (!objReader.readObj(topologyPath, m_assetTopologies[key])) {
                        CARBON_CRITICAL("failed to load asset topology");
                    }

                    const std::string masksFileData = ReadFile(maskPath);
                    const carbon::JsonElement jMasks = carbon::ReadJson(masksFileData);

                    for (const auto& [maskName, _] : jMasks.Map()) {
                        m_assetVertexWeights[key][maskName] = std::make_shared<VertexWeights<float>>(jMasks, maskName, m_assetTopologies[key].NumVertices());
                    }
                }
            }

            if (j.Contains("texture")) {
                m_textureFilename = makeAbsolute(j["texture"].String());
            }
        }
        catch(std::exception& e)
        {
            LOG_ERROR("failure to load template description {}: {}", filename, e.what());
            return false;
        }

        return true;
    }

    const Mesh<float>& Topology() const { return m_topology; }

    const SymmetryMapping& GetSymmetryMapping() const { return m_symmetryMapping; }

    bool HasVertexWeights(const std::string& maskName) const { return m_vertexWeights.find(maskName) != m_vertexWeights.end(); }

    const VertexWeights<float>& GetVertexWeights(const std::string& maskName) const {
        auto it = m_vertexWeights.find(maskName);
        if (it != m_vertexWeights.end()) {
            return *(it->second);
        } else {
            throw std::runtime_error("no vertex weights of name " + maskName);
        }
    }

    const VertexWeights<float>& GetAssetVertexWeights(const std::string& assetName, const std::string maskName) const {
        auto it = m_assetVertexWeights.find(assetName);
        if (it != m_assetVertexWeights.end()) {
            auto it_ = it->second.find(maskName);
            if (it_ != it->second.end()) {
                return *(it_->second);
            }
            else {
                throw std::runtime_error("no vertex weights of name " + maskName);
            }
        }
        else {
            throw std::runtime_error("no masks for asset " + assetName);
        }
    }

    const Mesh<float>& GetAssetTopology(const std::string& assetName) const {
        auto it = m_assetTopologies.find(assetName);
        if (it != m_assetTopologies.end()) {
            return it->second;
        }
        else {
            throw std::runtime_error("no topology for asset " + assetName);
        }
    }

    const MeshLandmarks<float>& GetMeshLandmarks() const { return m_meshLandmarks; }
    const MeshLandmarks<float>& GetEyeLeftMeshLandmarks() const { return m_eyeLeftMeshLandmarks; }
    const MeshLandmarks<float>& GetEyeRightMeshLandmarks() const { return m_eyeRightMeshLandmarks; }
    const MeshLandmarks<float>& GetTeethMeshLandmarks() const { return m_teethMeshLandmarks; }

    const std::string& IdentityModelFilename() const { return m_identityModelFilename; }

    const std::string& TextureFilename() const { return m_textureFilename; }

private:
    // topology
    Mesh<float> m_topology;

    // symmetry mapping
    SymmetryMapping m_symmetryMapping;

    // named weight maps
    std::map<std::string, std::shared_ptr<const VertexWeights<float>>> m_vertexWeights;

    // mesh landmark information
    MeshLandmarks<float> m_meshLandmarks;

    // mesh landmark information for the left eye ball
    MeshLandmarks<float> m_eyeLeftMeshLandmarks;

    // mesh landmark information for the right eye ball
    MeshLandmarks<float> m_eyeRightMeshLandmarks;

    // mesh landmark information for the teeth
    MeshLandmarks<float> m_teethMeshLandmarks;

    // identity model
    std::string m_identityModelFilename;

    // asset meshes
    std::map<std::string, Mesh<float>> m_assetTopologies;

    // asset weight maps
    std::map<std::string, std::map<std::string, std::shared_ptr<const VertexWeights<float>>>> m_assetVertexWeights;

    // texture with the guide lines
    std::string m_textureFilename;
};


} // namespace nls
} //namespace epic
