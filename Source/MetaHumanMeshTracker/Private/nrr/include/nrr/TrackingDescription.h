// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/io/JsonIO.h>
#include <nls/utils/FileIO.h>

#include "DataDescription.h"
#include "TemplateDescription.h"

#include <string>

namespace epic {
namespace nls {

/**
 * Tracking description defines all the data necessary for tracking.
 *
 * 1. Path to Data Description (describes the multiview image data)
 * 2. Path to Template Description (describes the mesh)
 * 3. Path to DNA file
 */
class TrackingDescription {
public:
    TrackingDescription() = default;

    bool Load(const std::string& filename, const std::string& dataDescriptionFilename = "")
    {
        const std::string filedata = ReadFile(filename);
        const carbon::JsonElement j = carbon::ReadJson(filedata);

        const std::string descriptionDirectory = std::filesystem::absolute(std::filesystem::path(filename)).parent_path().string();

        auto makeAbsolute = [&](const std::string& filename) {
            if (filename.empty()) { return filename; }
            else if (std::filesystem::path(filename).is_relative()) {
                return descriptionDirectory + "/" + filename;
            } else {
                return filename;
            }
        };

        // load template description
        if (j.Contains("template_description")) {
            const std::string templateDescriptionFilename = makeAbsolute(j["template_description"].String());
            if (!m_templateDescription.Load(templateDescriptionFilename)) {
                LOG_ERROR("failed to load template description in json file {}", templateDescriptionFilename);
                return false;
            }
        } else {
            LOG_ERROR("no template description in json file {}", filename);
            return false;
        }

        // load data description
        if (!dataDescriptionFilename.empty()) {
            if (j.Contains("data_description")) {
                LOG_ERROR("tracking description contains a data description, but a data description is also passed an argument. Only one "
                          "method for passing a data description is supported.");
                return false;
            }
            if (!m_dataDescription.Load(dataDescriptionFilename)) {
                LOG_ERROR("failed to load data description in json file {}", dataDescriptionFilename);
                return false;
            }
        } else {
            if (j.Contains("data_description")) {
                const std::string dataDescriptionFilenameFromJson = makeAbsolute(j["data_description"].String());
                if (!m_dataDescription.Load(dataDescriptionFilenameFromJson)) {
                    LOG_ERROR("failed to load data description in json file {}", dataDescriptionFilenameFromJson);
                    return false;
                }
            } else {
                LOG_ERROR("no data description in json file {}", filename);
                return false;
            }
        }

        // load dna data
        if (j.Contains("dna")) {
            m_dnaFilenmame = makeAbsolute(j["dna"].String());
        } else {
            LOG_ERROR("no dna file in json file {}", filename);
            return false;
        }

        if (j.Contains("pca")) {
            m_pcaFilename = makeAbsolute(j["pca"].String());
        }

        if (j.Contains("pose_based_solvers")) {
            m_poseBasedSolverFilenames = j["pose_based_solvers"].Get<std::vector<std::string>>();
            for (auto& solverFilename : m_poseBasedSolverFilenames) {
                solverFilename = makeAbsolute(solverFilename);
            }
        }

        // load the brow initialization camera
        if (j.Contains("brow_initialization_camera")) {
            m_browInitializationCamera = j["brow_initialization_camera"].String();
        }
        if (j.Contains("brow_mesh_landmarks")) {
            m_browMeshLandmarksFilename = makeAbsolute(j["brow_mesh_landmarks"].String());
        }

        if (j.Contains("markers_mesh_landmarks")) {
            m_markersLandmarksFilename = makeAbsolute(j["markers_mesh_landmarks"].String());
        }

        // load configuration data
        if (j.Contains("configuration")) {
            m_config = makeAbsolute(j["configuration"].String());
        }
        else {
            LOG_WARNING("no config info in json file {}, using the default configuration", filename);
        }

        // load configuration on which controls are *not* being optimized (deprecated: this is now part of the configuration itself)
        if (j.Contains("controls")) {
            m_controlsConfiguration = makeAbsolute(j["controls"].String());
            LOG_WARNING("using a separate controls file is deprecated, instead it should be part of the configuration with key \"fixed_gui_controls\"");
        }

        // load the flow cameras
        if (j.Contains("flow_cameras")) {
            m_flowCameras = j["flow_cameras"].Get<std::vector<std::string>>();
        }

        // load the reconstruction pairs
        if (j.Contains("reconstructions")) {
            if (!j["reconstructions"].IsArray()) {
                CARBON_CRITICAL("reconstructions key should point to array of camera pairs");
            }
            m_reconstructionPairs = j["reconstructions"].Get<std::vector<std::pair<std::string, std::string>>>();
        }

        if (j.Contains("solver_definitions")) {
            m_solverDefinitionsFilename = makeAbsolute(j["solver_definitions"].String());
        }
        else if (j.Contains("solver definitions")) {
            LOG_WARNING("\"solver definitions\" is deprecated, use \"solver_definitions\" instead");
            m_solverDefinitionsFilename = makeAbsolute(j["solver definitions"].String());
        }

        if (j.Contains("uv_reference")) {
            m_uvReferenceFilename = makeAbsolute(j["uv_reference"].String());
        }
        if (j.Contains("scale_cameras")) {
            m_scaleCameras = j["scale_cameras"].Get<float>();
        }
        return true;
    }

    const TemplateDescription& GetTemplateDescription() const { return m_templateDescription; }

    const DataDescription& GetDataDescription() const { return m_dataDescription; }

    const std::string& DNAFilename() const { return m_dnaFilenmame; }

    const std::string& ConfigurationPath() const { return m_config; }

    const std::string& ControlsConfigurationPath() const { return m_controlsConfiguration; }

    const std::vector<std::string>& FlowCameras() const { return m_flowCameras; }

    //! @returns the name of the camera for which we can initialize the meshlandamrks curves for the eye brows.
    const std::string& BrowInitializationCamera() const { return m_browInitializationCamera; }

    //! @returns the filename of the file that contains the meshlandmarks curves for the eye brows.
    const std::string& BrowMeshLandmarksFilename() const { return m_browMeshLandmarksFilename; }

    //! @returns the filename of the file that contains meshlandmarks for markers.
    const std::string& MarkersMeshLandmarksFilename() const { return m_markersLandmarksFilename; }

    const std::vector<std::pair<std::string, std::string>>& ReconstructionPairs() const { return m_reconstructionPairs; }

    const std::string& SolverDefinitionsFilename() const { return m_solverDefinitionsFilename; }

    const std::string& UVReferenceFilename() const { return m_uvReferenceFilename; }

    const std::string& PCAFilename() const { return m_pcaFilename; }

    const std::vector<std::string>& PoseBaseSolverFilenames() const { return m_poseBasedSolverFilenames; }

    float GetScaleCameras() const { return m_scaleCameras; }

private:
    // defines how to scale data to have the same scale as the rig.
    float m_scaleCameras = 1.0f;

    TemplateDescription m_templateDescription;
    DataDescription m_dataDescription;
    std::string m_dnaFilenmame;
    // filename for the tracking configuration including tracking weights, landmark/curve weights, symmetries, and fixed controls
    std::string m_config;
    // filename for json file containing which controls are fixed i.e. *not* being optimized (deprecated: this is now part of the configuration itself)
    std::string m_controlsConfiguration;
    std::vector<std::string> m_flowCameras;
    std::string m_browInitializationCamera;
    std::string m_browMeshLandmarksFilename;
    std::string m_markersLandmarksFilename;
    std::vector<std::pair<std::string, std::string>> m_reconstructionPairs;

    std::string m_solverDefinitionsFilename;
    std::string m_uvReferenceFilename;

    //! filename for a PCA rig version
    std::string m_pcaFilename;
    std::vector<std::string> m_poseBasedSolverFilenames;

};


} // namespace nls
} //namespace epic
