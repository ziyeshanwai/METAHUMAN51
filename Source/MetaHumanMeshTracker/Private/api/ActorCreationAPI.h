// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Defs.h"

#include <MeshInputData.h>
#include <LandmarkData.h>
#include <OpenCVCamera.h>

#include <map>
#include <memory>
#include <stdint.h>
#include <string>
#include <vector>

namespace dna {
class StreamReader;
}

namespace titan {
namespace api {

enum class FittingMaskType {
    RIGID,
    NONRIGID,
    FINE,
    TEETH
};

class TITAN_API ActorCreationAPI {
public:
    ActorCreationAPI();
    ~ActorCreationAPI();
    ActorCreationAPI(ActorCreationAPI&&) = delete;
    ActorCreationAPI(const ActorCreationAPI&) = delete;
    ActorCreationAPI& operator=(ActorCreationAPI&&) = delete;
    ActorCreationAPI& operator=(const ActorCreationAPI&) = delete;

    /**
    * Initialize actor creation.
    * @param[in] InConfigurationDirectory  Directory containing a template_description.json and a dna_description.json file.
    * @returns True if initialization is successful, False otherwise.
    */
    bool Init(const std::string& InConfigurationDirectory);

    /**
    * Set the depth input data for one frame.
    * @param[in] frameNum  Frame to set the data.
    * @param[in] InLandmarksDataPerCamera  The distorted landmarks for each camera.
    * @param[in] InDepthMaps Depthmap data per (depthmap) camera.
    * @returns True if setting the data was successful.
    *
    * @warning Fails if scan data was set before.
    */
    bool SetDepthInputData(int32_t frameNum,
                           const std::map<std::string, const std::map<std::string, FaceTrackingLandmarkData>>& InLandmarksDataPerCamera,
                           const std::map<std::string, const float*>& InDepthMaps);

    /**
    * Set the scan input data.
    * @param[in] In3dLandmarksData  Landmarks in 3D.
    * @param[in] In2dLandmarksData  The distorted landmarks per camera view.
    * @param[in] InScanData  Input scan data with triangles stored as numTriangles x 3 and vertices as numVertices x 3 in column major format.
    * @returns True if setting the data was successful.
    *
    * @warning Fails if depthmap data was set before.
    */
    bool SetScanInputData(const std::map<std::string, const FaceTrackingLandmarkData>& In3dLandmarksData,
                          const std::map<std::string, const std::map<std::string, FaceTrackingLandmarkData>>& In2dLandmarksData,
                          const MeshInputData& InScanData);

    /**
     * Set up the cameras for fitting.
     * @param[in] InCameras  Input cameras for landmarks and depth projection.
     * @returns True if successful, false upon any error.
     */
    bool SetCameras(const std::map<std::string, OpenCVCamera>& InCameras);

     /**
     * Get regularization for non-rigid fitting.
     * @returns regularization multiplier for non-rigid fitting.
     */
    float GetModelRegularization();

     /**
     * Set regularization for non-rigid fitting.
     * @param[in] regularization multiplier
     */
    void SetModelRegularization(float regularization);

     /**
     * Get offset regularization for per-vertex fitting.
     * @returns regularization multiplier for per-vertex fitting.
     */
    float GetPerVertexOffsetRegularization();

     /**
     * Set regularization for per-vertex fitting.
     * @param[in] regularization multiplier
     */
    void SetPerVertexOffsetRegularization(float regularization);

     /**
     * Get Laplacian regularization for per-vertex fitting.
     * @returns regularization multiplier for per-vertex fitting.
     */
    float GetPerVertexLaplacianRegularization();

     /**
     * Set Laplacian regularization for per-vertex fitting.
     * @param[in] regularization multiplier
     */
    void SetPerVertexLaplacianRegularization(float regularization);

     /**
     * Get ICP minimum distance threshold (used for all types of fitting).
     * @returns threshold value.
     */
    float GetMinimumDistanceThreshold();

     /**
     * Set ICP minimum distance threshold (used for all types of fitting).
     * @param[in] threshold value
     */
    void SetMinimumDistanceThreshold(float threshold);

     /**
     * Get landmarks weight (used for all types of fitting).
     * @returns landmarks weight multiplier for all types of fitting.
     */
    float GetLandmarksWeight();

     /**
     * Set landmarks weight (used for all types of fitting).
     * @param[in] weight value
     */
    void SetLandmarksWeight(float weight);

     /**
     * Get landmarks weight for inner lips (used for all types of fitting).
     * @returns landmarks weight multiplier for all types of fitting.
     */
    float GetInnerLipsLandmarksWeight();

     /**
     * Set landmarks weight (used for all types of fitting).
     * @param[in] weight value
     */
    void SetInnerLipsLandmarksWeight(float weight);

     /**
     * Get collision weight for inner lips (used for all types of fitting).
     * @returns collision weight multiplier for all types of fitting.
     */
    float GetInnerLipsCollisionWeight();

     /**
     * Set collision weight (used for all types of fitting).
     * @param[in] collision weight value
     */
    void SetInnerLipsCollisionWeight(float weight);

     /**
     * Get vertex weights for mask type.
     * @param[out] InVertexWeights weight values
     * @param[in] InMaskType mask type
     * @returns True if was successful.
     */
    bool GetFittingMask(float* OutVertexWeights, FittingMaskType InMaskType);

     /**
     * Set vertex weights for mask type.
     * @param[in] InVertexWeights weight values
     * @param[in] InMaskType mask type
     * @returns True if was successful.
     */
    bool SetFittingMask(float* InVertexWeights, FittingMaskType InMaskType);

    /**
     * Fit identity given input data.
     * @param[out] OutVertexPositions  Vertex positions describing new identity as numVertices x 3 in column major format placed in *RIG COORDINATE SPACE*.
     * @param[out] OutStackedToScanTransforms  Stacked 4x4 transform matrices in column major format.
     * @param[out] OutStackedToScanScales  Stacked scale values. Scale is not a linear part of ToScanTransform, and needs to be applied after the transformation.
     * @param[in] InNumIters  Number of iterations of the optimization.
     * @returns True if fitting was successful.
     */
    bool FitRigid(float* OutVertexPositions, float* OutStackedToScanTransforms, float* OutStackedToScanScales, int32_t InNumIters = 3);

    /**
     * Fit identity given input data.
     * @param[out] OutVertexPositions  Vertex positions describing new identity as numVertices x 3 in column major format placed in *RIG COORDINATE SPACE*.
     * @param[out] OutStackedToScanTransforms  Stacked 4x4 transform matrices in column major format.
     * @param[out] OutStackedToScanScales  Stacked scale values. Scale is not a linear part of ToScanTransform, and needs to be applied after the transformation.
     * @param[in] InNumIters  Number of iterations of the optimization.
     * @returns True if fitting was successful.
     */
    bool FitNonRigid(float* OutVertexPositions, float* OutStackedToScanTransforms, float* OutStackedToScanScales, int32_t InNumIters = 3, const bool InAutoMode = true);

    /**
     * Fit identity given input data.
     * @param[out] OutVertexPositions  Vertex positions describing new identity as numVertices x 3 in column major format placed in *RIG COORDINATE SPACE*.
     * @param[out] OutStackedToScanTransforms  Stacked 4x4 transform matrices in column major format.
     * @param[out] OutStackedToScanScales  Stacked scale values. Scale is not a linear part of ToScanTransform, and needs to be applied after the transformation.
     * @param[in] InNumIters  Number of iterations of the optimization.
     * @param[in] InAutoMode  Enable auto mode where optimization will have iterations with decreasing regularization (InNumIters then have no effect).
     * @returns True if fitting was successful.
     */
    bool FitPerVertex(float* OutVertexPositions, float* OutStackedToScanTransforms, float* OutStackedToScanScales, int32_t InNumIters = 3);

    /**
     * Update teeth model and position in the rig given input data.
     * @param[in] InDnaStream  Input DNA to be used as deformable model to fit the expression.
     * @param[out] OutVertexPositions  Vertex positions describing updated teeth fitted against the rig.
     * @returns True if fitting was successful.
     */
    bool FitTeeth(dna::StreamReader* InDnaStream, float* OutVertexPositions);

    //! Resets all input data
    bool ResetInputData();

private:
    struct Private;
    Private* m;
    };
}
}
