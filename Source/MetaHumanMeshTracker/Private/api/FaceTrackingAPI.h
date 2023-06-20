// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Defs.h"
#include "OpenCVCamera.h"
#include "OpticalFlowData.h"
#include "LandmarkData.h"

#include <functional>
#include <memory>
#include <stdint.h>

namespace dna {
class StreamReader;
class StreamWriter;
}

namespace titan {
namespace api {

class TITAN_API FaceTrackingAPI {
public:
    FaceTrackingAPI();
    ~FaceTrackingAPI();
    FaceTrackingAPI(FaceTrackingAPI&&) = delete;
    FaceTrackingAPI(const FaceTrackingAPI&) = delete;
    FaceTrackingAPI& operator=(FaceTrackingAPI&&) = delete;
    FaceTrackingAPI& operator=(const FaceTrackingAPI&) = delete;

    /**
     * Initialize face tracking.
     * @param[in] InConfigurationDirectory  Directory containing a template_description.json and a configuration.json file.
     *                                      Optional: if there is also a solver_definition.json file in the directory, then a
     *                                      hierarchical solve is used during tracking.
     * @param[in] InOptFlowData Optical flow related data (whethter to use optical flow, forward / backward flow and confidence)
     * @note
     *     Using confidence will cause further slowdown as additional optical flow pass is executed so it is not recommended.
     * @param[in] bIsRealtime whether workflow is realtime.
     * @returns True if initialization is successful, False otherwise.
     */
    bool Init(const std::string& InConfigurationDirectory, OpticalFlowData InOptFlowData = {}, bool bIsRealtime = false);

    /**
     * Load the DNA file.
     *
     * @returns True if the DNA file could be loaded.
     */
    bool LoadDNA(const std::string& InDNAPath);
    bool LoadDNA(dna::StreamReader* InDNAStream);

    /*
    * Load the PCA rig from a dna::StreamReader
    * 
    * @param[in] InOutPCARigStream:  a stream object which may be used to load in the PCA rig
    * @returns True if the PCA rig could be loaded
    */
    bool LoadPCARig(dna::StreamReader* InOutPCARigStream);

    /*
    * Save the PCA rig to a dna::StreamWriter
    * 
    * @param[in] InOutPCARigStream:  a stream object which may be used to save out the PCA rig
    * @returns True if the PCA rig could be saved
    */
    bool SavePCARig(dna::StreamWriter* InOutPCARigStream);

    /*
    * Load the pose-based solvers from a list of filenames
    * 
    * @param[in] InPoseBasedSolverFilenames: the filenames of the pose-based solver(s) to load
    * @returns True if the pose-based solvers could be loaded
    */
    bool LoadPoseBasedSolvers(const std::vector<std::string>& InPoseBasedSolverFilenames);
   
    /*
    * Set the pose-based solvers as a memory buffer (this allows the data to be stored within a UE asset)
    * Requires pose-based solvers to be present in the object.
    * 
    * @param[out] OutMemoryBuffer:  a memory buffer which on return will contain the pose-based solver(s) data
    * @returns True if successful
    */
    bool GetPoseBasedSolvers(std::vector<char>& OutMemoryBuffer) const;

    /*
    * Set the pose-based solvers from a memory buffer (this allows the data to be set from within a UE asset)
    * 
    * @returns True if successful
    */
    bool SetPoseBasedSolvers(std::vector<char>& InMemoryBuffer);

    /* 
    * (Re)-train the PCA model and pose-based solvers for the current DNA.
    * Must already have set the DNA and existing pose-based solvers for this to work.
    * This does a fast re-train of the pose-based solvers based upon the best settings and animation training data in the
    * existing pose-based solvers.
    * 
    * @returns True if successful.
    * @param[in] InProgressCallback:  progress callback function which takes a float (which is fraction of task complete)
    * @param[in] InIsCancelledCallback: callback function which returns a bool if function should terminate
    */
    bool TrainSolverModels(std::function<void(float)> InProgressCallback, std::function<bool(void)> InIsCancelledCallback);

    /**
     * Set up the cameras for tracking.
     * @returns True if successful, false upon any error.
     */
    bool SetCameras(const std::map<std::string, OpenCVCamera>& InCameras);

    /**
     * Specify the ranges for each camera.
     * @returns True if successful, false upon error.
     */
    bool SetCameraRanges(const std::map<std::string, std::pair<float, float>>& InCameraRanges);

    /**
     * Specify which cameras are used for stereo reconstruction.
     * @returns True if successful, false if the cameras have not been set up via SetCameras().
     */
    bool SetStereoCameraPairs(const std::vector<std::pair<std::string, std::string>>& InStereoReconstructionPairs);

    /**
     * Reset and set up a new track.
     * @param[in] InFrameStart  The first frame of the sequence.
     * @param[in] InFrameEnd  The last (not including) frame of the sequence.
     * @param[in] InOptFlowData Optical flow related data (whethter to use optical flow, forward / backward flow and confidence)
     * @note
     *     Using confidence will cause further slowdown as additional optical flow pass is executed so it is not recommended.
     * @returns True if successful, false upon error.
     */
    bool ResetTrack(int32_t InFrameStart, int32_t InFrameEnd, OpticalFlowData InOptFlowData = {});

    /**
     * Set the current input data and performs stereo reconstruction.
     * @param[in] InImageDataPerCamera      The distorted images per camera (only images that are used for stereo reconstruction are necessary).
     * @param[in] InLandmarksDataPerCamera  The distorted landmarks for each camera (at least 2 cameras need to have landmarks).
     * @param[in] InDepthMaps Depthmap data per (depthmap) camera. In most cases it will hold single entry.
     * @param[in] InEndLevel  End level for the reconstruction.
     * @returns True if setting the data was successful.
     */
    bool SetInputData(const std::map<std::string, const unsigned char*>& InImageDataPerCamera,
                      const std::map<std::string, const std::map<std::string, FaceTrackingLandmarkData>>& InLandmarksDataPerCamera,
                      const std::map<std::string, const float*>& InDepthMaps = std::map<std::string, const float*>(), 
                      int InEndLevel = 0);

    /**
     * Track frame @p InFrameNumber.
     * @param[in] InFrameNumber The frame number to track. Uses the previously set input data @see SetInputData()
     * @param[in] InFlowCameraImages Optical flow camera names with appropriate images.
     * @param[in] bInUseFastSolver If set to true, use the fast solver (PCA mesh-tracking + pose-based solve). Requires that both PCA rig and pose-based solvers have been set. 
     * @returns True if tracking was successful.
     */
    bool Track(int32_t InFrameNumber, const std::map<std::string, std::pair<float*, float*>>& InFlowCameraImages = {}, bool bInUseFastSolver = false);

    /**
     * Track frame @p InFrameNumber.
     * @param[in] InFrameNumber The frame number to track.
     * @param[in] InImageDataPerCamera      The distorted images per camera (only images that are used for stereo reconstruction are necessary).
     * @param[in] InLandmarksDataPerCamera  The distorted landmarks for each camera (at least 2 cameras need to have landmarks).
     * @param[in] InDepthMaps Depthmap data per (depthmap) camera. In most cases it will hold single entry.
     * @param[in] InFlowCameraImages Optical flow camera names with appropriate images.
     * @param[in] InEndLevel  End level for the reconstruction.
     * @param[in] bInUseFastSolver If set to true, use the fast solver (PCA mesh-tracking + pose-based solve). Requires that both PCA rig and pose-based solvers have been set. 
     * @returns True if tracking was successful.
     */
    bool Track(int32_t InFrameNumber,
               const std::map<std::string, const unsigned char*>& InImageDataPerCamera,
               const std::map<std::string, const std::map<std::string, FaceTrackingLandmarkData>>& InLandmarksDataPerCamera,
               const std::map<std::string, const float*>& InDepthMaps = std::map<std::string, const float*>(),
               const std::map<std::string, std::pair<float*, float*>>& InFlowCameraImages = {},
               int InEndLevel = 0,
               bool bInUseFastSolver = false
    );

    /**
     * Retrieve the tracking state
     * @param[in] InFrameNumber    The frame number for which to retrieve the state
     * @param[out] OutHeadPose     The head pose as a 4x4 affine transformation in column major format.
     * @param[out] OutRawControls  The raw rig control values.
     * @returns True if the state could be retrieved, False if there is no valid tracking state.
     */
    bool GetTrackingState(int32_t InFrameNumber, float* OutHeadPose, std::map<std::string, float>& OutRawControls);

    /**
     * @returns The number of stereo pairs.
     */
    int32_t GetNumStereoPairs() const;

    /**
     * Create the triangle mesh for stereo reconstruction @p InStereoPairIndex and call @p InReconstructionCallBack with the result.
     * @returns True if the mesh has been created and the callback has been called.
     */
    bool CreateMeshForStereoReconstructionAndCallback(int32_t InStereoPairIndex, std::function<void(int32_t InNumberOfPoints, int32_t InNumberOfTriangles, const float* InVerticesData, const int32_t* InTriangleIndices)> InReconstructionCallBack) const;

private:
    struct Private;
    Private* m{};
};

} // namespace api
} // namespace titan
