// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Templates/PimplPtr.h"
#include "MetaHumanCameraCalibration.h"
#include "TrackerOpticalFlowConfiguration.h"

class UDNAAsset;

struct FFrameTrackingContourData;
namespace dna
{
    class StreamReader;
    class StreamWriter;
}
namespace UE
{
    namespace Wrappers
    {
        enum TrackerState
        {
            OK = 0,
            INIT_FAILED = 1 << 0,
            LOAD_DNA_FAILED = 1 << 1,
            SET_CAMERAS_FAILED = 1 << 2,
            SET_CAMERA_RANGES_FAILED = 1 << 3,
            SET_STEREO_CAMERA_PAIRS_FAILED = 1 << 4
        };

        class METAHUMANMESHTRACKER_API FMetaHumanFaceTracker
        {
        public:

            FMetaHumanFaceTracker();

            /**
             * Initialize face tracking.
             * @param[in] InConfigurationDirectory  Directory containing a template_description.json and a configuration.json file.
             *                                      Optional: if there is also a solver_definition.json file in the directory, then a
             *                                      hierarchical solve is used during tracking.
             * @param[in] InOptFlowConfig  Optical flow configuration for tracking.
             * @returns True if initialization is successful, False otherwise.
             */
            bool Init(const FString& InConfigurationDirectory, const FTrackerOpticalFlowConfiguration& InOptFlowConfig = {});


            /**
             * Load the DNA file.
             *
             * @returns True if the DNA file could be loaded.
             */
            bool LoadDNA(const FString& InDNAFile);

			/**
			 * Load the DNA from a UDNAAsset.
			 *
			 * @returns True if the DNA could be loaded.
			 */
			bool LoadDNA(UDNAAsset* InDNAAsset);

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
			* Set the PCA rig as a memory buffer (this allows the data to be stored within a UE asset)
			* Requires PCA rig  to be present in the object.
			*
			* @param[out] OutMemoryBuffer:  a memory buffer which on return will contain the PCA rig data
			* @returns True if successful
			*/
			bool GetPCARig(TArray<uint8>& OutMemoryBuffer);

			/*
			* Set the PCA rig from a memory buffer (this allows the data to be set from within a UE asset)
			*
			* @returns True if successful
			*/
			bool SetPCARig(const TArray<uint8>& InMemoryBuffer);

			/*
            * Load the pose-based solvers from a list of filenames
            *
            * @param[in] InPoseBasedSolverFilenames: the filenames of the pose-based solver(s) to load
            * @returns True if the pose-based solvers could be loaded
            */
            bool LoadPoseBasedSolvers(const TArray<FString>& InPoseBasedSolverFilenames);

            /*
            * Set the pose-based solvers as a memory buffer (this allows the data to be stored within a UE asset)
            * Requires pose-based solvers to be present in the object.
            *
            * @param[out] OutMemoryBuffer:  a memory buffer which on return will contain the pose-based solver(s) data
            * @returns True if successful
            */
            bool GetPoseBasedSolvers(TArray<uint8>& OutMemoryBuffer);

            /*
            * Set the pose-based solvers from a memory buffer (this allows the data to be set from within a UE asset)
            *
            * @returns True if successful
            */
            bool SetPoseBasedSolvers(const TArray<uint8>& InMemoryBuffer);

            /*
            * (Re)-train the PCA model and pose-based solvers for the current DNA.
            * Must already have set the DNA and existing pose-based solvers for this to work.
            * This does a fast re-train of the pose-based solvers based upon the best settings and animation training data in the
            * existing pose-based solvers.
            *
            * @returns True if successful.
            */
			DECLARE_DELEGATE_OneParam(FOnTrainSolverModelsProgress, float);
			DECLARE_DELEGATE_OneParam(FOnTrainSolverModelsIsCancelled, bool&);
			bool TrainSolverModels(FOnTrainSolverModelsProgress InTrainSolverModelsProgressDelegate, FOnTrainSolverModelsIsCancelled InTrainSolverModelsIsCancelledDelegate);

            /**
             * Set up the cameras for tracking.
             * @returns True if successful, False upon any error.
             */
            bool SetCameras(const TArray<FMetaHumanCameraCalibration>& InCalibration);

            /**
             * Specify the ranges for each camera.
             * @returns True if successful, False upon error.
             */
            bool SetCameraRanges(const TMap<FString, TPair<float, float>>& InCameraRanges);

            /**
             * Specify which cameras are used for stereo reconstruction.
             * @returns True if successful, False if the cameras have not been set up via SetCameras().
             */
            bool SetStereoCameraPairs(const TArray<TPair<FString, FString>>& InStereoReconstructionPairs);

            /**
             * Reset and set up a new track.
             * @param[in] InFrameStart  The first frame of the sequence.
             * @param[in] InFrameEnd  The last (not including) frame of the sequence.
             * @param[in] InOptFlowConfig  Optical flow configuration for tracking.
             * @returns True if successful, False upon error.
             */
            bool ResetTrack(int32 InFrameStart, int32 InFrameEnd, const FTrackerOpticalFlowConfiguration& InOptFlowConfig = {});

            /**
             * Set the current input data and performs stereo reconstruction.
             * @param[in] InImageDataPerCamera      The distorted images per camera (only images that are used for stereo reconstruction are necessary).
             * @param[in] InLandmarksDataPerCamera  The distorted landmarks for each camera (at least 2 cameras need to have landmarks).
             * @param[in] InDepthmapDataPerCamera  The distorted depthmaps per depthmap camera
             * @param[in] InLevel - reconstruction level
             * @returns True if setting the data was successful.
             */

            bool SetInputData(const TMap<FString, const unsigned char*>& InImageDataPerCamera,
                const TMap<FString, const FFrameTrackingContourData*>& InLandmarksDataPerCamera,
                const TMap<FString, const float*>& InDepthmapDataPerCamera = TMap<FString, const float*>(), int32 InLevel = 0);

            /**
             * Track frame @p InFrameNumber.
             * @param[in] InFrameNumber The frame number to track. Uses the previously set input data @see SetInputData()
             * @param[in] InOpticalFlowImagesForCamera Collection of source and target images per camera.
             * @param[in] bUseFastSolver Whether or not to use pose-based solver.
             * @note 
             *     It'caller responsibility to provide images in real format (i.e. floating point representation). 
             * @returns True if tracking was successful.
             */
            bool Track(int32 InFrameNumber, const TMap<FString, TPair<float*, float*>>& InOpticalFlowImagesForCamera = {}, bool bUseFastSolver = false);

            /**
             * Retrieve the tracking state
             * @param[in] InFrameNumber    The frame number for which to retrieve the state
             * @param[out] OutHeadPose     The head pose as a 4x4 affine transformation in column major format.
             * @param[out] OutRawControls  The raw rig control values.
             * @returns True if the state could be retrieved, False if there is no valid tracking state.
             */
            bool GetTrackingState(int32 InFrameNumber, FTransform& OutHeadPose, TMap<FString, float>& OutRawControls);

            /**
             * @returns The number of stereo pairs.
             */
            int32 GetNumStereoPairs();

#if 0
            /**
             * Create the triangle mesh for stereo reconstruction @p InStereoPairIndex and call @p OutDepthMapData with the result.
             * @returns True if the mesh has been created and the callback has been called.
             */
            bool CreateMeshForStereoReconstructionAndCallback(int32 InStereoPairIndex, FMetaHumanDepthMapData& OutDepthMapData);
#endif

            /**
            * Returns current Tracker state.
            */
            uint32 GetTrackerState();

        private:

            struct Private;
            TPimplPtr<Private> Impl;
            FCriticalSection AccessMutex;
        };
    }
}
