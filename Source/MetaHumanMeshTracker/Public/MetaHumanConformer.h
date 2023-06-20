// Copyright Epic Games, Inc.All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Templates/PimplPtr.h"
#include "MetaHumanCameraCalibration.h"

namespace dna
{
class StreamReader;
}

struct FFrameTrackingContourData;
struct FTrackingContour3D;

namespace UE
{
    namespace Wrappers
    {

        /**
        * Brief@ FMetaHumanConformer is a wrapper class around core tech lib that provides conformed mesh 
        * i.e. identity fitting.
        * 
        */
        class METAHUMANMESHTRACKER_API FMetaHumanConformer
        {
        public:

            FMetaHumanConformer();

            /**
             * Initialize face fitting.
             * @param[in] InConfigurationDirectory  Directory containing a template_description.json and a dna_description.json file.
             * @returns True if initialization is successful, False otherwise.
             */
            bool Init(const FString& InConfigurationDirectory);

            /**
                * Set the depth input data for one frame.
                * @param[in] InFrameNum  Frame to set the data.
                * @param[in] InLandmarksDataPerCamera  The distorted landmarks for each camera.
                * @param[in] InDepthMaps Depthmap data per (depthmap) camera.
                * @returns True if setting the data was successful.
                *
                * @warning Fails if scan data was set before.
                */
            bool SetDepthInputData(int32_t InFrameNum,
                const TMap<FString, const FFrameTrackingContourData*>& InLandmarksDataPerCamera,
                const TMap<FString, const float*>& InDepthMaps);

            /**
            * Set the scan input data.
            * @param[in] InLandmarks2DData  The distorted 2D landmarks per camera view.
            * @param[in] InLandmarks3DData  The distorted 3D landmarks.
            * @param[in] InTrianlges  Input scan data with triangles stored as numTriangles x 3  in column major format.
            * @param[in] InVertices  Input scan data with vertices as numVertices x 3 in column major format.
            * @returns True if setting the data was successful.
            *
            * @warning Fails if depthmap data was set before.
            */
            bool SetScanInputData(const TMap<FString, const FFrameTrackingContourData*>& InLandmarks2DData,
                const TMap<FString, const FTrackingContour3D*>& InLandmarks3DData,
                const TArray<int32_t>& InTrianlges, const TArray<float>& InVertices);

            /**
             * Set up the cameras for fitting.
             * @param[in] InCameras  Input cameras for landmarks and depth projection.
             * @returns True if successful, False upon any error.
             */

            bool SetCameras(const TArray<FMetaHumanCameraCalibration>& InCalibrations);

            /**
             * Fit identity given input data.
			 * @param[out] Vertex positions describing new identity as numVertices x 3 in column major format.
			 * @param[out] Stacked rigid transforms, one for each input depth map (this will be 1 for a single mesh) containing 16 float values per transform
			 * @params[out] Stacked scales, one for each input depth map (this will be 1 value for a single mesh)
             * @returns True if fitting was successful.
             */
			bool FitIdentity(TArray<float>& OutVertices, TArray<float>& OutStackedToScanTransforms, TArray<float>& OutStackedToScanScales);

            /**
             * Update teeth model and position in the rig given input data.
             * @param[out] InOutDnaStream  DNA file with updated teeth asset and rig.
			 * @param[out] Vertex positions describing updated identity as numVertices x 3 in column major format.
             * @returns True if fitting and DNA update was successful.
             */
            bool FitTeeth(dna::StreamReader* InOutDnaStream, TArray<float>& OutVertices);

            /**
             * Clears previous configuration.
             * @returns True if successful.
             */
            bool ResetInputData();

        private:
            struct Private;
            TPimplPtr<Private> Impl;
            FCriticalSection AccessMutex;
        };
    }
}
