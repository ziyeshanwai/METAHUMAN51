// Copyright Epic Games, Inc. All Rights Reserved.

#include "MetaHumanConformer.h"
#include "FrameTrackingContourData.h"
#include "api/ActorCreationAPI.h"

namespace UE
{
namespace Wrappers
{
    static const int32_t NumElements= 72147;

    struct FMetaHumanConformer::Private
    {
        titan::api::ActorCreationAPI API;
		int32 NumInputs = 1;
    };

    FMetaHumanConformer::FMetaHumanConformer()
    {
        Impl = MakePimpl<Private>();
    }

    bool FMetaHumanConformer::Init(const FString& InConfigurationDirectory)
    {
        return Impl->API.Init(TCHAR_TO_ANSI(*InConfigurationDirectory));
    }

    bool FMetaHumanConformer::SetDepthInputData(int32_t InFrameNum,
        const TMap<FString, const FFrameTrackingContourData*>& InLandmarksDataPerCamera,
        const TMap<FString, const float*>& InDepthMaps)
    {
        const FScopeLock ScopeLock(&AccessMutex);
        std::map<std::string, const unsigned char*> ImageDataMap;
        std::map<std::string, const std::map<std::string, titan::api::FaceTrackingLandmarkData>> LandmarkMap;
        std::map<std::string, const float*> DepthMapDataMap;
		Impl->NumInputs = InDepthMaps.Num();

        for (const TPair<FString, const float*>& DpthForCamera : InDepthMaps)
        {
            DepthMapDataMap[TCHAR_TO_ANSI(*DpthForCamera.Key)] = DpthForCamera.Value;
        }

        for (const TPair<FString, const FFrameTrackingContourData*>& LandmarksForCamera : InLandmarksDataPerCamera)
        {
            std::map<std::string, titan::api::FaceTrackingLandmarkData> LandmarkMapForCamera;

            for (const TPair<FString, FTrackingContour>& Landmarks : LandmarksForCamera.Value->TrackingContours)
            {
                std::vector<float> ComponentValues;
                ComponentValues.reserve(Landmarks.Value.DensePoints.Num() * 2);

                for (const FVector2D& Point : Landmarks.Value.DensePoints)
                {
                    ComponentValues.push_back(Point.X);
                    ComponentValues.push_back(Point.Y);
                }

                LandmarkMapForCamera[TCHAR_TO_ANSI(*Landmarks.Key)] = titan::api::FaceTrackingLandmarkData::Create(ComponentValues.data(), nullptr, Landmarks.Value.DensePoints.Num(), 2);
            }

            LandmarkMap.emplace(std::make_pair(TCHAR_TO_ANSI(*LandmarksForCamera.Key), LandmarkMapForCamera));
        }

        return Impl->API.SetDepthInputData(InFrameNum,  LandmarkMap, DepthMapDataMap);
    }

    bool FMetaHumanConformer::SetScanInputData(const TMap<FString, const FFrameTrackingContourData*>& InLandmarks2DData,
        const TMap<FString, const FTrackingContour3D*>& InLandmarks3DData,
        const TArray<int32_t>& InTrianlges, const TArray<float>& InVertices)
    {
        const FScopeLock ScopeLock(&AccessMutex);
        std::map<std::string, const std::map<std::string, titan::api::FaceTrackingLandmarkData>> Landmark2DMap{};
		Impl->NumInputs = 1;

        for (const TPair<FString, const FFrameTrackingContourData*>& LandmarksForCamera : InLandmarks2DData)
        {
            std::map<std::string, titan::api::FaceTrackingLandmarkData> LandmarkMapForCamera;

            for (const TPair<FString, FTrackingContour>& Landmarks : LandmarksForCamera.Value->TrackingContours)
            {
            	if(Landmarks.Value.State.bActive)
            	{
            		std::vector<float> ComponentValues;
            		ComponentValues.reserve(Landmarks.Value.DensePoints.Num() * 2);

            		for (const FVector2D& Point : Landmarks.Value.DensePoints)
            		{
            			ComponentValues.push_back(Point.X);
            			ComponentValues.push_back(Point.Y);
            		}

            		LandmarkMapForCamera[TCHAR_TO_ANSI(*Landmarks.Key)] = titan::api::FaceTrackingLandmarkData::Create(ComponentValues.data(), nullptr, Landmarks.Value.DensePoints.Num(),2);
            	}
            }

            Landmark2DMap.emplace(std::make_pair(TCHAR_TO_ANSI(*LandmarksForCamera.Key), LandmarkMapForCamera));
        }

        std::map<std::string, const titan::api::FaceTrackingLandmarkData> Landmark3DMap{};

        for (const TPair<FString, const FTrackingContour3D*>& Landmarks : InLandmarks3DData)
        {
            
                std::vector<float> ComponentValues;
                ComponentValues.reserve(Landmarks.Value->DensePoints.Num() * 3);

                for (const FVector3d& Point : Landmarks.Value->DensePoints)
                {
                    ComponentValues.push_back(Point.X);
                    ComponentValues.push_back(Point.Y);
                    ComponentValues.push_back(Point.Z);
                }

                Landmark3DMap.emplace(std::make_pair(TCHAR_TO_ANSI(*Landmarks.Key),
                        titan::api::FaceTrackingLandmarkData::Create(ComponentValues.data(), nullptr, Landmarks.Value->DensePoints.Num(), 3)));
        }
    	
        titan::api::MeshInputData MeshInputData{ InTrianlges.Num() / 3,InTrianlges.GetData(), InVertices.Num() / 3, InVertices.GetData()};
        return Impl->API.SetScanInputData(Landmark3DMap, Landmark2DMap, MeshInputData);
    }

    bool FMetaHumanConformer::SetCameras(const TArray<FMetaHumanCameraCalibration>& InCalibrations)
    {
        std::map<std::string, titan::api::OpenCVCamera> Cameras;

        for (const FMetaHumanCameraCalibration& Calibration : InCalibrations)
        {
            titan::api::OpenCVCamera Camera;
            Camera.width = Calibration.ImageSizeX;
            Camera.height = Calibration.ImageSizeY;
            Camera.fx = Calibration.FX;
            Camera.fy = Calibration.FY;
            Camera.cx = Calibration.CX;
            Camera.cy = Calibration.CY;
            Camera.k1 = Calibration.K1;
            Camera.k2 = Calibration.K2;
            Camera.k3 = Calibration.K3;
            Camera.p1 = Calibration.P1;
            Camera.p2 = Calibration.P2;

            //! Transform from world coordinates to camera coordinates in column-major format.
            for (int32 Row = 0; Row < 4; Row++)
            {
                for (int32 Col = 0; Col < 4; Col++)
                {
                    Camera.Extrinsics[Row * 4 + Col] = Calibration.Transform.M[Row][Col];
                }
            }

            Cameras[TCHAR_TO_ANSI(*Calibration.Name)] = Camera;
        }

        return Impl->API.SetCameras(Cameras);
    }

	bool FMetaHumanConformer::FitIdentity(TArray<float>& OutVertices, TArray<float>& OutStackedToScanTransforms, TArray<float>& OutStackedToScanScales)
    {
        if (OutVertices.Num() != NumElements)
        {
            OutVertices.SetNum(NumElements);
        }

		if (OutStackedToScanTransforms.Num() != Impl->NumInputs * 16)
		{
			OutStackedToScanTransforms.SetNum(Impl->NumInputs * 16);
		}

		if (OutStackedToScanScales.Num() != Impl->NumInputs)
		{
			OutStackedToScanScales.SetNum(Impl->NumInputs);
		}

		const int32 NumRigitFitIterations = 5;

		// Only execute the next steps if the previous one didn't fail
		bool bResult = Impl->API.FitRigid(OutVertices.GetData(), OutStackedToScanTransforms.GetData(), OutStackedToScanScales.GetData(), NumRigitFitIterations);

		if (bResult)
		{
			bResult = Impl->API.FitNonRigid(OutVertices.GetData(), OutStackedToScanTransforms.GetData(), OutStackedToScanScales.GetData());
		}

		if (bResult)
		{
			bResult = Impl->API.FitPerVertex(OutVertices.GetData(), OutStackedToScanTransforms.GetData(), OutStackedToScanScales.GetData());
		}

		return bResult;
    }

    bool FMetaHumanConformer::FitTeeth(dna::StreamReader* InOutDnaStream, TArray<float>& OutVertices)
    {
		if (OutVertices.Num() != NumElements)
		{
			OutVertices.SetNum(NumElements);
		}

        return Impl->API.FitTeeth(InOutDnaStream, OutVertices.GetData());
    }

    bool FMetaHumanConformer::ResetInputData()
    {
        return Impl->API.ResetInputData();
    }
}
}
