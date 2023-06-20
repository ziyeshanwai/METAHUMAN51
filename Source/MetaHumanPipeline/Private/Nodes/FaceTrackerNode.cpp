// Copyright Epic Games, Inc. All Rights Reserved.

#include "Nodes/FaceTrackerNode.h"
#include "Pipeline/PipelineData.h"

#include "Math/TransformCalculus3D.h"
#include "MetaHumanFaceTracker.h"

namespace UE::MetaHuman::Pipeline
{

FFaceTrackerStereoNode::FFaceTrackerStereoNode(const FString& InName) : FNode("FaceTrackerStereo", InName)
{
	Pins.Add(FPin("UE Image 0 In", EPinDirection::Input, EPinType::UE_Image, 0));
	Pins.Add(FPin("Contours 0 In", EPinDirection::Input, EPinType::Contours, 0));
	Pins.Add(FPin("UE Image 1 In", EPinDirection::Input, EPinType::UE_Image, 1));
	Pins.Add(FPin("Contours 1 In", EPinDirection::Input, EPinType::Contours, 1));
	Pins.Add(FPin("Animation Out", EPinDirection::Output, EPinType::Animation));
}

bool FFaceTrackerStereoNode::Start(const TSharedPtr<FPipelineData>& InPipelineData)
{
	Tracker = MakeShared<UE::Wrappers::FMetaHumanFaceTracker>();

	if (Calibrations.Num() != 2)
	{
		InPipelineData->SetErrorNodeCode(ErrorCode::FailedToInitialize);
		InPipelineData->SetErrorNodeMessage("Must have 2 cameras");
		return false;
	}

	if (!Tracker->Init(ConfigurationDirectory))
	{
		InPipelineData->SetErrorNodeCode(ErrorCode::FailedToInitialize);
		InPipelineData->SetErrorNodeMessage("Failed to load config directory");
		return false;
	}

	bool bLoadedDNA = false;

	if (!DNAAsset.IsExplicitlyNull())
	{
		if (DNAAsset.IsValid())
		{
			bLoadedDNA = Tracker->LoadDNA(DNAAsset.Get());
		}
	}
	else
	{
		bLoadedDNA = Tracker->LoadDNA(DNAFile);
	}

	if (!bLoadedDNA)
	{
		InPipelineData->SetErrorNodeCode(ErrorCode::FailedToInitialize);
		InPipelineData->SetErrorNodeMessage("Failed to load dna file");
		return false;
	}

	if (!Tracker->SetCameras(Calibrations))
	{
		InPipelineData->SetErrorNodeCode(ErrorCode::FailedToInitialize);
		InPipelineData->SetErrorNodeMessage("Failed to set cameras");
		return false;
	}

	if (!Tracker->ResetTrack(0, 2000)) // FIX THIS - HARDWIRED MAX LIMIT OF 2000 FRAMES TO SOLVE, DONE FOR IPHONE (MONO) CASE
	{
		InPipelineData->SetErrorNodeCode(ErrorCode::FailedToInitialize);
		InPipelineData->SetErrorNodeMessage("Failed to reset track");
		return false;
	}

	TMap<FString, TPair<float, float>> Ranges;
	TArray<TPair<FString, FString>> Pairs;

	Ranges.Add(Calibrations[0].Name, TPair<float, float>(15.0, 60.0));
	Ranges.Add(Calibrations[1].Name, TPair<float, float>(15.0, 60.0));
	Pairs.Add(TPair<FString, FString>(Calibrations[0].Name, Calibrations[1].Name));

	if (!Tracker->SetCameraRanges(Ranges))
	{
		InPipelineData->SetErrorNodeCode(ErrorCode::FailedToInitialize);
		InPipelineData->SetErrorNodeMessage("Failed to set camera range");
		return false;
	}

	if (!Tracker->SetStereoCameraPairs(Pairs))
	{
		InPipelineData->SetErrorNodeCode(ErrorCode::FailedToInitialize);
		InPipelineData->SetErrorNodeMessage("Failed to set stereo pairs");
		return false;
	}

	return true;
}

bool FFaceTrackerStereoNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	const FUEImageDataType& Image0 = InPipelineData->GetData<FUEImageDataType>(Pins[0]);
	const FFrameTrackingContourData& Contours0 = InPipelineData->GetData<FFrameTrackingContourData>(Pins[1]);
	const FUEImageDataType& Image1 = InPipelineData->GetData<FUEImageDataType>(Pins[2]);
	const FFrameTrackingContourData& Contours1 = InPipelineData->GetData<FFrameTrackingContourData>(Pins[3]);

	TMap<FString, const unsigned char*> ImageDataMap;
	TMap<FString, const FFrameTrackingContourData*> LandmarkMap;

	ImageDataMap.Add(Calibrations[0].Name, Image0.Data.GetData());
	ImageDataMap.Add(Calibrations[1].Name, Image1.Data.GetData());

	LandmarkMap.Add(Calibrations[0].Name, &Contours0);
	LandmarkMap.Add(Calibrations[1].Name, &Contours1);

	if (Tracker->SetInputData(ImageDataMap, LandmarkMap))
	{
		int32 FrameNumber = InPipelineData->GetFrameNumber();

		if (Tracker->Track(FrameNumber))
		{
			FTransform HeadPose;
			TMap<FString, float> Controls;

			if (!Tracker->GetTrackingState(FrameNumber, HeadPose, Controls))
			{
				InPipelineData->SetErrorNodeCode(ErrorCode::FailedToTrack);
				InPipelineData->SetErrorNodeMessage("Failed to get state");
				return false;
			}

			FFrameAnimationData Animation;
			Animation.Pose = HeadPose;
			Animation.AnimationData = Controls;
			InPipelineData->SetData<FFrameAnimationData>(Pins[4], MoveTemp(Animation));
		}
		else
		{
			InPipelineData->SetErrorNodeCode(ErrorCode::FailedToTrack);
			InPipelineData->SetErrorNodeMessage("Failed to track");
			return false;
		}
	}
	else
	{
		InPipelineData->SetErrorNodeCode(ErrorCode::FailedToTrack);
		InPipelineData->SetErrorNodeMessage("Failed to set input data");
		return false;
	}

	return true;
}

bool FFaceTrackerStereoNode::End(const TSharedPtr<FPipelineData>& InPipelineData)
{
	Tracker = nullptr;

	return true;
}



FFaceTrackerIPhoneNode::FFaceTrackerIPhoneNode(const FString& InName) : FNode("FaceTrackerIPhone", InName)
{
	Pins.Add(FPin("UE Image In", EPinDirection::Input, EPinType::UE_Image));
	Pins.Add(FPin("Contours In", EPinDirection::Input, EPinType::Contours));
	Pins.Add(FPin("Depth In", EPinDirection::Input, EPinType::Depth));
	Pins.Add(FPin("Animation Out", EPinDirection::Output, EPinType::Animation));
}

bool FFaceTrackerIPhoneNode::Start(const TSharedPtr<FPipelineData>& InPipelineData)
{
	bIsFirstPass = true;

	Tracker = MakeShared<UE::Wrappers::FMetaHumanFaceTracker>();

	if (!Tracker->Init(ConfigurationDirectory))
	{
		InPipelineData->SetErrorNodeCode(ErrorCode::FailedToInitialize);
		InPipelineData->SetErrorNodeMessage("Failed to load config directory");
		return false;
	}

	bool bLoadedDNA = false;

	if (!DNAAsset.IsExplicitlyNull())
	{
		if (DNAAsset.IsValid())
		{
			bLoadedDNA = Tracker->LoadDNA(DNAAsset.Get());
		}
	}
	else
	{
		bLoadedDNA = Tracker->LoadDNA(DNAFile);
	}

	if (!bLoadedDNA)
	{
		InPipelineData->SetErrorNodeCode(ErrorCode::FailedToInitialize);
		InPipelineData->SetErrorNodeMessage("Failed to load dna file");
		return false;
	}

	if (!PoseBasedSolvers.IsEmpty())
	{
		if (!Tracker->SetPoseBasedSolvers(PoseBasedSolvers))
		{
			InPipelineData->SetErrorNodeCode(ErrorCode::FailedToInitialize);
			InPipelineData->SetErrorNodeMessage("Failed to set pose based solvers");
			return false;
		}

		if (!Tracker->SetPCARig(PCARig))
		{
			InPipelineData->SetErrorNodeCode(ErrorCode::FailedToInitialize);
			InPipelineData->SetErrorNodeMessage("Failed to set PCA rig");
			return false;
		}
	}

	return true;
}

bool FFaceTrackerIPhoneNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	if (bIsFirstPass)
	{
		bIsFirstPass = false;

		if (Calibrations.Num() != 2)
		{
			InPipelineData->SetErrorNodeCode(ErrorCode::FailedToTrack);
			InPipelineData->SetErrorNodeMessage("Must have 2 cameras");
			return false;
		}

		if (!Tracker->SetCameras(Calibrations))
		{
			InPipelineData->SetErrorNodeCode(ErrorCode::FailedToTrack);
			InPipelineData->SetErrorNodeMessage("Failed to set cameras");
			return false;
		}
	}

	const FUEImageDataType& Image = InPipelineData->GetData<FUEImageDataType>(Pins[0]);
	const FFrameTrackingContourData& Contours = InPipelineData->GetData<FFrameTrackingContourData>(Pins[1]);
	const FDepthDataType& Depth = InPipelineData->GetData<FDepthDataType>(Pins[2]);

	TMap<FString, const unsigned char*> ImageDataMap;
	TMap<FString, const FFrameTrackingContourData*> LandmarkMap;
	TMap<FString, const float*> DepthDataMap;

	ImageDataMap.Add(Calibrations[0].Name, Image.Data.GetData());
	LandmarkMap.Add(Calibrations[0].Name, &Contours);
	DepthDataMap.Add(Calibrations[1].Name, Depth.Data.GetData());

	if (!Tracker->ResetTrack(0, 1))
	{
		InPipelineData->SetErrorNodeCode(ErrorCode::FailedToTrack);
		InPipelineData->SetErrorNodeMessage("Failed to reset track");
		return false;
	}

	if (Tracker->SetInputData(ImageDataMap, LandmarkMap, DepthDataMap))
	{
		if (Tracker->Track(0, TMap<FString, TPair<float*, float*>>(), !PoseBasedSolvers.IsEmpty()))
		{
			FTransform HeadPose;
			TMap<FString, float> Controls;

			if (!Tracker->GetTrackingState(0, HeadPose, Controls))
			{
				InPipelineData->SetErrorNodeCode(ErrorCode::FailedToTrack);
				InPipelineData->SetErrorNodeMessage("Failed to get state");
				return false;
			}

			FFrameAnimationData Animation;
			Animation.Pose = HeadPose;
			Animation.AnimationData = Controls;
			InPipelineData->SetData<FFrameAnimationData>(Pins[3], MoveTemp(Animation));
		}
		else
		{
			if (bTrackingFailureIsError)
			{
				InPipelineData->SetErrorNodeCode(ErrorCode::FailedToTrack);
				InPipelineData->SetErrorNodeMessage("Failed to track");
				return false;
			}
			else
			{
				FFrameAnimationData Animation;
				InPipelineData->SetData<FFrameAnimationData>(Pins[3], MoveTemp(Animation));
			}
		}
	}
	else
	{
		InPipelineData->SetErrorNodeCode(ErrorCode::FailedToTrack);
		InPipelineData->SetErrorNodeMessage("Failed to set input data");
		return false;
	}

	return true;
}

bool FFaceTrackerIPhoneNode::End(const TSharedPtr<FPipelineData>& InPipelineData)
{
	Tracker = nullptr;

	return true;
}

}
