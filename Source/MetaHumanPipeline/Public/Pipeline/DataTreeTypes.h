// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Misc/TVariant.h"
#include "FrameTrackingContourData.h"
#include "FrameTrackingConfidenceData.h"
#include "FrameAnimationData.h"
#include "MetaHumanLiveLinkFrameData.h"

namespace UE::MetaHuman::Pipeline
{

class METAHUMANPIPELINE_API FInvalidDataType
{
};

enum class EPipelineExitStatus
{
	Unknown = 0,
	OutOfScope,
	AlreadyRunning,
	NotInGameThread,
	InvalidNodeTypeName,
	InvalidNodeName,
	DuplicateNodeName,
	InvalidPinName,
	DuplicatePinName,
	InvalidConnection,
	AmbiguousConnection,
	Unconnected,
	LoopConnection,
	Ok,
	Aborted,
	StartError,
	ProcessError,
	EndError,
	TooFast
};

class METAHUMANPIPELINE_API FUEImageDataType
{
public:

	int32 Width = -1;
	int32 Height = -1;
	TArray<uint8> Data; // bgra order, a=255 for fully opaque
};

class METAHUMANPIPELINE_API FUEGrayImageDataType
{
public:

	int32 Width = -1;
	int32 Height = -1;
	TArray<uint8> Data;
};

class METAHUMANPIPELINE_API FHSImageDataType
{
public:

	int32 Width = -1;
	int32 Height = -1;
	TArray<float> Data;
};

class METAHUMANPIPELINE_API FScalingDataType
{
public:

	float Factor = -1.0f;
};

class METAHUMANPIPELINE_API FDepthDataType
{
public:

	int32 Width = -1;
	int32 Height = -1;
	TArray<float> Data;
};

using FDataTreeType = TVariant<FInvalidDataType, EPipelineExitStatus, int32, float, bool, FString, FUEImageDataType, FUEGrayImageDataType, FHSImageDataType, FScalingDataType, FFrameTrackingContourData, FFrameAnimationData, FDepthDataType, FFrameTrackingConfidenceData, FMetaHumanLiveLinkFrameData>;

}
