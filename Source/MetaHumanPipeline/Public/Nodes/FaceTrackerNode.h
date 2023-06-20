// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Pipeline/Node.h"
#include <MetaHumanCameraCalibration.h>
#include "UObject/ObjectPtr.h"

class UDNAAsset;

namespace UE
{
	namespace Wrappers
	{
		class FMetaHumanFaceTracker;
	}
}

namespace UE::MetaHuman::Pipeline
{

class METAHUMANPIPELINE_API FFaceTrackerNodeBase
{
public:

	FString ConfigurationDirectory;
	FString DNAFile;
	TWeakObjectPtr<UDNAAsset> DNAAsset;
	TArray<FMetaHumanCameraCalibration> Calibrations;
	bool bIsFirstPass = true;
	bool bTrackingFailureIsError = true;

	enum ErrorCode
	{
		FailedToInitialize = 0,
		FailedToTrack
	};

protected:

	TSharedPtr<UE::Wrappers::FMetaHumanFaceTracker> Tracker = nullptr;
};

class METAHUMANPIPELINE_API FFaceTrackerStereoNode : public FNode, public FFaceTrackerNodeBase
{
public:

	FFaceTrackerStereoNode(const FString& InName);

	virtual bool Start(const TSharedPtr<FPipelineData>& InPipelineData) override;
	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
	virtual bool End(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

class METAHUMANPIPELINE_API FFaceTrackerIPhoneNode : public FNode, public FFaceTrackerNodeBase
{
public:

	FFaceTrackerIPhoneNode(const FString& InName);

	virtual bool Start(const TSharedPtr<FPipelineData>& InPipelineData) override;
	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
	virtual bool End(const TSharedPtr<FPipelineData>& InPipelineData) override;

	TArray<uint8> PoseBasedSolvers;
	TArray<uint8> PCARig;
};

}
