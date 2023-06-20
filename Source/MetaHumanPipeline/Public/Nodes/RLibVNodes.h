// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Pipeline/Node.h"

class FRLibVGenericTracker;
class FRLibVRefinementTracker;

namespace UE::MetaHuman::Pipeline
{

class METAHUMANPIPELINE_API FRLibVRefinementTrackerNode : public FNode
{
public:

	FRLibVRefinementTrackerNode(const FString& InName);

	virtual bool Start(const TSharedPtr<FPipelineData>& InPipelineData) override;
	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
	virtual bool End(const TSharedPtr<FPipelineData>& InPipelineData) override;

	FString ModelFile;

	enum ErrorCode
	{
		FailedToInitialize = 0,
		FailedToTrack
	};

private:

	TSharedPtr<FRLibVRefinementTracker> Tracker = nullptr;
};

}
