// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Pipeline/Node.h"
#include "Pipeline/PipelineData.h"

namespace UE::MetaHuman::Pipeline
{

class METAHUMANPIPELINE_API FJsonTrackerNode : public FNode
{
public:

	FJsonTrackerNode(const FString& InName);

	virtual bool Start(const TSharedPtr<FPipelineData>& InPipelineData) override;
	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
	virtual bool End(const TSharedPtr<FPipelineData>& InPipelineData) override;

	FString JsonFile;

	enum ErrorCode
	{
		FailedToLoadJsonFile = 0,
		InvalidData
	};

private:

	TArray<FFrameTrackingContourData> Contours;
};

class METAHUMANPIPELINE_API FOffsetContoursNode : public FNode
{
public:

	FOffsetContoursNode(const FString& InName);

	virtual bool Start(const TSharedPtr<FPipelineData>& InPipelineData) override;
	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;

	FVector2D ConstantOffset = FVector2D(0, 0);
	FVector2D RandomOffset = FVector2D(0, 0);

private:

	float RandomOffsetMinX = -1.f;
	float RandomOffsetMaxX = -1.f;
	float RandomOffsetMinY = -1.f;
	float RandomOffsetMaxY = -1.f;
};

}
