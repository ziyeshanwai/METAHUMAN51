// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Pipeline/Node.h"
#include "Pipeline/PipelineData.h"
#include "UObject/WeakObjectPtr.h"

#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"

#include "SupressWarnings.h"

class UNeuralNetwork;

namespace UE::MetaHuman::Pipeline
{
#if WITH_DEV_AUTOMATION_TESTS
	class METAHUMANPIPELINE_API FHyprsenseTestNode : public FNode
	{
	public:

		FHyprsenseTestNode(const FString& InName);
		virtual bool Start(const TSharedPtr<FPipelineData>& InPipelineData) override;
		virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
		virtual bool End(const TSharedPtr<FPipelineData>& InPipelineData) override;

		FString InJsonFilePath;
		FString OutJsonFilePath;

		TArray<FFrameTrackingContourData> ContourByFrame;
		TArray<TMap<FString, float>> ContourDiffAverageByFrame;
		TArray<float> TotalLandmarkDiffAverageByFrame;

		int32 FrameCount = 0;
		float TotalAverageInAllFrames = 0;
	};
#endif
}
