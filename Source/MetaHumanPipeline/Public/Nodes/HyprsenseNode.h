// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once
#include "Nodes/HyprsenseNodeBase.h"

class UNeuralNetwork;

namespace UE::MetaHuman::Pipeline
{
	class METAHUMANPIPELINE_API FHyprsenseNode : public FHyprsenseNodeBase
	{
	public:
		FHyprsenseNode(const FString& InName);

		virtual bool Start(const TSharedPtr<FPipelineData>& InPipelineData) override;
		virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
		
		bool SetTrackers(const TWeakObjectPtr<UNeuralNetwork>& InFaceTracker, const TWeakObjectPtr<UNeuralNetwork>& InFaceDetector, const TWeakObjectPtr<UNeuralNetwork>& InEyebrowTracker, const TWeakObjectPtr<UNeuralNetwork>& InEyeTracker, const TWeakObjectPtr<UNeuralNetwork>& InLipsTracker, const TWeakObjectPtr<UNeuralNetwork>& InNasolabialTracker);

	private:
		TArray<int32> TrackerPartHereInputSizeX = { 256, 256, 512, 512, 512, 256 };
		TArray<int32> TrackerPartHereInputSizeY = { 256, 256, 512, 512, 512, 256 };

		TArray<bool> ProcessPartHere = { true, true ,true, true, true, true, false };

	};
}
