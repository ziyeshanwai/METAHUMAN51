// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Nodes/HyprsenseNodeBase.h"

class UNeuralNetwork;

namespace UE::MetaHuman::Pipeline
{
	class METAHUMANPIPELINE_API FHyprsenseRealtimeNode : public FHyprsenseNodeBase
	{
	public:
		FHyprsenseRealtimeNode(const FString& InName);

		virtual bool Start(const TSharedPtr<FPipelineData>& InPipelineData) override;
		virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;

		bool SetTrackers(const TWeakObjectPtr<UNeuralNetwork>& InFaceTracker, const TWeakObjectPtr<UNeuralNetwork>& InFaceDetector, const TWeakObjectPtr<UNeuralNetwork>& InEyebrowTracker, const TWeakObjectPtr<UNeuralNetwork>& InEyeTracker, const TWeakObjectPtr<UNeuralNetwork>& InLipsNasolabialTracker);

	private:
		TArray<int32> TrackerPartHereInputSizeX = { 64, 64, 128, 128, 0, 0, 256 };
		TArray<int32> TrackerPartHereInputSizeY = { 64, 64, 128, 128, 0, 0, 256 };

		TArray<bool> ProcessPartHere = { true, true ,true, true, false, false, true };

	};


	// The managed node is a version of the above that take care of loading the correct NNI models
	// rather than these being specified by an externally.

	class METAHUMANPIPELINE_API FHyprsenseRealtimeManagedNode : public FHyprsenseRealtimeNode
	{
	public:
		FHyprsenseRealtimeManagedNode(const FString& InName);

		static TArray<FSoftObjectPath> GetNNIPaths();
	};
}
