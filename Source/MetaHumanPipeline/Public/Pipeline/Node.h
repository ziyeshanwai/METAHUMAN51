// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Pipeline/Pin.h"

namespace UE::MetaHuman::Pipeline
{

class FPipelineData;

class METAHUMANPIPELINE_API FNode
{
public:

	FNode(const FString& InTypeName, const FString& InName);
	virtual ~FNode();

	virtual bool Start(const TSharedPtr<FPipelineData>& InPipelineData) { return true; }
	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) { return true; }
	virtual bool Idle(const TSharedPtr<FPipelineData>& InPipelineData) { return true; }
	virtual bool End(const TSharedPtr<FPipelineData>& InPipelineData) { return true; }

	virtual bool RequiresIdle() const { return false; }
	virtual bool ProcessesExitFrame() const { return false; }

	FString TypeName;
	FString Name;
	FString ID;

	TArray<FPin> Pins;

	int32 QueueSize = 1;

	FString ToString() const;
};

}
