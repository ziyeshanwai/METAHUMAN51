// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Pipeline/Node.h"
#include "Pipeline/PipelineData.h"

namespace UE::MetaHuman::Pipeline
{

template<EPinType PinEnum, typename PinData>
class METAHUMANPIPELINE_API FReplaceNode : public FNode
{
public:

	FReplaceNode(const FString& InName) : FNode("Replace-" + InName, InName)
	{
		Pins.Add(FPin("Replace", EPinDirection::Input, PinEnum, 0));
		Pins.Add(FPin("With", EPinDirection::Input, PinEnum, 1));
		Pins.Add(FPin("Output", EPinDirection::Output, PinEnum));
	}

	bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override
	{
		const PinData& With = InPipelineData->GetData<PinData>(Pins[1]);

		PinData Output = With; // Should be a data tree alias not a copy!

		InPipelineData->SetData<PinData>(Pins[2], MoveTemp(Output));

		return true;
	}
};

}
