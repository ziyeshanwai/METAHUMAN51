// Copyright Epic Games, Inc. All Rights Reserved.

#include "Nodes/TestNodes.h"
#include "Pipeline/PipelineData.h"
#include "Pipeline/Log.h"

namespace UE::MetaHuman::Pipeline
{

FIntSrcNode::FIntSrcNode(const FString& InName) : FNode("IntSrc", InName)
{
	Pins.Add(FPin("Int Out", EPinDirection::Output, EPinType::Int));
}

bool FIntSrcNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	int32 FrameNumber = InPipelineData->GetFrameNumber();

	InPipelineData->SetData<int32>(Pins[0], Value + FrameNumber);

	return FrameNumber < NumberOfFrames;
}

FIntIncNode::FIntIncNode(const FString& InName) : FNode("IntInc", InName)
{
	Pins.Add(FPin("Int In", EPinDirection::Input, EPinType::Int));
	Pins.Add(FPin("Int Out", EPinDirection::Output, EPinType::Int));
}

bool FIntIncNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	int32 Input = InPipelineData->GetData<int32>(Pins[0]);

	InPipelineData->SetData<int32>(Pins[1], Input + 1);

	return true;
}

FIntDecNode::FIntDecNode(const FString& InName) : FNode("IntDec", InName)
{
	Pins.Add(FPin("Int In", EPinDirection::Input, EPinType::Int));
	Pins.Add(FPin("Int Out", EPinDirection::Output, EPinType::Int));
}

bool FIntDecNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	int32 Input = InPipelineData->GetData<int32>(Pins[0]);

	InPipelineData->SetData<int32>(Pins[1], Input - 1);

	return true;
}

FIntSumNode::FIntSumNode(const FString& InName) : FNode("IntSum", InName)
{
	Pins.Add(FPin("Int1 In", EPinDirection::Input, EPinType::Int, 0));
	Pins.Add(FPin("Int2 In", EPinDirection::Input, EPinType::Int, 1));
	Pins.Add(FPin("Int1 Out", EPinDirection::Output, EPinType::Int, 0));
	Pins.Add(FPin("Int2 Out", EPinDirection::Output, EPinType::Int, 1));
}

bool FIntSumNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	int32 Input1 = InPipelineData->GetData<int32>(Pins[0]);
	int32 Input2 = InPipelineData->GetData<int32>(Pins[1]);

	InPipelineData->SetData<int32>(Pins[2], Input1 + Input2);
	InPipelineData->SetData<int32>(Pins[3], Input1 - Input2);

	return true;
}

FIntLogNode::FIntLogNode(const FString& InName) : FNode("IntLog", InName)
{
	Pins.Add(FPin("Int In", EPinDirection::Input, EPinType::Int));
}

bool FIntLogNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	int32 Input = InPipelineData->GetData<int32>(Pins[0]);

	UE_LOG(LogMetaHumanPipeline, Display, TEXT("   Results [%s] [%i] Value [%i]"), *Name, InPipelineData->GetFrameNumber(), Input);

	return true;
}



FFltSrcNode::FFltSrcNode(const FString& InName) : FNode("FltSrc", InName)
{
	Pins.Add(FPin("Flt Out", EPinDirection::Output, EPinType::Float));
}

bool FFltSrcNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	int32 FrameNumber = InPipelineData->GetFrameNumber();

	InPipelineData->SetData<float>(Pins[0], Value + FrameNumber);

	return FrameNumber < NumberOfFrames;
}

FFltIncNode::FFltIncNode(const FString& InName) : FNode("FltInc", InName)
{
	Pins.Add(FPin("Flt In", EPinDirection::Input, EPinType::Float));
	Pins.Add(FPin("Flt Out", EPinDirection::Output, EPinType::Float));
}

bool FFltIncNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	float Input = InPipelineData->GetData<float>(Pins[0]);

	InPipelineData->SetData<float>(Pins[1], Input + 0.1f);

	return true;
}

FFltDecNode::FFltDecNode(const FString& InName) : FNode("FltDec", InName)
{
	Pins.Add(FPin("Flt In", EPinDirection::Input, EPinType::Float));
	Pins.Add(FPin("Flt Out", EPinDirection::Output, EPinType::Float));
}

bool FFltDecNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	float Input = InPipelineData->GetData<float>(Pins[0]);

	InPipelineData->SetData<float>(Pins[1], Input - 0.1f);

	return true;
}

FFltSumNode::FFltSumNode(const FString& InName) : FNode("FltSum", InName)
{
	Pins.Add(FPin("Flt1 In", EPinDirection::Input, EPinType::Float, 0));
	Pins.Add(FPin("Flt2 In", EPinDirection::Input, EPinType::Float, 1));
	Pins.Add(FPin("Flt1 Out", EPinDirection::Output, EPinType::Float, 0));
	Pins.Add(FPin("Flt2 Out", EPinDirection::Output, EPinType::Float, 1));
}

bool FFltSumNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	float Input1 = InPipelineData->GetData<float>(Pins[0]);
	float Input2 = InPipelineData->GetData<float>(Pins[1]);

	InPipelineData->SetData<float>(Pins[2], Input1 + Input2);
	InPipelineData->SetData<float>(Pins[3], Input1 - Input2);

	return true;
}

FFltLogNode::FFltLogNode(const FString& InName) : FNode("FltLog", InName)
{
	Pins.Add(FPin("Flt In", EPinDirection::Input, EPinType::Float));
}

bool FFltLogNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	float Input = InPipelineData->GetData<float>(Pins[0]);

	UE_LOG(LogMetaHumanPipeline, Display, TEXT("   Results [%s] [%i] Value [%f]"), *Name, InPipelineData->GetFrameNumber(), Input);

	return true;
}



FMixSrcNode::FMixSrcNode(const FString& InName) : FNode("MixSrc", InName)
{
	Pins.Add(FPin("Int Out", EPinDirection::Output, EPinType::Int));
	Pins.Add(FPin("Flt Out", EPinDirection::Output, EPinType::Float));
}

bool FMixSrcNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	int32 FrameNumber = InPipelineData->GetFrameNumber();

	InPipelineData->SetData<int32>(Pins[0], IntValue + FrameNumber);
	InPipelineData->SetData<float>(Pins[1], FltValue + FrameNumber);

	return FrameNumber < NumberOfFrames;
}

FMixIncNode::FMixIncNode(const FString& InName) : FNode("MixInc", InName)
{
	Pins.Add(FPin("Int In", EPinDirection::Input, EPinType::Int));
	Pins.Add(FPin("Flt In", EPinDirection::Input, EPinType::Float));
	Pins.Add(FPin("Int Out", EPinDirection::Output, EPinType::Int));
	Pins.Add(FPin("Flt Out", EPinDirection::Output, EPinType::Float));
}

bool FMixIncNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	int32 Input1 = InPipelineData->GetData<int32>(Pins[0]);
	float Input2 = InPipelineData->GetData<float>(Pins[1]);

	InPipelineData->SetData<int32>(Pins[2], Input1 + 1);
	InPipelineData->SetData<float>(Pins[3], Input2 + 0.1f);

	return true;
}

FMixDecNode::FMixDecNode(const FString& InName) : FNode("MixDec", InName)
{
	Pins.Add(FPin("Int In", EPinDirection::Input, EPinType::Int));
	Pins.Add(FPin("Flt In", EPinDirection::Input, EPinType::Float));
	Pins.Add(FPin("Int Out", EPinDirection::Output, EPinType::Int));
	Pins.Add(FPin("Flt Out", EPinDirection::Output, EPinType::Float));
}

bool FMixDecNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	int32 Input1 = InPipelineData->GetData<int32>(Pins[0]);
	float Input2 = InPipelineData->GetData<float>(Pins[1]);

	InPipelineData->SetData<int32>(Pins[2], Input1 - 1);
	InPipelineData->SetData<float>(Pins[3], Input2 - 0.1f);

	return true;
}

FMixSumNode::FMixSumNode(const FString& InName) : FNode("MixSum", InName)
{
	Pins.Add(FPin("Int1 In", EPinDirection::Input, EPinType::Int, 0));
	Pins.Add(FPin("Flt1 In", EPinDirection::Input, EPinType::Float, 0));
	Pins.Add(FPin("Int2 In", EPinDirection::Input, EPinType::Int, 1));
	Pins.Add(FPin("Flt2 In", EPinDirection::Input, EPinType::Float, 1));
	Pins.Add(FPin("Int1 Out", EPinDirection::Output, EPinType::Int, 0));
	Pins.Add(FPin("Flt1 Out", EPinDirection::Output, EPinType::Float, 0));
	Pins.Add(FPin("Int2 Out", EPinDirection::Output, EPinType::Int, 1));
	Pins.Add(FPin("Flt2 Out", EPinDirection::Output, EPinType::Float, 1));
}

bool FMixSumNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	int32 Input1 = InPipelineData->GetData<int32>(Pins[0]);
	float Input2 = InPipelineData->GetData<float>(Pins[1]);
	int32 Input3 = InPipelineData->GetData<int32>(Pins[2]);
	float Input4 = InPipelineData->GetData<float>(Pins[3]);

	InPipelineData->SetData<int32>(Pins[4], Input1 + Input3);
	InPipelineData->SetData<float>(Pins[5], Input2 + Input4);
	InPipelineData->SetData<int32>(Pins[6], Input1 - Input3);
	InPipelineData->SetData<float>(Pins[7], Input2 - Input4);

	return true;
}

FMixLogNode::FMixLogNode(const FString& InName) : FNode("MixLog", InName)
{
	Pins.Add(FPin("Int In", EPinDirection::Input, EPinType::Int));
	Pins.Add(FPin("Flt In", EPinDirection::Input, EPinType::Float));
}

bool FMixLogNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	int32 Input1 = InPipelineData->GetData<int32>(Pins[0]);
	float Input2 = InPipelineData->GetData<float>(Pins[1]);

	UE_LOG(LogMetaHumanPipeline, Display, TEXT("   Results [%s] [%i] Value [%i] [%f]"), *Name, InPipelineData->GetFrameNumber(), Input1, Input2);

	return true;
}

FErrorNode::FErrorNode(const FString& InName) : FIntIncNode(InName)
{
	TypeName = "Error";
}

bool FErrorNode::Start(const TSharedPtr<FPipelineData>& InPipelineData)
{
	FIntIncNode::Start(InPipelineData);

	if (ErrorOnStage == 0)
	{
		InPipelineData->SetErrorNodeCode(ErrorCode);
		InPipelineData->SetErrorNodeMessage("START ERROR");

		return false;
	}
	else
	{
		return true;
	}
}

bool FErrorNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	FIntIncNode::Process(InPipelineData);

	if (ErrorOnStage == 1 && InPipelineData->GetFrameNumber() == ErrorOnFrame)
	{
		InPipelineData->SetErrorNodeCode(ErrorCode);
		InPipelineData->SetErrorNodeMessage("PROCESS ERROR");

		return false;
	}
	else
	{
		return true;
	}
}

bool FErrorNode::End(const TSharedPtr<FPipelineData>& InPipelineData)
{
	FIntIncNode::End(InPipelineData);

	if (ErrorOnStage == 2)
	{
		InPipelineData->SetErrorNodeCode(ErrorCode);
		InPipelineData->SetErrorNodeMessage("END ERROR");

		return false;
	}
	else
	{
		return true;
	}
}

FIntsToFltNode::FIntsToFltNode(const FString& InName) : FNode("IntsToFlt", InName)
{
	Pins.Add(FPin("Int1 In", EPinDirection::Input, EPinType::Int));
	Pins.Add(FPin("Int2 In", EPinDirection::Input, EPinType::Int));
	Pins.Add(FPin("Flt Out", EPinDirection::Output, EPinType::Float));
}

bool FIntsToFltNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	int32 Input1 = InPipelineData->GetData<int32>(Pins[0]);
	int32 Input2 = InPipelineData->GetData<int32>(Pins[1]);

	InPipelineData->SetData<float>(Pins[2], Input1 + Input2);

	return true;
}

FProcessCountNode::FProcessCountNode(const FString& InName) : FIntIncNode(InName)
{
	TypeName = "ProcessCount";
}

bool FProcessCountNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	ProcessCount++;

	return FIntIncNode::Process(InPipelineData);
}

FDropFrameNode::FDropFrameNode(const FString& InName) : FNode("DropFrame", InName)
{
}

bool FDropFrameNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	if (InPipelineData->GetFrameNumber() % DropEvery == 0)
	{
		InPipelineData->SetDropFrame(true);
	}

	return true;
}

FBufferNode::FBufferNode(const FString& InName) : FNode("Buffer", InName)
{
}

bool FBufferNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	TSharedPtr<FPipelineData> PipelineData = MakeShared<FPipelineData>();
	InPipelineData->MoveTo(PipelineData);

	BufferedData.Add(PipelineData);

	InPipelineData->SetDropFrame(true);

	LastAdd = FPlatformTime::Seconds();

	return true;
}

bool FBufferNode::Idle(const TSharedPtr<FPipelineData>& InPipelineData)
{
	if (!BufferedData.IsEmpty() && (BufferedData.Num() > Capacity || (FPlatformTime::Seconds() - LastAdd) > Delay))
	{
		BufferedData[0]->MoveTo(InPipelineData);
		BufferedData.RemoveAt(0);
	}
	else
	{
		InPipelineData->SetDropFrame(true);
	}

	return true;
}

FSlowIntIncNode::FSlowIntIncNode(const FString& InName) : FIntIncNode(InName)
{
	TypeName = "SlowIntInc";
}

bool FSlowIntIncNode::Process(const TSharedPtr<FPipelineData>& InPipelineData)
{
	FPlatformProcess::Sleep(1.0f);

	return FIntIncNode::Process(InPipelineData);
}

}
