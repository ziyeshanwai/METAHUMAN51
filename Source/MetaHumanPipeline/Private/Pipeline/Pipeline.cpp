// Copyright Epic Games, Inc. All Rights Reserved.

#include "Pipeline/Pipeline.h"
#include "Pipeline/Node.h"
#include "Pipeline/PipelineProcess.h"
#include "Pipeline/PipelineData.h"

namespace UE::MetaHuman::Pipeline
{

void FPipelineRunParameters::SetMode(EPipelineMode InMode)
{
	Mode = InMode;
}

EPipelineMode FPipelineRunParameters::GetMode() const
{
	return Mode;
}

void FPipelineRunParameters::SetOnFrameComplete(const FFrameComplete& InOnFrameComplete)
{
	OnFrameComplete = InOnFrameComplete;
}

const FFrameComplete& FPipelineRunParameters::GetOnFrameComplete() const
{
	return OnFrameComplete;
}

void FPipelineRunParameters::SetOnProcessComplete(const FProcessComplete& InOnProcessComplete)
{
	OnProcessComplete = InOnProcessComplete;
}

const FProcessComplete& FPipelineRunParameters::GetOnProcessComplete() const
{
	return OnProcessComplete;
}

void FPipelineRunParameters::SetStartFrame(int32 InStartFrame)
{
	StartFrame = InStartFrame;
}

int32 FPipelineRunParameters::GetStartFrame() const
{
	return StartFrame;
}

void FPipelineRunParameters::SetEndFrame(int32 InEndFrame)
{
	EndFrame = InEndFrame;
}

int32 FPipelineRunParameters::GetEndFrame() const
{
	return EndFrame;
}

void FPipelineRunParameters::SetRestrictStartingToGameThread(bool bInRestrictStartingToGameThread)
{
	bRestrictStartingToGameThread = bInRestrictStartingToGameThread;
}

bool FPipelineRunParameters::GetRestrictStartingToGameThread() const
{
	return bRestrictStartingToGameThread;
}

void FPipelineRunParameters::SetProcessNodesInRandomOrder(bool bInProcessNodesInRandomOrder)
{
	bProcessNodesInRandomOrder = bInProcessNodesInRandomOrder;
}

bool FPipelineRunParameters::GetProcessNodesInRandomOrder() const
{
	return bProcessNodesInRandomOrder;
}



FPipeline::FPipeline()
{
	Process = NewObject<UMetaHumanPipelineProcess>();
	Process->AddToRoot();
}

FPipeline::~FPipeline()
{
	Reset();

	if (Process.IsValid())
	{
		Process->PipelineNowInvalid();
		Process->RemoveFromRoot();
	}

	Process = nullptr;
}

void FPipeline::Reset()
{
	StopProcess();

	Nodes.Reset();
	Connections.Reset();
}

void FPipeline::MakeConnection(const TSharedPtr<FNode>& InFrom, const TSharedPtr<FNode>& InTo, int32 InFromGroup, int32 InToGroup)
{
	Connections.Add(FConnection(InFrom, InTo, InFromGroup, InToGroup));
}

void FPipeline::Run(EPipelineMode InPipelineMode, const FFrameComplete& InOnFrameComplete, const FProcessComplete& InOnProcessComplete)
{
	FPipelineRunParameters Params;
	Params.SetMode(InPipelineMode);
	Params.SetOnFrameComplete(InOnFrameComplete);
	Params.SetOnProcessComplete(InOnProcessComplete);

	Run(Params);
}

void FPipeline::Run(const FPipelineRunParameters &InPipelineRunParameters)
{
	if (Process.IsValid())
	{
		Process->Start(Nodes, Connections, InPipelineRunParameters);
	}
	else
	{
		TSharedPtr<FPipelineData> PipelineData = MakeShared<FPipelineData>();

		PipelineData->SetExitStatus(UE::MetaHuman::Pipeline::EPipelineExitStatus::OutOfScope);
		PipelineData->SetErrorMessage("Pipeline out of scope");

		InPipelineRunParameters.GetOnProcessComplete().Broadcast(PipelineData);
	}
}

bool FPipeline::IsRunning() const
{
	return (Process.IsValid() && Process->IsRunning());
}

void FPipeline::Cancel()
{
	StopProcess();
}

FString FPipeline::ToString() const
{
	FString Message;

	Message += LINE_TERMINATOR;
	Message += LINE_TERMINATOR;
	Message += "--------------------";
	Message += LINE_TERMINATOR;
	Message += LINE_TERMINATOR;
	Message += TEXT("NODES:");
	Message += LINE_TERMINATOR;

	for (const TSharedPtr<FNode>& Node : Nodes)
	{
		Message += Node->ToString();
		Message += LINE_TERMINATOR;
	}

	Message += "--------------------";
	Message += LINE_TERMINATOR;
	Message += LINE_TERMINATOR;

	return Message;
}

void FPipeline::StopProcess()
{
	if (Process.IsValid())
	{
		Process->Stop();
	}
}

}
