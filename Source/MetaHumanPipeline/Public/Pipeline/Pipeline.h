// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Pipeline/Connection.h"
#include "Pipeline/DataTreeTypes.h"

class UMetaHumanPipelineProcess;

namespace UE::MetaHuman::Pipeline
{

class FNode;
class FPipelineData;

DECLARE_MULTICAST_DELEGATE_OneParam(FFrameComplete, TSharedPtr<FPipelineData> InPipelineData);
DECLARE_MULTICAST_DELEGATE_OneParam(FProcessComplete, TSharedPtr<FPipelineData> InPipelineData);

enum class EPipelineMode
{
	PushSync = 0,
	PushAsync,
	PushAsyncNodes,
	Pull
};

class METAHUMANPIPELINE_API FPipelineRunParameters
{
public:

	FPipelineRunParameters() = default;

	void SetMode(EPipelineMode InMode);
	EPipelineMode GetMode() const;

	void SetOnFrameComplete(const FFrameComplete& InOnFrameComplete);
	const FFrameComplete& GetOnFrameComplete() const;

	void SetOnProcessComplete(const FProcessComplete& InOnProcessComplete);
	const FProcessComplete& GetOnProcessComplete() const;

	void SetStartFrame(int32 InStartFrame);
	int32 GetStartFrame() const;

	void SetEndFrame(int32 InEndFrame);
	int32 GetEndFrame() const;

	void SetRestrictStartingToGameThread(bool bInRestrictStartingToGameThread);
	bool GetRestrictStartingToGameThread() const;

	void SetProcessNodesInRandomOrder(bool bInProcessNodesInRandomOrder);
	bool GetProcessNodesInRandomOrder() const;

private:

	EPipelineMode Mode = EPipelineMode::PushAsync;

	FFrameComplete OnFrameComplete;
	FProcessComplete OnProcessComplete;

	int32 StartFrame = 0;
	int32 EndFrame = -1;

	bool bRestrictStartingToGameThread = true;
	bool bProcessNodesInRandomOrder = true;

	// potentially other termination conditions here like timeouts
};

class METAHUMANPIPELINE_API FPipeline
{
public:

	FPipeline();
	~FPipeline();

	void Reset();

	template<typename T, typename... InArgTypes>
	TSharedPtr<T> MakeNode(InArgTypes&&... InArgs)
	{
		TSharedPtr<T> Node = MakeShared<T>(Forward<InArgTypes>(InArgs)...);
		Nodes.Add(Node);
		return Node;
	}

	void MakeConnection(const TSharedPtr<FNode>& InFrom, const TSharedPtr<FNode>& InTo, int32 InFromGroup = 0, int32 InToGroup = 0);

	void Run(EPipelineMode InPipelineMode, const FFrameComplete& InOnFrameComplete, const FProcessComplete& InOnProcessComplete);
	void Run(const FPipelineRunParameters& InPipelineRunParameters);
	bool IsRunning() const;
	void Cancel();

	FString ToString() const;

	static void TestCommandLine(); // Temporary test function
	static void TestGUI(FPipeline& Pipeline); // Temporary test function

private:

	TArray<TSharedPtr<FNode>> Nodes;
	TArray<FConnection> Connections;

	TWeakObjectPtr<UMetaHumanPipelineProcess> Process = nullptr;

	void StopProcess();
};

}
