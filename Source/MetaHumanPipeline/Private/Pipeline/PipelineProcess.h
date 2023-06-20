// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "ModelingTaskTypes.h"
#include "Pipeline/Pipeline.h"
#include "Pipeline/Node.h"
#include "PipelineProcess.generated.h"

namespace UE::MetaHuman::Pipeline
{

class FPipelineData;
using TNodePair = TPair<TSharedPtr<FNode>, TSharedPtr<FNode>>;

class FNodeInternal : public FNode
{
public:

	FNodeInternal(const FString& InName);
};

class FPipelineDataQueue
{
public:

	FPipelineDataQueue(const TNodePair& InNodePair);
	FPipelineDataQueue(const FPipelineDataQueue& InOther);

	bool IsEmpty() const;
	bool CanPush() const;

	void Push(const TSharedPtr<FPipelineData>& InPipelineData);

	TSharedPtr<FPipelineData> Pop();

	void RemoveFramesAbove(int32 InFrameNumber);

private:

	int32 MaxSize = 1;
	TArray<TSharedPtr<FPipelineData>> Queue;

	mutable FCriticalSection Mutex;
};

class FPipelineProcess : public UE::Geometry::FAbortableBackgroundTask
{
public:

	FPipelineProcess(const TArray<TSharedPtr<FNode>>& InNodes, const TArray<FConnection>& InConnections,
		const FPipelineRunParameters& InPipelineRunParameters, ENamedThreads::Type InCallingThread);

	void DoWork();

	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FPipelineProcess, STATGROUP_ThreadPoolAsyncTasks);
	}

	UE::Geometry::FAsyncTaskExecuterWithAbort<FPipelineProcess>* TaskWrapper;

	void StartNode(const TSharedPtr<FNode>& InNode);
	bool ProcessNode(const TSharedPtr<FNode>& InNode);
	void EndNode(const TSharedPtr<FNode>& InNode);

	bool IsRunning() const { return bIsRunning; }

private:

	TArray<TSharedPtr<FNode>> Nodes;
	TArray<FConnection> Connections;

	FPipelineRunParameters PipelineRunParameters;
	ENamedThreads::Type CallingThread;

	TSharedPtr<FNode> InternalSourceNode;
	TSharedPtr<FNode> InternalSyncNode;

	TMap<TSharedPtr<FNode>, TArray<TSharedPtr<FNode>>> UpstreamNodes;
	TMap<TSharedPtr<FNode>, TArray<TSharedPtr<FNode>>> DownstreamNodes;
	TMap<TNodePair, FPipelineDataQueue> Queue;

	void RunPipeline();
	void ConnectPins();
	void MakeNodeConnectionLookupsAndQueues();

	void PushSingleThreaded();
	void PushThreadPerNode();

	static constexpr int32 MaxRate = 500; // Arbitrary max processing rate otherwise game thread becomes overloaded handling "frame complete" messages

	FThreadSafeBool bIsRunning = true;
	FThreadSafeBool bIsProducing = true;

	static const int32 NotExited = -1; // Value for ExitFrame for when the value is not actually a frame number
	static const int32 ExitedOnStart = -2;
	static const int32 ExitedOnEnd = -3;

	FCriticalSection ExitMutex;
	int32 ExitFrame = FPipelineProcess::NotExited;
	TSharedPtr<FPipelineData> ExitPipelineData = nullptr;

	int32 Frame = 0;

	double StartTime = 0;
};

class FPipelineNodeProcess : public UE::Geometry::FAbortableBackgroundTask
{
public:

	FPipelineNodeProcess(const TSharedPtr<FNode>& InNode, FPipelineProcess* InPipelineProcess);
		
	void DoWork();

	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FPipelineNodeProcess, STATGROUP_ThreadPoolAsyncTasks);
	}

private:

	TSharedPtr<FNode> Node;
	FPipelineProcess* PipelineProcess;
};

}


UCLASS()
class UMetaHumanPipelineProcess : public UObject
{
	GENERATED_BODY()

public:

	UMetaHumanPipelineProcess();

	virtual void BeginDestroy() override;

	void Start(const TArray<TSharedPtr<UE::MetaHuman::Pipeline::FNode>>& InNodes,
		const TArray<UE::MetaHuman::Pipeline::FConnection>& InConnections,
		const UE::MetaHuman::Pipeline::FPipelineRunParameters& InPipelineRunParamaters);

	void Stop();

	bool IsRunning() const;

	void PipelineNowInvalid();

private:

	UE::MetaHuman::Pipeline::FPipelineRunParameters PipelineRunParameters;

	UE::Geometry::FAsyncTaskExecuterWithAbort<UE::MetaHuman::Pipeline::FPipelineProcess>* Process = nullptr;

	void FrameComplete(TSharedPtr<UE::MetaHuman::Pipeline::FPipelineData> InPipelineData);
	void ProcessComplete(TSharedPtr<UE::MetaHuman::Pipeline::FPipelineData> InPipelineData);
};
