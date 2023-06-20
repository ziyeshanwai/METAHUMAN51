// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Pipeline/Node.h"
#include "Pipeline/PipelineData.h"

#include "Async/Async.h"
#include "Async/Future.h"

namespace UE::MetaHuman::Pipeline
{

template<typename NodeType>
class FAsyncNode : public FNode
{

public:

	template<typename... InArgTypes>
	FAsyncNode(int32 NumberOfNodes, InArgTypes&&... InArgs) : FNode("", "")
	{
		check(NumberOfNodes > 0);

		for (int32 Index = 0; Index < NumberOfNodes; ++Index)
		{
			TSharedPtr<NodeType> Node = MakeShared<NodeType>(Forward<InArgTypes>(InArgs)...);

			AllNodes.Add(Node);
			AvailableNodes.Add(Node);
		}

		Name = AllNodes[0]->Name;
		TypeName = AllNodes[0]->TypeName + "-Async";
		Pins = AllNodes[0]->Pins;
	}

	bool Start(const TSharedPtr<FPipelineData>& InPipelineData) override
	{
		for (TSharedPtr<NodeType> Node : AllNodes)
		{
			Node->Pins = Pins;

			TSharedPtr<FPipelineData> PipelineData = MakeShared<FPipelineData>();

			Futures.Add(Async(EAsyncExecution::ThreadPool, [PipelineData, Node]()
			{
				FProcessData ProcessData;
	
				ProcessData.PipelineData = PipelineData;
				ProcessData.bProcessOk = Node->Start(ProcessData.PipelineData);

				return ProcessData;
			}));
		}

		bool bStartOk = true;

		while (!Futures.IsEmpty())
		{
			if (Futures[0].IsReady())
			{
				FProcessData ProcessData = Futures[0].Get();
				Futures.RemoveAt(0);

				if (bStartOk && !ProcessData.bProcessOk)
				{
					bStartOk = false;
					ProcessData.PipelineData->MoveTo(InPipelineData);
				}
			}
			else
			{
				FPlatformProcess::Sleep(0.001f);
			}
		}

		return bStartOk;
	}

	bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override
	{
		bool bProcessOk = true;
		bool bGotResults = false;

		if (AvailableNodes.IsEmpty())
		{
			Futures[0].Wait();
		}

		TSharedPtr<FPipelineData> PipelineData = MakeShared<FPipelineData>();
		InPipelineData->MoveTo(PipelineData);

		if (!Futures.IsEmpty() && Futures[0].IsReady())
		{
			FProcessData ProcessData = Futures[0].Get();
			Futures.RemoveAt(0);

			ProcessData.PipelineData->MoveTo(InPipelineData);

			AvailableNodes.Add(ProcessData.Node);

			bProcessOk = ProcessData.bProcessOk;
			bGotResults = true;
		}

		if (!bGotResults)
		{
			InPipelineData->SetDropFrame(true);
		}

		TSharedPtr<NodeType> Node = AvailableNodes[0];
		AvailableNodes.RemoveAt(0);

		Futures.Add(Async(EAsyncExecution::ThreadPool, [PipelineData, Node]()
		{
			FProcessData ProcessData;

			ProcessData.PipelineData = PipelineData;
			ProcessData.Node = Node;

			if (!ProcessData.PipelineData->GetEndFrameMarker() || ProcessData.Node->ProcessesExitFrame())
			{
				ProcessData.bProcessOk = ProcessData.Node->Process(ProcessData.PipelineData);
			}
			else
			{
				ProcessData.bProcessOk = true;
			}

			return ProcessData;
		}));

		return bProcessOk;
	}

	bool Idle(const TSharedPtr<FPipelineData>& InPipelineData) override
	{ 
		bool bProcessOk = true;

		if (!Futures.IsEmpty() && Futures[0].IsReady())
		{
			FProcessData ProcessData = Futures[0].Get();
			Futures.RemoveAt(0);

			ProcessData.PipelineData->MoveTo(InPipelineData);

			AvailableNodes.Add(ProcessData.Node);

			bProcessOk = ProcessData.bProcessOk;
		}
		else
		{
			InPipelineData->SetDropFrame(true);
		}

		return bProcessOk;
	}

	bool End(const TSharedPtr<FPipelineData>& InPipelineData) override
	{
		for (TFuture<FProcessData>& Future : Futures)
		{
			Future.Wait();
		}
		Futures.Reset();

		for (TSharedPtr<NodeType> Node : AllNodes)
		{
			TSharedPtr<FPipelineData> PipelineData = MakeShared<FPipelineData>();

			Futures.Add(Async(EAsyncExecution::ThreadPool, [PipelineData, Node]()
			{
				FProcessData ProcessData;

				ProcessData.PipelineData = PipelineData;
				ProcessData.bProcessOk = Node->End(ProcessData.PipelineData);

				return ProcessData;
			}));
		}

		bool bEndOk = true;

		while (!Futures.IsEmpty())
		{
			if (Futures[0].IsReady())
			{
				FProcessData ProcessData = Futures[0].Get();
				Futures.RemoveAt(0);

				if (bEndOk && !ProcessData.bProcessOk)
				{
					bEndOk = false;
					ProcessData.PipelineData->MoveTo(InPipelineData);
				}
			}
			else
			{
				FPlatformProcess::Sleep(0.001f);
			}
		}

		return bEndOk;
	}

	bool RequiresIdle() const override
	{
		return true;
	}

	bool ProcessesExitFrame() const override
	{
		return true;
	}

	const TArray<TSharedPtr<NodeType>>& GetNodes() const
	{
		return AllNodes;
	}

private:

	class FProcessData
	{
	public:
		TSharedPtr<FPipelineData> PipelineData;
		TSharedPtr<NodeType> Node;
		bool bProcessOk = false;
	};

	TArray<TSharedPtr<NodeType>> AllNodes;
	TArray<TSharedPtr<NodeType>> AvailableNodes;
	TArray<TFuture<FProcessData>> Futures;
};

}
