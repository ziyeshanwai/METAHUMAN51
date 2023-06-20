// Copyright Epic Games, Inc. All Rights Reserved.

#include "Pipeline/Pipeline.h"
#include "Pipeline/Log.h"
#include "Pipeline/PipelineData.h"
#include "Nodes/TestNodes.h"
#include "Nodes/ImageUtilNodes.h"
#include "Nodes/HyprsenseNode.h"
#include "Nodes/HyprsenseRealtimeNode.h"
#include "Nodes/HyprsenseTestNode.h"
#include "Nodes/RLibVNodes.h"
#include "Nodes/FaceTrackerNode.h"
#include "Nodes/TrackerUtilNodes.h"
#include "Nodes/AsyncNode.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "Interfaces/IPluginManager.h"

#include "NeuralNetwork.h"

namespace UE::MetaHuman::Pipeline
{

static void FrameComplete(TSharedPtr<FPipelineData> InPipelineData)
{
	int32 Frame = InPipelineData->GetFrameNumber();

	UE_LOG(LogMetaHumanPipeline, Warning, TEXT("FRAME COMPLETE %i"), Frame);
}

static void ProcessComplete(TSharedPtr<FPipelineData> InPipelineData)
{
	EPipelineExitStatus ExitStatus = InPipelineData->GetExitStatus();

	UE_LOG(LogMetaHumanPipeline, Warning, TEXT("PIPELINE EXIT STATUS %i"), ExitStatus);
}

void FPipeline::TestGUI(FPipeline& Pipeline)
{
	bool bIsRunning = Pipeline.IsRunning();

	Pipeline.Reset();

	if (bIsRunning)
	{
		return;
	}

	TSharedPtr<FIntSrcNode> Src1 = Pipeline.MakeNode<FIntSrcNode>("Src1");
	TSharedPtr<FIntSrcNode> Src2 = Pipeline.MakeNode<FIntSrcNode>("Src2");
	TSharedPtr<FIntIncNode> Inc1 = Pipeline.MakeNode<FIntIncNode>("Inc1");
	TSharedPtr<FIntIncNode> Inc2 = Pipeline.MakeNode<FIntIncNode>("Inc2");
	TSharedPtr<FIntIncNode> Inc3 = Pipeline.MakeNode<FIntIncNode>("Inc3");
	TSharedPtr<FIntIncNode> Inc4 = Pipeline.MakeNode<FIntIncNode>("Inc4");
	TSharedPtr<FIntIncNode> Inc5 = Pipeline.MakeNode<FIntIncNode>("Inc5");
	TSharedPtr<FIntSumNode> Sum1 = Pipeline.MakeNode<FIntSumNode>("Sum1");
	TSharedPtr<FIntLogNode> Log1 = Pipeline.MakeNode<FIntLogNode>("Log1");
	TSharedPtr<FIntLogNode> Log2 = Pipeline.MakeNode<FIntLogNode>("Log2");
	TSharedPtr<FIntLogNode> Log3 = Pipeline.MakeNode<FIntLogNode>("Log3");

	Src1->Value = 10;
	Src1->NumberOfFrames = 5;
	Src2->Value = 20;
	Src2->NumberOfFrames = 7;

	Pipeline.MakeConnection(Src1, Inc1);

	Pipeline.MakeConnection(Src2, Inc2);
	Pipeline.MakeConnection(Inc2, Inc3);
	Pipeline.MakeConnection(Inc2, Log1);
	Pipeline.MakeConnection(Inc3, Inc4);

	Pipeline.MakeConnection(Inc1, Sum1, 0, 0);
	Pipeline.MakeConnection(Inc4, Sum1, 0, 1);

	Pipeline.MakeConnection(Sum1, Inc5, 0, 0);
	Pipeline.MakeConnection(Sum1, Log2, 1, 0);

	Pipeline.MakeConnection(Inc5, Log3);

	FFrameComplete OnFrameComplete;
	FProcessComplete OnProcessComplete;

	OnFrameComplete.AddStatic(FrameComplete);
	OnProcessComplete.AddStatic(ProcessComplete);

	Pipeline.Run(EPipelineMode::PushAsync, OnFrameComplete, OnProcessComplete);
}

void FPipeline::TestCommandLine()
{
	TArray<FString> Tests;

//	Tests.Add("Int");
//	Tests.Add("Float");
//	Tests.Add("Mix");
//	Tests.Add("MultiSrc");
//	Tests.Add("MultiInput");
//	Tests.Add("MultiOutput");
//	Tests.Add("MultiCommonInput");
//	Tests.Add("MultiPath");
//	Tests.Add("NonDirectInput");
//	Tests.Add("NodeError-0-3-6");
//	Tests.Add("NodeError-1-3-7");
//	Tests.Add("NodeError-2-3-8");
//	Tests.Add("NodeError-3-3-9");
//	Tests.Add("PipelineError-1");
//	Tests.Add("PipelineError-2");
//	Tests.Add("PipelineError-3");
//	Tests.Add("PipelineError-4");
//	Tests.Add("PipelineError-5");
//	Tests.Add("PipelineError-6");
//	Tests.Add("PipelineError-7");
//	Tests.Add("PipelineError-8");
//	Tests.Add("PipelineError-9");
//	Tests.Add("PipelineError-10");
//	Tests.Add("PipelineError-11");
//	Tests.Add("Queue-5-100-1");
//	Tests.Add("Queue-100-5-1");
//	Tests.Add("Queue-100-100-1");
//	Tests.Add("Queue-5-100-10");
//	Tests.Add("Queue-100-5-10");
//	Tests.Add("Queue-100-100-10");
//	Tests.Add("Queue-5-100-1000");
//	Tests.Add("Queue-100-5-1000");
//	Tests.Add("Queue-100-100-1000");
//	Tests.Add("HyprsenseNativeRes");
//	Tests.Add("HyprsenseArbRes");
//	Tests.Add("RLibV");
//	Tests.Add("FaceTracker");
	Tests.Add("Depth");

	TArray<FString> TestsToRun;

	for (const FString& Test : Tests)
	{
		for (const FString& Method : { "PushSync" } )
		{
			for (const FString& Stage : { "Stage1", "Stage3" })
			{
				TestsToRun.Add("FPipelineTestAdvancedNodes " + Test + " " + Method + " " + Stage);
			}
		}
	}

	for (const FString& TestToRun : TestsToRun)
	{
		UE_LOG(LogMetaHumanPipeline, Warning, TEXT("TEST %s"), *TestToRun);

		FAutomationTestFramework& TestFramework = FAutomationTestFramework::Get();

		TestFramework.StartTestByName(TestToRun, 0);

		FAutomationTestExecutionInfo ExecutionInfo;
		if (TestFramework.StopTest(ExecutionInfo))
		{
			UE_LOG(LogMetaHumanPipeline, Warning, TEXT(" PASSED"));
		}
		else
		{
			UE_LOG(LogMetaHumanPipeline, Warning, TEXT(" FAILED"));

			for (const auto& Entry : ExecutionInfo.GetEntries())
			{
				if (Entry.Event.Type == EAutomationEventType::Error)
				{
					UE_LOG(LogMetaHumanPipeline, Warning, TEXT("  - %s"), *Entry.ToString());
				}
			}
		}
	}
}



#if WITH_DEV_AUTOMATION_TESTS

class FPipelineTestHelper
{
public:

	void Run(EPipelineMode InPipelineMode)
	{
		OnFrameComplete.AddRaw(this, &FPipelineTestHelper::FrameComplete);
		OnProcessComplete.AddRaw(this, &FPipelineTestHelper::ProcessComplete);

		FPipelineRunParameters PipelineRunParameters;
		PipelineRunParameters.SetMode(InPipelineMode);
		PipelineRunParameters.SetStartFrame(StartFrame);
		PipelineRunParameters.SetEndFrame(EndFrame);
		PipelineRunParameters.SetOnFrameComplete(OnFrameComplete);
		PipelineRunParameters.SetOnProcessComplete(OnProcessComplete);
		PipelineRunParameters.SetProcessNodesInRandomOrder(bProcessNodesInRandomOrder);

		StartTime = FPlatformTime::Seconds();

		Pipeline.Run(PipelineRunParameters);
	}

	FPipeline Pipeline;
	int32 FrameCompleteCount = 0;
	int32 ProcessCompleteCount = 0;
	EPipelineExitStatus ExitStatus = EPipelineExitStatus::Unknown;
	int32 ErrorNodeCode = -1;
	int32 StartFrame = 0;
	int32 EndFrame = -1;
	bool bProcessNodesInRandomOrder = true;
	bool bDropFramesPresent = false;
	TArray<TSharedPtr<FProcessCountNode>> ProcessCountNodes;
	double StartTime = 0;
	double EndTime = 0;

	TArray<TSharedPtr<const FPipelineData>> PipelineData;

private:

	FFrameComplete OnFrameComplete;
	FProcessComplete OnProcessComplete;

	void FrameComplete(TSharedPtr<FPipelineData> InPipelineData)
	{
		int32 FrameNumber = InPipelineData->GetFrameNumber();

		checkf(bDropFramesPresent || (StartFrame + FrameCompleteCount == FrameNumber), TEXT("Out of order"));

		++FrameCompleteCount;
		PipelineData.Add(InPipelineData);

		if (!ProcessCountNodes.IsEmpty() && FrameNumber < EndFrame / 2) // Process count naturally syncs towards end of sequence, so only check upto half way point
		{
			bool bInOrder = true;

			for (TSharedPtr<FProcessCountNode> Node : ProcessCountNodes)
			{
				bInOrder &= ((Node->ProcessCount - 1) == FrameNumber);
			}

			if (bProcessNodesInRandomOrder)
			{
				checkf(!bInOrder, TEXT("Node process in order"));
			}
			else
			{
				checkf(bInOrder, TEXT("Node process out of order"));
			}
		}
	}

	void ProcessComplete(TSharedPtr<FPipelineData> InPipelineData)
	{
		++ProcessCompleteCount;

		ExitStatus = InPipelineData->GetExitStatus();
		if (ExitStatus != EPipelineExitStatus::Ok)
		{
			ErrorNodeCode = InPipelineData->GetErrorNodeCode();
		}

		EndTime = FPlatformTime::Seconds();
	}
};

#define TEST_PIPELINE_DATA(A, B, C, D) \
 bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[(A)]->HasData<B>((C))); \
 bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[(A)]->GetData<B>((C)), (D)));

static TSharedPtr<FPipelineTestHelper> TestHelper = nullptr; // Assumes tests runs one at a time and in order!

DEFINE_LATENT_AUTOMATION_COMMAND_ONE_PARAMETER(FPipelineTestNodesComplete, FString, Name);

bool FPipelineTestNodesComplete::Update()
{
	return (TestHelper && TestHelper->ProcessCompleteCount != 0);
}

IMPLEMENT_COMPLEX_AUTOMATION_TEST(FPipelineTestBasicNodes, "MetaHuman.Pipeline.BasicNodes", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
IMPLEMENT_COMPLEX_AUTOMATION_TEST(FPipelineTestAdvancedNodes, "MetaHuman.Pipeline.AdvancedNodes", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

void FPipelineTestBasicNodes::GetTests(TArray<FString>& OutBeautifiedNames, TArray<FString>& OutTestCommands) const
{
	TArray<FString> Tests;

	Tests.Add("Int");
	Tests.Add("Float");
	Tests.Add("Mix");
	Tests.Add("MultiSrc");
	Tests.Add("MultiInput");
	Tests.Add("MultiOutput");
	Tests.Add("MultiCommonInput");
	Tests.Add("MultiPath");
	Tests.Add("NonDirectInput");
	Tests.Add("NodeError-0-3-6");
	Tests.Add("NodeError-1-3-7");
	Tests.Add("NodeError-2-3-8");
	Tests.Add("NodeError-3-3-9");
	Tests.Add("PipelineError-1");
	Tests.Add("PipelineError-2");
	Tests.Add("PipelineError-3");
	Tests.Add("PipelineError-4");
	Tests.Add("PipelineError-5");
	Tests.Add("PipelineError-6");
	Tests.Add("PipelineError-7");
	Tests.Add("PipelineError-8");
	Tests.Add("PipelineError-9");
	Tests.Add("PipelineError-10");
	Tests.Add("PipelineError-11");
	Tests.Add("Queue-5-20-1");
	Tests.Add("Queue-20-5-1");
	Tests.Add("Queue-20-20-1");
	Tests.Add("Queue-5-20-10");
	Tests.Add("Queue-20-5-10");
	Tests.Add("Queue-20-20-10");
	Tests.Add("Queue-5-20-1000");
	Tests.Add("Queue-20-5-1000");
	Tests.Add("Queue-20-20-1000");
	Tests.Add("StartEndFrame");
	Tests.Add("DropFrame");
	Tests.Add("Buffer");
	Tests.Add("NonAsync");
	Tests.Add("Async");

	for (const FString& Test : Tests)
	{
		for (const FString& Method : { "PushSync", "PushAsync", "PushAsyncNodes" })
		{
			for (const FString& Stage : { "Stage1", "Stage2", "Stage3" })
			{
				OutBeautifiedNames.Add(Test + " " + Method + " " + Stage);
				OutTestCommands.Add(Test + " " + Method + " " + Stage);
			}
		}
	}

	Tests.Reset();
	Tests.Add("RandomNodeOrder");
	Tests.Add("LinearNodeOrder");

	for (const FString& Test : Tests)
	{
		for (const FString& Method : { "PushSync" })
		{
			for (const FString& Stage : { "Stage1", "Stage2", "Stage3" })
			{
				OutBeautifiedNames.Add(Test + " " + Method + " " + Stage);
				OutTestCommands.Add(Test + " " + Method + " " + Stage);
			}
		}
	}
}

void FPipelineTestAdvancedNodes::GetTests(TArray<FString>& OutBeautifiedNames, TArray<FString>& OutTestCommands) const
{
	TArray<FString> Tests;

	Tests.Add("Hyprsense");
	Tests.Add("HyprsenseRealtime");
	Tests.Add("HyprsensePythonCompare");
	Tests.Add("Depth");
	Tests.Add("FaceTrackerIPhone");
	Tests.Add("Grayscale");
	Tests.Add("Crop");
	Tests.Add("Composite");
	Tests.Add("RLibVRefinement");
	Tests.Add("JsonTracker");

	for (const FString& Test : Tests)
	{
		for (const FString& Method : { "PushSync", "PushAsync", "PushAsyncNodes" })
		{
			for (const FString& Stage : { "Stage1", "Stage2", "Stage3" })
			{
				OutBeautifiedNames.Add(Test + " " + Method + " " + Stage);
				OutTestCommands.Add(Test + " " + Method + " " + Stage);
			}
		}
	}
}

bool FPipelineTestBasicNodes::RunTest(const FString& InTestCommand)
{
	bool bIsOK = true;

	TArray<FString> Tokens;
	InTestCommand.ParseIntoArray(Tokens, TEXT(" "), true);
	bIsOK &= TestEqual<int32>(TEXT("Well formed Parameters"), Tokens.Num(), 3);

	if (bIsOK)
	{
		TArray<FString> Params;
		Tokens[0].ParseIntoArray(Params, TEXT("-"), true);
		FString Test = Params[0];
		Params.RemoveAt(0);

		FString Method = Tokens[1];
		FString Stage = Tokens[2];

		if (Stage == "Stage1")
		{
			bIsOK &= TestInvalid(TEXT("Test helper set"), TestHelper);

			if (bIsOK)
			{
				TestHelper = MakeShared<FPipelineTestHelper>();

				if (Test == "Int")
				{
					TSharedPtr<FIntSrcNode> Src = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src");
					TSharedPtr<FIntIncNode> Inc = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc");
					TSharedPtr<FIntLogNode> Log = TestHelper->Pipeline.MakeNode<FIntLogNode>("Log");

					Src->Value = 10;
					Src->NumberOfFrames = 5;

					TestHelper->Pipeline.MakeConnection(Src, Inc);
					TestHelper->Pipeline.MakeConnection(Inc, Log);
				}
				else if (Test == "Float")
				{
					TSharedPtr<FFltSrcNode> Src = TestHelper->Pipeline.MakeNode<FFltSrcNode>("Src");
					TSharedPtr<FFltIncNode> Inc = TestHelper->Pipeline.MakeNode<FFltIncNode>("Inc");
					TSharedPtr<FFltLogNode> Log = TestHelper->Pipeline.MakeNode<FFltLogNode>("Log");

					Src->Value = 20.4f;
					Src->NumberOfFrames = 7;

					TestHelper->Pipeline.MakeConnection(Src, Inc);
					TestHelper->Pipeline.MakeConnection(Inc, Log);
				}
				else if (Test == "Mix")
				{
					TSharedPtr<FMixSrcNode> Src = TestHelper->Pipeline.MakeNode<FMixSrcNode>("Src");
					TSharedPtr<FMixIncNode> Inc = TestHelper->Pipeline.MakeNode<FMixIncNode>("Inc");
					TSharedPtr<FMixLogNode> Log = TestHelper->Pipeline.MakeNode<FMixLogNode>("Log");

					Src->IntValue = 30;
					Src->FltValue = 40.6f;
					Src->NumberOfFrames = 8;

					TestHelper->Pipeline.MakeConnection(Src, Inc);
					TestHelper->Pipeline.MakeConnection(Inc, Log);
				}
				else if (Test == "MultiSrc")
				{
					TSharedPtr<FIntSrcNode> Src1 = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src1");
					TSharedPtr<FIntSrcNode> Src2 = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src2");

					Src1->Value = 50;
					Src1->NumberOfFrames = 10;

					Src2->Value = 60;
					Src2->NumberOfFrames = 20;
				}
				else if (Test == "MultiInput")
				{
					TSharedPtr<FIntSrcNode> Src1 = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src1");
					TSharedPtr<FIntSrcNode> Src2 = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src2");
					TSharedPtr<FIntSumNode> Sum = TestHelper->Pipeline.MakeNode<FIntSumNode>("Sum");

					Src1->Value = 5;
					Src1->NumberOfFrames = 2;

					Src2->Value = 3;
					Src2->NumberOfFrames = 2;

					TestHelper->Pipeline.MakeConnection(Src1, Sum, 0, 0);
					TestHelper->Pipeline.MakeConnection(Src2, Sum, 0, 1);
				}
				else if (Test == "MultiOutput")
				{
					TSharedPtr<FIntSrcNode> Src1 = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src1");
					TSharedPtr<FIntSrcNode> Src2 = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src2");
					TSharedPtr<FIntSumNode> Sum = TestHelper->Pipeline.MakeNode<FIntSumNode>("Sum");
					TSharedPtr<FIntIncNode> Inc1 = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc1");
					TSharedPtr<FIntDecNode> Dec1 = TestHelper->Pipeline.MakeNode<FIntDecNode>("Dec1");
					TSharedPtr<FIntIncNode> Inc2 = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc2");
					TSharedPtr<FIntDecNode> Dec2 = TestHelper->Pipeline.MakeNode<FIntDecNode>("Dec2");

					Src1->Value = 5;
					Src1->NumberOfFrames = 2;

					Src2->Value = 3;
					Src2->NumberOfFrames = 2;

					TestHelper->Pipeline.MakeConnection(Src1, Sum, 0, 0);
					TestHelper->Pipeline.MakeConnection(Src2, Sum, 0, 1);
					TestHelper->Pipeline.MakeConnection(Sum, Inc1, 0, 0);
					TestHelper->Pipeline.MakeConnection(Sum, Dec1, 0, 0);
					TestHelper->Pipeline.MakeConnection(Sum, Inc2, 1, 0);
					TestHelper->Pipeline.MakeConnection(Sum, Dec2, 1, 0);
				}
				else if (Test == "MultiCommonInput")
				{
					TSharedPtr<FIntSrcNode> Src = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src");
					TSharedPtr<FIntSumNode> Sum = TestHelper->Pipeline.MakeNode<FIntSumNode>("Sum");

					Src->Value = 5;
					Src->NumberOfFrames = 2;

					TestHelper->Pipeline.MakeConnection(Src, Sum, 0, 0);
					TestHelper->Pipeline.MakeConnection(Src, Sum, 0, 1);
				}
				else if (Test == "MultiPath")
				{
					TSharedPtr<FIntSrcNode> Src = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src");
					TSharedPtr<FIntIncNode> Inc1 = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc1");
					TSharedPtr<FIntIncNode> Inc2 = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc2");
					TSharedPtr<FIntIncNode> Inc3 = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc3");
					TSharedPtr<FIntSumNode> Sum = TestHelper->Pipeline.MakeNode<FIntSumNode>("Sum");
					TSharedPtr<FIntIncNode> Inc4 = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc4");

					Src->NumberOfFrames = 33;
					Src->Value = 21;

					TestHelper->Pipeline.MakeConnection(Src, Inc1);
					TestHelper->Pipeline.MakeConnection(Inc1, Inc2);
					TestHelper->Pipeline.MakeConnection(Inc1, Inc3);
					TestHelper->Pipeline.MakeConnection(Inc2, Sum, 0, 0);
					TestHelper->Pipeline.MakeConnection(Inc3, Sum, 0, 1);
					TestHelper->Pipeline.MakeConnection(Sum, Inc4);
				}
				else if (Test == "NonDirectInput")
				{
					TSharedPtr<FMixSrcNode> Src = TestHelper->Pipeline.MakeNode<FMixSrcNode>("Src");
					TSharedPtr<FIntIncNode> Inc1 = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc1");
					TSharedPtr<FFltIncNode> Inc2 = TestHelper->Pipeline.MakeNode<FFltIncNode>("Inc2");

					Src->IntValue = 289;
					Src->FltValue = -67.3f;
					Src->NumberOfFrames = 23;

					TestHelper->Pipeline.MakeConnection(Src, Inc1);
					TestHelper->Pipeline.MakeConnection(Inc1, Inc2);
				}
				else if (Test == "NodeError")
				{
					TSharedPtr<FIntSrcNode> Src = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src");
					TSharedPtr<FErrorNode> Err = TestHelper->Pipeline.MakeNode<FErrorNode>("Err");

					Src->Value = 10;
					Src->NumberOfFrames = 5;

					Err->ErrorOnStage = FCString::Atoi(*Params[0]);
					Err->ErrorOnFrame = FCString::Atoi(*Params[1]);
					Err->ErrorCode = FCString::Atoi(*Params[2]);

					TestHelper->Pipeline.MakeConnection(Src, Err);
				}
				else if (Test == "PipelineError")
				{
					int32 TestNumber = FCString::Atoi(*Params[0]);

					if (TestNumber == 1)
					{
						TestHelper->Pipeline.MakeNode<FIntSrcNode>("");
					}
					else if (TestNumber == 2)
					{
						TestHelper->Pipeline.MakeNode<FIntSrcNode>("A.B");
					}
					else if (TestNumber == 3)
					{
						TestHelper->Pipeline.MakeNode<FIntSrcNode>("Reserved");
					}
					else if (TestNumber == 4)
					{
						TestHelper->Pipeline.MakeNode<FIntSrcNode>("Same name");
						TestHelper->Pipeline.MakeNode<FIntSrcNode>("Same name");
					}
					else if (TestNumber == 5)
					{
						TSharedPtr<FIntSrcNode> Src = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src");
						Src->Pins.Add(FPin("", EPinDirection::Output, EPinType::Float));
					}
					else if (TestNumber == 6)
					{
						TSharedPtr<FIntSrcNode> Src = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src");
						Src->Pins.Add(FPin("A.B", EPinDirection::Output, EPinType::Float));
					}
					else if (TestNumber == 7)
					{
						TSharedPtr<FIntSrcNode> Src = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src");
						Src->Pins.Add(FPin("Int Out", EPinDirection::Output, EPinType::Float));
					}
					else if (TestNumber == 8)
					{
						TSharedPtr<FIntSrcNode> Src1 = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src1");
						TSharedPtr<FIntSrcNode> Src2 = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src2");
						TSharedPtr<FIntIncNode> Inc = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc");

						TestHelper->Pipeline.MakeConnection(Src1, Inc);
						TestHelper->Pipeline.MakeConnection(Src2, Inc);
					}
					else if (TestNumber == 9)
					{
						TSharedPtr<FIntSrcNode> Src1 = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src1");
						TSharedPtr<FIntSrcNode> Src2 = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src2");
						TSharedPtr<FIntsToFltNode> IntsToFlt = TestHelper->Pipeline.MakeNode<FIntsToFltNode>("IntsToFlt");
						TSharedPtr<FIntIncNode> Inc = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc");

						TestHelper->Pipeline.MakeConnection(Src1, IntsToFlt, 0, 0);
						TestHelper->Pipeline.MakeConnection(Src2, IntsToFlt, 0, 1);
						TestHelper->Pipeline.MakeConnection(IntsToFlt, Inc);
					}
					else if (TestNumber == 10)
					{
						TSharedPtr<FIntSrcNode> Src = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src");
						TSharedPtr<FIntSumNode> Sum = TestHelper->Pipeline.MakeNode<FIntSumNode>("Sum");

						TestHelper->Pipeline.MakeConnection(Src, Sum);
					}
					else if (TestNumber == 11)
					{
						TSharedPtr<FIntIncNode> Inc1 = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc1");
						TSharedPtr<FIntIncNode> Inc2 = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc2");
						TSharedPtr<FIntIncNode> Inc3 = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc3");
						TSharedPtr<FIntIncNode> Inc4 = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc4");
						TSharedPtr<FIntIncNode> Inc5 = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc5");

						TestHelper->Pipeline.MakeConnection(Inc1, Inc2);
						TestHelper->Pipeline.MakeConnection(Inc2, Inc3);
						TestHelper->Pipeline.MakeConnection(Inc3, Inc4);
						TestHelper->Pipeline.MakeConnection(Inc4, Inc5);
						TestHelper->Pipeline.MakeConnection(Inc3, Inc1);
					}
					else
					{
						bIsOK &= TestTrue(TEXT("Known test number"), false);
					}
				}
				else if (Test == "Queue")
				{
					TSharedPtr<FIntSrcNode> Src = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src");

					Src->Value = 90;
					Src->NumberOfFrames = FCString::Atoi(*Params[0]);
					Src->QueueSize = FCString::Atoi(*Params[2]);

					TSharedPtr<FIntIncNode> Inc1 = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc1");
					Inc1->QueueSize = FCString::Atoi(*Params[2]);
					TestHelper->Pipeline.MakeConnection(Src, Inc1);

					TSharedPtr<FNode> UpstreamNode = Src;
					for (int32 Index = 0; Index < FCString::Atoi(*Params[1]); ++Index)
					{
						TSharedPtr<FIntIncNode> Inc2N = TestHelper->Pipeline.MakeNode<FIntIncNode>(FString::Printf(TEXT("Inc2-%04i"), Index));
						Inc2N->QueueSize = FCString::Atoi(*Params[2]);

						TestHelper->Pipeline.MakeConnection(UpstreamNode, Inc2N);
						UpstreamNode = Inc2N;
					}
				}
				else if (Test == "StartEndFrame")
				{
					TSharedPtr<FIntSrcNode> Src = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src");
					TSharedPtr<FIntIncNode> Inc = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc");
					TSharedPtr<FIntLogNode> Log = TestHelper->Pipeline.MakeNode<FIntLogNode>("Log");

					Src->Value = 10;
					Src->NumberOfFrames = 20;

					TestHelper->Pipeline.MakeConnection(Src, Inc);
					TestHelper->Pipeline.MakeConnection(Inc, Log);

					TestHelper->StartFrame = 3;
					TestHelper->EndFrame = 7;
				}
				else if (Test == "RandomNodeOrder" || Test == "LinearNodeOrder")
				{
					TSharedPtr<FIntSrcNode> Src = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src");

					Src->Value = 10;
					Src->NumberOfFrames = 50;

					TSharedPtr<FNode> UpstreamNode = Src;
					for (int32 Index = 0; Index < 100; ++Index)
					{
						TSharedPtr<FProcessCountNode> ProcessCountNode = TestHelper->Pipeline.MakeNode<FProcessCountNode>(FString::Printf(TEXT("ProcessCount-%04i"), Index));

						TestHelper->Pipeline.MakeConnection(UpstreamNode, ProcessCountNode);
						UpstreamNode = ProcessCountNode;

						TestHelper->ProcessCountNodes.Add(ProcessCountNode);
					}

					TestHelper->StartFrame = 0;
					TestHelper->EndFrame = 50;

					TestHelper->bProcessNodesInRandomOrder = (Test == "RandomNodeOrder");
				}
				else if (Test == "DropFrame")
				{
					TSharedPtr<FIntSrcNode> Src = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src");
					TSharedPtr<FDropFrameNode> Drop1 = TestHelper->Pipeline.MakeNode<FDropFrameNode>("Drop1");
					TSharedPtr<FIntIncNode> Inc = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc");
					TSharedPtr<FDropFrameNode> Drop2 = TestHelper->Pipeline.MakeNode<FDropFrameNode>("Drop2");
					TSharedPtr<FIntLogNode> Log = TestHelper->Pipeline.MakeNode<FIntLogNode>("Log");

					Src->Value = 400;
					Src->NumberOfFrames = 20;

					Drop1->DropEvery = 2;
					Drop2->DropEvery = 5;

					TestHelper->Pipeline.MakeConnection(Src, Drop1);
					TestHelper->Pipeline.MakeConnection(Drop1, Inc);
					TestHelper->Pipeline.MakeConnection(Inc, Drop2);
					TestHelper->Pipeline.MakeConnection(Drop2, Log);

					TestHelper->bDropFramesPresent = true;
				}
				else if (Test == "Buffer")
				{
					TSharedPtr<FIntSrcNode> Src = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src");
					TSharedPtr<FBufferNode> Buf1 = TestHelper->Pipeline.MakeNode<FBufferNode>("Buf1");
					TSharedPtr<FIntIncNode> Inc = TestHelper->Pipeline.MakeNode<FIntIncNode>("Inc");
					TSharedPtr<FBufferNode> Buf2 = TestHelper->Pipeline.MakeNode<FBufferNode>("Buf2");
					TSharedPtr<FIntLogNode> Log = TestHelper->Pipeline.MakeNode<FIntLogNode>("Log");

					Src->Value = 10;
					Src->NumberOfFrames = 5;

					TestHelper->Pipeline.MakeConnection(Src, Buf1);
					TestHelper->Pipeline.MakeConnection(Buf1, Inc);
					TestHelper->Pipeline.MakeConnection(Inc, Buf2);
					TestHelper->Pipeline.MakeConnection(Buf2, Log);
				}
				else if (Test == "NonAsync" || Test == "Async")
				{
					TSharedPtr<FIntSrcNode> Src = TestHelper->Pipeline.MakeNode<FIntSrcNode>("Src");

					TSharedPtr<FNode> Inc;
					if (Test == "NonAsync")
					{
						Inc = TestHelper->Pipeline.MakeNode<FSlowIntIncNode>("Inc");
					}
					else
					{
						Inc = TestHelper->Pipeline.MakeNode<FAsyncNode<FSlowIntIncNode>>(3, "Inc");
					}

					TSharedPtr<FIntLogNode> Log = TestHelper->Pipeline.MakeNode<FIntLogNode>("Log");

					Src->Value = 10;
					Src->NumberOfFrames = 5;

					TestHelper->Pipeline.MakeConnection(Src, Inc);
					TestHelper->Pipeline.MakeConnection(Inc, Log);
				}
				else
				{
					bIsOK &= TestTrue(TEXT("Known test"), false);
				}

				if (bIsOK)
				{
					if (Method == "PushSync")
					{
						TestHelper->Run(EPipelineMode::PushSync);
					}
					else if (Method == "PushAsync")
					{
						TestHelper->Run(EPipelineMode::PushAsync);
					}
					else if (Method == "PushAsyncNodes")
					{
						TestHelper->Run(EPipelineMode::PushAsyncNodes);
					}
					else
					{
						bIsOK &= TestTrue(TEXT("Known method"), false);
					}
				}
			}
		}
		else if (Stage == "Stage2")
		{
			ADD_LATENT_AUTOMATION_COMMAND(FPipelineTestNodesComplete(Test));
		}
		else if (Stage == "Stage3")
		{
			bIsOK &= TestValid(TEXT("Test helper set"), TestHelper);

			if (bIsOK)
			{
				bIsOK &= TestEqual(TEXT("Process complete count"), TestHelper->ProcessCompleteCount, 1);

				if (Test != "NodeError" && Test != "PipelineError")
				{
					bIsOK &= TestEqual(TEXT("Exit status"), TestHelper->ExitStatus, EPipelineExitStatus::Ok);
					bIsOK &= TestEqual(TEXT("Error node code"), TestHelper->ErrorNodeCode, -1);
				}

				if (Test == "Int" || Test == "Buffer")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 5);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						TEST_PIPELINE_DATA(Frame, int32, "Src.Int Out", 10 + Frame);
						TEST_PIPELINE_DATA(Frame, int32, "Inc.Int Out", 10 + Frame + 1);

						bIsOK &= TestTrue(TEXT("Data is correct type"), !TestHelper->PipelineData[Frame]->HasData<float>("Inc.Int Out"));
						bIsOK &= TestTrue(TEXT("Invalid data present"), !TestHelper->PipelineData[Frame]->HasData<int32>("Inc.BOGUS"));
						bIsOK &= TestTrue(TEXT("Invalid data present"), !TestHelper->PipelineData[Frame]->HasData<int32>("BOGUS.Int Out"));
					}
				}
				else if (Test == "Float")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 7);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						TEST_PIPELINE_DATA(Frame, float, "Src.Flt Out", 20.4f + Frame);
						TEST_PIPELINE_DATA(Frame, float, "Inc.Flt Out", 20.4f + Frame + 0.1f);

						bIsOK &= TestTrue(TEXT("Data is correct type"), !TestHelper->PipelineData[Frame]->HasData<int32>("Inc.Flt Out"));
						bIsOK &= TestTrue(TEXT("Invalid data present"), !TestHelper->PipelineData[Frame]->HasData<float>("Inc.BOGUS"));
						bIsOK &= TestTrue(TEXT("Invalid data present"), !TestHelper->PipelineData[Frame]->HasData<float>("BOGUS.Flt Out"));
					}
				}
				else if (Test == "Mix")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 8);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						TEST_PIPELINE_DATA(Frame, int32, "Src.Int Out", 30 + Frame);
						TEST_PIPELINE_DATA(Frame, float, "Src.Flt Out", 40.6f + Frame);
						TEST_PIPELINE_DATA(Frame, int32, "Inc.Int Out", 30 + Frame + 1);
						TEST_PIPELINE_DATA(Frame, float, "Inc.Flt Out", 40.6f + Frame + 0.1f);

						bIsOK &= TestTrue(TEXT("Data is correct type"), !TestHelper->PipelineData[Frame]->HasData<float>("Inc.Int Out"));
						bIsOK &= TestTrue(TEXT("Invalid data present"), !TestHelper->PipelineData[Frame]->HasData<int32>("Inc.BOGUS"));
						bIsOK &= TestTrue(TEXT("Invalid data present"), !TestHelper->PipelineData[Frame]->HasData<int32>("BOGUS.Int Out"));

						bIsOK &= TestTrue(TEXT("Data is correct type"), !TestHelper->PipelineData[Frame]->HasData<int32>("Inc.Flt Out"));
						bIsOK &= TestTrue(TEXT("Invalid data present"), !TestHelper->PipelineData[Frame]->HasData<float>("Inc.BOGUS"));
						bIsOK &= TestTrue(TEXT("Invalid data present"), !TestHelper->PipelineData[Frame]->HasData<float>("BOGUS.Flt Out"));
					}
				}
				else if (Test == "MultiSrc")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 10);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						TEST_PIPELINE_DATA(Frame, int32, "Src1.Int Out", 50 + Frame);
						TEST_PIPELINE_DATA(Frame, int32, "Src2.Int Out", 60 + Frame);
					}
				}
				else if (Test == "MultiInput")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 2);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						TEST_PIPELINE_DATA(Frame, int32, "Src1.Int Out", 5 + Frame);
						TEST_PIPELINE_DATA(Frame, int32, "Src2.Int Out", 3 + Frame);
						TEST_PIPELINE_DATA(Frame, int32, "Sum.Int1 Out", (5 + Frame) + (3 + Frame));
						TEST_PIPELINE_DATA(Frame, int32, "Sum.Int2 Out", (5 + Frame) - (3 + Frame));
					}
				}
				else if (Test == "MultiOutput")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 2);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						TEST_PIPELINE_DATA(Frame, int32, "Src1.Int Out", 5 + Frame);
						TEST_PIPELINE_DATA(Frame, int32, "Src2.Int Out", 3 + Frame);
						TEST_PIPELINE_DATA(Frame, int32, "Sum.Int1 Out", (5 + Frame) + (3 + Frame));
						TEST_PIPELINE_DATA(Frame, int32, "Sum.Int2 Out", (5 + Frame) - (3 + Frame));
						TEST_PIPELINE_DATA(Frame, int32, "Inc1.Int Out", (5 + Frame) + (3 + Frame) + 1);
						TEST_PIPELINE_DATA(Frame, int32, "Dec1.Int Out", (5 + Frame) + (3 + Frame) - 1);
						TEST_PIPELINE_DATA(Frame, int32, "Inc2.Int Out", (5 + Frame) - (3 + Frame) + 1);
						TEST_PIPELINE_DATA(Frame, int32, "Dec2.Int Out", (5 + Frame) - (3 + Frame) - 1);
					}
				}
				else if (Test == "MultiCommonInput")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 2);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						TEST_PIPELINE_DATA(Frame, int32, "Src.Int Out", 5 + Frame);
						TEST_PIPELINE_DATA(Frame, int32, "Sum.Int1 Out", (5 + Frame) + (5 + Frame));
						TEST_PIPELINE_DATA(Frame, int32, "Sum.Int2 Out", (5 + Frame) - (5 + Frame));
					}
				}
				else if (Test == "NonDirectInput")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 23);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						TEST_PIPELINE_DATA(Frame, int32, "Src.Int Out", 289 + Frame);
						TEST_PIPELINE_DATA(Frame, float, "Src.Flt Out", -67.3f + Frame);
						TEST_PIPELINE_DATA(Frame, int32, "Inc1.Int Out", 289 + Frame + 1);
						TEST_PIPELINE_DATA(Frame, float, "Inc2.Flt Out", -67.3f + Frame + 0.1f);
					}
				}
				else if (Test == "MultiPath")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 33);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						TEST_PIPELINE_DATA(Frame, int32, "Src.Int Out", 21 + Frame);
						TEST_PIPELINE_DATA(Frame, int32, "Inc1.Int Out", 21 + Frame + 1);
						TEST_PIPELINE_DATA(Frame, int32, "Inc2.Int Out", 21 + Frame + 1 + 1);
						TEST_PIPELINE_DATA(Frame, int32, "Inc3.Int Out", 21 + Frame + 1 + 1);
						TEST_PIPELINE_DATA(Frame, int32, "Sum.Int1 Out", (21 + Frame + 1 + 1) * 2);
						TEST_PIPELINE_DATA(Frame, int32, "Inc4.Int Out", (21 + Frame + 1 + 1) * 2 + 1);
					}
				}
				else if (Test == "NodeError")
				{
					if (FCString::Atoi(*Params[0]) == 0)
					{
						bIsOK &= TestEqual(TEXT("Exit status"), TestHelper->ExitStatus, EPipelineExitStatus::StartError);
						bIsOK &= TestEqual(TEXT("Error node code"), TestHelper->ErrorNodeCode, FCString::Atoi(*Params[2]));
						bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 0);
					}
					else if (FCString::Atoi(*Params[0]) == 2)
					{
						bIsOK &= TestEqual(TEXT("Exit status"), TestHelper->ExitStatus, EPipelineExitStatus::EndError);
						bIsOK &= TestEqual(TEXT("Error node code"), TestHelper->ErrorNodeCode, FCString::Atoi(*Params[2]));
						bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 5);
					}
					else
					{
						if (FCString::Atoi(*Params[0]) == 1)
						{
							bIsOK &= TestEqual(TEXT("Exit status"), TestHelper->ExitStatus, EPipelineExitStatus::ProcessError);
							bIsOK &= TestEqual(TEXT("Error node code"), TestHelper->ErrorNodeCode, FCString::Atoi(*Params[2]));
							bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, FCString::Atoi(*Params[1]));
						}
						else
						{ 
							bIsOK &= TestEqual(TEXT("Exit status"), TestHelper->ExitStatus, EPipelineExitStatus::Ok);
							bIsOK &= TestEqual(TEXT("Error node code"), TestHelper->ErrorNodeCode, -1);
							bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 5);
						}
				
						for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
						{
							TEST_PIPELINE_DATA(Frame, int32, "Src.Int Out", 10 + Frame);
							TEST_PIPELINE_DATA(Frame, int32, "Err.Int Out", 10 + Frame + 1);
						}
					}
				}
				else if (Test == "PipelineError")
				{
					int32 TestNumber = FCString::Atoi(*Params[0]);

					if (TestNumber == 1 || TestNumber == 2 || TestNumber == 3)
					{
						bIsOK &= TestEqual(TEXT("Exit status"), TestHelper->ExitStatus, EPipelineExitStatus::InvalidNodeName);
					}
					else if (TestNumber == 4)
					{
						bIsOK &= TestEqual(TEXT("Exit status"), TestHelper->ExitStatus, EPipelineExitStatus::DuplicateNodeName);
					}
					else if (TestNumber == 5 || TestNumber == 6)
					{
						bIsOK &= TestEqual(TEXT("Exit status"), TestHelper->ExitStatus, EPipelineExitStatus::InvalidPinName);
					}
					else if (TestNumber == 7)
					{
						bIsOK &= TestEqual(TEXT("Exit status"), TestHelper->ExitStatus, EPipelineExitStatus::DuplicatePinName);
					}
					else if (TestNumber == 8 || TestNumber == 9)
					{
						bIsOK &= TestEqual(TEXT("Exit status"), TestHelper->ExitStatus, EPipelineExitStatus::AmbiguousConnection);
					}
					else if (TestNumber == 10)
					{
						bIsOK &= TestEqual(TEXT("Exit status"), TestHelper->ExitStatus, EPipelineExitStatus::Unconnected);
					}
					else if (TestNumber == 11)
					{
						bIsOK &= TestEqual(TEXT("Exit status"), TestHelper->ExitStatus, EPipelineExitStatus::LoopConnection);
					}
					else
					{
						bIsOK &= TestTrue(TEXT("Known test number"), false);
					}

					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 0);
				}
				else if (Test == "Queue")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, FCString::Atoi(*Params[0]));

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						TEST_PIPELINE_DATA(Frame, int32, "Src.Int Out", 90 + Frame);
						TEST_PIPELINE_DATA(Frame, int32, "Inc1.Int Out", 90 + Frame + 1);

						for (int32 Index = 0; Index < FCString::Atoi(*Params[1]); ++Index)
						{
							FString Inc2N = FString::Printf(TEXT("Inc2-%04i.Int Out"), Index);
							TEST_PIPELINE_DATA(Frame, int32, Inc2N, 90 + Frame + 1 + Index);
						}
					}
				}
				else if (Test == "StartEndFrame")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 4);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						TEST_PIPELINE_DATA(Frame, int32, "Src.Int Out", 10 + Frame + 3);
						TEST_PIPELINE_DATA(Frame, int32, "Inc.Int Out", 10 + Frame + 3 + 1);
					}
				}
				else if (Test == "RandomNodeOrder" || Test == "LinearNodeOrder")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 50);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						for (int32 Index = 0; Index < 100; ++Index)
						{
							FString ProcessCountN = FString::Printf(TEXT("ProcessCount-%04i.Int Out"), Index);
							TEST_PIPELINE_DATA(Frame, int32, ProcessCountN, 10 + Frame + 1 + Index);
						}
					}
				}
				else if (Test == "DropFrame")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 8);

					int PresentFrames = 0;
					for (int32 AllFrames = 0; AllFrames < 20; ++AllFrames)
					{
						if (AllFrames % 2 == 0) continue;
						if (AllFrames % 5 == 0) continue;

						TEST_PIPELINE_DATA(PresentFrames, int32, "Src.Int Out", 400 + AllFrames);
						TEST_PIPELINE_DATA(PresentFrames, int32, "Inc.Int Out", 400 + AllFrames + 1);

						PresentFrames++;
					}

					bIsOK &= TestEqual(TEXT("Present frame completed count"), PresentFrames, TestHelper->FrameCompleteCount);
				}
				else if (Test == "NonAsync" || Test == "Async")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 5);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						TEST_PIPELINE_DATA(Frame, int32, "Src.Int Out", 10 + Frame);
						TEST_PIPELINE_DATA(Frame, int32, "Inc.Int Out", 10 + Frame + 1);
					}

					if (Test == "NonAsync")
					{
						bIsOK &= TestTrue(TEXT("Expected speed"), TestHelper->EndTime - TestHelper->StartTime > 4); // should take ~5 secs
					}
					else
					{
						bIsOK &= TestTrue(TEXT("Expected speed"), TestHelper->EndTime - TestHelper->StartTime < 3); // should take ~2 secs
					}
				}
				else
				{
					bIsOK &= TestTrue(TEXT("Known test"), false);
				}
			}

			TestHelper = nullptr;
		}
		else
		{
			bIsOK &= TestTrue(TEXT("Known stage"), false);
		}
	}

	return bIsOK;
}

bool FPipelineTestAdvancedNodes::RunTest(const FString& InTestCommand)
{
	bool bIsOK = true;

	TArray<FString> Tokens;
	InTestCommand.ParseIntoArray(Tokens, TEXT(" "), true);
	bIsOK &= TestEqual<int32>(TEXT("Well formed Parameters"), Tokens.Num(), 3);

	if (bIsOK)
	{
		TArray<FString> Params;
		Tokens[0].ParseIntoArray(Params, TEXT("-"), true);
		FString Test = Params[0];
		Params.RemoveAt(0);

		FString Method = Tokens[1];
		FString Stage = Tokens[2];

		if (Stage == "Stage1")
		{
			bIsOK &= TestInvalid(TEXT("Test helper set"), TestHelper);

			if (bIsOK)
			{
				TestHelper = MakeShared<FPipelineTestHelper>();

				FString InputDir = FPaths::ProjectContentDir() + "/TestInput";
				FString OutputDir = FPaths::ProjectIntermediateDir() + "/TestOutput";

				if (Test == "Hyprsense")
				{
					TSharedPtr<FUEImageLoadNode> Load = TestHelper->Pipeline.MakeNode<FUEImageLoadNode>("Load");
					TSharedPtr<FHyprsenseNode> Track = TestHelper->Pipeline.MakeNode<FHyprsenseNode>("Track");
					TSharedPtr<FBurnContoursNode> Burn = TestHelper->Pipeline.MakeNode<FBurnContoursNode>("Burn");
					TSharedPtr<FUEImageSaveNode> Save = TestHelper->Pipeline.MakeNode<FUEImageSaveNode>("Save");

					Load->FilePath = InputDir + "/Hyprsense/stacy-colour-100/%04d.png";
					Load->FrameNumberOffset = 0;

					UNeuralNetwork* FaceTracker{}, *FaceDetector{}, *EyebrowTracker{}, *EyeTracker{}, *LipsTracker{}, *NasolabialTracker{};

					FString FaceTrackerModelPath = "/MetaHuman/GenericTracker/FaceTracker.FaceTracker";
					FaceTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *FaceTrackerModelPath); // Must be done in game thread

					FString FaceDetectorModelPath = "/MetaHuman/GenericTracker/FaceDetector.FaceDetector";
					FaceDetector = LoadObject<UNeuralNetwork>(GetTransientPackage(), *FaceDetectorModelPath); // Must be done in game thread

					FString EyebrowTrackerModelPath = "/MetaHuman/GenericTracker/LeftBrow.LeftBrow";
					EyebrowTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *EyebrowTrackerModelPath); // Must be done in game thread

					FString EyeTrackerModelPath = "/MetaHuman/GenericTracker/LeftEye.LeftEye";
					EyeTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *EyeTrackerModelPath); // Must be done in game thread

					FString LipsTrackerModelPath = "/MetaHuman/GenericTracker/Lips.Lips";
					LipsTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *LipsTrackerModelPath); // Must be done in game thread

					FString NasolabialTrackerModelPath = "/MetaHuman/GenericTracker/Nasolabial.Nasolabial";
					NasolabialTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *NasolabialTrackerModelPath); // Must be done in game thread

					Track->SetTrackers(FaceTracker, FaceDetector, EyebrowTracker, EyeTracker, LipsTracker, NasolabialTracker);

					Save->FilePath = OutputDir + "/Hyprsense/%04d.png";
					Save->FrameNumberOffset = 0;

					TestHelper->Pipeline.MakeConnection(Load, Track);
					TestHelper->Pipeline.MakeConnection(Track, Burn);
					TestHelper->Pipeline.MakeConnection(Burn, Save);
				}
				else if (Test == "HyprsenseRealtime")
				{
					TSharedPtr<FUEImageLoadNode> Load = TestHelper->Pipeline.MakeNode<FUEImageLoadNode>("Load");
					TSharedPtr<FHyprsenseRealtimeNode> Track = TestHelper->Pipeline.MakeNode<FHyprsenseRealtimeNode>("Track");
					TSharedPtr<FBurnContoursNode> Burn = TestHelper->Pipeline.MakeNode<FBurnContoursNode>("Burn");
					TSharedPtr<FUEImageSaveNode> Save = TestHelper->Pipeline.MakeNode<FUEImageSaveNode>("Save");

					Load->FilePath = InputDir + "/Hyprsense/stacy-colour-100/%04d.png";
					Load->FrameNumberOffset = 0;

					UNeuralNetwork* FaceTracker{}, * FaceDetector{}, * EyebrowTracker{}, * EyeTracker{}, * LipsTracker{}, * LipsNasolabialTracker{};

					FString FaceTrackerModelPath = "/MetaHuman/GenericTracker/FaceTracker.FaceTracker";
					FaceTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *FaceTrackerModelPath); // Must be done in game thread

					FString FaceDetectorModelPath = "/MetaHuman/GenericTracker/FaceDetector.FaceDetector";
					FaceDetector = LoadObject<UNeuralNetwork>(GetTransientPackage(), *FaceDetectorModelPath); // Must be done in game thread

					FString EyebrowTrackerModelPath = "/MetaHuman/GenericTracker/LeftBrowRealtime.LeftBrowRealtime";
					EyebrowTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *EyebrowTrackerModelPath); // Must be done in game thread

					FString EyeTrackerModelPath = "/MetaHuman/GenericTracker/LeftEyeRealtime.LeftEyeRealtime";
					EyeTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *EyeTrackerModelPath); // Must be done in game thread

					FString LipsNasolabialTrackerModelPath = "/MetaHuman/GenericTracker/LipsNasolabialRealtime.LipsNasolabialRealtime";
					LipsNasolabialTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *LipsNasolabialTrackerModelPath); // Must be done in game thread


					Track->SetTrackers(FaceTracker, FaceDetector, EyebrowTracker, EyeTracker, LipsNasolabialTracker);

					Save->FilePath = OutputDir + "/HyprsenseRealtime/%04d.png";
					Save->FrameNumberOffset = 0;

					TestHelper->Pipeline.MakeConnection(Load, Track);
					TestHelper->Pipeline.MakeConnection(Track, Burn);
					TestHelper->Pipeline.MakeConnection(Burn, Save);
				}
				else if (Test == "HyprsensePythonCompare")
				{
					TSharedPtr<FUEImageLoadNode> Load = TestHelper->Pipeline.MakeNode<FUEImageLoadNode>("Load");
					TSharedPtr<FHyprsenseNode> Track = TestHelper->Pipeline.MakeNode<FHyprsenseNode>("Track");
					TSharedPtr<FHyprsenseTestNode> Compare = TestHelper->Pipeline.MakeNode<FHyprsenseTestNode>("Compare");

					Load->FilePath = InputDir + "/Hyprsense/stacy-colour-100/%04d.png";
					Load->FrameNumberOffset = 0;

					UNeuralNetwork* FaceTracker{}, * FaceDetector{}, * EyebrowTracker{}, * EyeTracker{}, * LipsTracker{}, * NasolabialTracker{};

					FString FaceTrackerModelPath = "/MetaHuman/GenericTracker/FaceTracker.FaceTracker";
					FaceTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *FaceTrackerModelPath); // Must be done in game thread

					FString FaceDetectorModelPath = "/MetaHuman/GenericTracker/FaceDetector.FaceDetector";
					FaceDetector = LoadObject<UNeuralNetwork>(GetTransientPackage(), *FaceDetectorModelPath); // Must be done in game thread

					FString EyebrowTrackerModelPath = "/MetaHuman/GenericTracker/LeftBrow.LeftBrow";
					EyebrowTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *EyebrowTrackerModelPath); // Must be done in game thread

					FString EyeTrackerModelPath = "/MetaHuman/GenericTracker/LeftEye.LeftEye";
					EyeTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *EyeTrackerModelPath); // Must be done in game thread

					FString LipsTrackerModelPath = "/MetaHuman/GenericTracker/Lips.Lips";
					LipsTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *LipsTrackerModelPath); // Must be done in game thread

					FString NasolabialTrackerModelPath = "/MetaHuman/GenericTracker/Nasolabial.Nasolabial";
					NasolabialTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *NasolabialTrackerModelPath); // Must be done in game thread

					Track->SetTrackers(FaceTracker, FaceDetector, EyebrowTracker, EyeTracker, LipsTracker, NasolabialTracker);

					Compare->InJsonFilePath = InputDir + "/Hyprsense/PythonContours.json";
					Compare->OutJsonFilePath = OutputDir + "/HyprsenseCompare/UnrealPythonDiff.json";

					TestHelper->Pipeline.MakeConnection(Load, Track);
					TestHelper->Pipeline.MakeConnection(Track, Compare);

				}
				else if (Test == "RLibVRefinement")
				{
					TSharedPtr<FUEImageLoadNode> Load = TestHelper->Pipeline.MakeNode<FUEImageLoadNode>("Load");
					TSharedPtr<FHyprsenseNode> Generic = TestHelper->Pipeline.MakeNode<FHyprsenseNode>("Generic");
					TSharedPtr<FOffsetContoursNode> Offset = TestHelper->Pipeline.MakeNode<FOffsetContoursNode>("Offset");
					TSharedPtr<FRLibVRefinementTrackerNode> Refinement = TestHelper->Pipeline.MakeNode<FRLibVRefinementTrackerNode>("Refinement");
					TSharedPtr<FBurnContoursNode> BurnGeneric = TestHelper->Pipeline.MakeNode<FBurnContoursNode>("BurnGeneric");
					TSharedPtr<FBurnContoursNode> BurnRefinement = TestHelper->Pipeline.MakeNode<FBurnContoursNode>("BurnRefinement");
					TSharedPtr<FUEImageCropNode> CropGeneric = TestHelper->Pipeline.MakeNode<FUEImageCropNode>("CropGeneric");
					TSharedPtr<FUEImageCropNode> CropRefinement = TestHelper->Pipeline.MakeNode<FUEImageCropNode>("CropRefinement");
					TSharedPtr<FUEImageCompositeNode> Composite = TestHelper->Pipeline.MakeNode<FUEImageCompositeNode>("Composite");
					TSharedPtr<FUEImageSaveNode> Save = TestHelper->Pipeline.MakeNode<FUEImageSaveNode>("Save");

					Load->FilePath = InputDir + "/Stacy-colour/%04d.png";
					Load->FrameNumberOffset = 0;

					UNeuralNetwork* FaceTracker{}, * FaceDetector{}, * EyebrowTracker{}, * EyeTracker{}, * LipsTracker{}, * NasolabialTracker{};

					FString FaceTrackerModelPath = "/MetaHuman/GenericTracker/FaceTracker.FaceTracker";
					FaceTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *FaceTrackerModelPath); // Must be done in game thread

					FString FaceDetectorModelPath = "/MetaHuman/GenericTracker/FaceDetector.FaceDetector";
					FaceDetector = LoadObject<UNeuralNetwork>(GetTransientPackage(), *FaceDetectorModelPath); // Must be done in game thread

					FString EyebrowTrackerModelPath = "/MetaHuman/GenericTracker/LeftBrow.LeftBrow";
					EyebrowTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *EyebrowTrackerModelPath); // Must be done in game thread

					FString EyeTrackerModelPath = "/MetaHuman/GenericTracker/LeftEye.LeftEye";
					EyeTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *EyeTrackerModelPath); // Must be done in game thread

					FString LipsTrackerModelPath = "/MetaHuman/GenericTracker/Lips.Lips";
					LipsTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *LipsTrackerModelPath); // Must be done in game thread

					FString NasolabialTrackerModelPath = "/MetaHuman/GenericTracker/Nasolabial.Nasolabial";
					NasolabialTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *NasolabialTrackerModelPath); // Must be done in game thread

					Generic->SetTrackers(FaceTracker, FaceDetector, EyebrowTracker, EyeTracker, LipsTracker, NasolabialTracker);

					Offset->RandomOffset = FVector2D(0, 2);

					Refinement->ModelFile = InputDir + "/RLibVRefinement/stacey_refiner.ref";

					BurnGeneric->Size = 2;

					BurnRefinement->Size = BurnGeneric->Size;

					CropGeneric->X = 120;
					CropGeneric->Y = 170;
					CropGeneric->Width = 250;
					CropGeneric->Height = 360;

					CropRefinement->X = CropGeneric->X;
					CropRefinement->Y = CropGeneric->Y;
					CropRefinement->Width = CropGeneric->Width;
					CropRefinement->Height = CropGeneric->Height;

					Save->FilePath = OutputDir + "/RLibVRefinement/%04d.png";
					Save->FrameNumberOffset = 0;
					
					TestHelper->Pipeline.MakeConnection(Load, Generic);
					TestHelper->Pipeline.MakeConnection(Generic, Offset);
					TestHelper->Pipeline.MakeConnection(Offset, BurnGeneric);
					TestHelper->Pipeline.MakeConnection(BurnGeneric, CropGeneric);
					TestHelper->Pipeline.MakeConnection(Offset, Refinement);
					TestHelper->Pipeline.MakeConnection(Refinement, BurnRefinement);
					TestHelper->Pipeline.MakeConnection(BurnRefinement, CropRefinement);
					TestHelper->Pipeline.MakeConnection(CropGeneric, Composite, 0, 0);
					TestHelper->Pipeline.MakeConnection(CropRefinement, Composite, 0, 1);
					TestHelper->Pipeline.MakeConnection(Composite, Save);
				}
				else if (Test == "Depth")
				{
					TSharedPtr<FDepthLoadNode> Load = TestHelper->Pipeline.MakeNode<FDepthLoadNode>("Load");
					TSharedPtr<FDepthToUEImageNode> Convert = TestHelper->Pipeline.MakeNode<FDepthToUEImageNode>("Convert");
					TSharedPtr<FUEImageSaveNode> Save = TestHelper->Pipeline.MakeNode<FUEImageSaveNode>("Save");

					Load->FilePath = InputDir + "/Stacy-depth/%04d.exr";

					Convert->Min = 20;
					Convert->Max = 30;

					Save->FilePath = OutputDir + "/Depth/%04d.png";

					TestHelper->Pipeline.MakeConnection(Load, Convert);
					TestHelper->Pipeline.MakeConnection(Convert, Save);
				}
				else if (Test == "FaceTrackerIPhone")
				{
					TSharedPtr<FUEImageLoadNode> Colour = TestHelper->Pipeline.MakeNode<FUEImageLoadNode>("Image");
					TSharedPtr<FUEImageToUEGrayImageNode> Gray = TestHelper->Pipeline.MakeNode<FUEImageToUEGrayImageNode>("Gray");
					TSharedPtr<FHyprsenseNode> Track = TestHelper->Pipeline.MakeNode<FHyprsenseNode>("Track");
					TSharedPtr<FRLibVRefinementTrackerNode> Refine = TestHelper->Pipeline.MakeNode<FRLibVRefinementTrackerNode>("Refine");
					TSharedPtr<FDepthLoadNode> Depth = TestHelper->Pipeline.MakeNode<FDepthLoadNode>("Depth");
					TSharedPtr<FFaceTrackerIPhoneNode> NLS = TestHelper->Pipeline.MakeNode<FFaceTrackerIPhoneNode>("NLS");

					Colour->FilePath = InputDir + "/Stacy-colour/%04d.png";

					UNeuralNetwork* FaceTracker{}, * FaceDetector{}, * EyebrowTracker{}, * EyeTracker{}, * LipsTracker{}, * NasolabialTracker{};

					FString FaceTrackerModelPath = "/MetaHuman/GenericTracker/FaceTracker.FaceTracker";
					FaceTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *FaceTrackerModelPath); // Must be done in game thread

					FString FaceDetectorModelPath = "/MetaHuman/GenericTracker/FaceDetector.FaceDetector";
					FaceDetector = LoadObject<UNeuralNetwork>(GetTransientPackage(), *FaceDetectorModelPath); // Must be done in game thread

					FString EyebrowTrackerModelPath = "/MetaHuman/GenericTracker/LeftBrow.LeftBrow";
					EyebrowTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *EyebrowTrackerModelPath); // Must be done in game thread

					FString EyeTrackerModelPath = "/MetaHuman/GenericTracker/LeftEye.LeftEye";
					EyeTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *EyeTrackerModelPath); // Must be done in game thread

					FString LipsTrackerModelPath = "/MetaHuman/GenericTracker/Lips.Lips";
					LipsTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *LipsTrackerModelPath); // Must be done in game thread

					FString NasolabialTrackerModelPath = "/MetaHuman/GenericTracker/Nasolabial.Nasolabial";
					NasolabialTracker = LoadObject<UNeuralNetwork>(GetTransientPackage(), *NasolabialTrackerModelPath); // Must be done in game thread

					Track->SetTrackers(FaceTracker, FaceDetector, EyebrowTracker, EyeTracker, LipsTracker, NasolabialTracker);

					Refine->ModelFile = InputDir + "/RLibVRefinement/stacey_refiner.ref";

					Depth->FilePath = InputDir + "/Stacy-depth/%04d.exr";

					NLS->ConfigurationDirectory = IPluginManager::Get().FindPlugin(UE_PLUGIN_NAME)->GetContentDir() + "/Solver/iphone/facialsolver";
					NLS->DNAFile = InputDir + "/FaceTracker/StaceyC_autorig_iphone_rigCalibration.dna";
					NLS->Calibrations.SetNum(2);

					NLS->Calibrations[0].Name = "colour";
					NLS->Calibrations[0].Camera = "colour";
					NLS->Calibrations[0].ImageSizeX = 480;
					NLS->Calibrations[0].ImageSizeY = 640;
					NLS->Calibrations[0].FX = 433.11529541015625;
					NLS->Calibrations[0].FY = 433.11529541015625;
					NLS->Calibrations[0].CX = 238.29920959472656;
					NLS->Calibrations[0].CY = 319.03814697265625;
					NLS->Calibrations[0].K1 = 0;
					NLS->Calibrations[0].K2 = 0;
					NLS->Calibrations[0].P1 = 0;
					NLS->Calibrations[0].P2 = 0;
					NLS->Calibrations[0].K3 = 0;
					NLS->Calibrations[0].Transform = FMatrix::Identity;

					NLS->Calibrations[1] = NLS->Calibrations[0];
					NLS->Calibrations[1].Name = "depth";
					NLS->Calibrations[1].Camera = "depth";

					TestHelper->Pipeline.MakeConnection(Colour, Gray);
					TestHelper->Pipeline.MakeConnection(Gray, Track);
					TestHelper->Pipeline.MakeConnection(Track, Refine);
					TestHelper->Pipeline.MakeConnection(Refine, NLS);
					TestHelper->Pipeline.MakeConnection(Depth, NLS);
				}
				else if (Test == "Grayscale")
				{
					TSharedPtr<FUEImageLoadNode> Load = TestHelper->Pipeline.MakeNode<FUEImageLoadNode>("Load");
					TSharedPtr<FUEImageToUEGrayImageNode> Colour2Gray = TestHelper->Pipeline.MakeNode<FUEImageToUEGrayImageNode>("Colour2Gray");
					TSharedPtr<FUEGrayImageToUEImageNode> Gray2Colour = TestHelper->Pipeline.MakeNode<FUEGrayImageToUEImageNode>("Gray2Colour");
					TSharedPtr<FUEImageSaveNode> Save = TestHelper->Pipeline.MakeNode<FUEImageSaveNode>("Save");

					Load->FilePath = InputDir + "/Stacy-colour/%04d.png";

					Save->FilePath = OutputDir + "/Grayscale/%04d.png";

					TestHelper->Pipeline.MakeConnection(Load, Colour2Gray);
					TestHelper->Pipeline.MakeConnection(Colour2Gray, Gray2Colour);
					TestHelper->Pipeline.MakeConnection(Gray2Colour, Save);
				}
				else if (Test == "Crop")
				{
					TSharedPtr<FUEImageLoadNode> Load = TestHelper->Pipeline.MakeNode<FUEImageLoadNode>("Load");
					TSharedPtr<FUEImageCropNode> Crop = TestHelper->Pipeline.MakeNode<FUEImageCropNode>("Crop");
					TSharedPtr<FUEImageSaveNode> Save = TestHelper->Pipeline.MakeNode<FUEImageSaveNode>("Save");

					Load->FilePath = InputDir + "/Stacy-colour/%04d.png";

					Crop->X = 100;
					Crop->Y = 200;
					Crop->Width = 300;
					Crop->Height = 400;

					Save->FilePath = OutputDir + "/Crop/%04d.png";

					TestHelper->Pipeline.MakeConnection(Load, Crop);
					TestHelper->Pipeline.MakeConnection(Crop, Save);
				}
				else if (Test == "Composite")
				{
					TSharedPtr<FUEImageLoadNode> Load = TestHelper->Pipeline.MakeNode<FUEImageLoadNode>("Load");
					TSharedPtr<FUEImageCropNode> Crop = TestHelper->Pipeline.MakeNode<FUEImageCropNode>("Crop");
					TSharedPtr<FUEImageCompositeNode> Composite = TestHelper->Pipeline.MakeNode<FUEImageCompositeNode>("Composite");
					TSharedPtr<FUEImageSaveNode> Save = TestHelper->Pipeline.MakeNode<FUEImageSaveNode>("Save");

					Load->FilePath = InputDir + "/Stacy-colour/%04d.png";

					Crop->X = 100;
					Crop->Y = 200;
					Crop->Width = 300;
					Crop->Height = 400;

					Save->FilePath = OutputDir + "/Composite/%04d.png";

					TestHelper->Pipeline.MakeConnection(Load, Crop);
					TestHelper->Pipeline.MakeConnection(Load, Composite, 0, 0);
					TestHelper->Pipeline.MakeConnection(Crop, Composite, 0, 1);
					TestHelper->Pipeline.MakeConnection(Composite, Save);
				}
				else if (Test == "JsonTracker")
				{
					TSharedPtr<FUEImageLoadNode> Load = TestHelper->Pipeline.MakeNode<FUEImageLoadNode>("Load");
					TSharedPtr<FJsonTrackerNode> Track = TestHelper->Pipeline.MakeNode<FJsonTrackerNode>("Track");
					TSharedPtr<FBurnContoursNode> Burn = TestHelper->Pipeline.MakeNode<FBurnContoursNode>("Burn");
					TSharedPtr<FUEImageSaveNode> Save = TestHelper->Pipeline.MakeNode<FUEImageSaveNode>("Save");

					Load->FilePath = InputDir + "/Stacy-colour/%04d.png";

					Track->JsonFile = InputDir + "/Stacy-tracking/tracking.json";

					Save->FilePath = OutputDir + "/Json/%04d.png";

					TestHelper->Pipeline.MakeConnection(Load, Track);
					TestHelper->Pipeline.MakeConnection(Track, Burn);
					TestHelper->Pipeline.MakeConnection(Burn, Save);
				}
				else
				{
					bIsOK &= TestTrue(TEXT("Known test"), false);
				}

				if (bIsOK)
				{
					if (Method == "PushSync")
					{
						TestHelper->Run(EPipelineMode::PushSync);
					}
					else if (Method == "PushAsync")
					{
						TestHelper->Run(EPipelineMode::PushAsync);
					}
					else if (Method == "PushAsyncNodes")
					{
						TestHelper->Run(EPipelineMode::PushAsyncNodes);
					}
					else
					{
						bIsOK &= TestTrue(TEXT("Known method"), false);
					}
				}
			}
		}
		else if (Stage == "Stage2")
		{
			ADD_LATENT_AUTOMATION_COMMAND(FPipelineTestNodesComplete(Test));
		}
		else if (Stage == "Stage3")
		{
			bIsOK &= TestValid(TEXT("Test helper set"), TestHelper);

			if (bIsOK)
			{
				bIsOK &= TestEqual(TEXT("Process complete count"), TestHelper->ProcessCompleteCount, 1);
				bIsOK &= TestEqual(TEXT("Exit status"), TestHelper->ExitStatus, EPipelineExitStatus::Ok);
				bIsOK &= TestEqual(TEXT("Error node code"), TestHelper->ErrorNodeCode, -1);

				if (Test == "Hyprsense")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 101);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FUEImageDataType>("Load.UE Image Out"));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Load.UE Image Out").Width, 480));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Load.UE Image Out").Height, 640));

						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FFrameTrackingContourData>("Track.Contours Out"));

						int32 TotalNumContours = 0;

						if (bIsOK)
						{
							TMap<FString, FTrackingContour> Contours = TestHelper->PipelineData[Frame]->GetData<FFrameTrackingContourData>("Track.Contours Out").TrackingContours;
							bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), Contours.Num(), 44));
							for (const TPair<FString, FTrackingContour>& Contour : Contours)
							{
								TotalNumContours += Contour.Value.DensePoints.Num();
							}
						}
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TotalNumContours, 532));

						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FUEImageDataType>("Burn.UE Image Out"));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Burn.UE Image Out").Width, 480));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Burn.UE Image Out").Height, 640));
					}
				}
				else if (Test == "HyprsenseRealtime")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 101);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FUEImageDataType>("Load.UE Image Out"));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Load.UE Image Out").Width, 480));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Load.UE Image Out").Height, 640));

						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FFrameTrackingContourData>("Track.Contours Out"));

						int32 TotalNumContours = 0;

						if (bIsOK)
						{
							TMap<FString, FTrackingContour> Contours = TestHelper->PipelineData[Frame]->GetData<FFrameTrackingContourData>("Track.Contours Out").TrackingContours;
							bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), Contours.Num(), 44));
							for (const TPair<FString, FTrackingContour>& Contour : Contours)
							{
								TotalNumContours += Contour.Value.DensePoints.Num();
							}
						}
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TotalNumContours, 532));

						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FUEImageDataType>("Burn.UE Image Out"));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Burn.UE Image Out").Width, 480));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Burn.UE Image Out").Height, 640));
					}
				}
				else if (Test == "HyprsensePythonCompare")
				{
					bIsOK &= TestValid(TEXT("Test helper set"), TestHelper);
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 101);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<float>("Compare.Avg Diff Out"));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<float>("Compare.Avg Diff Out"), 0.5f, 0.5f));
					}

				}
				else if (Test == "RLibVRefinement")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 10);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FFrameTrackingContourData>("Generic.Contours Out"));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FFrameTrackingContourData>("Generic.Contours Out").TrackingContours.Num(), 44));

						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FFrameTrackingContourData>("Offset.Contours Out"));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FFrameTrackingContourData>("Offset.Contours Out").TrackingContours.Num(), 44));

						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FFrameTrackingContourData>("Refinement.Contours Out"));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FFrameTrackingContourData>("Refinement.Contours Out").TrackingContours.Num(), 44));

						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FFrameTrackingConfidenceData>("Refinement.Confidence Out"));
						bIsOK &= (bIsOK && TestTrue(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FFrameTrackingConfidenceData>("Refinement.Confidence Out").Value > 0.01f));

						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FUEImageDataType>("Composite.UE Image Out"));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Composite.UE Image Out").Width, 500));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Composite.UE Image Out").Height, 360));
					}
				}
				else if (Test == "Depth")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 10);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FDepthDataType>("Load.Depth Out"));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FDepthDataType>("Load.Depth Out").Width, 480));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FDepthDataType>("Load.Depth Out").Height, 640));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FDepthDataType>("Load.Depth Out").Data.Num(), 480 * 640));

						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FUEImageDataType>("Convert.UE Image Out"));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Convert.UE Image Out").Width, 480));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Convert.UE Image Out").Height, 640));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Convert.UE Image Out").Data.Num(), 480 * 640 * 4));
					}
				}
				else if (Test == "FaceTrackerIPhone")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 10);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FFrameAnimationData>("NLS.Animation Out"));
						bIsOK &= (bIsOK && TestTrue(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FFrameAnimationData>("NLS.Animation Out").Pose.IsValid()));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FFrameAnimationData>("NLS.Animation Out").AnimationData.Num(), 250));
					}
				}
				else if (Test == "Grayscale")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 10);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FUEGrayImageDataType>("Colour2Gray.UE Gray Image Out"));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEGrayImageDataType>("Colour2Gray.UE Gray Image Out").Width, 480));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEGrayImageDataType>("Colour2Gray.UE Gray Image Out").Height, 640));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEGrayImageDataType>("Colour2Gray.UE Gray Image Out").Data.Num(), 480 * 640));

						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FUEImageDataType>("Gray2Colour.UE Image Out"));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Gray2Colour.UE Image Out").Width, 480));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Gray2Colour.UE Image Out").Height, 640));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Gray2Colour.UE Image Out").Data.Num(), 480 * 640 * 4));
					}
				}
				else if (Test == "Crop")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 10);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FUEImageDataType>("Crop.UE Image Out"));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Crop.UE Image Out").Width, 300));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Crop.UE Image Out").Height, 400));
					}
				}
				else if (Test == "Composite")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 10);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FUEImageDataType>("Composite.UE Image Out"));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Composite.UE Image Out").Width, 480 + 300));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FUEImageDataType>("Composite.UE Image Out").Height, 640));
					}
				}
				else if (Test == "JsonTracker")
				{
					bIsOK &= TestEqual(TEXT("Frame completed count"), TestHelper->FrameCompleteCount, 10);

					for (int32 Frame = 0; Frame < TestHelper->FrameCompleteCount; ++Frame)
					{
						bIsOK &= TestTrue(TEXT("Data present"), TestHelper->PipelineData[Frame]->HasData<FFrameTrackingContourData>("Track.Contours Out"));
						bIsOK &= (bIsOK && TestEqual(TEXT("Expected value"), TestHelper->PipelineData[Frame]->GetData<FFrameTrackingContourData>("Track.Contours Out").TrackingContours.Num(), 44));
					}
				}
				else
				{
					bIsOK &= TestTrue(TEXT("Known test"), false);
				}
			}

			TestHelper = nullptr;
		}
		else
		{
			bIsOK &= TestTrue(TEXT("Known stage"), false);
		}
	}

	return bIsOK;
}

#endif

}
