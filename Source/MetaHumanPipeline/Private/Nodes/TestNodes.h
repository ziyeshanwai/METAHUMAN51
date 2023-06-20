// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Pipeline/Node.h"

namespace UE::MetaHuman::Pipeline
{

class FIntSrcNode : public FNode
{
public:

	FIntSrcNode(const FString& InName);

	int32 Value = 1;
	int32 NumberOfFrames = 1;

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

class FIntIncNode : public FNode
{
public:

	FIntIncNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

class FIntDecNode : public FNode
{
public:

	FIntDecNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

class FIntSumNode : public FNode
{
public:

	FIntSumNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

class FIntLogNode : public FNode
{
public:

	FIntLogNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};



class FFltSrcNode : public FNode
{
public:

	FFltSrcNode(const FString& InName);

	float Value = 1.0f;
	int32 NumberOfFrames = 1;

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

class FFltIncNode : public FNode
{
public:

	FFltIncNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

class FFltDecNode : public FNode
{
public:

	FFltDecNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

class FFltSumNode : public FNode
{
public:

	FFltSumNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

class FFltLogNode : public FNode
{
public:

	FFltLogNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};



class FMixSrcNode : public FNode
{
public:

	FMixSrcNode(const FString& InName);

	int32 IntValue = 1;
	float FltValue = 1.0f;
	int32 NumberOfFrames = 1;

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

class FMixIncNode : public FNode
{
public:

	FMixIncNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

class FMixDecNode: public FNode
{
public:

	FMixDecNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

class FMixSumNode : public FNode
{
public:

	FMixSumNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

class FMixLogNode : public FNode
{
public:

	FMixLogNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};



class FErrorNode : public FIntIncNode
{
public:

	FErrorNode(const FString& InName);

	int32 ErrorOnStage = 1;
	int32 ErrorOnFrame = 1;
	int32 ErrorCode = 1;

	virtual bool Start(const TSharedPtr<FPipelineData>& InPipelineData) override;
	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
	virtual bool End(const TSharedPtr<FPipelineData>& InPipelineData) override;
};



class FIntsToFltNode : public FNode
{
public:

	FIntsToFltNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};



class FProcessCountNode : public FIntIncNode
{
public:

	FProcessCountNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;

	int32 ProcessCount = 0;
};



class FDropFrameNode : public FNode
{
public:

	FDropFrameNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;

	int32 DropEvery = 0;
};



class FBufferNode : public FNode
{
public:

	FBufferNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
	virtual bool Idle(const TSharedPtr<FPipelineData>& InPipelineData) override;

	virtual bool RequiresIdle() const override { return true; }
	virtual bool ProcessesExitFrame() const override { return true; }

	TArray<TSharedPtr<FPipelineData>> BufferedData;

	double LastAdd = 0;
	double Delay = 0.1;
	int32 Capacity = 3;
};



class FSlowIntIncNode : public FIntIncNode
{
public:

	FSlowIntIncNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

}
