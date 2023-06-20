// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "IntVectorTypes.h"
#include "Pipeline/Node.h"
#include "Pipeline/PipelineData.h"

class UTextureRenderTarget2D;

namespace UE::MetaHuman::Pipeline
{

class METAHUMANPIPELINE_API FUEImageLoadNode : public FNode
{
public:

	FUEImageLoadNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;

	FString FilePath;
	int32 FrameNumberOffset = 0;

	enum ErrorCode
	{
		FailedToLoadFile = 0,
		MissingFrameFormatSpecifier
	};
};

class METAHUMANPIPELINE_API FUEImageSaveNode : public FNode
{
public:

	FUEImageSaveNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;

	FString FilePath;
	int32 FrameNumberOffset = 0;

	enum ErrorCode
	{
		FailedToSaveFile = 0,
		FailedToCompressData,
		MissingFrameFormatSpecifier
	};
};

class METAHUMANPIPELINE_API FUEImageResizeNode : public FNode
{
public:

	FUEImageResizeNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;

	int32 MaxSize = 1;
};

class METAHUMANPIPELINE_API FUEImageCropNode : public FNode
{
public:

	FUEImageCropNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;

	int32 X = -1;
	int32 Y = -1;
	int32 Width = -1;
	int32 Height = -1;

	enum ErrorCode
	{
		BadValues = 0
	};
};

class METAHUMANPIPELINE_API FUEImageCompositeNode : public FNode
{
public:

	FUEImageCompositeNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

class METAHUMANPIPELINE_API FUEImageToUEGrayImageNode : public FNode
{
public:

	FUEImageToUEGrayImageNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

class METAHUMANPIPELINE_API FUEGrayImageToUEImageNode : public FNode
{
public:

	FUEGrayImageToUEImageNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

class METAHUMANPIPELINE_API FUEImageToHSImageNode : public FNode
{
public:

	FUEImageToHSImageNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
};

class METAHUMANPIPELINE_API FBurnContoursNode : public FNode
{
public:

	FBurnContoursNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;

	int32 Size = 1;
};

class METAHUMANPIPELINE_API FDepthLoadNode : public FNode
{
public:

	FDepthLoadNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;

	FString FilePath;
	int32 FrameNumberOffset = 0;

	enum ErrorCode
	{
		FailedToLoadFile = 0,
		MissingFrameFormatSpecifier
	};
};

class METAHUMANPIPELINE_API FDepthSaveNode : public FNode
{
public:

	FDepthSaveNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;

	FString FilePath;
	int32 FrameNumberOffset = 0;

	enum ErrorCode
	{
		FailedToSaveFile = 0,
		FailedToCompressData,
		MissingFrameFormatSpecifier
	};
};

class METAHUMANPIPELINE_API FDepthToUEImageNode : public FNode
{
public:

	FDepthToUEImageNode(const FString& InName);

	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;

	float Min = 0.0f;
	float Max = 1.0f;

	enum ErrorCode
	{
		BadRange = 0
	};
};

class METAHUMANPIPELINE_API FFColorToUEImageNode : public FNode
{
public:
	FFColorToUEImageNode(const FString& InName);
	
	virtual bool Process(const TSharedPtr<FPipelineData>& InPipelineData) override;
	
	TArray<FColor> Samples;
	int32 Width = -1;
	int32 Height = -1;

	enum ErrorCode
	{
		NoInputImage = 0
	};
}; 

}
