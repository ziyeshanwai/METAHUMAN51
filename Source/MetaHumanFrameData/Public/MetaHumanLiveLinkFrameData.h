// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "LiveLinkTypes.h"
#include "FrameTrackingContourData.h"

#include "MetaHumanLiveLinkFrameData.generated.h"



/**
 * Dynamic data for MetaHuman Live Link source.
 */
USTRUCT(BlueprintType)
struct METAHUMANFRAMEDATA_API FMetaHumanLiveLinkFrameData : public FLiveLinkBaseFrameData
{
	GENERATED_BODY()

	/** The number of bytes for this frame's compressed video data */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MetaHuman")
	int32 CompressedVideoBytes = 0;

	/** The number of bytes for this frame's compressed depth data */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MetaHuman")
	int32 CompressedDepthBytes = 0;

	/** The number of samples for this frame's audio data */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MetaHuman")
	int32 AudioSamples = 0;

	/** The compressed (JPEG encoded) video data for this frame */
	UPROPERTY(BlueprintReadWrite, Category = "MetaHuman")
	TArray<uint8> CompressedVideoData;

	/** The decompressed video data for this frame (decompressing may introduce a rotation) */
	UPROPERTY(BlueprintReadWrite, Category = "MetaHuman")
	TArray<uint8> DecompressedVideoData;

	/** The compressed (Oodle encoded) depth data for this frame */
	UPROPERTY(BlueprintReadWrite, Category = "MetaHuman")
	TArray<uint8> CompressedDepthData;

	/** The decompressed depth data for this frame (decompressing may introduce a rotation and scaling) */
	UPROPERTY(BlueprintReadWrite, Category = "MetaHuman")
	TArray<float> DecompressedDepthData;

	/** The audio data for this frame */
	UPROPERTY()
	TArray<int16> AudioData;

	/** The calculated tracking data for this frame */
	UPROPERTY()
	FFrameTrackingContourData ContourData;

	/** The pose transformation for this frame */
	UPROPERTY(BlueprintReadWrite, Category = "MetaHuman")
	FTransform Transform = FTransform::Identity;

	/** Timings at various stages */
	double ReadStartTime;
	double ReadEndTime;
	double ProcessStartTime;
	double ProcessEndTime;

	/** Processing statistics and control */
	int32 LoadNodeQueueSize = -1;
	bool bProcessFrame = true;
};
