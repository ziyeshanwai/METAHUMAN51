// Copyright Epic Games, Inc. All Rights Reserved.

// The purpose of this file it to define an interface to rlibv functionality that can
// be called by UE. Dont use dlib etc types here since that complicated the compile.

#pragma once

#include "CoreMinimal.h"
#include "FrameTrackingContourData.h"
#include "FrameTrackingConfidenceData.h"

class RLIBV_API FRLibVRefinementTracker
{
public:

	FRLibVRefinementTracker();
	virtual ~FRLibVRefinementTracker();

	bool Initialize(const FString& InModelFile);
	bool Track(int32 InWidth, int32 InHeight, const TArray<uint8>& InImageData, const FFrameTrackingContourData& InContours, FFrameTrackingContourData& OutContours, FFrameTrackingConfidenceData& OutConfidence);

	const FString& GetErrorMessage() const { return ErrorMessage; }

private:

	class FImpl;
	TUniquePtr<FImpl> Impl;

	FString ErrorMessage;
};
