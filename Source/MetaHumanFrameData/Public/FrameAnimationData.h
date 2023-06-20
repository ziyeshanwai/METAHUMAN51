// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "FrameAnimationData.generated.h"

USTRUCT()
struct METAHUMANFRAMEDATA_API FFrameAnimationData
{
	GENERATED_BODY()

	UPROPERTY()
	FTransform Pose = FTransform(FQuat(), FVector(), FVector());

	UPROPERTY()
	TMap<FString, float> AnimationData;

	bool ContainsData() const
	{
		return !AnimationData.IsEmpty();
	}
};
