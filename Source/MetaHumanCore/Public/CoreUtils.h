// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

namespace epic
{
namespace core
{

void METAHUMANCORE_API BurnPointsIntoImage(const TArray<FVector2D>& InPoints, int32 InWidth, int32 InHeight, TArray<uint8>& InImgData, uint8 InRed, uint8 InGreen, uint8 InBlue, int32 InSize, bool bInUseAntiAliasing = true);

void METAHUMANCORE_API BurnPointsIntoImage(const TArray<FVector2D>& InPoints, int32 InWidth, int32 InHeight, FColor* OutImgData, uint8 InRed, uint8 InGreen, uint8 InBlue, int32 InSize, bool bInUseAntiAliasing = true);

}
}
