// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

namespace UE::MetaHuman::Pipeline
{

enum class EPinDirection
{
	Input = 0,
	Output
};

enum class EPinType // Cant replace with std::type_info, due to RTTI
{
	Int = 0,
	Float,
	Bool,
	String,
	UE_Image,     // 4 channel BGRA
	UE_GrayImage, // single channel
	HS_Image,
	Points,
	Contours,
	Scaling,
	Animation,
	Depth,
	TrackingConfidence,
	LiveLinkFrame
};

class METAHUMANPIPELINE_API FPin
{
public:

	FPin(const FString& InName, EPinDirection InDirection, EPinType InType, int32 InGroup = 0);

	FString Name;
	EPinDirection Direction;
	EPinType Type;
	int32 Group;

	FString Address;

	FString ToString() const;
};

}
