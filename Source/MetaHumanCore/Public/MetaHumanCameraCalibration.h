// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

struct FMetaHumanCameraCalibration
{
public:
	FString Name;
	FString Camera;
	int32 ImageSizeX = 0;
	int32 ImageSizeY = 0;
	double FX = 0;
	double FY = 0;
	double CX = 0;
	double CY = 0;
	double K1 = 0;
	double K2 = 0;
	double P1 = 0;
	double P2 = 0;
	double K3 = 0;
	double K4 = 0;
	double K5 = 0;
	double K6 = 0;
	FMatrix Transform;
};