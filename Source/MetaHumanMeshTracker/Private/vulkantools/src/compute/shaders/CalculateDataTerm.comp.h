// Copyright Epic Games, Inc. All Rights Reserved.

#include "DataTermShaderParameters.h"

struct CalculateDataTermPushConstants
{
    int evenLen;
    int oddLen;
    int width;
    int height;
    DataTermShaderParams params;
};

static const int CalculateDataTermThreadSizeX = 32;
static const int CalculateDataTermThreadSizeY = 32;
