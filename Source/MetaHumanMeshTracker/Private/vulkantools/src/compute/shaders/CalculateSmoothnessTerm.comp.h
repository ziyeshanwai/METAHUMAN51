// Copyright Epic Games, Inc. All Rights Reserved.

#include "DataTermShaderParameters.h"

#define SMOOTHNESS_TERM_DIRECTION_HORIZONTAL 0
#define SMOOTHNESS_TERM_DIRECTION_VERTICAL   1

struct CalculateSmoothnessTermPushConstants
{
    int isRedPass;
    int evenLen;
    int oddLen;
    int width;
    int height;
    int direction;
    DataTermShaderParams params;
};

static const int CalculateSmoothnessTermThreadSizeX = 32;
static const int CalculateSmoothnessTermThreadSizeY = 32;
