// Copyright Epic Games, Inc. All Rights Reserved.

#define SOBEL_DIRECTION_HORIZONTAL 0
#define SOBEL_DIRECTION_VERTICAL   1

struct SobelPushConstants
{
    int width;
    int height;
    int direction;
};

static const int SobelComputeThreadSizeX = 32;
static const int SobelComputeThreadSizeY = 32;
