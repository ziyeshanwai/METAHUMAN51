// Copyright Epic Games, Inc. All Rights Reserved.

struct WarpPushConstants
{
    int outputWidth;
    int outputHeight;
    int maxX;
    int maxY;
    int interpolationMode;
};

static const int WarpComputeThreadSizeX = 32;
static const int WarpComputeThreadSizeY = 32;
