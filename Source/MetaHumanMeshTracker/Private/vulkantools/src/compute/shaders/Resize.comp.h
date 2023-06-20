// Copyright Epic Games, Inc. All Rights Reserved.

struct ResizePushConstants
{
    int outputWidth;
    int outputHeight;
    int maxX;
    int maxY;
    float scaleX;
    float scaleY;
    float scale;
    int interpolationMode;
};

static const int ResizeComputeThreadSizeX = 32;
static const int ResizeComputeThreadSizeY = 32;
