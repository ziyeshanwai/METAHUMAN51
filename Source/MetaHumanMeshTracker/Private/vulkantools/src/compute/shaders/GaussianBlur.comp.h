// Copyright Epic Games, Inc. All Rights Reserved.

struct GaussianBlurPushConstants {
    int width;
    int height;
    int kernelSize2; //!< half of (kernel size minus one)
    int xStep;
    int yStep;
};

static const int GaussianBlurComputeThreadSizeX = 32;
static const int GaussianBlurComputeThreadSizeY = 32;
