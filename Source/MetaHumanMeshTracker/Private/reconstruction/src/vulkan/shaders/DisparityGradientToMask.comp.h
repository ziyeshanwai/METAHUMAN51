// Copyright Epic Games, Inc. All Rights Reserved.

struct DisparityGradientToMaskPushConstants {
    int width;
    int height;
    float threshold;
};

static const int DisparityGradientToMaskComputeThreadSizeX = 32;
static const int DisparityGradientToMaskComputeThreadSizeY = 32;
