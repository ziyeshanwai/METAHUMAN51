// Copyright Epic Games, Inc. All Rights Reserved.

struct DisparityToDepthPushConstants {
    int width;
    int height;
    float fb;
    float offset;
    float scaleDisparity;
};

static const int DisparityToDepthComputeThreadSizeX = 32;
static const int DisparityToDepthComputeThreadSizeY = 32;
