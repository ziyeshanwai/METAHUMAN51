// Copyright Epic Games, Inc. All Rights Reserved.

struct StereoEstimateBrightnessScalingPushConstants {
    int width;
    int height;
};

static const int StereoEstimateBrightnessScalingComputeThreadSizeX = 32;
static const int StereoEstimateBrightnessScalingComputeThreadSizeY = 32;
