// Copyright Epic Games, Inc. All Rights Reserved.

struct StereoMaskByBrightnessScalingPushConstants {
    int width;
    int height;
    float mainScale;
    float scaleDifferenceThreshold;
};

static const int StereoMaskByBrightnessScalingComputeThreadSizeX = 32;
static const int StereoMaskByBrightnessScalingComputeThreadSizeY = 32;
