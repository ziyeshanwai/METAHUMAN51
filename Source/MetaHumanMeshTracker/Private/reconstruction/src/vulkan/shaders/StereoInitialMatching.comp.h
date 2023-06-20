// Copyright Epic Games, Inc. All Rights Reserved.

struct StereoInitialMatchingPushConstants {
    int width;
    int height;
    int minimumDisparity;
    int maximumDisparity;
};

static const int StereoInitialMatchingComputeThreadSizeX = 32;
static const int StereoInitialMatchingComputeThreadSizeY = 32;
