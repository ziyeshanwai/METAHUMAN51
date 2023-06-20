// Copyright Epic Games, Inc. All Rights Reserved.

struct StereoMatchingPushConstants {
    int width;
    int height;
    int numMatchingOffsets;
};

static const int StereoMatchingComputeThreadSizeX = 32;
static const int StereoMatchingComputeThreadSizeY = 32;
