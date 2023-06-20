// Copyright Epic Games, Inc. All Rights Reserved.

struct StereoUpsamplePushConstants {
    int width;
    int height;
};

static const int StereoUpsampleComputeThreadSizeX = 32;
static const int StereoUpsampleComputeThreadSizeY = 32;
