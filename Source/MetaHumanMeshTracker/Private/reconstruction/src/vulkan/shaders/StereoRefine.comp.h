// Copyright Epic Games, Inc. All Rights Reserved.

struct StereoRefinePushConstants {
    int width;
    int height;
    float ws;
};

static const int StereoRefineComputeThreadSizeX = 32;
static const int StereoRefineComputeThreadSizeY = 32;
