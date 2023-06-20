// Copyright Epic Games, Inc. All Rights Reserved.

struct StereoRematchingPushConstants {
    int width;
    int height;
};

static const int StereoRematchingComputeThreadSizeX = 32;
static const int StereoRematchingComputeThreadSizeY = 32;
