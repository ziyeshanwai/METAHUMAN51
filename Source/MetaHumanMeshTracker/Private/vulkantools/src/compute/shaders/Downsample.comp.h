// Copyright Epic Games, Inc. All Rights Reserved.

struct DownsamplePushConstants {
    int width;
    int height;
};

static const int DownsampleComputeThreadSizeX = 32;
static const int DownsampleComputeThreadSizeY = 32;
