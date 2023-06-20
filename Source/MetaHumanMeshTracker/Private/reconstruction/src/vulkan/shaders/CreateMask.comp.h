// Copyright Epic Games, Inc. All Rights Reserved.

struct CreateMaskPushConstants {
    int width;
    int height;
    float darkThreshold;
    float brightThreshold;
};

static const int CreateMaskComputeThreadSizeX = 32;
static const int CreateMaskComputeThreadSizeY = 32;