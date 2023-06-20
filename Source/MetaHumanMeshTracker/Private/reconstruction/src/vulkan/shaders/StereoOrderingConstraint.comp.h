// Copyright Epic Games, Inc. All Rights Reserved.

struct StereoOrderingConstraintPushConstants {
    int width;
    int height;
};

static const int StereoOrderingConstraintComputeThreadSizeX = 32;
static const int StereoOrderingConstraintComputeThreadSizeY = 32;
