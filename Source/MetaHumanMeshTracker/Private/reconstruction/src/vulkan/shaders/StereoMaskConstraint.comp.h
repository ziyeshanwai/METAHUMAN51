// Copyright Epic Games, Inc. All Rights Reserved.

struct StereoMaskConstraintPushConstants {
    int width;
    int height;
    int targetWidth;
};

static const int StereoMaskConstraintComputeThreadSizeX = 32;
static const int StereoMaskConstraintComputeThreadSizeY = 32;
