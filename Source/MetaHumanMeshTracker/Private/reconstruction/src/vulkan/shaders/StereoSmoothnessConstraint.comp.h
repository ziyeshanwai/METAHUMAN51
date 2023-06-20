// Copyright Epic Games, Inc. All Rights Reserved.

struct StereoSmoothnessConstraintPushConstants {
    int width;
    int height;
};

static const int StereoSmoothnessConstraintComputeThreadSizeX = 32;
static const int StereoSmoothnessConstraintComputeThreadSizeY = 32;
