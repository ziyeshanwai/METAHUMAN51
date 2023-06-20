// Copyright Epic Games, Inc. All Rights Reserved.

struct StereoUniquenessConstraintPushConstants {
    int width;
    int height;
};

static const int StereoUniquenessConstraintComputeThreadSizeX = 32;
static const int StereoUniquenessConstraintComputeThreadSizeY = 32;
