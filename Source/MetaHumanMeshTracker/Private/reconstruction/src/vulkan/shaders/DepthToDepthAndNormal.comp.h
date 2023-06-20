// Copyright Epic Games, Inc. All Rights Reserved.

struct DepthToDepthAndNormalPushConstants {
    int width;
    int height;
    float fx;
    float fy;
    float skew;
    float cx;
    float cy;
};

static const int DepthToDepthAndNormalComputeThreadSizeX = 32;
static const int DepthToDepthAndNormalComputeThreadSizeY = 32;
