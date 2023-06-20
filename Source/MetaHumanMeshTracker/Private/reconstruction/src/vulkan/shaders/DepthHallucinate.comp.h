// Copyright Epic Games, Inc. All Rights Reserved.

struct DepthHallucinatePushConstants {
    int width;
    int height;
    float scale;
};

static const int DepthHallucinateComputeThreadSizeX = 32;
static const int DepthHallucinateComputeThreadSizeY = 32;
